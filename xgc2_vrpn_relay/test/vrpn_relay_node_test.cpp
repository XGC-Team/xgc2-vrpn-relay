// Runs against the relay nodes started by vrpn_relay.test. This process plays the VRPN client
// (it publishes the tracker topics) and the consumers (it collects the relay outputs).
#include <gtest/gtest.h>

#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sys/wait.h>

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

template <typename Predicate> bool waitUntil(Predicate done, double timeout_s) {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_s);
    while (!done()) {
        if (ros::WallTime::now() > deadline) {
            return false;
        }
        ros::WallDuration(0.005).sleep();
    }
    return true;
}

template <typename Message> class Collector {
public:
    Collector(ros::NodeHandle& nh, const std::string& topic) {
        subscriber_ = nh.subscribe(topic, 1000, &Collector::onMessage, this);
    }

    // True once the relay's publisher is connected, so nothing published later can be lost.
    bool connected() const { return subscriber_.getNumPublishers() > 0; }

    std::size_t size() {
        std::lock_guard<std::mutex> lock(mutex_);
        return messages_.size();
    }

    std::vector<Message> all() {
        std::lock_guard<std::mutex> lock(mutex_);
        return messages_;
    }

    Message at(std::size_t index) {
        std::lock_guard<std::mutex> lock(mutex_);
        return messages_.at(index);
    }

    bool waitForCount(std::size_t count, double timeout_s = 5.0) {
        return waitUntil([&] { return size() >= count; }, timeout_s);
    }

private:
    void onMessage(const boost::shared_ptr<const Message>& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.push_back(*message);
    }

    ros::Subscriber subscriber_;
    std::mutex mutex_;
    std::vector<Message> messages_;
};

// The fake VRPN client of one tracker. Like vrpn_client_ros, it publishes the acceleration as a
// TwistStamped.
class FakeTracker {
public:
    FakeTracker(ros::NodeHandle& nh, const std::string& pose_topic, const std::string& twist_topic,
                const std::string& accel_topic) {
        pose_ = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
        twist_ = nh.advertise<geometry_msgs::TwistStamped>(twist_topic, 10);
        accel_ = nh.advertise<geometry_msgs::TwistStamped>(accel_topic, 10);
    }

    FakeTracker(ros::NodeHandle& nh, const std::string& tracker)
        : FakeTracker(nh, "/vrpn_client_node/" + tracker + "/pose",
                      "/vrpn_client_node/" + tracker + "/twist",
                      "/vrpn_client_node/" + tracker + "/accel") {}

    // The relay has subscribed to all three inputs.
    bool relayConnected() const {
        return pose_.getNumSubscribers() > 0 && twist_.getNumSubscribers() > 0 &&
               accel_.getNumSubscribers() > 0;
    }

    void publishPose(double x, double y, double z, double qw = 1.0,
                     const ros::Time& stamp = ros::Time::now()) {
        geometry_msgs::PoseStamped message;
        message.header.stamp = stamp;
        message.header.frame_id = "world";
        message.pose.position.x = x;
        message.pose.position.y = y;
        message.pose.position.z = z;
        message.pose.orientation.w = qw;
        pose_.publish(message);
    }

    void publishTwist(double vx, double wz, const ros::Time& stamp = ros::Time::now()) {
        geometry_msgs::TwistStamped message;
        message.header.stamp = stamp;
        message.header.frame_id = "world";
        message.twist.linear.x = vx;
        message.twist.angular.z = wz;
        twist_.publish(message);
    }

    void publishAccel(double ax, double wz, const ros::Time& stamp = ros::Time::now()) {
        geometry_msgs::TwistStamped message;
        message.header.stamp = stamp;
        message.header.frame_id = "world";
        message.twist.linear.x = ax;
        message.twist.angular.z = wz;
        accel_.publish(message);
    }

    void publishAccel(const geometry_msgs::TwistStamped& message) { accel_.publish(message); }

private:
    ros::Publisher pose_;
    ros::Publisher twist_;
    ros::Publisher accel_;
};

// A relay under test: the fake tracker feeding it and the collectors on its outputs.
struct Rig {
    Rig(ros::NodeHandle& nh, const std::string& name, bool with_vision = false)
        : tracker(nh, name), pose(nh, "/" + name + "/pose"), twist(nh, "/" + name + "/twist"),
          accel(nh, "/" + name + "/accel") {
        if (with_vision) {
            vision.reset(new Collector<geometry_msgs::PoseStamped>(nh, "/" + name + "/vision"));
        }
    }

    // Waits for the relay to be wired in both directions.
    bool ready() {
        return waitUntil(
            [&] {
                return tracker.relayConnected() && pose.connected() && twist.connected() &&
                       accel.connected() && (!vision || vision->connected());
            },
            20.0);
    }

    // Publishes a pose that must come out, and waits for it. Frames published one after the
    // other would overwrite each other in the relay's one-message queue, so a pose the test
    // depends on is always confirmed before the next one goes out.
    ::testing::AssertionResult relayed(double x, double y, double z, double qw = 1.0,
                                       const ros::Time& stamp = ros::Time::now()) {
        const std::size_t before = pose.size();
        tracker.publishPose(x, y, z, qw, stamp);
        if (!pose.waitForCount(before + 1)) {
            return ::testing::AssertionFailure() << "pose x=" << x << " was not relayed";
        }
        return ::testing::AssertionSuccess();
    }

    // Publishes a pose that must be dropped. The relay handles a frame in microseconds, so a
    // short wait is enough to see a wrongly accepted one; if it arrived later anyway, the next
    // confirmed pose would not match what the test expects.
    ::testing::AssertionResult dropped(double x, double y, double z, double qw = 1.0) {
        const std::size_t before = pose.size();
        tracker.publishPose(x, y, z, qw);
        ros::WallDuration(0.1).sleep();
        if (pose.size() != before) {
            return ::testing::AssertionFailure() << "pose x=" << x << " was relayed";
        }
        return ::testing::AssertionSuccess();
    }

    FakeTracker tracker;
    Collector<geometry_msgs::PoseStamped> pose;
    Collector<geometry_msgs::TwistStamped> twist;
    Collector<geometry_msgs::AccelStamped> accel;
    std::unique_ptr<Collector<geometry_msgs::PoseStamped>> vision;
};

ros::NodeHandle& nodeHandle() {
    static ros::NodeHandle nh;
    return nh;
}

TEST(VrpnRelay, ForwardsPoseTwistAndAccelOneForOne) {
    Rig rig(nodeHandle(), "relay_basic", true);
    ASSERT_TRUE(rig.ready());

    const ros::Time stamp(123, 456000000);
    // Not a unit quaternion: the relay normalizes it.
    ASSERT_TRUE(rig.relayed(1.0, 2.0, 0.5, 2.0, stamp));
    rig.tracker.publishTwist(0.25, -0.5, stamp);
    geometry_msgs::TwistStamped accel_in;
    accel_in.header.stamp = stamp;
    accel_in.header.frame_id = "world";
    accel_in.twist.linear.x = 0.75;
    accel_in.twist.linear.y = -0.25;
    accel_in.twist.linear.z = 0.5;
    accel_in.twist.angular.x = 0.125;
    accel_in.twist.angular.y = -0.0625;
    accel_in.twist.angular.z = 1.5;
    rig.tracker.publishAccel(accel_in);
    ASSERT_TRUE(rig.twist.waitForCount(1));
    ASSERT_TRUE(rig.accel.waitForCount(1));
    ASSERT_TRUE(rig.vision->waitForCount(1));

    const geometry_msgs::PoseStamped pose = rig.pose.at(0);
    EXPECT_EQ(pose.header.stamp, stamp);
    EXPECT_EQ(pose.header.frame_id, "world");
    EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.5);
    EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);

    const geometry_msgs::PoseStamped vision = rig.vision->at(0);
    EXPECT_EQ(vision.header.stamp, stamp);
    EXPECT_DOUBLE_EQ(vision.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(vision.pose.orientation.w, 1.0);

    const geometry_msgs::TwistStamped twist = rig.twist.at(0);
    EXPECT_EQ(twist.header.stamp, stamp);
    EXPECT_EQ(twist.header.frame_id, "world");
    EXPECT_DOUBLE_EQ(twist.twist.linear.x, 0.25);
    EXPECT_DOUBLE_EQ(twist.twist.angular.z, -0.5);

    // The client's accel TwistStamped comes out as an AccelStamped: linear to linear, angular to
    // angular, header copied.
    const geometry_msgs::AccelStamped accel = rig.accel.at(0);
    EXPECT_EQ(accel.header.stamp, stamp);
    EXPECT_EQ(accel.header.frame_id, "world");
    EXPECT_DOUBLE_EQ(accel.accel.linear.x, 0.75);
    EXPECT_DOUBLE_EQ(accel.accel.linear.y, -0.25);
    EXPECT_DOUBLE_EQ(accel.accel.linear.z, 0.5);
    EXPECT_DOUBLE_EQ(accel.accel.angular.x, 0.125);
    EXPECT_DOUBLE_EQ(accel.accel.angular.y, -0.0625);
    EXPECT_DOUBLE_EQ(accel.accel.angular.z, 1.5);
}

TEST(VrpnRelay, ReplacesAZeroPoseStampWithTheReceiptTime) {
    Rig rig(nodeHandle(), "relay_basic", true);
    ASSERT_TRUE(rig.ready());

    const ros::Time before = ros::Time::now();
    ASSERT_TRUE(rig.relayed(1.5, 2.0, 0.5, 1.0, ros::Time()));
    const ros::Time after = ros::Time::now();
    const geometry_msgs::PoseStamped pose = rig.pose.all().back();
    EXPECT_GE(pose.header.stamp, before);
    EXPECT_LE(pose.header.stamp, after);
}

TEST(VrpnRelay, DropsNonFiniteTwistAndAccel) {
    Rig rig(nodeHandle(), "relay_basic", true);
    ASSERT_TRUE(rig.ready());

    const std::size_t twists = rig.twist.size();
    const std::size_t accels = rig.accel.size();
    rig.tracker.publishTwist(kNaN, 0.0);
    rig.tracker.publishAccel(0.0, std::numeric_limits<double>::infinity());
    ros::WallDuration(0.2).sleep();
    EXPECT_EQ(rig.twist.size(), twists);
    EXPECT_EQ(rig.accel.size(), accels);

    // The relay is still alive and forwards the next finite ones.
    rig.tracker.publishTwist(3.0, 0.0);
    rig.tracker.publishAccel(4.0, 0.0);
    ASSERT_TRUE(rig.twist.waitForCount(twists + 1));
    ASSERT_TRUE(rig.accel.waitForCount(accels + 1));
    EXPECT_DOUBLE_EQ(rig.twist.all().back().twist.linear.x, 3.0);
    EXPECT_DOUBLE_EQ(rig.accel.all().back().accel.linear.x, 4.0);
}

TEST(VrpnRelay, RejectsBadPoses) {
    Rig rig(nodeHandle(), "relay_gate");
    ASSERT_TRUE(rig.ready());

    EXPECT_TRUE(rig.dropped(kNaN, 2.0, 0.5));      // nonfinite
    EXPECT_TRUE(rig.dropped(1.0, 2.0, 0.5, 0.0));  // bad_quat
    EXPECT_TRUE(rig.dropped(0.0, 0.0, 0.5));       // origin
    EXPECT_TRUE(rig.dropped(901.0, 2.0, 0.5));     // abs_xy
    ASSERT_TRUE(rig.relayed(1.0, 2.0, 0.5));       // first
    EXPECT_TRUE(rig.dropped(4.5, 2.0, 0.5));       // jump 3.5 m
    ASSERT_TRUE(rig.relayed(2.0, 2.0, 0.5));       // 1 m is fine
    // The jump was measured from the last accepted pose, which is still (2, 2).
    EXPECT_TRUE(rig.dropped(4.5, 2.0, 0.5));
    ASSERT_TRUE(rig.relayed(3.5, 2.0, 0.5));

    std::vector<double> xs;
    for (const geometry_msgs::PoseStamped& pose : rig.pose.all()) {
        xs.push_back(pose.pose.position.x);
    }
    EXPECT_EQ(xs, (std::vector<double>{1.0, 2.0, 3.5}));
}

TEST(VrpnRelay, RejectsAFrozenPoseUntilItMoves) {
    Rig rig(nodeHandle(), "relay_frozen");
    ASSERT_TRUE(rig.ready());

    // Inside the one second window a repeated pose is a pose.
    ASSERT_TRUE(rig.relayed(10.0, 5.0, 0.5));
    ASSERT_TRUE(rig.relayed(10.0, 5.0, 0.5));
    // Past it, the same pose is a frozen tracker.
    ros::WallDuration(1.2).sleep();
    EXPECT_TRUE(rig.dropped(10.0, 5.0, 0.5));
    EXPECT_TRUE(rig.dropped(10.0, 5.0, 0.5));
    // Moving by a centimeter brings it back, and the window restarts from there.
    ASSERT_TRUE(rig.relayed(10.01, 5.0, 0.5));
    ASSERT_TRUE(rig.relayed(10.01, 5.0, 0.5));
}

TEST(VrpnRelay, ThinsTheVisionPoseToTheTargetRate) {
    Rig rig(nodeHandle(), "relay_vision", true);
    ASSERT_TRUE(rig.ready());

    // 100 Hz of a slowly moving pose for three seconds.
    const ros::WallTime start = ros::WallTime::now();
    ros::WallRate rate(100.0);
    int sent = 0;
    while ((ros::WallTime::now() - start).toSec() < 3.0) {
        rig.tracker.publishPose(1.0 + 0.0005 * sent, 2.0, 0.5);
        ++sent;
        rate.sleep();
    }
    ros::WallDuration(0.3).sleep();

    const std::vector<geometry_msgs::PoseStamped> poses = rig.pose.all();
    const std::vector<geometry_msgs::PoseStamped> vision = rig.vision->all();
    // The pose output carries (nearly) every frame.
    EXPECT_GE(poses.size(), static_cast<std::size_t>(0.9 * sent));
    ASSERT_GE(vision.size(), 10u);
    const double span = (vision.back().header.stamp - vision.front().header.stamp).toSec();
    ASSERT_GT(span, 1.0);
    const double vision_hz = static_cast<double>(vision.size() - 1) / span;
    EXPECT_GT(vision_hz, 25.0) << "vision pose rate";
    EXPECT_LT(vision_hz, 35.0) << "vision pose rate";
    // The vision pose is one of the relayed poses, not a different stream.
    EXPECT_LT(vision.size(), poses.size());
    EXPECT_DOUBLE_EQ(vision.front().pose.position.x, poses.front().pose.position.x);
}

TEST(VrpnRelay, AdvertisesAVisionTopicOnlyWhenVisionOutIsSet) {
    // relay_basic and relay_vision set vision_out; the other four relays do not.
    ros::master::V_TopicInfo topics;
    ASSERT_TRUE(ros::master::getTopics(topics));
    std::set<std::string> vision_topics;
    for (const ros::master::TopicInfo& topic : topics) {
        if (topic.name.size() >= 7 &&
            topic.name.compare(topic.name.size() - 7, 7, "/vision") == 0) {
            vision_topics.insert(topic.name);
        }
    }
    EXPECT_EQ(vision_topics,
              (std::set<std::string>{"/relay_basic/vision", "/relay_vision/vision"}));
}

TEST(VrpnRelay, AlignsTheFirstAcceptedPoseAndLeavesMotionAlone) {
    Rig rig(nodeHandle(), "relay_align");
    ASSERT_TRUE(rig.ready());

    // Frames the gate rejects do not fix the alignment.
    EXPECT_TRUE(rig.dropped(0.0, 0.0, 0.5));
    EXPECT_TRUE(rig.dropped(kNaN, 1.0, 0.5));

    ASSERT_TRUE(rig.relayed(-13.19, -4.36, 0.38));
    ASSERT_TRUE(rig.relayed(-12.19, -3.36, 0.5));
    const std::vector<geometry_msgs::PoseStamped> poses = rig.pose.all();
    ASSERT_EQ(poses.size(), 2u);
    EXPECT_NEAR(poses[0].pose.position.x, 1.0, 1e-9);
    EXPECT_NEAR(poses[0].pose.position.y, -2.0, 1e-9);
    EXPECT_NEAR(poses[0].pose.position.z, 0.88, 1e-9);  // z only gets offset_z
    EXPECT_NEAR(poses[1].pose.position.x, 2.0, 1e-9);
    EXPECT_NEAR(poses[1].pose.position.y, -1.0, 1e-9);
    EXPECT_NEAR(poses[1].pose.position.z, 1.0, 1e-9);

    // Twist and acceleration are not shifted.
    rig.tracker.publishTwist(0.5, 0.0);
    ASSERT_TRUE(rig.twist.waitForCount(1));
    EXPECT_DOUBLE_EQ(rig.twist.at(0).twist.linear.x, 0.5);
}

TEST(VrpnRelay, AppliesAFixedOffsetAndAcceptsTheOriginWhenAsked) {
    Rig rig(nodeHandle(), "relay_offset");
    ASSERT_TRUE(rig.ready());

    ASSERT_TRUE(rig.relayed(0.0, 0.0, 1.0));
    ASSERT_TRUE(rig.relayed(0.5, 1.0, 1.5));
    const std::vector<geometry_msgs::PoseStamped> poses = rig.pose.all();
    ASSERT_EQ(poses.size(), 2u);
    EXPECT_DOUBLE_EQ(poses[0].pose.position.x, 0.5);
    EXPECT_DOUBLE_EQ(poses[0].pose.position.y, -1.0);
    EXPECT_DOUBLE_EQ(poses[0].pose.position.z, 1.25);
    EXPECT_DOUBLE_EQ(poses[1].pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(poses[1].pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(poses[1].pose.position.z, 1.75);
}

TEST(VrpnRelay, TakesItsInputsFromThePrefixAndTheExplicitNames) {
    ros::NodeHandle& nh = nodeHandle();
    // Pose and accel from <prefix>/<tracker>/..., twist from the explicit name.
    FakeTracker tracker(nh, "/custom_prefix/relay_topics/pose", "/explicit/twist",
                        "/custom_prefix/relay_topics/accel");
    Collector<geometry_msgs::PoseStamped> pose(nh, "/relay_topics/pose");
    Collector<geometry_msgs::TwistStamped> twist(nh, "/relay_topics/twist");
    Collector<geometry_msgs::AccelStamped> accel(nh, "/relay_topics/accel");
    ASSERT_TRUE(waitUntil(
        [&] {
            return tracker.relayConnected() && pose.connected() && twist.connected() &&
                   accel.connected();
        },
        20.0));

    tracker.publishPose(1.0, 2.0, 0.5);
    tracker.publishTwist(0.5, 0.0);
    tracker.publishAccel(0.25, 0.0);
    ASSERT_TRUE(pose.waitForCount(1));
    ASSERT_TRUE(twist.waitForCount(1));
    ASSERT_TRUE(accel.waitForCount(1));
    EXPECT_DOUBLE_EQ(pose.at(0).pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(twist.at(0).twist.linear.x, 0.5);
    EXPECT_DOUBLE_EQ(accel.at(0).accel.linear.x, 0.25);
}

// Starts the relay binary next to this test with one private parameter set, and returns its exit
// status. The node must refuse a parameter of the wrong type or a number that is not finite.
template <typename Value>
int runRelayWithParameter(const std::string& name, const std::string& parameter,
                          const Value& value) {
    char path[4096];
    const ssize_t length = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (length <= 0) {
        return -1;
    }
    path[length] = '\0';
    std::string directory(path);
    directory.erase(directory.find_last_of('/'));
    ros::param::set("/" + name + "/" + parameter, value);
    if (!ros::param::has("/" + name + "/" + parameter)) {
        return -1;
    }
    // A node that wrongly accepts the parameter would spin forever.
    const std::string command =
        "timeout 20 " + directory + "/vrpn_relay __name:=" + name + " >/dev/null 2>&1";
    const int status = std::system(command.c_str());
    ros::param::del("/" + name);
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

TEST(VrpnRelay, RefusesAParameterOfTheWrongType) {
    EXPECT_EQ(runRelayWithParameter("bad_relay_number", "offset_x", std::string("not a number")),
              1);
    EXPECT_EQ(runRelayWithParameter("bad_relay_bool", "align_xy", std::string("maybe")), 1);
    EXPECT_EQ(runRelayWithParameter("bad_relay_string", "vision_target_hz", std::string("fast")),
              1);
    EXPECT_EQ(runRelayWithParameter("bad_relay_topic", "pose_out", 3.5), 1);
}

TEST(VrpnRelay, RefusesANumberThatIsNotFinite) {
    EXPECT_EQ(
        runRelayWithParameter("bad_relay_inf", "offset_y", std::numeric_limits<double>::infinity()),
        1);
    EXPECT_EQ(runRelayWithParameter("bad_relay_nan", "max_jump_m", kNaN), 1);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "vrpn_relay_node_test");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    const int result = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return result;
}
