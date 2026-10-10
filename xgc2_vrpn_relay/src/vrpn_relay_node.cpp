// Relays one VRPN tracker onto local ROS topics.
//
// Every robot: pose / twist / accel, one-for-one after a quality gate.
// PX4 only: also publish a vision pose, thinned to the target rate.
//
// The logic lives in the ROS-free core library (xgc2_vrpn_relay_core); this file only reads the
// parameters, moves messages in and out, and logs.
#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include "xgc2_vrpn_relay/pose_relay.hpp"
#include "xgc2_vrpn_relay/vision_cadence.hpp"

namespace {

namespace core = xgc2_vrpn_relay;

struct NodeConfig {
    std::string tracker;
    std::string pose_in;
    std::string twist_in;
    std::string accel_in;
    std::string pose_out;
    std::string twist_out;
    std::string accel_out;
    std::string vision_out;  // empty: no vision pose
    double vision_target_hz = 30.0;
    core::PoseRelayConfig relay;
};

// Private parameters. A parameter that is set but has the wrong type is an error, not a silent
// fallback to the default.
std::string stringParam(ros::NodeHandle& nh, const std::string& name, const std::string& fallback) {
    if (!nh.hasParam(name)) {
        return fallback;
    }
    std::string value;
    if (!nh.getParam(name, value)) {
        throw std::runtime_error("parameter ~" + name + " must be a string");
    }
    return value;
}

// Numbers must be finite: a NaN bound would silently switch a check off, a NaN offset would
// poison every pose.
double doubleParam(ros::NodeHandle& nh, const std::string& name, double fallback) {
    if (!nh.hasParam(name)) {
        return fallback;
    }
    double value = 0.0;
    if (!nh.getParam(name, value) || !std::isfinite(value)) {
        throw std::runtime_error("parameter ~" + name + " must be a finite number");
    }
    return value;
}

bool boolParam(ros::NodeHandle& nh, const std::string& name, bool fallback) {
    if (!nh.hasParam(name)) {
        return fallback;
    }
    XmlRpc::XmlRpcValue value;
    nh.getParam(name, value);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        return static_cast<bool>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(value) != 0;
    }
    throw std::runtime_error("parameter ~" + name + " must be a boolean");
}

NodeConfig loadConfig(ros::NodeHandle& pnh) {
    NodeConfig config;
    config.tracker = stringParam(pnh, "tracker", "tracker");
    const std::string prefix = stringParam(pnh, "vrpn_prefix", "/vrpn_client_node");
    const std::string tracker_root = prefix + "/" + config.tracker;
    config.pose_in = stringParam(pnh, "pose_in", tracker_root + "/pose");
    config.twist_in = stringParam(pnh, "twist_in", tracker_root + "/twist");
    config.accel_in = stringParam(pnh, "accel_in", tracker_root + "/accel");
    config.pose_out = stringParam(pnh, "pose_out", "/pose");
    config.twist_out = stringParam(pnh, "twist_out", "/twist");
    config.accel_out = stringParam(pnh, "accel_out", "/accel");
    config.vision_out = stringParam(pnh, "vision_out", "");
    config.vision_target_hz = doubleParam(pnh, "vision_target_hz", 30.0);

    const core::PoseQualityConfig quality_defaults;
    core::PoseQualityConfig& quality = config.relay.quality;
    quality.reject_origin = boolParam(pnh, "reject_origin", quality_defaults.reject_origin);
    quality.max_abs_xy = doubleParam(pnh, "max_abs_xy", quality_defaults.max_abs_xy);
    quality.max_jump_m = doubleParam(pnh, "max_jump_m", quality_defaults.max_jump_m);
    quality.frozen_s = doubleParam(pnh, "frozen_s", quality_defaults.frozen_s);

    core::OffsetConfig& offset = config.relay.offset;
    offset.offset.x = doubleParam(pnh, "offset_x", 0.0);
    offset.offset.y = doubleParam(pnh, "offset_y", 0.0);
    offset.offset.z = doubleParam(pnh, "offset_z", 0.0);
    offset.align_xy = boolParam(pnh, "align_xy", false);
    offset.align_to_x = doubleParam(pnh, "align_to_x", 0.0);
    offset.align_to_y = doubleParam(pnh, "align_to_y", 0.0);
    return config;
}

core::Vec3 toCore(const geometry_msgs::Point& value) {
    return core::Vec3{value.x, value.y, value.z};
}

core::Vec3 toCore(const geometry_msgs::Vector3& value) {
    return core::Vec3{value.x, value.y, value.z};
}

core::Pose toCore(const geometry_msgs::Pose& value) {
    const geometry_msgs::Quaternion& q = value.orientation;
    return core::Pose{toCore(value.position), core::Quaternion{q.x, q.y, q.z, q.w}};
}

core::Motion toCore(const geometry_msgs::Twist& value) {
    return core::Motion{toCore(value.linear), toCore(value.angular)};
}

void toRos(const core::Pose& value, geometry_msgs::Pose* out) {
    out->position.x = value.position.x;
    out->position.y = value.position.y;
    out->position.z = value.position.z;
    out->orientation.x = value.orientation.x;
    out->orientation.y = value.orientation.y;
    out->orientation.z = value.orientation.z;
    out->orientation.w = value.orientation.w;
}

class VrpnRelay {
public:
    VrpnRelay(ros::NodeHandle& nh, const NodeConfig& config)
        : relay_(config.relay), vision_cadence_(config.vision_target_hz),
          align_to_x_(config.relay.offset.align_to_x), align_to_y_(config.relay.offset.align_to_y) {
        // Publishers first, so no callback can run before its output exists.
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(config.pose_out, 1);
        twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>(config.twist_out, 1);
        accel_pub_ = nh.advertise<geometry_msgs::AccelStamped>(config.accel_out, 1);
        if (!config.vision_out.empty()) {
            vision_pub_ = nh.advertise<geometry_msgs::PoseStamped>(config.vision_out, 1);
        }
        pose_sub_ = nh.subscribe(config.pose_in, 1, &VrpnRelay::onPose, this);
        twist_sub_ = nh.subscribe(config.twist_in, 1, &VrpnRelay::onTwist, this);
        accel_sub_ = nh.subscribe(config.accel_in, 1, &VrpnRelay::onAccel, this);
    }

private:
    // The receipt time, not the time the callback runs, drives frozen detection and the vision
    // cadence, so a busy spinner does not look like a frozen tracker.
    void onPose(const ros::MessageEvent<geometry_msgs::PoseStamped const>& event) {
        const geometry_msgs::PoseStamped& message = *event.getConstMessage();
        const ros::Time receipt = event.getReceiptTime();
        const double receipt_s = receipt.toSec();

        const core::PoseRelayResult result = relay_.process(toCore(message.pose), receipt_s);
        if (!result.accepted()) {
            ROS_WARN_THROTTLE(2.0, "drop vrpn pose: %s", core::toString(result.rejection));
            return;
        }
        if (result.aligned_now) {
            ROS_INFO("vrpn_relay align raw=(%.3f, %.3f) -> (%.3f, %.3f) offset=(%.3f, %.3f)",
                     message.pose.position.x, message.pose.position.y, align_to_x_, align_to_y_,
                     relay_.offset().x, relay_.offset().y);
        }

        geometry_msgs::PoseStamped out = message;
        toRos(result.pose, &out.pose);
        if (out.header.stamp.isZero()) {
            out.header.stamp = receipt;
        }
        pose_pub_.publish(out);
        if (vision_pub_ && vision_cadence_.take(receipt_s)) {
            vision_pub_.publish(out);
        }
    }

    void onTwist(const geometry_msgs::TwistStamped::ConstPtr& message) {
        if (!core::isFinite(toCore(message->twist))) {
            ROS_WARN_THROTTLE(2.0, "drop vrpn twist: non-finite value");
            return;
        }
        twist_pub_.publish(message);
    }

    // vrpn_client_ros publishes the acceleration as a TwistStamped (linear and angular
    // acceleration in the twist fields); it goes out as the AccelStamped it is.
    void onAccel(const geometry_msgs::TwistStamped::ConstPtr& message) {
        if (!core::isFinite(toCore(message->twist))) {
            ROS_WARN_THROTTLE(2.0, "drop vrpn accel: non-finite value");
            return;
        }
        geometry_msgs::AccelStamped out;
        out.header = message->header;
        out.accel.linear = message->twist.linear;
        out.accel.angular = message->twist.angular;
        accel_pub_.publish(out);
    }

    core::PoseRelay relay_;
    core::VisionPublishCadence vision_cadence_;
    const double align_to_x_;
    const double align_to_y_;
    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    ros::Publisher accel_pub_;
    ros::Publisher vision_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber accel_sub_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "vrpn_relay");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    try {
        const NodeConfig config = loadConfig(pnh);
        const core::OffsetConfig& offset = config.relay.offset;
        ROS_INFO("vrpn_relay tracker=%s pose=%s vision=%s target_hz=%.1f "
                 "offset=(%.3f, %.3f, %.3f) align_xy=%s align_to=(%.3f, %.3f)",
                 config.tracker.c_str(), config.pose_out.c_str(),
                 config.vision_out.empty() ? "off" : config.vision_out.c_str(),
                 config.vision_target_hz, offset.offset.x, offset.offset.y, offset.offset.z,
                 offset.align_xy ? "true" : "false", offset.align_to_x, offset.align_to_y);
        VrpnRelay relay(nh, config);
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("vrpn_relay: %s", error.what());
        return 1;
    }
    return 0;
}
