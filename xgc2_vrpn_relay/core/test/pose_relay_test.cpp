#include <gtest/gtest.h>

#include <limits>

#include "xgc2_vrpn_relay/pose_relay.hpp"

namespace xgc2_vrpn_relay {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

Pose poseAt(double x, double y, double z = 0.5, double qw = 1.0) {
    return Pose{Vec3{x, y, z}, Quaternion{0.0, 0.0, 0.0, qw}};
}

TEST(PoseRelay, AcceptedPoseIsNormalizedAndOffset) {
    PoseRelayConfig config;
    config.offset.offset = Vec3{0.5, -1.0, 0.25};
    PoseRelay relay(config);
    const PoseRelayResult result = relay.process(poseAt(1.0, 2.0, 3.0, 2.0), 1.0);
    ASSERT_TRUE(result.accepted());
    EXPECT_EQ(result.rejection, PoseRejection::kNone);
    EXPECT_FALSE(result.aligned_now);
    EXPECT_DOUBLE_EQ(result.pose.position.x, 1.5);
    EXPECT_DOUBLE_EQ(result.pose.position.y, 1.0);
    EXPECT_DOUBLE_EQ(result.pose.position.z, 3.25);
    EXPECT_DOUBLE_EQ(result.pose.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(relay.offset().x, 0.5);
}

TEST(PoseRelay, RejectedPoseCarriesTheReason) {
    PoseRelay relay{PoseRelayConfig()};
    const PoseRelayResult result = relay.process(poseAt(kNaN, 2.0), 1.0);
    EXPECT_FALSE(result.accepted());
    EXPECT_EQ(result.rejection, PoseRejection::kNonFinite);
    EXPECT_FALSE(result.aligned_now);
}

TEST(PoseRelay, QualityChecksSeeRawCoordinatesNotShiftedOnes) {
    PoseRelayConfig config;
    config.offset.offset = Vec3{-1.0, -2.0, 0.0};
    PoseRelay relay(config);
    // Shifted onto (0, 0), but the tracker did not report the origin.
    const PoseRelayResult shifted_to_origin = relay.process(poseAt(1.0, 2.0), 1.0);
    ASSERT_TRUE(shifted_to_origin.accepted());
    EXPECT_DOUBLE_EQ(shifted_to_origin.pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(shifted_to_origin.pose.position.y, 0.0);
    // The origin is a rejection when the tracker reports it, whatever the offset would make of it.
    PoseRelay relay_two(config);
    EXPECT_EQ(relay_two.process(poseAt(0.0, 0.0), 1.0).rejection, PoseRejection::kOrigin);
}

TEST(PoseRelay, XYBoundAppliesToTheRawPosition) {
    PoseRelayConfig config;
    config.offset.offset = Vec3{-100.0, 0.0, 0.0};
    PoseRelay relay(config);
    EXPECT_EQ(relay.process(poseAt(950.0, 1.0), 1.0).rejection, PoseRejection::kAbsXy);
    config.offset.offset = Vec3{100.0, 0.0, 0.0};
    PoseRelay relay_two(config);
    const PoseRelayResult result = relay_two.process(poseAt(850.0, 1.0), 1.0);
    ASSERT_TRUE(result.accepted());
    EXPECT_DOUBLE_EQ(result.pose.position.x, 950.0);
}

TEST(PoseRelay, FirstAcceptedPoseFixesTheAlignment) {
    PoseRelayConfig config;
    config.offset.align_xy = true;
    config.offset.align_to_x = 1.0;
    config.offset.align_to_y = -2.0;
    config.offset.offset.z = 0.1;
    PoseRelay relay(config);

    // Rejected frames do not align.
    EXPECT_EQ(relay.process(poseAt(0.0, 0.0), 1.00).rejection, PoseRejection::kOrigin);
    EXPECT_EQ(relay.process(poseAt(kNaN, 3.0), 1.01).rejection, PoseRejection::kNonFinite);
    EXPECT_EQ(relay.process(poseAt(2000.0, 3.0), 1.02).rejection, PoseRejection::kAbsXy);

    const PoseRelayResult first = relay.process(poseAt(-13.19, -4.36, 0.38), 1.03);
    ASSERT_TRUE(first.accepted());
    EXPECT_TRUE(first.aligned_now);
    EXPECT_NEAR(first.pose.position.x, 1.0, 1e-9);
    EXPECT_NEAR(first.pose.position.y, -2.0, 1e-9);
    EXPECT_NEAR(first.pose.position.z, 0.48, 1e-12);

    const PoseRelayResult second = relay.process(poseAt(-13.0, -4.0, 0.4), 1.04);
    ASSERT_TRUE(second.accepted());
    EXPECT_FALSE(second.aligned_now);
    EXPECT_NEAR(second.pose.position.x, 1.19, 1e-9);
    EXPECT_NEAR(second.pose.position.y, -1.64, 1e-9);
    EXPECT_NEAR(second.pose.position.z, 0.5, 1e-12);
    EXPECT_NEAR(relay.offset().x, 14.19, 1e-9);
    EXPECT_NEAR(relay.offset().y, 2.36, 1e-9);
}

TEST(PoseRelay, ARestartAlignsToTheNewFirstFrame) {
    PoseRelayConfig config;
    config.offset.align_xy = true;
    PoseRelay first_run(config);
    ASSERT_TRUE(first_run.process(poseAt(3.0, 4.0), 1.0).accepted());
    EXPECT_NEAR(first_run.offset().x, -3.0, 1e-12);

    PoseRelay second_run(config);
    const PoseRelayResult result = second_run.process(poseAt(10.0, -5.0), 2.0);
    ASSERT_TRUE(result.accepted());
    EXPECT_TRUE(result.aligned_now);
    EXPECT_NEAR(result.pose.position.x, 0.0, 1e-12);
    EXPECT_NEAR(result.pose.position.y, 0.0, 1e-12);
}

TEST(PoseRelay, JumpAndFrozenWorkOnTheRawPathWhenAligned) {
    PoseRelayConfig config;
    config.offset.align_xy = true;
    config.offset.align_to_x = 100.0;  // far from the raw coordinates
    PoseRelay relay(config);
    ASSERT_TRUE(relay.process(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(relay.process(poseAt(4.5, 2.0), 0.1).rejection, PoseRejection::kJump);
    EXPECT_TRUE(relay.process(poseAt(1.0, 2.0), 0.2).accepted());
    EXPECT_EQ(relay.process(poseAt(1.0, 2.0), 0.5).rejection, PoseRejection::kFrozen);
    const PoseRelayResult moved = relay.process(poseAt(1.5, 2.0), 0.6);
    ASSERT_TRUE(moved.accepted());
    EXPECT_FALSE(moved.aligned_now);
    EXPECT_NEAR(moved.pose.position.x, 100.5, 1e-12);
}

TEST(PoseRelay, FixedOffsetNeverReportsAlignment) {
    PoseRelayConfig config;
    config.offset.offset = Vec3{1.0, 1.0, 1.0};
    PoseRelay relay(config);
    for (int frame = 0; frame < 5; ++frame) {
        const PoseRelayResult result = relay.process(poseAt(2.0 + frame * 0.1, 3.0), frame * 0.01);
        ASSERT_TRUE(result.accepted());
        EXPECT_FALSE(result.aligned_now);
    }
}

TEST(PoseRelay, UsesTheQualityConfig) {
    PoseRelayConfig config;
    config.quality.reject_origin = false;
    config.quality.frozen_s = 0.0;
    PoseRelay relay(config);
    EXPECT_TRUE(relay.process(poseAt(0.0, 0.0), 0.0).accepted());
    EXPECT_TRUE(relay.process(poseAt(0.0, 0.0), 100.0).accepted());
}

}  // namespace
}  // namespace xgc2_vrpn_relay
