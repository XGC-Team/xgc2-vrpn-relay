// Builds against an installed xgc2_vrpn_relay_core, as the PX4 adapter will, and runs the
// operations it needs: pose validation and offset projection, twist validation, vision cadence.
#include <cstdio>
#include <limits>

#include "xgc2_vrpn_relay/pose_relay.hpp"
#include "xgc2_vrpn_relay/vision_cadence.hpp"

namespace relay = xgc2_vrpn_relay;

int main() {
    relay::PoseRelayConfig config;
    config.offset.offset = relay::Vec3{1.0, -2.0, 0.5};
    relay::PoseRelay pose_relay(config);

    relay::Pose raw;
    raw.position = relay::Vec3{2.0, 3.0, 1.0};
    raw.orientation = relay::Quaternion{0.0, 0.0, 0.0, 2.0};
    const relay::PoseRelayResult accepted = pose_relay.process(raw, 1.0);
    if (!accepted.accepted() || accepted.pose.position.x != 3.0 ||
        accepted.pose.position.y != 1.0 || accepted.pose.position.z != 1.5 ||
        accepted.pose.orientation.w != 1.0) {
        std::fprintf(stderr, "pose was not accepted and projected\n");
        return 1;
    }

    raw.position.x = std::numeric_limits<double>::quiet_NaN();
    if (pose_relay.process(raw, 1.01).rejection != relay::PoseRejection::kNonFinite) {
        std::fprintf(stderr, "a NaN pose was not rejected\n");
        return 2;
    }

    relay::Motion twist;
    twist.angular.z = std::numeric_limits<double>::infinity();
    if (relay::isFinite(twist)) {
        std::fprintf(stderr, "an infinite twist was accepted\n");
        return 3;
    }

    relay::VisionPublishCadence cadence;
    if (!cadence.take(0.0) || cadence.take(0.001)) {
        std::fprintf(stderr, "vision cadence did not thin the second frame\n");
        return 4;
    }
    return 0;
}
