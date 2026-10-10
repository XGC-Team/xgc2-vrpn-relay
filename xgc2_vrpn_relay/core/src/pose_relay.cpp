#include "xgc2_vrpn_relay/pose_relay.hpp"

namespace xgc2_vrpn_relay {

PoseRelay::PoseRelay(const PoseRelayConfig& config)
    : gate_(config.quality), offset_(config.offset) {}

PoseRelayResult PoseRelay::process(const Pose& raw, double receipt_s) {
    PoseRelayResult result;
    const PoseVerdict verdict = gate_.check(raw, receipt_s);
    result.rejection = verdict.rejection;
    if (!verdict.accepted()) {
        return result;
    }
    result.aligned_now = !offset_.aligned();
    result.pose.position = offset_.project(raw.position);
    result.pose.orientation = verdict.orientation;
    return result;
}

}  // namespace xgc2_vrpn_relay
