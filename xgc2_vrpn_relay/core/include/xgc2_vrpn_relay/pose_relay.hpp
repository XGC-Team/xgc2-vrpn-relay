#pragma once

#include "xgc2_vrpn_relay/offset.hpp"
#include "xgc2_vrpn_relay/quality.hpp"
#include "xgc2_vrpn_relay/types.hpp"

namespace xgc2_vrpn_relay {

struct PoseRelayConfig {
    PoseQualityConfig quality;
    OffsetConfig offset;
};

struct PoseRelayResult {
    PoseRejection rejection = PoseRejection::kNone;
    // Meaningful only when accepted: unit quaternion, position in the target frame.
    Pose pose;
    // True for the one frame that fixed the first-frame XY alignment.
    bool aligned_now = false;

    bool accepted() const { return rejection == PoseRejection::kNone; }
};

// The pose path of one tracker: the quality gate on the raw pose, then the world-frame offset on
// the accepted position. Quality checks always see raw coordinates, so the offset never changes
// what is accepted. Not thread-safe.
class PoseRelay {
public:
    explicit PoseRelay(const PoseRelayConfig& config);

    // receipt_s is the time the pose was received, in seconds. It drives frozen detection.
    PoseRelayResult process(const Pose& raw, double receipt_s);

    // The translation applied to accepted positions.
    const Vec3& offset() const { return offset_.offset(); }

private:
    PoseQualityGate gate_;
    WorldOffset offset_;
};

}  // namespace xgc2_vrpn_relay
