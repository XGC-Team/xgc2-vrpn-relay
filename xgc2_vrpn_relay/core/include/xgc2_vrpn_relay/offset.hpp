#pragma once

#include "xgc2_vrpn_relay/types.hpp"

namespace xgc2_vrpn_relay {

// The position translation applied after the quality gate. Orientation, twist and acceleration are
// never shifted: a change of origin does not change them.
struct OffsetConfig {
    // Fixed world-frame translation added to every position.
    Vec3 offset;
    // Instead of offset.x and offset.y, translate so that the first position handed to
    // WorldOffset::project lands on (align_to_x, align_to_y). There is no yaw alignment and z keeps
    // using offset.z.
    bool align_xy = false;
    double align_to_x = 0.0;
    double align_to_y = 0.0;
};

Vec3 translated(const Vec3& position, const Vec3& offset);

// The translation (z is 0) that moves position.xy onto (target_x, target_y).
Vec3 alignmentOffset(const Vec3& position, double target_x, double target_y);

class WorldOffset {
public:
    explicit WorldOffset(const OffsetConfig& config);

    // The position in the target frame. With align_xy, the first call fixes the offset.
    Vec3 project(const Vec3& position);

    // False only until the first call of project when align_xy is set.
    bool aligned() const { return aligned_; }

    // The translation project applies, with the x/y part still unset before alignment.
    const Vec3& offset() const { return offset_; }

private:
    OffsetConfig config_;
    Vec3 offset_;
    bool aligned_;
};

}  // namespace xgc2_vrpn_relay
