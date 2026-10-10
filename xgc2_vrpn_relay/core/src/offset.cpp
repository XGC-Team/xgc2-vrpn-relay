#include "xgc2_vrpn_relay/offset.hpp"

namespace xgc2_vrpn_relay {

Vec3 translated(const Vec3& position, const Vec3& offset) {
    return Vec3{position.x + offset.x, position.y + offset.y, position.z + offset.z};
}

Vec3 alignmentOffset(const Vec3& position, double target_x, double target_y) {
    return Vec3{target_x - position.x, target_y - position.y, 0.0};
}

WorldOffset::WorldOffset(const OffsetConfig& config)
    : config_(config), offset_(config.offset), aligned_(!config.align_xy) {}

Vec3 WorldOffset::project(const Vec3& position) {
    if (!aligned_) {
        const Vec3 xy = alignmentOffset(position, config_.align_to_x, config_.align_to_y);
        offset_.x = xy.x;
        offset_.y = xy.y;
        aligned_ = true;
    }
    return translated(position, offset_);
}

}  // namespace xgc2_vrpn_relay
