#include "xgc2_vrpn_relay/types.hpp"

#include <cmath>

namespace xgc2_vrpn_relay {

bool isFinite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool isFinite(const Quaternion& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) &&
           std::isfinite(value.w);
}

bool isFinite(const Pose& value) {
    return isFinite(value.position) && isFinite(value.orientation);
}

bool isFinite(const Motion& value) {
    return isFinite(value.linear) && isFinite(value.angular);
}

double distance(const Vec3& a, const Vec3& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double norm(const Quaternion& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w);
}

std::optional<Quaternion> normalized(const Quaternion& value) {
    if (!isFinite(value)) {
        return std::nullopt;
    }
    const double length = norm(value);
    if (!std::isfinite(length) || length < kMinQuaternionNorm) {
        return std::nullopt;
    }
    return Quaternion{value.x / length, value.y / length, value.z / length, value.w / length};
}

}  // namespace xgc2_vrpn_relay
