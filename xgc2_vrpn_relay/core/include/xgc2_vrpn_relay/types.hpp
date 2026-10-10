#pragma once

#include <optional>

namespace xgc2_vrpn_relay {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quaternion {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

struct Pose {
    Vec3 position;
    Quaternion orientation;
};

// Linear and angular part of a twist or an acceleration.
struct Motion {
    Vec3 linear;
    Vec3 angular;
};

// A quaternion shorter than this cannot be normalized into an orientation.
constexpr double kMinQuaternionNorm = 1.0e-6;

// True when every component is neither NaN nor infinite.
bool isFinite(const Vec3& value);
bool isFinite(const Quaternion& value);
bool isFinite(const Pose& value);
bool isFinite(const Motion& value);

double distance(const Vec3& a, const Vec3& b);
double norm(const Quaternion& value);

// The quaternion scaled to unit length, or no value when it is not finite or its norm (computed
// as the square root of the sum of squares) is not finite or below kMinQuaternionNorm.
std::optional<Quaternion> normalized(const Quaternion& value);

}  // namespace xgc2_vrpn_relay
