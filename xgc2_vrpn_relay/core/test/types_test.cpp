#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "xgc2_vrpn_relay/types.hpp"

namespace xgc2_vrpn_relay {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// The Python helper also had a case for a non-numeric value ("1"); the types rule that out.
TEST(Finite, AcceptsNumbers) {
    EXPECT_TRUE(isFinite(Vec3{1.0, 0.0, -2.5}));
    EXPECT_TRUE(isFinite(Quaternion{0.0, 0.0, 0.0, 1.0}));
    EXPECT_TRUE(isFinite(Pose{}));
    EXPECT_TRUE(isFinite(Motion{}));
}

TEST(Finite, RejectsNaNInEveryComponent) {
    EXPECT_FALSE(isFinite(Vec3{kNaN, 0.0, 0.0}));
    EXPECT_FALSE(isFinite(Vec3{0.0, kNaN, 0.0}));
    EXPECT_FALSE(isFinite(Vec3{0.0, 0.0, kNaN}));
    EXPECT_FALSE(isFinite(Quaternion{kNaN, 0.0, 0.0, 1.0}));
    EXPECT_FALSE(isFinite(Quaternion{0.0, kNaN, 0.0, 1.0}));
    EXPECT_FALSE(isFinite(Quaternion{0.0, 0.0, kNaN, 1.0}));
    EXPECT_FALSE(isFinite(Quaternion{0.0, 0.0, 0.0, kNaN}));
}

TEST(Finite, RejectsInfinityInEveryComponent) {
    EXPECT_FALSE(isFinite(Vec3{kInf, 0.0, 0.0}));
    EXPECT_FALSE(isFinite(Vec3{0.0, -kInf, 0.0}));
    EXPECT_FALSE(isFinite(Vec3{0.0, 0.0, kInf}));
    EXPECT_FALSE(isFinite(Quaternion{kInf, 0.0, 0.0, 1.0}));
    EXPECT_FALSE(isFinite(Quaternion{0.0, -kInf, 0.0, 1.0}));
    EXPECT_FALSE(isFinite(Quaternion{0.0, 0.0, kInf, 1.0}));
    EXPECT_FALSE(isFinite(Quaternion{0.0, 0.0, 0.0, -kInf}));
}

TEST(Finite, PoseChecksPositionAndOrientation) {
    Pose pose;
    pose.position.z = kNaN;
    EXPECT_FALSE(isFinite(pose));
    pose = Pose{};
    pose.orientation.w = kInf;
    EXPECT_FALSE(isFinite(pose));
}

TEST(Finite, MotionChecksLinearAndAngular) {
    for (const double bad : {kNaN, kInf, -kInf}) {
        Motion linear;
        linear.linear.y = bad;
        EXPECT_FALSE(isFinite(linear));
        Motion angular;
        angular.angular.z = bad;
        EXPECT_FALSE(isFinite(angular));
    }
    Motion large;
    large.linear = Vec3{1.0e300, -1.0e300, 0.0};
    EXPECT_TRUE(isFinite(large));
}

TEST(Geometry, DistanceIsEuclideanIn3D) {
    EXPECT_DOUBLE_EQ(distance(Vec3{1.0, 2.0, 0.5}, Vec3{1.0, 2.0, 0.5}), 0.0);
    EXPECT_DOUBLE_EQ(distance(Vec3{0.0, 0.0, 0.0}, Vec3{3.0, 4.0, 12.0}), 13.0);
    EXPECT_DOUBLE_EQ(distance(Vec3{3.0, 4.0, 12.0}, Vec3{0.0, 0.0, 0.0}), 13.0);
}

TEST(Quaternion, NormIsTheLengthOfTheFourComponents) {
    EXPECT_DOUBLE_EQ(norm(Quaternion{0.0, 0.0, 0.0, 1.0}), 1.0);
    EXPECT_DOUBLE_EQ(norm(Quaternion{1.0, 2.0, 2.0, 4.0}), 5.0);
}

TEST(Quaternion, NormalizedScalesToUnitLength) {
    const auto unit = normalized(Quaternion{0.0, 0.0, 0.0, 2.0});
    ASSERT_TRUE(unit.has_value());
    EXPECT_DOUBLE_EQ(unit->w, 1.0);

    const auto skew = normalized(Quaternion{1.0, 2.0, 2.0, 4.0});
    ASSERT_TRUE(skew.has_value());
    EXPECT_DOUBLE_EQ(norm(*skew), 1.0);
    EXPECT_DOUBLE_EQ(skew->x, 0.2);
    EXPECT_DOUBLE_EQ(skew->w, 0.8);
}

TEST(Quaternion, NormalizedRefusesAZeroOrTinyQuaternion) {
    EXPECT_FALSE(normalized(Quaternion{0.0, 0.0, 0.0, 0.0}).has_value());
    EXPECT_FALSE(normalized(Quaternion{0.0, 0.0, 0.0, 0.5e-6}).has_value());
    EXPECT_FALSE(normalized(Quaternion{-0.5e-6, 0.0, 0.0, 0.0}).has_value());
    EXPECT_TRUE(normalized(Quaternion{0.0, 0.0, 0.0, 2.0e-6}).has_value());
}

TEST(Quaternion, NormalizedRefusesNonFiniteComponentsAndAnOverflowingNorm) {
    EXPECT_FALSE(normalized(Quaternion{kNaN, 0.0, 0.0, 1.0}).has_value());
    EXPECT_FALSE(normalized(Quaternion{0.0, 0.0, 0.0, kInf}).has_value());
    // The sum of squares overflows although every component is finite.
    EXPECT_FALSE(normalized(Quaternion{1.0e200, 0.0, 0.0, 1.0e200}).has_value());
}

}  // namespace
}  // namespace xgc2_vrpn_relay
