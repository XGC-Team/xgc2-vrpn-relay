#include <gtest/gtest.h>

#include "xgc2_vrpn_relay/offset.hpp"

namespace xgc2_vrpn_relay {
namespace {

TEST(Offset, AlignCurrentToOrigin) {
    const Vec3 raw{-13.19, -4.36, 0.38};
    const Vec3 xy = alignmentOffset(raw, 0.0, 0.0);
    const Vec3 aligned = translated(raw, Vec3{xy.x, xy.y, 0.0});
    EXPECT_NEAR(aligned.x, 0.0, 1e-6);
    EXPECT_NEAR(aligned.y, 0.0, 1e-6);
    EXPECT_NEAR(aligned.z, 0.38, 1e-6);
}

TEST(Offset, AlignCurrentToZeroMinusTwo) {
    const Vec3 raw{-13.19, -4.36, 0.38};
    const Vec3 xy = alignmentOffset(raw, 0.0, -2.0);
    const Vec3 aligned = translated(raw, Vec3{xy.x, xy.y, 0.0});
    EXPECT_NEAR(aligned.x, 0.0, 1e-6);
    EXPECT_NEAR(aligned.y, -2.0, 1e-6);
    EXPECT_NEAR(aligned.z, 0.38, 1e-6);
}

TEST(Offset, FixedOffsetLeavesZ) {
    const Vec3 moved = translated(Vec3{1.0, 2.0, 3.0}, Vec3{0.5, -1.0, 0.0});
    EXPECT_EQ(moved.x, 1.5);
    EXPECT_EQ(moved.y, 1.0);
    EXPECT_EQ(moved.z, 3.0);
}

TEST(Offset, ZeroOffset) {
    const Vec3 moved = translated(Vec3{1.0, 2.0, 3.0}, Vec3{});
    EXPECT_EQ(moved.x, 1.0);
    EXPECT_EQ(moved.y, 2.0);
    EXPECT_EQ(moved.z, 3.0);
}

TEST(Offset, AlignmentOffsetHasNoZPart) {
    const Vec3 xy = alignmentOffset(Vec3{1.0, 2.0, 7.0}, 4.0, 6.0);
    EXPECT_EQ(xy.x, 3.0);
    EXPECT_EQ(xy.y, 4.0);
    EXPECT_EQ(xy.z, 0.0);
}

TEST(Offset, TranslationShiftsOnlyPositions) {
    // Orientation, twist and acceleration have no counterpart in the API: only positions move.
    const Vec3 moved = translated(Vec3{-1.0, 1.0, 0.0}, Vec3{2.0, -2.0, 0.25});
    EXPECT_EQ(moved.x, 1.0);
    EXPECT_EQ(moved.y, -1.0);
    EXPECT_EQ(moved.z, 0.25);
}

TEST(WorldOffset, FixedOffsetAppliesToEveryPosition) {
    OffsetConfig config;
    config.offset = Vec3{0.5, -1.0, 0.25};
    WorldOffset world(config);
    EXPECT_TRUE(world.aligned());
    const Vec3 first = world.project(Vec3{1.0, 2.0, 3.0});
    EXPECT_EQ(first.x, 1.5);
    EXPECT_EQ(first.y, 1.0);
    EXPECT_EQ(first.z, 3.25);
    const Vec3 second = world.project(Vec3{-4.0, 8.0, 0.0});
    EXPECT_EQ(second.x, -3.5);
    EXPECT_EQ(second.y, 7.0);
    EXPECT_EQ(second.z, 0.25);
}

TEST(WorldOffset, DefaultConfigDoesNotMoveAnything) {
    WorldOffset world{OffsetConfig()};
    const Vec3 moved = world.project(Vec3{1.0, 2.0, 3.0});
    EXPECT_EQ(moved.x, 1.0);
    EXPECT_EQ(moved.y, 2.0);
    EXPECT_EQ(moved.z, 3.0);
}

TEST(WorldOffset, FirstPositionDefinesTheAlignment) {
    OffsetConfig config;
    config.align_xy = true;
    config.align_to_x = 1.0;
    config.align_to_y = -2.0;
    WorldOffset world(config);
    EXPECT_FALSE(world.aligned());

    const Vec3 first = world.project(Vec3{-13.19, -4.36, 0.38});
    EXPECT_TRUE(world.aligned());
    EXPECT_NEAR(first.x, 1.0, 1e-9);
    EXPECT_NEAR(first.y, -2.0, 1e-9);
    EXPECT_DOUBLE_EQ(first.z, 0.38);

    // Later positions keep the offset of the first one.
    const Vec3 second = world.project(Vec3{-12.19, -3.36, 0.5});
    EXPECT_NEAR(second.x, 2.0, 1e-9);
    EXPECT_NEAR(second.y, -1.0, 1e-9);
    EXPECT_DOUBLE_EQ(second.z, 0.5);
    EXPECT_NEAR(world.offset().x, 1.0 + 13.19, 1e-9);
    EXPECT_NEAR(world.offset().y, -2.0 + 4.36, 1e-9);
}

TEST(WorldOffset, AlignmentToTheOriginIsTheDefaultTarget) {
    OffsetConfig config;
    config.align_xy = true;
    WorldOffset world(config);
    const Vec3 first = world.project(Vec3{5.0, -7.0, 1.0});
    EXPECT_NEAR(first.x, 0.0, 1e-12);
    EXPECT_NEAR(first.y, 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(first.z, 1.0);
}

TEST(WorldOffset, AlignmentIgnoresTheFixedXYOffsetButKeepsZ) {
    OffsetConfig config;
    config.offset = Vec3{100.0, 200.0, 0.5};
    config.align_xy = true;
    config.align_to_x = 3.0;
    config.align_to_y = 4.0;
    WorldOffset world(config);
    const Vec3 first = world.project(Vec3{10.0, 20.0, 1.0});
    EXPECT_NEAR(first.x, 3.0, 1e-12);
    EXPECT_NEAR(first.y, 4.0, 1e-12);
    EXPECT_DOUBLE_EQ(first.z, 1.5);
    const Vec3 second = world.project(Vec3{11.0, 19.0, 1.0});
    EXPECT_NEAR(second.x, 4.0, 1e-12);
    EXPECT_NEAR(second.y, 3.0, 1e-12);
    EXPECT_DOUBLE_EQ(second.z, 1.5);
}

TEST(WorldOffset, FixedOffsetStaysWhenAlignmentIsOff) {
    OffsetConfig config;
    config.offset = Vec3{1.0, 2.0, 3.0};
    config.align_to_x = 50.0;  // ignored without align_xy
    config.align_to_y = 60.0;
    WorldOffset world(config);
    const Vec3 moved = world.project(Vec3{0.5, 0.5, 0.5});
    EXPECT_EQ(moved.x, 1.5);
    EXPECT_EQ(moved.y, 2.5);
    EXPECT_EQ(moved.z, 3.5);
}

}  // namespace
}  // namespace xgc2_vrpn_relay
