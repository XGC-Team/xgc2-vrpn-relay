#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "xgc2_vrpn_relay/vision_cadence.hpp"

namespace xgc2_vrpn_relay {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// The emission times in `published` are fed through take() with the spacing the Python tests
// listed them with, so each of them is emitted by the period rule alone.
VisionPublishCadence cadenceAfter(const std::vector<double>& published, double target_hz = 30.0,
                                  std::size_t window = 5u) {
    VisionPublishCadence cadence(target_hz, window);
    for (const double time : published) {
        EXPECT_TRUE(cadence.take(time)) << time;
    }
    return cadence;
}

TEST(VisionCadence, FirstFrame) {
    VisionPublishCadence cadence(30.0);
    EXPECT_TRUE(cadence.take(0.0));
}

TEST(VisionCadence, DisabledTarget) {
    VisionPublishCadence cadence(0.0);
    EXPECT_TRUE(cadence.take(0.0));
    EXPECT_TRUE(cadence.take(0.01));
    EXPECT_TRUE(cadence.take(0.01));
}

TEST(VisionCadence, NegativeTargetIsDisabledToo) {
    VisionPublishCadence cadence(-5.0);
    for (int frame = 0; frame < 20; ++frame) {
        EXPECT_TRUE(cadence.take(frame * 0.001));
    }
}

TEST(VisionCadence, DropWhenFaster) {
    VisionPublishCadence cadence = cadenceAfter({0.0});
    EXPECT_FALSE(cadence.take(0.01));
}

TEST(VisionCadence, EmitAtPeriod) {
    VisionPublishCadence cadence = cadenceAfter({0.0});
    EXPECT_TRUE(cadence.take(1.0 / 30.0));
}

TEST(VisionCadence, EmitWhenSlower) {
    VisionPublishCadence cadence = cadenceAfter({0.0});
    EXPECT_TRUE(cadence.take(0.1));
}

TEST(VisionCadence, ClockRollback) {
    VisionPublishCadence cadence = cadenceAfter({1.0});
    EXPECT_TRUE(cadence.take(0.0));
}

TEST(VisionCadence, WindowCatchUpWhenRecentRateIsLow) {
    VisionPublishCadence cadence = cadenceAfter({0.00, 0.05, 0.10, 0.15, 0.20});
    EXPECT_TRUE(cadence.take(0.21));
}

TEST(VisionCadence, WindowCatchUpStopsOnceTheRecentRateIsReached) {
    // Five emissions over 0.14 s are 36 Hz, above the target: nothing is owed.
    VisionPublishCadence cadence = cadenceAfter({0.000, 0.034, 0.068, 0.102, 0.136});
    EXPECT_FALSE(cadence.take(0.14));
}

TEST(VisionCadence, A120HzInputStaysNear30HzOutput) {
    VisionPublishCadence cadence(30.0, 5u);
    int emits = 0;
    for (int index = 0; index < 120; ++index) {
        if (cadence.take(index / 120.0)) {
            ++emits;
        }
    }
    EXPECT_EQ(emits, 30);
    EXPECT_FALSE(cadence.take(119.5 / 120.0));
}

TEST(VisionCadence, TheDefaultsAreThirtyHertzAndAWindowOfFive) {
    VisionPublishCadence defaults;
    VisionPublishCadence explicit_values(30.0, 5u);
    for (int index = 0; index < 600; ++index) {
        const double now = index / 100.0;
        EXPECT_EQ(defaults.take(now), explicit_values.take(now)) << index;
    }
}

TEST(VisionCadence, TheWindowHoldsTheLastFiveEmissions) {
    // Six emissions, one every 0.0334 s: just above the 1 / 30 s period, so the period rule emits
    // each of them. A window of five forgets the first one.
    const std::vector<double> published = {-0.0334, 0.0, 0.0334, 0.0668, 0.1002, 0.1336};
    // Five emissions over 0.1667 s are 29.99 Hz, below the target: the catch-up rule emits.
    EXPECT_TRUE(cadenceAfter(published).take(0.1667));
    // Five over 0.16665 s are 30.003 Hz: nothing is owed.
    EXPECT_FALSE(cadenceAfter(published).take(0.16665));
    // A window of six also sees the first emission: six over 0.20005 s are 29.99 Hz.
    EXPECT_TRUE(cadenceAfter(published, 30.0, 6u).take(0.16665));
    // A window of four sees four over 0.1333 s at 0.1667, which is 30.01 Hz.
    EXPECT_FALSE(cadenceAfter(published, 30.0, 4u).take(0.1667));
}

TEST(VisionCadence, TwentySecondsOf120HzInputEmitWhatThePythonHelperEmitted) {
    // The period rule compares frame spacings that are exactly 1 / 30 s on paper but off by a
    // rounding error in doubles, so a few emissions slip by one frame and the long-run rate ends
    // slightly under 30 Hz. The original Python helper counted 585 for the same input.
    VisionPublishCadence cadence(30.0, 5u);
    int emits = 0;
    for (int frame = 0; frame < 2400; ++frame) {
        if (cadence.take(frame / 120.0)) {
            ++emits;
        }
    }
    EXPECT_EQ(emits, 585);
}

TEST(VisionCadence, SlowInputPassesEveryFrame) {
    VisionPublishCadence cadence(30.0, 5u);
    for (int frame = 0; frame < 100; ++frame) {
        EXPECT_TRUE(cadence.take(frame * 0.05)) << frame;  // 20 Hz
    }
}

TEST(VisionCadence, SurvivesAClockRewindAfterALongRun) {
    VisionPublishCadence cadence(30.0, 5u);
    for (int frame = 0; frame < 2400; ++frame) {
        cadence.take(frame / 120.0);
    }
    EXPECT_FALSE(cadence.take(kNaN));
    EXPECT_TRUE(cadence.take(0.0));
    EXPECT_FALSE(cadence.take(0.005));
    EXPECT_TRUE(cadence.take(0.035));
}

TEST(VisionCadence, NonFiniteTimeIsNeverEmittedOrCounted) {
    VisionPublishCadence cadence(30.0, 5u);
    EXPECT_FALSE(cadence.take(kNaN));
    EXPECT_FALSE(cadence.take(std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(cadence.take(0.0));  // still the first frame
}

TEST(VisionCadence, EqualTimestampsAreDroppedAfterTheFirst) {
    VisionPublishCadence cadence(30.0, 5u);
    EXPECT_TRUE(cadence.take(1.0));
    EXPECT_FALSE(cadence.take(1.0));
    EXPECT_FALSE(cadence.take(1.0));
}

TEST(VisionCadence, RejectsAnInvalidConfiguration) {
    EXPECT_THROW(VisionPublishCadence(kNaN, 5u), std::invalid_argument);
    EXPECT_THROW(VisionPublishCadence(std::numeric_limits<double>::infinity(), 5u),
                 std::invalid_argument);
    EXPECT_THROW(VisionPublishCadence(30.0, 0u), std::invalid_argument);
    EXPECT_NO_THROW(VisionPublishCadence(0.0, 1u));
}

TEST(VisionCadence, AWindowOfOneUsesOnlyThePeriodAndThatSingleSpan) {
    VisionPublishCadence cadence(30.0, 1u);
    EXPECT_TRUE(cadence.take(0.0));
    // One emission 0.02 s ago is 50 Hz: drop. The span rule never fires with a window of one
    // before the period passes, because 1 / span >= 30 exactly when span <= 1 / 30.
    EXPECT_FALSE(cadence.take(0.02));
    EXPECT_TRUE(cadence.take(0.034));
}

}  // namespace
}  // namespace xgc2_vrpn_relay
