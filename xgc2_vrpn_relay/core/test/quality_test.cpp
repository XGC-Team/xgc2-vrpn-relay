#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "xgc2_vrpn_relay/quality.hpp"

namespace xgc2_vrpn_relay {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// One evaluatePose call with the defaults of the original Python helper tests: a pose at
// (1, 2, 0.5) with the identity orientation, received at t = 1 s, no history.
struct Case {
    Pose pose = Pose{Vec3{1.0, 2.0, 0.5}, Quaternion{0.0, 0.0, 0.0, 1.0}};
    PoseHistory history;
    double now_s = 1.0;
    PoseQualityConfig config;

    PoseVerdict run() const { return evaluatePose(pose, history, now_s, config); }
};

TEST(PoseQuality, AcceptsUnitQuaternion) {
    const PoseVerdict verdict = Case().run();
    EXPECT_TRUE(verdict.accepted());
    EXPECT_STREQ(toString(verdict.rejection), "");
    EXPECT_DOUBLE_EQ(verdict.orientation.w, 1.0);
}

TEST(PoseQuality, NormalizesQuaternion) {
    Case c;
    c.pose.orientation.w = 2.0;
    const PoseVerdict verdict = c.run();
    EXPECT_TRUE(verdict.accepted());
    EXPECT_DOUBLE_EQ(norm(verdict.orientation), 1.0);
    EXPECT_DOUBLE_EQ(verdict.orientation.w, 1.0);
}

TEST(PoseQuality, NormalizesAllComponents) {
    Case c;
    c.pose.orientation = Quaternion{1.0, 2.0, 2.0, 4.0};
    const PoseVerdict verdict = c.run();
    ASSERT_TRUE(verdict.accepted());
    EXPECT_DOUBLE_EQ(verdict.orientation.x, 0.2);
    EXPECT_DOUBLE_EQ(verdict.orientation.y, 0.4);
    EXPECT_DOUBLE_EQ(verdict.orientation.z, 0.4);
    EXPECT_DOUBLE_EQ(verdict.orientation.w, 0.8);
}

TEST(PoseQuality, RejectsNonFinite) {
    Case c;
    c.pose.position.x = kNaN;
    const PoseVerdict verdict = c.run();
    EXPECT_FALSE(verdict.accepted());
    EXPECT_EQ(verdict.rejection, PoseRejection::kNonFinite);
    EXPECT_STREQ(toString(verdict.rejection), "nonfinite");
}

TEST(PoseQuality, RejectsNonFiniteInEveryField) {
    for (const double bad : {kNaN, kInf, -kInf}) {
        for (int field = 0; field < 7; ++field) {
            Case c;
            double* values[] = {&c.pose.position.x,    &c.pose.position.y,    &c.pose.position.z,
                                &c.pose.orientation.x, &c.pose.orientation.y, &c.pose.orientation.z,
                                &c.pose.orientation.w};
            *values[field] = bad;
            EXPECT_EQ(c.run().rejection, PoseRejection::kNonFinite) << "field " << field;
        }
    }
}

TEST(PoseQuality, RejectsDegenerateQuaternion) {
    Case c;
    c.pose.orientation = Quaternion{0.0, 0.0, 0.0, 0.0};
    const PoseVerdict verdict = c.run();
    EXPECT_FALSE(verdict.accepted());
    EXPECT_EQ(verdict.rejection, PoseRejection::kBadQuaternion);
    EXPECT_STREQ(toString(verdict.rejection), "bad_quat");
}

TEST(PoseQuality, QuaternionNormBoundIs1e6) {
    Case c;
    c.pose.orientation = Quaternion{0.0, 0.0, 0.0, 0.9e-6};
    EXPECT_EQ(c.run().rejection, PoseRejection::kBadQuaternion);
    c.pose.orientation = Quaternion{0.0, 0.0, 0.0, 1.1e-6};
    const PoseVerdict verdict = c.run();
    ASSERT_TRUE(verdict.accepted());
    EXPECT_DOUBLE_EQ(verdict.orientation.w, 1.0);
}

TEST(PoseQuality, RejectsAQuaternionWhoseNormOverflows) {
    Case c;
    c.pose.orientation = Quaternion{1.0e200, 0.0, 0.0, 1.0e200};
    EXPECT_EQ(c.run().rejection, PoseRejection::kBadQuaternion);
}

TEST(PoseQuality, RejectsOrigin) {
    Case c;
    c.pose.position.x = 0.0;
    c.pose.position.y = 0.0;
    const PoseVerdict verdict = c.run();
    EXPECT_FALSE(verdict.accepted());
    EXPECT_EQ(verdict.rejection, PoseRejection::kOrigin);
    EXPECT_STREQ(toString(verdict.rejection), "origin");
}

TEST(PoseQuality, OriginIgnoresZ) {
    Case c;
    c.pose.position = Vec3{0.0, 0.0, 1.5};
    EXPECT_EQ(c.run().rejection, PoseRejection::kOrigin);
}

TEST(PoseQuality, NegativeZeroIsTheOrigin) {
    Case c;
    c.pose.position = Vec3{-0.0, 0.0, 0.5};
    EXPECT_EQ(c.run().rejection, PoseRejection::kOrigin);
}

TEST(PoseQuality, OneZeroAxisIsNotTheOrigin) {
    Case c;
    c.pose.position = Vec3{0.0, 0.25, 0.5};
    EXPECT_TRUE(c.run().accepted());
    c.pose.position = Vec3{0.25, 0.0, 0.5};
    EXPECT_TRUE(c.run().accepted());
}

TEST(PoseQuality, AllowsOriginWhenConfigured) {
    Case c;
    c.pose.position.x = 0.0;
    c.pose.position.y = 0.0;
    c.config.reject_origin = false;
    const PoseVerdict verdict = c.run();
    EXPECT_TRUE(verdict.accepted());
    EXPECT_STREQ(toString(verdict.rejection), "");
}

TEST(PoseQuality, RejectsLargeXY) {
    Case c;
    c.pose.position.x = 901.0;
    const PoseVerdict verdict = c.run();
    EXPECT_FALSE(verdict.accepted());
    EXPECT_EQ(verdict.rejection, PoseRejection::kAbsXy);
    EXPECT_STREQ(toString(verdict.rejection), "abs_xy");
}

TEST(PoseQuality, XYBoundAppliesToBothAxesAndBothSigns) {
    for (const Vec3 position :
         {Vec3{-901.0, 2.0, 0.5}, Vec3{1.0, 901.0, 0.5}, Vec3{1.0, -901.0, 0.5}}) {
        Case c;
        c.pose.position = position;
        EXPECT_EQ(c.run().rejection, PoseRejection::kAbsXy);
    }
}

TEST(PoseQuality, XYBoundIsExclusiveAndIgnoresZ) {
    Case c;
    c.pose.position = Vec3{900.0, -900.0, 5000.0};
    EXPECT_TRUE(c.run().accepted());
    c.pose.position = Vec3{900.0001, 2.0, 0.5};
    EXPECT_EQ(c.run().rejection, PoseRejection::kAbsXy);
}

TEST(PoseQuality, XYBoundFollowsTheConfig) {
    Case c;
    c.pose.position = Vec3{901.0, 2.0, 0.5};
    c.config.max_abs_xy = 1000.0;
    EXPECT_TRUE(c.run().accepted());
    c.config.max_abs_xy = 0.0;
    c.pose.position = Vec3{1.0e6, -1.0e6, 0.5};
    EXPECT_TRUE(c.run().accepted());
    c.config.max_abs_xy = -1.0;
    EXPECT_TRUE(c.run().accepted());
}

TEST(PoseQuality, RejectsJump) {
    Case c;
    c.history.last_accepted = Vec3{1.0, 2.0, 0.5};
    c.pose.position = Vec3{4.1, 2.0, 0.5};
    const PoseVerdict verdict = c.run();
    EXPECT_FALSE(verdict.accepted());
    EXPECT_EQ(verdict.rejection, PoseRejection::kJump);
    EXPECT_STREQ(toString(verdict.rejection), "jump");
}

TEST(PoseQuality, JumpLimitIsExclusive) {
    Case c;
    c.history.last_accepted = Vec3{1.0, 2.0, 0.5};
    c.pose.position = Vec3{3.0, 2.0, 0.5};  // exactly 2 m
    EXPECT_TRUE(c.run().accepted());
    c.pose.position = Vec3{3.0001, 2.0, 0.5};
    EXPECT_EQ(c.run().rejection, PoseRejection::kJump);
}

TEST(PoseQuality, JumpMeasuresAllThreeAxes) {
    Case c;
    c.history.last_accepted = Vec3{1.0, 2.0, 0.5};
    c.pose.position = Vec3{1.0, 2.0, 3.0};  // 2.5 m straight up
    EXPECT_EQ(c.run().rejection, PoseRejection::kJump);
    c.pose.position = Vec3{2.2, 3.2, 1.7};  // sqrt(3) * 1.2 = 2.078 m diagonal
    EXPECT_EQ(c.run().rejection, PoseRejection::kJump);
    c.pose.position = Vec3{1.9, 2.9, 1.4};  // sqrt(3) * 0.9 = 1.559 m diagonal
    EXPECT_TRUE(c.run().accepted());
}

TEST(PoseQuality, JumpFollowsTheConfig) {
    Case c;
    c.history.last_accepted = Vec3{1.0, 2.0, 0.5};
    c.pose.position = Vec3{4.1, 2.0, 0.5};
    c.config.max_jump_m = 5.0;
    EXPECT_TRUE(c.run().accepted());
    c.config.max_jump_m = 0.0;
    c.pose.position = Vec3{800.0, 2.0, 0.5};
    EXPECT_TRUE(c.run().accepted());
    c.config.max_jump_m = -2.0;
    EXPECT_TRUE(c.run().accepted());
}

TEST(PoseQuality, NoJumpCheckWithoutAnAcceptedPose) {
    Case c;
    c.pose.position = Vec3{800.0, 700.0, 0.5};
    EXPECT_TRUE(c.run().accepted());
}

TEST(PoseQuality, RejectsFrozen) {
    Case c;
    c.history.last_move = PoseHistory::Movement{Vec3{1.0, 2.0, 0.5}, 0.0};
    c.now_s = 0.5;
    const PoseVerdict verdict = c.run();
    EXPECT_FALSE(verdict.accepted());
    EXPECT_EQ(verdict.rejection, PoseRejection::kFrozen);
    EXPECT_STREQ(toString(verdict.rejection), "frozen");
}

TEST(PoseQuality, AcceptsAMovedPoseAfterTheFrozenWindow) {
    Case c;
    c.pose.position.x = 1.2;
    c.history.last_move = PoseHistory::Movement{Vec3{1.0, 2.0, 0.5}, 0.0};
    c.now_s = 0.5;
    const PoseVerdict verdict = c.run();
    EXPECT_TRUE(verdict.accepted());
    EXPECT_STREQ(toString(verdict.rejection), "");
}

TEST(PoseQuality, FrozenNeedsTheFullWindow) {
    Case c;
    c.history.last_move = PoseHistory::Movement{Vec3{1.0, 2.0, 0.5}, 10.0};
    c.now_s = 10.499;
    EXPECT_TRUE(c.run().accepted());
    c.now_s = 10.5;
    EXPECT_EQ(c.run().rejection, PoseRejection::kFrozen);
    c.now_s = 3600.0;
    EXPECT_EQ(c.run().rejection, PoseRejection::kFrozen);
}

TEST(PoseQuality, FrozenWindowFollowsTheConfig) {
    Case c;
    c.history.last_move = PoseHistory::Movement{Vec3{1.0, 2.0, 0.5}, 0.0};
    c.now_s = 0.6;
    c.config.frozen_s = 1.0;
    EXPECT_TRUE(c.run().accepted());
    c.config.frozen_s = 0.0;
    c.now_s = 3600.0;
    EXPECT_TRUE(c.run().accepted());
    c.config.frozen_s = -1.0;
    EXPECT_TRUE(c.run().accepted());
}

TEST(PoseQuality, MovementThresholdIs1e4MetersIn3D) {
    // 2^-14 = 6.1e-5 m is below the threshold, 2^-13 = 1.22e-4 m above it; both are exact.
    Case c;
    c.history.last_move = PoseHistory::Movement{Vec3{1.0, 2.0, 0.5}, 0.0};
    c.now_s = 5.0;
    c.pose.position = Vec3{1.0 + std::ldexp(1.0, -14), 2.0, 0.5};
    EXPECT_EQ(c.run().rejection, PoseRejection::kFrozen);
    c.pose.position = Vec3{1.0 + std::ldexp(1.0, -13), 2.0, 0.5};
    EXPECT_TRUE(c.run().accepted());
    c.pose.position = Vec3{1.0, 2.0, 0.5 + std::ldexp(1.0, -13)};
    EXPECT_TRUE(c.run().accepted());
}

TEST(PoseQuality, FrozenIsMeasuredFromTheLastMoveNotTheLastFrame) {
    // Twelve frames of 5e-5 m each have drifted 6e-4 m from the last move, so the pose is alive
    // even though no single step reached the threshold.
    Case c;
    c.history.last_move = PoseHistory::Movement{Vec3{1.0, 2.0, 0.5}, 0.0};
    c.history.last_accepted = Vec3{1.0 + 11 * 5.0e-5, 2.0, 0.5};
    c.pose.position = Vec3{1.0 + 12 * 5.0e-5, 2.0, 0.5};
    c.now_s = 5.0;
    EXPECT_TRUE(c.run().accepted());
}

TEST(PoseQuality, ChecksRunInTheDocumentedOrder) {
    // Everything wrong at once: the first check in the list names the rejection.
    Case c;
    c.pose.position = Vec3{0.0, 0.0, 0.5};
    c.pose.orientation = Quaternion{0.0, 0.0, 0.0, 0.0};
    EXPECT_EQ(c.run().rejection, PoseRejection::kBadQuaternion);
    c.pose.orientation = Quaternion{0.0, 0.0, 0.0, 1.0};
    EXPECT_EQ(c.run().rejection, PoseRejection::kOrigin);

    c.pose.position = Vec3{901.0, 2.0, 0.5};
    c.history.last_accepted = Vec3{1.0, 2.0, 0.5};
    c.history.last_move = PoseHistory::Movement{Vec3{901.0, 2.0, 0.5}, 0.0};
    c.now_s = 10.0;
    EXPECT_EQ(c.run().rejection, PoseRejection::kAbsXy);

    c.pose.position = Vec3{5.0, 2.0, 0.5};
    c.history.last_move = PoseHistory::Movement{Vec3{5.0, 2.0, 0.5}, 0.0};
    EXPECT_EQ(c.run().rejection, PoseRejection::kJump);

    c.history.last_accepted = Vec3{5.0, 2.0, 0.5};
    EXPECT_EQ(c.run().rejection, PoseRejection::kFrozen);

    c.pose.position.x = kNaN;
    EXPECT_EQ(c.run().rejection, PoseRejection::kNonFinite);
}

TEST(PoseRejectionName, MatchesTheLegacyReasonStrings) {
    EXPECT_STREQ(toString(PoseRejection::kNone), "");
    EXPECT_STREQ(toString(PoseRejection::kNonFinite), "nonfinite");
    EXPECT_STREQ(toString(PoseRejection::kBadQuaternion), "bad_quat");
    EXPECT_STREQ(toString(PoseRejection::kOrigin), "origin");
    EXPECT_STREQ(toString(PoseRejection::kAbsXy), "abs_xy");
    EXPECT_STREQ(toString(PoseRejection::kJump), "jump");
    EXPECT_STREQ(toString(PoseRejection::kFrozen), "frozen");
}

TEST(PoseHistoryUpdate, FirstAcceptedPoseSetsBothMarks) {
    PoseHistory history;
    history.recordAccepted(Vec3{1.0, 2.0, 0.5}, 3.0);
    ASSERT_TRUE(history.last_accepted.has_value());
    ASSERT_TRUE(history.last_move.has_value());
    EXPECT_DOUBLE_EQ(history.last_accepted->x, 1.0);
    EXPECT_DOUBLE_EQ(history.last_move->position.y, 2.0);
    EXPECT_DOUBLE_EQ(history.last_move->time_s, 3.0);
}

TEST(PoseHistoryUpdate, LastMoveAdvancesOnlyWhenThePoseMoved) {
    PoseHistory history;
    history.recordAccepted(Vec3{1.0, 2.0, 0.5}, 3.0);
    history.recordAccepted(Vec3{1.0 + std::ldexp(1.0, -14), 2.0, 0.5}, 3.5);
    EXPECT_DOUBLE_EQ(history.last_move->time_s, 3.0);
    EXPECT_DOUBLE_EQ(history.last_move->position.x, 1.0);
    EXPECT_DOUBLE_EQ(history.last_accepted->x, 1.0 + std::ldexp(1.0, -14));
    history.recordAccepted(Vec3{1.0 + std::ldexp(1.0, -13), 2.0, 0.5}, 4.0);
    EXPECT_DOUBLE_EQ(history.last_move->time_s, 4.0);
    EXPECT_DOUBLE_EQ(history.last_move->position.x, 1.0 + std::ldexp(1.0, -13));
}

Pose poseAt(double x, double y, double z = 0.5) {
    return Pose{Vec3{x, y, z}, Quaternion{0.0, 0.0, 0.0, 1.0}};
}

TEST(PoseQualityGate, AcceptsTheFirstPoseAndRemembersIt) {
    PoseQualityGate gate;
    EXPECT_FALSE(gate.history().last_accepted.has_value());
    EXPECT_TRUE(gate.check(poseAt(1.0, 2.0), 1.0).accepted());
    ASSERT_TRUE(gate.history().last_accepted.has_value());
    EXPECT_DOUBLE_EQ(gate.history().last_accepted->x, 1.0);
}

TEST(PoseQualityGate, RejectedPosesLeaveTheHistoryAlone) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 1.0).accepted());
    EXPECT_EQ(gate.check(poseAt(10.0, 2.0), 1.1).rejection, PoseRejection::kJump);
    EXPECT_EQ(gate.check(poseAt(0.0, 0.0), 1.2).rejection, PoseRejection::kOrigin);
    EXPECT_DOUBLE_EQ(gate.history().last_accepted->x, 1.0);
    EXPECT_DOUBLE_EQ(gate.history().last_move->time_s, 1.0);
}

TEST(PoseQualityGate, AJumpIsRejectedUntilThePoseReturnsNearTheLastAcceptedPose) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 1.0).accepted());
    // Ten m away for 0.04 s, far below jump_reacquire_s: every frame there is a jump from the last
    // accepted pose.
    for (int frame = 0; frame < 5; ++frame) {
        EXPECT_EQ(gate.check(poseAt(11.0 + 0.01 * frame, 2.0), 1.1 + 0.01 * frame).rejection,
                  PoseRejection::kJump);
    }
    // Back within 2 m of the last accepted pose.
    EXPECT_TRUE(gate.check(poseAt(2.5, 2.0), 1.2).accepted());
    EXPECT_EQ(gate.check(poseAt(11.0, 2.0), 1.3).rejection, PoseRejection::kJump);
}

TEST(PoseQualityGate, FrozenPoseIsRejectedUntilItMoves) {
    PoseQualityGate gate;
    EXPECT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_TRUE(gate.check(poseAt(1.0, 2.0), 0.2).accepted());
    EXPECT_TRUE(gate.check(poseAt(1.0, 2.0), 0.49).accepted());
    EXPECT_EQ(gate.check(poseAt(1.0, 2.0), 0.5).rejection, PoseRejection::kFrozen);
    EXPECT_EQ(gate.check(poseAt(1.0, 2.0), 0.6).rejection, PoseRejection::kFrozen);
    EXPECT_EQ(gate.check(poseAt(1.0, 2.0), 30.0).rejection, PoseRejection::kFrozen);
    // It moves: accepted again, and the frozen window restarts from this frame.
    EXPECT_TRUE(gate.check(poseAt(1.01, 2.0), 30.1).accepted());
    EXPECT_TRUE(gate.check(poseAt(1.01, 2.0), 30.4).accepted());
    EXPECT_EQ(gate.check(poseAt(1.01, 2.0), 30.7).rejection, PoseRejection::kFrozen);
}

TEST(PoseQualityGate, SteadyMotionIsNeverFrozen) {
    PoseQualityGate gate;
    for (int frame = 0; frame < 1000; ++frame) {
        const double t = 0.01 * frame;
        ASSERT_TRUE(gate.check(poseAt(1.0 + 0.001 * frame, 2.0), t).accepted()) << frame;
    }
}

TEST(PoseQualityGate, UsesTheConfig) {
    PoseQualityConfig config;
    config.reject_origin = false;
    config.max_abs_xy = 10.0;
    config.max_jump_m = 0.5;
    config.frozen_s = 0.0;
    PoseQualityGate gate(config);
    EXPECT_TRUE(gate.check(poseAt(0.0, 0.0), 0.0).accepted());
    EXPECT_EQ(gate.check(poseAt(0.0, 11.0), 0.1).rejection, PoseRejection::kAbsXy);
    EXPECT_EQ(gate.check(poseAt(0.6, 0.0), 0.2).rejection, PoseRejection::kJump);
    EXPECT_TRUE(gate.check(poseAt(0.0, 0.0), 1000.0).accepted());
}

TEST(PoseQualityGate, ReturnsTheNormalizedQuaternion) {
    PoseQualityGate gate;
    Pose pose = poseAt(1.0, 2.0);
    pose.orientation = Quaternion{0.0, 0.0, 3.0, 4.0};
    const PoseVerdict verdict = gate.check(pose, 1.0);
    ASSERT_TRUE(verdict.accepted());
    EXPECT_DOUBLE_EQ(verdict.orientation.z, 0.6);
    EXPECT_DOUBLE_EQ(verdict.orientation.w, 0.8);
}

// A frame every 1/16 s: sums of these are exact in binary, so the boundaries below are exact.
constexpr double kFrame = 0.0625;

// Feeds `count` frames of a body that stays at (x + frame * 1/16 m, 2), starting at start_s.
// Returns the number of frames the gate accepted.
int feedFarFrames(PoseQualityGate* gate, double x, double start_s, int count) {
    int accepted = 0;
    for (int frame = 0; frame < count; ++frame) {
        if (gate->check(poseAt(x + frame * kFrame, 2.0), start_s + frame * kFrame).accepted()) {
            ++accepted;
        }
    }
    return accepted;
}

TEST(JumpReacquire, IsOnByDefaultForOneSecond) {
    EXPECT_DOUBLE_EQ(PoseQualityConfig().jump_reacquire_s, 1.0);
}

TEST(JumpReacquire, ATrackThatStaysIsAcceptedAfterTheConfiguredTime) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    // The body shows up 10 m away at t = 5 s and stays: 16 frames fill the first second.
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 16), 0);
    ASSERT_TRUE(gate.history().jump_run.has_value());
    EXPECT_DOUBLE_EQ(gate.history().jump_run->start_s, 5.0);
    EXPECT_DOUBLE_EQ(gate.history().last_accepted->x, 1.0);

    // The frame one second after the first one is the new start of the track.
    const PoseVerdict verdict = gate.check(poseAt(12.0, 2.0), 6.0);
    EXPECT_TRUE(verdict.accepted());
    EXPECT_TRUE(verdict.reacquired);
    EXPECT_EQ(verdict.rejection, PoseRejection::kNone);
    EXPECT_DOUBLE_EQ(verdict.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(gate.history().last_accepted->x, 12.0);
    EXPECT_FALSE(gate.history().jump_run.has_value());

    // From there it is an ordinary track: near frames pass, the old place is the far one now.
    const PoseVerdict next = gate.check(poseAt(12.0 + kFrame, 2.0), 6.0 + kFrame);
    EXPECT_TRUE(next.accepted());
    EXPECT_FALSE(next.reacquired);
    EXPECT_EQ(gate.check(poseAt(1.0, 2.0), 6.125).rejection, PoseRejection::kJump);
}

TEST(JumpReacquire, TheFrameJustBeforeTheConfiguredTimeIsStillRejected) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 15), 0);  // 5.0 .. 5.875
    // 0.9375 s and 0.96875 s after the first frame of the run.
    EXPECT_EQ(gate.check(poseAt(12.0, 2.0), 5.9375).rejection, PoseRejection::kJump);
    EXPECT_EQ(gate.check(poseAt(12.0, 2.0), 5.96875).rejection, PoseRejection::kJump);
    EXPECT_TRUE(gate.check(poseAt(12.0, 2.0), 6.0).reacquired);
}

TEST(JumpReacquire, TheConfiguredTimeIsTheConfiguredTime) {
    PoseQualityConfig config;
    config.jump_reacquire_s = 0.25;
    PoseQualityGate gate(config);
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 4), 0);
    EXPECT_TRUE(gate.check(poseAt(11.25, 2.0), 5.25).reacquired);
}

TEST(JumpReacquire, AGlitchShorterThanTheTimeIsStillRejected) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    // 15 frames, 0.9375 s, at a far place; then the body is back where it was.
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 15), 0);
    const PoseVerdict back = gate.check(poseAt(1.0 + kFrame, 2.0), 5.9375);
    EXPECT_TRUE(back.accepted());
    EXPECT_FALSE(back.reacquired);
    EXPECT_FALSE(gate.history().jump_run.has_value());
    // The next glitch starts from zero: it needs its own full second.
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 6.0, 15), 0);
    EXPECT_DOUBLE_EQ(gate.history().last_accepted->x, 1.0 + kFrame);
}

TEST(JumpReacquire, AGhostThatWandersBetweenPlacesIsNeverAccepted) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    int accepted = 0;
    for (int frame = 0; frame < 160; ++frame) {           // ten seconds
        const double x = (frame % 2 == 0) ? 11.0 : 30.0;  // 19 m apart
        accepted += gate.check(poseAt(x, 2.0), 5.0 + frame * kFrame).accepted() ? 1 : 0;
    }
    EXPECT_EQ(accepted, 0);
    EXPECT_DOUBLE_EQ(gate.history().last_accepted->x, 1.0);
}

TEST(JumpReacquire, ADriftingTrackIsFollowedWhileEachStepStaysWithinTheJumpLimit) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    // The body moves on at 1.9 m per frame: far from the anchor, close to the last frame.
    int accepted = 0;
    for (int frame = 0; frame < 20; ++frame) {
        accepted +=
            gate.check(poseAt(11.0 + 1.9 * frame, 2.0), 5.0 + frame * kFrame).accepted() ? 1 : 0;
    }
    EXPECT_EQ(accepted, 4);  // frames 16..19 of the 20 are past the first second
}

TEST(JumpReacquire, FramesFartherApartThanTheJumpLimitDoNotFormARun) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(gate.check(poseAt(11.0, 2.0), 5.0).rejection, PoseRejection::kJump);
    // 2.0 m from the previous frame continues the run, 2.5 m starts a new one.
    EXPECT_EQ(gate.check(poseAt(13.0, 2.0), 5.5).rejection, PoseRejection::kJump);
    EXPECT_DOUBLE_EQ(gate.history().jump_run->start_s, 5.0);
    EXPECT_EQ(gate.check(poseAt(15.5, 2.0), 5.75).rejection, PoseRejection::kJump);
    EXPECT_DOUBLE_EQ(gate.history().jump_run->start_s, 5.75);
    EXPECT_EQ(gate.check(poseAt(15.5, 2.0), 6.5).rejection, PoseRejection::kJump);
    EXPECT_TRUE(gate.check(poseAt(15.5, 2.0), 6.75).reacquired);
}

TEST(JumpReacquire, AFrameThatFailsAnotherCheckEndsTheRun) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 12), 0);  // 0.6875 s
    ASSERT_TRUE(gate.history().jump_run.has_value());
    EXPECT_EQ(gate.check(poseAt(std::numeric_limits<double>::quiet_NaN(), 2.0), 5.75).rejection,
              PoseRejection::kNonFinite);
    EXPECT_FALSE(gate.history().jump_run.has_value());
    // The second that follows has to be complete on its own.
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.8125, 16), 0);
    EXPECT_TRUE(gate.check(poseAt(12.0, 2.0), 6.8125).reacquired);
}

TEST(JumpReacquire, AnOriginFrameEndsTheRunToo) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 12), 0);
    EXPECT_EQ(gate.check(poseAt(0.0, 0.0), 5.75).rejection, PoseRejection::kOrigin);
    EXPECT_FALSE(gate.history().jump_run.has_value());
}

TEST(JumpReacquire, AFrameThatPassesTheJumpCheckEndsTheRun) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 12), 0);
    EXPECT_TRUE(gate.check(poseAt(1.5, 2.0), 5.75).accepted());
    EXPECT_FALSE(gate.history().jump_run.has_value());
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.8125, 15), 0);
}

TEST(JumpReacquire, ALongGapBetweenTwoFarFramesStartsTheRunAgain) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(gate.check(poseAt(11.0, 2.0), 5.0).rejection, PoseRejection::kJump);
    // The tracker went silent for 4 s: two sightings that far apart in time are not a track.
    EXPECT_EQ(gate.check(poseAt(11.5, 2.0), 9.0).rejection, PoseRejection::kJump);
    EXPECT_DOUBLE_EQ(gate.history().jump_run->start_s, 9.0);
    EXPECT_EQ(gate.check(poseAt(11.5, 2.0), 9.5).rejection, PoseRejection::kJump);
    EXPECT_TRUE(gate.check(poseAt(11.5, 2.0), 10.0).reacquired);
}

TEST(JumpReacquire, AClockRewindStartsTheRunAgain) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(gate.check(poseAt(11.0, 2.0), 50.0).rejection, PoseRejection::kJump);
    EXPECT_EQ(gate.check(poseAt(11.0, 2.0), 40.0).rejection, PoseRejection::kJump);
    EXPECT_DOUBLE_EQ(gate.history().jump_run->start_s, 40.0);
}

TEST(JumpReacquire, TurnedOffAJumpStaysRejected) {
    for (const double off : {0.0, -1.0}) {
        PoseQualityConfig config;
        config.jump_reacquire_s = off;
        PoseQualityGate gate(config);
        ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
        EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 160), 0);  // ten seconds
        EXPECT_FALSE(gate.history().jump_run.has_value());
        EXPECT_DOUBLE_EQ(gate.history().last_accepted->x, 1.0);
        EXPECT_TRUE(gate.check(poseAt(1.5, 2.0), 20.0).accepted());
    }
}

TEST(JumpReacquire, DoesNotApplyWithoutAJumpCheck) {
    PoseQualityConfig config;
    config.max_jump_m = 0.0;
    PoseQualityGate gate(config);
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    const PoseVerdict far = gate.check(poseAt(500.0, 2.0), 0.5);
    EXPECT_TRUE(far.accepted());
    EXPECT_FALSE(far.reacquired);
    EXPECT_FALSE(gate.history().jump_run.has_value());
}

TEST(JumpReacquire, RestartsFrozenDetectionAtTheNewTrack) {
    PoseQualityGate gate;
    ASSERT_TRUE(gate.check(poseAt(1.0, 2.0), 0.0).accepted());
    EXPECT_EQ(feedFarFrames(&gate, 11.0, 5.0, 16), 0);
    ASSERT_TRUE(gate.check(poseAt(12.0, 2.0), 6.0).reacquired);
    EXPECT_DOUBLE_EQ(gate.history().last_move->time_s, 6.0);
    EXPECT_DOUBLE_EQ(gate.history().last_move->position.x, 12.0);
    // Standing still at the new place is frozen after frozen_s from the re-acquisition.
    EXPECT_TRUE(gate.check(poseAt(12.0, 2.0), 6.25).accepted());
    EXPECT_TRUE(gate.check(poseAt(12.0, 2.0), 6.4375).accepted());
    EXPECT_EQ(gate.check(poseAt(12.0, 2.0), 6.5).rejection, PoseRejection::kFrozen);
}

TEST(JumpReacquire, EvaluatePoseDecidesFromTheHistoryAndLeavesItAlone) {
    PoseQualityConfig config;
    PoseHistory history;
    history.last_accepted = Vec3{1.0, 2.0, 0.5};
    history.last_move = PoseHistory::Movement{Vec3{1.0, 2.0, 0.5}, 0.0};
    history.jump_run = PoseHistory::JumpRun{Vec3{11.0, 2.0, 0.5}, 5.0, 5.9375};

    PoseVerdict early = evaluatePose(poseAt(11.0625, 2.0), history, 5.96875, config);
    EXPECT_EQ(early.rejection, PoseRejection::kJump);
    EXPECT_FALSE(early.reacquired);

    PoseVerdict stayed = evaluatePose(poseAt(11.0625, 2.0), history, 6.0, config);
    EXPECT_TRUE(stayed.accepted());
    EXPECT_TRUE(stayed.reacquired);
    EXPECT_DOUBLE_EQ(stayed.orientation.w, 1.0);

    // Not within the jump limit of the run's latest frame: no.
    EXPECT_EQ(evaluatePose(poseAt(20.0, 2.0), history, 6.0, config).rejection,
              PoseRejection::kJump);
    // The history is only read.
    EXPECT_DOUBLE_EQ(history.last_accepted->x, 1.0);
    EXPECT_DOUBLE_EQ(history.jump_run->start_s, 5.0);
    EXPECT_DOUBLE_EQ(history.jump_run->last_s, 5.9375);

    // Without a run in the history, even the far frame of a long track is only a jump.
    history.jump_run.reset();
    EXPECT_EQ(evaluatePose(poseAt(11.0, 2.0), history, 6.0, config).rejection,
              PoseRejection::kJump);
}

TEST(JumpReacquire, HistoryBookkeeping) {
    PoseQualityConfig config;
    PoseHistory history;
    history.recordAccepted(Vec3{1.0, 2.0, 0.5}, 0.0);
    history.recordJumpRejected(Vec3{11.0, 2.0, 0.5}, 5.0, config);
    ASSERT_TRUE(history.jump_run.has_value());
    EXPECT_DOUBLE_EQ(history.jump_run->start_s, 5.0);
    history.recordJumpRejected(Vec3{11.5, 2.0, 0.5}, 5.5, config);
    EXPECT_DOUBLE_EQ(history.jump_run->start_s, 5.0);
    EXPECT_DOUBLE_EQ(history.jump_run->last_s, 5.5);
    EXPECT_DOUBLE_EQ(history.jump_run->last_position.x, 11.5);
    history.recordAccepted(Vec3{1.0, 2.0, 0.5}, 6.0);
    EXPECT_FALSE(history.jump_run.has_value());

    history.recordJumpRejected(Vec3{11.0, 2.0, 0.5}, 7.0, config);
    history.recordReacquired(Vec3{11.0, 2.0, 0.5}, 8.0);
    EXPECT_FALSE(history.jump_run.has_value());
    EXPECT_DOUBLE_EQ(history.last_accepted->x, 11.0);
    EXPECT_DOUBLE_EQ(history.last_move->time_s, 8.0);
}

}  // namespace
}  // namespace xgc2_vrpn_relay
