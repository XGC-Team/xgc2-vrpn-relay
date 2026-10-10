#pragma once

#include <optional>

#include "xgc2_vrpn_relay/types.hpp"

namespace xgc2_vrpn_relay {

// A pose that moved by less than this since the last time it moved counts as not moving.
constexpr double kMovedThresholdM = 1.0e-4;

enum class PoseRejection {
    kNone,
    kNonFinite,
    kBadQuaternion,
    kOrigin,
    kAbsXy,
    kJump,
    kFrozen,
};

// Short reason names used in logs: "", "nonfinite", "bad_quat", "origin", "abs_xy", "jump",
// "frozen".
const char* toString(PoseRejection rejection);

// Thresholds of the pose quality gate. A numeric bound of 0 or less switches that check off.
struct PoseQualityConfig {
    // Reject (x, y) == (0, 0), which is what trackers report when they lose a body.
    bool reject_origin = true;
    // Reject |x| or |y| above this [m].
    double max_abs_xy = 900.0;
    // Reject a move from the last accepted pose above this [m].
    double max_jump_m = 2.0;
    // Reject a pose that has not moved for at least this long [s].
    double frozen_s = 0.5;
    // A track that jumped more than max_jump_m and stays there is accepted as the new track
    // after this long [s]: the frames that fail only the jump check must keep coming, each
    // within max_jump_m of the one before. 0 or less never accepts such a track, which then
    // stays rejected until the pose comes back.
    double jump_reacquire_s = 1.0;
};

// What the gate remembers about the poses it accepted. All positions are the raw tracker
// coordinates, before any offset.
struct PoseHistory {
    struct Movement {
        Vec3 position;  // where the pose last moved by at least kMovedThresholdM
        double time_s;  // when it was received there
    };

    // Frames in a row that failed only the jump check, each within max_jump_m of the one
    // before and no more than jump_reacquire_s after it. Any other frame ends the run.
    struct JumpRun {
        Vec3 last_position;  // the latest frame of the run
        double start_s;      // when its first frame was received
        double last_s;       // when its latest frame was received
    };

    std::optional<Vec3> last_accepted;
    std::optional<Movement> last_move;
    std::optional<JumpRun> jump_run;

    // An accepted pose becomes the last accepted pose, and the last move when it moved.
    void recordAccepted(const Vec3& position, double now_s);
    // A pose accepted as a track that jumped and stayed: it restarts the history there, with the
    // last move at this very frame.
    void recordReacquired(const Vec3& position, double now_s);
    // A pose rejected by the jump check extends the jump run, or starts a new one.
    void recordJumpRejected(const Vec3& position, double now_s, const PoseQualityConfig& config);
};

struct PoseVerdict {
    PoseRejection rejection = PoseRejection::kNone;
    // The pose is accepted because the track jumped and stayed for jump_reacquire_s.
    bool reacquired = false;
    Quaternion orientation;  // unit quaternion, meaningful only when the pose was accepted

    bool accepted() const { return rejection == PoseRejection::kNone; }
};

// Checks one raw pose against the history. The checks run in the order of PoseRejection: finite
// values, usable quaternion, origin, |x|/|y| bound, jump from the last accepted pose (3D
// distance), frozen (not moved by kMovedThresholdM since last_move.time_s, which is at least
// frozen_s ago). A pose that fails the jump check is still accepted, as reacquired, when the
// history's jump run continues with it and has lasted jump_reacquire_s; it skips the frozen
// check, which restarts at that pose. now_s is the receipt time of the pose in seconds. The
// function does not change the history.
PoseVerdict evaluatePose(const Pose& pose, const PoseHistory& history, double now_s,
                         const PoseQualityConfig& config = PoseQualityConfig());

// evaluatePose plus the history update: an accepted pose becomes the last accepted pose, and
// the last move when it moved. A rejected pose never becomes the reference; it only builds up
// the jump run that can, after jump_reacquire_s, move the reference to where the track stays.
class PoseQualityGate {
public:
    explicit PoseQualityGate(const PoseQualityConfig& config = PoseQualityConfig());

    PoseVerdict check(const Pose& pose, double now_s);

    const PoseHistory& history() const { return history_; }

private:
    PoseQualityConfig config_;
    PoseHistory history_;
};

}  // namespace xgc2_vrpn_relay
