#include "xgc2_vrpn_relay/quality.hpp"

#include <cmath>

namespace xgc2_vrpn_relay {
namespace {

// Whether a frame belongs to the jump run: near the latest frame of the run in space (the
// frames of a track that stays are, a ghost that wanders is not) and not long after it in time.
bool continuesRun(const PoseHistory::JumpRun& run, const Vec3& position, double now_s,
                  const PoseQualityConfig& config) {
    return now_s >= run.last_s && now_s - run.last_s <= config.jump_reacquire_s &&
           distance(position, run.last_position) <= config.max_jump_m;
}

}  // namespace

const char* toString(PoseRejection rejection) {
    switch (rejection) {
    case PoseRejection::kNone:
        return "";
    case PoseRejection::kNonFinite:
        return "nonfinite";
    case PoseRejection::kBadQuaternion:
        return "bad_quat";
    case PoseRejection::kOrigin:
        return "origin";
    case PoseRejection::kAbsXy:
        return "abs_xy";
    case PoseRejection::kJump:
        return "jump";
    case PoseRejection::kFrozen:
        return "frozen";
    }
    return "";
}

void PoseHistory::recordAccepted(const Vec3& position, double now_s) {
    if (!last_move || distance(position, last_move->position) >= kMovedThresholdM) {
        last_move = Movement{position, now_s};
    }
    last_accepted = position;
    jump_run.reset();
}

void PoseHistory::recordReacquired(const Vec3& position, double now_s) {
    last_move = Movement{position, now_s};
    last_accepted = position;
    jump_run.reset();
}

void PoseHistory::recordJumpRejected(const Vec3& position, double now_s,
                                     const PoseQualityConfig& config) {
    if (config.jump_reacquire_s <= 0.0) {
        jump_run.reset();
    } else if (jump_run && continuesRun(*jump_run, position, now_s, config)) {
        jump_run->last_position = position;
        jump_run->last_s = now_s;
    } else {
        jump_run = JumpRun{position, now_s, now_s};
    }
}

PoseVerdict evaluatePose(const Pose& pose, const PoseHistory& history, double now_s,
                         const PoseQualityConfig& config) {
    PoseVerdict verdict;
    if (!isFinite(pose)) {
        verdict.rejection = PoseRejection::kNonFinite;
        return verdict;
    }
    const std::optional<Quaternion> unit = normalized(pose.orientation);
    if (!unit) {
        verdict.rejection = PoseRejection::kBadQuaternion;
        return verdict;
    }
    const Vec3& position = pose.position;
    if (config.reject_origin && position.x == 0.0 && position.y == 0.0) {
        verdict.rejection = PoseRejection::kOrigin;
        return verdict;
    }
    if (config.max_abs_xy > 0.0 &&
        (std::abs(position.x) > config.max_abs_xy || std::abs(position.y) > config.max_abs_xy)) {
        verdict.rejection = PoseRejection::kAbsXy;
        return verdict;
    }
    if (history.last_accepted && config.max_jump_m > 0.0 &&
        distance(position, *history.last_accepted) > config.max_jump_m) {
        const bool stayed = config.jump_reacquire_s > 0.0 && history.jump_run &&
                            continuesRun(*history.jump_run, position, now_s, config) &&
                            now_s - history.jump_run->start_s >= config.jump_reacquire_s;
        if (!stayed) {
            verdict.rejection = PoseRejection::kJump;
            return verdict;
        }
        verdict.reacquired = true;
        verdict.orientation = *unit;
        return verdict;
    }
    if (history.last_move && config.frozen_s > 0.0 &&
        distance(position, history.last_move->position) < kMovedThresholdM &&
        now_s - history.last_move->time_s >= config.frozen_s) {
        verdict.rejection = PoseRejection::kFrozen;
        return verdict;
    }
    verdict.orientation = *unit;
    return verdict;
}

PoseQualityGate::PoseQualityGate(const PoseQualityConfig& config) : config_(config) {}

PoseVerdict PoseQualityGate::check(const Pose& pose, double now_s) {
    const PoseVerdict verdict = evaluatePose(pose, history_, now_s, config_);
    if (verdict.reacquired) {
        history_.recordReacquired(pose.position, now_s);
    } else if (verdict.accepted()) {
        history_.recordAccepted(pose.position, now_s);
    } else if (verdict.rejection == PoseRejection::kJump) {
        history_.recordJumpRejected(pose.position, now_s, config_);
    } else {
        history_.jump_run.reset();
    }
    return verdict;
}

}  // namespace xgc2_vrpn_relay
