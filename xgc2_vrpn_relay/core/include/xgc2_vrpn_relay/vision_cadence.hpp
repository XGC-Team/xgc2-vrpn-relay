#pragma once

#include <cstddef>
#include <deque>

namespace xgc2_vrpn_relay {

// Rate limit for the PX4 vision pose, decided per incoming frame with no timer and no held
// value: a stale pose is never repeated after the tracker stops.
//
// A frame is emitted when it is the first one, when at least 1 / target_rate_hz passed since the
// last emission, when the clock went backwards, or when the last `window` emissions averaged below
// the target rate. Faster input is therefore thinned to about the target rate, and slower input
// passes unchanged.
class VisionPublishCadence {
public:
    // A target rate of 0 or less switches limiting off. Throws std::invalid_argument for a
    // non-finite rate or a zero window.
    explicit VisionPublishCadence(double target_rate_hz = 30.0, std::size_t window = 5u);

    // Decides whether the frame received at now_seconds is emitted, and counts it when it is. A
    // non-finite time is never emitted. Not thread-safe.
    bool take(double now_seconds);

private:
    bool shouldEmit(double now_seconds) const;

    double target_rate_hz_;
    std::size_t window_;
    std::deque<double> published_times_;
};

}  // namespace xgc2_vrpn_relay
