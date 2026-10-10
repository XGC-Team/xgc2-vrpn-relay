#include "xgc2_vrpn_relay/vision_cadence.hpp"

#include <cmath>
#include <stdexcept>

namespace xgc2_vrpn_relay {

VisionPublishCadence::VisionPublishCadence(double target_rate_hz, std::size_t window)
    : target_rate_hz_(target_rate_hz), window_(window) {
    if (!std::isfinite(target_rate_hz) || window == 0u) {
        throw std::invalid_argument("vision publish cadence is invalid");
    }
}

bool VisionPublishCadence::take(double now_seconds) {
    if (!std::isfinite(now_seconds) || !shouldEmit(now_seconds)) {
        return false;
    }
    published_times_.push_back(now_seconds);
    if (published_times_.size() > window_) {
        published_times_.pop_front();
    }
    return true;
}

bool VisionPublishCadence::shouldEmit(double now_seconds) const {
    if (target_rate_hz_ <= 0.0 || published_times_.empty()) {
        return true;
    }
    const double since_last = now_seconds - published_times_.back();
    if (since_last < 0.0 || since_last >= 1.0 / target_rate_hz_) {
        return true;
    }
    const double span = now_seconds - published_times_.front();
    return span > 0.0 && static_cast<double>(published_times_.size()) / span < target_rate_hz_;
}

}  // namespace xgc2_vrpn_relay
