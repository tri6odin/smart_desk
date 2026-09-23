#include "desk_inputs.hpp"

namespace smart_desk {

TelemetryTracker::TelemetryTracker(ControllerConfig config) : config_(config) {}

void TelemetryTracker::receive(const Frame& frame, std::uint32_t now_ms) {
    const auto message = decode_controller_frame(frame);
    if (!message || message->flags != 0 ||
        (message->status != 0x01 && message->status != 0x02) ||
        message->height_tenths_cm < config_.physical_min_tenths_cm ||
        message->height_tenths_cm > config_.physical_max_tenths_cm) {
        available_ = false;
        state_.stable_frames = 0;
        ++state_.anomaly_sequence;
        return;
    }

    const bool continuous =
        available_ && static_cast<std::uint32_t>(now_ms - last_ms_) <= kMaxAgeMs;
    if (continuous && message->height_tenths_cm == state_.height_tenths_cm) {
        if (state_.stable_frames < config_.full_settle_frames) {
            ++state_.stable_frames;
        }
    } else {
        state_.stable_frames = 1;
    }
    state_.height_tenths_cm = message->height_tenths_cm;
    last_ms_ = now_ms;
    available_ = true;
}

TelemetrySnapshot TelemetryTracker::snapshot(std::uint32_t now_ms) const {
    auto result = state_;
    result.fresh =
        available_ && static_cast<std::uint32_t>(now_ms - last_ms_) <= kMaxAgeMs;
    if (!result.fresh) {
        result.stable_frames = 0;
    }
    return result;
}

void InputMailbox::dial(int direction, std::uint32_t now_ms) {
    if (direction != 1 && direction != -1) {
        stop();
        return;
    }
    ++pending_.dial_sequence;
    pending_.dial_direction = direction;
    pending_.last_dial_ms = now_ms;
    if (active_direction_ != 0 && direction != active_direction_) {
        stop();
    }
}

void InputMailbox::button(ButtonRequest request) {
    if (request == ButtonRequest::kNone) {
        return;
    }
    if (active_direction_ != 0) {
        stop();
    } else if (!pending_.stop_requested) {
        pending_.button = request;
    }
}

void InputMailbox::stop() {
    pending_.stop_requested = true;
    pending_.button = ButtonRequest::kNone;
}

InputSnapshot InputMailbox::consume() {
    const auto snapshot = pending_;
    pending_.button = ButtonRequest::kNone;
    pending_.stop_requested = false;
    return snapshot;
}

}  // namespace smart_desk
