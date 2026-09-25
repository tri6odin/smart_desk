#include "desk_inputs.hpp"

namespace smart_desk {

bool LongPressDetector::press(std::uint32_t now) {
    if (pressed_) {
        return false;
    }
    pressed_at_ = now;
    pressed_ = true;
    fired_ = false;
    return true;
}

bool LongPressDetector::poll(std::uint32_t now) {
    if (!pressed_ || fired_ ||
        static_cast<std::uint32_t>(now - pressed_at_) < threshold_) {
        return false;
    }
    fired_ = true;
    return true;
}

bool LongPressDetector::release(std::uint32_t now) {
    const bool fire = poll(now);
    pressed_ = false;
    fired_ = false;
    return fire;
}

void LongPressDetector::reset() {
    pressed_at_ = 0;
    pressed_ = false;
    fired_ = false;
}

PresetPressResult PresetGestureDetector::press(PresetButton button,
                                               std::uint32_t now) {
    bool* pressed = button == PresetButton::kSit ? &sit_pressed_
                                                 : &stand_pressed_;
    std::uint32_t* pressed_at = button == PresetButton::kSit
                                    ? &sit_pressed_at_
                                    : &stand_pressed_at_;
    PresetPressResult result;
    if (!*pressed) {
        *pressed = true;
        *pressed_at = now;
        result.new_press = true;
    }
    if (sit_pressed_ && stand_pressed_ && !cancelled_) {
        chord_seen_ = true;
        result.chord_started = chord_.press(now);
    }
    return result;
}

bool PresetGestureDetector::poll(std::uint32_t now) {
    return chord_.poll(now);
}

PresetReleaseResult PresetGestureDetector::release(std::uint32_t now) {
    PresetReleaseResult result;
    result.sit_pressed = sit_pressed_;
    result.stand_pressed = stand_pressed_;
    result.sit_duration = now - sit_pressed_at_;
    result.stand_duration = now - stand_pressed_at_;
    result.suppress_presets = chord_seen_;
    result.start_calibration = chord_.release(now);
    sit_pressed_at_ = 0;
    stand_pressed_at_ = 0;
    sit_pressed_ = false;
    stand_pressed_ = false;
    chord_seen_ = false;
    cancelled_ = false;
    return result;
}

bool PresetGestureDetector::cancel() {
    if (!sit_pressed_ && !stand_pressed_) {
        return false;
    }
    chord_.reset();
    chord_seen_ = true;
    cancelled_ = true;
    return true;
}

void PresetGestureDetector::reset() {
    chord_.reset();
    sit_pressed_at_ = 0;
    stand_pressed_at_ = 0;
    sit_pressed_ = false;
    stand_pressed_ = false;
    chord_seen_ = false;
    cancelled_ = false;
}

TelemetryTracker::TelemetryTracker(ControllerConfig config) : config_(config) {}

void TelemetryTracker::receive(const Frame& frame, std::uint32_t now_ms) {
    const auto kind = classify_controller_frame(frame);
    if (kind != ControllerFrameKind::kHeight) {
        available_ = false;
        state_.stable_frames = 0;
        ++state_.anomaly_sequence;
        if (kind == ControllerFrameKind::kReset) {
            ++state_.reset_frame_sequence;
        } else if (kind == ControllerFrameKind::kUnknown) {
            ++state_.unknown_frame_sequence;
        }
        return;
    }

    ++state_.height_frame_sequence;
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
    // The retained direction in SETTLING describes the previous motion, not
    // whether the desk is still moving. Let the FSM check current telemetry.
    if (state_ != ControlState::kIdle && state_ != ControlState::kSettling) {
        stop();
    } else if (!pending_.stop_requested) {
        pending_.button = request;
    }
}

void InputMailbox::stop() {
    pending_.stop_requested = true;
    pending_.button = ButtonRequest::kNone;
}

void InputMailbox::request_calibration() {
    pending_.calibration_requested = true;
}

void InputMailbox::user_activity() {
    ++pending_.user_activity_sequence;
}

InputSnapshot InputMailbox::consume() {
    const auto snapshot = pending_;
    pending_.button = ButtonRequest::kNone;
    pending_.stop_requested = false;
    pending_.calibration_requested = false;
    return snapshot;
}

}  // namespace smart_desk
