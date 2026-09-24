#include "desk_recovery.hpp"

namespace smart_desk {

DeskRecoveryController::DeskRecoveryController(RecoveryConfig config)
    : config_(config) {}

bool DeskRecoveryController::elapsed(std::uint32_t now, std::uint32_t since,
                                     std::uint32_t duration) const {
    return static_cast<std::uint32_t>(now - since) >= duration;
}

RecoveryOutput DeskRecoveryController::output(
    Command command, RecoveryReason reason, bool reset_motion_controller,
    bool force_override) const {
    return RecoveryOutput{command, state_, reason,
                          force_override || state_ != RecoveryState::kReady,
                          reset_motion_controller, false};
}

RecoveryOutput DeskRecoveryController::update(const RecoveryInput& input) {
    RecoveryReason reason = RecoveryReason::kNone;

    const bool unknown_frame =
        input.unknown_frame_sequence != seen_unknown_sequence_;
    const bool reset_frame =
        input.reset_frame_sequence != seen_reset_sequence_;
    seen_unknown_sequence_ = input.unknown_frame_sequence;
    seen_reset_sequence_ = input.reset_frame_sequence;
    if (unknown_frame) {
        fault_latched_ = true;
    }

    if (input.calibration_requested) {
        suppress_cancel_action_ = false;
        state_ = RecoveryState::kCalibrating;
        stage_started_ms_ = input.now_ms;
        activity_baseline_ = input.user_activity_sequence;
        reason = RecoveryReason::kCalibrationStarted;
        return output(Command::kDownHold, reason, true);
    }

    if ((state_ == RecoveryState::kCalibrating ||
         state_ == RecoveryState::kAwaitingTelemetry) &&
        input.user_activity_sequence != activity_baseline_) {
        cancelled_activity_sequence_ = input.user_activity_sequence;
        suppress_cancel_action_ = !fault_latched_;
        state_ = fault_latched_ ? RecoveryState::kLocked
                                : RecoveryState::kReady;
        return output(Command::kIdle, RecoveryReason::kCalibrationCancelled,
                      true, true);
    }

    if (state_ == RecoveryState::kCalibrating) {
        if (reset_frame) {
            state_ = RecoveryState::kAwaitingTelemetry;
            stage_started_ms_ = input.now_ms;
            height_baseline_ = input.height_frame_sequence;
            return output(Command::kIdle, RecoveryReason::kResetAcknowledged);
        }
        // Error/status frames are expected while recovering a controller
        // fault. They keep the fault latched but must not interrupt DOWN;
        // only RST, user cancellation or timeout ends this stage.
        if (elapsed(input.now_ms, stage_started_ms_,
                    config_.calibration_timeout_ms)) {
            fault_latched_ = true;
            state_ = RecoveryState::kLocked;
            return output(Command::kIdle, RecoveryReason::kCalibrationTimeout,
                          true);
        }
        return output(Command::kDownHold, reason);
    }

    if (state_ == RecoveryState::kAwaitingTelemetry) {
        if (unknown_frame) {
            state_ = RecoveryState::kLocked;
            return output(Command::kIdle,
                          RecoveryReason::kUnknownControllerFrame, true);
        }
        const bool new_height =
            input.height_frame_sequence != height_baseline_;
        if (new_height && input.telemetry_fresh &&
            input.telemetry_stable_frames >=
                config_.completion_stable_frames) {
            fault_latched_ = false;
            state_ = RecoveryState::kReady;
            return output(Command::kIdle,
                          RecoveryReason::kCalibrationComplete, true, true);
        }
        if (elapsed(input.now_ms, stage_started_ms_,
                    config_.telemetry_timeout_ms)) {
            fault_latched_ = true;
            state_ = RecoveryState::kLocked;
            return output(Command::kIdle, RecoveryReason::kTelemetryTimeout,
                          true);
        }
        return output(Command::kIdle, reason);
    }

    if (state_ == RecoveryState::kLocked) {
        if (reset_frame) {
            suppress_cancel_action_ = false;
            state_ = RecoveryState::kAwaitingTelemetry;
            stage_started_ms_ = input.now_ms;
            activity_baseline_ = input.user_activity_sequence;
            height_baseline_ = input.height_frame_sequence;
            return output(Command::kIdle, RecoveryReason::kResetAcknowledged);
        }
        return output(Command::kIdle, reason);
    }

    if (unknown_frame) {
        state_ = RecoveryState::kLocked;
        return output(Command::kIdle,
                      RecoveryReason::kUnknownControllerFrame, true);
    }
    auto result = output(Command::kIdle, reason);
    if (suppress_cancel_action_) {
        if (input.user_activity_sequence == cancelled_activity_sequence_) {
            result.suppress_normal_input = true;
        } else {
            suppress_cancel_action_ = false;
        }
    }
    return result;
}

const char* recovery_state_name(RecoveryState state) {
    switch (state) {
        case RecoveryState::kReady: return "READY";
        case RecoveryState::kLocked: return "LOCKED";
        case RecoveryState::kCalibrating: return "CALIBRATING";
        case RecoveryState::kAwaitingTelemetry: return "AWAITING_TELEMETRY";
    }
    return "UNKNOWN";
}

const char* recovery_reason_name(RecoveryReason reason) {
    switch (reason) {
        case RecoveryReason::kNone: return "none";
        case RecoveryReason::kUnknownControllerFrame:
            return "unknown controller frame";
        case RecoveryReason::kCalibrationStarted: return "calibration started";
        case RecoveryReason::kCalibrationCancelled:
            return "calibration cancelled by user input";
        case RecoveryReason::kResetAcknowledged: return "RST acknowledged";
        case RecoveryReason::kCalibrationComplete:
            return "calibration complete";
        case RecoveryReason::kCalibrationTimeout: return "RST timeout";
        case RecoveryReason::kTelemetryTimeout:
            return "post-RST telemetry timeout";
    }
    return "unknown";
}

}  // namespace smart_desk
