#include "desk_controller.hpp"

#include <utility>

namespace smart_desk {
namespace {

constexpr std::uint16_t kMinimumPresetGapTenthsCm = 100;

}  // namespace

DeskController::DeskController(ControllerConfig config) : config_(std::move(config)) {}

bool DeskController::valid_presets(std::uint16_t sit_tenths_cm,
                                   std::uint16_t stand_tenths_cm) const {
    return sit_tenths_cm >= config_.soft_min_tenths_cm &&
           stand_tenths_cm <= config_.soft_max_tenths_cm &&
           static_cast<std::uint32_t>(sit_tenths_cm) + kMinimumPresetGapTenthsCm <=
               stand_tenths_cm;
}

bool DeskController::set_presets(std::uint16_t sit_tenths_cm,
                                 std::uint16_t stand_tenths_cm) {
    if (!valid_presets(sit_tenths_cm, stand_tenths_cm)) {
        return false;
    }
    config_.sit_target_tenths_cm = sit_tenths_cm;
    config_.stand_target_tenths_cm = stand_tenths_cm;
    return true;
}

bool DeskController::valid_height(const ControllerInput& input) const {
    return input.telemetry_fresh &&
           input.height_tenths_cm >= config_.physical_min_tenths_cm &&
           input.height_tenths_cm <= config_.physical_max_tenths_cm;
}

bool DeskController::elapsed(std::uint32_t now, std::uint32_t since,
                             std::uint32_t duration) const {
    return static_cast<std::uint32_t>(now - since) >= duration;
}

std::uint32_t DeskController::consume_dial_reports(const ControllerInput& input) {
    const std::uint32_t count = input.dial_sequence - consumed_dial_sequence_;
    consumed_dial_sequence_ = input.dial_sequence;
    return count;
}

bool DeskController::full_step_inside_soft_limits(const ControllerInput& input,
                                                  int direction) const {
    return direction > 0
               ? static_cast<std::uint32_t>(input.height_tenths_cm) + 10U <=
                     config_.soft_max_tenths_cm
               : static_cast<std::uint32_t>(input.height_tenths_cm) >=
                     static_cast<std::uint32_t>(config_.soft_min_tenths_cm) + 10U;
}

void DeskController::begin_dial_classification(const ControllerInput& input,
                                               int direction, std::uint32_t reports,
                                               std::uint32_t required_stable_frames) {
    begin_motion(
        input, direction,
        direction > 0 ? config_.soft_max_tenths_cm : config_.soft_min_tenths_cm);
    gesture_event_count_ = reports;
    required_start_stable_frames_ = required_stable_frames;
    allow_hold_ = !at_braking_point(input.height_tenths_cm);
    state_ = ControlState::kDialClassify;
}

bool DeskController::at_braking_point(std::uint16_t height) const {
    const int brake_height =
        direction_ > 0
            ? static_cast<int>(target_height_) - config_.up_coast_tenths_cm
            : static_cast<int>(target_height_) + config_.down_coast_tenths_cm;
    return direction_ > 0 ? height >= brake_height : height <= brake_height;
}

bool DeskController::hold_classification_ready() const {
    return allow_hold_ && gesture_event_count_ >= config_.hold_escalation_reports;
}

ControllerOutput DeskController::begin_manual_hold(const ControllerInput& input) {
    begin_motion(input, direction_, target_height_);
    state_ = ControlState::kManualHold;
    const Command command = direction_ > 0 ? Command::kUpHold : Command::kDownHold;
    return output(command, TransitionReason::kClassifiedHold);
}

void DeskController::begin_motion(const ControllerInput& input, int direction,
                                  std::uint16_t target_height) {
    direction_ = direction;
    start_height_ = input.height_tenths_cm;
    target_height_ = target_height;
    progress_height_ = input.height_tenths_cm;
    motion_start_ms_ = input.now_ms;
    progress_ms_ = input.now_ms;
    settle_start_ms_ = input.now_ms;
    start_anomaly_sequence_ = input.anomaly_sequence;
    have_progress_ = false;
}

void DeskController::begin_settling(const ControllerInput& input) {
    state_ = ControlState::kSettling;
    settle_start_ms_ = input.now_ms;
    settle_last_dial_ms_ = input.last_dial_ms;
}

void DeskController::become_idle() {
    state_ = ControlState::kIdle;
    gesture_event_count_ = 0;
    direction_ = 0;
    have_progress_ = false;
}

ControllerOutput DeskController::output(Command command,
                                        TransitionReason reason) const {
    return ControllerOutput{command, state_, reason};
}

ControllerOutput DeskController::update(const ControllerInput& input) {
    // Inputs are a replacement snapshot, never a queue. Consume dial reports
    // even when a higher-priority button rejects or replaces the gesture.
    const std::uint32_t pending_reports = consume_dial_reports(input);
    if (state_ != ControlState::kIdle && input.request != ControlRequest::kNone) {
        if (state_ == ControlState::kSettling) {
            // Re-evaluate readiness before treating a new button as STOP.
            // Otherwise the first press on the very tick that confirms the
            // completed stop gets lost. Rejected requests are never queued.
            const bool stopped = !input.stop_requested && valid_height(input) &&
                input.telemetry_settled &&
                input.telemetry_stable_frames >= config_.full_settle_frames &&
                elapsed(input.now_ms, settle_start_ms_, config_.rearm_quiet_ms) &&
                elapsed(input.now_ms, input.last_dial_ms, config_.rearm_quiet_ms);
            if (stopped) {
                become_idle();
                return update_idle(input, 0);
            }
            // A rejected button must not extend the settling watchdog.
        } else {
            begin_settling(input);
        }
        return output(Command::kIdle, TransitionReason::kButtonDuringMotion);
    }
    switch (state_) {
        case ControlState::kIdle: return update_idle(input, pending_reports);
        case ControlState::kDialClassify:
            return update_classify(input, pending_reports);
        case ControlState::kManualStep: return update_step(input, pending_reports);
        case ControlState::kManualHold:
        case ControlState::kPresetHold: return update_hold(input, pending_reports);
        case ControlState::kSettling: return update_settling(input, pending_reports);
    }
    return output(Command::kIdle, TransitionReason::kNone);
}

ControllerOutput DeskController::update_idle(const ControllerInput& input,
                                             std::uint32_t pending_reports) {
    Command command = Command::kIdle;
    TransitionReason reason = TransitionReason::kNone;
    if (input.stop_requested) {
        return output(command, TransitionReason::kStopRequested);
    }

    if (input.request != ControlRequest::kNone) {
        const std::uint16_t target = input.request == ControlRequest::kGoSit
                                         ? config_.sit_target_tenths_cm
                                         : config_.stand_target_tenths_cm;
        if (!valid_height(input) || !input.telemetry_settled) {
            return output(command, TransitionReason::kTelemetryNotReady);
        }
        const std::uint16_t distance = input.height_tenths_cm >= target
                                           ? input.height_tenths_cm - target
                                           : target - input.height_tenths_cm;
        // Preserve the proven pre-refactor behavior: a preset farther than
        // one centimetre away must emit a command. Coast compensation decides
        // when an active HOLD stops; it must not suppress the initial command.
        if (distance <= 10) {
            return output(command, TransitionReason::kAlreadyAtTarget);
        }
        begin_motion(input, target > input.height_tenths_cm ? 1 : -1, target);
        state_ = ControlState::kPresetHold;
        command = direction_ > 0 ? Command::kUpHold : Command::kDownHold;
        return output(command, TransitionReason::kClassifiedHold);
    }

    if (pending_reports == 0 ||
        elapsed(input.now_ms, input.last_dial_ms, config_.manual_release_ms)) {
        return output(command, reason);
    }

    const std::uint32_t reports = pending_reports;
    if (input.dial_direction != 1 && input.dial_direction != -1) {
        return output(command, TransitionReason::kInvalidDirection);
    }
    if (!valid_height(input) || !input.telemetry_settled) {
        return output(command, TransitionReason::kTelemetryNotReady);
    }
    if (!full_step_inside_soft_limits(input, input.dial_direction)) {
        return output(command, TransitionReason::kSoftLimit);
    }

    begin_dial_classification(input, input.dial_direction, reports,
                              config_.full_settle_frames);
    if (hold_classification_ready()) {
        return begin_manual_hold(input);
    }
    return output(command, TransitionReason::kDialStarted);
}

ControllerOutput DeskController::update_classify(const ControllerInput& input,
                                                 std::uint32_t pending_reports) {
    Command command = Command::kIdle;
    if (pending_reports != 0) {
        const std::uint32_t reports = pending_reports;
        if (input.dial_direction != direction_) {
            become_idle();
            return output(command, TransitionReason::kDialReversal);
        }
        gesture_event_count_ += reports;
    }
    if (input.stop_requested) {
        become_idle();
        return output(command, TransitionReason::kStopRequested);
    }
    if (!valid_height(input) ||
        input.telemetry_stable_frames < required_start_stable_frames_) {
        become_idle();
        return output(command, TransitionReason::kTelemetryNotReady);
    }
    if (input.anomaly_sequence != start_anomaly_sequence_) {
        become_idle();
        return output(command, TransitionReason::kUnexpectedControllerFrame);
    }
    // A slow detent deliberately waits classify_ms from the first writer
    // snapshot, which may itself arrive one UART period after the report.
    // Reject a stalled classifier without expiring that normal STEP delay.
    if (elapsed(input.now_ms, motion_start_ms_,
                config_.classify_ms + config_.manual_release_ms)) {
        become_idle();
        return output(command, TransitionReason::kDialReleased);
    }
    if (!full_step_inside_soft_limits(input, direction_)) {
        become_idle();
        return output(command, TransitionReason::kSoftLimit);
    }
    allow_hold_ = !at_braking_point(input.height_tenths_cm);
    if (hold_classification_ready()) {
        if (elapsed(input.now_ms, input.last_dial_ms, config_.manual_release_ms)) {
            become_idle();
            return output(command, TransitionReason::kDialReleased);
        }
        // Continued rotation is no longer ambiguous.  Start HOLD
        // immediately instead of waiting out the STEP deadline.
        return begin_manual_hold(input);
    }
    if (!elapsed(input.now_ms, motion_start_ms_, config_.classify_ms)) {
        return output(command, TransitionReason::kNone);
    }

    begin_motion(input, direction_, target_height_);
    state_ = ControlState::kManualStep;
    command = direction_ > 0 ? Command::kUpStep : Command::kDownStep;
    return output(command, TransitionReason::kClassifiedStep);
}

void DeskController::update_progress(const ControllerInput& input) {
    if (!have_progress_ && input.height_tenths_cm != start_height_) {
        have_progress_ = true;
        progress_ms_ = input.now_ms;
    }
    const bool progressed = direction_ > 0 ? input.height_tenths_cm > progress_height_
                                           : input.height_tenths_cm < progress_height_;
    if (progressed) {
        progress_height_ = input.height_tenths_cm;
        progress_ms_ = input.now_ms;
        have_progress_ = true;
    }
}

ControllerOutput DeskController::update_step(const ControllerInput& input,
                                             std::uint32_t pending_reports) {
    Command command = Command::kIdle;
    TransitionReason reason = TransitionReason::kNone;
    bool dial_reversal = false;
    if (pending_reports != 0) {
        dial_reversal = input.dial_direction != direction_;
    }

    update_progress(input);

    if (input.stop_requested) {
        begin_settling(input);
        reason = TransitionReason::kStopRequested;
    } else if (dial_reversal) {
        begin_settling(input);
        reason = TransitionReason::kDialReversal;
    } else if (!valid_height(input)) {
        begin_settling(input);
        reason = TransitionReason::kTelemetryStale;
    } else if (input.anomaly_sequence != start_anomaly_sequence_) {
        begin_settling(input);
        reason = TransitionReason::kUnexpectedControllerFrame;
    } else if (!have_progress_ &&
               elapsed(input.now_ms, motion_start_ms_, config_.motion_start_grace_ms)) {
        begin_settling(input);
        reason = TransitionReason::kNoInitialMovement;
    } else if (elapsed(input.now_ms, motion_start_ms_, config_.step_watchdog_ms)) {
        begin_settling(input);
        reason = TransitionReason::kStepWatchdog;
    } else if (have_progress_ && input.telemetry_settled &&
               elapsed(input.now_ms, motion_start_ms_,
                       config_.step_completion_min_ms)) {
        become_idle();
        reason = TransitionReason::kStepComplete;
    }
    return output(command, reason);
}

ControllerOutput DeskController::update_hold(const ControllerInput& input,
                                             std::uint32_t pending_reports) {
    Command command = Command::kIdle;
    TransitionReason reason = TransitionReason::kNone;
    const bool manual = state_ == ControlState::kManualHold;
    bool dial_override = false;
    bool dial_reversal = false;
    if (pending_reports != 0) {
        dial_override = !manual;
        dial_reversal = manual && input.dial_direction != direction_;
    }

    update_progress(input);

    const bool telemetry_reversed =
        direction_ > 0
            ? static_cast<std::uint32_t>(input.height_tenths_cm) +
                      config_.telemetry_reversal_tenths_cm <=
                  progress_height_
            : input.height_tenths_cm >= static_cast<std::uint32_t>(progress_height_) +
                                            config_.telemetry_reversal_tenths_cm;
    const bool boundary_reached = at_braking_point(progress_height_);

    if (input.stop_requested) {
        reason = TransitionReason::kStopRequested;
    } else if (dial_reversal) {
        reason = TransitionReason::kDialReversal;
    } else if (dial_override) {
        reason = TransitionReason::kDialOverride;
    } else if (!valid_height(input)) {
        reason = TransitionReason::kTelemetryStale;
    } else if (input.anomaly_sequence != start_anomaly_sequence_) {
        reason = TransitionReason::kUnexpectedControllerFrame;
    } else if (telemetry_reversed) {
        reason = TransitionReason::kTelemetryReversed;
    } else if (boundary_reached) {
        reason = TransitionReason::kBoundaryReached;
    } else if (manual &&
               elapsed(input.now_ms, input.last_dial_ms, config_.manual_release_ms)) {
        reason = TransitionReason::kDialReleased;
    } else if (manual &&
               elapsed(input.now_ms, motion_start_ms_, config_.manual_max_run_ms)) {
        reason = TransitionReason::kManualWatchdog;
    } else if (!manual &&
               elapsed(input.now_ms, motion_start_ms_, config_.preset_max_run_ms)) {
        reason = TransitionReason::kPresetWatchdog;
    } else if (have_progress_ &&
               elapsed(input.now_ms, progress_ms_, config_.motion_stall_ms)) {
        reason = TransitionReason::kHeightStalled;
    } else if (!have_progress_ &&
               elapsed(input.now_ms, motion_start_ms_, config_.motion_start_grace_ms)) {
        reason = TransitionReason::kNoInitialMovement;
    }

    if (reason == TransitionReason::kNone) {
        command = direction_ > 0 ? Command::kUpHold : Command::kDownHold;
    } else {
        begin_settling(input);
    }
    return output(command, reason);
}

ControllerOutput DeskController::update_settling(const ControllerInput& input,
                                                 std::uint32_t pending_reports) {
    Command command = Command::kIdle;
    TransitionReason reason = TransitionReason::kNone;
    if (pending_reports != 0) {
        const bool valid_direction =
            input.dial_direction == 1 || input.dial_direction == -1;
        const bool same_direction =
            valid_direction && input.dial_direction == direction_;
        const std::uint32_t required_frames = same_direction
                                                  ? config_.same_direction_rearm_frames
                                                  : config_.full_settle_frames;
        const bool quiet_before_new_gesture =
            elapsed(input.last_dial_ms, settle_last_dial_ms_, config_.rearm_quiet_ms);
        const bool can_rearm =
            !input.stop_requested && input.request == ControlRequest::kNone &&
            valid_direction && valid_height(input) &&
            !elapsed(input.now_ms, input.last_dial_ms, config_.manual_release_ms) &&
            input.telemetry_stable_frames >= required_frames &&
            quiet_before_new_gesture &&
            full_step_inside_soft_limits(input, input.dial_direction);
        const std::uint32_t reports = pending_reports;
        if (can_rearm) {
            begin_dial_classification(input, input.dial_direction, reports,
                                      required_frames);
            if (hold_classification_ready()) {
                return begin_manual_hold(input);
            }
            return output(command, TransitionReason::kDialStarted);
        }
        settle_last_dial_ms_ = input.last_dial_ms;
    }
    const bool input_quiet =
        elapsed(input.now_ms, settle_last_dial_ms_, config_.rearm_quiet_ms);
    const bool finished =
        input.telemetry_fresh && input.telemetry_settled && input_quiet;
    const bool watchdog =
        elapsed(input.now_ms, settle_start_ms_, config_.settle_watchdog_ms);
    if (finished || watchdog) {
        become_idle();
        reason =
            watchdog ? TransitionReason::kSettleWatchdog : TransitionReason::kSettled;
    }
    return output(command, reason);
}

const char* control_state_name(ControlState state) {
    switch (state) {
        case ControlState::kIdle: return "IDLE";
        case ControlState::kDialClassify: return "DIAL_CLASSIFY";
        case ControlState::kManualStep: return "MANUAL_STEP";
        case ControlState::kManualHold: return "MANUAL_HOLD";
        case ControlState::kPresetHold: return "PRESET_HOLD";
        case ControlState::kSettling: return "SETTLING";
    }
    return "UNKNOWN";
}

const char* transition_reason_name(TransitionReason reason) {
    switch (reason) {
        case TransitionReason::kNone: return "none";
        case TransitionReason::kDialStarted: return "dial started";
        case TransitionReason::kClassifiedStep: return "classified STEP";
        case TransitionReason::kClassifiedHold: return "classified HOLD";
        case TransitionReason::kStopRequested: return "STOP requested";
        case TransitionReason::kButtonDuringMotion: return "button during motion";
        case TransitionReason::kDialReversal: return "dial reversal";
        case TransitionReason::kDialOverride: return "dial override";
        case TransitionReason::kDialReleased: return "dial released";
        case TransitionReason::kTelemetryNotReady: return "telemetry not ready";
        case TransitionReason::kTelemetryStale: return "telemetry stale";
        case TransitionReason::kUnexpectedControllerFrame:
            return "unexpected controller frame";
        case TransitionReason::kTelemetryReversed: return "telemetry reversed";
        case TransitionReason::kNoInitialMovement: return "no initial movement";
        case TransitionReason::kHeightStalled: return "height stalled";
        case TransitionReason::kStepComplete: return "STEP complete";
        case TransitionReason::kStepWatchdog: return "STEP watchdog";
        case TransitionReason::kBoundaryReached: return "boundary reached";
        case TransitionReason::kManualWatchdog: return "manual watchdog";
        case TransitionReason::kPresetWatchdog: return "preset watchdog";
        case TransitionReason::kSettled: return "settled";
        case TransitionReason::kSettleWatchdog: return "settle watchdog";
        case TransitionReason::kSoftLimit: return "soft limit";
        case TransitionReason::kAlreadyAtTarget: return "already at target";
        case TransitionReason::kInvalidDirection: return "invalid direction";
    }
    return "unknown";
}

}  // namespace smart_desk
