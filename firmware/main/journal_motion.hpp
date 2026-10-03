#pragma once

#include "desk_controller.hpp"
#include "desk_journal.hpp"
#include "desk_recovery.hpp"

// Translate control decisions to stable journal metadata without changing
// commands or timing. Only annotate decisions which stop actual motion.
namespace journal_motion {
using smart_desk::journal::Context;
using smart_desk::journal::StopReason;

constexpr Context context(smart_desk::ControlState state, Context preset) {
    using State = smart_desk::ControlState;
    if (state == State::kManualStep || state == State::kManualHold ||
        state == State::kDialClassify) return Context::kDial;
    return state == State::kPresetHold ? preset : Context::kNone;
}

constexpr StopReason reason(smart_desk::TransitionReason value) {
    using Reason = smart_desk::TransitionReason;
    switch (value) {
        case Reason::kStopRequested: return StopReason::kStopRequested;
        case Reason::kButtonDuringMotion: return StopReason::kButtonDuringMotion;
        case Reason::kDialReversal: return StopReason::kDialReversal;
        case Reason::kDialOverride: return StopReason::kDialOverride;
        case Reason::kDialReleased: return StopReason::kDialReleased;
        case Reason::kTelemetryStale: return StopReason::kTelemetryStale;
        case Reason::kUnexpectedControllerFrame: return StopReason::kUnexpectedFrame;
        case Reason::kTelemetryReversed: return StopReason::kTelemetryReversed;
        case Reason::kNoInitialMovement: return StopReason::kNoInitialMovement;
        case Reason::kHeightStalled: return StopReason::kHeightStalled;
        case Reason::kStepWatchdog: return StopReason::kStepWatchdog;
        case Reason::kBoundaryReached: return StopReason::kTargetBrake;
        case Reason::kManualWatchdog: return StopReason::kManualWatchdog;
        case Reason::kPresetWatchdog: return StopReason::kPresetWatchdog;
        default: return StopReason::kNone;
    }
}

constexpr StopReason reason(smart_desk::RecoveryReason value) {
    using Reason = smart_desk::RecoveryReason;
    switch (value) {
        case Reason::kUnknownControllerFrame: return StopReason::kUnexpectedFrame;
        case Reason::kCalibrationCancelled: return StopReason::kCalibrationCancelled;
        case Reason::kResetAcknowledged: return StopReason::kResetAcknowledged;
        case Reason::kCalibrationTimeout: return StopReason::kCalibrationTimeout;
        default: return StopReason::kNone;
    }
}
constexpr StopReason motion_stop(smart_desk::ControlState previous,
                                smart_desk::TransitionReason value) {
    using State = smart_desk::ControlState;
    if (previous != State::kManualStep && previous != State::kManualHold &&
        previous != State::kPresetHold) return StopReason::kNone;
    return reason(value);
}
}  // namespace journal_motion
