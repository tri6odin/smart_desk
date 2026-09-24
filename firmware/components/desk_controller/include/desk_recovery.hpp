#pragma once

#include <cstdint>

#include "desk_protocol.hpp"

namespace smart_desk {

enum class RecoveryState {
    kReady,
    kLocked,
    kCalibrating,
    kAwaitingTelemetry,
};

enum class RecoveryReason {
    kNone,
    kUnknownControllerFrame,
    kCalibrationStarted,
    kCalibrationCancelled,
    kResetAcknowledged,
    kCalibrationComplete,
    kCalibrationTimeout,
    kTelemetryTimeout,
};

struct RecoveryConfig {
    std::uint32_t calibration_timeout_ms = 60000;
    std::uint32_t telemetry_timeout_ms = 10000;
    std::uint32_t completion_stable_frames = 3;
};

struct RecoveryInput {
    std::uint32_t now_ms = 0;
    std::uint32_t height_frame_sequence = 0;
    std::uint32_t reset_frame_sequence = 0;
    std::uint32_t unknown_frame_sequence = 0;
    std::uint32_t user_activity_sequence = 0;
    std::uint32_t telemetry_stable_frames = 0;
    bool telemetry_fresh = false;
    bool calibration_requested = false;
};

struct RecoveryOutput {
    Command command = Command::kIdle;
    RecoveryState state = RecoveryState::kReady;
    RecoveryReason reason = RecoveryReason::kNone;
    bool override_motion = false;
    bool reset_motion_controller = false;
    bool suppress_normal_input = false;
};

// Controls the exceptional recovery path independently of normal desk motion.
// Reported height is deliberately not consulted while driving DOWN: a desk
// needing calibration cannot be trusted to report its position correctly.
class DeskRecoveryController {
   public:
    explicit DeskRecoveryController(RecoveryConfig config = {},
                                    bool initially_locked = false);

    RecoveryOutput update(const RecoveryInput& input);
    RecoveryState state() const { return state_; }
    bool locked() const { return state_ != RecoveryState::kReady; }

   private:
    bool elapsed(std::uint32_t now, std::uint32_t since,
                 std::uint32_t duration) const;
    RecoveryOutput output(Command command, RecoveryReason reason,
                          bool reset_motion_controller = false,
                          bool force_override = false) const;

    RecoveryConfig config_;
    RecoveryState state_ = RecoveryState::kReady;
    bool fault_latched_ = false;
    std::uint32_t stage_started_ms_ = 0;
    std::uint32_t activity_baseline_ = 0;
    std::uint32_t reset_baseline_ = 0;
    std::uint32_t height_baseline_ = 0;
    std::uint32_t seen_unknown_sequence_ = 0;
    std::uint32_t cancelled_activity_sequence_ = 0;
    bool suppress_cancel_action_ = false;
};

const char* recovery_state_name(RecoveryState state);
const char* recovery_reason_name(RecoveryReason reason);

}  // namespace smart_desk
