#pragma once

#include <cstdint>

#include "desk_protocol.hpp"

namespace smart_desk {

enum class ControlState {
    kIdle,
    kDialClassify,
    kManualStep,
    kManualHold,
    kPresetHold,
    kSettling,
};

enum class ControlRequest {
    kNone,
    kGoSit,
    kGoStand,
};

enum class TransitionReason {
    kNone,
    kDialStarted,
    kClassifiedStep,
    kClassifiedHold,
    kStopRequested,
    kButtonDuringMotion,
    kDialReversal,
    kDialOverride,
    kDialReleased,
    kTelemetryNotReady,
    kTelemetryStale,
    kUnexpectedControllerFrame,
    kTelemetryReversed,
    kNoInitialMovement,
    kHeightStalled,
    kStepComplete,
    kStepWatchdog,
    kBoundaryReached,
    kManualWatchdog,
    kPresetWatchdog,
    kSettled,
    kSettleWatchdog,
    kSoftLimit,
    kAlreadyAtTarget,
    kInvalidDirection,
};

struct ControllerConfig {
    std::uint16_t physical_min_tenths_cm = 700;
    std::uint16_t physical_max_tenths_cm = 1200;
    std::uint16_t sit_tenths_cm = 800;
    std::uint16_t stand_tenths_cm = 1150;
    std::uint16_t up_coast_tenths_cm = 25;
    std::uint16_t down_coast_tenths_cm = 20;
    std::uint16_t telemetry_reversal_tenths_cm = 10;
    std::uint32_t hold_escalation_reports = 4;
    std::uint32_t classify_ms = 100;
    std::uint32_t manual_release_ms = 350;
    std::uint32_t step_completion_min_ms = 500;
    // A normal autonomous STEP is already expected to react within 2.5 s.
    // This outer deadline only closes a fault path where movement was seen but
    // the settled predicate never becomes true.
    std::uint32_t step_watchdog_ms = 5000;
    std::uint32_t motion_start_grace_ms = 2500;
    std::uint32_t motion_stall_ms = 2000;
    std::uint32_t preset_max_run_ms = 20000;
    std::uint32_t manual_max_run_ms = 30000;
    std::uint32_t settle_min_ms = 1500;
    std::uint32_t settle_no_motion_ms = 3000;
    std::uint32_t settle_watchdog_ms = 7000;
    std::uint32_t rearm_quiet_ms = 500;
};

struct ControllerInput {
    std::uint32_t now_ms = 0;
    std::uint16_t height_tenths_cm = 0;
    bool telemetry_fresh = false;
    bool telemetry_settled = false;
    std::uint32_t anomaly_sequence = 0;

    // Monotonic modulo-2^32 counter. Multiple reports may be coalesced into
    // one snapshot; dial_direction describes the latest report in it.
    std::uint32_t dial_sequence = 0;
    int dial_direction = 0;
    std::uint32_t last_dial_ms = 0;

    bool stop_requested = false;
    ControlRequest request = ControlRequest::kNone;
};

struct ControllerOutput {
    Command command = Command::kIdle;
    ControlState state = ControlState::kIdle;
    TransitionReason reason = TransitionReason::kNone;
};

class DeskController {
public:
    explicit DeskController(ControllerConfig config = {});

    ControllerOutput update(const ControllerInput& input);

    ControlState state() const { return state_; }
    int direction() const { return direction_; }
    std::uint32_t consumed_dial_sequence() const {
        return consumed_dial_sequence_;
    }
    const ControllerConfig& config() const { return config_; }
    bool set_presets(std::uint16_t sit_tenths_cm,
                     std::uint16_t stand_tenths_cm);

private:
    bool valid_height(const ControllerInput& input) const;
    bool valid_presets(std::uint16_t sit_tenths_cm,
                       std::uint16_t stand_tenths_cm) const;
    bool elapsed(std::uint32_t now, std::uint32_t since,
                 std::uint32_t duration) const;
    std::uint32_t consume_dial_reports(const ControllerInput& input);
    void begin_motion(const ControllerInput& input, int direction,
                      std::uint16_t target_height);
    void begin_settling(const ControllerInput& input);
    void become_idle();
    ControllerOutput output(Command command, TransitionReason reason) const;

    ControllerConfig config_;
    ControlState state_ = ControlState::kIdle;
    std::uint32_t consumed_dial_sequence_ = 0;
    std::uint32_t gesture_event_count_ = 0;
    int direction_ = 0;
    std::uint16_t start_height_ = 0;
    std::uint16_t target_height_ = 0;
    std::uint16_t progress_height_ = 0;
    std::uint32_t motion_start_ms_ = 0;
    std::uint32_t progress_ms_ = 0;
    std::uint32_t settle_start_ms_ = 0;
    std::uint32_t start_anomaly_sequence_ = 0;
    bool have_progress_ = false;
    bool moved_ = false;
    bool allow_hold_ = true;
};

const char* control_state_name(ControlState state);
const char* transition_reason_name(TransitionReason reason);

}  // namespace smart_desk
