#include "desk_inputs.hpp"
#include "desk_recovery.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            std::cerr << __FILE__ << ':' << __LINE__ << ": " #condition "\n"; \
            ++failures;                                                       \
        }                                                                     \
    } while (false)

using namespace smart_desk;

Frame height_frame(std::uint16_t height, std::uint8_t status = 1) {
    Frame frame{0x55,
                0xAA,
                8,
                2,
                static_cast<std::uint8_t>(height >> 8),
                static_cast<std::uint8_t>(height),
                status,
                0};
    frame[7] = calculate_checksum(frame);
    return frame;
}

Frame service_frame(std::uint8_t type, std::uint8_t byte4,
                    std::uint8_t byte5, std::uint8_t status) {
    Frame frame{0x55, 0xAA, 8, type, byte4, byte5, status, 0};
    frame[7] = calculate_checksum(frame);
    return frame;
}

void test_telemetry_requires_contiguous_samples() {
    TelemetryTracker tracker;
    CHECK(!tracker.snapshot(0).fresh);
    for (std::uint32_t i = 0; i < 8; ++i) {
        tracker.receive(height_frame(900), i * 100);
    }
    CHECK(tracker.snapshot(700).stable_frames == 5);
    CHECK(tracker.snapshot(1000).fresh);
    CHECK(!tracker.snapshot(1001).fresh);
    CHECK(tracker.snapshot(1001).stable_frames == 0);
    tracker.receive(height_frame(900), 1100);
    CHECK(tracker.snapshot(1100).stable_frames == 1);
    tracker.receive(height_frame(901), 1200);
    CHECK(tracker.snapshot(1200).stable_frames == 1);
}

void test_anomaly_invalidates_height_until_new_samples() {
    for (auto bad : {height_frame(900, 3), height_frame(699), height_frame(1201),
                     height_frame(0x4000 | 900), Frame{0x55, 0xAA, 8, 1, 0, 0, 1, 8}}) {
        TelemetryTracker tracker;
        for (std::uint32_t i = 0; i < 5; ++i) {
            tracker.receive(height_frame(900), i * 100);
        }
        tracker.receive(bad, 450);
        CHECK(!tracker.snapshot(450).fresh);
        CHECK(tracker.snapshot(450).stable_frames == 0);
        CHECK(tracker.snapshot(450).anomaly_sequence == 1);
        tracker.receive(height_frame(900, 2), 500);
        CHECK(tracker.snapshot(500).fresh);
        CHECK(tracker.snapshot(500).stable_frames == 1);
        CHECK(tracker.snapshot(500).anomaly_sequence == 1);
    }
}

void test_telemetry_across_clock_wrap() {
    TelemetryTracker tracker;
    const auto last = std::numeric_limits<std::uint32_t>::max() - 99;
    tracker.receive(height_frame(900), last);
    CHECK(tracker.snapshot(0).fresh);
    tracker.receive(height_frame(900), 0);
    CHECK(tracker.snapshot(0).stable_frames == 2);
    CHECK(!tracker.snapshot(301).fresh);
}

void test_long_press_detector_fires_once_and_handles_wrap() {
    LongPressDetector detector(5000);
    CHECK(detector.press(100));
    CHECK(!detector.press(200));
    CHECK(!detector.poll(5099));
    CHECK(detector.poll(5100));
    CHECK(!detector.poll(6000));
    CHECK(!detector.release(7000));

    detector.press(8000);
    CHECK(!detector.release(9000));

    detector.press(UINT32_MAX - 999);
    CHECK(!detector.poll(3999));
    CHECK(detector.poll(4000));
    detector.reset();
    CHECK(!detector.poll(10000));
}

void test_tracker_separates_known_service_and_unknown_frames() {
    TelemetryTracker tracker;
    tracker.receive(service_frame(0x0F, 0x00, 0x0F, 0x01), 0);
    auto snapshot = tracker.snapshot(0);
    CHECK(snapshot.unknown_frame_sequence == 0);
    CHECK(snapshot.reset_frame_sequence == 0);

    tracker.receive(service_frame(0x04, 0x00, 0x01, 0x01), 100);
    snapshot = tracker.snapshot(100);
    CHECK(snapshot.reset_frame_sequence == 1);
    CHECK(snapshot.unknown_frame_sequence == 0);

    tracker.receive(service_frame(0x05, 0x00, 0x01, 0x01), 200);
    snapshot = tracker.snapshot(200);
    CHECK(snapshot.reset_frame_sequence == 1);
    CHECK(snapshot.unknown_frame_sequence == 1);

    tracker.receive(height_frame(700), 300);
    snapshot = tracker.snapshot(300);
    CHECK(snapshot.height_frame_sequence == 1);
    CHECK(snapshot.fresh);
}

void test_recovery_ignores_height_until_rst_then_unlocks() {
    DeskRecoveryController recovery;
    RecoveryInput input;
    input.telemetry_fresh = true;
    input.telemetry_stable_frames = 5;

    input.unknown_frame_sequence = 1;
    auto result = recovery.update(input);
    CHECK(result.state == RecoveryState::kLocked);
    CHECK(result.command == Command::kIdle);
    CHECK(result.override_motion);

    input.now_ms = 50;
    input.unknown_frame_sequence = 2;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kLocked);
    CHECK(result.reason == RecoveryReason::kNone);
    CHECK(!result.reset_motion_controller);

    input.now_ms = 100;
    input.calibration_requested = true;
    input.user_activity_sequence = 4;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kCalibrating);
    CHECK(result.command == Command::kDownHold);

    input.calibration_requested = false;
    input.now_ms = 20000;
    input.height_frame_sequence = 100;
    input.unknown_frame_sequence = 3;
    input.telemetry_fresh = false;
    input.telemetry_stable_frames = 0;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kCalibrating);
    CHECK(result.command == Command::kDownHold);

    input.now_ms = 25000;
    input.reset_frame_sequence = 1;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kAwaitingTelemetry);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 25500;
    input.height_frame_sequence = 101;
    input.telemetry_fresh = true;
    input.telemetry_stable_frames = 3;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kReady);
    CHECK(result.override_motion);  // Emit one final IDLE; no input may leak through.
    CHECK(result.reset_motion_controller);
    input.now_ms = 25600;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kReady);
    CHECK(!result.override_motion);
}

void test_any_new_user_activity_cancels_calibration() {
    DeskRecoveryController recovery;
    RecoveryInput input;
    input.calibration_requested = true;
    input.user_activity_sequence = 10;
    auto result = recovery.update(input);
    CHECK(result.state == RecoveryState::kCalibrating);

    input.calibration_requested = false;
    input.now_ms = 100;
    input.user_activity_sequence = 11;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kReady);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == RecoveryReason::kCalibrationCancelled);
    input.now_ms = 150;
    result = recovery.update(input);
    CHECK(result.suppress_normal_input);
    input.now_ms = 175;
    input.user_activity_sequence = 12;
    result = recovery.update(input);
    CHECK(!result.suppress_normal_input);

    input.now_ms = 200;
    input.unknown_frame_sequence = 1;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kLocked);
    input.now_ms = 300;
    input.calibration_requested = true;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kCalibrating);
    input.calibration_requested = false;
    input.now_ms = 400;
    input.user_activity_sequence = 13;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kLocked);
}

void test_calibration_timeout_locks_until_new_rst_attempt() {
    RecoveryConfig config;
    config.calibration_timeout_ms = 1000;
    DeskRecoveryController recovery(config);
    RecoveryInput input;
    input.calibration_requested = true;
    recovery.update(input);
    input.calibration_requested = false;
    input.now_ms = 1000;
    const auto result = recovery.update(input);
    CHECK(result.state == RecoveryState::kLocked);
    CHECK(result.reason == RecoveryReason::kCalibrationTimeout);
}

void test_unknown_frame_does_not_interrupt_active_calibration() {
    DeskRecoveryController recovery;
    RecoveryInput input;
    input.calibration_requested = true;
    auto result = recovery.update(input);
    CHECK(result.state == RecoveryState::kCalibrating);
    CHECK(result.command == Command::kDownHold);

    input.calibration_requested = false;
    input.now_ms = 100;
    input.unknown_frame_sequence = 1;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kCalibrating);
    CHECK(result.command == Command::kDownHold);
    CHECK(result.override_motion);

    // RST completes the same recovery attempt despite the preceding error.
    input.now_ms = 200;
    input.reset_frame_sequence = 1;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kAwaitingTelemetry);
    CHECK(result.command == Command::kIdle);

    DeskRecoveryController cancelled;
    RecoveryInput cancelled_input;
    cancelled_input.calibration_requested = true;
    cancelled.update(cancelled_input);
    cancelled_input.calibration_requested = false;
    cancelled_input.unknown_frame_sequence = 1;
    cancelled_input.now_ms = 100;
    CHECK(cancelled.update(cancelled_input).state ==
          RecoveryState::kCalibrating);
    cancelled_input.user_activity_sequence = 1;
    cancelled_input.now_ms = 200;
    result = cancelled.update(cancelled_input);
    CHECK(result.state == RecoveryState::kLocked);
    CHECK(result.reason == RecoveryReason::kCalibrationCancelled);
}

void test_calibration_request_is_allowed_with_simultaneous_error_frame() {
    DeskRecoveryController recovery;
    RecoveryInput input;
    input.user_activity_sequence = 7;
    input.unknown_frame_sequence = 1;
    input.calibration_requested = true;
    const auto result = recovery.update(input);
    CHECK(result.state == RecoveryState::kCalibrating);
    CHECK(result.command == Command::kDownHold);
    CHECK(result.reason == RecoveryReason::kCalibrationStarted);
}

void test_locked_state_accepts_rst_and_reboot_starts_clean() {
    DeskRecoveryController recovery;
    RecoveryInput input;
    input.user_activity_sequence = 7;
    input.unknown_frame_sequence = 1;
    auto result = recovery.update(input);
    CHECK(result.state == RecoveryState::kLocked);

    input.now_ms = 100;
    input.reset_frame_sequence = 1;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kAwaitingTelemetry);

    input.now_ms = 200;
    input.height_frame_sequence = 1;
    input.telemetry_fresh = true;
    input.telemetry_stable_frames = 3;
    result = recovery.update(input);
    CHECK(result.state == RecoveryState::kReady);

    DeskRecoveryController after_reboot;
    CHECK(after_reboot.update({}).state == RecoveryState::kReady);
}

void test_stop_dominates_button_in_both_orders() {
    for (bool stop_first : {false, true}) {
        InputMailbox inputs;
        if (stop_first) inputs.stop();
        inputs.button(ButtonRequest::kGoStand);
        if (!stop_first) inputs.stop();
        inputs.dial(1, 100);
        const auto snapshot = inputs.consume();
        CHECK(snapshot.stop_requested);
        CHECK(snapshot.button == ButtonRequest::kNone);
        CHECK(snapshot.dial_sequence == 1);
        const auto next = inputs.consume();
        CHECK(!next.stop_requested);
        CHECK(next.button == ButtonRequest::kNone);
        CHECK(next.dial_sequence == 1);
    }
}

void test_reversal_and_buttons_stop_active_motion() {
    for (auto request : {ButtonRequest::kGoSit, ButtonRequest::kGoStand,
                         ButtonRequest::kSaveSit, ButtonRequest::kSaveStand}) {
        InputMailbox inputs;
        inputs.set_controller_state(ControlState::kManualHold, 1);
        inputs.button(request);
        CHECK(inputs.consume().stop_requested);
        inputs.dial(-1, 50);
        const auto snapshot = inputs.consume();
        CHECK(snapshot.stop_requested);
        CHECK(snapshot.dial_direction == -1);
        CHECK(snapshot.last_dial_ms == 50);
    }
    InputMailbox inputs;
    inputs.dial(0, 0);
    CHECK(inputs.consume().stop_requested);
}

void test_button_slot_replaces_without_queue() {
    InputMailbox inputs;
    inputs.button(ButtonRequest::kGoSit);
    inputs.button(ButtonRequest::kGoStand);
    CHECK(inputs.consume().button == ButtonRequest::kGoStand);
    CHECK(inputs.consume().button == ButtonRequest::kNone);
}

void test_calibration_request_is_edge_triggered_and_activity_is_monotonic() {
    InputMailbox inputs;
    inputs.user_activity();
    inputs.request_calibration();
    auto snapshot = inputs.consume();
    CHECK(snapshot.user_activity_sequence == 1);
    CHECK(snapshot.calibration_requested);
    snapshot = inputs.consume();
    CHECK(snapshot.user_activity_sequence == 1);
    CHECK(!snapshot.calibration_requested);
}

void test_disconnect_cancels_inputs_end_to_end() {
    InputMailbox inputs;
    DeskController controller;
    ControllerInput input;
    input.telemetry_fresh = true;
    input.telemetry_settled = true;
    input.telemetry_stable_frames = 5;
    input.height_tenths_cm = 900;
    inputs.dial(1, 0);
    inputs.button(ButtonRequest::kGoStand);
    inputs.stop();  // BLE disconnect uses exactly the same cancellation path.
    const auto snapshot = inputs.consume();
    input.stop_requested = snapshot.stop_requested;
    input.dial_sequence = snapshot.dial_sequence;
    input.dial_direction = snapshot.dial_direction;
    CHECK(controller.update(input).command == Command::kIdle);
    input.stop_requested = false;
    input.now_ms = 100;
    CHECK(controller.update(input).state == ControlState::kIdle);
}

void test_settled_preset_survives_ble_mailbox_and_fsm() {
    InputMailbox inputs;
    DeskController controller;
    ControllerInput input;
    input.telemetry_fresh = true;
    input.telemetry_settled = true;
    input.telemetry_stable_frames = 5;
    input.height_tenths_cm = 750;
    input.dial_direction = -1;
    input.dial_sequence = 3;
    controller.update(input);
    input.now_ms = 100;
    input.height_tenths_cm = 720;
    CHECK(controller.update(input).state == ControlState::kSettling);
    inputs.set_controller_state(controller.state(), controller.direction());

    inputs.button(ButtonRequest::kGoSit);
    const auto snapshot = inputs.consume();
    CHECK(!snapshot.stop_requested);
    CHECK(snapshot.button == ButtonRequest::kGoSit);
    input.now_ms = 700;
    input.height_tenths_cm = 700;
    input.stop_requested = snapshot.stop_requested;
    input.request = snapshot.button == ButtonRequest::kGoSit
                        ? ControlRequest::kGoSit : ControlRequest::kNone;
    CHECK(controller.update(input).command == Command::kUpHold);
    CHECK(inputs.consume().button == ButtonRequest::kNone);
}
}  // namespace

int main() {
    test_telemetry_requires_contiguous_samples();
    test_anomaly_invalidates_height_until_new_samples();
    test_telemetry_across_clock_wrap();
    test_long_press_detector_fires_once_and_handles_wrap();
    test_tracker_separates_known_service_and_unknown_frames();
    test_recovery_ignores_height_until_rst_then_unlocks();
    test_any_new_user_activity_cancels_calibration();
    test_calibration_timeout_locks_until_new_rst_attempt();
    test_unknown_frame_does_not_interrupt_active_calibration();
    test_calibration_request_is_allowed_with_simultaneous_error_frame();
    test_locked_state_accepts_rst_and_reboot_starts_clean();
    test_stop_dominates_button_in_both_orders();
    test_reversal_and_buttons_stop_active_motion();
    test_button_slot_replaces_without_queue();
    test_calibration_request_is_edge_triggered_and_activity_is_monotonic();
    test_disconnect_cancels_inputs_end_to_end();
    test_settled_preset_survives_ble_mailbox_and_fsm();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
