#include "desk_controller.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

int failures = 0;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::cerr << __FILE__ << ':' << __LINE__                           \
                      << ": CHECK failed: " #condition << '\n';               \
            ++failures;                                                        \
        }                                                                      \
    } while (false)

using smart_desk::Command;
using smart_desk::ControlRequest;
using smart_desk::ControlState;
using smart_desk::ControllerInput;
using smart_desk::DeskController;
using smart_desk::TransitionReason;

ControllerInput stable_input(std::uint32_t now_ms,
                             std::uint16_t height_tenths_cm) {
    ControllerInput input;
    input.now_ms = now_ms;
    input.height_tenths_cm = height_tenths_cm;
    input.telemetry_fresh = true;
    input.telemetry_settled = true;
    input.telemetry_stable_frames = 5;
    input.last_dial_ms = now_ms;
    return input;
}

void add_dial(ControllerInput& input, std::uint32_t sequence,
              int direction, std::uint32_t report_ms) {
    input.dial_sequence = sequence;
    input.dial_direction = direction;
    input.last_dial_ms = report_ms;
}

void test_slow_detent_is_one_autonomous_step() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);

    auto result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
    CHECK(result.command == Command::kIdle);

    // One physical detent can produce two press reports.  It must still be a
    // STEP, not a HOLD.
    input.now_ms = 80;
    add_dial(input, 2, +1, 80);
    result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 299;
    result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 300;
    result = controller.update(input);
    CHECK(result.state == ControlState::kManualStep);
    CHECK(result.command == Command::kUpStep);
    CHECK(result.reason == TransitionReason::kClassifiedStep);

    input.now_ms = 500;
    input.height_tenths_cm = 1004;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    result = controller.update(input);
    CHECK(result.state == ControlState::kManualStep);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 900;
    input.height_tenths_cm = 1010;
    input.telemetry_settled = true;
    result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kStepComplete);
}

void test_moderate_rotation_enters_hold_on_third_report_without_step() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);
    controller.update(input);

    input.now_ms = 100;
    add_dial(input, 2, +1, 100);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
    CHECK(result.command == Command::kIdle);

    // The measured D100H can pause for about 270 ms before continued reports.
    // Report three proves rotation and starts HOLD immediately, without STEP.
    input.now_ms = 270;
    add_dial(input, 3, +1, 270);
    result = controller.update(input);
    CHECK(result.state == ControlState::kManualHold);
    CHECK(result.command == Command::kUpHold);
    CHECK(result.reason == TransitionReason::kClassifiedHold);
}

void test_short_fast_spin_from_90_stops_after_input_silence() {
    DeskController controller;
    auto input = stable_input(0, 900);
    add_dial(input, 1, +1, 0);
    controller.update(input);

    input.now_ms = 80;
    add_dial(input, 8, +1, 80);
    controller.update(input);

    input.now_ms = 100;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kManualHold);
    CHECK(result.command == Command::kUpHold);

    input.now_ms = 300;
    input.height_tenths_cm = 905;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    result = controller.update(input);
    CHECK(result.state == ControlState::kManualHold);
    CHECK(result.command == Command::kUpHold);

    input.now_ms = 500;
    input.height_tenths_cm = 910;
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kDialReleased);
    CHECK(input.height_tenths_cm < 1150);
}

void test_continuous_reports_keep_hold_live() {
    DeskController controller;
    auto input = stable_input(0, 900);
    add_dial(input, 1, +1, 0);
    controller.update(input);

    input.now_ms = 100;
    add_dial(input, 5, +1, 100);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kManualHold);
    CHECK(result.command == Command::kUpHold);

    for (std::uint32_t now = 200; now <= 1200; now += 100) {
        input.now_ms = now;
        input.height_tenths_cm = static_cast<std::uint16_t>(900 + now / 10);
        input.telemetry_settled = false;
        input.telemetry_stable_frames = 0;
        add_dial(input, 5 + now / 100, +1, now);
        result = controller.update(input);
        CHECK(result.state == ControlState::kManualHold);
        CHECK(result.command == Command::kUpHold);
    }
}

void test_reversal_stops_without_starting_opposite_motion() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);
    controller.update(input);
    input.now_ms = 100;
    add_dial(input, 5, +1, 100);
    controller.update(input);

    input.now_ms = 200;
    input.height_tenths_cm = 1003;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    add_dial(input, 6, -1, 200);
    const auto result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kDialReversal);
}

void test_reports_during_settling_are_discarded_not_queued() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);
    controller.update(input);
    input.now_ms = 100;
    add_dial(input, 5, +1, 100);
    controller.update(input);

    input.now_ms = 500;
    input.height_tenths_cm = 1010;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.reason == TransitionReason::kDialReleased);

    input.now_ms = 900;
    add_dial(input, 30, +1, 900);
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 1400;
    input.telemetry_settled = true;
    input.telemetry_stable_frames = 5;
    input.last_dial_ms = 900;
    result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 1500;
    result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.command == Command::kIdle);
}

void test_step_with_never_settled_telemetry_has_deadline() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);
    controller.update(input);
    input.now_ms = 300;
    controller.update(input);

    input.now_ms = 500;
    input.height_tenths_cm = 1005;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kManualStep);

    input.now_ms = 5300;
    input.height_tenths_cm = 1010;
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kStepWatchdog);
}

void test_reports_arriving_as_step_completes_are_not_queued() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);
    controller.update(input);

    input.now_ms = 300;
    controller.update(input);

    input.now_ms = 900;
    input.height_tenths_cm = 1010;
    add_dial(input, 3, +1, 700);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.reason == TransitionReason::kStepComplete);

    input.now_ms = 1000;
    result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.command == Command::kIdle);
}

void test_stand_brakes_at_measured_coast_point() {
    DeskController controller;
    auto input = stable_input(0, 900);
    input.request = ControlRequest::kGoStand;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kPresetHold);
    CHECK(result.command == Command::kUpHold);

    input.request = ControlRequest::kNone;
    input.now_ms = 5000;
    input.height_tenths_cm = 1125;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kBoundaryReached);
}

void test_sit_brakes_at_measured_coast_point() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    input.request = ControlRequest::kGoSit;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kPresetHold);
    CHECK(result.command == Command::kDownHold);

    input.request = ControlRequest::kNone;
    input.now_ms = 5000;
    input.height_tenths_cm = 820;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kBoundaryReached);
}

void test_rewriting_presets_does_not_change_manual_limits() {
    DeskController manual_controller;
    CHECK(manual_controller.set_presets(900, 1100));

    auto input = stable_input(0, 850);
    add_dial(input, 1, -1, 0);
    auto result = manual_controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
    CHECK(result.reason == TransitionReason::kDialStarted);

    DeskController preset_controller;
    CHECK(preset_controller.set_presets(900, 1100));
    input = stable_input(0, 1000);
    input.request = ControlRequest::kGoSit;
    result = preset_controller.update(input);
    CHECK(result.state == ControlState::kPresetHold);
    CHECK(result.command == Command::kDownHold);

    input.request = ControlRequest::kNone;
    input.now_ms = 3000;
    input.height_tenths_cm = 920;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    result = preset_controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.reason == TransitionReason::kBoundaryReached);
}

void test_same_direction_gesture_rearms_after_quiet_and_three_frames() {
    DeskController controller;
    auto input = stable_input(0, 900);
    add_dial(input, 1, +1, 0);
    controller.update(input);
    input.now_ms = 100;
    add_dial(input, 5, +1, 100);
    controller.update(input);

    input.now_ms = 500;
    input.height_tenths_cm = 910;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);

    input.now_ms = 1000;
    input.height_tenths_cm = 912;
    input.telemetry_stable_frames = 3;
    add_dial(input, 6, +1, 1000);
    result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
    CHECK(result.reason == TransitionReason::kDialStarted);

    input.now_ms = 1300;
    result = controller.update(input);
    CHECK(result.state == ControlState::kManualStep);
    CHECK(result.command == Command::kUpStep);
}

void test_opposite_gesture_waits_for_full_stop_but_is_not_queued() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);
    controller.update(input);
    input.now_ms = 100;
    add_dial(input, 5, +1, 100);
    controller.update(input);

    input.now_ms = 500;
    input.height_tenths_cm = 1010;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);

    input.now_ms = 1000;
    input.height_tenths_cm = 1012;
    input.telemetry_stable_frames = 3;
    add_dial(input, 6, -1, 1000);
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 1600;
    input.telemetry_settled = true;
    input.telemetry_stable_frames = 5;
    add_dial(input, 7, -1, 1600);
    result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
    CHECK(result.reason == TransitionReason::kDialStarted);
}

void test_soft_limits_block_only_outward_direction() {
    for (const int direction : {-1, +1}) {
        DeskController controller;
        auto input = stable_input(0, direction > 0 ? 1180 : 720);
        add_dial(input, 1, direction, 0);
        auto result = controller.update(input);
        CHECK(result.state == ControlState::kIdle);
        CHECK(result.reason == TransitionReason::kSoftLimit);

        input.now_ms = 100;
        add_dial(input, 2, -direction, 100);
        result = controller.update(input);
        CHECK(result.state == ControlState::kDialClassify);
    }
}

void test_manual_hold_brakes_before_working_limits() {
    for (const int direction : {-1, +1}) {
        DeskController controller;
        auto input = stable_input(0, direction > 0 ? 1100 : 800);
        add_dial(input, 3, direction, 0);
        const auto hold = direction > 0 ? Command::kUpHold : Command::kDownHold;
        CHECK(controller.update(input).command == hold);

        input.now_ms = 100;
        input.telemetry_settled = false;
        input.telemetry_stable_frames = 0;
        input.height_tenths_cm = direction > 0 ? 1154 : 741;
        CHECK(controller.update(input).command == hold);

        input.now_ms = 200;
        input.height_tenths_cm = direction > 0 ? 1155 : 740;
        const auto result = controller.update(input);
        CHECK(result.command == Command::kIdle);
        CHECK(result.state == ControlState::kSettling);
        CHECK(result.reason == TransitionReason::kBoundaryReached);
    }
}

void test_soft_limits_recover_when_already_outside_configured_range() {
    smart_desk::ControllerConfig config;
    config.soft_min_tenths_cm = 800;
    config.soft_max_tenths_cm = 1150;
    DeskController controller(config);
    auto input = stable_input(0, 1160);
    add_dial(input, 1, +1, 0);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.reason == TransitionReason::kSoftLimit);

    input.now_ms = 100;
    add_dial(input, 2, -1, 100);
    result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);

    DeskController low_controller(config);
    input = stable_input(0, 790);
    add_dial(input, 1, -1, 0);
    result = low_controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.reason == TransitionReason::kSoftLimit);

    input.now_ms = 100;
    add_dial(input, 2, +1, 100);
    result = low_controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
}

void test_stop_during_manual_hold_is_immediate_and_not_queued() {
    DeskController controller;
    auto input = stable_input(0, 900);
    add_dial(input, 1, +1, 0);
    controller.update(input);

    input.now_ms = 100;
    add_dial(input, 5, +1, 100);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kManualHold);
    CHECK(result.command == Command::kUpHold);

    input.now_ms = 200;
    input.height_tenths_cm = 903;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    input.stop_requested = true;
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kStopRequested);
}

void test_dial_sequence_wrap_is_counted_modulo_32_bits() {
    DeskController controller;
    auto input = stable_input(0, 900);
    input.dial_sequence = UINT32_MAX - 1U;
    input.stop_requested = true;
    controller.update(input);

    input.stop_requested = false;
    input.dial_sequence = 2;
    input.dial_direction = +1;
    input.last_dial_ms = 0;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kManualHold);
    CHECK(result.command == Command::kUpHold);
}

void test_stop_during_preset_does_not_queue_new_preset() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    input.request = ControlRequest::kGoStand;
    controller.update(input);

    input.now_ms = 500;
    input.request = ControlRequest::kGoSit;
    input.height_tenths_cm = 1010;
    input.telemetry_settled = false;
    input.telemetry_stable_frames = 0;
    const auto result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kButtonDuringMotion);
}

}  // namespace

void test_preset_consumes_overlapping_dial_even_when_rejected() {
    for (const bool fresh : {false, true}) {
        DeskController controller;
        auto input = stable_input(0, 900);
        add_dial(input, 1, +1, 0);
        input.telemetry_fresh = fresh;
        input.request = ControlRequest::kGoStand;
        controller.update(input);
        CHECK(controller.consumed_dial_sequence() == 1);

        input.now_ms = 100;
        input.telemetry_fresh = true;
        input.request = ControlRequest::kNone;
        const auto result = controller.update(input);
        CHECK(result.state == (fresh ? ControlState::kPresetHold
                                     : ControlState::kIdle));
    }
}

void test_expired_dial_cannot_start_motion() {
    for (const std::uint32_t count : {1U, 3U}) {
        DeskController controller;
        auto input = stable_input(1000, 900);
        add_dial(input, count, +1, 600);
        const auto result = controller.update(input);
        CHECK(result.command == Command::kIdle);
        CHECK(result.state == ControlState::kIdle);
        CHECK(controller.consumed_dial_sequence() == count);
    }
}

void test_preset_does_not_pulse_hold_inside_braking_zone() {
    for (const auto request : {ControlRequest::kGoSit, ControlRequest::kGoStand}) {
        DeskController controller;
        auto input = stable_input(0, request == ControlRequest::kGoSit ? 815 : 1130);
        input.request = request;
        CHECK(controller.update(input).command == Command::kIdle);
    }
}

void test_classification_rechecks_limits_before_step() {
    DeskController controller;
    auto input = stable_input(0, 1170);
    add_dial(input, 1, +1, 0);
    CHECK(controller.update(input).state == ControlState::kDialClassify);
    input.now_ms = 300;
    input.height_tenths_cm = 1175;
    CHECK(controller.update(input).command == Command::kIdle);
}

void test_step_survives_normal_writer_sampling_delay() {
    DeskController controller;
    auto input = stable_input(90, 900);
    add_dial(input, 1, +1, 0);
    CHECK(controller.update(input).state == ControlState::kDialClassify);
    input.now_ms = 390;
    CHECK(controller.update(input).command == Command::kUpStep);
}

void test_invalid_height_stops_hold_in_both_directions() {
    for (const int direction : {-1, +1}) {
        DeskController controller;
        auto input = stable_input(0, 900);
        add_dial(input, 3, direction, 0);
        controller.update(input);
        input.now_ms = 100;
        input.height_tenths_cm = direction > 0 ? 699 : 1201;
        CHECK(controller.update(input).command == Command::kIdle);
    }
}

void test_stop_and_telemetry_failure_in_every_active_state() {
    for (const int direction : {-1, +1}) {
        for (const auto state : {ControlState::kDialClassify,
                                ControlState::kManualStep,
                                ControlState::kManualHold,
                                ControlState::kPresetHold}) {
            for (const int fault : {0, 1, 2}) {
                DeskController controller;
                auto input = stable_input(0, 900);
                if (state == ControlState::kPresetHold) {
                    input.request = direction > 0 ? ControlRequest::kGoStand
                                                  : ControlRequest::kGoSit;
                } else {
                    add_dial(input, state == ControlState::kManualHold ? 3 : 1,
                             direction, 0);
                }
                auto result = controller.update(input);
                if (state == ControlState::kManualStep) {
                    input.now_ms = 300;
                    result = controller.update(input);
                }
                CHECK(result.state == state);
                input.request = ControlRequest::kNone;
                input.now_ms += 100;
                if (fault == 0) input.stop_requested = true;
                if (fault == 1) input.telemetry_fresh = false;
                if (fault == 2) ++input.anomaly_sequence;
                result = controller.update(input);
                CHECK(result.command == Command::kIdle);
                CHECK(result.state == ControlState::kIdle ||
                      result.state == ControlState::kSettling);
            }
        }
    }
}

void test_clock_wrap_preserves_classification_and_lease() {
    DeskController controller;
    auto input = stable_input(UINT32_MAX - 199, 900);
    add_dial(input, 1, +1, input.now_ms);
    controller.update(input);
    input.now_ms = 100;  // 300 ms later, across wrap.
    CHECK(controller.update(input).command == Command::kUpStep);

    DeskController hold;
    input = stable_input(UINT32_MAX - 199, 900);
    add_dial(input, 3, +1, input.now_ms);
    CHECK(hold.update(input).command == Command::kUpHold);
    input.now_ms = 149;
    CHECK(hold.update(input).command == Command::kUpHold);
    input.now_ms = 150;  // Exactly 350 ms of silence.
    CHECK(hold.update(input).reason == TransitionReason::kDialReleased);
}

void test_absolute_hold_deadlines_with_continued_progress() {
    for (const bool manual : {false, true}) {
        DeskController controller;
        auto input = stable_input(0, 750);
        if (manual) add_dial(input, 3, +1, 0);
        else input.request = ControlRequest::kGoStand;
        CHECK(controller.update(input).command == Command::kUpHold);
        input.request = ControlRequest::kNone;
        const std::uint32_t deadline = manual ? 30000 : 20000;
        for (input.now_ms = 100; input.now_ms < deadline; input.now_ms += 100) {
            ++input.height_tenths_cm;
            if (manual) add_dial(input, input.dial_sequence + 1, +1, input.now_ms);
            CHECK(controller.update(input).command == Command::kUpHold);
        }
        ++input.height_tenths_cm;
        if (manual) add_dial(input, input.dial_sequence + 1, +1, input.now_ms);
        const auto result = controller.update(input);
        CHECK(result.command == Command::kIdle);
        CHECK(result.reason == (manual ? TransitionReason::kManualWatchdog
                                       : TransitionReason::kPresetWatchdog));
    }
}

void test_generated_input_sequences_preserve_safety_invariants() {
    // Deterministic simulated input stress; this is not a mechanical soak test.
    std::uint32_t seed = 0xC0FFEE;
    const auto next = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return seed;
    };
    DeskController controller;
    auto input = stable_input(UINT32_MAX - 10000, 900);
    for (unsigned i = 0; i < 200000; ++i) {
        input.now_ms += next() % 401;
        input.height_tenths_cm = static_cast<std::uint16_t>(690 + next() % 521);
        input.telemetry_fresh = next() % 8 != 0;
        input.telemetry_stable_frames = next() % 6;
        input.telemetry_settled = input.telemetry_stable_frames == 5;
        input.stop_requested = next() % 13 == 0;
        input.request = static_cast<ControlRequest>(next() % 3);
        if (next() % 5 == 0) ++input.anomaly_sequence;
        if (next() % 3 != 0) {
            add_dial(input, input.dial_sequence + 1 + next() % 4,
                     next() % 2 == 0 ? 1 : -1, input.now_ms);
        }
        const auto result = controller.update(input);
        CHECK(controller.consumed_dial_sequence() == input.dial_sequence);
        if (result.command != Command::kIdle) {
            CHECK(!input.stop_requested);
            CHECK(input.telemetry_fresh);
            CHECK(input.height_tenths_cm >= 700 && input.height_tenths_cm <= 1200);
            if (result.command == Command::kUpStep) CHECK(input.height_tenths_cm <= 1190);
            if (result.command == Command::kDownStep) CHECK(input.height_tenths_cm >= 710);
        }
    }
}

void test_preset_starts_on_first_press_after_boundary_has_settled() {
    for (const bool lower : {true, false}) {
        DeskController controller;
        auto input = stable_input(0, lower ? 750 : 1150);
        add_dial(input, 3, lower ? -1 : +1, 0);
        controller.update(input);

        input.now_ms = 100;
        input.height_tenths_cm = lower ? 740 : 1155;
        input.telemetry_settled = false;
        input.telemetry_stable_frames = 0;
        CHECK(controller.update(input).state == ControlState::kSettling);

        // The button arrives with the snapshot that first proves the stop.
        // It must not be discarded just because the previous state is SETTLING.
        input.now_ms = 700;
        input.height_tenths_cm = lower ? 720 : 1180;
        input.telemetry_settled = true;
        input.telemetry_stable_frames = 5;
        input.request = lower ? ControlRequest::kGoSit : ControlRequest::kGoStand;
        const auto result = controller.update(input);
        CHECK(result.state == ControlState::kPresetHold);
        CHECK(result.command == (lower ? Command::kUpHold : Command::kDownHold));
    }
}

void test_preset_during_coast_is_discarded_without_delayed_motion() {
    for (int fault = 0; fault < 6; ++fault) {
        DeskController controller;
        auto input = stable_input(0, 750);
        add_dial(input, 3, -1, 0);
        controller.update(input);
        input.now_ms = 100;
        input.height_tenths_cm = 720;
        controller.update(input);

        input.now_ms = 700;
        input.height_tenths_cm = 700;
        input.request = ControlRequest::kGoSit;
        if (fault == 0) input.telemetry_fresh = false;
        if (fault == 1) input.telemetry_settled = false;
        if (fault == 2) input.telemetry_stable_frames = 3;
        if (fault == 3) input.stop_requested = true;
        if (fault == 4) input.last_dial_ms = 600;
        if (fault == 5) input.now_ms = 200;
        const auto rejected = controller.update(input);
        CHECK(rejected.command == Command::kIdle);
        CHECK(rejected.state == ControlState::kSettling);

        input = stable_input(1300, 700);
        input.dial_sequence = 3;
        input.dial_direction = -1;
        input.last_dial_ms = 600;
        CHECK(controller.update(input).command == Command::kIdle);
        input.now_ms = 1400;
        CHECK(controller.update(input).state == ControlState::kIdle);
        input.request = ControlRequest::kGoSit;
        CHECK(controller.update(input).command == Command::kUpHold);
    }
}

void test_repeated_rejected_presets_do_not_extend_settling_watchdog() {
    DeskController controller;
    auto input = stable_input(0, 750);
    add_dial(input, 3, -1, 0);
    controller.update(input);
    input.now_ms = 100;
    input.height_tenths_cm = 720;
    controller.update(input);
    input.telemetry_fresh = false;
    input.request = ControlRequest::kGoSit;
    for (input.now_ms = 200; input.now_ms < 7100; input.now_ms += 100) {
        CHECK(controller.update(input).command == Command::kIdle);
    }
    input.request = ControlRequest::kNone;
    const auto result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.reason == TransitionReason::kSettleWatchdog);
}

int main() {
    test_preset_starts_on_first_press_after_boundary_has_settled();
    test_preset_during_coast_is_discarded_without_delayed_motion();
    test_repeated_rejected_presets_do_not_extend_settling_watchdog();
    test_stop_and_telemetry_failure_in_every_active_state();
    test_clock_wrap_preserves_classification_and_lease();
    test_absolute_hold_deadlines_with_continued_progress();
    test_generated_input_sequences_preserve_safety_invariants();
    test_preset_consumes_overlapping_dial_even_when_rejected();
    test_expired_dial_cannot_start_motion();
    test_preset_does_not_pulse_hold_inside_braking_zone();
    test_classification_rechecks_limits_before_step();
    test_step_survives_normal_writer_sampling_delay();
    test_invalid_height_stops_hold_in_both_directions();
    test_slow_detent_is_one_autonomous_step();
    test_moderate_rotation_enters_hold_on_third_report_without_step();
    test_short_fast_spin_from_90_stops_after_input_silence();
    test_continuous_reports_keep_hold_live();
    test_reversal_stops_without_starting_opposite_motion();
    test_reports_during_settling_are_discarded_not_queued();
    test_step_with_never_settled_telemetry_has_deadline();
    test_reports_arriving_as_step_completes_are_not_queued();
    test_stand_brakes_at_measured_coast_point();
    test_sit_brakes_at_measured_coast_point();
    test_rewriting_presets_does_not_change_manual_limits();
    test_same_direction_gesture_rearms_after_quiet_and_three_frames();
    test_opposite_gesture_waits_for_full_stop_but_is_not_queued();
    test_soft_limits_block_only_outward_direction();
    test_manual_hold_brakes_before_working_limits();
    test_soft_limits_recover_when_already_outside_configured_range();
    test_stop_during_manual_hold_is_immediate_and_not_queued();
    test_dial_sequence_wrap_is_counted_modulo_32_bits();
    test_stop_during_preset_does_not_queue_new_preset();

    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }

    std::cout << "all desk controller tests passed\n";
    return EXIT_SUCCESS;
}
