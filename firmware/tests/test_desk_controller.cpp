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

    input.now_ms = 100;
    result = controller.update(input);
    CHECK(result.state == ControlState::kManualStep);
    CHECK(result.command == Command::kUpStep);
    CHECK(result.reason == TransitionReason::kClassifiedStep);

    input.now_ms = 300;
    input.height_tenths_cm = 1004;
    input.telemetry_settled = false;
    result = controller.update(input);
    CHECK(result.state == ControlState::kManualStep);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 700;
    input.height_tenths_cm = 1010;
    input.telemetry_settled = true;
    result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kStepComplete);
}

void test_fast_rotation_enters_hold_without_step() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);
    controller.update(input);

    input.now_ms = 80;
    add_dial(input, 4, +1, 80);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 100;
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
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.reason == TransitionReason::kDialReleased);

    input.now_ms = 900;
    add_dial(input, 30, +1, 900);
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 2500;
    input.telemetry_settled = true;
    input.last_dial_ms = 900;
    result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.command == Command::kIdle);

    input.now_ms = 2600;
    result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.command == Command::kIdle);
}

void test_step_with_never_settled_telemetry_has_deadline() {
    DeskController controller;
    auto input = stable_input(0, 1000);
    add_dial(input, 1, +1, 0);
    controller.update(input);
    input.now_ms = 100;
    controller.update(input);

    input.now_ms = 300;
    input.height_tenths_cm = 1005;
    input.telemetry_settled = false;
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kManualStep);

    input.now_ms = 5100;
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

    input.now_ms = 100;
    controller.update(input);

    input.now_ms = 700;
    input.height_tenths_cm = 1010;
    add_dial(input, 3, +1, 700);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.reason == TransitionReason::kStepComplete);

    input.now_ms = 800;
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
    result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kBoundaryReached);
}

void test_soft_limits_block_only_outward_direction() {
    DeskController controller;
    auto input = stable_input(0, 1150);
    add_dial(input, 1, +1, 0);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.reason == TransitionReason::kSoftLimit);

    input.now_ms = 100;
    add_dial(input, 2, -1, 100);
    result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);
}

void test_soft_limits_recover_when_already_outside_configured_range() {
    DeskController controller;
    auto input = stable_input(0, 1160);
    add_dial(input, 1, +1, 0);
    auto result = controller.update(input);
    CHECK(result.state == ControlState::kIdle);
    CHECK(result.reason == TransitionReason::kSoftLimit);

    input.now_ms = 100;
    add_dial(input, 2, -1, 100);
    result = controller.update(input);
    CHECK(result.state == ControlState::kDialClassify);

    DeskController low_controller;
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
    CHECK(result.state == ControlState::kDialClassify);

    input.now_ms = 100;
    result = controller.update(input);
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
    const auto result = controller.update(input);
    CHECK(result.state == ControlState::kSettling);
    CHECK(result.command == Command::kIdle);
    CHECK(result.reason == TransitionReason::kButtonDuringMotion);
}

}  // namespace

int main() {
    test_slow_detent_is_one_autonomous_step();
    test_fast_rotation_enters_hold_without_step();
    test_short_fast_spin_from_90_stops_after_input_silence();
    test_continuous_reports_keep_hold_live();
    test_reversal_stops_without_starting_opposite_motion();
    test_reports_during_settling_are_discarded_not_queued();
    test_step_with_never_settled_telemetry_has_deadline();
    test_reports_arriving_as_step_completes_are_not_queued();
    test_stand_brakes_at_measured_coast_point();
    test_sit_brakes_at_measured_coast_point();
    test_soft_limits_block_only_outward_direction();
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
