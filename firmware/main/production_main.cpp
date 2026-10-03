#include <array>
#include <atomic>
#include <cstdint>

#include "ble_hid_host.hpp"
#include "desk_inputs.hpp"
#include "desk_recovery.hpp"
#include "preset_store.hpp"
#include "persistent_journal.hpp"
#include "uart_transport.hpp"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr int kTxGpio = 17;
constexpr int kRxGpio = 18;
constexpr TickType_t kPanelPeriod = pdMS_TO_TICKS(100);
constexpr smart_desk::ControllerConfig kControllerConfig{};

// One lock protects the complete input snapshot. No UART, logging, NVS or
// controller execution occurs while it is held.
portMUX_TYPE g_inputs_mux = portMUX_INITIALIZER_UNLOCKED;
smart_desk::TelemetryTracker g_telemetry(kControllerConfig);
smart_desk::InputMailbox g_inputs;
std::atomic<std::uint32_t> g_panel_heartbeat{0};

std::uint32_t now_ms() {
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
}

struct RuntimeInputs {
    smart_desk::TelemetrySnapshot telemetry;
    smart_desk::InputSnapshot ble;
    std::uint32_t now_ms;
};

RuntimeInputs take_inputs() {
    portENTER_CRITICAL(&g_inputs_mux);
    const auto now = now_ms();
    const RuntimeInputs result{g_telemetry.snapshot(now), g_inputs.consume(), now};
    portEXIT_CRITICAL(&g_inputs_mux);
    return result;
}

void ble_motion_action(const BleHidAction& action, void*) {
    using smart_desk::ButtonRequest;
    portENTER_CRITICAL(&g_inputs_mux);
    switch (action.type) {
        case BleHidActionType::kDialRaw: g_inputs.dial(action.value, now_ms()); break;
        case BleHidActionType::kStop: g_inputs.stop(); break;
        case BleHidActionType::kDisconnected:
            g_inputs.user_activity();
            g_inputs.stop();
            break;
        case BleHidActionType::kGoSit: g_inputs.button(ButtonRequest::kGoSit); break;
        case BleHidActionType::kGoStand:
            g_inputs.button(ButtonRequest::kGoStand);
            break;
        case BleHidActionType::kSaveSit:
            g_inputs.button(ButtonRequest::kSaveSit);
            break;
        case BleHidActionType::kSaveStand:
            g_inputs.button(ButtonRequest::kSaveStand);
            break;
        case BleHidActionType::kDialDelta: break;  // Production uses raw reports.
        case BleHidActionType::kUserActivity: g_inputs.user_activity(); break;
        case BleHidActionType::kStartCalibration:
            g_inputs.request_calibration();
            break;
    }
    portEXIT_CRITICAL(&g_inputs_mux);
}

void reader_task(void*) {
    smart_desk::StreamParser parser(smart_desk::Direction::kControllerToPanel);
    std::array<std::uint8_t, 64> bytes{};
    while (true) {
        const int received =
            uart_read_bytes(UART_NUM_1, bytes.data(), bytes.size(), pdMS_TO_TICKS(100));
        for (int index = 0; index < received; ++index) {
            const auto frame = parser.feed(bytes[static_cast<std::size_t>(index)]);
            if (const auto rejected = parser.take_rejected()) {
                persistent_journal::receive(*rejected, true);
            }
            if (!frame) {
                continue;
            }
            portENTER_CRITICAL(&g_inputs_mux);
            // Capture before publishing telemetry to the writer, so the
            // fault's RX record precedes its safety TX in the journal.
            persistent_journal::receive(*frame);
            g_telemetry.receive(*frame, now_ms());
            portEXIT_CRITICAL(&g_inputs_mux);
        }
    }
}

bool transmit(smart_desk::Command command,
              smart_desk::journal::Context context = smart_desk::journal::Context::kNone,
              bool routine_step_idle = false) {
    const auto frame = smart_desk::make_panel_frame(command);
    const int written = uart_write_bytes(UART_NUM_1, frame.data(), frame.size());
    if (written != static_cast<int>(frame.size()) ||
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(50)) != ESP_OK) {
        persistent_journal::transmit_error(command);
        return false;
    }
    persistent_journal::transmitted(command, context, routine_step_idle);
    return true;
}

void panel_writer_task(void*) {
    auto presets = smart_desk::load_presets(kControllerConfig);
    persistent_journal::presets(presets.sit, presets.stand, presets.sit, presets.stand);
    auto controller_config = kControllerConfig;
    controller_config.sit_target_tenths_cm = presets.sit;
    controller_config.stand_target_tenths_cm = presets.stand;
    smart_desk::DeskController controller(controller_config);
    smart_desk::DeskRecoveryController recovery;

    // Ignore any BLE reports received while this task was being created.
    smart_desk::ControllerInput initial_input;
    initial_input.dial_sequence = take_inputs().ble.dial_sequence;
    initial_input.stop_requested = true;
    controller.update(initial_input);
    auto preset_context = smart_desk::journal::Context::kNone;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        const auto inputs = take_inputs();
        const auto& telemetry = inputs.telemetry;
        const auto& dial = inputs.ble;
        const std::uint16_t control_height = telemetry.height_tenths_cm;
        const bool control_fresh = telemetry.fresh;
        const bool control_settled =
            telemetry.stable_frames >= controller.config().full_settle_frames;

        smart_desk::RecoveryInput recovery_input;
        recovery_input.now_ms = inputs.now_ms;
        recovery_input.height_frame_sequence = telemetry.height_frame_sequence;
        recovery_input.reset_frame_sequence = telemetry.reset_frame_sequence;
        recovery_input.unknown_frame_sequence = telemetry.unknown_frame_sequence;
        recovery_input.user_activity_sequence = dial.user_activity_sequence;
        recovery_input.telemetry_stable_frames = telemetry.stable_frames;
        recovery_input.telemetry_fresh = telemetry.fresh;
        recovery_input.calibration_requested = dial.calibration_requested;
        const auto recovery_output = recovery.update(recovery_input);
        if (recovery_output.reset_motion_controller) {
            const auto current_config = controller.config();
            controller = smart_desk::DeskController(current_config);
            smart_desk::ControllerInput sync_input;
            sync_input.now_ms = inputs.now_ms;
            sync_input.dial_sequence = dial.dial_sequence;
            sync_input.stop_requested = true;
            controller.update(sync_input);
        }
        // Send the safety override before any preset NVS write.
        // Recovery must never delay the first IDLE after an unexpected frame.
        if (recovery_output.override_motion ||
            recovery_output.suppress_normal_input) {
            portENTER_CRITICAL(&g_inputs_mux);
            g_inputs.set_controller_state(smart_desk::ControlState::kIdle, 0);
            portEXIT_CRITICAL(&g_inputs_mux);
            const auto context = recovery_output.state == smart_desk::RecoveryState::kCalibrating
                ? smart_desk::journal::Context::kCalibration
                : smart_desk::journal::Context::kNone;
            if (transmit(recovery_output.command, context)) {
                g_panel_heartbeat.fetch_add(1);
                if (recovery_output.command == smart_desk::Command::kIdle) {
                    persistent_journal::maintain_after_idle();
                }
            } else {
                portENTER_CRITICAL(&g_inputs_mux);
                g_inputs.stop();
                g_inputs.user_activity();
                portEXIT_CRITICAL(&g_inputs_mux);
            }
            persistent_journal::cycle_completed(telemetry.unknown_frame_sequence);
        }

        if (recovery_output.override_motion ||
            recovery_output.suppress_normal_input) {
            xTaskDelayUntil(&last_wake, kPanelPeriod);
            continue;
        }

        bool stop_requested = dial.stop_requested;
        const auto raw_request = dial.button;

        smart_desk::ButtonRequest request = raw_request;
        const bool save_request = request == smart_desk::ButtonRequest::kSaveSit ||
                                  request == smart_desk::ButtonRequest::kSaveStand;
        if (controller.state() != smart_desk::ControlState::kIdle &&
            save_request) {
            stop_requested = true;
            request = smart_desk::ButtonRequest::kNone;
        }

        if (controller.state() == smart_desk::ControlState::kIdle && !stop_requested &&
            (request == smart_desk::ButtonRequest::kSaveSit ||
             request == smart_desk::ButtonRequest::kSaveStand)) {
            if (control_fresh && control_settled &&
                control_height >= controller.config().physical_min_tenths_cm &&
                control_height <= controller.config().physical_max_tenths_cm) {
                smart_desk::DeskPresets candidate = presets;
                if (request == smart_desk::ButtonRequest::kSaveSit) {
                    candidate.sit = control_height;
                } else {
                    candidate.stand = control_height;
                }
                if ((candidate.sit != presets.sit || candidate.stand != presets.stand) &&
                    controller.valid_presets(candidate.sit, candidate.stand) &&
                    smart_desk::save_presets(candidate, controller.config()) &&
                    controller.set_presets(candidate.sit, candidate.stand)) {
                    persistent_journal::presets(
                        presets.sit, presets.stand, candidate.sit, candidate.stand,
                        request == smart_desk::ButtonRequest::kSaveSit
                            ? smart_desk::journal::Context::kSit
                            : smart_desk::journal::Context::kStand);
                    presets = candidate;
                }
            }
            request = smart_desk::ButtonRequest::kNone;
            // Persistence may block. Never start a dial command using the
            // telemetry/input snapshot captured before the NVS operation.
            stop_requested = true;
        }

        smart_desk::ControlRequest motion_request = smart_desk::ControlRequest::kNone;
        if (request == smart_desk::ButtonRequest::kGoSit) {
            motion_request = smart_desk::ControlRequest::kGoSit;
        } else if (request == smart_desk::ButtonRequest::kGoStand) {
            motion_request = smart_desk::ControlRequest::kGoStand;
        }

        smart_desk::ControllerInput controller_input;
        controller_input.now_ms = inputs.now_ms;
        controller_input.height_tenths_cm = control_height;
        controller_input.telemetry_fresh = control_fresh;
        controller_input.telemetry_settled = control_settled;
        controller_input.telemetry_stable_frames = telemetry.stable_frames;
        controller_input.anomaly_sequence = telemetry.anomaly_sequence;
        controller_input.dial_sequence = dial.dial_sequence;
        controller_input.dial_direction = dial.dial_direction;
        controller_input.last_dial_ms = dial.last_dial_ms;
        controller_input.stop_requested = stop_requested;
        controller_input.request = motion_request;

        const smart_desk::ControllerOutput controller_output =
            controller.update(controller_input);
        portENTER_CRITICAL(&g_inputs_mux);
        g_inputs.set_controller_state(controller.state(), controller.direction());
        portEXIT_CRITICAL(&g_inputs_mux);

        auto journal_context = smart_desk::journal::Context::kNone;
        if (controller_output.state == smart_desk::ControlState::kManualStep ||
            controller_output.state == smart_desk::ControlState::kManualHold) {
            journal_context = smart_desk::journal::Context::kDial;
        } else if (controller_output.state == smart_desk::ControlState::kPresetHold) {
            if (motion_request == smart_desk::ControlRequest::kGoSit) {
                preset_context = smart_desk::journal::Context::kSit;
            } else if (motion_request == smart_desk::ControlRequest::kGoStand) {
                preset_context = smart_desk::journal::Context::kStand;
            }
            journal_context = preset_context;
        }
        const bool routine_step_idle =
            controller_output.state == smart_desk::ControlState::kManualStep &&
            controller_output.reason == smart_desk::TransitionReason::kNone;
        if (transmit(controller_output.command, journal_context, routine_step_idle)) {
            g_panel_heartbeat.fetch_add(1);
            if (controller_output.command == smart_desk::Command::kIdle &&
                controller_output.state == smart_desk::ControlState::kIdle) {
                persistent_journal::maintain_after_idle();
            }
        } else {
            // Drop active intent after a transport failure. A stalled writer
            // remains covered by the independent heartbeat supervisor.
            portENTER_CRITICAL(&g_inputs_mux);
            g_inputs.stop();
            portEXIT_CRITICAL(&g_inputs_mux);
        }
        persistent_journal::cycle_completed(telemetry.unknown_frame_sequence);
        xTaskDelayUntil(&last_wake, kPanelPeriod);
    }
}

void panel_watchdog_task(void*) {
    std::uint32_t previous = g_panel_heartbeat.load();
    unsigned missed_seconds = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        const std::uint32_t current = g_panel_heartbeat.load();
        if (current != previous) {
            previous = current;
            missed_seconds = 0;
            continue;
        }

        ++missed_seconds;
        if (missed_seconds >= 3) {
            esp_restart();
        }
    }
}

}  // namespace

extern "C" void app_main() {
    configure_uart(UART_NUM_1, kTxGpio, kRxGpio);
    ESP_ERROR_CHECK(transmit(smart_desk::Command::kIdle) ? ESP_OK : ESP_FAIL);
    persistent_journal::begin();
    persistent_journal::transmitted(smart_desk::Command::kIdle);
    persistent_journal::start_worker();
    // BLE initialization owns NVS initialization. Start the writer afterwards
    // so its persisted preset read cannot race nvs_flash_init().
    start_ble_hid_host(ble_motion_action, nullptr, false);
    ESP_ERROR_CHECK(
        xTaskCreate(reader_task, "controller_rx", 4096, nullptr, 5, nullptr) == pdPASS
            ? ESP_OK
            : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(
        xTaskCreate(panel_writer_task, "panel_tx", 6144, nullptr, 5, nullptr) == pdPASS
            ? ESP_OK
            : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(panel_watchdog_task, "panel_watchdog", 2048, nullptr, 6,
                                nullptr) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
}
