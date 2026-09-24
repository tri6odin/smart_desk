#include <array>
#include <atomic>
#include <cstdint>

#include "ble_hid_logger.hpp"
#include "desk_inputs.hpp"
#include "desk_recovery.hpp"
#include "preset_store.hpp"
#include "uart_transport.hpp"
#include "esp_log.h"
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
    smart_desk::Frame last_frame{};
    bool have_last_frame = false;
    while (true) {
        const int received =
            uart_read_bytes(UART_NUM_1, bytes.data(), bytes.size(), pdMS_TO_TICKS(100));
        for (int index = 0; index < received; ++index) {
            const auto frame = parser.feed(bytes[static_cast<std::size_t>(index)]);
            if (!frame) {
                continue;
            }
            portENTER_CRITICAL(&g_inputs_mux);
            g_telemetry.receive(*frame, now_ms());
            portEXIT_CRITICAL(&g_inputs_mux);
            if (!have_last_frame || *frame != last_frame) {
                const auto kind = smart_desk::classify_controller_frame(*frame);
                if (kind == smart_desk::ControllerFrameKind::kReset) {
                    ESP_LOGW("desk.controller", "RST frame received");
                } else if (kind ==
                           smart_desk::ControllerFrameKind::kUnknown) {
                    ESP_LOGE(
                        "desk.controller",
                        "unknown valid frame; motion locked until successful RST");
                }
                ESP_LOG_BUFFER_HEX_LEVEL("desk.controller", frame->data(),
                                         frame->size(), ESP_LOG_INFO);
                last_frame = *frame;
                have_last_frame = true;
            }
        }
    }
}

bool transmit(smart_desk::Command command) {
    const auto frame = smart_desk::make_panel_frame(command);
    const int written = uart_write_bytes(UART_NUM_1, frame.data(), frame.size());
    if (written != static_cast<int>(frame.size()) ||
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(50)) != ESP_OK) {
        ESP_LOGE("desk.control", "UART TX failed: %d/8 bytes", written);
        return false;
    }
    return true;
}

void panel_writer_task(void*) {
    auto presets = smart_desk::load_presets(kControllerConfig);
    auto controller_config = kControllerConfig;
    controller_config.sit_target_tenths_cm = presets.sit;
    controller_config.stand_target_tenths_cm = presets.stand;
    smart_desk::DeskController controller(controller_config);
    const bool persisted_recovery_lock = smart_desk::load_recovery_lock();
    smart_desk::DeskRecoveryController recovery({}, persisted_recovery_lock);
    bool recovery_lock_persisted = persisted_recovery_lock;
    bool recovery_lock_save_attempted = false;
    std::uint32_t recovery_lock_last_attempt_ms = 0;
    if (persisted_recovery_lock) {
        ESP_LOGE("desk.recovery",
                 "persistent motion lock active; hold dial 10 s to calibrate");
    }

    // Ignore any BLE reports received while this task was being created.
    smart_desk::ControllerInput initial_input;
    initial_input.dial_sequence = take_inputs().ble.dial_sequence;
    initial_input.stop_requested = true;
    controller.update(initial_input);
    smart_desk::ControlState logged_state = controller.state();
    smart_desk::RecoveryState logged_recovery_state = recovery.state();
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
            logged_state = controller.state();
        }
        // Send the safety override before logging or NVS writes. A persistent
        // lock must never delay the first IDLE after an unexpected frame.
        if (recovery_output.override_motion ||
            recovery_output.suppress_normal_input) {
            portENTER_CRITICAL(&g_inputs_mux);
            g_inputs.set_controller_state(smart_desk::ControlState::kIdle, 0);
            portEXIT_CRITICAL(&g_inputs_mux);
            if (transmit(recovery_output.command)) {
                g_panel_heartbeat.fetch_add(1);
            } else {
                portENTER_CRITICAL(&g_inputs_mux);
                g_inputs.stop();
                g_inputs.user_activity();
                portEXIT_CRITICAL(&g_inputs_mux);
            }
        }

        if (recovery_output.state == smart_desk::RecoveryState::kLocked &&
            !recovery_lock_persisted &&
            (!recovery_lock_save_attempted ||
             static_cast<std::uint32_t>(
                 inputs.now_ms - recovery_lock_last_attempt_ms) >= 1000)) {
            recovery_lock_save_attempted = true;
            recovery_lock_last_attempt_ms = inputs.now_ms;
            if (!smart_desk::save_recovery_lock(true)) {
                ESP_LOGE("desk.recovery", "could not persist motion lock");
            } else {
                recovery_lock_persisted = true;
            }
        } else if (recovery_output.reason ==
                   smart_desk::RecoveryReason::kCalibrationComplete) {
            if (recovery_lock_persisted &&
                !smart_desk::save_recovery_lock(false)) {
                ESP_LOGE("desk.recovery", "could not clear persistent motion lock");
            } else {
                recovery_lock_persisted = false;
            }
        }

        if (recovery_output.reason != smart_desk::RecoveryReason::kNone ||
            recovery_output.state != logged_recovery_state) {
            ESP_LOGW("desk.recovery", "%s -> %s: %s",
                     smart_desk::recovery_state_name(logged_recovery_state),
                     smart_desk::recovery_state_name(recovery_output.state),
                     smart_desk::recovery_reason_name(recovery_output.reason));
            logged_recovery_state = recovery_output.state;
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
            ESP_LOGW("desk.control", "button during motion -> STOP; action not queued");
            stop_requested = true;
            request = smart_desk::ButtonRequest::kNone;
        }

        if (controller.state() == smart_desk::ControlState::kIdle && !stop_requested &&
            (request == smart_desk::ButtonRequest::kSaveSit ||
             request == smart_desk::ButtonRequest::kSaveStand)) {
            if (!control_fresh || !control_settled ||
                control_height < controller.config().physical_min_tenths_cm ||
                control_height > controller.config().physical_max_tenths_cm) {
                ESP_LOGE("desk.control",
                         "preset save BLOCKED: fresh=%d settled=%d height=%u.%u",
                         control_fresh, control_settled, control_height / 10,
                         control_height % 10);
            } else {
                smart_desk::DeskPresets candidate = presets;
                if (request == smart_desk::ButtonRequest::kSaveSit) {
                    candidate.sit = control_height;
                } else {
                    candidate.stand = control_height;
                }
                if (!controller.valid_presets(candidate.sit, candidate.stand)) {
                    ESP_LOGE(
                        "desk.control",
                        "preset save BLOCKED: require 70..120 cm and SIT+10<=STAND");
                } else if (!smart_desk::save_presets(candidate, controller.config())) {
                    ESP_LOGE("desk.control", "preset save failed in NVS");
                } else if (!controller.set_presets(candidate.sit, candidate.stand)) {
                    ESP_LOGE("desk.control",
                             "internal preset validation disagrees with NVS");
                } else {
                    presets = candidate;
                    ESP_LOGW("desk.control",
                             "saved SIT=%u.%u STAND=%u.%u cm; limits unchanged at "
                             "70.0..120.0",
                             presets.sit / 10, presets.sit % 10, presets.stand / 10,
                             presets.stand % 10);
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

        if (controller_output.reason != smart_desk::TransitionReason::kNone ||
            controller_output.state != logged_state) {
            ESP_LOGW("desk.control", "%s -> %s: %s at %u.%u cm",
                     smart_desk::control_state_name(logged_state),
                     smart_desk::control_state_name(controller_output.state),
                     smart_desk::transition_reason_name(controller_output.reason),
                     control_height / 10, control_height % 10);
            logged_state = controller_output.state;
        }

        if (transmit(controller_output.command)) {
            g_panel_heartbeat.fetch_add(1);
        } else {
            // Drop active intent after a transport failure. A stalled writer
            // remains covered by the independent heartbeat supervisor.
            portENTER_CRITICAL(&g_inputs_mux);
            g_inputs.stop();
            portEXIT_CRITICAL(&g_inputs_mux);
        }
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
        ESP_LOGE("desk.watchdog", "panel writer heartbeat missing: %u s",
                 missed_seconds);
        if (missed_seconds >= 3) {
            ESP_LOGE("desk.watchdog", "panel writer stalled; restarting ESP");
            esp_restart();
        }
    }
}

}  // namespace

extern "C" void app_main() {
    ESP_LOGW("smart_desk", "BLE CONTROLLER; original panel must be disconnected");
    ESP_LOGW("smart_desk", "SIT/STAND from NVS; limits=72..118 cm; no queue");
    ESP_LOGI("smart_desk", "GPIO17=TX; GPIO18=height RX");

    configure_uart(UART_NUM_1, kTxGpio, kRxGpio);
    ESP_ERROR_CHECK(transmit(smart_desk::Command::kIdle) ? ESP_OK : ESP_FAIL);
    // BLE initialization owns NVS initialization. Start the writer afterwards
    // so its persisted preset read cannot race nvs_flash_init().
    start_ble_hid_logger(ble_motion_action, nullptr, false);
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
