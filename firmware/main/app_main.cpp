#include <array>
#include <atomic>
#include <cstdint>

#include "desk_protocol.hpp"
#include "ble_hid_host.hpp"
#include "driver/gpio.h"
#include "uart_transport.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

namespace {

constexpr int kPanelToControllerRxGpio = 17;
constexpr int kControllerToPanelRxGpio = 18;
constexpr TickType_t kPanelPeriod = pdMS_TO_TICKS(100);

struct ReaderConfig {
    uart_port_t uart;
    int rx_gpio;
    smart_desk::Direction direction;
    const char* tag;
};

#if CONFIG_DESK_MODE_READ_ONLY
ReaderConfig kPanelReader{UART_NUM_1, kPanelToControllerRxGpio,
                          smart_desk::Direction::kPanelToController,
                          "desk.panel"};
ReaderConfig kControllerReader{UART_NUM_2, kControllerToPanelRxGpio,
                               smart_desk::Direction::kControllerToPanel,
                               "desk.controller"};
#elif CONFIG_DESK_MODE_IDLE_EMULATOR || \
    CONFIG_DESK_MODE_STEP_DOWN_TEST || \
    CONFIG_DESK_MODE_HOLD_DOWN_TEST || \
    CONFIG_DESK_MODE_BLE_HID_LOGGER || \
    CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || \
    CONFIG_DESK_MODE_BLE_CALIBRATION
ReaderConfig kEmulatorReader{UART_NUM_1, kControllerToPanelRxGpio,
                             smart_desk::Direction::kControllerToPanel,
                             "desk.controller"};
#endif

#if CONFIG_DESK_MODE_STEP_DOWN_TEST || \
    CONFIG_DESK_MODE_HOLD_DOWN_TEST || \
    CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || \
    CONFIG_DESK_MODE_BLE_CALIBRATION
constexpr std::uint16_t kTestMinHeightTenthsCm = 1100;
constexpr std::uint16_t kTestMaxHeightTenthsCm = 1200;
constexpr TickType_t kTelemetryMaxAge = pdMS_TO_TICKS(500);
constexpr std::uint32_t kConsistentHeightFrameCap = 5;
std::atomic<std::uint16_t> g_height_tenths_cm{0};
std::atomic<TickType_t> g_height_tick{0};
std::atomic<bool> g_have_height{false};
std::atomic<std::uint32_t> g_consistent_height_frames{0};
std::atomic<std::uint32_t> g_anomaly_sequence{0};
#endif

#if CONFIG_DESK_MODE_STEP_DOWN_TEST
std::atomic<bool> g_step_requested{false};
std::atomic<bool> g_step_used{false};
#endif

#if CONFIG_DESK_MODE_HOLD_DOWN_TEST
constexpr TickType_t kHoldMaxDuration = pdMS_TO_TICKS(2500);
constexpr TickType_t kHoldStartGrace = pdMS_TO_TICKS(1200);
constexpr TickType_t kHoldStallTimeout = pdMS_TO_TICKS(500);
constexpr std::uint16_t kHoldMaxTravelTenthsCm = 30;
std::atomic<bool> g_boot_pressed{false};
#endif

#if CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || CONFIG_DESK_MODE_BLE_CALIBRATION
constexpr std::uint16_t kBleTestMinHeightTenthsCm = 750;
constexpr std::uint16_t kBleTestMaxHeightTenthsCm = 1150;
constexpr TickType_t kBleTelemetryMaxAge = pdMS_TO_TICKS(300);
constexpr std::uint32_t kBleRequiredConsistentFrames = 3;
constexpr TickType_t kBleTestMaxDuration = pdMS_TO_TICKS(2500);
constexpr TickType_t kBleTestStartGrace = pdMS_TO_TICKS(1200);
constexpr TickType_t kBleTestStallTimeout = pdMS_TO_TICKS(600);
constexpr std::uint16_t kBleTestCommandedTravelLimitTenthsCm = 20;
// Moving telemetry has a measured sawtooth of at least 0.5 cm, plausibly
// because the dual-motor controller alternates two column positions.  Do not
// treat one such sample as reversal; raw opposite dial input still requests
// an immediate stop independently of this guard.
constexpr std::uint16_t kBleTelemetryReversalLimitTenthsCm = 10;
std::atomic<int> g_ble_pending_delta_cm{0};
std::atomic<bool> g_ble_stop_requested{false};
std::atomic<int> g_ble_active_direction{0};
std::atomic<bool> g_ble_test_used{false};
#if CONFIG_DESK_MODE_BLE_CALIBRATION
std::atomic<bool> g_ble_accept_input{true};
std::atomic<TickType_t> g_ble_last_raw_tick{0};
#endif

void ble_motion_action(const BleHidAction& action, void*) {
    switch (action.type) {
        case BleHidActionType::kDialRaw: {
#if CONFIG_DESK_MODE_BLE_CALIBRATION
            g_ble_last_raw_tick.store(xTaskGetTickCount());
#endif
            const int active_direction = g_ble_active_direction.load();
            if (active_direction != 0 && action.value != active_direction) {
                // This flag bypasses any event queue. The next 100 ms UART
                // slot is guaranteed to be IDLE.
                g_ble_stop_requested.store(true);
            }
            break;
        }
        case BleHidActionType::kDialDelta:
#if CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST
            if (action.value == 1 || action.value == -1) {
                // Replacement slot, deliberately not an accumulating queue.
                g_ble_pending_delta_cm.store(action.value);
            } else {
                ESP_LOGW("desk.ble_test",
                         "rejected fast dial delta=%+d; one-shot accepts +/-1",
                         action.value);
            }
#elif CONFIG_DESK_MODE_BLE_CALIBRATION
            if (!g_ble_accept_input.load()) {
                ESP_LOGW("desk.cal", "dial delta=%+d discarded: busy",
                         action.value);
            } else if (action.value == 1 || action.value == -1 ||
                       action.value == 3 || action.value == -3) {
                // One replacement slot, never an accumulating movement queue.
                g_ble_pending_delta_cm.store(action.value);
            }
#else
            // Production mode consumes raw detents immediately. The delayed
            // burst classifier is disabled, so no turn count can become a
            // movement queue.
#endif
            break;
        case BleHidActionType::kStop:
        case BleHidActionType::kDisconnected:
            g_ble_stop_requested.store(true);
            g_ble_pending_delta_cm.store(0);
            break;
        case BleHidActionType::kGoSit:
        case BleHidActionType::kGoStand:
        case BleHidActionType::kSaveSit:
        case BleHidActionType::kSaveStand:
            ESP_LOGW("desk.ble_test",
                     "preset action ignored in one-shot motion test");
            break;
        case BleHidActionType::kUserActivity:
        case BleHidActionType::kStartCalibration:
            // Recovery is a production-only control path. Lab motion modes
            // deliberately ignore its internal BLE notifications.
            break;
    }
}
#endif

void log_frame(const ReaderConfig& config, const smart_desk::Frame& frame) {
    if (config.direction == smart_desk::Direction::kPanelToController) {
        const auto message = smart_desk::decode_panel_frame(frame);
        if (message.has_value()) {
            ESP_LOGI(config.tag, "command=%s code=0x%02X status=0x%02X",
                     smart_desk::command_name(message->command_code),
                     message->command_code, message->status);
        }
        return;
    }

    const auto message = smart_desk::decode_controller_frame(frame);
    ESP_LOG_BUFFER_HEX_LEVEL(config.tag, frame.data(), frame.size(),
                             ESP_LOG_INFO);
    if (message.has_value()) {
        ESP_LOGI(config.tag, "height=%u.%u cm flags=0x%02X status=0x%02X",
                 message->height_tenths_cm / 10,
                 message->height_tenths_cm % 10, message->flags,
                 message->status);
    } else {
        ESP_LOGI(config.tag, "valid non-height frame type=0x%02X", frame[3]);
    }
}

void reader_task(void* raw_config) {
    const auto& config = *static_cast<const ReaderConfig*>(raw_config);
    smart_desk::StreamParser parser(config.direction);
    smart_desk::Frame last_frame{};
    bool have_last_frame = false;
    std::array<std::uint8_t, 64> bytes{};

    while (true) {
        const int received = uart_read_bytes(config.uart, bytes.data(),
                                             bytes.size(), pdMS_TO_TICKS(100));
        for (int index = 0; index < received; ++index) {
            const auto frame = parser.feed(bytes[static_cast<std::size_t>(index)]);
            if (!frame.has_value()) {
                continue;
            }

#if CONFIG_DESK_MODE_STEP_DOWN_TEST || \
    CONFIG_DESK_MODE_HOLD_DOWN_TEST || \
    CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || \
    CONFIG_DESK_MODE_BLE_CALIBRATION
            if (config.direction ==
                smart_desk::Direction::kControllerToPanel) {
                const auto message =
                    smart_desk::decode_controller_frame(*frame);
                // По штатным захватам status=0x01 используется в покое и при
                // движении вверх, а status=0x02 — при движении вниз.
                const bool normal_status =
                    message.has_value() &&
                    (message->status == 0x01 || message->status == 0x02);
                const bool normal_height =
                    normal_status && message->flags == 0x00 &&
                    message->height_tenths_cm >= 700 &&
                    message->height_tenths_cm <= 1300;
                if (normal_height) {
                    const std::uint16_t previous_height =
                        g_height_tenths_cm.load();
                    // During motion valid samples often advance by 0.1 cm and
                    // can sawtooth by about 0.5 cm.  Only identical consecutive
                    // heights count as settled.
                    const bool consistent =
                        g_have_height.load() &&
                        message->height_tenths_cm == previous_height;
                    if (consistent) {
                        const std::uint32_t old_count =
                            g_consistent_height_frames.load();
                        if (old_count < kConsistentHeightFrameCap) {
                            g_consistent_height_frames.store(old_count + 1);
                        }
                    } else {
                        g_consistent_height_frames.store(1);
                    }
                    g_height_tenths_cm.store(message->height_tenths_cm);
                    g_height_tick.store(xTaskGetTickCount());
                    g_have_height.store(true);
                } else {
                    g_consistent_height_frames.store(0);
                    g_anomaly_sequence.fetch_add(1);
                }
            }
#endif

            // На неподвижном столе одинаковые кадры идут около 10 раз/с.
            // Для читаемого первого лога печатаем только изменения.
            if (!have_last_frame || *frame != last_frame) {
                log_frame(config, *frame);
                last_frame = *frame;
                have_last_frame = true;
            }
        }
    }
}

#if CONFIG_DESK_MODE_IDLE_EMULATOR || \
    CONFIG_DESK_MODE_STEP_DOWN_TEST || \
    CONFIG_DESK_MODE_HOLD_DOWN_TEST || \
    CONFIG_DESK_MODE_BLE_HID_LOGGER || \
    CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || \
    CONFIG_DESK_MODE_BLE_CALIBRATION
void panel_writer_task(void*) {
    const auto idle = smart_desk::make_panel_frame(smart_desk::Command::kIdle);
#if CONFIG_DESK_MODE_STEP_DOWN_TEST
    const auto down_step =
        smart_desk::make_panel_frame(smart_desk::Command::kDownStep);
#endif
#if CONFIG_DESK_MODE_HOLD_DOWN_TEST
    const auto down_hold =
        smart_desk::make_panel_frame(smart_desk::Command::kDownHold);
    bool active = false;
    bool locked = false;
    bool have_progress = false;
    std::uint16_t start_height = 0;
    std::uint16_t lowest_height = 0;
    TickType_t start_tick = 0;
    TickType_t progress_tick = 0;
    std::uint32_t start_anomaly_sequence = 0;
#endif
#if CONFIG_DESK_MODE_BLE_CALIBRATION
    const auto down_step =
        smart_desk::make_panel_frame(smart_desk::Command::kDownStep);
    const auto up_step =
        smart_desk::make_panel_frame(smart_desk::Command::kUpStep);
    const auto down_hold =
        smart_desk::make_panel_frame(smart_desk::Command::kDownHold);
    const auto up_hold =
        smart_desk::make_panel_frame(smart_desk::Command::kUpHold);
    constexpr std::uint16_t kUpCoastTenthsCm = 11;
    constexpr std::uint16_t kDownCoastTenthsCm = 18;
    constexpr std::uint32_t kSettledFrames = 5;
    constexpr TickType_t kCalibrationMaxHold = pdMS_TO_TICKS(4000);
    constexpr TickType_t kCalibrationStartGrace = pdMS_TO_TICKS(1200);
    constexpr TickType_t kCalibrationStall = pdMS_TO_TICKS(700);
    constexpr std::uint16_t kCalibrationMaxCommandedTravelTenthsCm = 50;
    constexpr TickType_t kCalibrationSettleMin = pdMS_TO_TICKS(1500);
    constexpr TickType_t kCalibrationNoMotionSettle = pdMS_TO_TICKS(3000);
    constexpr TickType_t kCalibrationSettleWatchdog = pdMS_TO_TICKS(7000);
    constexpr TickType_t kCalibrationInputQuiet = pdMS_TO_TICKS(500);
    bool hold_active = false;
    bool settling = false;
    bool have_progress = false;
    bool moved = false;
    int direction = 0;
    int requested_delta_cm = 0;
    std::uint16_t start_height = 0;
    std::uint16_t target_height = 0;
    std::uint16_t progress_height = 0;
    TickType_t motion_start_tick = 0;
    TickType_t progress_tick = 0;
    TickType_t settle_start_tick = 0;
    std::uint32_t start_anomaly_sequence = 0;
#endif
#if CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST
    const auto down_hold =
        smart_desk::make_panel_frame(smart_desk::Command::kDownHold);
    const auto up_hold =
        smart_desk::make_panel_frame(smart_desk::Command::kUpHold);
    bool active = false;
    bool locked = false;
    bool have_progress = false;
    int direction = 0;
    std::uint16_t start_height = 0;
    std::uint16_t target_height = 0;
    std::uint16_t progress_height = 0;
    TickType_t start_tick = 0;
    TickType_t progress_tick = 0;
    std::uint32_t start_anomaly_sequence = 0;
#endif
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        const smart_desk::Frame* outgoing = &idle;

#if CONFIG_DESK_MODE_STEP_DOWN_TEST
        if (g_step_requested.exchange(false) && !g_step_used.load()) {
            const TickType_t now = xTaskGetTickCount();
            const TickType_t last_height = g_height_tick.load();
            const std::uint16_t height = g_height_tenths_cm.load();
            const bool fresh = g_have_height.load() &&
                               (now - last_height <= kTelemetryMaxAge);
            const bool safe_height = height >= kTestMinHeightTenthsCm &&
                                     height <= kTestMaxHeightTenthsCm;

            if (fresh && safe_height) {
                outgoing = &down_step;
                g_step_used.store(true);
                ESP_LOGW("desk.test",
                         "TX one DOWN_STEP at height=%u.%u cm; now locked",
                         height / 10, height % 10);
            } else {
                ESP_LOGE("desk.test",
                         "BLOCKED: fresh=%d height=%u.%u cm",
                         fresh, height / 10, height % 10);
            }
        }
#endif

#if CONFIG_DESK_MODE_BLE_CALIBRATION
        const TickType_t cal_now = xTaskGetTickCount();
        const std::uint16_t cal_height = g_height_tenths_cm.load();
        const bool cal_fresh =
            g_have_height.load() &&
            (cal_now - g_height_tick.load() <= kBleTelemetryMaxAge);
        const bool cal_settled =
            g_consistent_height_frames.load() >= kSettledFrames;

        if (!hold_active && !settling && g_ble_accept_input.load()) {
            const int delta_cm = g_ble_pending_delta_cm.exchange(0);
            if (delta_cm != 0) {
                const int requested_target =
                    static_cast<int>(cal_height) + delta_cm * 10;
                const bool safe_start =
                    cal_fresh && cal_settled &&
                    cal_height >= kBleTestMinHeightTenthsCm &&
                    cal_height <= kBleTestMaxHeightTenthsCm;
                const bool safe_target =
                    requested_target >= kBleTestMinHeightTenthsCm &&
                    requested_target <= kBleTestMaxHeightTenthsCm;

                if (!safe_start || !safe_target) {
                    ESP_LOGE("desk.cal",
                             "BLOCKED: fresh=%d settled=%d height=%u.%u target=%d.%d",
                             cal_fresh, cal_settled, cal_height / 10,
                             cal_height % 10, requested_target / 10,
                             requested_target >= 0 ? requested_target % 10 : 0);
                } else {
                    direction = delta_cm > 0 ? 1 : -1;
                    requested_delta_cm = delta_cm;
                    start_height = cal_height;
                    target_height =
                        static_cast<std::uint16_t>(requested_target);
                    progress_height = cal_height;
                    motion_start_tick = cal_now;
                    progress_tick = cal_now;
                    have_progress = false;
                    moved = false;
                    start_anomaly_sequence = g_anomaly_sequence.load();
                    g_ble_active_direction.store(direction);
                    g_ble_stop_requested.store(false);
                    g_ble_accept_input.store(false);

                    if (delta_cm == 1 || delta_cm == -1) {
                        outgoing = direction > 0 ? &up_step : &down_step;
                        settling = true;
                        settle_start_tick = cal_now;
                        ESP_LOGW("desk.cal",
                                 "STEP start: %u.%u -> %u.%u cm dir=%+d",
                                 start_height / 10, start_height % 10,
                                 target_height / 10, target_height % 10,
                                 direction);
                    } else {
                        hold_active = true;
                        ESP_LOGW("desk.cal",
                                 "HOLD start: %u.%u -> %u.%u cm dir=%+d coast=%u.%u",
                                 start_height / 10, start_height % 10,
                                 target_height / 10, target_height % 10,
                                 direction,
                                 (direction > 0 ? kUpCoastTenthsCm
                                                : kDownCoastTenthsCm) /
                                     10,
                                 (direction > 0 ? kUpCoastTenthsCm
                                                : kDownCoastTenthsCm) %
                                     10);
                    }
                }
            }
        }

        if (hold_active) {
            const char* stop_reason = nullptr;
            const bool progressed =
                direction > 0 ? cal_height > progress_height
                              : cal_height < progress_height;
            if (progressed) {
                progress_height = cal_height;
                progress_tick = cal_now;
                have_progress = true;
                moved = true;
            }

            const std::uint16_t coast =
                direction > 0 ? kUpCoastTenthsCm : kDownCoastTenthsCm;
            const std::uint16_t brake_height =
                direction > 0 ? target_height - coast
                              : target_height + coast;
            const bool brake_point_reached =
                direction > 0 ? progress_height >= brake_height
                              : progress_height <= brake_height;
            const bool telemetry_reversed =
                direction > 0
                    ? cal_height + kBleTelemetryReversalLimitTenthsCm <=
                          progress_height
                    : cal_height >= progress_height +
                          kBleTelemetryReversalLimitTenthsCm;
            const std::uint16_t commanded_travel =
                direction > 0
                    ? (progress_height >= start_height
                           ? progress_height - start_height
                           : 0)
                    : (start_height >= progress_height
                           ? start_height - progress_height
                           : 0);

            if (g_ble_stop_requested.load()) {
                stop_reason = "BLE STOP/reversal/disconnect";
            } else if (!cal_fresh) {
                stop_reason = "telemetry stale";
            } else if (g_anomaly_sequence.load() != start_anomaly_sequence) {
                stop_reason = "unexpected controller frame";
            } else if (cal_now - motion_start_tick >= kCalibrationMaxHold) {
                stop_reason = "4.0 s watchdog";
            } else if (commanded_travel >=
                       kCalibrationMaxCommandedTravelTenthsCm) {
                stop_reason = "5.0 cm commanded-travel guard";
            } else if (telemetry_reversed) {
                stop_reason = "telemetry reversed >=1.0 cm";
            } else if (brake_point_reached) {
                stop_reason = "coast-compensated brake point";
            } else if (have_progress &&
                       cal_now - progress_tick >= kCalibrationStall) {
                stop_reason = "height stalled";
            } else if (!have_progress &&
                       cal_now - motion_start_tick >=
                           kCalibrationStartGrace) {
                stop_reason = "no initial movement";
            }

            if (stop_reason == nullptr) {
                outgoing = direction > 0 ? &up_hold : &down_hold;
            } else {
                hold_active = false;
                settling = true;
                settle_start_tick = cal_now;
                g_ble_pending_delta_cm.store(0);
                ESP_LOGW("desk.cal",
                         "HOLD -> IDLE: %s at %u.%u cm; waiting for settle",
                         stop_reason, cal_height / 10, cal_height % 10);
            }
        }

        if (settling) {
            const bool directional_motion =
                direction > 0 ? cal_height >= start_height + 2
                              : start_height >= cal_height + 2;
            moved = moved || directional_motion;
            const bool enough_time =
                cal_now - settle_start_tick >= kCalibrationSettleMin;
            const bool no_motion_timeout =
                cal_now - motion_start_tick >= kCalibrationNoMotionSettle;
            const bool input_quiet =
                cal_now - g_ble_last_raw_tick.load() >=
                kCalibrationInputQuiet;
            const bool finished =
                cal_fresh && cal_settled && enough_time && input_quiet &&
                (moved || no_motion_timeout);
            const bool settle_watchdog =
                cal_now - settle_start_tick >= kCalibrationSettleWatchdog;

            if (finished || settle_watchdog) {
                const int actual_delta_tenths =
                    static_cast<int>(cal_height) -
                    static_cast<int>(start_height);
                const unsigned actual_delta_abs =
                    actual_delta_tenths >= 0
                        ? static_cast<unsigned>(actual_delta_tenths)
                        : static_cast<unsigned>(-actual_delta_tenths);
                ESP_LOGW("desk.cal",
                         "RUN complete: requested=%+d cm actual=%c%u.%u cm final=%u.%u%s",
                         requested_delta_cm,
                         actual_delta_tenths >= 0 ? '+' : '-',
                         actual_delta_abs / 10, actual_delta_abs % 10,
                         cal_height / 10, cal_height % 10,
                         settle_watchdog ? " (settle watchdog)" : "");
                settling = false;
                direction = 0;
                g_ble_active_direction.store(0);
                g_ble_stop_requested.store(false);
                g_ble_pending_delta_cm.store(0);
                g_ble_accept_input.store(true);
                ESP_LOGI("desk.cal", "READY for next separated gesture");
            }
        }
#endif

#if CONFIG_DESK_MODE_HOLD_DOWN_TEST
        const TickType_t now = xTaskGetTickCount();
        const std::uint16_t height = g_height_tenths_cm.load();
        const bool fresh = g_have_height.load() &&
                           (now - g_height_tick.load() <= kTelemetryMaxAge);

        if (!active && !locked && g_boot_pressed.load()) {
            const bool safe_height = height >= kTestMinHeightTenthsCm &&
                                     height <= kTestMaxHeightTenthsCm;
            if (fresh && safe_height) {
                active = true;
                start_height = height;
                lowest_height = height;
                start_tick = now;
                progress_tick = now;
                start_anomaly_sequence = g_anomaly_sequence.load();
                ESP_LOGW("desk.test",
                         "DOWN_HOLD started at %u.%u cm (one run only)",
                         height / 10, height % 10);
            } else {
                ESP_LOGE("desk.test",
                         "BLOCKED start: fresh=%d height=%u.%u cm",
                         fresh, height / 10, height % 10);
            }
        }

        if (active) {
            const char* stop_reason = nullptr;
            if (!g_boot_pressed.load()) {
                stop_reason = "BOOT released";
            } else if (!fresh) {
                stop_reason = "telemetry stale";
            } else if (g_anomaly_sequence.load() != start_anomaly_sequence) {
                stop_reason = "unexpected controller frame";
            } else if (now - start_tick >= kHoldMaxDuration) {
                stop_reason = "2.5 s watchdog";
            } else if (start_height >= height &&
                       start_height - height >= kHoldMaxTravelTenthsCm) {
                stop_reason = "3.0 cm travel limit";
            } else if (height > lowest_height + 2) {
                stop_reason = "direction reversal";
            } else {
                if (height < lowest_height) {
                    lowest_height = height;
                    progress_tick = now;
                    have_progress = true;
                }
                if (have_progress &&
                    now - progress_tick >= kHoldStallTimeout) {
                    stop_reason = "height stalled";
                } else if (!have_progress &&
                           now - start_tick >= kHoldStartGrace) {
                    stop_reason = "no initial movement";
                }
            }

            if (stop_reason == nullptr) {
                outgoing = &down_hold;
            } else {
                active = false;
                locked = true;
                ESP_LOGW("desk.test",
                         "DOWN_HOLD stopped: %s at %u.%u cm; now locked",
                         stop_reason, height / 10, height % 10);
            }
        }
#endif

#if CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST
        const TickType_t now = xTaskGetTickCount();
        const std::uint16_t height = g_height_tenths_cm.load();
        const bool fresh = g_have_height.load() &&
                           (now - g_height_tick.load() <= kBleTelemetryMaxAge);
        const bool settled_before_start =
            g_consistent_height_frames.load() >=
            kBleRequiredConsistentFrames;

        if (!active && !locked && !g_ble_test_used.load()) {
            const int delta_cm = g_ble_pending_delta_cm.exchange(0);
            if (delta_cm != 0) {
                const int requested_target =
                    static_cast<int>(height) + delta_cm * 10;
                const bool safe_start =
                    fresh && settled_before_start &&
                    height >= kBleTestMinHeightTenthsCm &&
                    height <= kBleTestMaxHeightTenthsCm;
                const bool safe_target =
                    requested_target >= kBleTestMinHeightTenthsCm &&
                    requested_target <= kBleTestMaxHeightTenthsCm;
                if (safe_start && safe_target) {
                    active = true;
                    direction = delta_cm > 0 ? 1 : -1;
                    start_height = height;
                    target_height =
                        static_cast<std::uint16_t>(requested_target);
                    progress_height = height;
                    start_tick = now;
                    progress_tick = now;
                    have_progress = false;
                    start_anomaly_sequence = g_anomaly_sequence.load();
                    g_ble_active_direction.store(direction);
                    g_ble_stop_requested.store(false);
                    g_ble_test_used.store(true);
                    ESP_LOGW("desk.ble_test",
                             "ONE-SHOT start: %u.%u -> %u.%u cm dir=%+d",
                             start_height / 10, start_height % 10,
                             target_height / 10, target_height % 10,
                             direction);
                } else {
                    ESP_LOGE("desk.ble_test",
                             "BLOCKED: fresh=%d settled=%d height=%u.%u target=%d.%d",
                             fresh, settled_before_start, height / 10,
                             height % 10,
                             requested_target / 10,
                             requested_target >= 0 ? requested_target % 10 : 0);
                }
            }
        }

        if (active) {
            const char* stop_reason = nullptr;
            const std::uint16_t commanded_travel =
                direction > 0
                    ? (height >= start_height ? height - start_height : 0)
                    : (start_height >= height ? start_height - height : 0);
            const bool target_reached =
                direction > 0 ? height >= target_height
                              : height <= target_height;
            const bool telemetry_reversed =
                direction > 0
                    ? height + kBleTelemetryReversalLimitTenthsCm <=
                          progress_height
                    : height >= progress_height +
                          kBleTelemetryReversalLimitTenthsCm;
            const bool progressed =
                direction > 0 ? height > progress_height
                              : height < progress_height;

            if (g_ble_stop_requested.load()) {
                stop_reason = "BLE STOP/reversal/disconnect";
            } else if (!fresh) {
                stop_reason = "telemetry stale";
            } else if (g_anomaly_sequence.load() != start_anomaly_sequence) {
                stop_reason = "unexpected controller frame";
            } else if (now - start_tick >= kBleTestMaxDuration) {
                stop_reason = "2.5 s watchdog";
            } else if (commanded_travel >=
                       kBleTestCommandedTravelLimitTenthsCm) {
                stop_reason = "2.0 cm commanded-travel guard";
            } else if (telemetry_reversed) {
                stop_reason = "telemetry direction reversal";
            } else if (target_reached) {
                stop_reason = "1.0 cm target reached";
            } else {
                if (progressed) {
                    progress_height = height;
                    progress_tick = now;
                    have_progress = true;
                }
                if (have_progress &&
                    now - progress_tick >= kBleTestStallTimeout) {
                    stop_reason = "height stalled";
                } else if (!have_progress &&
                           now - start_tick >= kBleTestStartGrace) {
                    stop_reason = "no initial movement";
                }
            }

            if (stop_reason == nullptr) {
                outgoing = direction > 0 ? &up_hold : &down_hold;
            } else {
                active = false;
                locked = true;
                g_ble_active_direction.store(0);
                g_ble_pending_delta_cm.store(0);
                ESP_LOGW("desk.ble_test",
                         "ONE-SHOT stop: %s at %u.%u cm; IDLE locked",
                         stop_reason, height / 10, height % 10);
            }
        }
#endif

        const int written = uart_write_bytes(UART_NUM_1, outgoing->data(),
                                             outgoing->size());
        if (written != static_cast<int>(outgoing->size())) {
            ESP_LOGE("desk.idle", "UART write failed: %d/8 bytes", written);
        }
        xTaskDelayUntil(&last_wake, kPanelPeriod);
    }
}
#endif

#if CONFIG_DESK_MODE_STEP_DOWN_TEST || CONFIG_DESK_MODE_HOLD_DOWN_TEST
void boot_button_task(void*) {
    constexpr gpio_num_t kBootButton = GPIO_NUM_0;
    ESP_ERROR_CHECK(gpio_reset_pin(kBootButton));
    ESP_ERROR_CHECK(gpio_set_direction(kBootButton, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(kBootButton, GPIO_PULLUP_ONLY));

    bool armed = true;
    while (true) {
        const bool pressed = gpio_get_level(kBootButton) == 0;
        if (pressed && armed) {
            armed = false;
#if CONFIG_DESK_MODE_STEP_DOWN_TEST
            if (!g_step_used.load()) {
                g_step_requested.store(true);
                ESP_LOGI("desk.test", "BOOT press: DOWN_STEP requested");
            } else {
                ESP_LOGW("desk.test", "BOOT press ignored: test already used");
            }
#elif CONFIG_DESK_MODE_HOLD_DOWN_TEST
            g_boot_pressed.store(true);
            ESP_LOGI("desk.test", "BOOT pressed");
#endif
        } else if (!pressed) {
#if CONFIG_DESK_MODE_HOLD_DOWN_TEST
            if (!armed) {
                ESP_LOGI("desk.test", "BOOT released");
            }
            g_boot_pressed.store(false);
#endif
            armed = true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

}  // namespace

extern "C" void app_main() {
#if CONFIG_DESK_MODE_READ_ONLY
    ESP_LOGI("smart_desk", "read-only sniffer; GPIO17/GPIO18 are RX inputs");

    configure_uart(kPanelReader.uart, UART_PIN_NO_CHANGE, kPanelReader.rx_gpio);
    configure_uart(kControllerReader.uart, UART_PIN_NO_CHANGE,
                   kControllerReader.rx_gpio);

    xTaskCreate(reader_task, "panel_rx", 4096, &kPanelReader, 5, nullptr);
    xTaskCreate(reader_task, "controller_rx", 4096, &kControllerReader, 5,
                nullptr);
#elif CONFIG_DESK_MODE_IDLE_EMULATOR
    ESP_LOGW("smart_desk",
             "IDLE-ONLY emulator; original panel must be disconnected");
    ESP_LOGI("smart_desk", "GPIO17=TX IDLE only; GPIO18=height RX");

    configure_uart(UART_NUM_1, kPanelToControllerRxGpio,
                   kControllerToPanelRxGpio);
    xTaskCreate(reader_task, "controller_rx", 4096, &kEmulatorReader, 5,
                nullptr);
    xTaskCreate(panel_writer_task, "idle_tx", 3072, nullptr, 5, nullptr);
#elif CONFIG_DESK_MODE_STEP_DOWN_TEST
    ESP_LOGW("smart_desk",
             "ONE-SHOT DOWN test; original panel must be disconnected");
    ESP_LOGI("smart_desk",
             "GPIO17=TX; GPIO18=height RX; BOOT sends at most one DOWN_STEP");

    configure_uart(UART_NUM_1, kPanelToControllerRxGpio,
                   kControllerToPanelRxGpio);
    xTaskCreate(reader_task, "controller_rx", 4096, &kEmulatorReader, 5,
                nullptr);
    xTaskCreate(panel_writer_task, "panel_tx", 3072, nullptr, 5, nullptr);
    xTaskCreate(boot_button_task, "boot_button", 3072, nullptr, 4, nullptr);
#elif CONFIG_DESK_MODE_HOLD_DOWN_TEST
    ESP_LOGW("smart_desk",
             "WATCHDOG HOLD-DOWN test; original panel must be disconnected");
    ESP_LOGI("smart_desk",
             "Hold BOOT for DOWN; release/stale/stall/reversal/time/travel stop");

    configure_uart(UART_NUM_1, kPanelToControllerRxGpio,
                   kControllerToPanelRxGpio);
    xTaskCreate(reader_task, "controller_rx", 4096, &kEmulatorReader, 5,
                nullptr);
    xTaskCreate(panel_writer_task, "panel_tx", 4096, nullptr, 5, nullptr);
    xTaskCreate(boot_button_task, "boot_button", 3072, nullptr, 4, nullptr);
#elif CONFIG_DESK_MODE_BLE_HID_LOGGER
    ESP_LOGW("smart_desk",
             "BLE HID LOGGER; BLE reports cannot generate desk commands");
    ESP_LOGI("smart_desk", "GPIO17=TX IDLE only; GPIO18=height RX");

    configure_uart(UART_NUM_1, kPanelToControllerRxGpio,
                   kControllerToPanelRxGpio);
    xTaskCreate(reader_task, "controller_rx", 4096, &kEmulatorReader, 5,
                nullptr);
    xTaskCreate(panel_writer_task, "idle_tx", 3072, nullptr, 5, nullptr);
    start_ble_hid_host();
#elif CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST
    ESP_LOGW("smart_desk",
             "BLE ONE-SHOT MOTION TEST; original panel must be disconnected");
    ESP_LOGW("smart_desk",
             "one slow dial gesture only; presets/fast dial rejected");
    ESP_LOGI("smart_desk", "GPIO17=TX; GPIO18=height RX");

    configure_uart(UART_NUM_1, kPanelToControllerRxGpio,
                   kControllerToPanelRxGpio);
    xTaskCreate(reader_task, "controller_rx", 4096, &kEmulatorReader, 5,
                nullptr);
    xTaskCreate(panel_writer_task, "panel_tx", 4096, nullptr, 5, nullptr);
    start_ble_hid_host(ble_motion_action, nullptr);
#elif CONFIG_DESK_MODE_BLE_CALIBRATION
    ESP_LOGW("smart_desk",
             "BLE MULTI-RUN CALIBRATION; original panel must be disconnected");
    ESP_LOGW("smart_desk",
             "slow dial=STEP 1 cm; fast dial=coast-compensated HOLD 3 cm");
    ESP_LOGI("smart_desk", "GPIO17=TX; GPIO18=height RX");

    configure_uart(UART_NUM_1, kPanelToControllerRxGpio,
                   kControllerToPanelRxGpio);
    xTaskCreate(reader_task, "controller_rx", 4096, &kEmulatorReader, 5,
                nullptr);
    xTaskCreate(panel_writer_task, "panel_tx", 5120, nullptr, 5, nullptr);
    start_ble_hid_host(ble_motion_action, nullptr);
#else
#error "Select a smart desk firmware mode"
#endif
}
