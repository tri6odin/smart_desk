#include <array>
#include <atomic>
#include <cstdint>

#include "desk_protocol.hpp"
#include "ble_hid_logger.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "sdkconfig.h"

namespace {

constexpr int kPanelToControllerRxGpio = 17;
constexpr int kControllerToPanelRxGpio = 18;
constexpr int kBaudRate = 9600;
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
#elif CONFIG_DESK_MODE_IDLE_EMULATOR || CONFIG_DESK_MODE_STEP_DOWN_TEST || \
    CONFIG_DESK_MODE_HOLD_DOWN_TEST || CONFIG_DESK_MODE_BLE_HID_LOGGER || \
    CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || CONFIG_DESK_MODE_BLE_CALIBRATION || \
    CONFIG_DESK_MODE_BLE_CONTROLLER
ReaderConfig kEmulatorReader{UART_NUM_1, kControllerToPanelRxGpio,
                             smart_desk::Direction::kControllerToPanel,
                             "desk.controller"};
#endif

#if CONFIG_DESK_MODE_STEP_DOWN_TEST || CONFIG_DESK_MODE_HOLD_DOWN_TEST || \
    CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || CONFIG_DESK_MODE_BLE_CALIBRATION || \
    CONFIG_DESK_MODE_BLE_CONTROLLER
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

#if CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || CONFIG_DESK_MODE_BLE_CALIBRATION || \
    CONFIG_DESK_MODE_BLE_CONTROLLER
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

#if CONFIG_DESK_MODE_BLE_CONTROLLER
enum class ControllerRequest : int {
    kNone,
    kGoSit,
    kGoStand,
    kSaveSit,
    kSaveStand,
};

std::atomic<std::uint32_t> g_ble_raw_sequence{0};
std::atomic<int> g_ble_raw_direction{0};
std::atomic<TickType_t> g_ble_raw_tick{0};
std::atomic<ControllerRequest> g_controller_request{ControllerRequest::kNone};
#endif

void ble_motion_action(const BleHidAction& action, void*) {
    switch (action.type) {
        case BleHidActionType::kDialRaw: {
#if CONFIG_DESK_MODE_BLE_CALIBRATION
            g_ble_last_raw_tick.store(xTaskGetTickCount());
#elif CONFIG_DESK_MODE_BLE_CONTROLLER
            g_ble_raw_tick.store(xTaskGetTickCount());
            g_ble_raw_direction.store(action.value);
            g_ble_raw_sequence.fetch_add(1);
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
#if CONFIG_DESK_MODE_BLE_CONTROLLER
            g_controller_request.store(ControllerRequest::kNone);
#endif
            break;
        case BleHidActionType::kGoSit:
#if CONFIG_DESK_MODE_BLE_CONTROLLER
            if (g_ble_active_direction.load() == 0) {
                g_controller_request.store(ControllerRequest::kGoSit);
            } else {
                g_ble_stop_requested.store(true);
            }
            break;
#endif
        case BleHidActionType::kGoStand:
#if CONFIG_DESK_MODE_BLE_CONTROLLER
            if (g_ble_active_direction.load() == 0) {
                g_controller_request.store(ControllerRequest::kGoStand);
            } else {
                g_ble_stop_requested.store(true);
            }
            break;
#endif
        case BleHidActionType::kSaveSit:
#if CONFIG_DESK_MODE_BLE_CONTROLLER
            g_controller_request.store(ControllerRequest::kSaveSit);
            break;
#endif
        case BleHidActionType::kSaveStand:
#if CONFIG_DESK_MODE_BLE_CONTROLLER
            g_controller_request.store(ControllerRequest::kSaveStand);
            break;
#else
            ESP_LOGW("desk.ble_test",
                     "preset action ignored in one-shot motion test");
            break;
#endif
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

#if CONFIG_DESK_MODE_STEP_DOWN_TEST || CONFIG_DESK_MODE_HOLD_DOWN_TEST || \
    CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || CONFIG_DESK_MODE_BLE_CALIBRATION || \
    CONFIG_DESK_MODE_BLE_CONTROLLER
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

void configure_uart(uart_port_t uart, int tx_gpio, int rx_gpio) {
    const uart_config_t uart_config{
        .baud_rate = kBaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .rx_glitch_filt_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    ESP_ERROR_CHECK(uart_driver_install(uart, 1024, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart, tx_gpio, rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(gpio_set_pull_mode(static_cast<gpio_num_t>(rx_gpio),
                                       GPIO_FLOATING));
}

#if CONFIG_DESK_MODE_BLE_CONTROLLER
struct DeskPresets {
    std::uint16_t sit = 800;
    std::uint16_t stand = 1150;
};

constexpr std::uint16_t kPhysicalMinTenthsCm = 700;
constexpr std::uint16_t kPhysicalMaxTenthsCm = 1200;
constexpr std::uint16_t kMinimumPresetGapTenthsCm = 100;

bool valid_presets(const DeskPresets& presets) {
    return presets.sit >= kPhysicalMinTenthsCm &&
           presets.stand <= kPhysicalMaxTenthsCm &&
           presets.sit + kMinimumPresetGapTenthsCm <= presets.stand;
}

DeskPresets load_presets() {
    DeskPresets presets;
    nvs_handle_t handle = 0;
    if (nvs_open("desk", NVS_READONLY, &handle) == ESP_OK) {
        DeskPresets stored;
        const esp_err_t sit_result = nvs_get_u16(handle, "sit", &stored.sit);
        const esp_err_t stand_result =
            nvs_get_u16(handle, "stand", &stored.stand);
        nvs_close(handle);
        if (sit_result == ESP_OK && stand_result == ESP_OK &&
            valid_presets(stored)) {
            presets = stored;
        }
    }
    ESP_LOGW("desk.control", "SIT/min=%u.%u cm STAND/max=%u.%u cm",
             presets.sit / 10, presets.sit % 10,
             presets.stand / 10, presets.stand % 10);
    return presets;
}

bool save_presets(const DeskPresets& presets) {
    if (!valid_presets(presets)) {
        return false;
    }
    nvs_handle_t handle = 0;
    if (nvs_open("desk", NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    const bool ok = nvs_set_u16(handle, "sit", presets.sit) == ESP_OK &&
                    nvs_set_u16(handle, "stand", presets.stand) == ESP_OK &&
                    nvs_commit(handle) == ESP_OK;
    nvs_close(handle);
    return ok;
}
#endif

#if CONFIG_DESK_MODE_IDLE_EMULATOR || CONFIG_DESK_MODE_STEP_DOWN_TEST || \
    CONFIG_DESK_MODE_HOLD_DOWN_TEST || CONFIG_DESK_MODE_BLE_HID_LOGGER || \
    CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST || CONFIG_DESK_MODE_BLE_CALIBRATION || \
    CONFIG_DESK_MODE_BLE_CONTROLLER
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
#if CONFIG_DESK_MODE_BLE_CONTROLLER
    const auto down_step =
        smart_desk::make_panel_frame(smart_desk::Command::kDownStep);
    const auto up_step =
        smart_desk::make_panel_frame(smart_desk::Command::kUpStep);
    const auto down_hold =
        smart_desk::make_panel_frame(smart_desk::Command::kDownHold);
    const auto up_hold =
        smart_desk::make_panel_frame(smart_desk::Command::kUpHold);
    enum class ProductionState {
        kIdle,
        kDialClassify,
        kManualStep,
        kManualHold,
        kPresetHold,
        kSettling,
    };
    constexpr std::uint32_t kProductionSettledFrames = 5;
    // The D100H can leave gaps of roughly 270 ms between report groups even
    // while the wheel is still being turned. Keep the lease above that gap;
    // this remains a short live-input lease, never a movement queue.
    constexpr TickType_t kManualHoldRelease = pdMS_TO_TICKS(350);
    // One slow physical click can produce two Consumer Control press reports
    // on this D100H.  The first report still starts STEP immediately, but only
    // a continuing stream of at least four reports may escalate it to HOLD.
    constexpr std::uint32_t kHoldEscalationReports = 4;
    // One UART slot is enough for the D100H to distinguish a slow detent
    // (normally at most two reports) from continuous rotation (at least four).
    // Classify before sending anything so the power board never sees a STEP
    // changed into HOLD while the autonomous STEP is already running.
    constexpr TickType_t kDialClassifyWindow = pdMS_TO_TICKS(100);
    // A later detent after the autonomous 1 cm STEP has actually settled is a
    // new STEP, not permission to turn it into a continuous HOLD.
    constexpr TickType_t kStepCompletionMin = pdMS_TO_TICKS(500);
    // J8 height telemetry is deliberately noisy while both columns start and
    // can oscillate for more than a second before its directional envelope
    // moves.  A real standstill is still bounded, but must not be inferred
    // from a single 0.7 s flat part of that sawtooth.
    constexpr TickType_t kMotionStartGrace = pdMS_TO_TICKS(2500);
    constexpr TickType_t kMotionStall = pdMS_TO_TICKS(2000);
    constexpr TickType_t kPresetMaxRun = pdMS_TO_TICKS(20000);
    constexpr TickType_t kManualMaxRun = pdMS_TO_TICKS(30000);
    constexpr TickType_t kSettleMin = pdMS_TO_TICKS(1500);
    constexpr TickType_t kSettleNoMotion = pdMS_TO_TICKS(3000);
    constexpr TickType_t kSettleWatchdog = pdMS_TO_TICKS(7000);
    constexpr TickType_t kRearmQuiet = pdMS_TO_TICKS(500);
    // Short UP runs coasted 1.1..1.3 cm, but a full 81->115 cm preset run
    // coasted 2.0..2.2 cm.  Use the long-run worst case plus margin so the
    // software maximum is never deliberately crossed. DOWN measured 1.8..1.9.
    constexpr std::uint16_t kUpBoundaryCoastTenthsCm = 25;
    constexpr std::uint16_t kDownBoundaryCoastTenthsCm = 20;

    DeskPresets presets = load_presets();
    ProductionState production_state = ProductionState::kIdle;
    std::uint32_t consumed_raw_sequence = g_ble_raw_sequence.load();
    std::uint32_t gesture_event_count = 0;
    int direction = 0;
    std::uint16_t start_height = 0;
    std::uint16_t target_height = 0;
    std::uint16_t progress_height = 0;
    TickType_t motion_start_tick = 0;
    TickType_t progress_tick = 0;
    TickType_t settle_start_tick = 0;
    bool have_progress = false;
    bool moved = false;
    bool allow_hold_escalation = true;
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

#if CONFIG_DESK_MODE_BLE_CONTROLLER
        const TickType_t control_now = xTaskGetTickCount();
        const std::uint16_t control_height = g_height_tenths_cm.load();
        const bool control_fresh =
            g_have_height.load() &&
            control_now - g_height_tick.load() <= kBleTelemetryMaxAge;
        const bool control_settled =
            g_consistent_height_frames.load() >= kProductionSettledFrames;
        const std::uint32_t raw_sequence = g_ble_raw_sequence.load();
        std::uint32_t new_raw_events =
            raw_sequence - consumed_raw_sequence;
        bool have_new_raw = new_raw_events != 0;
        const int raw_direction = g_ble_raw_direction.load();
        ControllerRequest request =
            g_controller_request.exchange(ControllerRequest::kNone);

        if (production_state != ProductionState::kIdle &&
            request != ControllerRequest::kNone) {
            ESP_LOGW("desk.control",
                     "button during motion -> STOP; action not queued");
            g_ble_stop_requested.store(true);
            request = ControllerRequest::kNone;
        }

        if (production_state == ProductionState::kIdle) {
            if (g_ble_stop_requested.exchange(false)) {
                // STOP is latched only until all already received input is
                // consumed. A fresh, later gesture can start normally.
                consumed_raw_sequence = raw_sequence;
            } else if (request == ControllerRequest::kSaveSit ||
                       request == ControllerRequest::kSaveStand) {
                if (!control_fresh || !control_settled ||
                    control_height < kPhysicalMinTenthsCm ||
                    control_height > kPhysicalMaxTenthsCm) {
                    ESP_LOGE("desk.control",
                             "preset save BLOCKED: fresh=%d settled=%d height=%u.%u",
                             control_fresh, control_settled,
                             control_height / 10, control_height % 10);
                } else {
                    DeskPresets candidate = presets;
                    if (request == ControllerRequest::kSaveSit) {
                        candidate.sit = control_height;
                    } else {
                        candidate.stand = control_height;
                    }
                    if (!valid_presets(candidate)) {
                        ESP_LOGE("desk.control",
                                 "preset save BLOCKED: require 70..120 cm and SIT+10<=STAND");
                    } else if (!save_presets(candidate)) {
                        ESP_LOGE("desk.control", "preset save failed in NVS");
                    } else {
                        presets = candidate;
                        ESP_LOGW("desk.control",
                                 "saved SIT/min=%u.%u STAND/max=%u.%u cm",
                                 presets.sit / 10, presets.sit % 10,
                                 presets.stand / 10, presets.stand % 10);
                    }
                }
            } else if (request == ControllerRequest::kGoSit ||
                       request == ControllerRequest::kGoStand) {
                target_height = request == ControllerRequest::kGoSit
                                    ? presets.sit
                                    : presets.stand;
                const unsigned distance =
                    control_height >= target_height
                        ? control_height - target_height
                        : target_height - control_height;
                if (!control_fresh || !control_settled ||
                    control_height < kPhysicalMinTenthsCm ||
                    control_height > kPhysicalMaxTenthsCm) {
                    ESP_LOGE("desk.control",
                             "preset BLOCKED: fresh=%d settled=%d height=%u.%u",
                             control_fresh, control_settled,
                             control_height / 10, control_height % 10);
                } else if (distance <= 10) {
                    ESP_LOGI("desk.control",
                             "preset already within 1.0 cm: current=%u.%u target=%u.%u",
                             control_height / 10, control_height % 10,
                             target_height / 10, target_height % 10);
                } else {
                    direction = target_height > control_height ? 1 : -1;
                    gesture_event_count = 0;
                    start_height = control_height;
                    progress_height = control_height;
                    motion_start_tick = control_now;
                    progress_tick = control_now;
                    have_progress = false;
                    moved = false;
                    start_anomaly_sequence = g_anomaly_sequence.load();
                    g_ble_active_direction.store(direction);
                    production_state = ProductionState::kPresetHold;
                    outgoing = direction > 0 ? &up_hold : &down_hold;
                    ESP_LOGW("desk.control",
                             "PRESET start: %u.%u -> %u.%u cm dir=%+d",
                             start_height / 10, start_height % 10,
                             target_height / 10, target_height % 10,
                             direction);
                }
            } else if (have_new_raw) {
                consumed_raw_sequence = raw_sequence;
                const bool valid_direction =
                    raw_direction == 1 || raw_direction == -1;
                const bool full_step_inside_limit =
                    raw_direction > 0
                        ? static_cast<unsigned>(control_height) + 10U <=
                              presets.stand
                        : static_cast<unsigned>(control_height) >=
                              static_cast<unsigned>(presets.sit) + 10U;
                if (!valid_direction || !control_fresh || !control_settled ||
                    control_height < kPhysicalMinTenthsCm ||
                    control_height > kPhysicalMaxTenthsCm) {
                    ESP_LOGE("desk.control",
                             "dial BLOCKED: dir=%d fresh=%d settled=%d height=%u.%u",
                             raw_direction, control_fresh, control_settled,
                             control_height / 10, control_height % 10);
                } else if (!full_step_inside_limit) {
                    ESP_LOGW("desk.control",
                             "SOFT LIMIT: blocked dir=%+d at %u.%u (SIT=%u.%u STAND=%u.%u)",
                             raw_direction, control_height / 10,
                             control_height % 10, presets.sit / 10,
                             presets.sit % 10, presets.stand / 10,
                             presets.stand % 10);
                } else {
                    direction = raw_direction;
                    gesture_event_count = new_raw_events;
                    start_height = control_height;
                    target_height = direction > 0 ? presets.stand : presets.sit;
                    progress_height = control_height;
                    motion_start_tick = control_now;
                    progress_tick = control_now;
                    settle_start_tick = control_now;
                    have_progress = false;
                    moved = false;
                    start_anomaly_sequence = g_anomaly_sequence.load();
                    g_ble_active_direction.store(direction);
                    g_ble_stop_requested.store(false);
                    const std::uint16_t boundary_coast =
                        direction > 0 ? kUpBoundaryCoastTenthsCm
                                      : kDownBoundaryCoastTenthsCm;
                    const std::uint16_t boundary_brake =
                        direction > 0 ? target_height - boundary_coast
                                      : target_height + boundary_coast;
                    const bool inside_boundary_brake_zone =
                        direction > 0 ? control_height >= boundary_brake
                                      : control_height <= boundary_brake;
                    allow_hold_escalation = !inside_boundary_brake_zone;
                    production_state = ProductionState::kDialClassify;
                    ESP_LOGW("desk.control",
                             "DIAL classify dir=%+d reports=%lu at %u.%u cm",
                             direction,
                             static_cast<unsigned long>(new_raw_events),
                             start_height / 10, start_height % 10);
                }
                // Do not feed the same snapshot into the newly entered state
                // below.  It was already consumed by the idle-state decision.
                have_new_raw = false;
                new_raw_events = 0;
            }
        }

        if (production_state == ProductionState::kDialClassify) {
            if (have_new_raw) {
                consumed_raw_sequence = raw_sequence;
                if (raw_direction != direction) {
                    g_ble_stop_requested.store(true);
                } else {
                    gesture_event_count += new_raw_events;
                }
                // This snapshot belongs to classification and must not be
                // counted again by the state selected below.
                have_new_raw = false;
                new_raw_events = 0;
            }

            const char* cancel_reason = nullptr;
            if (g_ble_stop_requested.load()) {
                cancel_reason = "STOP/reversal/disconnect";
            } else if (!control_fresh || !control_settled) {
                cancel_reason = "telemetry not ready";
            } else if (g_anomaly_sequence.load() != start_anomaly_sequence) {
                cancel_reason = "unexpected controller frame";
            }

            if (cancel_reason != nullptr) {
                ESP_LOGW("desk.control", "DIAL cancelled before motion: %s",
                         cancel_reason);
                production_state = ProductionState::kIdle;
                direction = 0;
                gesture_event_count = 0;
                g_ble_active_direction.store(0);
                g_ble_stop_requested.store(false);
            } else if (control_now - motion_start_tick >=
                       kDialClassifyWindow) {
                // The desk has not been commanded yet, so refresh the motion
                // baseline at the exact slot in which STEP or HOLD starts.
                start_height = control_height;
                progress_height = control_height;
                motion_start_tick = control_now;
                progress_tick = control_now;
                have_progress = false;
                moved = false;
                start_anomaly_sequence = g_anomaly_sequence.load();

                if (allow_hold_escalation &&
                    gesture_event_count >= kHoldEscalationReports) {
                    production_state = ProductionState::kManualHold;
                    outgoing = direction > 0 ? &up_hold : &down_hold;
                    ESP_LOGW("desk.control",
                             "DIAL classified HOLD dir=%+d reports=%lu latency~100ms",
                             direction,
                             static_cast<unsigned long>(gesture_event_count));
                } else {
                    production_state = ProductionState::kManualStep;
                    outgoing = direction > 0 ? &up_step : &down_step;
                    ESP_LOGW("desk.control",
                             "DIAL classified STEP dir=%+d reports=%lu latency~100ms%s",
                             direction,
                             static_cast<unsigned long>(gesture_event_count),
                             allow_hold_escalation
                                 ? ""
                                 : " (HOLD suppressed near limit)");
                }
            }
        }

        if (production_state == ProductionState::kManualStep) {
            // The dual-column telemetry can initially move up to 0.5 cm in
            // the opposite direction. Any change from the stable start still
            // proves that the autonomous STEP started; directional progress
            // remains tracked separately for the final delta.
            if (control_height != start_height) {
                have_progress = true;
                moved = true;
            }
            const bool progressed = direction > 0
                                        ? control_height > progress_height
                                        : control_height < progress_height;
            if (progressed) {
                progress_height = control_height;
                progress_tick = control_now;
                have_progress = true;
                moved = true;
            }

            const char* stop_reason = nullptr;
            if (g_ble_stop_requested.load()) {
                stop_reason = "STOP/reversal/disconnect";
            } else if (!control_fresh) {
                stop_reason = "telemetry stale";
            } else if (g_anomaly_sequence.load() != start_anomaly_sequence) {
                stop_reason = "unexpected controller frame";
            } else if (!have_progress &&
                       control_now - motion_start_tick >= kMotionStartGrace) {
                stop_reason = "no initial movement";
            }

            const bool step_finished =
                moved && control_fresh && control_settled &&
                control_now - motion_start_tick >= kStepCompletionMin;

            if (stop_reason != nullptr) {
                production_state = ProductionState::kSettling;
                settle_start_tick = control_now;
                ESP_LOGW("desk.control", "STEP -> IDLE: %s", stop_reason);
            } else if (step_finished) {
                const int actual = static_cast<int>(control_height) -
                                   static_cast<int>(start_height);
                const unsigned actual_abs =
                    static_cast<unsigned>(actual >= 0 ? actual : -actual);
                ESP_LOGW("desk.control",
                         "STEP complete actual=%c%u.%u cm final=%u.%u",
                         actual >= 0 ? '+' : '-', actual_abs / 10,
                         actual_abs % 10,
                         control_height / 10, control_height % 10);
                production_state = ProductionState::kIdle;
                direction = 0;
                g_ble_active_direction.store(0);
                g_ble_stop_requested.store(false);
            } else if (have_new_raw) {
                consumed_raw_sequence = raw_sequence;
                if (raw_direction != direction) {
                    production_state = ProductionState::kSettling;
                    settle_start_tick = control_now;
                    ESP_LOGW("desk.control", "STEP -> IDLE: dial reversal");
                } else {
                    gesture_event_count += new_raw_events;
                    ESP_LOGI("desk.control",
                             "dial reports discarded during autonomous STEP; total=%lu",
                             static_cast<unsigned long>(gesture_event_count));
                }
            }
        }

        if (production_state == ProductionState::kManualHold ||
            production_state == ProductionState::kPresetHold) {
            if (have_new_raw) {
                consumed_raw_sequence = raw_sequence;
                gesture_event_count += new_raw_events;
            }
            // An initial change in either direction proves that the power
            // controller reacted.  Directional extrema below remain the only
            // source for target/limit decisions and refresh the stall timer.
            // This mirrors the STEP handling and tolerates the observed
            // startup sawtooth (for example 102.0 -> 102.5 before moving down).
            if (!have_progress && control_height != start_height) {
                have_progress = true;
                moved = true;
                progress_tick = control_now;
            }
            const bool progressed = direction > 0
                                        ? control_height > progress_height
                                        : control_height < progress_height;
            if (progressed) {
                progress_height = control_height;
                progress_tick = control_now;
                have_progress = true;
                moved = true;
            }
            const bool telemetry_reversed =
                direction > 0
                    ? control_height + kBleTelemetryReversalLimitTenthsCm <=
                          progress_height
                    : control_height >= progress_height +
                          kBleTelemetryReversalLimitTenthsCm;
            const std::uint16_t coast =
                direction > 0 ? kUpBoundaryCoastTenthsCm
                              : kDownBoundaryCoastTenthsCm;
            const std::uint16_t brake_height =
                direction > 0 ? target_height - coast
                              : target_height + coast;
            const bool boundary_reached =
                direction > 0 ? progress_height >= brake_height
                              : progress_height <= brake_height;
            const bool manual =
                production_state == ProductionState::kManualHold;
            const char* stop_reason = nullptr;

            if (g_ble_stop_requested.load()) {
                stop_reason = "STOP/reversal/disconnect";
            } else if (production_state == ProductionState::kPresetHold &&
                       have_new_raw) {
                stop_reason = "dial override during preset";
            } else if (!control_fresh) {
                stop_reason = "telemetry stale";
            } else if (g_anomaly_sequence.load() != start_anomaly_sequence) {
                stop_reason = "unexpected controller frame";
            } else if (telemetry_reversed) {
                stop_reason = "telemetry reversed >=1.0 cm";
            } else if (boundary_reached) {
                stop_reason = manual ? "software hard limit brake point"
                                     : "preset brake point";
            } else if (manual &&
                       control_now - g_ble_raw_tick.load() >=
                           kManualHoldRelease) {
                stop_reason = "dial released";
            } else if (manual &&
                       control_now - motion_start_tick >= kManualMaxRun) {
                stop_reason = "30 s manual watchdog";
            } else if (!manual &&
                       control_now - motion_start_tick >= kPresetMaxRun) {
                stop_reason = "20 s preset watchdog";
            } else if (have_progress &&
                       control_now - progress_tick >= kMotionStall) {
                stop_reason = "height stalled";
            } else if (!have_progress &&
                       control_now - motion_start_tick >= kMotionStartGrace) {
                stop_reason = "no initial movement";
            }

            if (stop_reason == nullptr) {
                outgoing = direction > 0 ? &up_hold : &down_hold;
            } else {
                production_state = ProductionState::kSettling;
                settle_start_tick = control_now;
                ESP_LOGW("desk.control",
                         "HOLD -> IDLE: %s at %u.%u; events=%lu",
                         stop_reason, control_height / 10,
                         control_height % 10,
                         static_cast<unsigned long>(gesture_event_count));
            }
        }

        if (production_state == ProductionState::kSettling) {
            if (have_new_raw) {
                consumed_raw_sequence = raw_sequence;
                ESP_LOGI("desk.control",
                         "dial discarded during coast/settling");
            }
            const bool directional_motion =
                direction > 0 ? control_height >= start_height + 2
                              : start_height >= control_height + 2;
            moved = moved || directional_motion;
            const bool enough_time =
                control_now - settle_start_tick >= kSettleMin;
            const bool input_quiet =
                control_now - g_ble_raw_tick.load() >= kRearmQuiet;
            const bool finished =
                control_fresh && control_settled && enough_time && input_quiet &&
                (moved || control_now - motion_start_tick >= kSettleNoMotion);
            const bool watchdog =
                control_now - settle_start_tick >= kSettleWatchdog;
            if (finished || watchdog) {
                const int actual = static_cast<int>(control_height) -
                                   static_cast<int>(start_height);
                const unsigned actual_abs =
                    static_cast<unsigned>(actual >= 0 ? actual : -actual);
                ESP_LOGW("desk.control",
                         "IDLE stable: actual=%c%u.%u cm final=%u.%u%s",
                         actual >= 0 ? '+' : '-', actual_abs / 10,
                         actual_abs % 10, control_height / 10,
                         control_height % 10,
                         watchdog ? " (settle watchdog)" : "");
                production_state = ProductionState::kIdle;
                direction = 0;
                gesture_event_count = 0;
                g_ble_active_direction.store(0);
                g_ble_stop_requested.store(false);
                g_controller_request.store(ControllerRequest::kNone);
                ESP_LOGI("desk.control", "READY");
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
    start_ble_hid_logger();
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
    start_ble_hid_logger(ble_motion_action, nullptr);
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
    start_ble_hid_logger(ble_motion_action, nullptr);
#elif CONFIG_DESK_MODE_BLE_CONTROLLER
    ESP_LOGW("smart_desk",
             "BLE CONTROLLER; original panel must be disconnected");
    ESP_LOGW("smart_desk",
             "SIT/min=80 cm, STAND/max=115 cm; dial is immediate/no queue");
    ESP_LOGI("smart_desk", "GPIO17=TX; GPIO18=height RX");

    configure_uart(UART_NUM_1, kPanelToControllerRxGpio,
                   kControllerToPanelRxGpio);
    // BLE initialization owns NVS initialization. Start the writer afterwards
    // so its persisted preset read cannot race nvs_flash_init().
    start_ble_hid_logger(ble_motion_action, nullptr, false);
    xTaskCreate(reader_task, "controller_rx", 4096, &kEmulatorReader, 5,
                nullptr);
    xTaskCreate(panel_writer_task, "panel_tx", 6144, nullptr, 5, nullptr);
#else
#error "Select a smart desk firmware mode"
#endif
}
