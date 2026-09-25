#include "ble_hid_logger.hpp"
#include "ble_bonds.hpp"

#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>

#include "desk_inputs.hpp"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_hid_common.h"
#include "esp_hidh.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {

constexpr char kTag[] = "d100h.logger";
constexpr EventBits_t kScanParamsReady = BIT0;
constexpr EventBits_t kScanFinished = BIT1;
constexpr EventBits_t kOpenFinished = BIT2;
constexpr EventBits_t kDisconnected = BIT3;
constexpr EventBits_t kPrivacyReady = BIT4;
constexpr std::uint32_t kScanSeconds = 10;
// The D100H can pause for roughly 270 ms between the first event and the
// accelerated repeats of one physical flick.  A shorter window split one
// measured flick into +1 cm and +3 cm actions, so wait for 400 ms of actual
// silence before committing a dial gesture.
constexpr TickType_t kDialBurstQuietTime = pdMS_TO_TICKS(400);
constexpr TickType_t kLongPressThreshold = pdMS_TO_TICKS(4500);
constexpr TickType_t kForgetBondHoldTime = pdMS_TO_TICKS(5000);
constexpr TickType_t kCalibrationHoldTime = pdMS_TO_TICKS(10000);
constexpr std::uint16_t kPlayPauseUsage = 0x00CD;

EventGroupHandle_t g_events = nullptr;
std::array<std::uint8_t, 6> g_candidate_address{};
std::uint8_t g_candidate_address_type = 0;
std::atomic<bool> g_have_candidate{false};
std::atomic<bool> g_connected{false};
std::atomic<bool> g_open_pending{false};
bool g_publish_delayed_dial_delta = true;

portMUX_TYPE g_semantic_lock = portMUX_INITIALIZER_UNLOCKED;
smart_desk::LongPressDetector g_forget_bond_button(
    static_cast<std::uint32_t>(kForgetBondHoldTime));
smart_desk::PresetGestureDetector g_preset_buttons(
    static_cast<std::uint32_t>(kCalibrationHoldTime));
int g_dial_direction = 0;
std::uint32_t g_dial_event_count = 0;
TickType_t g_dial_last_tick = 0;
bool g_dial_cancelled_until_quiet = false;
BleHidActionCallback g_action_callback = nullptr;
void* g_action_callback_context = nullptr;

void publish_action(BleHidActionType type, int value = 0) {
    if (g_action_callback != nullptr) {
        g_action_callback(BleHidAction{type, value},
                          g_action_callback_context);
    }
}

void reset_semantic_state() {
    taskENTER_CRITICAL(&g_semantic_lock);
    g_forget_bond_button.reset();
    g_preset_buttons.reset();
    g_dial_direction = 0;
    g_dial_event_count = 0;
    g_dial_last_tick = 0;
    g_dial_cancelled_until_quiet = false;
    taskEXIT_CRITICAL(&g_semantic_lock);
}

bool contains_text(const char* haystack, const char* needle) {
    return haystack != nullptr && std::strstr(haystack, needle) != nullptr;
}

bool is_d100h_name(const char* name) {
    return contains_text(name, "Ulanzi") || contains_text(name, "Dial") ||
           contains_text(name, "KEHWIN");
}

void format_address(const std::uint8_t* address, char* output,
                    std::size_t output_size) {
    std::snprintf(output, output_size, "%02X:%02X:%02X:%02X:%02X:%02X",
                  address[0], address[1], address[2], address[3], address[4],
                  address[5]);
}

void queue_forget_current_bond() {
    if (ble_bonds::queue_forget_connected()) {
        ESP_LOGW(kTag, "ACTION STOP; forgetting current D100H bond");
        publish_action(BleHidActionType::kStop);
    } else {
        ESP_LOGW(kTag, "forget bond ignored: no connected D100H");
    }
}

void configure_security() {
    esp_ble_auth_req_t auth = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t capabilities = ESP_IO_CAP_NONE;
    std::uint8_t key_size = 16;
    std::uint8_t init_keys = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    std::uint8_t response_keys = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(
        ESP_BLE_SM_AUTHEN_REQ_MODE, &auth, sizeof(auth)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(
        ESP_BLE_SM_IOCAP_MODE, &capabilities, sizeof(capabilities)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(
        ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(
        ESP_BLE_SM_SET_INIT_KEY, &init_keys, sizeof(init_keys)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(
        ESP_BLE_SM_SET_RSP_KEY, &response_keys, sizeof(response_keys)));
}

void log_button_release(const char* name, TickType_t duration) {
    if (duration >= kLongPressThreshold) {
        ESP_LOGW(kTag, "ACTION SAVE_%s (held %lu ms)", name,
                 static_cast<unsigned long>(duration * portTICK_PERIOD_MS));
        publish_action(std::strcmp(name, "SIT") == 0
                           ? BleHidActionType::kSaveSit
                           : BleHidActionType::kSaveStand);
    } else {
        ESP_LOGW(kTag, "ACTION GO_%s (held %lu ms)", name,
                 static_cast<unsigned long>(duration * portTICK_PERIOD_MS));
        publish_action(std::strcmp(name, "SIT") == 0
                           ? BleHidActionType::kGoSit
                           : BleHidActionType::kGoStand);
    }
}

void record_dial_event(int direction) {
    const TickType_t now = xTaskGetTickCount();
    bool reversal = false;

    // The motion layer receives physical direction immediately so that an
    // opposite turn can brake without waiting for gesture classification.
    publish_action(BleHidActionType::kDialRaw, direction);

    taskENTER_CRITICAL(&g_semantic_lock);
    if (g_dial_cancelled_until_quiet) {
        // Never queue reports while a reversal/braking gesture is active.
        // A new gesture is required after the quiet period.
        g_dial_last_tick = now;
    } else if (g_dial_event_count != 0 && g_dial_direction != direction) {
        reversal = true;
        g_dial_cancelled_until_quiet = true;
        g_dial_direction = 0;
        g_dial_event_count = 0;
        g_dial_last_tick = now;
    } else {
        g_dial_direction = direction;
        ++g_dial_event_count;
        g_dial_last_tick = now;
    }
    taskEXIT_CRITICAL(&g_semantic_lock);

    if (reversal) {
        ESP_LOGW(kTag,
                 "ACTION STOP/CANCEL (dial reversal; discard until quiet)");
        publish_action(BleHidActionType::kStop);
    }
}

void interpret_consumer_control(const std::uint8_t* bytes,
                                std::size_t length) {
    if (length < 2) {
        return;
    }

    const std::uint16_t usage =
        static_cast<std::uint16_t>(bytes[0]) |
        (static_cast<std::uint16_t>(bytes[1]) << 8);
    const TickType_t now = xTaskGetTickCount();

    // A dial or unrelated button while a preset is pending cancels the whole
    // gesture. Do not let that report move the desk or start BLE unpairing.
    if (usage != 0 && usage != 0x00B6 && usage != 0x00B5) {
        bool cancelled = false;
        taskENTER_CRITICAL(&g_semantic_lock);
        cancelled = g_preset_buttons.cancel();
        taskEXIT_CRITICAL(&g_semantic_lock);
        if (cancelled) {
            ESP_LOGW(kTag, "ACTION STOP (preset gesture interrupted)");
            publish_action(BleHidActionType::kUserActivity);
            publish_action(BleHidActionType::kStop);
            return;
        }
    }

    if (usage == 0x00E9) {
        publish_action(BleHidActionType::kUserActivity);
        record_dial_event(+1);
        return;
    }
    if (usage == 0x00EA) {
        publish_action(BleHidActionType::kUserActivity);
        record_dial_event(-1);
        return;
    }
    if (usage == 0x00E2) {
        publish_action(BleHidActionType::kUserActivity);
        ESP_LOGW(kTag, "ACTION STOP (dial press)");
        publish_action(BleHidActionType::kStop);
        return;
    }

    if (usage == 0x00B6) {
        smart_desk::PresetPressResult result;
        taskENTER_CRITICAL(&g_semantic_lock);
        result = g_preset_buttons.press(
            smart_desk::PresetButton::kSit, static_cast<std::uint32_t>(now));
        taskEXIT_CRITICAL(&g_semantic_lock);
        if (result.new_press) {
            publish_action(BleHidActionType::kUserActivity);
        }
        if (result.chord_started) {
            ESP_LOGW(kTag,
                     "ACTION STOP (SIT+STAND); hold both 10 s for calibration");
            publish_action(BleHidActionType::kStop);
        }
        ESP_LOGI(kTag, "SIT pressed");
        return;
    }
    if (usage == 0x00B5) {
        smart_desk::PresetPressResult result;
        taskENTER_CRITICAL(&g_semantic_lock);
        result = g_preset_buttons.press(
            smart_desk::PresetButton::kStand,
            static_cast<std::uint32_t>(now));
        taskEXIT_CRITICAL(&g_semantic_lock);
        if (result.new_press) {
            publish_action(BleHidActionType::kUserActivity);
        }
        if (result.chord_started) {
            ESP_LOGW(kTag,
                     "ACTION STOP (SIT+STAND); hold both 10 s for calibration");
            publish_action(BleHidActionType::kStop);
        }
        ESP_LOGI(kTag, "STAND pressed");
        return;
    }
    if (usage == kPlayPauseUsage) {
        bool new_press = false;
        taskENTER_CRITICAL(&g_semantic_lock);
        new_press =
            g_forget_bond_button.press(static_cast<std::uint32_t>(now));
        taskEXIT_CRITICAL(&g_semantic_lock);
        if (new_press) {
            publish_action(BleHidActionType::kUserActivity);
        }
        ESP_LOGI(kTag, "PLAY/PAUSE pressed; hold 5 s to forget BLE bond");
        return;
    }

    if (usage != 0) {
        publish_action(BleHidActionType::kUserActivity);
        return;
    }

    bool forget_bond = false;
    smart_desk::PresetReleaseResult preset_release;
    taskENTER_CRITICAL(&g_semantic_lock);
    preset_release =
        g_preset_buttons.release(static_cast<std::uint32_t>(now));
    forget_bond =
        g_forget_bond_button.release(static_cast<std::uint32_t>(now));
    taskEXIT_CRITICAL(&g_semantic_lock);

    if (preset_release.sit_pressed && !preset_release.suppress_presets) {
        log_button_release("SIT", preset_release.sit_duration);
    }
    if (preset_release.stand_pressed && !preset_release.suppress_presets) {
        log_button_release("STAND", preset_release.stand_duration);
    }
    if (forget_bond) {
        queue_forget_current_bond();
    }
    if (preset_release.start_calibration) {
        ESP_LOGW(kTag,
                 "ACTION START_CALIBRATION (SIT+STAND held 10 s)");
        publish_action(BleHidActionType::kStartCalibration);
    }
}

void semantic_timer_task(void*) {
    while (true) {
        int direction = 0;
        std::uint32_t count = 0;
        bool rearmed_after_reversal = false;
        bool forget_bond = false;
        bool start_calibration = false;

        taskENTER_CRITICAL(&g_semantic_lock);
        const TickType_t now = xTaskGetTickCount();
        forget_bond =
            g_forget_bond_button.poll(static_cast<std::uint32_t>(now));
        start_calibration =
            g_preset_buttons.poll(static_cast<std::uint32_t>(now));
        if (g_dial_cancelled_until_quiet &&
            now - g_dial_last_tick >= kDialBurstQuietTime) {
            g_dial_cancelled_until_quiet = false;
            rearmed_after_reversal = true;
        } else if (g_dial_event_count != 0 &&
            now - g_dial_last_tick >= kDialBurstQuietTime) {
            direction = g_dial_direction;
            count = g_dial_event_count;
            g_dial_direction = 0;
            g_dial_event_count = 0;
        }
        taskEXIT_CRITICAL(&g_semantic_lock);

        if (forget_bond) {
            queue_forget_current_bond();
        }
        if (start_calibration) {
            ESP_LOGW(kTag,
                     "ACTION START_CALIBRATION (SIT+STAND held 10 s)");
            publish_action(BleHidActionType::kStartCalibration);
        }
        ble_bonds::process_requests();

        if (rearmed_after_reversal) {
            ESP_LOGI(kTag,
                     "dial rearmed after quiet; new gesture required");
        }
        if (count != 0 && g_publish_delayed_dial_delta) {
            const int delta = (count == 1 ? 1 : 3) * direction;
            ESP_LOGW(kTag,
                     "ACTION DIAL_DELTA=%+d cm (events=%lu)",
                     delta, static_cast<unsigned long>(count));
            publish_action(BleHidActionType::kDialDelta, delta);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void hid_event(void*, esp_event_base_t, std::int32_t event_id,
               void* event_data) {
    const auto event = static_cast<esp_hidh_event_t>(event_id);
    auto* data = static_cast<esp_hidh_event_data_t*>(event_data);

    switch (event) {
        case ESP_HIDH_OPEN_EVENT: {
            g_open_pending.store(false);
            if (data->open.status == ESP_OK) {
                g_connected.store(true);
                ble_bonds::refresh();
                const auto* address = esp_hidh_dev_bda_get(data->open.dev);
                char address_text[18] = {};
                if (address != nullptr) {
                    format_address(address, address_text, sizeof(address_text));
                    ble_bonds::connected(address);
                }
                ESP_LOGW(kTag, "CONNECTED %s name='%s'", address_text,
                         esp_hidh_dev_name_get(data->open.dev));
                if (g_action_callback == nullptr) {
                    ESP_LOGI(kTag,
                             "HID dry-run enabled; desk stays IDLE");
                } else {
                    ESP_LOGW(kTag, "live HID action callback enabled");
                }
                esp_hidh_dev_dump(data->open.dev, stdout);
            } else {
                ESP_LOGE(kTag, "open failed: %s",
                         esp_err_to_name(data->open.status));
            }
            xEventGroupSetBits(g_events, kOpenFinished);
            break;
        }
        case ESP_HIDH_INPUT_EVENT: {
            ESP_LOGD(kTag,
                     "INPUT usage=%s map=%u report_id=%u length=%u",
                     esp_hid_usage_str(data->input.usage),
                     data->input.map_index, data->input.report_id,
                     data->input.length);
            ESP_LOG_BUFFER_HEX_LEVEL(kTag, data->input.data,
                                     data->input.length, ESP_LOG_DEBUG);
            if (data->input.usage == ESP_HID_USAGE_CCONTROL &&
                data->input.report_id == 2) {
                interpret_consumer_control(data->input.data,
                                           data->input.length);
            }
            break;
        }
        case ESP_HIDH_FEATURE_EVENT:
            ESP_LOGI(kTag,
                     "FEATURE usage=%s map=%u report_id=%u length=%u",
                     esp_hid_usage_str(data->feature.usage),
                     data->feature.map_index, data->feature.report_id,
                     data->feature.length);
            ESP_LOG_BUFFER_HEX_LEVEL(kTag, data->feature.data,
                                     data->feature.length, ESP_LOG_INFO);
            break;
        case ESP_HIDH_BATTERY_EVENT:
            ESP_LOGI(kTag, "battery=%u%% status=%s", data->battery.level,
                     esp_err_to_name(data->battery.status));
            break;
        case ESP_HIDH_CLOSE_EVENT:
            ESP_LOGW(kTag, "DISCONNECTED reason=%d", data->close.reason);
            ESP_LOGW(kTag, "ACTION STOP (BLE disconnected)");
            reset_semantic_state();
            publish_action(BleHidActionType::kDisconnected);
            g_connected.store(false);
            g_open_pending.store(false);
            ble_bonds::disconnected();
            esp_hidh_dev_free(data->close.dev);
            xEventGroupSetBits(g_events, kDisconnected);
            break;
        default:
            ESP_LOGI(kTag, "HID event=%ld", static_cast<long>(event_id));
            break;
    }
}

void inspect_advertisement(esp_ble_gap_cb_param_t::ble_scan_result_evt_param& result) {
    std::uint8_t name_length = 0;
    auto* name_data = esp_ble_resolve_adv_data_by_type(
        result.ble_adv, result.adv_data_len + result.scan_rsp_len,
        ESP_BLE_AD_TYPE_NAME_CMPL, &name_length);
    if (name_data == nullptr) {
        name_data = esp_ble_resolve_adv_data_by_type(
            result.ble_adv, result.adv_data_len + result.scan_rsp_len,
            ESP_BLE_AD_TYPE_NAME_SHORT, &name_length);
    }

    char name[64] = {};
    if (name_data != nullptr && name_length != 0) {
        const std::size_t copy_length =
            name_length < sizeof(name) - 1 ? name_length : sizeof(name) - 1;
        std::memcpy(name, name_data, copy_length);
        name[copy_length] = '\0';
    }

    const bool matched_name = is_d100h_name(name);
    const bool matched_bond = ble_bonds::matches(result.bda);
    if ((!matched_name && !matched_bond) || g_connected.load() ||
        g_open_pending.load() || g_have_candidate.load()) {
        return;
    }

    std::memcpy(g_candidate_address.data(), result.bda,
                g_candidate_address.size());
    g_candidate_address_type = result.ble_addr_type;
    g_have_candidate.store(true);

    char address_text[18] = {};
    format_address(result.bda, address_text, sizeof(address_text));
    ESP_LOGW(kTag, "candidate %s name='%s' RSSI=%d match=%s", address_text,
             name, result.rssi, matched_bond ? "stored-bond" : "name");
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_stop_scanning());
}

void gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* data) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            xEventGroupSetBits(g_events, kScanParamsReady);
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (data->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                inspect_advertisement(data->scan_rst);
            } else if (data->scan_rst.search_evt ==
                       ESP_GAP_SEARCH_INQ_CMPL_EVT) {
                xEventGroupSetBits(g_events, kScanFinished);
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            xEventGroupSetBits(g_events, kScanFinished);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI(kTag, "accepting BLE security request");
            esp_ble_gap_security_rsp(data->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (data->ble_security.auth_cmpl.success) {
                ESP_LOGI(kTag, "pairing/authentication successful");
                ble_bonds::authenticated(data->ble_security.auth_cmpl.bd_addr);
            } else {
                ESP_LOGE(kTag, "pairing failed, reason=0x%02X",
                         data->ble_security.auth_cmpl.fail_reason);
            }
            break;
        case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
            if (data->remove_bond_dev_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGW(kTag, "BLE bond removed");
            } else {
                ESP_LOGE(kTag, "BLE bond removal failed, status=%d",
                         data->remove_bond_dev_cmpl.status);
            }
            ble_bonds::removal_complete(
                data->remove_bond_dev_cmpl.status == ESP_BT_STATUS_SUCCESS);
            break;
        default:
            break;
    }
}

void scan_task(void*) {
    xEventGroupWaitBits(g_events, kScanParamsReady | kPrivacyReady,
                        pdTRUE, pdTRUE,
                        portMAX_DELAY);

    while (true) {
        g_have_candidate.store(false);
        xEventGroupClearBits(g_events,
                             kScanFinished | kOpenFinished | kDisconnected);
        ESP_LOGI(kTag, "scanning for Ulanzi/Dial BLE HID (%lu s)",
                 static_cast<unsigned long>(kScanSeconds));
        const esp_err_t scan_result = esp_ble_gap_start_scanning(kScanSeconds);
        if (scan_result != ESP_OK) {
            ESP_LOGE(kTag, "scan start failed: %s",
                     esp_err_to_name(scan_result));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        xEventGroupWaitBits(g_events, kScanFinished, pdTRUE, pdTRUE,
                            portMAX_DELAY);
        if (!g_have_candidate.load()) {
            ESP_LOGI(kTag, "D100H not found; retrying");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        g_open_pending.store(true);
        if (esp_hidh_dev_open(g_candidate_address.data(),
                              ESP_HID_TRANSPORT_BLE,
                              g_candidate_address_type) == nullptr) {
            g_open_pending.store(false);
            ESP_LOGE(kTag, "could not start HID connection");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        xEventGroupWaitBits(g_events, kOpenFinished, pdTRUE, pdTRUE,
                            portMAX_DELAY);
        if (!g_connected.load()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        xEventGroupWaitBits(g_events, kDisconnected, pdTRUE, pdTRUE,
                            portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

}  // namespace

void start_ble_hid_logger(BleHidActionCallback callback,
                          void* callback_context,
                          bool publish_delayed_dial_delta) {
    g_action_callback = callback;
    g_action_callback_context = callback_context;
    g_publish_delayed_dial_delta = publish_delayed_dial_delta;
    g_events = xEventGroupCreate();
    ESP_ERROR_CHECK(g_events != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t controller_config =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&controller_config));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    esp_bluedroid_config_t bluedroid_config =
        BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_config));
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event));
    ESP_ERROR_CHECK(
        esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));

    const esp_hidh_config_t hid_config{
        .callback = hid_event,
        .event_stack_size = 6144,
        .callback_arg = nullptr,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&hid_config));
    configure_security();
    ble_bonds::refresh();

    esp_ble_scan_params_t scan_parameters{
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
    };
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_parameters));
    // D100H resumes a bonded session by connecting to the same controller
    // identity it paired with. Keep the ESP's public address stable: using an
    // RPA here made the remote advertise, but every open failed with HCI 0x3e.
    ESP_LOGI(kTag, "using stable public BLE identity for bonded reconnect");
    xEventGroupSetBits(g_events, kPrivacyReady);
    ESP_ERROR_CHECK(xTaskCreate(scan_task, "d100h_scan", 6144, nullptr, 3,
                               nullptr) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(semantic_timer_task, "d100h_semantic", 3072,
                               nullptr, 3, nullptr) == pdPASS
                        ? ESP_OK : ESP_ERR_NO_MEM);
}
