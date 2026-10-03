#include "lab_hid_trace.hpp"

#include "esp_log.h"

namespace lab_hid_trace {
namespace {
constexpr const char* kNames[]{
    "DIAL_RAW", "DIAL_DELTA", "STOP", "GO_SIT", "GO_STAND", "SAVE_SIT",
    "SAVE_STAND", "DISCONNECTED", "USER_ACTIVITY", "START_CALIBRATION",
};
}

void report(std::uint8_t id, const std::uint8_t* bytes, std::size_t length) {
    ESP_LOGI("lab.hid", "report_id=%u length=%u", id, static_cast<unsigned>(length));
    ESP_LOG_BUFFER_HEX_LEVEL("lab.hid", bytes, length, ESP_LOG_INFO);
}

void action(BleHidActionType type, int value) {
    const auto index = static_cast<unsigned>(type);
    if (index < sizeof(kNames) / sizeof(kNames[0])) {
        ESP_LOGI("lab.hid", "%s value=%d", kNames[index], value);
    }
}
}
