#include "preset_store.hpp"

#include "esp_log.h"
#include "nvs.h"

namespace smart_desk {

DeskPresets load_presets(const ControllerConfig& config) {
    DeskPresets presets{config.sit_target_tenths_cm, config.stand_target_tenths_cm};
    nvs_handle_t handle = 0;
    if (nvs_open("desk", NVS_READONLY, &handle) == ESP_OK) {
        DeskPresets stored;
        const esp_err_t sit_result = nvs_get_u16(handle, "sit", &stored.sit);
        const esp_err_t stand_result = nvs_get_u16(handle, "stand", &stored.stand);
        nvs_close(handle);
        if (sit_result == ESP_OK && stand_result == ESP_OK &&
            DeskController(config).valid_presets(stored.sit, stored.stand)) {
            presets = stored;
        }
    }
    ESP_LOGW("desk.control", "SIT=%u.%u cm STAND=%u.%u cm limits=%u.%u..%u.%u",
             presets.sit / 10, presets.sit % 10, presets.stand / 10,
             presets.stand % 10,
             config.soft_min_tenths_cm / 10, config.soft_min_tenths_cm % 10,
             config.soft_max_tenths_cm / 10, config.soft_max_tenths_cm % 10);
    return presets;
}

bool save_presets(const DeskPresets& presets, const ControllerConfig& config) {
    if (!DeskController(config).valid_presets(presets.sit, presets.stand)) {
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

bool load_recovery_lock() {
    nvs_handle_t handle = 0;
    std::uint8_t locked = 0;
    const esp_err_t open_result = nvs_open("desk", NVS_READONLY, &handle);
    if (open_result == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (open_result != ESP_OK) {
        ESP_LOGE("desk.recovery", "could not read motion lock: %s",
                 esp_err_to_name(open_result));
        return true;
    }
    const esp_err_t result = nvs_get_u8(handle, "recovery_lock", &locked);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (result != ESP_OK) {
        ESP_LOGE("desk.recovery", "could not read motion lock value: %s",
                 esp_err_to_name(result));
        return true;
    }
    return locked != 0;
}

bool save_recovery_lock(bool locked) {
    nvs_handle_t handle = 0;
    if (nvs_open("desk", NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    const bool ok =
        nvs_set_u8(handle, "recovery_lock", locked ? 1 : 0) == ESP_OK &&
        nvs_commit(handle) == ESP_OK;
    nvs_close(handle);
    return ok;
}

}  // namespace smart_desk
