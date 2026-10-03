#include "preset_store.hpp"

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

}  // namespace smart_desk
