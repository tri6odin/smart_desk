#pragma once

#include <cstdint>

#define ESP_BD_ADDR_LEN 6
#define ESP_BLE_ID_KEY_MASK 0x04
#define CONFIG_BT_SMP_MAX_BONDS 15

using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;

struct esp_ble_bond_dev_t {
    std::uint8_t bd_addr[ESP_BD_ADDR_LEN]{};
    std::uint8_t bd_addr_type = 0;
    struct {
        std::uint8_t key_mask = 0;
        struct {
            std::uint8_t static_addr[ESP_BD_ADDR_LEN]{};
        } pid_key;
    } bond_key;
};

int esp_ble_get_bond_device_num();
esp_err_t esp_ble_get_bond_device_list(int* count, esp_ble_bond_dev_t* list);
esp_err_t esp_ble_remove_bond_device(const std::uint8_t* address);
const char* esp_err_to_name(esp_err_t value);
