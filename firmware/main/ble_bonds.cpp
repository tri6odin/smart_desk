#include "ble_bonds.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>

#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace ble_bonds {
namespace {

constexpr char kTag[] = "d100h.bonds";
constexpr std::size_t kCapacity = CONFIG_BT_SMP_MAX_BONDS;
static_assert(kCapacity > 0);
std::array<esp_ble_bond_dev_t, kCapacity> g_devices{};
std::size_t g_count = 0;
portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;
std::array<std::uint8_t, ESP_BD_ADDR_LEN> g_connected{};
bool g_connected_valid = false;
std::array<std::uint8_t, ESP_BD_ADDR_LEN> g_retained{};
bool g_retained_valid = false;
std::array<std::uint8_t, ESP_BD_ADDR_LEN> g_forget{};
bool g_forget_requested = false;
bool g_prune_requested = false;
bool g_removal_in_flight = false;

bool same_address(const std::uint8_t* first, const std::uint8_t* second) {
    return std::memcmp(first, second, ESP_BD_ADDR_LEN) == 0;
}

bool matches_device(const esp_ble_bond_dev_t& device,
                    const std::uint8_t* address) {
    return same_address(device.bd_addr, address) ||
           ((device.bond_key.key_mask & ESP_BLE_ID_KEY_MASK) != 0 &&
            same_address(device.bond_key.pid_key.static_addr, address));
}

void format_address(const std::uint8_t* address, char* text) {
    std::snprintf(text, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
                  address[0], address[1], address[2], address[3], address[4],
                  address[5]);
}

}  // namespace

void refresh() {
    // GAP callbacks have limited stack. A full 15-entry IDF bond list belongs
    // on the heap, not in a callback's local stack frame.
    std::unique_ptr<esp_ble_bond_dev_t[]> devices(
        new (std::nothrow) esp_ble_bond_dev_t[kCapacity]{});
    if (!devices) {
        ESP_LOGE(kTag, "could not allocate BLE bond snapshot");
        return;
    }
    int count = esp_ble_get_bond_device_num();
    if (count < 0 || count > static_cast<int>(kCapacity)) {
        ESP_LOGE(kTag, "invalid BLE bond count=%d (capacity=%u)", count,
                 static_cast<unsigned>(kCapacity));
        return;
    }
    if (count != 0) {
        int requested = count;
        const esp_err_t result = esp_ble_get_bond_device_list(
            &requested, devices.get());
        if (result != ESP_OK || requested < 0 || requested > count) {
            ESP_LOGE(kTag, "could not read BLE bonds: %s count=%d",
                     esp_err_to_name(result), requested);
            return;
        }
        count = requested;
    }
    portENTER_CRITICAL(&g_lock);
    std::copy_n(devices.get(), kCapacity, g_devices.begin());
    g_count = static_cast<std::size_t>(count);
    portEXIT_CRITICAL(&g_lock);
    ESP_LOGI(kTag, "stored BLE bonds=%d", count);
    for (int index = 0; index < count; ++index) {
        char text[18] = {};
        const auto& bond = devices[static_cast<std::size_t>(index)];
        format_address(bond.bd_addr, text);
        ESP_LOGI(kTag, "bond[%d]=%s type=%u keys=0x%02x", index, text,
                 static_cast<unsigned>(bond.bd_addr_type),
                 static_cast<unsigned>(bond.bond_key.key_mask));
    }
}

bool matches(const std::uint8_t* address) {
    bool matched = false;
    portENTER_CRITICAL(&g_lock);
    for (std::size_t index = 0; index < g_count; ++index) {
        if (matches_device(g_devices[index], address)) {
            matched = true;
            break;
        }
    }
    portEXIT_CRITICAL(&g_lock);
    return matched;
}

void connected(const std::uint8_t* address) {
    portENTER_CRITICAL(&g_lock);
    std::memcpy(g_connected.data(), address, g_connected.size());
    g_connected_valid = true;
    portEXIT_CRITICAL(&g_lock);
}

void disconnected() {
    portENTER_CRITICAL(&g_lock);
    g_connected_valid = false;
    portEXIT_CRITICAL(&g_lock);
}

void authenticated(const std::uint8_t* address) {
    refresh();
    std::size_t matches = 0;
    std::array<std::uint8_t, ESP_BD_ADDR_LEN> retained{};
    std::size_t count = 0;
    portENTER_CRITICAL(&g_lock);
    g_retained_valid = false;
    g_prune_requested = false;
    count = g_count;
    for (std::size_t index = 0; index < g_count; ++index) {
        if (matches_device(g_devices[index], address)) {
            std::memcpy(retained.data(), g_devices[index].bd_addr,
                        retained.size());
            ++matches;
        }
    }
    if (matches == 1) {
        g_retained = retained;
        g_retained_valid = true;
        g_prune_requested = g_count > 1;
    }
    portEXIT_CRITICAL(&g_lock);
    if (matches != 1 && count > 1) {
        ESP_LOGW(kTag,
                 "authenticated address matches %u identities; keeping %u bonds",
                 static_cast<unsigned>(matches), static_cast<unsigned>(count));
    }
}

bool queue_forget_connected() {
    bool queued = false;
    portENTER_CRITICAL(&g_lock);
    if (g_connected_valid && !g_forget_requested) {
        g_forget = g_connected;
        g_forget_requested = true;
        g_prune_requested = false;
        g_retained_valid = false;
        queued = true;
    }
    portEXIT_CRITICAL(&g_lock);
    return queued;
}

void process_requests() {
    std::array<std::uint8_t, ESP_BD_ADDR_LEN> address{};
    bool requested = false;
    bool pruning = false;
    portENTER_CRITICAL(&g_lock);
    if (!g_removal_in_flight) {
        if (g_forget_requested) {
            address = g_forget;
            g_forget_requested = false;
            requested = true;
        } else if (g_prune_requested && g_retained_valid) {
            for (std::size_t index = 0; index < g_count; ++index) {
                if (!same_address(g_devices[index].bd_addr,
                                  g_retained.data())) {
                    std::memcpy(address.data(), g_devices[index].bd_addr,
                                address.size());
                    requested = true;
                    pruning = true;
                    break;
                }
            }
            if (!requested) {
                g_prune_requested = false;
            }
        }
        if (requested) {
            g_removal_in_flight = true;
        }
    }
    portEXIT_CRITICAL(&g_lock);
    if (!requested) {
        return;
    }
    char text[18] = {};
    format_address(address.data(), text);
    const esp_err_t result = esp_ble_remove_bond_device(address.data());
    if (result == ESP_OK) {
        ESP_LOGW(kTag, "%s bond %s", pruning ? "retiring old" : "forgetting", text);
    } else {
        ESP_LOGE(kTag, "could not remove bond %s: %s", text,
                 esp_err_to_name(result));
        portENTER_CRITICAL(&g_lock);
        g_removal_in_flight = false;
        g_prune_requested = false;
        portEXIT_CRITICAL(&g_lock);
    }
}

void removal_complete(bool success) {
    portENTER_CRITICAL(&g_lock);
    g_removal_in_flight = false;
    if (!success) {
        g_prune_requested = false;
    }
    portEXIT_CRITICAL(&g_lock);
    refresh();
}

}  // namespace ble_bonds
