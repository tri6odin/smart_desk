#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

#include "ble_bonds.hpp"
#include "esp_gap_ble_api.h"

namespace {

using Address = std::array<std::uint8_t, ESP_BD_ADDR_LEN>;
std::vector<esp_ble_bond_dev_t> stored;
Address pending_removal{};
int removal_requests = 0;

esp_ble_bond_dev_t bond(std::uint8_t id, std::uint8_t identity = 0) {
    esp_ble_bond_dev_t result{};
    result.bd_addr[5] = id;
    if (identity != 0) {
        result.bond_key.key_mask = ESP_BLE_ID_KEY_MASK;
        result.bond_key.pid_key.static_addr[5] = identity;
    }
    return result;
}

void complete_removal() {
    stored.erase(std::remove_if(stored.begin(), stored.end(),
                                [](const auto& device) {
                                    return std::memcmp(device.bd_addr,
                                                       pending_removal.data(),
                                                       ESP_BD_ADDR_LEN) == 0;
                                }),
                 stored.end());
    ble_bonds::removal_complete(true);
}

void reset(std::initializer_list<esp_ble_bond_dev_t> devices) {
    stored.assign(devices);
    removal_requests = 0;
    ble_bonds::disconnected();
    ble_bonds::refresh();
    const Address unknown{9, 9, 9, 9, 9, 9};
    ble_bonds::authenticated(unknown.data());
}

void test_unique_identity_is_retained() {
    reset({bond(1), bond(2), bond(3, 42)});
    const Address identity{0, 0, 0, 0, 0, 42};
    ble_bonds::authenticated(identity.data());
    ble_bonds::process_requests();
    assert(removal_requests == 1);
    assert(pending_removal[5] != 3);
    ble_bonds::process_requests();
    assert(removal_requests == 1);  // Wait for asynchronous callback.
    complete_removal();
    ble_bonds::process_requests();
    assert(removal_requests == 2);
    assert(pending_removal[5] != 3);
    complete_removal();
    ble_bonds::process_requests();
    assert(removal_requests == 2);
    assert(stored.size() == 1 && stored[0].bd_addr[5] == 3);
}

void test_ambiguous_or_unknown_identity_keeps_every_bond() {
    reset({bond(1, 42), bond(2, 42), bond(3)});
    const Address identity{0, 0, 0, 0, 0, 42};
    ble_bonds::authenticated(identity.data());
    ble_bonds::process_requests();
    assert(removal_requests == 0);
    const Address unknown{0, 0, 0, 0, 0, 99};
    ble_bonds::authenticated(unknown.data());
    ble_bonds::process_requests();
    assert(removal_requests == 0);
    assert(stored.size() == 3);
}

void test_explicit_forget_needs_connection() {
    reset({bond(7)});
    assert(!ble_bonds::queue_forget_connected());
    const Address connected{0, 0, 0, 0, 0, 7};
    ble_bonds::connected(connected.data());
    assert(ble_bonds::queue_forget_connected());
    ble_bonds::process_requests();
    assert(removal_requests == 1 && pending_removal[5] == 7);
    complete_removal();
    assert(stored.empty());
    ble_bonds::disconnected();
}

}  // namespace

int esp_ble_get_bond_device_num() {
    return static_cast<int>(stored.size());
}

esp_err_t esp_ble_get_bond_device_list(int* count, esp_ble_bond_dev_t* list) {
    assert(*count >= static_cast<int>(stored.size()));
    std::copy(stored.begin(), stored.end(), list);
    *count = static_cast<int>(stored.size());
    return ESP_OK;
}

esp_err_t esp_ble_remove_bond_device(const std::uint8_t* address) {
    std::copy_n(address, ESP_BD_ADDR_LEN, pending_removal.begin());
    ++removal_requests;
    return ESP_OK;
}

const char* esp_err_to_name(esp_err_t) {
    return "error";
}

int main() {
    test_unique_identity_is_retained();
    test_ambiguous_or_unknown_identity_keeps_every_bond();
    test_explicit_forget_needs_connection();
}
