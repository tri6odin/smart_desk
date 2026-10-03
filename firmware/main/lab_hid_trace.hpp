#pragma once

#include <cstddef>
#include <cstdint>
#include "ble_hid_host.hpp"

// Linked only by the explicit BLE HID laboratory mode, never by production.
namespace lab_hid_trace {
void report(std::uint8_t id, const std::uint8_t* bytes, std::size_t length);
void action(BleHidActionType type, int value);
}
