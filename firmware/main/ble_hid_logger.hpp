#pragma once

enum class BleHidActionType {
    kDialRaw,
    kDialDelta,
    kStop,
    kGoSit,
    kGoStand,
    kSaveSit,
    kSaveStand,
    kDisconnected,
};

struct BleHidAction {
    BleHidActionType type;
    int value;
};

using BleHidActionCallback = void (*)(const BleHidAction& action,
                                      void* context);

// A null callback preserves the logger-only safety boundary: HID input is
// decoded and logged but cannot reach the desk motion layer.
void start_ble_hid_logger(BleHidActionCallback callback = nullptr,
                          void* callback_context = nullptr,
                          bool publish_delayed_dial_delta = true);
