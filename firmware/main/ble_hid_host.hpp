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
    kUserActivity,
    kStartCalibration,
};

struct BleHidAction {
    BleHidActionType type;
    int value;
};

using BleHidActionCallback = void (*)(const BleHidAction& action,
                                      void* context);

// A null callback keeps decoded HID input disconnected from desk motion.
// Live report tracing is linked only by the explicit lab HID mode.
void start_ble_hid_host(BleHidActionCallback callback = nullptr,
                          void* callback_context = nullptr,
                          bool publish_delayed_dial_delta = true);
