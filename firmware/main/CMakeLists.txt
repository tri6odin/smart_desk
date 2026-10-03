set(main_sources "app_main.cpp")
if(CONFIG_DESK_MODE_BLE_CONTROLLER)
    set(main_sources "production_main.cpp" "preset_store.cpp"
        "persistent_journal.cpp")
endif()
list(APPEND main_sources "uart_transport.cpp")

if(CONFIG_DESK_MODE_BLE_HID_LOGGER OR CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST OR
   CONFIG_DESK_MODE_BLE_CALIBRATION OR CONFIG_DESK_MODE_BLE_CONTROLLER)
    list(APPEND main_sources "ble_hid_host.cpp" "ble_bonds.cpp")
endif()

if(CONFIG_DESK_MODE_BLE_HID_LOGGER)
    list(APPEND main_sources "lab_hid_trace.cpp")
endif()

idf_component_register(
    SRCS ${main_sources}
    INCLUDE_DIRS "."
    REQUIRES desk_controller desk_protocol desk_journal esp_driver_uart esp_driver_gpio
    # Component requirements are expanded before project Kconfig values are
    # available, so these cannot be conditional on DESK_MODE_* here.  Keeping
    # them unconditional only exposes their headers; non-BLE modes still omit
    # ble_hid_host.cpp and do not link live Bluetooth code.
    PRIV_REQUIRES bt esp_hid nvs_flash esp_timer esp_partition esp_app_format
)
