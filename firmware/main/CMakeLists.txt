set(main_sources "app_main.cpp")
if(CONFIG_DESK_MODE_BLE_CONTROLLER)
    set(main_sources "production_main.cpp" "preset_store.cpp"
        "runtime_diagnostics.cpp")
endif()
list(APPEND main_sources "uart_transport.cpp")

if(CONFIG_DESK_MODE_BLE_HID_LOGGER OR CONFIG_DESK_MODE_BLE_ONE_SHOT_TEST OR
   CONFIG_DESK_MODE_BLE_CALIBRATION OR CONFIG_DESK_MODE_BLE_CONTROLLER)
    list(APPEND main_sources "ble_hid_logger.cpp" "ble_bonds.cpp")
endif()

idf_component_register(
    SRCS ${main_sources}
    INCLUDE_DIRS "."
    REQUIRES desk_controller desk_protocol esp_driver_uart esp_driver_gpio
    # Component requirements are expanded before project Kconfig values are
    # available, so these cannot be conditional on DESK_MODE_* here.  Keeping
    # them unconditional only exposes their headers; non-BLE modes still omit
    # ble_hid_logger.cpp and do not link live Bluetooth code.
    PRIV_REQUIRES bt esp_hid nvs_flash esp_timer
)
