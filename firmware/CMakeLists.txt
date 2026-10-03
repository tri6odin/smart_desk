cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smart_desk)

if(CONFIG_DESK_MODE_BLE_CONTROLLER AND
   (NOT CONFIG_PARTITION_TABLE_CUSTOM OR
    NOT CONFIG_PARTITION_TABLE_CUSTOM_FILENAME STREQUAL "partitions.journal.csv"))
    message(FATAL_ERROR "Production requires partitions.journal.csv. Regenerate the ignored sdkconfig from sdkconfig.ble-controller.defaults; see docs/persistent-journal.md.")
endif()

if(CONFIG_DESK_MODE_BLE_CONTROLLER AND
   (NOT CONFIG_ESP_CONSOLE_NONE OR NOT CONFIG_ESP_CONSOLE_SECONDARY_NONE OR
    NOT CONFIG_LOG_DEFAULT_LEVEL EQUAL 0 OR
    NOT CONFIG_LOG_MAXIMUM_LEVEL EQUAL 0 OR
    NOT CONFIG_BOOTLOADER_LOG_LEVEL EQUAL 0))
    message(FATAL_ERROR "Production has no live console diagnostics. Use a fresh SDKCONFIG from sdkconfig.ble-controller.defaults; see firmware/README.md.")
endif()
