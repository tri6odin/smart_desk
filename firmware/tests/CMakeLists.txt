cmake_minimum_required(VERSION 3.16)
project(smart_desk_protocol_tests LANGUAGES CXX)

enable_testing()

option(DESK_ENABLE_SANITIZERS "Check host tests with AddressSanitizer and UBSan" OFF)
set(DESK_SANITIZERS "address,undefined" CACHE STRING "Sanitizers supported by the host runtime")
if(DESK_ENABLE_SANITIZERS)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        message(FATAL_ERROR "DESK_ENABLE_SANITIZERS requires Clang or GCC")
    endif()
    add_compile_options(-fsanitize=${DESK_SANITIZERS} -fno-omit-frame-pointer)
    add_link_options(-fsanitize=${DESK_SANITIZERS})
endif()

add_library(desk_protocol
    ../components/desk_protocol/desk_protocol.cpp
)
target_include_directories(desk_protocol PUBLIC
    ../components/desk_protocol/include
)
target_compile_features(desk_protocol PUBLIC cxx_std_17)
target_compile_options(desk_protocol PRIVATE -Wall -Wextra -Werror -Wpedantic)

add_executable(test_desk_protocol test_desk_protocol.cpp)
target_link_libraries(test_desk_protocol PRIVATE desk_protocol)
target_compile_options(test_desk_protocol PRIVATE -Wall -Wextra -Werror -Wpedantic)

add_test(NAME desk_protocol COMMAND test_desk_protocol)

add_library(desk_controller
    ../components/desk_controller/desk_controller.cpp
    ../components/desk_controller/desk_inputs.cpp
    ../components/desk_controller/desk_recovery.cpp
)
target_include_directories(desk_controller PUBLIC
    ../components/desk_controller/include
)
target_link_libraries(desk_controller PUBLIC desk_protocol)
target_compile_features(desk_controller PUBLIC cxx_std_17)
target_compile_options(desk_controller PRIVATE -Wall -Wextra -Werror -Wpedantic)

add_executable(test_desk_controller test_desk_controller.cpp)
target_link_libraries(test_desk_controller PRIVATE desk_controller)
target_compile_options(test_desk_controller PRIVATE -Wall -Wextra -Werror -Wpedantic)

add_test(NAME desk_controller COMMAND test_desk_controller)

add_executable(test_desk_inputs test_desk_inputs.cpp)
target_link_libraries(test_desk_inputs PRIVATE desk_controller)
target_compile_options(test_desk_inputs PRIVATE -Wall -Wextra -Werror -Wpedantic)
add_test(NAME desk_inputs COMMAND test_desk_inputs)

add_executable(test_ble_bonds test_ble_bonds.cpp ../main/ble_bonds.cpp)
target_include_directories(test_ble_bonds PRIVATE stubs ../main)
target_compile_features(test_ble_bonds PRIVATE cxx_std_17)
target_compile_options(test_ble_bonds PRIVATE -Wall -Wextra -Werror -Wpedantic)
add_test(NAME ble_bonds COMMAND test_ble_bonds)

set_tests_properties(desk_protocol desk_controller desk_inputs ble_bonds
    PROPERTIES TIMEOUT 30)
if(DESK_ENABLE_SANITIZERS)
    # UBSan normally recovers after some errors; CI must fail on the first one.
    set_tests_properties(desk_protocol desk_controller desk_inputs ble_bonds PROPERTIES
        ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
endif()
