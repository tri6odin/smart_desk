cmake_minimum_required(VERSION 3.16)
project(smart_desk_protocol_tests LANGUAGES CXX)

enable_testing()

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
