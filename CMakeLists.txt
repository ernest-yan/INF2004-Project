cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_line_barcode C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# 1. Update paths to look inside src/
add_executable(pico_integration_example
    src/pico_integration_example.c
    src/line_barcode.c
)

# 2. Tell CMake where to find header files in include/
target_include_directories(pico_integration_example PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(pico_integration_example
    pico_stdlib
)

pico_enable_stdio_usb(pico_integration_example 1)
pico_enable_stdio_uart(pico_integration_example 0)

pico_add_extra_outputs(pico_integration_example)