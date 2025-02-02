cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(calibrate C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(calibrate)
target_sources(calibrate PRIVATE calibrate.c)

pico_enable_stdio_usb(calibrate 1)
pico_enable_stdio_uart(calibrate 0)

add_subdirectory("../../src" qmc5883l)

target_link_libraries(calibrate PRIVATE qmc5883l pico_stdlib hardware_i2c)

pico_add_extra_outputs(calibrate)
