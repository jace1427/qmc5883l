cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(calibrated C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(calibrated)
target_sources(calibrated PRIVATE calibrated.c)

pico_enable_stdio_usb(calibrated 1)
pico_enable_stdio_uart(calibrated 0)

add_subdirectory("../../src" qmc5883l)

target_link_libraries(calibrated PRIVATE qmc5883l pico_stdlib hardware_i2c)

pico_add_extra_outputs(calibrated)
