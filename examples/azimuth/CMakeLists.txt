cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(azimuth C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(azimuth)
target_sources(azimuth PRIVATE azimuth.c)

pico_enable_stdio_usb(azimuth 1)
pico_enable_stdio_uart(azimuth 0)

add_subdirectory("../../src" qmc5883l)

target_link_libraries(azimuth PRIVATE qmc5883l pico_stdlib hardware_i2c)

pico_add_extra_outputs(azimuth)
