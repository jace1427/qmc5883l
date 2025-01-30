cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(test)
target_sources(test PRIVATE test.c)

pico_enable_stdio_usb(test 1)
pico_enable_stdio_uart(test 0)

add_subdirectory(src qmc5883l)

target_link_libraries(test PRIVATE qmc5883l pico_stdlib hardware_i2c)

pico_add_extra_outputs(test)
