cmake_minimum_required(VERSION 3.13)

add_library(qmc5883l INTERFACE)
target_sources(qmc5883l INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/qmc5883l.c)

target_link_libraries(qmc5883l INTERFACE pico_stdlib hardware_i2c)

target_include_directories(qmc5883l INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
                                              ${CMAKE_CURRENT_BINARY_DIR})
