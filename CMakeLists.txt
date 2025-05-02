cmake_minimum_required(VERSION 3.13)
project(mavlink_projec)

add_executable(mavlink main.cpp)

target_compile_options(mavlink PRIVATE )#-Waddress-of-packed-member)

set_target_properties(mavlink PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

