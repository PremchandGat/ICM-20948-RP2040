cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(icm_read C CXX ASM)

set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(icm_read
    icm_read.cpp
)

# pull in common dependencies
target_link_libraries(icm_read pico_stdlib hardware_uart hardware_spi)
 
# create map/bin/hex file etc.
pico_add_extra_outputs(icm_read)

pico_enable_stdio_uart(icm_read 1)