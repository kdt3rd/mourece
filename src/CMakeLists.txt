
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(mourece)

pico_sdk_init()

add_executable(mourece
  main.c
)

target_link_libraries(mourece pico_stdlib hardware_spi)
pico_enable_stdio_usb(mourece 1)
# could do serial output instead (or both)
pico_enable_stdio_uart(mourece 0)

pico_add_extra_outputs(mourece)

