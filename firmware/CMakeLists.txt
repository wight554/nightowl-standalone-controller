cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(nightowl_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GEN_SCRIPT    ${CMAKE_CURRENT_LIST_DIR}/../scripts/gen_motor_config.py)
set(MOTOR_INI     ${CMAKE_CURRENT_LIST_DIR}/../config.ini)
set(MOTOR_CONFIG_H ${CMAKE_CURRENT_LIST_DIR}/include/motor_config.h)

add_custom_command(
    OUTPUT  ${MOTOR_CONFIG_H}
    COMMAND ${Python3_EXECUTABLE} ${GEN_SCRIPT} ${MOTOR_INI} ${MOTOR_CONFIG_H}
    DEPENDS ${MOTOR_INI} ${GEN_SCRIPT}
    COMMENT "Generating motor_config.h from config.ini"
)
add_custom_target(motor_config DEPENDS ${MOTOR_CONFIG_H})

add_executable(nightowl_controller
  src/main.c
  src/tmc2209.c
  src/neopixel.c
)

target_include_directories(nightowl_controller PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_generate_pio_header(nightowl_controller
  ${CMAKE_CURRENT_LIST_DIR}/third_party/ws2812.pio
)

add_dependencies(nightowl_controller motor_config)

target_link_libraries(nightowl_controller
  pico_stdlib
  hardware_gpio
  hardware_irq
  hardware_pwm
  hardware_clocks
  hardware_flash
  hardware_sync
  hardware_pio
)

pico_enable_stdio_usb(nightowl_controller 1)
pico_enable_stdio_uart(nightowl_controller 0)

pico_add_extra_outputs(nightowl_controller)
