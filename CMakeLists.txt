# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)


# Alterar para o caminho de onde esta o kernel do seu FreeRtos
set(FREERTOS_KERNEL_PATH "C:/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)


project(EstacaoAlertaEnchenteMultitarefa C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

include_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(EstacaoAlertaEnchenteMultitarefa EstacaoAlertaEnchenteMultitarefa.c lib/ssd1306.c)

pico_set_program_name(EstacaoAlertaEnchenteMultitarefa "EstacaoAlertaEnchenteMultitarefa")
pico_set_program_version(EstacaoAlertaEnchenteMultitarefa "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(EstacaoAlertaEnchenteMultitarefa 0)
pico_enable_stdio_usb(EstacaoAlertaEnchenteMultitarefa 1)

# Generate PIO header
pico_generate_pio_header(EstacaoAlertaEnchenteMultitarefa ${CMAKE_CURRENT_LIST_DIR}/lib/ws2812.pio)

# Add the standard library to the build
target_link_libraries(EstacaoAlertaEnchenteMultitarefa
        hardware_pwm
        hardware_adc
        hardware_i2c
        hardware_pio
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4
        pico_stdlib
        )

# Add the standard include files to the build
target_include_directories(EstacaoAlertaEnchenteMultitarefa PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(EstacaoAlertaEnchenteMultitarefa)

