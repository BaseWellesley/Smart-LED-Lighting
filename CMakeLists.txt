cmake_minimum_required(VERSION 3.13)

# Pull in SDK
# Pull in SDK (must be before project)
include(cmake/pico_sdk_import.cmake)
include(cmake/example_auto_set_url.cmake)

# Pull in FreeRTOS
include(cmake/FreeRTOS_Kernel_import.cmake)

# Set the project name and the PICO_BOARD
set(PROGRAM_NAME freertos_project)
set(PICO_BOARD pico_w)

project(freertos_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

add_compile_options(-Wall
    -Wno-format  
    -Wno-unused-function
    -Wno-maybe-uninitialized
    -Wno-unused-result  # Suppress unused result warnings
    )

# add_subdirectory(servo)
add_executable(picow_iot
    picow_iot.c
    FreeRTOSConfig.h
    lwipopts.h
    mbedtls_config.h

)

# Set the include directories for the target

target_include_directories(picow_iot PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(picow_iot
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap1
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
    pico_lwip_mbedtls
    hardware_gpio
    pico_mbedtls
    pico_lwip_mqtt
    hardware_adc
    hardware_pwm
    pico_multicore
)

# enable usb output, disable uart output
pico_enable_stdio_usb(picow_iot 1)
pico_enable_stdio_uart(picow_iot 0)
pico_add_extra_outputs(picow_iot)
