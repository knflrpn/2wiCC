# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(2wiCC C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(2wiCC
    src/2wiCC.c
    src/usb_descriptors.c
)

pico_set_program_name(2wiCC "2wiCC")
pico_set_program_version(2wiCC "0.1")

# Generate PIO header
pico_generate_pio_header(2wiCC ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

# No std output
pico_enable_stdio_uart(2wiCC 0)
pico_enable_stdio_usb(2wiCC 0)
target_compile_definitions(2wiCC PRIVATE
  PICO_DEFAULT_UART=0
  PICO_DEFAULT_UART_TX_PIN=0
  PICO_DEFAULT_UART_RX_PIN=1
)

# Add the standard library to the build
target_link_libraries(2wiCC
        pico_stdlib
    )

# Add the standard include files to the build
target_include_directories(2wiCC PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# Add any user requested libraries
target_link_libraries(2wiCC 
        hardware_spi
        hardware_pio
        hardware_timer
        pico_multicore
        tinyusb_device
        tinyusb_board 
        )

pico_add_extra_outputs(2wiCC)

