# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ILI9488_Init C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ILI9488_Init
    main.c
    ILI9488.c
    PIO_SPI_16_24.c)

pico_set_program_name(ILI9488_Init "ILI9488_Init")
pico_set_program_version(ILI9488_Init "0.1")

# Generate PIO header
#pico_generate_pio_header(ILI9488_Init ${CMAKE_CURRENT_LIST_DIR}/pio_spi_test.pio)


# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ILI9488_Init 0)
pico_enable_stdio_usb(ILI9488_Init 1)

# Add the standard library to the build
target_link_libraries(ILI9488_Init
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(ILI9488_Init PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(ILI9488_Init 
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_clocks
        )

pico_add_extra_outputs(ILI9488_Init)

