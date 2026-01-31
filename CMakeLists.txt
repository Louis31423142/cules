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
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(mpu6050_i2c C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add libary
add_library(mpu6050_i2c_lib INTERFACE)
target_sources(mpu6050_i2c_lib INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/mpu6050_i2c_lib.c
        )
target_include_directories(mpu6050_i2c_lib INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(mpu6050_i2c_lib INTERFACE
    pico_stdlib
    hardware_i2c
        )

# Add executable. Default name is the project name, version 0.1

add_executable(mpu6050_i2c
        example.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(mpu6050_i2c
        pico_stdlib 
        mpu6050_i2c_lib
        )

# enable usb output
pico_enable_stdio_usb(mpu6050_i2c 1)
pico_enable_stdio_uart(mpu6050_i2c 0)

target_include_directories(mpu6050_i2c PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
    )

# create map/bin/hex file etc.
pico_add_extra_outputs(mpu6050_i2c)
