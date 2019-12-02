#=============================================================================#
# Author: Gregory DEPUILLE                                                    #
# Date:   13/05/2016                                                          #
# Description: Arduino CMake Robot Elfa                                       #
#=============================================================================#

# Contrôle de la version minimale de CMake
cmake_minimum_required(VERSION 3.8.2)

# Définition de la tool chain de build
set(CMAKE_TOOLCHAIN_FILE ./cmake/Arduino-Toolchain.cmake)

# Fix bug in avrdude.conf included in ArduinoSDK
set(ARDUINO_CMAKE_AVRDUDE_CONFIG_PATH /etc/avrdude/avrdude.conf)

# Compilateur flags
#set(ARDUINO_C_FLAGS      "-ffunction-sections -fdata-sections")
#set(ARDUINO_CXX_FLAGS    "${ARDUINO_C_FLAGS} -fno-exceptions")
#set(ARDUINO_LINKER_FLAGS "-Wl,--gc-sections")

set(ARDUINO_CXX_FLAGS    "-std=c++11")

if (NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message(STATUS "Debug mode ON")
    add_definitions(-DDEBUG_MODE)
endif()

#====================================================================#
#  Setup Project                                                     #
#====================================================================#

project(ElfaRobot C CXX ASM)
arduino_cmake_project(ElfaRobot BOARD_NAME megaADK BOARD_CPU atmega2560)

# Build
add_arduino_executable(ElfaRobot src/main.cpp src/define.h)

# Wire Lib
find_arduino_library(Wire Wire)
link_arduino_library(ElfaRobot Wire)

# RobotSystemLib
find_arduino_library(RobotSystem robot-system-lib)
link_arduino_library(ElfaRobot RobotSystem)

# Adafruit Libs
find_arduino_library(AdafruitGFX Adafruit_GFX 3RD_PARTY)
find_arduino_library(AdafruitSSD1306 Adafruit_SSD1306 3RD_PARTY)

link_arduino_library(AdafruitSSD1306 AdafruitGFX)
link_arduino_library(ElfaRobot AdafruitGFX)
link_arduino_library(ElfaRobot AdafruitSSD1306)

# Upload
set_target_upload_port(ElfaRobot /dev/ttyACM0)
