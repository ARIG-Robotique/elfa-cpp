#=============================================================================#
# Author: Gregory DEPUILLE                                                    #
# Date:   13/05/2016                                                          #
# Description: Arduino CMake Robot Elfa                                       #
#=============================================================================#

# Contrôle de la version minimale de CMake
cmake_minimum_required(VERSION 3.8.2)

# Pas d'upload par défaut
if (NOT DEFINED ARDUINO_UPLOAD)
    set(ARDUINO_UPLOAD false)
endif()

# Définition de la tool chain de build
set(CMAKE_TOOLCHAIN_FILE ./cmake/Arduino-Toolchain.cmake)

# Compilateur flags
#set(ARDUINO_C_FLAGS      "-ffunction-sections -fdata-sections")
#set(ARDUINO_CXX_FLAGS    "${ARDUINO_C_FLAGS} -fno-exceptions")
#set(ARDUINO_LINKER_FLAGS "-Wl,--gc-sections")

set(ARDUINO_CXX_FLAGS    "-std=c++11")

#add_definitions(-DDEBUG_MODE)

#====================================================================#
#  Setup Project                                                     #
#====================================================================#

project(ElfaRobot C CXX ASM)

# Récupération de la board
get_board_id(board_id mega atmega2560)

# Build
add_arduino_executable(ElfaRobot ${board_id} src/main.cpp src/define.h)

# RobotSystemLib
file(GLOB_RECURSE ROBOT_SYSTEM_LIB_SOURCES "./robot-system-lib/src/*.cpp" "./robot-system-lib/src/*.c")
file(GLOB_RECURSE ROBOT_SYSTEM_LIB_HEADERS "./robot-system-lib/src/*.h")
add_library(RobotSystem STATIC ${ROBOT_SYSTEM_LIB_SOURCES} ${ROBOT_SYSTEM_LIB_HEADERS})
target_include_directories(RobotSystem PUBLIC ./robot-system-lib/src)
link_arduino_library(ElfaRobot RobotSystem ${board_id})

# Wire Lib
link_platform_library(RobotSystem Wire ${board_id})
link_platform_library(ElfaRobot Wire ${board_id})

# Adafruit Libs
find_arduino_library(AdafruitGFX Adafruit_GFX ${board_id} 3RD_PARTY)
link_arduino_library(ElfaRobot AdafruitGFX ${board_id})
find_arduino_library(AdafruitSSD1306 Adafruit_SSD1306 ${board_id} 3RD_PARTY)
link_arduino_library(AdafruitSSD1306 AdafruitGFX ${board_id})
link_arduino_library(ElfaRobot AdafruitSSD1306 ${board_id})

# Upload
if (ARDUINO_UPLOAD)
    message("-- Upload enabled")
    upload_arduino_target(ElfaRobot "${board_id}" /dev/ttyUSB0)
else()
    message("-- Upload disabled")
endif()
