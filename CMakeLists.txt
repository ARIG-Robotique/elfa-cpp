#=============================================================================#
# Author: Gregory DEPUILLE                                                    #
# Date:   13/05/2016                                                          #
# Description: Arduino CMake Robot Elfa                                       #
#=============================================================================#

# Inclusions des .h de la lib
include_directories(../robot-system-lib/src)

# Génération du programme par défaut
generate_arduino_firmware(ElfaMega
    SRCS src/main.cpp
    HDRS src/define.h
    ARDLIBS Adafruit_GFX Adafruit_SSD1306
    LIBS RobotMega
    BOARD mega2560
)