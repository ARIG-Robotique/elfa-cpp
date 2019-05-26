/*
 * define.h
 *
 *  Created on: 27 janv. 2013
 *      Author: mythril
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#define VERSION               2019

#define NB_INIT_STEP          7

#define MAX_RAW_GP            300
#define MIN_RAW_GP            90
#define SEUIL_PRESENCE_ROBOT  65 // En cm (a peu près vrai)

// ---------------------- //
// Adresse des cartes I2C //
// ---------------------- //
#define NB_I2C_DEVICE         2

#define OLED_LCD_ADD_BOARD    0x3C
#define SD21_ADD_BOARD        0x61

// ------------------------------- //
// Configuration des servo moteurs //
// ------------------------------- //
#define SERVO_ASC_NB       1

// Valeur ascenseur electron
#define SPEED_ASC          10
#define ASC_HAUT           2190
#define ASC_HAUT_MOINS     1970
#define ASC_BAS     	   700

// --------------- //
// IO des capteurs //
// --------------- //

// Output
#define OLED_RST       		4
#define BANDEAU_LED         8

// Input
#define AU                  15
#define SELECT_MODE         16
#define SELECT_POSITION     17
#define GP2D         		A0

enum CheckRobot {
    PARTI,
    PRESENT
};

enum Mode {
    AUTO,
    MANUEL
};

enum AscenseurMode {
    HAUT, PULSE
};

struct GP2D12Result {
    float raw;
    float cm;
};

#endif /* DEFINE_H_ */
