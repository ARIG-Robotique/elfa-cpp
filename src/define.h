/*
 * define.h
 *
 *  Created on: 27 janv. 2013
 *      Author: mythril
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#define VERSION              2017

#define TPS_MATCH            90000 // 90 sec pour palier au pb de réaction du bonhomme
#define END_TOUT             95000 // 95 sec c'est vraiment la fin de tout

#define SEUIL_PRESENCE_ROBOT  35 // En cm (a peu près vrai)
#define TPS_CYCLE_DEPOSE_FULL 80000 // En ms temps a partir duquel on ne revient pas
#define TPS_CYCLE_ANNULE      85000 // En ms temps ou on lache l'affaire

// ---------------------- //
// Adresse des cartes I2C //
// ---------------------- //
#define NB_I2C_DEVICE         2

#define OLED_LCD_ADD_BOARD    0x3C
#define SD21_ADD_BOARD        0x61

// ------------------------------- //
// Configuration des servo moteurs //
// ------------------------------- //
#define SERVO_ASC_NB       6
#define SERVO_INC_NB       7

// Valeur + = monte
#define SPEED_ASC          0
#define ASC_START          1140
#define ASC_BAS     	   580
#define ASC_PRE_DEPOSE     1570
#define ASC_DEPOSE         2370

// Valeur - = anti horaire
#define SPEED_INC_NORM     0
#define SPEED_INC_COMB     15
#define INC_START     	   1580
#define INC_PRISE          2430
#define INC_PRE_DEPOSE     970
#define INC_DEPOSE         550

// --------------------- //
// Configuration moteurs //
// --------------------- //
#define LOW_SPEED          10
#define HIGH_SPEED         255
#define STOP_SPEED         0

// --------------- //
// IO des capteurs //
// --------------- //

// Output
#define OLED_RST       		4

#define PWM_HELICE         	8
#define IN2_HELICE       	17

// Input
#define AU                  15
#define TIRETTE             16
#define GP2D         		A0

enum CheckRobot {
    PAS_PRESENT,
    PRESENT
};

#endif /* DEFINE_H_ */
