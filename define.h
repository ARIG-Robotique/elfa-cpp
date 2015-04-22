/*
 * define.h
 *
 *  Created on: 27 janv. 2013
 *      Author: mythril
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#define VERSION						1

//#define MAIN_DEBUG_MODE

#define TPS_MATCH                   89800 // 89,8 sec pour palier au pb de réaction du bonhomme
#define END_TOUT					90000 // 90 sec c'est vraiment la fin de tout

#define EQUIPE_VERTE				true
#define EQUIPE_JAUNE				false

// ---------------------- //
// Adresse des cartes I2C //
// ---------------------- //
#define NB_I2C_DEVICE				12

#define OLED_LCD_ADD_BOARD			0x3C
#define PCF_GYRO_ADD_BOARD			0x3E
#define PCF_CAPTEURS_ADD_BOARD		0x3F
#define GP2D_ADD_BOARD				0x48
#define MD22_ADD_BOARD				0x58
#define SD21_ADD_BOARD				0x61
#define ENCODEUR_DROIT_BOARD	  	0xB0
#define ENCODEUR_GAUCHE_BOARD	 	0xB2

// --------------------------------- //
// Configuration de l'asservissement //
// --------------------------------- //

#define TIME_ASSERV_MS				10

// -------------------------------- //
// Configuration moteurs propulsion //
// -------------------------------- //

#define LEFT_MOTOR					ASSIGN_MOTOR_1
#define RIGHT_MOTOR					ASSIGN_MOTOR_2

// ------------------------------- //
// Configuration des servo moteurs //
// ------------------------------- //

#define SERVO_STAB					1
#define SERVO_GP2D					2
#define SERVO_TAPIS_HAUT			3
#define SERVO_TAPIS_BAS				4

#define SPEED_STAB					15
#define SPEED_GP2D					25
#define SPEED_TAPIS					5

#define STAB_BAS					620
#define STAB_HAUT                   2000

#define GP2D_GARAGE					1010
#define GP2D_MATCH 					1120
#define GP2D_ESCALIER				1840

#define TAPIS_HAUT_OUVERT			1500
#define TAPIS_HAUT_FERME			1500
#define TAPIS_BAS_OUVERT			1500
#define TAPIS_BAS_FERME				1500

// ------------------------------------ //
// Gestion de la stabilisation escalier //
// ------------------------------------ //

#define SENS_BEQUILLE_MONTE			LOW
#define SENS_BEQUILLE_DESCENT		HIGH

// --------------- //
// IO des capteurs //
// --------------- //

// Output
#define OLED_RST       		4

#define PWM_R          		10
#define PWM_G          		9
#define PWM_B          		8

#define PWM_MOTA       		11
#define DIR_MOTA       		12
#define PWM_MOTB       		6
#define DIR_MOTB       		7

// Input
#define PIN_IRQ_1_2    		2
#define PIN_IRQ_3_4    		3
#define PIN_IRQ_5      		18
#define PIN_IRQ_6      		19
#define EQUIPE         		A0
#define CURRENT_MOTA   		A1
#define CURRENT_MOTB   		A2

// Input Expanders
#define SW_BEQUILLE	   		0
#define SW_TIRETTE     		1

// Input Analogique
#define GP2D_DROIT_COTE		0
#define GP2D_DROIT_FRONT	1
#define GP2D_GAUCHE_FRONT	2
#define GP2D_GAUCHE_COTE	3

// Interrupts
#define ISR_1_2        		0
#define ISR_3_4        		1
#define ISR_5          		5
#define ISR_6          		4

#endif /* DEFINE_H_ */
