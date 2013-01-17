/*
 * define.h
 *
 *  Created on: 1 janv. 2013
 *      Author: mythril
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#define VERSION			1

// Mode debug : 1 => ON ; 0 => OFF
#define DEBUG_MODE 		1

// Definition du mode de comptage
// 1 : CHA front montant
// 2 : CHA et CHB front montant
// 4 : CHA et CHB change
#define MULT_MODE_1X	1
#define MULT_MODE_2X	2
#define MULT_MODE_4X	4
#define MULT_MODE_SEL	MULT_MODE_1X

// Addresse des composants I2C
#define ADD_CODEUR_DROIT 	0xB0
#define ADD_CODEUR_GAUCHE	0xB2

// Pin des IO
#define CDX_DROIT_SORTIE	1
#define CDX_DROIT_RENTRE	2
#define CDX_GAUCHE_SORTIE	3
#define CDX_GAUCHE_RENTRE	4
// TODO : Ajouter les GP2D

// Pin pour les moteurs
#define MOT_DROIT_VITESSE	1
#define MOT_DROIT_SENS		1
#define MOT_DROIT_BRAKE		1
#define MOT_DROIT_COURANT	A1
#define MOT_DROIT_ALERT		1

#define MOT_GAUCHE_VITESSE	1
#define MOT_GAUCHE_SENS		1
#define MOT_GAUCHE_BRAKE	1
#define MOT_GAUCHE_COURANT	A2
#define MOT_GAUCHE_ALERT	1

// Pin pour les Servo moteurs
#define SERVO_VERRE			12

// Position pour les servos
#define SERVO_VERRE_OUVERT	50
#define SERVO_VERRE_FERME	50

#endif /* DEFINE_H_ */
