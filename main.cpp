#include <Arduino.h>
#include <Wire.h>
#include <utils/Convertion.h>
#include <robot/RobotManager.h>

#include "define.h"

// Prototype des fonctions
void setup();
void loop();

// Classe de convertion
Convertion Conv = Convertion(4.044, 11.36);
RobotManager RM = RobotManager();

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

// Point d'entrée du programme
int main(void) {
	// Initialisation du SDK Arduino. A réécrire si on veut customisé tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

	while(true) {
		// Boucle infinie pour le fonctionnement.
		loop();
	}
}

// Method de configuration pour le fonctionnement du programme
void setup() {
	// ------------------------------------------------------------- //
	// Initialisation du port série en debug seulement (cf define.h) //
	// ------------------------------------------------------------- //
#ifdef DEBUG_MODE
	Serial.begin(115200);
	Serial.println(" == INITIALISATION ELFA ==");
#endif

	// ------------------------- //
	// Définition des broches IO //
	// ------------------------- //
	/*pinMode(ADD1, INPUT);*/

#ifdef DEBUG_MODE
	Serial.println(" - IO [OK]");
#endif

	// ------------------------ //
	// Configuration du bus I2C //
	// ------------------------ //
	Wire.begin();
#ifdef DEBUG_MODE
		Serial.println(" - I2C [OK] (Master)");
#endif

	RM.init();
}

// Méthode appelé encore et encore, tant que la carte reste alimenté.
void loop() {
	// TODO : IA pour le robot

	// Processing de l'asservissement.
	RM.process();
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //

// Réinitialisation des valeurs de comptage
void resetEncodeursValues() {
	// Reset sur les cartes codeurs
	// TODO : Ajouter dans le robot manager
}
