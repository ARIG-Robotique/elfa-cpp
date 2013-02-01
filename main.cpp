#include <Arduino.h>
#include <Wire.h>
#include <robot/utils/Convertion.h>
#include <robot/RobotManager.h>

#include "define.h"

// Prototype des fonctions
void setup();
void loop();

// Classe de convertion
Convertion Conv = Convertion(4.044, 11.36);

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

// Point d'entr�e du programme
int main(void) {
	// Initialisation du SDK Arduino. A r��crire si on veut customis� tout le bouzin.
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
	// Initialisation du port s�rie en debug seulement (cf define.h) //
	// ------------------------------------------------------------- //
#ifdef DEBUG_MODE
	Serial.begin(115200);
	Serial.println(" == INITIALISATION PETIT ROBOT ==");
#endif

	// ------------------------- //
	// D�finition des broches IO //
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
}

// M�thode appel� encore et encore, tant que la carte reste aliment�.
void loop() {
	// TODO : IA pour le robot

	// Processing de l'asservissement.
	RM.process();
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //

// R�initialisation des valeurs de comptage
void resetEncodeursValues() {
	// Reset sur les cartes codeurs
	// TODO : Ajouter �a dans le robot manager
}
