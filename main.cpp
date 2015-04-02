#include <Arduino.h>
#include <Wire.h>

#include <robot/system/encoders/ARIGEncodeurs.h>
#include <robot/system/motors/MD22.h>
#include <robot/system/servos/SD21.h>
#include <robot/RobotManager.h>
#include <utils/Convertion.h>
#include <utils/I2CUtils.h>

#include <adafruit/Adafruit_GFX.h>
#include <adafruit/Adafruit_SSD1306.h>

#include "define.h"

// Prototype des fonctions principale
void setup();
void matchLoop();
void endMatch();
void nextEtape();
void heartBeat();

// Prototype des fonction d'interruptions
void refreshCapteurs();
void refreshGyro();

// Prototype des fonctions business
boolean hasObstacle();
void servosHome();

// Heartbeat variables
int heartTimePrec;
int heartTime;
boolean heart;

// Classe de convertion (pour 2000 CPR, rayon 19,9 et entraxe 161)
Convertion Conv = Convertion(15.9954716675272, 44.9469570072585);

// Classe de gestion du robot (asserv, odométrie, pathfinding, evittement, etc...)
RobotManager robotManager = RobotManager();

// I2C Boards
SD21 servoManager = SD21(SD21_ADD_BOARD);
MD22 motorsPropulsion = MD22(MD22_ADD_BOARD, 0, 0);
ARIGEncodeurs encodeurs = ARIGEncodeurs(ENCODEUR_GAUCHE_BOARD, ENCODEUR_DROIT_BOARD);
Adafruit_SSD1306 lcd = Adafruit_SSD1306(OLED_RST);
// TODO : Capteurs
//PCF8574 ioGyro = PCF8574(PCF_GYRO_ADD_BOARD);
//PCF8574 ioCapteurs = PCF8574(PCF_CAPTEURS_ADD_BOARD);
//CapteursAnaI2C ioGp2D = CapteursAnaI2C(GP2D_ADD_BOARD);

// Gestion des étapes
int gestEtapes;

// ------------------------ //
// Configuration des rampes //
// ------------------------ //
const double rampAccDistance = 200.0; // en mm/s2
const double rampDecDistance = 200.0; // en mm/s2

const double rampAccOrientation = 200.0; // en mm/s2
const double rampDecOrientation = 200.0; // en mm/s2

// -------------- //
// Parametres PID //
// -------------- //
const double kpDistance = 0.30;
const double kiDistance = 0.20;
const double kdDistance = 0.90;

const double kpOrientation = 1.00;
const double kiOrientation = 0.10;
const double kdOrientation = 1.00;

// Constantes d'ajustement pour les roues folles
const double coefRoueDroite = 1.00;
const double coefRoueGauche = 1.00;

// Variable pour l'équipe
byte team;

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

// Methode de configuration pour le fonctionnement du programme
void setup() {
	// ------------------------------------------------------------- //
	// Initialisation du port série en debug seulement (cf define.h) //
	// ------------------------------------------------------------- //
#ifdef DEBUG_MODE
	Serial.begin(115200);
	Serial.println(" == Initialisation robot Elfa ==");
#endif

	// ---------- //
	// Config I2C //
	// ---------- //
	Wire.begin();
#ifdef DEBUG_MODE
	Serial.println(" - I2C [OK] (Master)");
#endif
	i2cUtils.pullup(false);
	i2cUtils.fastSpeed(false);

	// Tempo attente pour boot autres cartes
	delay(4000);
	byte nbDevices = i2cUtils.scan();
	if (nbDevices != NB_I2C_DEVICE) {
#ifdef DEBUG_MODE
		Serial.println(" [ ERROR ] Il manque des périphériques I2C. Check the connections");
#endif
		// Il manque des périphérique on bloque tout
		while(1 == 1);
	}

	// ------------------------ //
	// Initialisation ecran LCD //
	// ------------------------ //
	lcd.begin(SSD1306_SWITCHCAPVCC, OLED_LCD_ADD_BOARD);
	lcd.display();
	lcd.setTextSize(1);
	lcd.setTextColor(WHITE);

	// ------------- //
	// Servo manager //
	// ------------- //
#ifdef DEBUG_MODE
	servoManager.printVersion();
#endif

	// Configuration des vitesses
	servoManager.setSpeed(SERVO_STAB, SPEED_STAB);
	servoManager.setSpeed(SERVO_GP2D, SPEED_GP2D);
	servoManager.setSpeed(SERVO_TAPIS_HAUT, SPEED_TAPIS);
	servoManager.setSpeed(SERVO_TAPIS_BAS, SPEED_TAPIS);

	// --------------------- //
	// Moteurs de propulsion //
	// --------------------- //
#ifdef DEBUG_MODE
	motorsPropulsion.printVersion();
#endif
	motorsPropulsion.assignMotors(ASSIGN_MOTOR_2, ASSIGN_MOTOR_1);

	// --------- //
	// Encodeurs //
	// --------- //
	encodeurs.setCoefs(coefRoueGauche, coefRoueDroite);

	// ------------- //
	// Robot manager //
	// ------------- //
	robotManager.setMotorsImpl(&motorsPropulsion);
	robotManager.setEncodeursImpl(&encodeurs);
	robotManager.setHasObstacle(hasObstacle);
	robotManager.setSampleTime(TIME_ASSERV_MS);
	robotManager.setPIDDistance(kpDistance, kiDistance, kdDistance);
	robotManager.setPIDOrientation(kpOrientation, kiOrientation, kdOrientation);
	robotManager.setRampAcc(rampAccDistance, rampAccOrientation);
	robotManager.setRampDec(rampDecDistance, rampDecOrientation);
	robotManager.init();

#ifdef DEBUG_MODE
	Serial.println(" - Robot manager [OK]");
#endif

	// -- //
	// IO //
	// -- //

	// Inputs AVR numérique
	pinMode(PIN_IRQ_1_2, INPUT);
	pinMode(PIN_IRQ_3_4, INPUT);
	pinMode(PIN_IRQ_5, INPUT);
	pinMode(PIN_IRQ_6, INPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Inputs numérique AVR [OK]");
#endif

	// Inputs AVR analogique
	pinMode(EQUIPE, INPUT);
	pinMode(CURRENT_MOTA, INPUT);
	pinMode(CURRENT_MOTB, INPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Inputs analogique AVR [OK]");
#endif

	// Outputs AVR numérique
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(OLED_RST, OUTPUT);
	pinMode(DIR_MOTA, OUTPUT);
	pinMode(DIR_MOTB, OUTPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Outputs numérique AVR [OK]");
#endif

	// Outputs AVR analogique
	pinMode(PWM_R, OUTPUT);
	pinMode(PWM_G, OUTPUT);
	pinMode(PWM_B, OUTPUT);
	pinMode(PWM_MOTA, OUTPUT);
	pinMode(PWM_MOTB, OUTPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Outputs analogique (PWM) AVR [OK]");
#endif

	// Inputs Expander Capteurs


	// Inputs Expander Gyro


	// Interruptions
	attachInterrupt(ISR_1_2, refreshCapteurs, FALLING);
	attachInterrupt(ISR_3_4, refreshGyro, FALLING);

	// Configuration par défaut des variables
	heartTime = heartTimePrec = millis();
	heart = false;

	servosHome();

	// Init Gestion Etapes
	gestEtapes = 0;
}

// Point d'entrée du programme
int main(void) {
	// Initialisation du SDK Arduino. A réécrire si on veut customiser tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

	// Récupération de la couleur de l'équipe
	team = analogRead(EQUIPE) > 128 ? EQUIPE_JAUNE : EQUIPE_VERTE;
#ifdef DEBUG_MODE
	// Affichage de la couleur de l'équipe
	Serial.print(" Equipe -> ");
	Serial.println((team == EQUIPE_JAUNE) ? "JAUNE" : "VERTE");

	// Procédure d'initialisation Robot (calage, tirette, etc).
	Serial.println(" == INIT MATCH ==");
#endif

	lcd.clearDisplay();
	lcd.println("Initialisation [OK]");
	lcd.print("Equipe -> ");lcd.println((team == EQUIPE_JAUNE) ? "JAUNE" : "VERTE");
	lcd.display();
	delay(10000);
	/*if (!capteurs.readCapteurValue(TIRETTE)) {
#ifdef DEBUG_MODE
		Serial.println(" -> /!\\ La tirette n'est pas présente il faut d'abord la mettre !");
		while(!capteurs.readCapteurValue(TIRETTE));
		delay(1000);
#endif
	}*/

#ifdef DEBUG_MODE
	Serial.println(" -> Attente depart tirette ...");
#endif

	/*while(capteurs.readCapteurValue(TIRETTE)) {
		heartBeat();
		if (Serial.available()) {
			if (Serial.read() == 's') { // La touche s de la liaison série est égal à la tirette
				break;
			}
		}
	}*/

	// Démarrage du comptage
	unsigned long startMatch = millis();
	unsigned long t;

	// Reset des valeurs codeurs lors des différents mouvements de positionnement
	robotManager.resetEncodeurs();

	team = analogRead(EQUIPE) > 128 ? EQUIPE_JAUNE : EQUIPE_VERTE;
#ifdef DEBUG_MODE
	Serial.print(" - Equipe : ");
#endif
	if (team == EQUIPE_JAUNE) {
#ifdef DEBUG_MODE
		Serial.println("JAUNE");
#endif
		robotManager.setPosition(Conv.mmToPulse(2850), Conv.mmToPulse(250), Conv.degToPulse(135));
	} else {
#ifdef DEBUG_MODE
		Serial.println("VERTE");
#endif
		robotManager.setPosition(Conv.mmToPulse(150), Conv.mmToPulse(250), Conv.degToPulse(45));
	}

	// Pour tester //
	// TODO : A supprimer
	//robotManager.setPosition(0, 0, 0);
	robotManager.setPosition(Conv.mmToPulse(300), Conv.mmToPulse(300), Conv.degToPulse(90));

#ifdef DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");

	// En tête de log
	Serial.println("Gauche;Droit;X;Y;A;Type;Cons. Dist.;Cons. Orient.;PID Dist. setPoint;PID Dist. In;PID Dist. sumErr;PID Dist. Out;PID O setPoint;PID O In;PID O sumErr;PID O Out;Approche;Atteint");
#endif

	do {
		heartBeat();
		matchLoop();

		// Gestion du temps
		t = millis();
		lcd.clearDisplay();
		lcd.print("Time : ");lcd.print((int) (t - startMatch) / 1000);lcd.println(" s");
		lcd.display();
	} while(t - startMatch <= TPS_MATCH);

	// Plus de mouvement on arrete les moteurs.
	robotManager.stop();

#ifdef DEBUG_MODE
	Serial.println(" == FIN DU MATCH ==");
#endif
	while(millis() - startMatch <= END_TOUT);
	endMatch();

	// Action de clignotement de la la led built-in pour montrer que la programme fonctionne toujours.
	while(true) {
		heartBeat();
	}
}

// ---------------------------------------------------------------------------- //
// Méthode appelé encore et encore, tant que le temps du match n'est pas écoulé //
// ---------------------------------------------------------------------------- //
void matchLoop() {
	if(robotManager.getTrajetAtteint() || robotManager.getTrajetEnApproche()) {
		nextEtape();
	}

	// Processing de l'asservissement.
	robotManager.process();
}

void nextEtape(){
	switch (gestEtapes) {
	// Pour tester les valeurs de convertions
	case 0 :
		robotManager.setVitesse(300.0, 600.0);
		robotManager.gotoPointMM(300.0, 2500.0, false);
		gestEtapes++;
		break;
	case 1 :
		robotManager.setVitesse(300.0, 600.0);
		robotManager.gotoPointMM(1800.0, 2800.0, false);
		gestEtapes++;
		break;
	case 2 :
		robotManager.setVitesse(300.0, 600.0);
		robotManager.gotoPointMM(2000.0, 300.0, false);
		gestEtapes++;
		break;
	case 3 :
		robotManager.setVitesse(100.0, 300.0);
		robotManager.gotoPointMM(300.0, 300.0, true);
		gestEtapes++;
		break;
	}
}

// ----------------------------------- //
// Méthode appelé pour la fin du match //
// ----------------------------------- //
void endMatch() {
#ifdef DEBUG_MODE
	Serial.println(" == FIN MATCH ==");
#endif

	lcd.clearDisplay();
	lcd.setTextSize(4);
	lcd.println("FIN");
	lcd.display();
}

// ------------------------------------------------------- //
// ------------------ INTERRUPTION METHODS --------------- //
// ------------------------------------------------------- //

void refreshCapteurs() {

}

void refreshGyro() {

}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //

/*
 * Méthode pour le battement de coeur
 */
void heartBeat() {
	heartTime = millis();
	if (heartTime - heartTimePrec > 1000) {
		heartTimePrec = heartTime;
		digitalWrite(LED_BUILTIN, (heart) ? HIGH : LOW);
		heart = !heart;
	}
}

/*
 * Méthode pour placer les bras à la maison
 */
void servosHome() {
	servoManager.setPosition(SERVO_STAB, STAB_BAS);
	servoManager.setPosition(SERVO_GP2D, GP2D_GARAGE);
	servoManager.setPosition(SERVO_TAPIS_BAS, TAPIS_BAS_FERME);
	servoManager.setPosition(SERVO_TAPIS_HAUT, TAPIS_HAUT_FERME);
}

/*
 * Méthode retournant l'information de présence d'un obstacle (adversaire ???)
 */
boolean hasObstacle() {
	return false;
/*
	// Juste les deux de devant et les deux de dérriere
	boolean obstacle = capteurs.readCapteurValue(AVANT_DROIT)
			|| capteurs.readCapteurValue(AVANT_GAUCHE)
			|| capteurs.readCapteurValue(ARRIERE_DROIT)
			|| capteurs.readCapteurValue(ARRIERE_GAUCHE);

	if (isInPresentArea()) {
		if (team == BLEU) {
			// Les cadeaux sont a droite
			obstacle = obstacle || capteurs.readCapteurValue(LATERAL_AVANT_GAUCHE)
						|| capteurs.readCapteurValue(LATERAL_ARRIERE_GAUCHE);
		} else if (team == ROUGE) {
			// Les cadeaux sont a gauche
			obstacle = obstacle || capteurs.readCapteurValue(LATERAL_AVANT_DROIT)
						|| capteurs.readCapteurValue(LATERAL_ARRIERE_DROIT);
		}
	} else {
		// Pas dans la zone cadeaux, on active tous les capteurs avant et arriere
		obstacle = obstacle || capteurs.readCapteurValue(LATERAL_AVANT_GAUCHE)
				|| capteurs.readCapteurValue(LATERAL_AVANT_DROIT)
				|| capteurs.readCapteurValue(LATERAL_ARRIERE_GAUCHE)
				|| capteurs.readCapteurValue(LATERAL_ARRIERE_DROIT);
	}
	return obstacle;
*/
}
