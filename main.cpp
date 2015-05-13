#include <Arduino.h>
#include <Wire.h>

#include <robot/system/encoders/ARIGEncodeurs.h>
#include <robot/system/capteurs/BoardPCF8574.h>
#include <robot/system/capteurs/BoardI2CADC.h>
#include <robot/system/motors/PWMMotor.h>
#include <robot/system/motors/SD21Motors.h>
#include <robot/system/servos/SD21.h>
#include <robot/RobotManager.h>
#include <utils/Convertion.h>
#include <utils/I2CUtils.h>
#include <filters/Pid.h>

// Ecran LCD
#include <adafruit/Adafruit_GFX.h>
#include <adafruit/Adafruit_SSD1306.h>

#include "define.h"

// Prototype des fonctions principale
void setup();
void matchLoop();
void endMatch();
void nextEtape();
void heartBeat();

// Prototype des fonctions business
//float read_gp2d12_range(int reading);
boolean hasObstacle();
boolean lectureEquipe();
int averageLatGauche(int newValue);
int averageGauche(int newValue);
int averageDroit(int newValue);
int averageLatDroit(int newValue);

// Heartbeat variables
int heartTimePrec;
int heartTime;
boolean heart;

// Classe de convertion (pour 500 CPR x 4)
// Calcul angle : 360° = 6405 p => 17.791666667 p/°
// Calcul distance : 1544mm = 10000 p => 6.476683938 p/mm
Convertion Conv = Convertion(6.476683938, 17.791666667);

// Moteur pour la béquille
PWMMotor motBequille = PWMMotor(DIR_MOTB, PWM_MOTB, CURRENT_MOTB);

// Classe de gestion du robot (asserv, odométrie, pathfinding, evittement, etc...)
RobotManager robotManager = RobotManager();

// I2C Boards
SD21 servoManager = SD21(SD21_ADD_BOARD);
SD21Motors motorsPropulsion = SD21Motors(SD21_ADD_BOARD);
ARIGEncodeurs encodeurs = ARIGEncodeurs(ENCODEUR_GAUCHE_BOARD, ENCODEUR_DROIT_BOARD);
Adafruit_SSD1306 lcd = Adafruit_SSD1306(OLED_RST);
BoardPCF8574 ioGyro = BoardPCF8574("Gyro", PCF_GYRO_ADD_BOARD);
BoardPCF8574 ioCapteurs = BoardPCF8574("Num", PCF_CAPTEURS_ADD_BOARD);
BoardI2CADC ioGp2D = BoardI2CADC(GP2D_ADD_BOARD);

// Gestion des étapes
int gestEtapes;

// ------------------------ //
// Configuration des rampes //
// ------------------------ //
const double rampAccDistance = 800.0; // en mm/s2
const double rampDecDistance = 800.0; // en mm/s2

const double rampAccOrientation = 800.0; // en mm/s2
const double rampDecOrientation = 800.0; // en mm/s2

// -------------- //
// Parametres PID //
// -------------- //
const double kpD = 1.00;
const double kiD = 0.75;
const double kdD = 0.25;

const double kpO = 1.00;
const double kiO = 0.50;
const double kdO = 0.00;

// Constantes d'ajustement pour les roues folles
const double coefRoueDroite = 1.00;
const double coefRoueGauche = 1.00;

// Variable pour l'équipe
boolean team;

// Variables lectures GP
const int nbValues = 50;
long valuesLatGauche[nbValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
long valuesGauche[nbValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
long valuesDroit[nbValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
long valuesLatDroit[nbValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

// Methode de configuration pour le fonctionnement du programme
void setup() {
	// ------------------------------------------------------------- //
	// Initialisation du port série en debug seulement (cf define.h) //
	// ------------------------------------------------------------- //
#ifdef MAIN_DEBUG_MODE
	Serial.begin(115200);
	Serial.println(" == Initialisation robot Elfa ==");
#endif

	// ---------- //
	// Config I2C //
	// ---------- //
	Wire.begin();
#ifdef MAIN_DEBUG_MODE
	Serial.println(" - I2C [OK] (Master)");
#endif
	i2cUtils.pullup(false);
	i2cUtils.fastSpeed(false);

	// Tempo attente pour boot autres cartes
	delay(4000);

	// ------------------------ //
	// Initialisation ecran LCD //
	// ------------------------ //
	lcd.begin(SSD1306_SWITCHCAPVCC, OLED_LCD_ADD_BOARD);
	lcd.display();
	lcd.setTextSize(1);
	lcd.setTextColor(WHITE);

	// Affichage du logo
	delay(2000);

	byte nbDevices = i2cUtils.scan();
	if (nbDevices != NB_I2C_DEVICE) {
#ifdef MAIN_DEBUG_MODE
		Serial.println(" [ ERROR ] Il manque des périphériques I2C. Tous est bien branché ?");
#endif
		lcd.clearDisplay();
		lcd.setTextSize(2);
		lcd.println("    /!\\");
		lcd.println("Erreur I2C");
		lcd.print(nbDevices);lcd.print(" / ");lcd.println(NB_I2C_DEVICE);
		lcd.display();

		// Il manque des périphérique on bloque tout
		while(1 == 1);
	}

	// ------------- //
	// Servo manager //
	// ------------- //
#ifdef MAIN_DEBUG_MODE
	//servoManager.printVersion();
#endif

	// --------------------- //
	// Moteurs de propulsion //
	// --------------------- //
	motorsPropulsion.assignMotors(LEFT_MOTOR, RIGHT_MOTOR);

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
	robotManager.setPIDDistance(kpD, kiD, kdD);
	robotManager.setPIDOrientation(kpO, kiO, kdO);
	robotManager.setRampAcc(rampAccDistance, rampAccOrientation);
	robotManager.setRampDec(rampDecDistance, rampDecOrientation);
	robotManager.init();

#ifdef MAIN_DEBUG_MODE
	Serial.println(" - Robot manager [OK]");
#endif

	// -------- //
	// Béquille //
	// ------- //
	motBequille.stop();

#ifdef MAIN_DEBUG_MODE
	Serial.println(" - Config bequille [OK]");
#endif

	// -- //
	// IO //
	// -- //

	// Inputs AVR numérique
	pinMode(PIN_IRQ_1_2, INPUT);
	pinMode(PIN_IRQ_3_4, INPUT);
	pinMode(PIN_IRQ_5, INPUT);
	pinMode(PIN_IRQ_6, INPUT);
#ifdef MAIN_DEBUG_MODE
	Serial.println(" - Inputs numérique AVR [OK]");
#endif

	// Inputs AVR analogique
	pinMode(EQUIPE, INPUT);
	pinMode(CURRENT_MOTA, INPUT);
	pinMode(CURRENT_MOTB, INPUT);
#ifdef MAIN_DEBUG_MODE
	Serial.println(" - Inputs analogique AVR [OK]");
#endif

	// Outputs AVR numérique
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(OLED_RST, OUTPUT);
	pinMode(DIR_MOTA, OUTPUT);
	pinMode(DIR_MOTB, OUTPUT);
#ifdef MAIN_DEBUG_MODE
	Serial.println(" - Outputs numérique AVR [OK]");
#endif

	// Outputs AVR analogique
	pinMode(PWM_R, OUTPUT);
	pinMode(PWM_G, OUTPUT);
	pinMode(PWM_B, OUTPUT);
	pinMode(PWM_MOTA, OUTPUT);
	pinMode(PWM_MOTB, OUTPUT);
#ifdef MAIN_DEBUG_MODE
	Serial.println(" - Outputs analogique (PWM) AVR [OK]");
#endif

	// Inputs Expander Capteurs
	ioGyro.refresh();

	// Inputs Expander Gyro
	ioCapteurs.refresh();

	// Configuration par défaut des variables
	heartTime = heartTimePrec = millis();
	heart = false;

	// Configuration des vitesses servos moteurs et position initiale
	servoManager.setPositionAndSpeed(SERVO_STAB, SPEED_STAB, STAB_BAS);
	servoManager.setPositionAndSpeed(SERVO_GP2D, SPEED_GP2D, GP2D_GARAGE);
	servoManager.setPositionAndSpeed(SERVO_TAPIS_BAS, SPEED_TAPIS, TAPIS_BAS_FERME);
	servoManager.setPositionAndSpeed(SERVO_TAPIS_HAUT, SPEED_TAPIS, TAPIS_HAUT_FERME);

	// Initialisation des valeurs GP
	for (int i = 0 ; i < nbValues ; i++) {
		hasObstacle();
	}

	// Init Gestion Etapes
	gestEtapes = 0;
}

// Point d'entrée du programme
int main(void) {
	// Initialisation du SDK Arduino.
	// A réécrire si on veut customiser tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

	// Récupération de la couleur de l'équipe
	team = lectureEquipe();
#ifdef MAIN_DEBUG_MODE
	// Affichage de la couleur de l'équipe
	Serial.print(" -> Equipe : ");
	Serial.println((team == EQUIPE_JAUNE) ? "JAUNE" : "VERTE");

	// Procédure d'initialisation Robot (calage, tirette, etc).
	Serial.println(" == INIT MATCH ==");
#endif

	lcd.clearDisplay();
	lcd.println("Initialisation [OK]");
	lcd.print("Equipe : ");lcd.println((team == EQUIPE_JAUNE) ? "JAUNE" : "VERTE");
	lcd.display();
	delay(5000);

	// TODO Contrôle AU

#ifdef MAIN_DEBUG_MODE
		Serial.println(" -> Positionnement de la béquille");
#endif

	lcd.clearDisplay();
	lcd.setCursor(0, 0);
	lcd.println("Positionnement");
	lcd.println("bequille");
	lcd.display();

	// Descente jusqu'a perdre le fin de course
#ifdef MAIN_DEBUG_MODE
	Serial.println("Descente");
#endif
	motBequille.cmd(255);
	while(ioCapteurs.readCapteurValue(SW_BEQUILLE));
	motBequille.stop();
	delay(1000);

	// on remonte jusqu'au fin de course
#ifdef MAIN_DEBUG_MODE
	Serial.println("Monte");
#endif
	motBequille.cmd(-255);
	while(!ioCapteurs.readCapteurValue(SW_BEQUILLE));
	motBequille.stop();

	// Contrôle présence de la tirette
	if (!ioCapteurs.readCapteurValue(SW_TIRETTE)) {
		lcd.clearDisplay();
		lcd.setCursor(0, 0);
		lcd.println("/!\\/!\\/!\\/!\\/!\\/!\\");
		lcd.println("    Manque tirette");
		lcd.println("/!\\/!\\/!\\/!\\/!\\/!\\");
		lcd.display();

#ifdef MAIN_DEBUG_MODE
		Serial.println(" -> /!\\ La tirette n'est pas presente il faut d'abord la mettre !");
#endif

		while(!ioCapteurs.readCapteurValue(SW_TIRETTE));
		delay(1000);
	}

	// Attente du lancement du match.
#ifdef MAIN_DEBUG_MODE
	Serial.println(" -> Attente depart tirette ...");
#endif

	lcd.clearDisplay();
	lcd.setCursor(0, 0);
	lcd.println("Attente depart");
	lcd.println("tirette");
	lcd.display();

	while(ioCapteurs.readCapteurValue(SW_TIRETTE)) {
		team = lectureEquipe();
		heartBeat();
#ifdef MAIN_DEBUG_MODE
		if (Serial.available()) {
			if (Serial.read() == 's') { // La touche s de la liaison série est égal à la tirette
				break;
			}
		}
#endif
	}

	// Démarrage du comptage
	unsigned long startMatch = millis();
	unsigned long t;

	// Reset des valeurs codeurs lors des différents mouvements de positionnement
	robotManager.resetEncodeurs();
	servoManager.setPosition(SERVO_GP2D, GP2D_MATCH);

	if (team == EQUIPE_JAUNE) {
		robotManager.setPosition(Conv.mmToPulse(1000), Conv.mmToPulse(165), Conv.degToPulse(90));
	} else {
		robotManager.setPosition(Conv.mmToPulse(1000), Conv.mmToPulse(2835), Conv.degToPulse(-90));
	}

#ifdef MAIN_DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");

	// En tête de log
	Serial.println("#Gauche;Droit;X;Y;A;Type;Cons. Dist.;Cons. Orient.;PID Dist. setPoint;PID Dist. In;PID Dist. sumErr;PID Dist. Out;PID O setPoint;PID O In;PID O sumErr;PID O Out;Approche;Atteint");
	//Serial.println("#Cons. Roll;Input Roll;Error;Output PID;Courant");
#endif

	// On efface l'écran
	lcd.clearDisplay();
	lcd.display();

	do {
		heartBeat();
		matchLoop();

		// Gestion du temps
		t = millis();

		// Affichage des informations de base
		lcd.clearDisplay();
		lcd.setCursor(0,0);
		lcd.print("T ");lcd.print((t - startMatch) / 1000);lcd.print(" E ");lcd.println(gestEtapes - 1);
		lcd.print("X ");lcd.print((int) Conv.pulseToMm(robotManager.getPosition().getX()));lcd.print(" Y ");lcd.print((int) Conv.pulseToMm(robotManager.getPosition().getY()));lcd.print(" A ");lcd.println((int) Conv.pulseToDeg(robotManager.getPosition().getAngle()));
		lcd.print("Tap ");lcd.print(robotManager.getTrajetEnApproche());lcd.print(" Tat ");lcd.println(robotManager.getTrajetAtteint());
		lcd.display();
	} while(t - startMatch <= TPS_MATCH);

	// Plus de mouvement on arrete les moteurs.
	robotManager.stop();
	motBequille.stop();

	servoManager.setPosition(SERVO_TAPIS_BAS, TAPIS_BAS_OUVERT);
	servoManager.setPosition(SERVO_TAPIS_HAUT, TAPIS_HAUT_OUVERT);

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

	// Step première marches
	if (gestEtapes == 3) {
		// On stop le moteur
		if (ioCapteurs.readCapteurValue(IND_POSITION_1)) {
			motBequille.stop();
		}
	}

	// Step autres marches
	if (gestEtapes >= 5) {
		if (ioCapteurs.readCapteurValue(IND_POSITION_2)) {
			motBequille.cmd(-255);
		}
	}

	// Step fin escalier
	if (gestEtapes >= 7) {
		if (ioCapteurs.readCapteurValue(SW_BEQUILLE)) {
			motBequille.stop();
		}
	}

	// Processing de l'asservissement.
	robotManager.process();
}

// Gestion basique de la stratégie.
// Chemin prédéfinie
void nextEtape(){
	// Etapes >= 0 & < 100 : Cycle normal
	// Etapes >= 100 : Evittement
	switch (gestEtapes) {
	case 0 :
		// Point de passage à la con
		robotManager.setVitesse(800.0, 800.0);
		if (team == EQUIPE_JAUNE) {
			robotManager.gotoPointMM(1000, 1380, true);
		} else {
			robotManager.gotoPointMM(1000, 1620, true);
		}
		gestEtapes++;
		break;

	case 1 :
		// Orientation face aux marches
		robotManager.setVitesse(800.0, 800.0);
		if (team == EQUIPE_JAUNE) {
			robotManager.tourneDeg(90);
		} else {
			robotManager.tourneDeg(-90);
		}
		gestEtapes++;
		break;

	case 2 :
		// Devant les marches
		robotManager.setVitesse(800.0, 800.0);
		if (team == EQUIPE_JAUNE) {
			robotManager.gotoPointMM(730, 1380.0, true);
		} else {
			robotManager.gotoPointMM(730, 1620.0, true);
		}
		motBequille.cmd(255);
		gestEtapes++;
		break;

	case 3 :
		servoManager.setPosition(SERVO_STAB, STAB_HAUT);
		servoManager.setPosition(SERVO_TAPIS_HAUT, TAPIS_HAUT_OUVERT);
		gestEtapes++;
		break;

	case 4 :
		// On pousse sur la première position
		if (ioCapteurs.readCapteurValue(IND_POSITION_1)) {
			motBequille.stop();
			gestEtapes++;
		}
		break;

	case 5 :
		// On avance la première marche.
		robotManager.setVitesse(800.0, 800.0);
		robotManager.avanceMM(120);
		motBequille.cmd(255);
		gestEtapes++;
		break;

	case 6 :
		// On positionne a fond
		servoManager.setPosition(SERVO_GP2D, GP2D_ESCALIER);
		gestEtapes++;
		break;

	case 7 :
		// On fini notre ascension
		robotManager.setVitesse(600.0, 800.0);
		robotManager.avanceMM(400);
		gestEtapes++;
		break;

	case 8 :
		// On stabilise tout le bouzin
		motBequille.cmd(-255);
		gestEtapes++;
		break;

	case 9 :
		if (ioCapteurs.readCapteurValue(SW_BEQUILLE)) {
			servoManager.setPosition(SERVO_STAB, STAB_BAS);
			motBequille.stop();
			gestEtapes++;
		}
		break;

	case 10 :
		robotManager.setVitesse(800.0, 800.0);
		robotManager.avanceMM(50);
		gestEtapes++;
		break;
	}
}

// ----------------------------------- //
// Méthode appelé pour la fin du match //
// ----------------------------------- //
void endMatch() {
#ifdef MAIN_DEBUG_MODE
	Serial.println(" == FIN MATCH ==");
#endif

	lcd.clearDisplay();
	lcd.setCursor(0,0);
	lcd.setTextSize(4);
	lcd.println("FIN");
	lcd.display();
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

		// Clignotement de la LED embarqué
		digitalWrite(LED_BUILTIN, (heart) ? HIGH : LOW);

		// Clignotement de la couleur de l'équipe
		if (heart) {
			if (team == EQUIPE_JAUNE) {
				analogWrite(PWM_R, 255);
				analogWrite(PWM_G, 255);
				analogWrite(PWM_B, 0);
			} else {
				analogWrite(PWM_R, 0);
				analogWrite(PWM_G, 255);
				analogWrite(PWM_B, 0);
			}
		} else {
			analogWrite(PWM_R, 0);
			analogWrite(PWM_G, 0);
			analogWrite(PWM_B, 0);
		}
		heart = !heart;
	}
}

/*
 * Lecture de l'équipe selectioné
 */
boolean lectureEquipe() {
	return (analogRead(EQUIPE) > 128) ? EQUIPE_JAUNE : EQUIPE_VERTE;
}

/*
 * Méthode retournant l'information de présence d'un obstacle (adversaire ???)
 */
boolean hasObstacle() {
	int latGaucheAdc = averageLatGauche(ioGp2D.readCapteurValue(GP2D_GAUCHE_COTE));
	int latDroitAdc = averageLatDroit(ioGp2D.readCapteurValue(GP2D_DROIT_COTE));
	int gaucheAdc = averageGauche(ioGp2D.readCapteurValue(GP2D_GAUCHE_FRONT));
	int droitAdc = averageDroit(ioGp2D.readCapteurValue(GP2D_DROIT_FRONT));

	if (gestEtapes <= 3) {
		boolean latGauche = latGaucheAdc > 1450;
		boolean latDroit = latDroitAdc > 1450;
		boolean gauche = gaucheAdc > 1800;
		boolean droit = droitAdc > 1800;

		return latGauche || latDroit || gauche || droit;
	} else if (gestEtapes > 8) {
		// TODO : Vide
		return false;
	} else {
		return false;
	}

#ifdef MAIN_DEBUG_MODE
	/*Serial.print(latGaucheAdc);Serial.print(";");
	Serial.print(gaucheAdc);Serial.print(";");
	Serial.print(droitAdc);Serial.print(";");
	Serial.print(latDroitAdc);Serial.println(";");*/
#endif


}

// Moving average
int averageLatGauche(int newValue) {
	long value = newValue;
	for (int i = nbValues - 1 ; i > 0 ; i--) {
		valuesLatGauche[i] = valuesLatGauche[i - 1];
		value += valuesLatGauche[i];
	}
	valuesLatGauche[0] = newValue;
	return value / nbValues;
}
int averageGauche(int newValue) {
	long value = newValue;
	for (int i = nbValues - 1 ; i > 0 ; i--) {
		valuesGauche[i] = valuesGauche[i - 1];
		value += valuesGauche[i];
	}
	valuesGauche[0] = newValue;
	return value / nbValues;
}
int averageDroit(int newValue) {
	long value = newValue;
	for (int i = nbValues - 1 ; i > 0 ; i--) {
		valuesDroit[i] = valuesDroit[i - 1];
		value += valuesDroit[i];
	}
	valuesDroit[0] = newValue;
	return value / nbValues;
}
int averageLatDroit(int newValue) {
	long value = newValue;
	for (int i = nbValues - 1 ; i > 0 ; i--) {
		valuesLatDroit[i] = valuesLatDroit[i - 1];
		value += valuesLatDroit[i];
	}
	valuesLatDroit[0] = newValue;
	return value / nbValues;
}
