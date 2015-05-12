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

// 10 DOF
#include <adafruit/Adafruit_10DOF.h>

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
Pid pidBequille = Pid();

// Classe de gestion du robot (asserv, odométrie, pathfinding, evittement, etc...)
RobotManager robotManager = RobotManager();
Adafruit_10DOF dof = Adafruit_10DOF();

// I2C Boards
SD21 servoManager = SD21(SD21_ADD_BOARD);
SD21Motors motorsPropulsion = SD21Motors(SD21_ADD_BOARD);
ARIGEncodeurs encodeurs = ARIGEncodeurs(ENCODEUR_GAUCHE_BOARD, ENCODEUR_DROIT_BOARD);
Adafruit_SSD1306 lcd = Adafruit_SSD1306(OLED_RST);
BoardPCF8574 ioGyro = BoardPCF8574("Gyro", PCF_GYRO_ADD_BOARD);
BoardPCF8574 ioCapteurs = BoardPCF8574("Num", PCF_CAPTEURS_ADD_BOARD);
BoardI2CADC ioGp2D = BoardI2CADC(GP2D_ADD_BOARD);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

// Gestion des étapes
int gestEtapes;

// ------------------------ //
// Configuration des rampes //
// ------------------------ //
const double rampAccDistance = 800.0; // en mm/s2
const double rampDecDistance = 800.0; // en mm/s2

const double rampAccOrientation = 1000.0; // en mm/s2
const double rampDecOrientation = 1000.0; // en mm/s2

// -------------- //
// Parametres PID //
// -------------- //
const double kpD = 1.00;
const double kiD = 0.50;
const double kdD = 0.25;

const double kpO = 0.50;
const double kiO = 0.25;
const double kdO = 1.00;

const double kpB = 40.00;
const double kiB = 2.00;
const double kdB = 0.00;

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

// Evennement pour l'orientation sur les marches
double rollCons;
double rollOutput;
sensors_event_t accelEvt;
sensors_event_t magEvt;
sensors_vec_t orientation;

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

	// ------------------------------------ //
	// Gyroscope / Accelerometre / Pression //
	// ------------------------------------ //

	if (!accel.begin() || !mag.begin()) {
#ifdef MAIN_DEBUG_MODE
		Serial.println(" [ ERROR ] Oops, j'ai perdu mon Accelerometre et Magnetometre");
#endif
		lcd.clearDisplay();
		lcd.setTextSize(1);
		lcd.println("   /!\\");
		lcd.println("  Erreur");
		lcd.println("Accel, Mag");
		lcd.display();

		// Il manque des périphérique on bloque tout
		while(1 == 1);
	}

#ifdef MAIN_DEBUG_MODE
	sensor_t sensor;

	accel.getSensor(&sensor);
	Serial.println(F(" -> ACCELEROMETER"));
	Serial.print  (F("    * Sensor    : ")); Serial.println(sensor.name);
	Serial.print  (F("    * Driver Ver: ")); Serial.println(sensor.version);
	Serial.print  (F("    * Unique ID : ")); Serial.println(sensor.sensor_id);
	Serial.print  (F("    * Max Value : ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
	Serial.print  (F("    * Min Value : ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
	Serial.print  (F("    * Resolution: ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));

	mag.getSensor(&sensor);
	Serial.println(F(" -> MAGNETOMETER"));
	Serial.print  (F("    * Sensor    : ")); Serial.println(sensor.name);
	Serial.print  (F("    * Driver Ver: ")); Serial.println(sensor.version);
	Serial.print  (F("    * Unique ID : ")); Serial.println(sensor.sensor_id);
	Serial.print  (F("    * Max Value : ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
	Serial.print  (F("    * Min Value : ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
	Serial.print  (F("    * Resolution: ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));
#endif

	// Init value Accel + Mag
	accel.getEvent(&accelEvt);
	mag.getEvent(&magEvt);
	dof.fusionGetOrientation(&accelEvt, &magEvt, &orientation);

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
	pidBequille.setTunings(kpB, kiB, kdB);
	pidBequille.reset();

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
	// TODO : A changer
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
	motBequille.cmd(200);
	while(ioCapteurs.readCapteurValue(SW_BEQUILLE));
	motBequille.stop();
	delay(1000);

	// on remonte jusqu'au fin de course
#ifdef MAIN_DEBUG_MODE
	Serial.println("Monte");
#endif
	motBequille.cmd(-200);
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

	team = lectureEquipe();
	if (team == EQUIPE_JAUNE) {
		robotManager.setPosition(Conv.mmToPulse(1000), Conv.mmToPulse(165), Conv.degToPulse(90));
	} else {
		robotManager.setPosition(Conv.mmToPulse(1000), Conv.mmToPulse(2835), Conv.degToPulse(-90));
	}

	// FIXME : A supprimer.
	robotManager.setPosition(Conv.mmToPulse(1000), Conv.mmToPulse(165), Conv.degToPulse(90));

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
		lcd.print("Roll ");lcd.println(orientation.roll);
		lcd.display();
	} while(t - startMatch <= TPS_MATCH);

	// Plus de mouvement on arrete les moteurs.
	motBequille.stop();
	robotManager.stop();

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

	// Gestion de l'asservissement gyroscopique
	if (gestEtapes >= 4) {
		accel.getEvent(&accelEvt);
		mag.getEvent(&magEvt);
		dof.fusionGetOrientation(&accelEvt, &magEvt, &orientation);
		rollOutput = pidBequille.compute(rollCons, orientation.roll);
		if (ioCapteurs.readCapteurValue(SW_BEQUILLE) && rollOutput < 0) {
			// On ne fait rien
			pidBequille.reset();
			motBequille.stop();
		} else {
			motBequille.cmd(rollOutput);
		}
#ifdef MAIN_DEBUG_MODE
		double e = pidBequille.getError();
		Serial.print(rollCons);
		Serial.print(";");Serial.print(orientation.roll);
		Serial.print(";");Serial.print(e);
		Serial.print(";");Serial.print(rollOutput);
		Serial.print(";");Serial.println(motBequille.current());
#endif
	}
}

// Gestion basique de la stratégie.
// Chemin prédéfinie
void nextEtape(){
	// Etapes >= 0 & < 100 : Cycle normal
	// Etapes >= 100 : Evittement
	switch (gestEtapes) {
	case 0 :
		// Point de passage à la con
		robotManager.setVitesse(600.0, 800.0);
		robotManager.gotoPointMM(1000, 1000, false);
		gestEtapes++;
		break;

	case 1 :
		// Devant les marches
		robotManager.setVitesse(600.0, 800.0);
		robotManager.gotoPointMM(730, 1335.0, true);
		gestEtapes++;
		break;

	case 2 :
		// Orientation face aux marches
		robotManager.setVitesse(600.0, 800.0);
		robotManager.gotoOrientationDeg(180);
		gestEtapes++;
		break;

	case 3 :
		// Récupération de la valeur de consigne
		pidBequille.reset();
		accel.getEvent(&accelEvt);
		mag.getEvent(&magEvt);
		dof.fusionGetOrientation(&accelEvt, &magEvt, &orientation);
		rollCons = orientation.roll;

		// Stabilisation relaché
		servoManager.setPosition(SERVO_STAB, STAB_HAUT);

		// Montée des marches
		robotManager.setVitesse(200.0, 400.0);
		robotManager.avanceMM(600);
		gestEtapes++;
		break;
	case 4 :
		// Bas des marches tans que l'on as le capteur bequille
		if (!ioCapteurs.readCapteurValue(SW_BEQUILLE)) {
			// Fin de course perdu, l'asserv commence
			gestEtapes++;
		}
		break;
	case 5 :
		// Dès que l'on récupère le fin de course on stop tout
		if (ioCapteurs.readCapteurValue(SW_BEQUILLE)) {
			robotManager.avanceMM(0);
			gestEtapes++;
		}
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

	// Juste les deux de devant et les deux de dérriere
	int latGaucheAdc = averageLatGauche(ioGp2D.readCapteurValue(GP2D_GAUCHE_COTE));
	int latDroitAdc = averageLatDroit(ioGp2D.readCapteurValue(GP2D_DROIT_COTE));
	int gaucheAdc = averageGauche(ioGp2D.readCapteurValue(GP2D_GAUCHE_FRONT));
	int droitAdc = averageDroit(ioGp2D.readCapteurValue(GP2D_DROIT_FRONT));

	boolean latGauche = latGaucheAdc > 1000;
	boolean latDroit = latDroitAdc > 1000;
	boolean gauche = gaucheAdc > 1140;
	boolean droit = droitAdc > 1140;

#ifdef MAIN_DEBUG_MODE
	/*Serial.print(latGaucheAdc);Serial.print(";");
	Serial.print(gaucheAdc);Serial.print(";");
	Serial.print(droitAdc);Serial.print(";");
	Serial.print(latDroitAdc);Serial.println(";");*/
#endif

	return latGauche || latDroit || gauche || droit;
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
