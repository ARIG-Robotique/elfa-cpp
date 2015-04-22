#include <Arduino.h>
#include <Wire.h>

#include <robot/system/encoders/ARIGEncodeurs.h>
#include <robot/system/capteurs/BoardPCF8574.h>
#include <robot/system/capteurs/BoardI2CADC.h>
#include <robot/system/motors/PWMMotor.h>
#include <robot/system/motors/MD22.h>
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
boolean hasObstacle();
boolean lectureEquipe();

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
MD22 motorsPropulsion = MD22(MD22_ADD_BOARD, MODE_1, 0);
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

const double rampAccOrientation = 800.0; // en mm/s2
const double rampDecOrientation = 800.0; // en mm/s2

// -------------- //
// Parametres PID //
// -------------- //
const double kpD = 1.00;
const double kiD = 0.50;
const double kdD = 0.25;

const double kpO = 0.50;
const double kiO = 0.25;
const double kdO = 1.00;

const double kpB = 2.00;
const double kiB = 0.20;
const double kdB = 0.00;

// Constantes d'ajustement pour les roues folles
const double coefRoueDroite = 1.00;
const double coefRoueGauche = 1.00;

// Variable pour l'équipe
boolean team;

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
	Serial.println("Descente");
	motBequille.cmd(200);
	while(ioCapteurs.readCapteurValue(SW_BEQUILLE));
	motBequille.stop();
	delay(500);

	// on remonte jusqu'au fin de course
	Serial.println("Monte");
	motBequille.cmd(-200);
	while(!ioCapteurs.readCapteurValue(SW_BEQUILLE));
	motBequille.stop();

	// Contrôle présence de la tirette
	if (!ioCapteurs.readCapteurValue(SW_TIRETTE)) {
		lcd.clearDisplay();
		lcd.setCursor(0, 0);
		lcd.println("/!\\ Manque tirette !");
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

	team = lectureEquipe();
	if (team == EQUIPE_JAUNE) {
		robotManager.setPosition(Conv.mmToPulse(2850), Conv.mmToPulse(250), Conv.degToPulse(135));
	} else {
		robotManager.setPosition(Conv.mmToPulse(150), Conv.mmToPulse(250), Conv.degToPulse(45));
	}

	// Pour tester //
	// TODO : A supprimer
	robotManager.setPosition(Conv.mmToPulse(1090), Conv.mmToPulse(35), Conv.degToPulse(90));

#ifdef MAIN_DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");

	// En tête de log
	//Serial.println("#Gauche;Droit;X;Y;A;Type;Cons. Dist.;Cons. Orient.;PID Dist. setPoint;PID Dist. In;PID Dist. sumErr;PID Dist. Out;PID O setPoint;PID O In;PID O sumErr;PID O Out;Approche;Atteint");
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

#ifdef MAIN_DEBUG_MODE
		// Affichage des informations de base
		lcd.clearDisplay();
		lcd.setCursor(0,0);
		lcd.print("Time : ");lcd.print((t - startMatch) / 1000);lcd.println(" s");
		lcd.print("X : ");lcd.println(Conv.pulseToMm(robotManager.getPosition().getX()));
		lcd.print("Y : ");lcd.println(Conv.pulseToMm(robotManager.getPosition().getY()));
		lcd.print("A : ");lcd.println(Conv.pulseToDeg(robotManager.getPosition().getAngle()));
		lcd.display();
#endif
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

		// On est aligné devant les marches.
		//if (gestEtapes == 1) {
		if (gestEtapes == 4) {
			// Récupération de la valeur de consigne
			pidBequille.reset();
			accel.getEvent(&accelEvt);
			mag.getEvent(&magEvt);
			dof.fusionGetOrientation(&accelEvt, &magEvt, &orientation);
			rollCons = orientation.roll;

			// Stabilisation relaché
			servoManager.setPosition(SERVO_STAB, STAB_HAUT);
			//gestEtapes = 2;
			gestEtapes = 5;
		}
	}

	// Processing de l'asservissement.
	robotManager.process();

	// Gestion de l'asservissement gyroscopique
	//if (gestEtapes >= 2) {
	if (gestEtapes >= 5) {
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
		robotManager.setVitesse(500.0, 500.0);
		//robotManager.avanceMM(730.0);
		//robotManager.setVitesse(400.0, 500.0);
		robotManager.gotoPointMM(1050.0, 1263.0, false);
		gestEtapes++;
		break;
	case 1 :
		robotManager.setVitesse(400.0, 500.0);
		robotManager.gotoPointMM(1050.0, 1513.0, false);
		gestEtapes++;
		break;
	case 2 : // Devant les marches
		robotManager.setVitesse(100.0, 500.0);
		robotManager.gotoPointMM(750.0, 1533.0, true);
		gestEtapes++;
		break;
	case 3 :
		robotManager.setVitesse(500.0, 500.0);
		robotManager.gotoOrientationDeg(180);
		gestEtapes++;
		break;
	case 5 : // Montée des marches
		robotManager.setVitesse(200.0, 500.0);
		robotManager.avanceMM(730.0);
		gestEtapes++;
		break;
	case 6 :
		// Bas des marches tans que l'on as le capteur bequille
		if (!ioCapteurs.readCapteurValue(SW_BEQUILLE)) {
			// Fin de course perdu, l'asserv commence
			gestEtapes++;
		}
		break;
	case 7 :
		// Dès que l'on récupère le fin de cours on stop tout
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
