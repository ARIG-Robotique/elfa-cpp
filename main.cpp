#include <Arduino.h>
#include <Wire.h>

#include <robot/system/encoders/ARIGEncodeurs.h>
#include <robot/system/capteurs/BoardPCF8574.h>
#include <robot/system/capteurs/BoardI2CADC.h>
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

// Classe de gestion du robot (asserv, odométrie, pathfinding, evittement, etc...)
RobotManager robotManager = RobotManager();

// I2C Boards
SD21 servoManager = SD21(SD21_ADD_BOARD);
MD22 motorsPropulsion = MD22(MD22_ADD_BOARD, MODE_1, 0);
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
const double kiD = 0.50;
const double kdD = 0.25;

const double kpO = 0.50;
const double kiO = 0.25;
const double kdO = 1.00;


// Constantes d'ajustement pour les roues folles
const double coefRoueDroite = 1.00;
const double coefRoueGauche = 1.00;

// Variable pour l'équipe
boolean team;

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

	// ------------------------ //
	// Initialisation ecran LCD //
	// ------------------------ //
	lcd.begin(SSD1306_SWITCHCAPVCC, OLED_LCD_ADD_BOARD);
	lcd.display();
	lcd.setTextSize(1);
	lcd.setTextColor(WHITE);

	// Affichage du logo
	delay(2000);

#ifdef DEBUG_MODE
	lcd.dim(true);
#else
	lcd.dim(false);
#endif

	byte nbDevices = i2cUtils.scan();
	if (nbDevices != NB_I2C_DEVICE) {
#ifdef DEBUG_MODE
		Serial.println(" [ ERROR ] Il manque des périphériques I2C. Tous est bien branché ?");
#endif
		lcd.clearDisplay();
		lcd.setTextSize(2);
		lcd.println("Erreur I2C");
		lcd.print(nbDevices);lcd.print(" / ");lcd.println(NB_I2C_DEVICE);
		lcd.display();

		// Il manque des périphérique on bloque tout
		while(1 == 1);
	}

	// ------------- //
	// Servo manager //
	// ------------- //
#ifdef DEBUG_MODE
	servoManager.printVersion();
#endif

	// --------------------- //
	// Moteurs de propulsion //
	// --------------------- //
#ifdef DEBUG_MODE
	motorsPropulsion.printVersion();
#endif
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
#ifdef DEBUG_MODE
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

#ifdef DEBUG_MODE
		Serial.println(" -> Positionnement de la béquille");
#endif

	lcd.clearDisplay();
	lcd.setCursor(0, 0);
	lcd.println("Positionnement");
	lcd.println("bequille");
	lcd.display();

	/*digitalWrite(DIR_MOTB, SENS_BEQUILLE_DESCENT);
	analogWrite(PWM_MOTB, 200);
	while(ioCapteurs.readCapteurValue(SW_BEQUILLE));
	analogWrite(PWM_MOTB, 0);
	delay(500);*/

	digitalWrite(DIR_MOTB, SENS_BEQUILLE_MONTE);
	analogWrite(PWM_MOTB, 200);
	while(!ioCapteurs.readCapteurValue(SW_BEQUILLE));
	analogWrite(PWM_MOTB, 0);

	// Contrôle présence de la tirette
	if (!ioCapteurs.readCapteurValue(SW_TIRETTE)) {
		lcd.clearDisplay();
		lcd.setCursor(0, 0);
		lcd.println("/!\\ Manque tirette !");
		lcd.display();

#ifdef DEBUG_MODE
		Serial.println(" -> /!\\ La tirette n'est pas presente il faut d'abord la mettre !");
#endif

		while(!ioCapteurs.readCapteurValue(SW_TIRETTE));
		delay(1000);
	}

	// Attente du lancement du match.
#ifdef DEBUG_MODE
	Serial.println(" -> Attente depart tirette ...");
#endif

	lcd.clearDisplay();
	lcd.println("Attente depart");
	lcd.println("tirette");
	lcd.display();

	while(ioCapteurs.readCapteurValue(SW_TIRETTE)) {
		heartBeat();
#ifdef DEBUG_MODE
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

#ifdef DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");

	// En tête de log
	Serial.println("#Gauche;Droit;X;Y;A;Type;Cons. Dist.;Cons. Orient.;PID Dist. setPoint;PID Dist. In;PID Dist. sumErr;PID Dist. Out;PID O setPoint;PID O In;PID O sumErr;PID O Out;Approche;Atteint");
#endif

	do {
		heartBeat();
		matchLoop();

		// Gestion du temps
		t = millis();

		// Affichage des informations de base
		lcd.clearDisplay();
		lcd.setCursor(0,0);
		lcd.print("Time : ");lcd.print((t - startMatch) / 1000);lcd.println(" s");
		lcd.print("X : ");lcd.println(Conv.pulseToMm(robotManager.getPosition().getX()));
		lcd.print("Y : ");lcd.println(Conv.pulseToMm(robotManager.getPosition().getY()));
		lcd.print("A : ");lcd.println(Conv.pulseToDeg(robotManager.getPosition().getAngle()));
		lcd.display();
	} while(t - startMatch <= TPS_MATCH);

	// Plus de mouvement on arrete les moteurs.
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
}

// Gestion basique de la stratégie.
// Chemin prédéfinie
void nextEtape(){
	// Etapes >= 0 & < 100 : Cycle normal
	// Etapes >= 100 : Evittement
	case 0 :
		robotManager.setVitesse(200.0, 200.0);
		//robotManager.avanceMM(10000);
		//robotManager.tourneDeg(4*6405);
		robotManager.gotoPointMM(0.0, 680.0, true);
		gestEtapes++;
		break;
	case 1 :
		robotManager.setVitesse(200.0, 200.0);
		robotManager.gotoPointMM(380.0, 680.0, true);
		gestEtapes++;
		break;
	case 2 :
		robotManager.setVitesse(200.0, 200.0);
		robotManager.gotoPointMM(380.0, 0.0, true);
		gestEtapes++;
		break;
	case 3 :
		robotManager.setVitesse(200.0, 300.0);
		robotManager.alignFrontTo(0.0, 0.0);
		gestEtapes++;
		break;
	case 4 :
		robotManager.setVitesse(200.0, 200.0);
		robotManager.gotoPointMM(0.0, 0.0, true);
		gestEtapes++;
		break;
	case 5 :
		robotManager.setVitesse(200.0, 300.0);
		robotManager.alignFrontTo(0.0, 680.0);
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
