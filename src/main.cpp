#include <Arduino.h>
#include <Wire.h>

#include <robot/system/servos/SD21.h>
#include <utils/I2CUtils.h>

// Ecran LCD
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "define.h"

// Prototype des fonctions principale
void setup();
void matchLoop(unsigned long elapsedTime, float distanceRobot);
void funnyAction();
void endMatch();

// Prototype des fonctions business
void stopMotor();
void speedMotor(int speed);
bool hasAU();
bool hasTirette();
float distanceRobot();
void initMatchServos();
void cycleDeposeAndReturn();
void cycleDeposeOnly();

// I2C Boards
SD21 servoManager = SD21(SD21_ADD_BOARD);
Adafruit_SSD1306 lcd = Adafruit_SSD1306(OLED_RST);

CheckRobot nerell = PAS_PRESENT;

const int nbValues = 100;
int valuesDistance[nbValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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
	lcd.setTextColor(WHITE);

	// Affichage du logo
	delay(1000);

    // Nom du robot
    lcd.clearDisplay();
    lcd.setTextSize(2);
    lcd.println(" * ARIG *");
    lcd.println(" * ELFA *");
    lcd.display();
    delay(2000);

	byte nbDevices = i2cUtils.scan();
	if (nbDevices != NB_I2C_DEVICE) {
#ifdef DEBUG_MODE
		Serial.println(" [ ERROR ] Il manque des périphériques I2C. Tous est bien branché ?");
#endif
		lcd.clearDisplay();
		lcd.setTextSize(2);
		lcd.println("    /!\\");
		lcd.println("Erreur I2C");
		lcd.print(nbDevices);lcd.print(" / ");lcd.println(NB_I2C_DEVICE);
		lcd.display();

		// Il manque des périphériques on bloque tout
		while(true);
	}

	// ------------- //
	// Servo manager //
	// ------------- //
#ifdef DEBUG_MODE
	servoManager.printVersion();
#endif

	// -- //
	// IO //
	// -- //

	// Inputs AVR numérique
	pinMode(AU, INPUT);
	pinMode(TIRETTE, INPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Inputs numérique AVR [OK]");
#endif

	// Inputs AVR analogique
	pinMode(GP2D, INPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Inputs analogique AVR [OK]");
#endif

	// Outputs AVR numérique
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(OLED_RST, OUTPUT);

#ifdef DEBUG_MODE
	Serial.println(" - Outputs numérique AVR [OK]");
#endif

	// Outputs AVR analogique
    pinMode(PWM_HELICE, OUTPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Outputs analogique (PWM) AVR [OK]");
#endif

    // Config servos
    servoManager.setPositionAndSpeed(SERVO_ASC_NB, SPEED_ASC, ASC_START);
    servoManager.setPositionAndSpeed(SERVO_INC_NB, SPEED_INC_NORM, INC_START);
#ifdef DEBUG_MODE
    Serial.println(" - Config servo SD21 [OK]");
#endif

    stopMotor();
#ifdef DEBUG_MODE
    Serial.println(" - Config moteur hélisse [OK]");
#endif

    // Initialisation GP2D moyenne
    for (int i = 0; i < nbValues ; i++) {
        lcd.clearDisplay();
        lcd.setTextSize(2);
        lcd.setCursor(0, 0);
        lcd.println("GP2D: cm");
        lcd.println(distanceRobot());
        lcd.display();
    }
}

// Point d'entrée du programme
int main(void) {
	// Initialisation du SDK Arduino.
	// A réécrire si on veut customiser tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

#ifdef DEBUG_MODE
	// Affichage de la couleur de l'équipe
	// Procédure d'initialisation Robot (calage, tirette, etc).
	Serial.println(" == INIT MATCH ==");
#endif

	lcd.clearDisplay();
	lcd.println("Initialisation [OK]");
	lcd.display();
	delay(5000);

#ifdef DEBUG_MODE
		Serial.println(" -> Positionnement de la béquille");
#endif

    if (!hasAU()) {
#ifdef DEBUG_MODE
        Serial.println("/!\\ AU KO");
#endif
        lcd.clearDisplay();
        lcd.setTextSize(2);
        lcd.setCursor(0, 0);
        lcd.println("    /!\\");
        lcd.println("   AU KO");
        lcd.display();

        while(!hasAU());
        delay(1000);
    }

	// Contrôle présence de la tirette
	if (!hasTirette()) {
		lcd.clearDisplay();
        lcd.setTextSize(2);
		lcd.setCursor(0, 0);
        lcd.println("    /!\\");
		lcd.println("Pas tirette");
		lcd.display();

#ifdef DEBUG_MODE
		Serial.println(" -> /!\\ La tirette n'est pas presente il faut d'abord la mettre !");
#endif

		while(!hasTirette())
		delay(1000);
	}

	// Attente du lancement du match.
#ifdef DEBUG_MODE
	Serial.println(" -> Attente depart tirette ...");
#endif

	lcd.clearDisplay();
    lcd.setTextSize(2);
	lcd.setCursor(0, 0);
	lcd.println("Attente");
	lcd.println("start ...");
	lcd.display();

	while(hasTirette()) {
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
    float distance;

#ifdef DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");
#endif

	// On efface l'écran
	lcd.clearDisplay();
    lcd.setTextSize(2);
	lcd.display();

    // On met les servos en position init de match
    initMatchServos();


	do {
		// Gestion du temps
		t = millis();

        // Lecture distance
        distance = distanceRobot();

		// Affichage des informations de base
		lcd.clearDisplay();
		lcd.setCursor(0,0);
		lcd.print("T ");lcd.print((t - startMatch) / 1000);lcd.println(" s");
        lcd.print("D ");lcd.print(distance);lcd.print(" cm");
		lcd.display();

        matchLoop(t - startMatch, distance);

	} while(t - startMatch <= TPS_MATCH);

    // Funny action
    funnyAction();

	while(millis() - startMatch <= END_TOUT);
	endMatch();

	// Fin de tout. Boucle infini pour eviter de dépiler la stack mémoire et éxécuter
    // des instructions aléatoire.
	while(true) {
		// NOP
	}
}

// ---------------------------------------------------------------------------- //
// Méthode appelé encore et encore, tant que le temps du match n'est pas écoulé //
// ---------------------------------------------------------------------------- //
void matchLoop(unsigned long elapsedTime, float distanceRobot) {
    if (distanceRobot < SEUIL_PRESENCE_ROBOT) {
        nerell = PRESENT;
    } else if (distanceRobot > SEUIL_PRESENCE_ROBOT && nerell == PRESENT) {
        nerell = PAS_PRESENT;

        if (elapsedTime >= TPS_CYCLE_DEPOSE_FULL && elapsedTime < TPS_CYCLE_ANNULE) {
            cycleDeposeOnly();
        } else if (elapsedTime < TPS_CYCLE_DEPOSE_FULL) {
            cycleDeposeAndReturn();
        }
    }
}

// -------------------------------------------- //
// Méthode appele pour effectué la funny action //
// -------------------------------------------- //
void funnyAction() {
    lcd.clearDisplay();
    lcd.setCursor(0,0);
    lcd.setTextSize(2);
    lcd.println("* FUNNY  *");
    lcd.println("* ACTION *");
    lcd.display();

//	for(int i = LOW_SPEED ; i <= HIGH_SPEED ; i++) {
//		speedMotor(i);
//		delay(4000 / (HIGH_SPEED - LOW_SPEED));
//	}

    speedMotor(LOW_SPEED);
    delay(1000);
    speedMotor(HIGH_SPEED);
    delay(3000);
    stopMotor();
}

// ----------------------------------- //
// Méthode appelé pour la fin du match //
// ----------------------------------- //
void endMatch() {
#ifdef DEBUG_MODE
	Serial.println(" == FIN MATCH ==");
#endif

	lcd.clearDisplay();
	lcd.setTextSize(2);
    lcd.setCursor(0,0);
    lcd.println(" *  FIN  *");
    lcd.println(" * MATCH *");
	lcd.display();
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //
void stopMotor() {
    speedMotor(STOP_SPEED);
}
void speedMotor(int speed) {
	if (speed > 255)  {
		speed = 255;
	} else if (speed < 0) {
		speed = 0;
	}

    if (speed == STOP_SPEED) {
        digitalWrite(IN2_HELICE, LOW);
    } else {
        digitalWrite(IN2_HELICE, HIGH);
    }

    analogWrite(PWM_HELICE, speed);
}

bool hasAU() {
    return digitalRead(AU) == HIGH;
}
bool hasTirette() {
    return digitalRead(TIRETTE) == HIGH;
}

float distanceRobot() {
    int sensorRaw = analogRead(GP2D);
    int value = sensorRaw;
    for (int i = nbValues - 1 ; i > 0 ; i--) {
        valuesDistance[i] = valuesDistance[i - 1];
        value += valuesDistance[i];
    }
    valuesDistance[0] = sensorRaw;
    float average = value / nbValues;
    // Conversion en cm
    float resultCm = (6787.0 / (average - 3.0)) - 4.0; // http://www.acroname.com/robotics/info/articles/irlinear/irlinear.html
    return resultCm;
}

void initMatchServos() {
    servoManager.setPosition(SERVO_INC_NB, INC_PRISE);
    servoManager.setPosition(SERVO_ASC_NB, ASC_BAS);
}

void cycleDeposeAndReturn() {
    cycleDeposeOnly();
    delay(5000);

    lcd.clearDisplay();
    lcd.setTextSize(2);
    lcd.setCursor(0,0);
    lcd.println("* RETOUR *");
    lcd.display();

    servoManager.setPosition(SERVO_ASC_NB, ASC_PRE_DEPOSE);
    servoManager.setPosition(SERVO_INC_NB, INC_PRE_DEPOSE);
    delay(1000);
    servoManager.setPosition(SERVO_ASC_NB, ASC_START);
    delay(1000);
    servoManager.setPositionAndSpeed(SERVO_INC_NB, SPEED_INC_NORM, INC_PRISE);
    delay(1000);
    servoManager.setPosition(SERVO_ASC_NB, ASC_BAS);
}

void cycleDeposeOnly() {
    lcd.clearDisplay();
    lcd.setTextSize(2);
    lcd.setCursor(0,0);
    lcd.println("* DEPOSE *");
    lcd.display();

    servoManager.setPosition(SERVO_ASC_NB, ASC_START);
    delay(2000);
    servoManager.setPosition(SERVO_INC_NB, INC_PRE_DEPOSE);
    delay(1000);
    servoManager.setPosition(SERVO_ASC_NB, ASC_PRE_DEPOSE);
    delay(1000);
    servoManager.setPosition(SERVO_ASC_NB, ASC_DEPOSE);
    servoManager.setPositionAndSpeed(SERVO_INC_NB, SPEED_INC_COMB, INC_DEPOSE);
}