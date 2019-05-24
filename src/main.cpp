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
void progressSetup(String message, unsigned long delayMs = 500);
void matchLoop(GP2D12Result distanceRobot);

// Prototype des fonctions business
bool hasAU();
void setMode();
void setManualPosition();
void initMatchServos();
GP2D12Result distanceRobot();

// I2C Boards
SD21 servoManager = SD21(SD21_ADD_BOARD);
Adafruit_SSD1306 lcd = Adafruit_SSD1306(OLED_RST);

CheckRobot nerell = PARTI;
Mode mode = MANUEL;

int setupStep = 0;
const int nbValues = 100;
int valuesRaw[nbValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
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
	Serial.println(" == Initialisation experience Elfa ==");
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
    lcd.println("   ARIG  ");
    lcd.print("ELFA ");lcd.println(VERSION);
    lcd.display();
    delay(1000);
    lcd.startscrollleft(0x00, 0x0F);
    delay(10000);
    lcd.stopscroll();

    progressSetup("Scan I2C");
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
    progressSetup("Servos Manager");
#ifdef DEBUG_MODE
	servoManager.printVersion();
#endif

	// -- //
	// IO //
	// -- //

	// Inputs AVR numérique
    progressSetup("IN num. AVR");
	pinMode(AU, INPUT);
	pinMode(SELECT_MODE, INPUT);
    pinMode(SELECT_POSITION, INPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Inputs numérique AVR [OK]");
#endif

	// Inputs AVR analogique
    progressSetup("IN ana. AVR");
	pinMode(GP2D, INPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Inputs analogique AVR [OK]");
#endif

	// Outputs AVR numérique
    progressSetup("OUT num. AVR");
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(OLED_RST, OUTPUT);

#ifdef DEBUG_MODE
	Serial.println(" - Outputs numérique AVR [OK]");
#endif

    // Config servos
    progressSetup("Pos. init servo");
    servoManager.setPositionAndSpeed(SERVO_ASC_NB, SPEED_ASC, ASC_BAS);
#ifdef DEBUG_MODE
    Serial.println(" - Config servo SD21 [OK]");
#endif

    // Initialisation GP2D moyenne
    for (int i = 0; i < nbValues ; i++) {
        progressSetup("Init GP2D", 5);
        distanceRobot();
    }
}

// Point d'entrée du programme
int main() {
	// Initialisation du SDK.
	// A réécrire si on veut customiser tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

#ifdef DEBUG_MODE
	// Affichage de la couleur de l'équipe
	// Procédure d'initialisation Robot (calage, tirette, etc).
	Serial.println(" == INIT MATCH ==");
#endif

    progressSetup("Initialisation [OK]");
	delay(5000);

    if (!hasAU()) {
#ifdef DEBUG_MODE
        Serial.println("/!\\ AU KO");
#endif
        lcd.clearDisplay();
        lcd.setTextSize(4);
        lcd.setCursor(0, 0);
        lcd.println("/!\\ AU KO");
        lcd.display();
        delay(1000);
        lcd.startscrollleft(0x00, 0x0F);

        while(!hasAU()) {
            delay(500);
        }
        lcd.stopscroll();
        delay(1000);
    }

    // Doit être en manuel avant le match
    setMode();
    if (mode == AUTO) {
#ifdef DEBUG_MODE
        Serial.println("/!\\ Mode AUTO invalide au démarrage");
#endif
        lcd.clearDisplay();
        lcd.setTextSize(4);
        lcd.setCursor(0, 0);
        lcd.println("/!\\ Mode MANUEL attendu");
        lcd.display();
        delay(1000);
        lcd.startscrollleft(0x00, 0x0F);

        while(mode == AUTO) {
            setMode();
            delay(500);
        }
        lcd.stopscroll();
        delay(1000);
    }


    // Mode Manuel avant match
#ifdef DEBUG_MODE
    Serial.println("Mode Manuel");
#endif
    lcd.clearDisplay();
    lcd.setTextSize(2);
    lcd.setCursor(0, 0);
    lcd.println("Mode Manuel");
    lcd.display();
    do {
        setMode();
        setManualPosition();
        delay(500);
    } while(mode != AUTO);

	// Démarrage du comptage
    GP2D12Result distance;

#ifdef DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");
#endif

	// On efface l'écran
	lcd.clearDisplay();
	lcd.display();

    // On met les servos en position init de match
    initMatchServos();

	do {
        // Lecture distance
        distance = distanceRobot();

		// Affichage des informations de base
		lcd.clearDisplay();
        lcd.setTextSize(1);
		lcd.setCursor(0,0);
        lcd.print("D ");lcd.print(distance.cm);lcd.println(" cm");
        lcd.print("  ");lcd.print(distance.raw);lcd.println(" raw");
        lcd.print("Nerell : ");lcd.println(nerell == PRESENT ? "PRESENT" : "PARTI");
		lcd.display();

        matchLoop(distance);

	} while(nerell == PRESENT || nerell == PARTI); // while(true); mais sans le warning de CLion
}

// ---------------------------------------------------------------------------- //
// Méthode appelé encore et encore, tant que le temps du match n'est pas écoulé //
// ---------------------------------------------------------------------------- //
void matchLoop(GP2D12Result distanceRobot) {
    if (distanceRobot.cm < SEUIL_PRESENCE_ROBOT) {
        nerell = PRESENT;
    } else if (distanceRobot.cm > SEUIL_PRESENCE_ROBOT && nerell == PRESENT) {
        nerell = PARTI;
        servoManager.setPosition(SERVO_ASC_NB, ASC_HAUT);
    }
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //

bool hasAU() {
    return digitalRead(AU) == HIGH;
}
void setMode() {
    mode = digitalRead(SELECT_MODE) == HIGH ? AUTO : MANUEL;
}
void setManualPosition() {
    if (digitalRead(SELECT_POSITION) == HIGH) {
        servoManager.setPosition(SERVO_ASC_NB, ASC_BAS);
    } else {
        servoManager.setPosition(SERVO_ASC_NB, ASC_HAUT);
    }
}


GP2D12Result distanceRobot() {
    int sensorRaw = analogRead(GP2D);
    if (sensorRaw > MAX_RAW_GP) {
        sensorRaw = MAX_RAW_GP;
    } else if (sensorRaw < MIN_RAW_GP) {
        sensorRaw = MIN_RAW_GP;
    }

    int value = sensorRaw;
    for (int i = nbValues - 1 ; i > 0 ; i--) {
        valuesRaw[i] = valuesRaw[i - 1];
        value += valuesRaw[i];
    }
    valuesRaw[0] = sensorRaw;
    float rawAverage = value / nbValues;
    // Conversion en cm
    float cmAverage = (6787.0 / (rawAverage - 3.0)) - 4.0; // http://www.acroname.com/robotics/info/articles/irlinear/irlinear.html

    GP2D12Result res;
    res.raw = rawAverage;
    res.cm = cmAverage;

    return res;
}

void initMatchServos() {
    servoManager.setPosition(SERVO_ASC_NB, ASC_BAS);
}

void progressSetup(String message, unsigned long delayMs) {
    long progress = map(++setupStep, 0, NB_INIT_STEP + nbValues, 0, lcd.width() - 20);

    lcd.clearDisplay();
    lcd.setCursor(0,0);
    lcd.setTextSize(1);
    lcd.println(message);
    lcd.fillRect(10, 18, (int) progress, 29, WHITE);
    lcd.display();
    delay(delayMs);
}
