//
// Created by gdepuille on 19/01/2020.
//

#include "SD21.h"

#define SD21_I2C_PORT		hi2c1
#define SD21_I2C_ADDR           (0x61 << 1)
#define SD21_VERSION_REGISTER   0x40

extern I2C_HandleTypeDef SD21_I2C_PORT;
uint8_t sd21_GetBaseRegister(uint8_t servoNb);
uint8_t sd21_CheckServo(uint8_t servoNb);

/*
 * Méthode pour positionner un servo moteur
 */
void sd21_SetPosition(uint8_t servoNb, uint16_t position) {
  if (sd21_CheckServo(servoNb)) {
    uint8_t datas[] = { sd21_GetBaseRegister(servoNb) + 1, position & 0xFF, position >> 8 };
    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&SD21_I2C_PORT, SD21_I2C_ADDR, datas, 3, HAL_MAX_DELAY);
    if (result == HAL_OK) {

    }
  }
}

/*
 * Méthode pour définir la vitesse de rotation d'un servo moteur.
 */
void sd21_SetSpeed(uint8_t servoNb, uint8_t speed) {
  if (sd21_CheckServo(servoNb)) {
    uint8_t datas[] = { sd21_GetBaseRegister(servoNb) + 1, speed };
    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&SD21_I2C_PORT, SD21_I2C_ADDR, datas, 2, HAL_MAX_DELAY);
    if (result == HAL_OK) {

    }
  }
}

/*
 * Méthode pour définir la vitesse et la position dans la même transmission I2C
 */
void sd21_SetPositionAndSpeed(uint8_t servoNb, uint8_t speed, uint16_t position) {
  if (sd21_CheckServo(servoNb)) {
    uint8_t datas[] = { sd21_GetBaseRegister(servoNb), speed, position & 0xFF, position >> 8 };
    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&SD21_I2C_PORT, SD21_I2C_ADDR, datas, 4, HAL_MAX_DELAY);
    if (result == HAL_OK) {

    }
  }
}

/*
 * Méthode pour le contrôle du numéro du servo
 * Renvoi true si entre 1 et 21 inclus, false sinon
 */
uint8_t sd21_CheckServo(uint8_t servoNb) {
  return (servoNb >= 1 && servoNb <= 21);
}

/*
 * Renvoi le registre de base pour un servo.
 * Par éxemple pour le servo 1 :
 *  0 : SPEED REGISTER
 *  1 : LOW uint8_t POSITION REGISTER
 *  2 : HIGH uint8_t POSITION REGISTER
 */
uint8_t sd21_GetBaseRegister(uint8_t servoNb) {
  return servoNb * 3 - 3;
}

/*
 * Cette méthode affiche la version de la carte sur la liaison serie en mode
 * debug
 */
uint8_t sd21_GetVersion() {
  HAL_StatusTypeDef result;

  uint8_t reg = SD21_VERSION_REGISTER;
  result = HAL_I2C_Master_Transmit(&SD21_I2C_PORT, SD21_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
  if (result == HAL_OK) {

  }

  uint8_t datas[1];
  result = HAL_I2C_Master_Receive(&SD21_I2C_PORT, SD21_I2C_ADDR, datas, 1, HAL_MAX_DELAY);
  if (result == HAL_OK) {

  }
  return datas[0];
}
