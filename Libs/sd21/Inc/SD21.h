//
// Created by gdepuille on 19/01/2020.
//

#ifndef SD21_H_
#define SD21_H_

#include <stdint.h>

#if defined(STM32F3)
#include "stm32f3xx_hal.h"
#else
#error "SD21 library was tested only on STM32F3 MCU families. Please modify WS2812.h if you know what you are doing"
#endif

void sd21_SetPosition(uint8_t servoNb, uint16_t position);
void sd21_SetSpeed(uint8_t servoNb, uint8_t speed);
void sd21_SetPositionAndSpeed(uint8_t servoNb, uint8_t speed, uint16_t position);

uint8_t sd21_GetVersion();

#endif /* SD21_H_ */
