//
// Created by gdepuille on 17/01/2020.
//

#ifndef _WS2812_H
#define _WS2812_H

#include <stdint.h>

#include "stm32f4xx_hal.h"


#define TRAY_LED_NUMBER     24
#define TRAIL_LENGTH   4
#define COLUMN_LED_NUMBER   8

#define TRAY_LED_OFFSET     0
#define COLUMN1_LED_OFFSET  TRAY_LED_NUMBER
#define COLUMN2_LED_OFFSET  (TRAY_LED_NUMBER + COLUMN_LED_NUMBER)

#define BLINKING_LED_PERIOD  5

#define WS2812_FREQ		    800000 // it is fixed: WS2812 require 800kHz
#define TIMER_CLOCK_FREQ	80000000 // can be modified - multiples of 0.8MHz are suggested
#define TIMER_PERIOD		99 //TIMER_CLOCK_FREQ / WS2812_FREQ
#define LED_NUMBER		    (TRAY_LED_NUMBER + 2 * COLUMN_LED_NUMBER) // how many LEDs the MCU should control?
#define LED_DATA_SIZE		LED_NUMBER * 24
#define RESET_SLOTS_BEGIN	100
#define RESET_SLOTS_END		100
#define LED_BUFFER_SIZE		(RESET_SLOTS_BEGIN + LED_DATA_SIZE + RESET_SLOTS_END)
#define WS2812_0		    (TIMER_PERIOD / 3)      // WS2812's zero high time is long about one third of the period
#define WS2812_1		    (TIMER_PERIOD * 2 / 3)  // WS2812's one high time is long about two thirds of the period
#define WS2812_RESET		0

extern TIM_HandleTypeDef htim5;
#define WS2812_TIM     htim5
#define WS2812_CHANNEL TIM_CHANNEL_3

#pragma GCC diagnostic ignored "-Wunused-variable"
static uint32_t LedBuffer[LED_BUFFER_SIZE];
#pragma GCC diagnostic pop

void ws2812_Init(void);

void ws2812_SetLedColor(uint32_t ledNumber, uint8_t red, uint8_t green, uint8_t blue);
void ws2812_SetAllLedsColor(uint8_t red, uint8_t green, uint8_t blue);
void ws2812_FadeToBlack(uint8_t scaleBy);
void circularTray(void);
void rocketColumns(void);
void ws2812_Reset(void);
void blinkingRed(void);
void setTrayColor(uint8_t red, uint8_t green, uint8_t blue);
void validation(int validationPeriod);

#endif // _WS2812_H
