//
// Created by gdepuille on 17/01/2020.
//

#ifndef _WS2812_H
#define _WS2812_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define WS2812_FREQ		    800000 // it is fixed: WS2812 require 800kHz
#define TIMER_CLOCK_FREQ	16000000 // can be modified - multiples of 0.8MHz are suggested
#define TIMER_PERIOD		TIMER_CLOCK_FREQ / WS2812_FREQ
#define LED_NUMBER		    2 // how many LEDs the MCU should control?
#define LED_DATA_SIZE		LED_NUMBER * 24
#define RESET_SLOTS_BEGIN	50
#define RESET_SLOTS_END		50
#define WS2812_LAST_SLOT	1
#define LED_BUFFER_SIZE		RESET_SLOTS_BEGIN + LED_DATA_SIZE + WS2812_LAST_SLOT + RESET_SLOTS_END
#define WS2812_0		    TIMER_PERIOD / 3      // WS2812's zero high time is long about one third of the period
#define WS2812_1		    TIMER_PERIOD * 2 / 3  // WS2812's one high time is long about two thirds of the period
#define WS2812_RESET		0

#pragma GCC diagnostic ignored "-Wunused-variable"
static uint8_t LedBuffer[LED_BUFFER_SIZE];
#pragma GCC diagnostic pop

void ws2812_Init(void);

void ws2812_SetLedColor(uint32_t ledNumber, uint8_t red, uint8_t green, uint8_t blue);
void ws2812_SetAllLedsColor(uint8_t red, uint8_t green, uint8_t blue);
void ws2812_FadeToBlack(uint8_t scaleBy);

#endif // _WS2812_H
