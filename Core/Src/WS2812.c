//
// Created by gdepuille on 17/01/2020.
//

#include "WS2812.h"

extern TIM_HandleTypeDef htim5;
extern TIM_OC_InitTypeDef htim5Config;

uint8_t r[LED_NUMBER];
uint8_t g[LED_NUMBER];
uint8_t b[LED_NUMBER];

void ws2812_fillBufferBlack(void);
void ws2812_fillBufferWhite(void);

void ws2812_Init() {
  ws2812_fillBufferBlack();

  // Start PWM Generator from DMA
  HAL_TIM_PWM_ConfigChannel(&htim5, &htim5Config, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_3, (uint32_t *) LedBuffer, LED_BUFFER_SIZE);

}

void ws2812_SetAllLedsColor(uint8_t red, uint8_t green, uint8_t blue) {
  for (uint32_t idx = 0 ; idx < LED_NUMBER ; idx++) {
    ws2812_SetLedColor(idx, red, green, blue);
  }
}

void ws2812_FadeToBlack(uint8_t scaleBy) {

  uint16_t scale = 256 - scaleBy;
  for (uint32_t idx = 0 ; idx < LED_NUMBER ; idx++) {
    uint8_t newRed = (((uint16_t) r[idx]) * scale) >> 8;
    uint8_t newGreen = (((uint16_t) g[idx]) * scale) >> 8;;
    uint8_t newBlue = (((uint16_t) b[idx]) * scale) >> 8;;

    ws2812_SetLedColor(idx, newRed, newGreen, newBlue);
  }
}

void ws2812_SetLedColor(uint32_t ledNumber, uint8_t red, uint8_t green,
                         uint8_t blue) {
  uint8_t tempBuffer[24];
  uint32_t i;
  uint32_t ledIndex = ledNumber % LED_NUMBER;

  r[ledIndex] = red;
  g[ledIndex] = green;
  b[ledIndex] = blue;

  for (i = 0; i < 8; i++) // GREEN data
    tempBuffer[i] = ((green << i) & 0x80) ? WS2812_1 : WS2812_0;
  for (i = 0; i < 8; i++) // RED
    tempBuffer[8 + i] = ((red << i) & 0x80) ? WS2812_1 : WS2812_0;
  for (i = 0; i < 8; i++) // BLUE
    tempBuffer[16 + i] = ((blue << i) & 0x80) ? WS2812_1 : WS2812_0;

  for (i = 0; i < 24; i++)
    LedBuffer[RESET_SLOTS_BEGIN + ledIndex * 24 + i] = tempBuffer[i];
}

void ws2812_fillBufferBlack() {
  // All LEDs off
  uint32_t index, buffIndex;
  buffIndex = 0;

  for (index = 0; index < RESET_SLOTS_BEGIN; index++) {
    LedBuffer[buffIndex] = WS2812_RESET;
    buffIndex++;
  }
  for (index = 0; index < LED_DATA_SIZE; index++) {
    LedBuffer[buffIndex] = WS2812_0;
    buffIndex++;
  }
  LedBuffer[buffIndex] = WS2812_0;
  buffIndex++;
  for (index = 0; index < RESET_SLOTS_END; index++) {
    LedBuffer[buffIndex] = 0;
    buffIndex++;
  }
}

void ws2812_fillBufferWhite() {
  // All LEDs on
  uint32_t index, buffIndex;
  buffIndex = 0;

  for (index = 0; index < RESET_SLOTS_BEGIN; index++) {
    LedBuffer[buffIndex] = WS2812_RESET;
    buffIndex++;
  }
  for (index = 0; index < LED_DATA_SIZE; index++) {
    LedBuffer[buffIndex] = WS2812_1;
    buffIndex++;
  }
  LedBuffer[buffIndex] = WS2812_0;
  buffIndex++;
  for (index = 0; index < RESET_SLOTS_END; index++) {
    LedBuffer[buffIndex] = 0;
    buffIndex++;
  }
}

