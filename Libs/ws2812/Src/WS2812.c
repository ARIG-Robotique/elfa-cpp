//
// Created by gregorydepuille@sglk.local on 17/01/2020.
//

#include "WS2812.h"

extern TIM_HandleTypeDef htim2;
extern TIM_OC_InitTypeDef htim2Config;

void ws2812_init() {
  ws2812_fillBufferBlack();
  ws2812_update();
}

void ws2812_update() {
  HAL_TIM_PWM_ConfigChannel(&htim2, &htim2Config, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *) LedBuffer, LED_BUFFER_SIZE);
}

void ws2812_setAllLedsColor(uint8_t red, uint8_t green, uint8_t blue) {
  for (uint32_t idx = 0 ; idx < LED_NUMBER ; idx++) {
    ws2812_setLedColor(idx, red, green, blue);
  }
}

void ws2812_setLedColor(uint32_t ledNumber, uint8_t red, uint8_t green,
                         uint8_t blue) {
  uint8_t tempBuffer[24];
  uint32_t i;
  uint32_t ledIndex = ledNumber % LED_NUMBER;

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

