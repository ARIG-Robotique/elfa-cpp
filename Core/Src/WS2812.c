//
// Created by gdepuille on 17/01/2020.
//

#include "WS2812.h"
#include "main.h"

uint8_t r[LED_NUMBER];
uint8_t g[LED_NUMBER];
uint8_t b[LED_NUMBER];

int circularIndex;
enum RocketColor{ROCKET_WHITE, ROCKET_GREEN, ROCKET_RED, ROCKET_BLUE, ROCKET_NB} rocketColor;
int columnIndex;

int blinkingCounter;

void ws2812_fillBuffer(uint8_t colorValue);
void ws2812_fillBufferBlack(void);
void ws2812_fillBufferWhite(void);

void ws2812_Init(void) {
	ws2812_Reset();
	// Start PWM Generator from DMA
    if (HAL_TIM_PWM_Start_DMA(&WS2812_TIM, WS2812_CHANNEL, (uint32_t *) LedBuffer, LED_BUFFER_SIZE) != HAL_OK){
    	Error_Handler();
    }
}


void ws2812_Reset(void)
{
	ws2812_fillBufferBlack();
	circularIndex = 0;
	columnIndex = 0;
	rocketColor = ROCKET_GREEN;
	blinkingCounter = 0;
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

void ws2812_SetLedColor(uint32_t ledNumber, uint8_t red, uint8_t green, uint8_t blue) {
	uint32_t tempBuffer[24];
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
	 ws2812_fillBuffer(WS2812_0);
}

void ws2812_fillBufferWhite() {
	 ws2812_fillBuffer(WS2812_1);
}

void ws2812_fillBuffer(uint8_t colorValue) {
	// All LEDs on
	uint32_t index;

	for (index = 0; index < RESET_SLOTS_BEGIN; index++) {
		LedBuffer[index] = WS2812_RESET;
	}
	for (index = RESET_SLOTS_BEGIN; index < RESET_SLOTS_BEGIN + LED_DATA_SIZE; index++) {
		LedBuffer[index] = colorValue;
	}

	for (index = RESET_SLOTS_BEGIN + LED_DATA_SIZE ;
			index < RESET_SLOTS_BEGIN + LED_DATA_SIZE + RESET_SLOTS_END;
			index++) {
		LedBuffer[index] = WS2812_RESET;
	}
}

void circularTray(void){

	for(int ledNumber = 0 ; ledNumber < TRAY_LED_NUMBER ; ledNumber++)
	{
		int newLedNumber = (ledNumber + circularIndex) % TRAY_LED_NUMBER;
		if (ledNumber / 4 % 2)
			ws2812_SetLedColor(newLedNumber, 0, 255, 0);
		else
			ws2812_SetLedColor(newLedNumber, 127, 127, 127);
	}

	circularIndex++;
	circularIndex %= TRAY_LED_NUMBER;
}


void setColumnLed(int columnNumber, uint8_t red, uint8_t green, uint8_t blue){
	if(columnNumber < 0 || columnNumber >= COLUMN_LED_NUMBER)
		return;

	int column1 = COLUMN1_LED_OFFSET + columnNumber, column2 = COLUMN2_LED_OFFSET + columnNumber;

	ws2812_SetLedColor(column1, red, green, blue);
	ws2812_SetLedColor(column2, red, green, blue);
}


void rocketColumns(void){
	for(int ledNumber = 0 ; ledNumber < COLUMN_LED_NUMBER ; ledNumber++)
	{
		setColumnLed(ledNumber, 0, 0, 0);
	}

	for(int trailIndex = 0 ; trailIndex < TRAIL_LENGTH ; trailIndex++)
	{
		int ledNumber = columnIndex - trailIndex;

		if(ledNumber >= 0 && ledNumber < COLUMN_LED_NUMBER)
		{
			uint8_t intensity =  255 / (1 << trailIndex);

			switch(rocketColor){
			case ROCKET_GREEN:
				setColumnLed(ledNumber, 0, intensity, 0);
				break;
			case ROCKET_RED:
				setColumnLed(ledNumber, intensity, 0, 0);
				break;
			case ROCKET_BLUE:
				setColumnLed(ledNumber, 0, 0, intensity);
				break;
			case ROCKET_WHITE:
				setColumnLed(ledNumber, intensity / 3, intensity / 3, intensity / 3);
				break;
			default:
				break;
			}
		}
	}

	columnIndex++;
	if(columnIndex > COLUMN_LED_NUMBER + TRAIL_LENGTH)
	{
		columnIndex = 0;
		rocketColor++;
		rocketColor %= ROCKET_NB;
	}
}


int testIndex = 0;
void testLed(){
	for(int ledNumber = 0 ; ledNumber < TRAY_LED_NUMBER ; ledNumber++)
		ws2812_SetLedColor(ledNumber, 0, 0, 0);

	testIndex++;
	testIndex %= LED_NUMBER * 10;
}


void blinkingRed(void)
{
	if(blinkingCounter < BLINKING_LED_PERIOD / 2)
		ws2812_SetAllLedsColor(255, 0, 0);
	else
		ws2812_SetAllLedsColor(0, 0, 0);

	blinkingCounter++;
	blinkingCounter %= BLINKING_LED_PERIOD;
}
