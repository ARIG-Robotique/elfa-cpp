/*
 * tasks.c
 *
 *  Created on: 2 mai 2022
 *      Author: MatPy
 */

#include "WS2812.h"
#include "tim.h"
#include "cmsis_os.h"
#include "app.h"
#include "usart.h"
#include "stdlib.h"
#include "stdio.h"


#define MAX_DUTY_CYCLE   67
#define MIN_DUTY_CYCLE   64
#define DUTY_CYCLE_RATIO 100

#define VALIDATION_PERIOD 20

#define MOTOR_AXIS_GEARS 20
#define TRAY_AXIS_GEARS  70
#define GEARS_RATIO      (TRAY_AXIS_GEARS / MOTOR_AXIS_GEARS)

#define ENCODER_PULSES_PER_ROTATION  663
#define MOTOR_ROTATION_SPEED_12V     143

#define PERIOD_STATE_MACHINE 100
#define PERIOD_LED           100
#define PERIOD_MOTOR         100
#define FREQ_MOTOR           (1000 / PERIOD_MOTOR)



enum state {INIT, VALIDATION, WAIT, PRES_ROBOT, STATUETTE_OK};

enum LedCmd{LED_OFF, LED_RED, LED_PATTERN, LED_OK};
enum MotorCmd{MOTOR_OFF, MOTOR_ON};

enum LedCmd ledCmd = LED_OFF;
enum MotorCmd motorCmd = MOTOR_OFF;

int32_t traySpeed = 0;

int validationCounter;

void stateMachine()
{
	enum state current_state = INIT, next_state = INIT;

	for(;;)
	{
		int pres_robot = !HAL_GPIO_ReadPin(PRES_AVANT_GPIO_Port, PRES_AVANT_Pin);
		int au = !HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin);

		switch(current_state)
		{
		case INIT:
			motorCmd = MOTOR_OFF;
			validationCounter = VALIDATION_PERIOD;
			if(!au){
				next_state = VALIDATION;
			}
			else{
				ledCmd = LED_RED;
			}
			break;

		case VALIDATION:
			ledCmd = LED_OK;
			validationCounter--;

			if(validationCounter <= 0){
				next_state = WAIT;
			}
			break;

		case WAIT:
			if(au)
			{
				next_state = INIT;
			}
			else if(pres_robot)
			{
				next_state = PRES_ROBOT;
			}
			ledCmd = LED_OFF;
			break;

		case PRES_ROBOT:
			if(au)
			{
				next_state = INIT;
			}
			else if(!pres_robot)
			{
				next_state = STATUETTE_OK;
			}
			ledCmd = LED_PATTERN;
			break;

		case STATUETTE_OK:
			if(au)
			{
				next_state = INIT;
			}
			motorCmd = MOTOR_ON;
			break;
		}

		current_state = next_state;
		osDelay(PERIOD_STATE_MACHINE);
	}
}


void ledTask()
{
	ws2812_Init();

	for(;;){
		switch(ledCmd){
		case LED_OFF:
			ws2812_Reset();
			break;

		case LED_OK:
			ws2812_SetAllLedsColor(0, 255, 0);
			break;

		case LED_RED:
			blinkingRed();
			break;

		case LED_PATTERN:
			//circularTray();
			//rocketColumns();
			testLed();
			break;
		}
		int delay = 100;
		if(traySpeed > 5 && traySpeed < 100)
			delay = 1000 * 60 / traySpeed / TRAY_LED_NUMBER;
		osDelay(delay);
	}
}

void motorTask()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_RESET);
	TIM3->CCR2 = 90;

	uint32_t counterValue = 0, lastCounterValue = 0;
	int32_t encoderDiff = 0, motorSpeed = 0;

	for(;;){
		switch(motorCmd){
		case MOTOR_ON:
			HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_SET);
			break;

		case MOTOR_OFF:
			HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_RESET);
			break;
		}

		counterValue = TIM2->CNT;
		encoderDiff = (int32_t) (counterValue / 4) - (int32_t) (lastCounterValue / 4);
		motorSpeed = encoderDiff * 60 * FREQ_MOTOR / ENCODER_PULSES_PER_ROTATION;

		traySpeed = motorSpeed / GEARS_RATIO;

		lastCounterValue = counterValue;

		osDelay(PERIOD_MOTOR);
	}
}

