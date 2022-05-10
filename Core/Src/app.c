/*
 * tasks.c
 *
 *  Created on: 2 mai 2022
 *      Author: MatPy
 */

#include "WS2812.h"
#include "tim.h"
#include "cmsis_os.h"


#define MAX_DUTY_CYCLE   67
#define MIN_DUTY_CYCLE   64
#define DUTY_CYCLE_RATIO 100

#define MOTOR_AXIS_GEARS 20
#define TRAY_AXIS_GEARS  70

#define ENCODER_PULSES_PER_ROTATION  663
#define MOTOR_ROTATION_SPEED_12V     143

#define PERIOD 100

enum state {INIT, WAIT, PRES_ROBOT, STATUETTE_OK};

enum LedState{LED_OFF, LED_RED, LED_PATTERN};
enum MotorState{MOTOR_OFF, MOTOR_ON};

enum LedState ledUp = LED_OFF;

enum MotorState motorUp = MOTOR_OFF;

void stateMachine()
{
	enum state current_state = INIT, next_state = INIT;

	/* Infinite loop */
	for(;;)
	{
		int pres_robot = !HAL_GPIO_ReadPin(PRES_AVANT_GPIO_Port, PRES_AVANT_Pin);
		int au = !HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin);

		switch(current_state)
		{
		case INIT:
			if(!au){
				next_state = WAIT;
			}
			else
			{
				ledUp = LED_RED;
				motorUp = MOTOR_OFF;
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
			else
			{
				ledUp = LED_OFF;
				motorUp = MOTOR_OFF;
			}
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
			break;

		case STATUETTE_OK:
			if(au)
			{
				next_state = INIT;
			}
			else
			{
				motorUp = MOTOR_ON;
				ledUp = LED_PATTERN;
			}
			break;
		}

		current_state = next_state;
		osDelay(PERIOD);
	}
}


void ledTask()
{
	ws2812_Init();

	ws2812_SetAllLedsColor(50, 50, 50);

	osDelay(2000);
	ws2812_Reset();

	for(;;){
		switch(ledUp){
		case LED_OFF:
			ws2812_Reset();
			break;

		case LED_RED:
			ws2812_SetAllLedsColor(255, 0, 0);
			break;

		case LED_PATTERN:
			circularTray();
			rocketColumns();
			break;
		}

		osDelay(100);
	}
}

void motorTask()
{
	// PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_RESET);
	TIM3->CCR2 = 90;

	HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_RESET);


	for(;;){
		switch(motorUp){
		case MOTOR_ON:
			HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_SET);
			break;

		case MOTOR_OFF:
			HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_RESET);
			break;
		}
		osDelay(100);
	}
}

