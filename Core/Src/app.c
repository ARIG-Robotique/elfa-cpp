/*
 * tasks.c
 *
 *  Created on: 2 mai 2022
 *      Author: MatPy
 */

#include "WS2812.h"
#include "tim.h"
#include "cmsis_os.h"
#include "usart.h"
#include "motor.h"

#define VALIDATION_PERIOD    20

#define PERIOD_STATE_MACHINE 100
#define PERIOD_LED           100
#define PERIOD_MOTOR         100

#define MOTOR_CHECK_SPEED  40
#define MOTOR_OK_SPEED     60

enum state {INIT, VALIDATION, WAIT, PRES_ROBOT, STATUETTE_OK};

enum LedCmd{LED_OFF, LED_RED, LED_PATTERN, LED_OK, LED_TRAY_LIGHT};
enum MotorCmd{MOTOR_OFF, MOTOR_ON, MOTOR_CHECK};

enum LedCmd ledCmd = LED_OFF;
enum MotorCmd motorCmd = MOTOR_OFF;

int validationCounter;

void stateMachine()
{
	enum state current_state = INIT, next_state = INIT;

	for(;;)
	{
		int pres_robot = !HAL_GPIO_ReadPin(PRES_AVANT_GPIO_Port, PRES_AVANT_Pin);
		int pres_statuette = !HAL_GPIO_ReadPin(PRES_STATUETTE_GPIO_Port, PRES_STATUETTE_Pin);
		int au = !HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin);

		switch(current_state)
		{
		case INIT:
			motorCmd = MOTOR_OFF;
			validationCounter = 0;
			if(!au){
				next_state = VALIDATION;
			}
			else{
				ledCmd = LED_RED;
			}
			break;

		case VALIDATION:
			if(au){
				next_state = INIT;
			}
			else{
				ledCmd = LED_OK;
				validationCounter++;

				if(validationCounter > VALIDATION_PERIOD){
					next_state = WAIT;
				}
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
			else if(pres_statuette)
			{
				next_state = STATUETTE_OK;
			}
			ledCmd = LED_OFF;
			break;

		case PRES_ROBOT:
			if(au)
			{
				next_state = INIT;
			}
			else if(pres_statuette)
			{
				next_state = STATUETTE_OK;
			}
			motorCmd = MOTOR_CHECK;
			ledCmd = LED_TRAY_LIGHT;
			break;

		case STATUETTE_OK:
			if(au)
			{
				next_state = INIT;
			}
			ledCmd = LED_PATTERN;
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
			validation(VALIDATION_PERIOD);
			break;

		case LED_RED:
			blinkingRed();
			break;

		case LED_PATTERN:
			circularTray();
			rocketColumns();
			break;

		case LED_TRAY_LIGHT:
			setTrayColor(64, 64, 64);
			break;
		}

		osDelay(PERIOD_LED);
	}
}

void motorTask()
{
	motorInit();

	for(;;){
		switch(motorCmd){
		case MOTOR_CHECK:
			setMotorSpeed(MOTOR_CHECK_SPEED);
			break;

		case MOTOR_ON:
			setMotorSpeed(MOTOR_OK_SPEED);
			break;

		case MOTOR_OFF:
			setMotorSpeed(0);
			break;
		}

		controlSpeed(PERIOD_MOTOR);
		osDelay(PERIOD_MOTOR);
	}
}

