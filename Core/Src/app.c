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

char ledUp = 0;
char motorUp = 0;

void stateMachine()
{
	enum state current_state = INIT, next_state;

	/* Infinite loop */
	for(;;)
	{
		int pres_robot = !HAL_GPIO_ReadPin(PRES_AVANT_GPIO_Port, PRES_AVANT_Pin);
		int au = HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin);

		switch(current_state)
		{
		case INIT:
			ledUp = 0;
			motorUp = 0;

			next_state = WAIT;
			break;

		case WAIT:
			if(!au)
			{
				next_state = INIT;
			}
			else if(pres_robot)
			{
				next_state = PRES_ROBOT;
			}
			break;

		case PRES_ROBOT:
			if(!au)
			{
				next_state = INIT;
			}
			else if(!pres_robot)
			{
				next_state = STATUETTE_OK;
			}
			break;

		case STATUETTE_OK:
			if(!au)
			{
				next_state = INIT;
			}
			else
			{
				motorUp = 1;
				ledUp = 1;
			}
			break;
		}

		current_state = next_state;
		osDelay(PERIOD);
	}
}


void ledTask()
{
	char lastLedUp = 0;

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

	for(;;){
		if(lastLedUp != ledUp){
			if(ledUp){
				TIM5->CCR3 = PWM_TIMER_ARR - PWM_TIMER_ARR * 2 / 3;
			}
			else{
				TIM5->CCR3 = PWM_TIMER_ARR - PWM_TIMER_ARR * 1 / 3;
			}
		}
		else{
			TIM5->CCR3 = PWM_TIMER_ARR;
		}
		lastLedUp = ledUp;
		osDelay(100);
	}
}

void motorTask()
{
	// PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_SET);
	TIM3->CCR2 = 0;

	for(;;){
		if(motorUp){
			HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_RESET);
		}
		osDelay(100);
	}
}

