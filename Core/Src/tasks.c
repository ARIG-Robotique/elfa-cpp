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

void stateMachine()
{
	// PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	ws2812_Init();
	//ws2812_SetAllLedsColor(255, 255, 255);

	enum state current_state = INIT, next_state;

	uint32_t duty_cycle;
	int acceleration_direction;
	int rotation_direction;

	uint32_t last_encoder_value = 0;

	/* Infinite loop */
	for(;;)
	{
		int pres_robot = !HAL_GPIO_ReadPin(PRES_AVANT_GPIO_Port, PRES_AVANT_Pin);
		int au = HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin);
		uint32_t encoder_value = TIM2->CNT;

		uint32_t encoder_diff = encoder_value - last_encoder_value;

		uint32_t rotation_round_per_second = encoder_diff * 1000 / PERIOD / ENCODER_PULSES_PER_ROTATION;

		printf("test");

		switch(current_state)
		{
		case INIT:
			// Extinction Moteur
			HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_RESET);
			acceleration_direction = 1;
			rotation_direction = 1;

			// Vitesse
			duty_cycle = MIN_DUTY_CYCLE;

			next_state = WAIT;
			break;

		case WAIT:
			if(pres_robot)
			{
				next_state = PRES_ROBOT;
			}

			if(!au)
			{
				next_state = INIT;
			}
			break;

		case PRES_ROBOT:
			if(!pres_robot)
			{
				next_state = STATUETTE_OK;
			}

			if(!au)
			{
				next_state = INIT;
			}
			break;

		case STATUETTE_OK:
			// Allumage Moteur

			if(!au)
			{
				next_state = INIT;
			}
			else
			{
				HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_SET);

				TIM3->CCR2 = PWM_TIMER_ARR / 2;

//				if(duty_cycle >= MAX_DUTY_CYCLE)
//					acceleration_direction = -1;
//
//				else if(duty_cycle <= MIN_DUTY_CYCLE)
//				{
//					rotation_direction *= -1;
//					acceleration_direction = 1;
//				}
//
//				if(rotation_direction == 1)
//				{
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_SET);
//				}
//				else if(rotation_direction == -1)
//				{
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_SET);
//				}
//
//				duty_cycle += 1 * acceleration_direction;
//
//				TIM3->CCR2 = PWM_TIMER_ARR - (duty_cycle * PWM_TIMER_ARR / DUTY_CYCLE_RATIO);


			}
			break;
		}

		last_encoder_value = encoder_value;
		current_state = next_state;
		osDelay(PERIOD);
	}
}
