#include "motor.h"
#include "gpio.h"
#include "tim.h"
#include <math.h>

#define MAX_DUTY_CYCLE   67
#define MIN_DUTY_CYCLE   64
#define DUTY_CYCLE_RATIO 100

#define MOTOR_REDUCTOR_RATIO 51

#define MOTOR_AXIS_GEARS 20
#define TRAY_AXIS_GEARS  70

#define ENCODER_PULSES_PER_ROTATION  663
#define MOTOR_ROTATION_SPEED_12V     143

float orderSpeed = 0;

void motorInit(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_SET);

	setMotorSpeed(0);
}

void setMotorSpeed(float speed){
	if(speed == 0){
		HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(MOT_STBY_GPIO_Port, MOT_STBY_Pin, GPIO_PIN_SET);
	}

	if(speed >= 0){
		HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(MOT_AIN1_GPIO_Port, MOT_AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOT_AIN2_GPIO_Port, MOT_AIN2_Pin, GPIO_PIN_RESET);
	}

	orderSpeed = fabs(speed);
}

void controlSpeed(int msElapsed){
	TIM3->CCR2 = orderSpeed;
}
