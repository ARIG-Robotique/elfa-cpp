/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_TIMER_ARR 99
#define ENCODER_CHA_Pin GPIO_PIN_15
#define ENCODER_CHA_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define AU_Pin GPIO_PIN_13
#define AU_GPIO_Port GPIOG
#define PRES_AVANT_Pin GPIO_PIN_12
#define PRES_AVANT_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOK
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOG
#define ENCODER_CHB_Pin GPIO_PIN_1
#define ENCODER_CHB_GPIO_Port GPIOA
#define WAKEUP_Pin GPIO_PIN_0
#define WAKEUP_GPIO_Port GPIOA
#define WS2812_Pin GPIO_PIN_2
#define WS2812_GPIO_Port GPIOA
#define MOT_STBY_2_Pin GPIO_PIN_6
#define MOT_STBY_2_GPIO_Port GPIOA
#define MOT_AIN2_Pin GPIO_PIN_6
#define MOT_AIN2_GPIO_Port GPIOH
#define MOT_PWM_Pin GPIO_PIN_7
#define MOT_PWM_GPIO_Port GPIOA
#define MOT_STBY_Pin GPIO_PIN_14
#define MOT_STBY_GPIO_Port GPIOB
#define MOT_AIN1_Pin GPIO_PIN_15
#define MOT_AIN1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
