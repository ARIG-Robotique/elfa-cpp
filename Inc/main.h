/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {
    LEDS_ERROR = 0U,
    LEDS_OK,
    LEDS_MATCH,
    LEDS_BLANK
} LedsState;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// Valeur ascenseur
#define SPEED_ASC        10
#define ASC_INIT         1500
#define ASC_HAUT         2460
#define ASC_BAS     	   700

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BluePushButton_Pin GPIO_PIN_13
#define BluePushButton_GPIO_Port GPIOC
#define NeopixelSignal_Pin GPIO_PIN_0
#define NeopixelSignal_GPIO_Port GPIOA
#define GreenLed_Pin GPIO_PIN_5
#define GreenLed_GPIO_Port GPIOA
#define AscenseurPWM_Pin GPIO_PIN_6
#define AscenseurPWM_GPIO_Port GPIOA
#define DeclenchementRobot_Pin GPIO_PIN_8
#define DeclenchementRobot_GPIO_Port GPIOA
#define PositionPhare_Pin GPIO_PIN_9
#define PositionPhare_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ArretUrgence_Pin GPIO_PIN_5
#define ArretUrgence_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
