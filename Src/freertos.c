/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "WS2812.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void initialisation();
Equipe couleurEquipe();
bool auDebloque();
bool declenchementRobot();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

LedsState ledsState = LEDS_BLANK;
int ascenseurPosition = ASC_BAS;
int ascenseurPositionPrec = -1;
int ascenseurPositionTarget = ASC_BAS;

Equipe equipe = JAUNE;
const CouleurRGB jauneColor = {
  .r = 0,
  .g = 100,
  .b = 100,
};
const CouleurRGB bleuColor = {
  .r = 0,
  .g = 0,
  .b = 100,
};

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ledsTask */
osThreadId_t ledsTaskHandle;
const osThreadAttr_t ledsTask_attributes = {
  .name = "ledsTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for heartBeatTimer */
osTimerId_t heartBeatTimerHandle;
const osTimerAttr_t heartBeatTimer_attributes = {
  .name = "heartBeatTimer"
};
/* Definitions for servoTimer */
osTimerId_t servoTimerHandle;
const osTimerAttr_t servoTimer_attributes = {
  .name = "servoTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartLedsTask(void *argument);
void heartBeatCallback(void *argument);
void servoCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of heartBeatTimer */
  heartBeatTimerHandle = osTimerNew(heartBeatCallback, osTimerPeriodic, NULL, &heartBeatTimer_attributes);

  /* creation of servoTimer */
  servoTimerHandle = osTimerNew(servoCallback, osTimerPeriodic, NULL, &servoTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartMainTask, NULL, &mainTask_attributes);

  /* creation of ledsTask */
  ledsTaskHandle = osThreadNew(StartLedsTask, NULL, &ledsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  Equipe equipePrec;

  osTimerStart(heartBeatTimerHandle, 1000);

  initialisation();
  osTimerStart(servoTimerHandle, SPEED_ASC);
  equipePrec = equipe;

  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while(true)
  {
    equipe = couleurEquipe();
    if (equipe != equipePrec) {
      equipePrec = equipe;
      LedsState prec = ledsState;
      ledsState = LEDS_MATCH;
      osDelay(5000);
      ledsState = prec;
    }

    if (declenchementRobot()) {
      ascenseurPositionTarget = ASC_HAUT;
      ledsState = LEDS_MATCH;
    } else {
      ascenseurPositionTarget = ASC_BAS;
      ledsState = LEDS_BLANK;
    }

    osDelay(1000);
  }
#pragma clang diagnostic pop
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartLedsTask */
/**
* @brief Function implementing the ledsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedsTask */
void StartLedsTask(void *argument)
{
  /* USER CODE BEGIN StartLedsTask */
  int ledIndex = 1, ledIndexPrev, ledIndexSuiv;
  int nbCycleBeforeFlash = 0;
  CouleurRGB applyColor;

  // Init LEDs
  ws2812_Init();

  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while(true)
  {
    switch (ledsState) {
      case LEDS_BLANK: {
        ws2812_SetAllLedsColor(0, 0, 0);
        osDelay(500);
        break;
      }

      case LEDS_ERROR: {
        ws2812_SetAllLedsColor(100, 0, 0);
        osDelay(100);
        ws2812_SetAllLedsColor(0, 0, 0);
        osDelay(300);
        break;
      }

      case LEDS_OK: {
        ws2812_SetAllLedsColor(0, 100, 0);
        osDelay(100);
        ws2812_SetAllLedsColor(0, 0, 0);
        osDelay(300);
        break;
      }

      case LEDS_MATCH: {
        ledIndexSuiv = ledIndex + 1;
        if (ledIndexSuiv >= LED_NUMBER/2) {
          ledIndexSuiv = 0;
        }
        ledIndexPrev = ledIndex - 1;
        if (ledIndexPrev < 0) {
          ledIndexPrev = LED_NUMBER/2 - 1;
        }

        if (equipe == JAUNE) {
          applyColor = jauneColor;
        } else {
          applyColor = bleuColor;
        }

        ws2812_SetAllLedsColor(0, 0, 0);
        ws2812_SetLedColor(ledIndexPrev, applyColor.r / 2, applyColor.g / 2, applyColor.b / 2);
        ws2812_SetLedColor(ledIndex, applyColor.r, applyColor.g, applyColor.b);
        ws2812_SetLedColor(ledIndexSuiv, applyColor.r / 2, applyColor.g / 2, applyColor.b / 2);
        ws2812_SetLedColor(ledIndexPrev + LED_NUMBER/2, applyColor.r / 2, applyColor.g / 2, applyColor.b / 2);
        ws2812_SetLedColor(ledIndex + LED_NUMBER/2, applyColor.r, applyColor.g, applyColor.b);
        ws2812_SetLedColor(ledIndexSuiv + LED_NUMBER/2, applyColor.r / 2, applyColor.g / 2, applyColor.b / 2);

        ledIndex++;
        if (ledIndex >= LED_NUMBER/2) {
          ledIndex = 0;
          nbCycleBeforeFlash++;

          if (nbCycleBeforeFlash > 3) {
            nbCycleBeforeFlash = 0;
            for (int idx = 0; idx < 3; idx++) {
              ws2812_SetAllLedsColor(100, 100, 100);
              osDelay(100);
              ws2812_SetAllLedsColor(0, 0, 0);
              osDelay(300);
            }
          }
        }

        osDelay(100);
      }
    }
  }
#pragma clang diagnostic pop
  /* USER CODE END StartLedsTask */
}

/* heartBeatCallback function */
void heartBeatCallback(void *argument)
{
  /* USER CODE BEGIN heartBeatCallback */
  HAL_GPIO_TogglePin(GreenLed_GPIO_Port, GreenLed_Pin);
  /* USER CODE END heartBeatCallback */
}

/* servoCallback function */
void servoCallback(void *argument)
{
  /* USER CODE BEGIN servoCallback */
  if (ascenseurPosition < ascenseurPositionTarget) {
    ascenseurPosition += 10;
    if (ascenseurPosition > ascenseurPositionTarget) {
      ascenseurPosition = ascenseurPositionTarget;
    }
  } else if (ascenseurPosition > ascenseurPositionTarget) {
    ascenseurPosition -= 10;
    if (ascenseurPosition < ascenseurPositionTarget) {
      ascenseurPosition = ascenseurPositionTarget;
    }
  }

  if (ascenseurPosition != ascenseurPositionPrec) {
    ascenseurPositionPrec = ascenseurPosition;
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, ascenseurPosition);
  }

  /* USER CODE END servoCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void initialisation() {
  // Check de l'AU au boot
  if (!auDebloque()) {

    // AU KO, erreur
    ledsState = LEDS_ERROR;
    while(!auDebloque()) {
      osDelay(500);
    }
  }
  ledsState = LEDS_OK;
  osDelay(2000);

  // Affichage en fonction de la couleur selectionné
  equipe = couleurEquipe();
  ledsState = LEDS_MATCH;
  osDelay(5000);

  // Fin cycle d'init
  ledsState = LEDS_BLANK;
}

// Détermination du mode de fonctionnement
Equipe couleurEquipe() {
  if(HAL_GPIO_ReadPin(SelectionEquipe_GPIO_Port, SelectionEquipe_Pin) == GPIO_PIN_RESET) {
    return BLEU;
  } else {
    return JAUNE;
  }
}

// Est-ce que l'arret d'urgence est
bool auDebloque() {
  return HAL_GPIO_ReadPin(ArretUrgence_GPIO_Port, ArretUrgence_Pin) == GPIO_PIN_SET;
}

bool declenchementRobot() {
  return HAL_GPIO_ReadPin(DeclenchementRobot_GPIO_Port, DeclenchementRobot_Pin) == GPIO_PIN_RESET;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
