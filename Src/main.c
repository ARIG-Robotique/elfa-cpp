/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <WS2812.h>
//#include <ssd1306.h>
#include <SD21.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;

osThreadId heartBeatTaskHandle;
osThreadId ledsUpdateTaskHandle;
osThreadId servoTaskHandle;
osThreadId mainProcessTaskHandle;
/* USER CODE BEGIN PV */

LedsState ledsState = LEDS_BLANK;
int ascenseurPosition = ASC_BAS;

TIM_OC_InitTypeDef htim2Config;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void heartBeat(void const * argument);
void ledsUpdate(void const * argument);
void servo(void const * argument);
void mainProcess(void const * argument);

/* USER CODE BEGIN PFP */

void initialisation();

bool positionPhare();

bool auDebloque();

bool declenchementRobot();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of heartBeatTask */
  osThreadDef(heartBeatTask, heartBeat, osPriorityIdle, 0, 128);
  heartBeatTaskHandle = osThreadCreate(osThread(heartBeatTask), NULL);

  /* definition and creation of ledsUpdateTask */
  osThreadDef(ledsUpdateTask, ledsUpdate, osPriorityRealtime, 0, 128);
  ledsUpdateTaskHandle = osThreadCreate(osThread(ledsUpdateTask), NULL);

  /* definition and creation of servoTask */
  osThreadDef(servoTask, servo, osPriorityIdle, 0, 128);
  servoTaskHandle = osThreadCreate(osThread(servoTask), NULL);

  /* definition and creation of mainProcessTask */
  osThreadDef(mainProcessTask, mainProcess, osPriorityNormal, 0, 128);
  mainProcessTaskHandle = osThreadCreate(osThread(mainProcessTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
    htim2.Init.RepetitionCounter = LED_BUFFER_SIZE + 1;
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (uint32_t)( (SystemCoreClock / TIMER_CLOCK_FREQ) - 1);
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIMER_PERIOD - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

    htim2Config = sConfigOC;

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GreenLed_GPIO_Port, GreenLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BluePushButton_Pin */
  GPIO_InitStruct.Pin = BluePushButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BluePushButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 PC0 PC1
                           PC2 PC3 PC4 PC5
                           PC6 PC7 PC8 PC9
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA6 PA7
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GreenLed_Pin */
  GPIO_InitStruct.Pin = GreenLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GreenLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DeclenchementRobot_Pin PositionPhare_Pin */
  GPIO_InitStruct.Pin = DeclenchementRobot_Pin|PositionPhare_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ArretUrgence_Pin */
  GPIO_InitStruct.Pin = ArretUrgence_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ArretUrgence_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void initialisation() {
    // Init screen
    //ssd1306_Init();

    //ssd1306_Fill(Black);
    //ssd1306_SetCursor(2, 0);
    //ssd1306_WriteString("Scan I2C bus :", Font_7x10, White);

    HAL_StatusTypeDef result;
    uint8_t i, nbDevice = 0;
    for (i = 1; i < 128; i++) {
        /*
         * the HAL wants a left aligned i2c address
         * &hi2c1 is the handle
         * (uint16_t)(i<<1) is the i2c address left aligned
         * retries 2
         * timeout 2
         */
        result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 2, 2);
        if (result == HAL_OK) {
            //char buf[10];
            //sprintf(buf, "Add 0x%X", i); // Received an ACK at that address
            //ssd1306_SetCursor(2, (nbDevice + 1) * 10);
            //ssd1306_WriteString(buf, Font_7x10, White);
            nbDevice++;
        }
    }
    //ssd1306_UpdateScreen();
    osDelay(4000);

    if (nbDevice != NB_I2C_DEVICES) {
        ledsState = LEDS_ERROR;

//        ssd1306_Fill(Black);
//        ssd1306_SetCursor(2, 0);
//        ssd1306_WriteString("Erreur I2C", Font_7x10, White);
//        ssd1306_UpdateScreen();

        // Il manque des devices on bloque tout
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
        while (true);
#pragma clang diagnostic pop
    }

    if (!auDebloque()) {
        ledsState = LEDS_ERROR;

//        ssd1306_Fill(Black);
//        ssd1306_SetCursor(2, 0);
//        ssd1306_WriteString("AU KO", Font_7x10, White);
//        ssd1306_UpdateScreen();

        while(!auDebloque()) {
            osDelay(500);
        }
    }
    ledsState = LEDS_OK;

//    uint8_t sd21Version = sd21_GetVersion();
//    char buf[10];
//    sprintf(buf, "SD21 v %i", sd21Version);
//    ssd1306_Fill(Black);
//    ssd1306_SetCursor(2, 0);
//    ssd1306_WriteString(buf, Font_7x10, White);
//    ssd1306_UpdateScreen();

    // Position servo init
    sd21_SetPositionAndSpeed(SERVO_ASC_NB, SPEED_ASC, ASC_BAS);

    osDelay(4000);

    ledsState = LEDS_BLANK;
}

// Détermination du mode de fonctionnement
bool positionPhare() {
    return HAL_GPIO_ReadPin(PositionPhare_GPIO_Port, PositionPhare_Pin) == GPIO_PIN_RESET;
}

// Est-ce que l'arret d'urgence est
bool auDebloque() {
    return HAL_GPIO_ReadPin(ArretUrgence_GPIO_Port, ArretUrgence_Pin) == GPIO_PIN_SET;
}

bool declenchementRobot() {
    return HAL_GPIO_ReadPin(DeclenchementRobot_GPIO_Port, DeclenchementRobot_Pin) == GPIO_PIN_RESET;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_heartBeat */
/**
 * @brief  Function implementing the heartBeatTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_heartBeat */
void heartBeat(void const * argument)
{
  /* USER CODE BEGIN 5 */

    /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        HAL_GPIO_TogglePin(GreenLed_GPIO_Port, GreenLed_Pin);

        osDelay(1000);
    }
#pragma clang diagnostic pop
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ledsUpdate */
/**
 * @brief Function implementing the ledsUpdateTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ledsUpdate */
void ledsUpdate(void const * argument)
{
  /* USER CODE BEGIN ledsUpdate */
    int ledIndex = 1, ledIndexPrev, ledIndexSuiv;
    //int ledIndex2 = 7, ledIndex2Prev, ledIndex2Suiv;

    // Init LEDs
    ws2812_Init();
    ws2812_SetAllLedsColor(0, 0, 0);

    /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
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
                if (ledIndexSuiv >= LED_NUMBER) {
                    ledIndexSuiv = 0;
                }
                /*
                ledIndex2Suiv = ledIndex2 + 1;
                if (ledIndex2Suiv >= LED_NUMBER) {
                    ledIndex2Suiv = 0;
                }
                 */
                ledIndexPrev = ledIndex - 1;
                if (ledIndexPrev < 0) {
                    ledIndexPrev = LED_NUMBER - 1;
                }
                /*
                ledIndex2Prev = ledIndex2 - 1;
                if (ledIndex2Prev < 0) {
                    ledIndex2Prev = LED_NUMBER - 1;
                }
                */

                ws2812_SetAllLedsColor(0, 0, 0);
                ws2812_SetLedColor(ledIndexPrev, 50, 50, 50);
                ws2812_SetLedColor(ledIndex, 100, 100, 100);
                ws2812_SetLedColor(ledIndexSuiv, 50, 50, 50);

                //ws2812_SetLedColor(ledIndex2Prev, 10, 0, 0);
                //ws2812_SetLedColor(ledIndex2, 50, 0, 50);
                //ws2812_SetLedColor(ledIndex2Suiv, 10, 0, 0);

                ledIndex++;
                //ledIndex2--;
                if (ledIndex >= LED_NUMBER) {
                    ledIndex = 0;

                    for (int idx = 0; idx < 3 ; idx++) {
                        ws2812_SetAllLedsColor(100, 100, 100);
                        osDelay(100);
                        ws2812_SetAllLedsColor(0, 0, 0);
                        osDelay(300);
                    }
                }
                /*
                if (ledIndex2 < 0) {
                    ledIndex2 = LED_NUMBER - 1;
                }
                */

                osDelay(200);
            }
        }
    }
#pragma clang diagnostic pop
  /* USER CODE END ledsUpdate */
}

/* USER CODE BEGIN Header_servo */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo */
void servo(void const * argument)
{
  /* USER CODE BEGIN servo */
    int ascenseurPositionPrec = -1;

    sd21_SetPositionAndSpeed(SERVO_ASC_NB, SPEED_ASC, ascenseurPosition);

    /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        if (ascenseurPosition != ascenseurPositionPrec) {
            ascenseurPositionPrec = ascenseurPosition;

            sd21_SetPosition(SERVO_ASC_NB, ascenseurPosition);
        }

        osDelay(100);
    }
#pragma clang diagnostic pop
  /* USER CODE END servo */
}

/* USER CODE BEGIN Header_mainProcess */
/**
* @brief Function implementing the mainProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mainProcess */
void mainProcess(void const * argument)
{
  /* USER CODE BEGIN mainProcess */

    initialisation();

    /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        if (declenchementRobot()) {
            ascenseurPosition = ASC_HAUT;
            ledsState = LEDS_MATCH;
        } else {
            ascenseurPosition = ASC_BAS;
            ledsState = LEDS_BLANK;
        }

        osDelay(1000);
    }
#pragma clang diagnostic pop
  /* USER CODE END mainProcess */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
