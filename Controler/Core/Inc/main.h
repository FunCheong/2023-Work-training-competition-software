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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
void msgLog(const char *format, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_System_Pin GPIO_PIN_3
#define LED_System_GPIO_Port GPIOE
#define Dir1_Pin GPIO_PIN_5
#define Dir1_GPIO_Port GPIOC
#define Dir3_Pin GPIO_PIN_0
#define Dir3_GPIO_Port GPIOB
#define Dir2_Pin GPIO_PIN_1
#define Dir2_GPIO_Port GPIOB
#define DirStep_Pin GPIO_PIN_2
#define DirStep_GPIO_Port GPIOB
#define Dir4_Pin GPIO_PIN_7
#define Dir4_GPIO_Port GPIOE
#define ClipLimit_Pin GPIO_PIN_9
#define ClipLimit_GPIO_Port GPIOE
#define ST7735_BL_Pin GPIO_PIN_10
#define ST7735_BL_GPIO_Port GPIOE
#define ST7735_CS_Pin GPIO_PIN_11
#define ST7735_CS_GPIO_Port GPIOE
#define ST7735_DC_Pin GPIO_PIN_13
#define ST7735_DC_GPIO_Port GPIOE
#define ST7735_RES_Pin GPIO_PIN_15
#define ST7735_RES_GPIO_Port GPIOE
#define LED_Green_Pin GPIO_PIN_8
#define LED_Green_GPIO_Port GPIOD
#define LED_Blue_Pin GPIO_PIN_9
#define LED_Blue_GPIO_Port GPIOD
#define LED_White_Pin GPIO_PIN_10
#define LED_White_GPIO_Port GPIOD
#define KEY2_Pin GPIO_PIN_11
#define KEY2_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_14
#define KEY1_GPIO_Port GPIOD
#define KEY0_Pin GPIO_PIN_15
#define KEY0_GPIO_Port GPIOD
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define Pi_Ready_Pin GPIO_PIN_0
#define Pi_Ready_GPIO_Port GPIOD
#define Pi_Reset_Pin GPIO_PIN_2
#define Pi_Reset_GPIO_Port GPIOD
#define Pi_Switch_Pin GPIO_PIN_4
#define Pi_Switch_GPIO_Port GPIOD
#define PMW3901_CS_Pin GPIO_PIN_7
#define PMW3901_CS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
#define Key_PollingPeriod               (1)
#define UC_PollingPeriod                (20)
#define UC_TimerResetPeriod             (10000)
#define SupportServoID                  (6)
#define ClipServoID                     (2)
#define StoreServoID                    (5)
#define StepsEachGrid_Y                 (5150)
#define StepsEachGrid_X                 (5453)
#define StepsEach180                    (10800)
#define CommunicationInterval           (3)   // In ms
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
