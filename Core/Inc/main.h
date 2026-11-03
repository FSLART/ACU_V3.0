/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#define WDT_PULSE_Pin GPIO_PIN_0
#define WDT_PULSE_GPIO_Port GPIOC
#define SDC_FEEDBACK_Pin GPIO_PIN_1
#define SDC_FEEDBACK_GPIO_Port GPIOC
#define HB_Pin GPIO_PIN_5
#define HB_GPIO_Port GPIOC
#define CAN_INDICATO_Pin GPIO_PIN_0
#define CAN_INDICATO_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define Rear_Solenoid_Pin GPIO_PIN_12
#define Rear_Solenoid_GPIO_Port GPIOB
#define Front_Solenoid_Pin GPIO_PIN_13
#define Front_Solenoid_GPIO_Port GPIOB
#define ASSI_YELLOW_Pin GPIO_PIN_14
#define ASSI_YELLOW_GPIO_Port GPIOB
#define ASSI_BLUE_Pin GPIO_PIN_15
#define ASSI_BLUE_GPIO_Port GPIOB
#define MS1_Pin GPIO_PIN_6
#define MS1_GPIO_Port GPIOC
#define MS2_Pin GPIO_PIN_7
#define MS2_GPIO_Port GPIOC
#define MS3_Pin GPIO_PIN_8
#define MS3_GPIO_Port GPIOC
#define MS4_Pin GPIO_PIN_9
#define MS4_GPIO_Port GPIOC
#define MS5_Pin GPIO_PIN_8
#define MS5_GPIO_Port GPIOA
#define ASMS_Pin GPIO_PIN_2
#define ASMS_GPIO_Port GPIOD
#define MS_BTN_Pin GPIO_PIN_3
#define MS_BTN_GPIO_Port GPIOB
#define MS_BTN_EXTI_IRQn EXTI3_IRQn
#define IGN_BTN_Pin GPIO_PIN_4
#define IGN_BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Application types live in acu.h */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
