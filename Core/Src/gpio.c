/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WDT_PULSE_Pin|HB_Pin|MS1_Pin|MS2_Pin
                          |MS3_Pin|MS4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAN_INDICATO_Pin|LED1_Pin|LED2_Pin|Rear_Solenoid_Pin
                          |Front_Solenoid_Pin|ASSI_YELLOW_Pin|ASSI_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MS5_GPIO_Port, MS5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WDT_PULSE_Pin HB_Pin MS1_Pin MS2_Pin
                           MS3_Pin MS4_Pin */
  GPIO_InitStruct.Pin = WDT_PULSE_Pin|HB_Pin|MS1_Pin|MS2_Pin
                          |MS3_Pin|MS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SDC_FEEDBACK_Pin */
  GPIO_InitStruct.Pin = SDC_FEEDBACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDC_FEEDBACK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_INDICATO_Pin LED1_Pin LED2_Pin Rear_Solenoid_Pin
                           Front_Solenoid_Pin ASSI_YELLOW_Pin ASSI_BLUE_Pin */
  GPIO_InitStruct.Pin = CAN_INDICATO_Pin|LED1_Pin|LED2_Pin|Rear_Solenoid_Pin
                          |Front_Solenoid_Pin|ASSI_YELLOW_Pin|ASSI_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MS5_Pin */
  GPIO_InitStruct.Pin = MS5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MS5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ASMS_Pin */
  GPIO_InitStruct.Pin = ASMS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ASMS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MS_BTN_Pin */
  GPIO_InitStruct.Pin = MS_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MS_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IGN_BTN_Pin */
  GPIO_InitStruct.Pin = IGN_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IGN_BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
