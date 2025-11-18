/*
 * Configures GPIO pins for the application.
 * This includes standard I/O (LED, Button) and the EXTI line for the ADXL345 interrupt.
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
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
  __HAL_RCC_SYSCFG_CLK_ENABLE();


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /* B1 (PC13) as plain input, no EXTI */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /* Configure ADXL345_INT1_Pin (PA7) as EXTI on falling edge for active-low interrupt. */
  GPIO_InitStruct.Pin  = ADXL345_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(ADXL345_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init: disable B1's EXTI15_10, enable EXTI9_5 for PA7 */
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  
  /*
   * Set EXTI interrupt priority.
   * Priority 4 is numerically higher (lower logical priority) than I2C/DMA (prio 3).
   * This ensures that an in-progress I2C/DMA data transfer (triggered by a previous interrupt)
   * completes before a new EXTI is handled. This prevents race conditions in the
   * sensor data handling state machine.
   */
  HAL_NVIC_SetPriority(ADXL345_INT1_EXTI_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(ADXL345_INT1_EXTI_IRQn);


}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */