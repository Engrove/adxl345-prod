/* PATCH SUMMARY:
 * Problem: I2C communication in interrupt or DMA mode fails because the necessary DMA controller and I2C event/error interrupts were not configured. This resulted in a NULL DMA handle (hi2c1->hdmarx) and prevented non-blocking I2C operations required for data sampling.
 * High-level change: In HAL_I2C_MspInit, added the complete initialization for the I2C1_RX DMA stream (DMA1 Stream 0, Channel 1). Linked this DMA handle to the I2C1 handle. Enabled the NVIC interrupts for I2C1 events, I2C1 errors, and the I2C1_RX DMA stream. Updated HAL_I2C_MspDeInit to tear down this configuration.
 * Key functions / ISRs touched: HAL_I2C_MspInit, HAL_I2C_MspDeInit.
 * Relation to Analyst plan: This directly implements the analyst's task to 'identify why hi2c1->hdmarx is NULL and correct the DMA configuration for I2C1 receive operations.' It provides the missing low-level configuration for DMA-based I2C.
 * Behavioural impact: Non-blocking I2C functions (HAL_I2C_Mem_Read_DMA, HAL_I2C_Mem_Read_IT) will now function correctly, allowing the sensor driver to read FIFO data without blocking the main loop. This is critical for high-speed data acquisition.
 * Notes on deviations: None.
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
DMA_HandleTypeDef hdma_i2c1_rx;
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000; // Increased to 400kHz for better performance margin
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Stream0;
    hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1 interrupt Init */
    /* PATCH ISR-001: Set I2C/DMA priority HIGHER than EXTI to prevent deadlock.
     * CRITICAL: I2C event and DMA completion interrupts must preempt sensor EXTI
     * (prio 4) to ensure state machine completes processing and returns to IDLE
     * before next EXTI trigger. Without this, re-entrant EXTI is rejected by
     * state guard, INT1 stays HIGH, no new rising edge, system deadlocks.
     * 
     * Priority hierarchy:
     *   3: I2C/DMA (data transfer - highest real-time requirement)
     *   4: EXTI (sensor watermark trigger)
     *   5: TIM3 (backup timer)
     *   6: USART2/DMA (communication)
     */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

    /* DMA interrupt init */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

    /* DMA interrupt deinit */
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */