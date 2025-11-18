/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* PATCH NVIC-001: Changed from PRIORITYGROUP_0 to PRIORITYGROUP_4
   * ROOT CAUSE: PRIORITYGROUP_0 allocates 0 bits to preempt priority and 4 bits
   * to sub-priority, which completely disables interrupt preemption. This caused
   * all interrupts to operate at the same preemption level (0), making the RC-004
   * priority settings (TIM3=5, USART2=6) ineffective.
   * 
   * SOLUTION: PRIORITYGROUP_4 allocates all 4 bits to preempt priority, enabling
   * full hierarchical interrupt preemption (16 levels: 0-15). This allows TIM3
   * (sampling timer, prio=5) to preempt USART2/DMA (communication, prio=6),
   * which is critical for deterministic sensor data acquisition.
   * 
   * IMPACT: Fixes complete system failure where RingBuffer sampling stalled due
   * to I2C/DMA operations being blocked by lower-priority UART transmissions.
   * 
   * TRACEABILITY: Solves diagnostics FAIL: NVIC_TIM3, NVIC_USART2, NVIC_HIERARCHY,
   * RB_SAMPLES (scenario 17.17). Enables proper function of ZERO, ARM, DAMP, LIVE.
   */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
