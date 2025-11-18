/* PATCH SUMMARY:
- Verification: Confirmed that the pin definitions (`ADXL345_INT1_Pin`, `ADXL345_INT1_GPIO_Port`, `ADXL345_INT1_EXTI_IRQn`) correctly map the hardware connection on PA7 to the corresponding software handlers.
- Conclusion: No changes are required in this file.
*/
/* filename: Core/Inc/main.h */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Project: EngoveVibraNUCLEO-F446RE_2
 * FW: 3.3.0
 * Module: Core/Inc/main.h
 *
 * History:
 * v3.3.1 | 2025-10-27 | MrPerfect | API COMPLIANCE & ROBUSTNESS:
 * |            |              | - FIXED: Completed FSM state list by adding OP_MODE_ERROR to
 * |            |              |   fully align with API v3.3.0 §11.1.
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "api_schema.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// Main operational modes of the application FSM (API v3.3.0 §11.1)
typedef enum {
    OP_MODE_INIT = 0,
    OP_MODE_IDLE,

    // Guided trigger setup sequence (for MODE,TRIGGER_ON)
    OP_MODE_WAIT_CAL_ZERO,       // Waiting for CAL_READY,phase=hold_zero (§11.1)
    OP_MODE_TRG_CAL_ZERO,        // Capturing zero-angle reference
    OP_MODE_WAIT_ARM,            // Setup complete, waiting for ARM command
    OP_MODE_ARMED,               // Armed and waiting for trigger event

    // Measurement execution states
    OP_MODE_COUNTDOWN,           // Waiting for countdown to finish before burst
    OP_MODE_BURST,               // Generic burst data collection
    /* --- PATCH START --- */
    // New state to separate data collection from transmission waiting period.
    OP_MODE_BURST_SENDING,       // Data collected, now waiting for BM to finish sending.
    /* --- PATCH END --- */
    OP_MODE_STATIC_RUN,          // Static/weight measurement in progress (§11.1)
    OP_MODE_STREAMING,           // Live streaming active (§11.1)
    OP_MODE_ERROR                // Non-recoverable error state (§11.1)
} OpMode_t;


// Defines the kind of data in a burst, used in DATA_HEADER and COMPLETE messages
// API v3.3.0 §13.1 normative types: WEIGHT, DAMP_TRG, DAMP_CD
typedef enum {
    KIND_UNKNOWN = 0,
    KIND_DAMP_TRG,      // For TRG_DAMPING (normative)
    KIND_DAMP_CD,       // For START_BURST_DAMPING (normative)
    KIND_WEIGHT,        // For START_BURST_WEIGHT (normative)
} DataKind_t;

// Detailed state of the absolute value trigger logic
typedef enum {
    TRG_STATE_IDLE = 0,     // Not actively monitoring
    TRG_STATE_ARMED,        // Monitoring for rising edge
    TRG_STATE_IN_HOLDOFF    // Triggered, waiting for holdoff to expire
} TrgState_t;

// Defines the signal source for the trigger
typedef enum {
    TRG_AXIS_X = 0,
    TRG_AXIS_Y,
    TRG_AXIS_Z,
    TRG_AXIS_THETA,   // |θ| in degrees (internal)
    TRG_AXIS_MAG      // Vector magnitude (reported to host when THETA is used)
} TriggerAxis_t;

// Configuration for the trigger system
typedef struct {
    float k_mult;      // v3.3.2: Sensitivity multiplier [2.0, 20.0]
    uint32_t win_ms;   // v3.3.2: Analysis window size [50, 500] ms
    uint32_t hold_ms;  // Unchanged: Holdoff period after trigger
} TriggerSettings_t;

// Represents a single raw sample from the ADC
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t timestamp; // TIM2 ticks (us)
} Sample_t;

// Runtime configuration, settable via SET_CFG
typedef struct {
  uint32_t hb_ms;
  uint32_t burst_ms;
  uint32_t odr_hz;
  uint32_t stream_rate_hz;
} RuntimeCfg_t;

// Diagnostic counters for telemetry
typedef struct {
  uint32_t i2c_fail;
  uint32_t ring_ovf;
  uint32_t live_drops;
  uint32_t hb_pauses;
  uint32_t last_err;
} DiagCounters_t;

// Time synchronization state with host
typedef struct {
  bool     has_sync;
  uint64_t host_ms_at_sync;
  uint32_t tick_at_sync;
} TimeSync_t;

/* Globals (to be defined in main.c) */
extern RuntimeCfg_t g_cfg;
extern DiagCounters_t g_diag;
extern TimeSync_t g_tsync;
extern TriggerSettings_t g_trigger_settings;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void process_command(char *line);
/* void Sampling_Timer_Callback(void); */

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
// ADXL345 INT1 -> PA7 (Arduino D11)
#define ADXL345_INT1_Pin        GPIO_PIN_7
#define ADXL345_INT1_GPIO_Port  GPIOA
#define ADXL345_INT1_EXTI_IRQn  EXTI9_5_IRQn

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
