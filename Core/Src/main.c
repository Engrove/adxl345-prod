/* PATCH SUMMARY:
- Fix: Inkluderade den nya diagnostikfilen `dev_diagnostics.h` för att möjliggöra lågnivåfelsökning.
*/
/* filename: Core/Src/main.c */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// Include all modules
#include "app_context.h"
#include "comm.h"
#include "command_handler.h"
#include "sensor_hal.h"
#include "trigger_logic.h"
#include "burst_mgr.h"
#include "streaming.h"
#include "telemetry.h"
#include "countdown.h"
#include "transport_blocks.h" // For BM_Init dependency
#include "dev_diagnostics.h"  // NY: Inkludera diagnostikmodulen

// The single global application context
static AppContext_t g_app_context;

// Prototypes for auto-generated functions
void SystemClock_Config(void);

// --- main.h compatibility ---
// Define globals that were previously in main.c but are now in app_context.
// These are for linking with auto-generated headers that might extern them.
// The new modules should not use these.
RuntimeCfg_t g_cfg;
DiagCounters_t g_diag;
TimeSync_t g_tsync;
TriggerSettings_t g_trigger_settings;

// --- Retarget stdout to UART ---
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
  if (g_app_context.is_dumping) {
    return ch; // Suppress printf during data dumps
  }
  char c = (char)ch;
  Telemetry_Write(&c, 1);
  return ch;
}

int main(void) {
    // HAL & Clock Initialization
    HAL_Init();
    SystemClock_Config();

    // Peripheral Initialization
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();

    // Start microsecond timer
    HAL_TIM_Base_Start(&htim2);

    // Initialize all software modules, passing the context and HAL handles
    AppContext_Init(&g_app_context, &htim2, &htim3, &hi2c1);
    
    // Copy context defaults to legacy globals for compatibility
    g_cfg = g_app_context.cfg;
    g_trigger_settings = g_app_context.trigger_settings;

    COMM_Init();
    BM_Init(PROTO_WINDOW_DEFAULT, PROTO_BLOCK_LINES_DEFAULT, PROTO_MAX_RETRIES);
    Telemetry_Init(&g_app_context); // Must be initialized before Sensor_Init to report errors

    if (Sensor_Init(&g_app_context) != HAL_OK) {
        Telemetry_SendERROR("SENSOR_INIT", 999, "I2C init failed");
        HAL_Delay(100); // Give UART a chance to send before halting
        Error_Handler();
    }
    CmdHandler_Init(&g_app_context);
    Trigger_Init(&g_app_context);
    BurstManager_Init(&g_app_context);
    Streaming_Init(&g_app_context);
    Countdown_Init();
    
    // Start listening for commands
    COMM_StartRx();
    HAL_Delay(250); // Sensor settle time

    // Perform initial calibration
    Sensor_PerformOffsetCalibration(&g_app_context);
    AppContext_SetOpMode(&g_app_context, OP_MODE_IDLE);

    // Main application loop
    while (1) {
        // 1. Handle incoming communication
        CmdHandler_ProcessInput(&g_app_context);

        // 2. Data acquisition is now fully interrupt driven. No polling needed.

        // 3. Pump the state machines of each active module
        Telemetry_Pump(&g_app_context);
        BurstManager_Pump(&g_app_context);
        Trigger_Pump(&g_app_context);
        Streaming_Pump(&g_app_context);
        Countdown_Tick();

        // 4. Handle global flags (like STOP)
        if (g_app_context.stop_flag) {
            CmdHandler_HandleStop(&g_app_context); // Delegate stop logic
            g_app_context.stop_flag = false;
        }

        // 5. Handle is_dumping flag reset
        if (g_app_context.is_dumping) {
            const bool is_busy = (g_app_context.op_mode == OP_MODE_BURST || g_app_context.op_mode == OP_MODE_BURST_SENDING);
            if (!is_busy && COMM_TxIsIdle() && !BM_IsActive()) {
                g_app_context.is_dumping = false;
            }
        }

        // 6. Update UI
        Telemetry_UpdateLED(&g_app_context);

        // 7. Yield if idle
        if (g_app_context.op_mode == OP_MODE_IDLE && !g_app_context.is_dumping) {
            HAL_Delay(5);
        }
    }
}

// SystemClock_Config, Error_Handler, etc. remain here...

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  Telemetry_SendERROR("HAL", 0, "fatal_error");
  HAL_Delay(100);
  while (1)
  {
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      HAL_Delay(50);
  }
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
  printf("ASSERT FAILED: file %s on line %lu" PROTO_EOL, file, line);
}
#endif /* USE_FULL_ASSERT */
