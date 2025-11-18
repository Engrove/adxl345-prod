/*
 * Manages the ADXL345 sensor via I2C.
 * Implements a non-blocking, DMA-driven state machine for data acquisition,
 * triggered by FIFO watermark interrupts.
 */
/* filename: Core/Src/sensor_hal.c */
#include "main.h"
#include "sensor_hal.h"
#include "app_context.h"
#include "streaming.h"
#include "telemetry.h"
#include <string.h>

// --- Private Defines ---
#define ACCEL_SENSOR_ADDR (0x53 << 1) // 8-bit HAL address (0xA6/0xA7), SDO=GND
#define ACCEL_REG_DEVID 0x00
#define ACCEL_REG_POWER_CTL 0x2D
#define ACCEL_REG_DATA_FORMAT 0x31
#define ACCEL_REG_BW_RATE 0x2C
#define ACCEL_REG_DATAX0 0x32
#define ACCEL_REG_INT_ENABLE  0x2E
#define ACCEL_REG_INT_MAP     0x2F
#define ACCEL_REG_INT_SOURCE  0x30
#define ACCEL_REG_FIFO_CTL    0x38
#define ACCEL_REG_FIFO_STATUS 0x39

// Per datasheet, FULL_RES mode has a fixed sensitivity of ~3.9mg/LSB regardless of G-range.
// 1 LSB = 0.00390625 g * 9.80665 m/s^2/g = 0.038245935 m/s^2
#define ADXL_LSB_TO_MS2 0.038245935f

#define OFFSET_CAL_SAMPLES ((DEFAULT_ODR_HZ / 4 > 100) ? (DEFAULT_ODR_HZ / 4) : 100)
#define OFFSET_CAL_MAX_DURATION_MS 5000

#define SENSOR_WRITE_VERIFY_RETRIES 3
#define SENSOR_WRITE_VERIFY_DELAY_MS 1

#define DMA_RX_BUF_SAMPLES 32 // Must be >= FIFO watermark level
#define DMA_RX_BUF_SIZE (DMA_RX_BUF_SAMPLES * 6)

// --- Static variables ---
// Static context pointer for use in ISR callbacks
static AppContext_t* s_ctx = NULL;

// Sensor data buffers and state
static Sample_t sample_ring_buffer[SAMPLE_RING_BUFFER_SIZE];
volatile uint16_t sample_ring_head = 0;
volatile uint16_t sample_ring_tail = 0;

// State flags
volatile bool g_sampling_active = false;
volatile I2CState_t g_i2c_state = I2C_STATE_IDLE;

// Non-blocking I2C buffers
static uint8_t g_fifo_status_buf[1];
static uint8_t g_dma_rx_buf[DMA_RX_BUF_SIZE] __attribute__ ((aligned (4))); // Align for DMA
static volatile uint8_t g_samples_to_read = 0;

// Calibration and preview data
static float offX_ms2 = 0.0f, offY_ms2 = 0.0f, offZ_ms2 = 0.0f;
static PreviewSnap_t g_preview;

// Diagnostic counters
volatile uint32_t g_debug_exti_callback_count = 0;
volatile uint32_t g_debug_exti_rejected_sampling = 0;
volatile uint32_t g_debug_exti_rejected_context = 0;
volatile uint32_t g_debug_exti_rejected_state = 0;
volatile uint32_t g_debug_dma_start_ok = 0;
volatile uint32_t g_debug_dma_start_fail = 0;
volatile uint32_t g_debug_dma_complete_count = 0;
volatile uint32_t g_debug_samples_processed = 0;


// --- Private Function Prototypes ---
static HAL_StatusTypeDef Sensor_WriteVerifyReg(AppContext_t* ctx, uint8_t reg, uint8_t value_to_write);
static void I2C1_WaitReady(uint32_t to_ms);
static void I2C1_AbortIfBusy(uint32_t to_ms);
static HAL_StatusTypeDef Sensor_ReadRawSampleBlocking(AppContext_t* ctx, int16_t* x, int16_t* y, int16_t* z);

// --- Public Functions ---

HAL_StatusTypeDef Sensor_Init(AppContext_t* ctx) {
    s_ctx = ctx; // Store context for ISRs

    // 1. Check device ID
    uint8_t devid = 0;
    if (HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_DEVID, I2C_MEMADD_SIZE_8BIT, &devid, 1, 100) != HAL_OK || devid != 0xE5) {
        return HAL_ERROR;
    }

    // --- Normative Initialization Sequence ---

    // 2. Place sensor in STANDBY mode to allow configuration.
    if (Sensor_WriteVerifyReg(ctx, ACCEL_REG_POWER_CTL, 0x00) != HAL_OK) {
        return HAL_ERROR;
    }

    // 3. Set Output Data Rate.
    if (Sensor_SetODR(ctx, ctx->cfg.odr_hz) != HAL_OK) {
        return HAL_ERROR;
    }

    // 4. Set data format: INT_INVERT=1 (Active-LOW), FULL_RES=1, Range=±2g.
    if (Sensor_WriteVerifyReg(ctx, ACCEL_REG_DATA_FORMAT, 0x28) != HAL_OK) {
        return HAL_ERROR;
    }

    // 5. Set FIFO to Stream mode with 16-sample watermark.
    if (Sensor_WriteVerifyReg(ctx, ACCEL_REG_FIFO_CTL, 0x90) != HAL_OK) {
        return HAL_ERROR;
    }

    // 6. Map all interrupts to INT1 pin.
    if (Sensor_WriteVerifyReg(ctx, ACCEL_REG_INT_MAP, 0x00) != HAL_OK) {
        return HAL_ERROR;
    }

    // 7. Enable the WATERMARK interrupt source.
    if (Sensor_WriteVerifyReg(ctx, ACCEL_REG_INT_ENABLE, 0x02) != HAL_OK) {
        return HAL_ERROR;
    }

    // 8. Activate Measurement Mode.
    if (Sensor_WriteVerifyReg(ctx, ACCEL_REG_POWER_CTL, (1 << 3)) != HAL_OK) { // 0x08 = Measure
        return HAL_ERROR;
    }

    // 9. Clear any latched interrupts from the initial power-on/measurement sequence.
    // This is done by reading the INT_SOURCE register, preventing a missed initial interrupt.
    uint8_t int_source;
    if (HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_INT_SOURCE, I2C_MEMADD_SIZE_8BIT, &int_source, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }
    
    HAL_Delay(20); // Wait for sensor to stabilize.
    return HAL_OK;
}

void Sensor_PerformOffsetCalibration(AppContext_t* ctx) {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int n = 0;
    uint32_t start_time = HAL_GetTick();
    uint8_t i2c_buf[6];
    while (n < OFFSET_CAL_SAMPLES && (HAL_GetTick() - start_time < OFFSET_CAL_MAX_DURATION_MS)) {
        if (HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_DATAX0, I2C_MEMADD_SIZE_8BIT, i2c_buf, 6, 10) == HAL_OK) {
            sum_x += (float)((int16_t)((i2c_buf[1] << 8) | i2c_buf[0]));
            sum_y += (float)((int16_t)((i2c_buf[3] << 8) | i2c_buf[2]));
            sum_z += (float)((int16_t)((i2c_buf[5] << 8) | i2c_buf[4]));
            n++;
        }
        HAL_Delay(1);
    }
    if (n > 0) {
        offX_ms2 = (sum_x / n) * ADXL_LSB_TO_MS2;
        offY_ms2 = (sum_y / n) * ADXL_LSB_TO_MS2;
        offZ_ms2 = (sum_z / n) * ADXL_LSB_TO_MS2;
    }
}

void Sensor_StartSampling(AppContext_t* ctx) {
    __disable_irq();
    sample_ring_head = 0;
    sample_ring_tail = 0;
    g_i2c_state = I2C_STATE_IDLE;
    g_sampling_active = true;
    __enable_irq();
}

void Sensor_StopSampling(AppContext_t* ctx) {
    __disable_irq();
    g_sampling_active = false;
    __enable_irq();
}

bool Sensor_IsSampling(AppContext_t* ctx) {
    (void)ctx;
    bool is_active;
    __disable_irq();
    is_active = g_sampling_active;
    __enable_irq();
    return is_active;
}

HAL_StatusTypeDef Sensor_SetODR(AppContext_t* ctx, uint32_t odr_hz) {
    uint8_t rate_code;
    if (odr_hz >= 3200) rate_code = 0x0F;
    else if (odr_hz >= 1600) rate_code = 0x0E;
    else if (odr_hz >= 800) rate_code = 0x0D;
    else if (odr_hz >= 400) rate_code = 0x0C;
    else if (odr_hz >= 200) rate_code = 0x0B;
    else rate_code = 0x0A; // Default 100Hz

    I2C1_AbortIfBusy(10);
    I2C1_WaitReady(10);
    HAL_StatusTypeDef status = Sensor_WriteVerifyReg(ctx, ACCEL_REG_BW_RATE, rate_code);
    if (status != HAL_OK) {
        Telemetry_SendERROR("I2C", 10, "set_odr_busy");
    }
    return status;
}

uint32_t Sensor_SnapODR(uint32_t req) {
    if (req >= 3200) return 3200;
    if (req >= 1600) return 1600;
    if (req >= 800) return 800;
    if (req >= 400) return 400;
    if (req >= 200) return 200;
    return 100;
}

void Sensor_ReconfigureTimer(AppContext_t* ctx, uint32_t odr_hz) {
    if (odr_hz == 0) odr_hz = 1;
    
    HAL_TIM_Base_Stop_IT(ctx->htim3);
    ctx->htim3->Init.Prescaler = 899;
    uint32_t period = 100000U / odr_hz;
    if (period < 1U) period = 1U;
    ctx->htim3->Init.Period = (uint32_t)(period - 1U);
    if (HAL_TIM_Base_Init(ctx->htim3) != HAL_OK) {
        Error_Handler();
    }
}

bool Sensor_GetSample(Sample_t* sample) {
    bool has_data;
    __disable_irq();
    if (sample_ring_head != sample_ring_tail) {
        *sample = sample_ring_buffer[sample_ring_tail];
        sample_ring_tail = (sample_ring_tail + 1U) % SAMPLE_RING_BUFFER_SIZE;
        has_data = true;
    } else {
        has_data = false;
    }
    __enable_irq();
    return has_data;
}

uint32_t Sensor_TicksToUs(AppContext_t* ctx, uint32_t ticks) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t timclk = pclk1;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        timclk *= 2U;
    }
    uint32_t tick_hz = timclk / (ctx->htim2->Init.Prescaler + 1U);
    if (tick_hz == 0U) return 0U;
    uint64_t us = ((uint64_t)ticks * 1000000ULL) / tick_hz;
    return (us > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)us;
}

const PreviewSnap_t* Sensor_GetPreviewSnapshot(AppContext_t* ctx) {
    bool was_running = Sensor_IsSampling(ctx);
    if (was_running) {
        Sensor_StopSampling(ctx);
    }

    __disable_irq();
    uint16_t head = sample_ring_head;
    uint16_t tail = sample_ring_tail;
    __enable_irq();

    uint16_t n = 0;
    while (tail != head && n < SAMPLE_RING_BUFFER_SIZE) {
        g_preview.buf[n++] = sample_ring_buffer[tail];
        tail = (tail + 1) % SAMPLE_RING_BUFFER_SIZE;
    }
    g_preview.count = n;

    if (was_running) {
        Sensor_StartSampling(ctx);
    }
    return &g_preview;
}

void Sensor_ConvertToMps2(AppContext_t* ctx, const Sample_t* raw, float* ax, float* ay, float* az) {
    (void)ctx;
    *ax = (float)raw->x * ADXL_LSB_TO_MS2 - offX_ms2;
    *ay = (float)raw->y * ADXL_LSB_TO_MS2 - offY_ms2;
    *az = (float)raw->z * ADXL_LSB_TO_MS2 - offZ_ms2;
}

HAL_StatusTypeDef Sensor_PerformSelfTest(AppContext_t* ctx, uint8_t avg_count, uint8_t settle_count, uint32_t force_odr_hz, AdxlSelfTestResult_t* results) {
    HAL_StatusTypeDef status = HAL_OK;
    bool was_sampling = Sensor_IsSampling(ctx);

    uint8_t old_power_ctl = 0, old_data_format = 0, old_bw_rate = 0, old_fifo_ctl = 0, old_int_enable = 0;
    bool registers_saved = false;

    memset(results, 0, sizeof(AdxlSelfTestResult_t));
    if (avg_count == 0) avg_count = 16;

    if (was_sampling) {
        Sensor_StopSampling(ctx);
    }
    
    // Save registers that will be modified
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_POWER_CTL, I2C_MEMADD_SIZE_8BIT, &old_power_ctl, 1, 100);
    if (status != HAL_OK) goto cleanup;
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_DATA_FORMAT, I2C_MEMADD_SIZE_8BIT, &old_data_format, 1, 100);
    if (status != HAL_OK) goto cleanup;
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_BW_RATE, I2C_MEMADD_SIZE_8BIT, &old_bw_rate, 1, 100);
    if (status != HAL_OK) goto cleanup;
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_FIFO_CTL, I2C_MEMADD_SIZE_8BIT, &old_fifo_ctl, 1, 100);
    if (status != HAL_OK) goto cleanup;
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &old_int_enable, 1, 100);
    if (status != HAL_OK) goto cleanup;
    registers_saved = true;
    
    // Put sensor in standby and bypass FIFO for single-sample polling
    if (Sensor_WriteVerifyReg(ctx, ACCEL_REG_POWER_CTL, 0x00) != HAL_OK) goto cleanup_error;
    if (Sensor_WriteVerifyReg(ctx, ACCEL_REG_FIFO_CTL, 0x00) != HAL_OK) goto cleanup_error;

    uint32_t test_odr = (force_odr_hz > 0) ? force_odr_hz : 400;
    status = Sensor_SetODR(ctx, test_odr);
    if (status != HAL_OK) goto cleanup;

    uint8_t test_data_format = (1 << 3) | 0x03; // FULL_RES, +/-16g
    status = Sensor_WriteVerifyReg(ctx, ACCEL_REG_DATA_FORMAT, test_data_format);
    if (status != HAL_OK) goto cleanup;

    status = Sensor_WriteVerifyReg(ctx, ACCEL_REG_POWER_CTL, (1 << 3)); // Measurement mode
    if (status != HAL_OK) goto cleanup;
    
    HAL_Delay(20); 

    int32_t sum_x_off = 0, sum_y_off = 0, sum_z_off = 0;
    for (uint8_t i = 0; i < avg_count; i++) {
        int16_t x, y, z;
        status = Sensor_ReadRawSampleBlocking(ctx, &x, &y, &z);
        if (status != HAL_OK) goto cleanup;
        sum_x_off += x; sum_y_off += y; sum_z_off += z;
    }
    results->x_off = sum_x_off / avg_count;
    results->y_off = sum_y_off / avg_count;
    results->z_off = sum_z_off / avg_count;
    
    uint8_t st_on_data_format = test_data_format | (1 << 7);
    status = Sensor_WriteVerifyReg(ctx, ACCEL_REG_DATA_FORMAT, st_on_data_format);
    if (status != HAL_OK) goto cleanup;

    for (uint8_t i = 0; i < settle_count; i++) {
        int16_t x, y, z;
        if ((status = Sensor_ReadRawSampleBlocking(ctx, &x, &y, &z)) != HAL_OK) goto cleanup;
    }

    int32_t sum_x_on = 0, sum_y_on = 0, sum_z_on = 0;
    for (uint8_t i = 0; i < avg_count; i++) {
        int16_t x, y, z;
        status = Sensor_ReadRawSampleBlocking(ctx, &x, &y, &z);
        if (status != HAL_OK) goto cleanup;
        sum_x_on += x; sum_y_on += y; sum_z_on += z;
    }
    results->x_on = sum_x_on / avg_count;
    results->y_on = sum_y_on / avg_count;
    results->z_on = sum_z_on / avg_count;

    results->x_st = results->x_on - results->x_off;
    results->y_st = results->y_on - results->y_off;
    results->z_st = results->z_on - results->z_off;

    // Datasheet Table 12 (LSB), full-resolution mode.
    // Valid for all VS per datasheet.
    bool x_ok = (results->x_st >= 50  && results->x_st <= 540);
    bool y_ok = (results->y_st >= -540 && results->y_st <= -50);
    bool z_ok = (results->z_st >= 75  && results->z_st <= 875);
    results->health_pass = (x_ok && y_ok && z_ok);


    goto cleanup; // Normal exit

cleanup_error:
    status = HAL_ERROR;

cleanup:
    if (registers_saved) {
        Sensor_WriteVerifyReg(ctx, ACCEL_REG_DATA_FORMAT, old_data_format);
        Sensor_WriteVerifyReg(ctx, ACCEL_REG_BW_RATE, old_bw_rate);
        Sensor_WriteVerifyReg(ctx, ACCEL_REG_FIFO_CTL, old_fifo_ctl);
        Sensor_WriteVerifyReg(ctx, ACCEL_REG_INT_ENABLE, old_int_enable);
        Sensor_WriteVerifyReg(ctx, ACCEL_REG_POWER_CTL, old_power_ctl);
    }
    
    if (was_sampling) {
        Sensor_StartSampling(ctx);
    }
    return status;
}


// --- HAL Callback Implementations ---

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // This timer is not used for sensor data acquisition in FIFO watermark interrupt mode.
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C1) {
        return;
    }

    switch (g_i2c_state) {
    case I2C_STATE_WAIT_FIFO_DATA: {
        g_debug_dma_complete_count++;
        
        // DMA transfer of FIFO data is complete. Process the received samples.
        uint8_t samples_to_process = DMA_RX_BUF_SAMPLES;
        
        for (uint8_t i = 0; i < samples_to_process; i++) {
            uint16_t next_head = (sample_ring_head + 1) % SAMPLE_RING_BUFFER_SIZE;
            if (next_head == sample_ring_tail) {
                if(s_ctx) s_ctx->diag.ring_ovf++;
                break; // Stop processing if buffer is full
            } else {
                Sample_t* s = &sample_ring_buffer[sample_ring_head];
                uint8_t* p_sample_data = &g_dma_rx_buf[i * 6];
                s->x = (int16_t)((p_sample_data[1] << 8) | p_sample_data[0]);
                s->y = (int16_t)((p_sample_data[3] << 8) | p_sample_data[2]);
                s->z = (int16_t)((p_sample_data[5] << 8) | p_sample_data[4]);
                s->timestamp = __HAL_TIM_GET_COUNTER(s_ctx->htim2);

                Streaming_ProcessSampleFromISR(s_ctx, s);
                
                g_debug_samples_processed++;
                sample_ring_head = next_head;
            }
        }

        // After processing, check FIFO_STATUS to see if more samples need to be drained.
        g_i2c_state = I2C_STATE_DRAIN_STATUS;
        if (HAL_I2C_Mem_Read_IT(s_ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_FIFO_STATUS, I2C_MEMADD_SIZE_8BIT, g_fifo_status_buf, 1) != HAL_OK) {
            g_i2c_state = I2C_STATE_IDLE;
            if (s_ctx) s_ctx->diag.i2c_fail++;
        }
        break;
    }

    case I2C_STATE_DRAIN_STATUS: {
        uint8_t total_samples_in_fifo = g_fifo_status_buf[0] & 0x3F;

        if (total_samples_in_fifo > 0) {
            // Samples are present. Start a new DMA read to drain another batch.
            g_samples_to_read = total_samples_in_fifo;
            if (g_samples_to_read > DMA_RX_BUF_SAMPLES) {
                g_samples_to_read = DMA_RX_BUF_SAMPLES;
            }
            
            g_i2c_state = I2C_STATE_WAIT_FIFO_DATA;
            HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(s_ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_DATAX0, I2C_MEMADD_SIZE_8BIT, g_dma_rx_buf, g_samples_to_read * 6);
            if (status != HAL_OK) {
                g_debug_dma_start_fail++;
                g_i2c_state = I2C_STATE_IDLE;
                if (s_ctx) s_ctx->diag.i2c_fail++;
            } else {
                g_debug_dma_start_ok++;
            }
        } else {
            // FIFO is empty. Read INT_SOURCE to clear the sensor's internal interrupt latch.
            g_i2c_state = I2C_STATE_CLEAR_INT_SOURCE;
            if (HAL_I2C_Mem_Read_IT(s_ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_INT_SOURCE, I2C_MEMADD_SIZE_8BIT, g_fifo_status_buf, 1) != HAL_OK) {
                g_i2c_state = I2C_STATE_IDLE;
                if (s_ctx) s_ctx->diag.i2c_fail++;
            }
        }
        break;
    }

    case I2C_STATE_CLEAR_INT_SOURCE:
        // The read of INT_SOURCE is complete. The state machine can return to idle for the next EXTI.
        g_i2c_state = I2C_STATE_IDLE;
        break;

    default:
        g_i2c_state = I2C_STATE_IDLE;
        break;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        if (s_ctx) {
            s_ctx->diag.i2c_fail++;
        }
        g_i2c_state = I2C_STATE_IDLE; // Reset state machine on error
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ADXL345_INT1_Pin) {
        g_debug_exti_callback_count++;
        
        if (!g_sampling_active) {
            g_debug_exti_rejected_sampling++;
            return;
        }
        
        if (!s_ctx) {
            g_debug_exti_rejected_context++;
            return;
        }
        
        if (g_i2c_state != I2C_STATE_IDLE) {
            g_debug_exti_rejected_state++;
            return;
        }
        
        if (g_sampling_active && s_ctx && g_i2c_state == I2C_STATE_IDLE) {
             // Start the non-blocking FIFO drain sequence.
             g_samples_to_read = DMA_RX_BUF_SAMPLES;
             g_i2c_state = I2C_STATE_WAIT_FIFO_DATA;
             
             HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(s_ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_DATAX0, I2C_MEMADD_SIZE_8BIT, g_dma_rx_buf, g_samples_to_read * 6);
             
             if (status == HAL_OK) {
                 g_debug_dma_start_ok++;
             } else {
                 g_debug_dma_start_fail++;
                 g_i2c_state = I2C_STATE_IDLE;
                 s_ctx->diag.i2c_fail++;
             }
        }
    }
}

// --- Private Helper Functions ---

static HAL_StatusTypeDef Sensor_WriteVerifyReg(AppContext_t* ctx, uint8_t reg, uint8_t value_to_write) {
    uint8_t read_value = 0;
    HAL_StatusTypeDef status;

    for (int i = 0; i < SENSOR_WRITE_VERIFY_RETRIES; i++) {
        // Attempt to write the value
        status = HAL_I2C_Mem_Write(ctx->hi2c1, ACCEL_SENSOR_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value_to_write, 1, 100);
        if (status != HAL_OK) {
            HAL_Delay(SENSOR_WRITE_VERIFY_DELAY_MS);
            continue; // Retry write
        }

        // Attempt to read it back
        status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &read_value, 1, 100);
        if (status != HAL_OK) {
            HAL_Delay(SENSOR_WRITE_VERIFY_DELAY_MS);
            continue; // Retry read
        }
        
        // Compare
        if (read_value == value_to_write) {
            return HAL_OK; // Success
        }

        // Mismatch, delay before next retry
        HAL_Delay(SENSOR_WRITE_VERIFY_DELAY_MS);
    }
    
    // If we exit the loop, it means all retries have failed
    return HAL_ERROR;
}

static void I2C1_WaitReady(uint32_t to_ms) {
    if (!s_ctx) return;
    uint32_t t0 = HAL_GetTick();
    while (HAL_I2C_GetState(s_ctx->hi2c1) != HAL_I2C_STATE_READY) {
        if ((HAL_GetTick() - t0) >= to_ms) {
            break;
        }
    }
}

static void I2C1_AbortIfBusy(uint32_t to_ms) {
    if (!s_ctx) return;
    uint32_t t0 = HAL_GetTick();
    while (HAL_I2C_GetState(s_ctx->hi2c1) != HAL_I2C_STATE_READY) {
        #if defined(HAL_I2C_Master_Abort_IT)
        (void)HAL_I2C_Master_Abort_IT(s_ctx->hi2c1, ACCEL_SENSOR_ADDR);
        #endif
        if ((HAL_GetTick() - t0) >= to_ms) {
            break;
        }
    }
}

static HAL_StatusTypeDef Sensor_ReadRawSampleBlocking(AppContext_t* ctx, int16_t* x, int16_t* y, int16_t* z) {
    uint32_t start_tick = HAL_GetTick();
    const uint32_t timeout_ms = 100; // Timeout for waiting for a sample
    uint8_t int_source;
    HAL_StatusTypeDef status;

    // Wait for DATA_READY bit by polling the register. Used for self-test only.
    while (HAL_GetTick() - start_tick < timeout_ms) {
        status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_INT_SOURCE, I2C_MEMADD_SIZE_8BIT, &int_source, 1, 10);
        if (status == HAL_OK && (int_source & (1 << 7))) {
            // Data is ready, now read it. The previous read cleared the INT_SOURCE register.
            uint8_t data_buf[6];
            status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_DATAX0, I2C_MEMADD_SIZE_8BIT, data_buf, 6, 50);
            if (status == HAL_OK) {
                *x = (int16_t)((data_buf[1] << 8) | data_buf[0]);
                *y = (int16_t)((data_buf[3] << 8) | data_buf[2]);
                *z = (int16_t)((data_buf[5] << 8) | data_buf[4]);
            }
            return status;
        }
        HAL_Delay(1); // Don't busy-spin too hard
    }

    return HAL_TIMEOUT; // Timed out waiting for data ready
}