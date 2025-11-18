/* PATCH SUMMARY:
 * Fix: Exporterade ringbuffertens pekare (`sample_ring_head`, `sample_ring_tail`) för att möjliggöra lågnivådiagnostik (DIAG_HW_TEST) av datainsamlingsflödet.
 * Fix: Exporterade I2CState_t och g_i2c_state för att lösa byggfel i dev_diagnostics.c.
 * Fix: Exporterade g_sampling_active och alla g_debug_-räknare för att lösa länkningsfel.
 * Key functions / ISRs touched: Added extern declarations.
 */
/* filename: Core/Inc/sensor_hal.h */
#ifndef SENSOR_HAL_H
#define SENSOR_HAL_H

#include "app_context.h"

#define SAMPLE_RING_BUFFER_SIZE 512

// A snapshot of the ring buffer for the PREVIEW command.
typedef struct {
  uint16_t count;
  Sample_t buf[SAMPLE_RING_BUFFER_SIZE];
} PreviewSnap_t;

// A struct to hold the results of the ADXL345 self-test.
typedef struct {
    int16_t x_off;
    int16_t y_off;
    int16_t z_off;
    int16_t x_on;
    int16_t y_on;
    int16_t z_on;
    int16_t x_st;
    int16_t y_st;
    int16_t z_st;
    bool health_pass;
} AdxlSelfTestResult_t;

// --- FIX: Flyttad från sensor_hal.c för att göra den publik för diagnostik ---
// --- I2C State Machine ---
typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_WAIT_FIFO_DATA, // State after EXTI, waiting for DMA read of FIFO data
    I2C_STATE_DRAIN_STATUS,   // State after data read, waiting for FIFO_STATUS read
    I2C_STATE_CLEAR_INT_SOURCE
} I2CState_t;


// --- Exporterade variabler för diagnostik ---
extern volatile uint16_t sample_ring_head;
extern volatile uint16_t sample_ring_tail;
extern volatile I2CState_t g_i2c_state; // FIX: Exponerad för diagnostik
extern volatile bool g_sampling_active; // FIX: Exponerad för diagnostik

// --- FIX: Exporterade debug-räknare för diagnostik ---
extern volatile uint32_t g_debug_exti_callback_count;
extern volatile uint32_t g_debug_exti_rejected_sampling;
extern volatile uint32_t g_debug_exti_rejected_context;
extern volatile uint32_t g_debug_exti_rejected_state;
extern volatile uint32_t g_debug_dma_start_ok;
extern volatile uint32_t g_debug_dma_start_fail;
extern volatile uint32_t g_debug_dma_complete_count;
extern volatile uint32_t g_debug_samples_processed;


/**
 * @brief Initializes the ADXL345 sensor.
 * @param ctx Pointer to the application context.
 * @return HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef Sensor_Init(AppContext_t* ctx);

/**
 * @brief Performs initial offset calibration for the accelerometer.
 * @param ctx Pointer to the application context.
 */
void Sensor_PerformOffsetCalibration(AppContext_t* ctx);

/**
 * @brief Enables sampling.
 * @param ctx Pointer to the application context.
 */
void Sensor_StartSampling(AppContext_t* ctx);

/**
 * @brief Disables sampling.
 * @param ctx Pointer to the application context.
 */
void Sensor_StopSampling(AppContext_t* ctx);

/**
 * @brief Checks if sampling is currently active.
 * @param ctx Pointer to the application context.
 * @return True if sampling is active, false otherwise.
 */
bool Sensor_IsSampling(AppContext_t* ctx);

/**
 * @brief Sets the Output Data Rate (ODR) on the accelerometer hardware.
 * @param ctx Pointer to the application context.
 * @param odr_hz The desired ODR in Hz.
 * @return HAL status of the I2C write operation.
 */
HAL_StatusTypeDef Sensor_SetODR(AppContext_t* ctx, uint32_t odr_hz);

/**
 * @brief Reconfigures the sampling timer (TIM3) to match a new ODR.
 * @param ctx Pointer to the application context.
 * @param odr_hz The new ODR in Hz.
 */
void Sensor_ReconfigureTimer(AppContext_t* ctx, uint32_t odr_hz);

/**
 * @brief Finds the closest supported ODR value for a given request.
 * @param req_odr The requested ODR.
 * @return The actual supported ODR that will be used.
 */
uint32_t Sensor_SnapODR(uint32_t req_odr);

/**
 * @brief Safely retrieves one sample from the internal ring buffer.
 * @param[out] sample Pointer to a Sample_t struct to be filled.
 * @return True if a sample was retrieved, false if the buffer was empty.
 */
bool Sensor_GetSample(Sample_t* sample);

/**
 * @brief Converts TIM2 timer ticks into microseconds.
 * @param ctx Pointer to the application context.
 * @param ticks The number of ticks from TIM2.
 * @return The equivalent duration in microseconds.
 */
uint32_t Sensor_TicksToUs(AppContext_t* ctx, uint32_t ticks);

/**
 * @brief Takes a snapshot of the current sample ring buffer for preview purposes.
 * @param ctx Pointer to the application context.
 * @return A const pointer to the internal preview snapshot buffer.
 */
const PreviewSnap_t* Sensor_GetPreviewSnapshot(AppContext_t* ctx);

/**
 * @brief Converts a raw sample to calibrated m/s^2 values.
 * @param ctx Pointer to the application context.
 * @param raw Pointer to the raw Sample_t data.
 * @param[out] ax Pointer to store the calibrated X-axis value in m/s^2.
 * @param[out] ay Pointer to store the calibrated Y-axis value in m/s^2.
 * @param[out] az Pointer to store the calibrated Z-axis value in m/s^2.
 */
void Sensor_ConvertToMps2(AppContext_t* ctx, const Sample_t* raw, float* ax, float* ay, float* az);

/**
 * @brief Performs the ADXL345 self-test procedure.
 * @note This is a blocking function that takes direct control of the sensor.
 * It should only be called when the system is idle.
 * @param ctx Pointer to the application context.
 * @param avg_count The number of samples to average for OFF and ON states.
 * @param settle_count The number of samples to discard after enabling self-test.
 * @param force_odr_hz If non-zero, forces a specific ODR for the test.
 * @param[out] results Pointer to a struct to store the test results.
 * @return HAL_StatusTypeDef HAL_OK on success, HAL_ERROR or HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef Sensor_PerformSelfTest(AppContext_t* ctx, uint8_t avg_count, uint8_t settle_count, uint32_t force_odr_hz, AdxlSelfTestResult_t* results);

#endif // SENSOR_HAL_H
