/* filename: Core/Inc/streaming.h */
#ifndef STREAMING_H
#define STREAMING_H

#include "app_context.h"

/**
 * @brief Initializes the streaming module.
 * @param ctx Pointer to the application context.
 */
void Streaming_Init(AppContext_t* ctx);

/**
 * @brief Starts the live streaming of sensor data.
 * @param ctx Pointer to the application context.
 */
void Streaming_Start(AppContext_t* ctx);

/**
 * @brief Stops the live streaming of sensor data.
 * @param ctx Pointer to the application context.
 */
void Streaming_Stop(AppContext_t* ctx);

/**
 * @brief Pumps the streaming module to send out prepared data.
 *        Should be called in the main application loop.
 * @param ctx Pointer to the application context.
 */
void Streaming_Pump(AppContext_t* ctx);

/**
 * @brief Processes a new sample from an ISR context for potential streaming.
 *        This function handles decimation and prepares data for the main loop pump.
 * @param ctx Pointer to the application context.
 * @param s Pointer to the new sample.
 */
void Streaming_ProcessSampleFromISR(AppContext_t* ctx, const Sample_t* s);

/**
 * @brief Reconfigures streaming state after a major change (e.g., ODR change).
 * @param ctx Pointer to the application context.
 */
void Streaming_Reconfigure(AppContext_t* ctx);

/**
 * @brief Updates the internal decimation divider based on current configuration.
 * @param ctx Pointer to the application context.
 */
void Streaming_UpdateDivider(AppContext_t* ctx);

/**
 * @brief Gets the current decimation divider value.
 * @param ctx Pointer to the application context.
 * @return The decimation divider.
 */
uint32_t Streaming_GetDivider(AppContext_t* ctx);

#endif // STREAMING_H