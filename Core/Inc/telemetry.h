/* filename: Core/Inc/telemetry.h */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "app_context.h"

/**
 * @brief Initializes the telemetry module.
 * @param ctx Pointer to the application context.
 */
void Telemetry_Init(AppContext_t* ctx);

/**
 * @brief Pumps the telemetry module for periodic tasks, like sending heartbeats.
 *        Should be called regularly in the main application loop.
 * @param ctx Pointer to the application context.
 */
void Telemetry_Pump(AppContext_t* ctx);

// --- Standard Message Senders ---

/**
 * @brief Sends a STATUS message with the current operational and trigger state.
 * @param ctx Pointer to the application context.
 */
void Telemetry_SendStatus(AppContext_t* ctx);

/**
 * @brief Sends a CFG message with the current runtime configuration.
 * @param ctx Pointer to the application context.
 */
void Telemetry_SendCfg(AppContext_t* ctx);

/**
 * @brief Sends a TRG_SETTINGS message with the current trigger configuration.
 * @param ctx Pointer to the application context.
 */
void Telemetry_SendTrgSettings(AppContext_t* ctx);

/**
 * @brief Sends a generic ACK message for a given command subject.
 * @param subject The command being acknowledged (e.g., CMD_STOP).
 */
void Telemetry_SendACK(const char* subject);

/**
 * @brief Sends a NACK message with a subject, reason, and error code.
 * @param subject The subject of the NACK.
 * @param reason A string describing the reason for the NACK.
 * @param code A numerical error code.
 */
void Telemetry_SendNACK(const char* subject, const char* reason, uint32_t code);

/**
 * @brief Sends an ERROR message with source, code, and a descriptive message.
 * @param src The module or source of the error (e.g., "FSM").
 * @param code A numerical error code.
 * @param msg A descriptive error message string.
 */
void Telemetry_SendERROR(const char* src, uint32_t code, const char* msg);

/**
 * @brief Sends a specialized ACK for the STREAM_START command, including rate info.
 * @param ctx Pointer to the application context.
 */
void Telemetry_SendStreamStartACK(AppContext_t* ctx);

/**
 * @brief Sends a CAL_INFO message to the host.
 * @param ctx Pointer to the application context.
 */
void Telemetry_SendCalInfo(AppContext_t* ctx);

/**
 * @brief Sends diagnostic information to the host.
 * @param ctx Pointer to the application context.
 */
void Telemetry_SendDiag(AppContext_t* ctx);

/**
 * @brief Sends a snapshot of the most recent sensor data as a PREVIEW.
 * @param ctx Pointer to the application context.
 */
void Telemetry_SendPreview(AppContext_t* ctx);

// --- UI and Utility ---

/**
 * @brief Updates the state of the onboard LED based on the current application mode.
 * @param ctx Pointer to the application context.
 */
void Telemetry_UpdateLED(AppContext_t* ctx);

/**
 * @brief Blocks until the UART TX buffer is idle, with a timeout.
 */
void Telemetry_Flush(void);

#endif // TELEMETRY_H