/* FILENAME: Core/Inc/comm.h */
/*
 * Project: EngoveVibraNUCLEO-F446RE_2
 * FW: 3.3.2
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h> /* For size_t */
#include "stm32f4xx_hal.h"

/* --- Configuration --- */
#define COMM_TX_RING_SIZE 4096U   /* Ökat för att undvika drop/trunkering vid burst-sekvenser */
#define COMM_TX_DMA_SIZE  512U    /* Större DMA-chunk => färre avbrott och jämnare tömning */
#define RX_RING_BUFFER_SIZE 2048U /* Flyttad hit från comm.c för global synlighet */


/* GCC printf-format checking where available */
#if defined(__GNUC__)
#define COMM_PRINTF_ATTR(fmt_idx, va_idx) __attribute__((format(printf, fmt_idx, va_idx)))
#else
#define COMM_PRINTF_ATTR(fmt_idx, va_idx)
#endif


/**
 * @brief Initializes the communication module.
 */
void COMM_Init(void);

/**
 * @brief Starts the non-blocking UART reception.
 * Must be called once to begin listening for commands.
 */
void COMM_StartRx(void);

/**
 * @brief Processes incoming data from the ring buffer.
 * This function should be called periodically in the main loop.
 */
void COMM_Process(void);

/**
 * @brief Processes incoming data with a time and line budget.
 * Prevents blocking the main loop for too long.
 */
void COMM_Process_Budgeted(void);

/**
 * @brief Sends a null-terminated string over UART atomically.
 * The entire string is either enqueued or dropped if there is not enough space.
 * @param s The string to send.
 */
void COMM_Send(const char *s);

/**
 * @brief Sends a formatted string over UART (printf-style) atomically.
 * The entire formatted string is either enqueued or dropped.
 * @param fmt The format string.
 * @param ... Variable arguments for the format string.
 * @return The number of characters enqueued, or a negative value on error.
 */
int COMM_Sendf(const char *fmt, ...) COMM_PRINTF_ATTR(1, 2);

/**
 * @brief Callback function to be invoked by the UART ISR (via HAL) when new data is received.
 * This function is for internal use by the HAL driver redirection.
 * @param huart Pointer to the UART handle.
 * @param Size The number of bytes received.
 */
void COMM_OnRxEvent(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief Callback for when a DMA TX transfer is complete.
 * @param huart Pointer to the UART handle.
 */
void COMM_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief Strong implementation of telemetry write function.
 * Atomically enqueues a block of data for transmission. The entire block is
 * either enqueued or dropped if there is not enough space.
 * @param data Data to send
 * @param len Length of data
 * @return Number of bytes successfully enqueued. 0 if dropped.
 */
size_t Telemetry_Write(const char* data, size_t len);

/* TX Buffer introspection */
uint16_t COMM_TxFree(void);

/* TX is truly idle only when DMA is not active, ring is empty, and no staged DMA length. */
void    COMM_SendLine(const char* s);
int     COMM_SendfLine(const char* fmt, ...);
uint8_t COMM_TxIsIdle(void);

/* Blocking variants for correctness-critical transmissions */
size_t  Telemetry_WriteBlocking(const char* data, size_t len);
int     COMM_SendfBlocking(const char* fmt, ...);

/* Diagnostic getters */
uint32_t COMM_TxDropCount(void);
uint32_t COMM_RxOverflowCount(void);
uint16_t COMM_TxRingUsage(void);
uint16_t COMM_RxRingUsage(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMM_H_ */
