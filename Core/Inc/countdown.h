/* filename: Core/Inc/countdown.h */
#ifndef COUNTDOWN_H
#define COUNTDOWN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initializes the countdown module.
 */
void Countdown_Init(void);

/**
 * @brief Starts the countdown timer for a specified number of seconds.
 * @param seconds The number of seconds to count down from. Clamped internally.
 *        Sends COUNTDOWN_ID id=<seconds> immediately.
 */
void Countdown_Start(uint8_t seconds);

/**
 * @brief Stops the countdown immediately and sends COUNTDOWN_ID id=0.
 *        Ensures the stop message is only sent once.
 */
void Countdown_Stop(void);

/**
 * @brief Processes the countdown timer.
 *        This function should be called periodically in the main loop.
 *        It handles sending COUNTDOWN_ID messages every second.
 */
void Countdown_Tick(void);

/**
 * @brief Checks if the countdown is currently active.
 * @return 1 if active, 0 otherwise.
 */
bool Countdown_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* COUNTDOWN_H */
/* filename: Core/Inc/countdown.h */
