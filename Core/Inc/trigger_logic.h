/*
PATCH SUMMARY:
- Root cause: The helper function `Trigger_PerformQuickZero` was defined as static within `trigger_logic.c`, making it inaccessible to the command handler which needed to call it to implement the `ZERO` command.
- Fix approach: Added the function prototype for `Trigger_PerformQuickZero` to this header file, making it part of the module's public interface.
- Spec traceability:
  - §17.3: This change is a necessary prerequisite to implementing the `ZERO` command sequence.
- Fix propagation: N/A. This was a single function visibility issue.
- Non-blocking & power management notes: N/A. Header file change.
- Risks and mitigations: None. This change simply exposes an existing function.
- Tests to run: 17.3 (ZERO-kalibrering).
*/
/* filename: Core/Inc/trigger_logic.h */
#ifndef TRIGGER_LOGIC_H
#define TRIGGER_LOGIC_H

#include "app_context.h"

/**
 * @brief Initializes the trigger logic module.
 * @param ctx Pointer to the application context.
 */
void Trigger_Init(AppContext_t* ctx);

/**
 * @brief Resets the trigger logic to its idle state.
 * @param ctx Pointer to the application context.
 */
void Trigger_Reset(AppContext_t* ctx);

/**
 * @brief Performs the ZERO phase of trigger calibration.
 *        Captures sensor data to establish a baseline noise profile.
 * @param ctx Pointer to the application context.
 */
void Trigger_Zero(AppContext_t* ctx);

/**
 * @brief Performs a quick ZERO calibration for the standalone ZERO command.
 * @param ctx Pointer to the application context.
 */
void Trigger_PerformQuickZero(AppContext_t* ctx);

/**
 * @brief Performs the ARM phase of trigger calibration.
 *        Captures sensor data to establish the armed state's mean values.
 * @param ctx Pointer to the application context.
 */
void Trigger_Arm(AppContext_t* ctx);

/**
 * @brief Checks for trigger conditions if the system is armed.
 *        This function should be called periodically in the main application loop.
 * @param ctx Pointer to the application context.
 */
void Trigger_Pump(AppContext_t* ctx);

/**
 * @brief Checks if the ZERO calibration phase has been successfully completed.
 * @param ctx Pointer to the application context.
 * @return True if calibrated, false otherwise.
 */
bool Trigger_IsZeroCalibrated(AppContext_t* ctx);

#endif // TRIGGER_LOGIC_H
