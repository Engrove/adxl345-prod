/*
PATCH SUMMARY:
- Root cause: Pre-analytic report identified three issues: incorrect firmware version, an unconditionally compiled test-only variable, and a redundant header file. These violate API spec and code hygiene standards.
- Fix approach:
  1. Updated FW_VERSION macro from "3.4.0" to "3.3.7" to align with API spec §10.
  2. Enclosed the 'test_trigger_flag' declaration within #ifdef ENABLE_TEST_HOOKS to ensure it is only present in test builds, per API spec §18 (Invariant 12).
  3. The redundant header Core/Inc/cmd_handler.h is slated for deletion from the project.
- Spec traceability: §10 (Handshake), §18 (Invariant 12).
- Fix propagation: The conditional compilation of 'test_trigger_flag' was propagated to its initialization in app_context.c.
- Risks and mitigations: Low risk. Changes are spec-compliant corrections and code hygiene improvements. Test builds must define ENABLE_TEST_HOOKS to avoid compilation errors related to the test command.
*/
/* filename: Core/Inc/app_context.h */
#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include "main.h"

// --- Application-wide Constants (moved from main.c) ---
#define FW_VERSION "3.3.7"
#define DEFAULT_ODR_HZ 800
#define DEFAULT_BURST_MS 5000
#define DEFAULT_HB_MS 1000
#define DEFAULT_STREAM_HZ 100
#define REF_CAPTURE_DURATION_MS 2000

// This struct consolidates all global and shared state for the entire application.
// A pointer to an instance of this struct will be passed to all modules.
typedef struct AppContext {
    // Core state machine
    volatile OpMode_t op_mode;
    volatile TrgState_t trg_state;
    uint32_t state_timer_start_ms;

    // Configuration
    RuntimeCfg_t cfg;
    TriggerSettings_t trigger_settings;

    // Real-time data and flags
    TimeSync_t tsync;
    DiagCounters_t diag;
    volatile bool stop_flag;
    volatile bool is_dumping;
    volatile bool burst_abort_pending;

#ifdef ENABLE_TEST_HOOKS
    volatile bool test_trigger_flag;
#endif

    // Pointers to HAL handles to avoid global extern declarations
    TIM_HandleTypeDef* htim2; // For microsecond timestamps
    TIM_HandleTypeDef* htim3; // For sampling
    I2C_HandleTypeDef* hi2c1; // For sensor

} AppContext_t;

// Initializes the context with default values and HAL handles.
void AppContext_Init(AppContext_t* ctx, TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, I2C_HandleTypeDef* hi2c1);

// Centralized function to change the operational mode and send a STATUS update.
void AppContext_SetOpMode(AppContext_t* ctx, OpMode_t new_mode);

#endif // APP_CONTEXT_H
