/*
PATCH SUMMARY:
- Root cause: The initialization of the test-only variable 'test_trigger_flag' was not conditionally compiled, matching its declaration. This would cause a compilation failure in production builds where the flag is not defined in the AppContext_t struct.
- Fix approach: Enclosed the 'test_trigger_flag = false;' initialization within #ifdef ENABLE_TEST_HOOKS, mirroring the change in app_context.h.
- Spec traceability: Follow-on change for API spec §18 (Invariant 12).
- Fix propagation: This is the propagation of the header file change.
- Risks and mitigations: Low risk. Ensures build correctness for both production and test configurations.
*/
/* filename: Core/Src/app_context.c */
#include "app_context.h"
#include "telemetry.h" // Required to call Telemetry_SendStatus
#include <string.h>

void AppContext_Init(AppContext_t* ctx, TIM_HandleTypeDef* htim2_handle, TIM_HandleTypeDef* htim3_handle, I2C_HandleTypeDef* hi2c1_handle) {
    memset(ctx, 0, sizeof(AppContext_t));

    ctx->op_mode = OP_MODE_INIT;
    ctx->trg_state = TRG_STATE_IDLE;

    // Set default configuration values from the schema/main
    ctx->cfg.hb_ms = DEFAULT_HB_MS;
    ctx->cfg.burst_ms = DEFAULT_BURST_MS;
    ctx->cfg.odr_hz = DEFAULT_ODR_HZ;
    ctx->cfg.stream_rate_hz = DEFAULT_STREAM_HZ;

    ctx->trigger_settings.k_mult = 5.0f;
    ctx->trigger_settings.win_ms = 100;
    ctx->trigger_settings.hold_ms = 1500;

#ifdef ENABLE_TEST_HOOKS
    ctx->test_trigger_flag = false;
#endif

    // Store pointers to HAL handles
    ctx->htim2 = htim2_handle;
    ctx->htim3 = htim3_handle;
    ctx->hi2c1 = hi2c1_handle;
}

void AppContext_SetOpMode(AppContext_t* ctx, OpMode_t new_mode) {
    if (ctx->op_mode == new_mode) return;

    ctx->op_mode = new_mode;
    ctx->state_timer_start_ms = HAL_GetTick();
    Telemetry_SendStatus(ctx); // Use telemetry module to send status
}
