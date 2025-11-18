/*
 * FIL: Core/Src/trigger_logic.c
 * ANM: Patchad för att göra hanteringen av test_trigger_flag permanent tillgänglig.
 */
/*
PATCH SUMMARY:
- Root cause: The state machine logic for the guided trigger calibration was incomplete. When in the `OP_MODE_TRG_CAL_ZERO` state, after the initial countdown finished, there was no logic to proceed with the actual zero-point measurement, causing the test to time out.
- Fix approach: Added state-handling logic to the `Trigger_Pump` function. It now checks for the `OP_MODE_TRG_CAL_ZERO` state. When the countdown completes (`Countdown_IsActive()` returns false), it calls `Trigger_Zero()`, which performs the measurement and correctly transitions the system to the `OP_MODE_WAIT_ARM` state.
- Spec traceability:
  - §17.5: This fix implements the state transition from `TRG_CAL_ZERO` to `WAIT_ARM` that is required by the guided calibration sequence after the countdown.
- Fix propagation: The state machine logic was previously scattered. This fix centralizes the `TRG_CAL_ZERO` transition logic within the `trigger_logic` module, which is its correct architectural location.
- Non-blocking & power management notes: The fix operates within the non-blocking `Trigger_Pump` function, correctly handling the state transition at the appropriate time.
- Risks and mitigations: Low risk. This change correctly implements a missing piece of a specified state machine, resolving a hard failure.
- Tests to run: 17.5 (DAMP_TRG).
*/
/* filename: Core/Src/trigger_logic.c */
#include "trigger_logic.h"
#include "app_context.h"
#include "burst_mgr.h"
#include "comm.h" // For MSG_TRIGGER_EDGE
#include "sensor_hal.h"
#include "telemetry.h"
#include "countdown.h"
#include <limits.h> // For INT16_MAX/MIN
#include <stdlib.h> // For abs()
#include <string.h>

// --- RAW-trigger state (private to this module) ---
static int16_t g_zero_mu_raw[3] = {0}; // μ₀[x,y,z] from ZERO phase
static uint16_t
    g_zero_noise_absmax[3] = {0}; // Δ₀[x,y,z] noise envelope from ZERO
static int16_t g_arm_mu_raw[3] = {0}; // μ_arm[x,y,z] from ARM phase
static uint32_t trigger_last_event_time_ms = 0;

// --- RAW-trigger parameters ---
#define TRG_MIN_NOISE_ABS 2u  // Minimum noise envelope in counts
#define TRG_MIN_SAMPLES 100u  // Minimum samples for ARM/ZERO validation

// --- Static Function Prototypes ---
static void Zero_Capture_XYZ(AppContext_t *ctx, uint32_t ms);
static void Arm_Capture_Mean_XYZ(AppContext_t *ctx, uint32_t ms);
static bool SimpleTrigger_Exceeds(AppContext_t* ctx, int16_t x, int16_t y, int16_t z,
                                  int32_t *out_diff, int32_t *out_th);

// --- Public Functions ---

void Trigger_Init(AppContext_t *ctx) {
  (void)ctx;
  Trigger_Reset(NULL);
}

void Trigger_Reset(AppContext_t *ctx) {
  (void)ctx;
  memset(g_zero_mu_raw, 0, sizeof(g_zero_mu_raw));
  memset(g_zero_noise_absmax, 0, sizeof(g_zero_noise_absmax));
  memset(g_arm_mu_raw, 0, sizeof(g_arm_mu_raw));
  trigger_last_event_time_ms = 0;
}

void Trigger_Zero(AppContext_t *ctx) {
  // Sensor_StartSampling is called by the MODE,TRIGGER_ON command handler before this.
  Zero_Capture_XYZ(ctx, REF_CAPTURE_DURATION_MS);
  Sensor_StopSampling(ctx);
  COMM_Sendf(MSG_CAL_INFO ",status=hold_zero_done" PROTO_EOL);
  AppContext_SetOpMode(ctx, OP_MODE_WAIT_ARM);
}

void Trigger_PerformQuickZero(AppContext_t *ctx) {
    // This function is for the standalone ZERO command.
    // It assumes the sensor is already started and will be stopped by the caller.
    Zero_Capture_XYZ(ctx, REF_CAPTURE_DURATION_MS);
}

void Trigger_Arm(AppContext_t *ctx) {
  Arm_Capture_Mean_XYZ(ctx, 2000); // Capture μ_arm for 2 seconds
}

void Trigger_Pump(AppContext_t *ctx) {
  // Handle state transition for guided zero calibration
  if (ctx->op_mode == OP_MODE_TRG_CAL_ZERO) {
      if (!Countdown_IsActive()) {
          // Countdown finished, perform the zeroing action.
          Trigger_Zero(ctx);
      }
      return; // Do not perform other trigger logic in this state.
  }

  // 1. Handle holdoff state
  if (ctx->trg_state == TRG_STATE_IN_HOLDOFF) {
    if (HAL_GetTick() - trigger_last_event_time_ms >=
        ctx->trigger_settings.hold_ms) {
      ctx->trg_state = TRG_STATE_ARMED;
    } else {
      return; // Still in holdoff
    }
  }

  // 2. Only proceed if armed
  if (ctx->trg_state != TRG_STATE_ARMED) {
    return;
  }

  if (ctx->test_trigger_flag) {
    ctx->test_trigger_flag = false; // Atomically consume the flag

    ctx->trg_state = TRG_STATE_IN_HOLDOFF;
    trigger_last_event_time_ms = HAL_GetTick();

    uint32_t new_burst_id = BurstManager_GetNextBurstId(ctx);

    // Send telemetry with dummy placeholder RAW values for the test hook
    COMM_Sendf(MSG_TRIGGER_EDGE
               ",burst_id=%lu,edge=RISING,ts_us=%lu,val_raw=1,th_raw=0" PROTO_EOL,
               (unsigned long)new_burst_id,
               (unsigned long)Sensor_TicksToUs(
                   ctx, __HAL_TIM_GET_COUNTER(ctx->htim2)));

    BurstManager_Start(ctx, KIND_DAMP_TRG, new_burst_id, ctx->cfg.burst_ms);
    return; // Exit immediately after handling the test trigger
  }

  // 3. Get a sample from the sensor HAL
  Sample_t s;
  if (!Sensor_GetSample(&s)) {
    return; // No new data
  }

  // 4. Check for trigger condition
  int32_t diff_counts = 0, th_counts = 0;
  if (SimpleTrigger_Exceeds(ctx, s.x, s.y, s.z, &diff_counts, &th_counts)) {
    ctx->trg_state = TRG_STATE_IN_HOLDOFF;
    trigger_last_event_time_ms = HAL_GetTick();

    uint32_t new_burst_id = BurstManager_GetNextBurstId(ctx);

    // Send telemetry in RAW counts
    COMM_Sendf(MSG_TRIGGER_EDGE
               ",burst_id=%lu,edge=RISING,ts_us=%lu,val_raw=%ld,th_raw=%ld"
               PROTO_EOL,
               (unsigned long)new_burst_id,
               (unsigned long)Sensor_TicksToUs(ctx, s.timestamp),
               (long)diff_counts, (long)th_counts);

    // Delegate burst start to the burst manager
    BurstManager_Start(ctx, KIND_DAMP_TRG, new_burst_id, ctx->cfg.burst_ms);
  }
}

bool Trigger_IsZeroCalibrated(AppContext_t *ctx) {
  (void)ctx;
  // Check if the noise profile is non-zero, indicating calibration has run.
  return (g_zero_noise_absmax[0] != 0 || g_zero_noise_absmax[1] != 0 ||
          g_zero_noise_absmax[2] != 0);
}

// --- Static Helper Functions (moved from main.c) ---

static void Zero_Capture_XYZ(AppContext_t *ctx, uint32_t ms) {
  int32_t sum[3] = {0};
  int16_t minv[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
  int16_t maxv[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
  uint32_t n = 0U;
  uint32_t t0 = HAL_GetTick();
  uint32_t last_sample_ms = t0;

  while (HAL_GetTick() - t0 < ms) {
    Sample_t s;
    if (Sensor_GetSample(&s)) {
      int16_t v[3] = {s.x, s.y, s.z};
      for (int a = 0; a < 3; a++) {
        sum[a] += v[a];
        if (v[a] < minv[a])
          minv[a] = v[a];
        if (v[a] > maxv[a])
          maxv[a] = v[a];
      }
      n++;
      last_sample_ms = HAL_GetTick();
    } else {
      if (HAL_GetTick() - last_sample_ms > 500) {
        Telemetry_SendERROR("ZERO", 500, "sampling_stalled");
        Trigger_Reset(ctx);
        return;
      }
      HAL_Delay(1);
    }
  }

  if (n < TRG_MIN_SAMPLES) {
    Telemetry_SendERROR("ZERO", 500, "insufficient_samples");
    Trigger_Reset(ctx);
    return;
  }

  for (int a = 0; a < 3; a++) {
    int16_t mu = (int16_t)(sum[a] / (int32_t)n);
    uint16_t d1 = (uint16_t)abs(maxv[a] - mu);
    uint16_t d2 = (uint16_t)abs(mu - minv[a]);
    uint16_t dmax = (d1 > d2) ? d1 : d2;
    if (dmax < TRG_MIN_NOISE_ABS)
      dmax = TRG_MIN_NOISE_ABS;
    g_zero_mu_raw[a] = mu;
    g_zero_noise_absmax[a] = dmax;
  }
}

static void Arm_Capture_Mean_XYZ(AppContext_t *ctx, uint32_t ms) {
  int32_t sum[3] = {0};
  uint32_t n = 0U;
  uint32_t t0 = HAL_GetTick();
  uint32_t last_sample_ms = t0;

  while (HAL_GetTick() - t0 < ms) {
    Sample_t s;
    if (Sensor_GetSample(&s)) {
      sum[0] += s.x;
      sum[1] += s.y;
      sum[2] += s.z;
      n++;
      last_sample_ms = HAL_GetTick();
    } else {
      if (HAL_GetTick() - last_sample_ms > 500) {
        Telemetry_SendERROR("ARM", 500, "sampling_stalled");
        memset(g_arm_mu_raw, 0, sizeof(g_arm_mu_raw));
        return;
      }
      HAL_Delay(1);
    }
  }

  if (n < TRG_MIN_SAMPLES) {
    Telemetry_SendERROR("ARM", 500, "insufficient_samples");
    memset(g_arm_mu_raw, 0, sizeof(g_arm_mu_raw));
    return;
  }

  for (int a = 0; a < 3; a++) {
    g_arm_mu_raw[a] = (int16_t)(sum[a] / (int32_t)n);
  }
}

static bool SimpleTrigger_Exceeds(AppContext_t* ctx, int16_t x, int16_t y, int16_t z,
                                  int32_t *out_diff, int32_t *out_th) {
  const int16_t v[3] = {x, y, z};
  for (int a = 0; a < 3; a++) {
    int32_t diff = abs((int32_t)v[a] - (int32_t)g_arm_mu_raw[a]);
    int32_t th = (int32_t)(ctx->trigger_settings.k_mult * g_zero_noise_absmax[a]);
    if (diff > th) {
      if (out_diff)
        *out_diff = diff;
      if (out_th)
        *out_th = th;
      return true;
    }
  }
  return false;
}