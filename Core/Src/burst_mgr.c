/*
PATCH SUMMARY:
- Root cause: The burst sampling logic in `OP_MODE_BURST` lacked stall detection. If sensor data stopped arriving, the logic would wait for the full burst duration and then report a burst with zero samples, causing test timeouts (e.g., waiting for a `SUMMARY` message that was never sent).
- Fix approach: Implemented a stall detection mechanism in `BurstManager_Pump` for the `OP_MODE_BURST` state. It now tracks the time of the last received sample. If no new samples arrive for over 500ms (and at least one sample has already been received), the burst is considered stalled, an error is reported, and the operation is aborted cleanly.
- Spec traceability:
  - This is a robustness improvement to correctly handle burst collection failures, which manifest as timeouts in tests §17.4 and §17.6.
- Fix propagation: This fix normalizes the behavior of data capture routines. The stall detection pattern, already present in `Zero_Capture_XYZ` and `Arm_Capture_Mean_XYZ` in `trigger_logic.c`, has now been correctly adapted for the non-blocking `BurstManager_Pump` function.
- Non-blocking & power management notes: The fix is implemented in a non-blocking way, consistent with the pump-based architecture.
- Risks and mitigations: Minimal risk. This change makes the burst capture more robust and prevents system hangs on sensor failure, which is a net improvement.
- Tests to run: 17.4 (STATIC ROBUST), 17.6 (DAMP_CD).
*/
/* filename: Core/Src/burst_mgr.c */
#include "burst_mgr.h"
#include "app_context.h"
#include "sensor_hal.h"
#include "telemetry.h"
#include "transport_blocks.h"
#include "comm.h"
#include "countdown.h"
#include "api_parse.h" // <-- ADDED MISSING INCLUDE for api_parse_u32

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

// =================================================================================
// SECTION 1: LOW-LEVEL TRANSPORT LOGIC (from original burst_mgr.c)
// =================================================================================

typedef struct {
    uint8_t  active;
    uint8_t  waiting_ack_complete;
    BM_Type  type;
    uint32_t burst_id;
    uint8_t  done_pending;
    uint8_t  aborted;
    uint32_t abort_code;
    uint16_t samples;
    uint32_t odr_hz;
} bm_ctx_t;

static volatile bm_ctx_t g_bm = {0};

static const char* _type_str(BM_Type t) {
    switch (t) {
        case BM_TYPE_WEIGHT:   return "WEIGHT";
        case BM_TYPE_DAMP_TRG: return "DAMP_TRG";
        case BM_TYPE_DAMP_CD:  return "DAMP_CD";
        default: return "UNKNOWN";
    }
}

void BM_Init(uint16_t window, uint16_t blk_lines, uint8_t max_retries) {
    TB_Init(window, blk_lines, max_retries);
    memset((void*)&g_bm, 0, sizeof(g_bm));
}

void BM_Begin(BM_Type type, uint32_t burst_id, uint32_t ts0_us, uint16_t samples, uint32_t odr_hz) {
    g_bm.type = type;
    g_bm.burst_id = burst_id;
    g_bm.active = 1;
    g_bm.waiting_ack_complete = 0;
    g_bm.samples = samples;
    g_bm.odr_hz = odr_hz;

    COMM_Sendf("DATA_HEADER,type=%s,burst_id=%lu,ts0_us=%lu,samples=%u,mode=CSV" PROTO_EOL,
               _type_str(type), (unsigned long)burst_id, (unsigned long)ts0_us, (unsigned)samples);
    TB_BeginBurst(burst_id);
}

int BM_Enqueue(const TB_BlockGen* blk) {
    return TB_EnqueueBlock(blk);
}

void BM_Pump(void) {
    if (!g_bm.active) return;
    TB_Pump();

    if (g_bm.done_pending && !g_bm.waiting_ack_complete && TB_IsIdle()) {
        if (g_bm.aborted) {
            COMM_Sendf("COMPLETE,burst_id=%lu,reason=aborted,code=%lu" PROTO_EOL,
                       (unsigned long)g_bm.burst_id, (unsigned long)g_bm.abort_code);
        } else {
            uint32_t time_ms = 0u;
            if (g_bm.odr_hz > 0u) {
                uint64_t num = (uint64_t)g_bm.samples * 1000u + (uint64_t)(g_bm.odr_hz / 2u);
                time_ms = (uint32_t)(num / (uint64_t)g_bm.odr_hz);
            }
            COMM_Sendf("COMPLETE,burst_id=%lu,samples=%u,dropped=%u,time_ms=%lu" PROTO_EOL,
                       (unsigned long)g_bm.burst_id, (unsigned)g_bm.samples, 0u, (unsigned long)time_ms);
        }
        g_bm.waiting_ack_complete = 1;
        g_bm.done_pending = 0;
    }
}

void BM_EndOk(void) {
    if (!g_bm.active) return;
    g_bm.aborted = 0;
    g_bm.done_pending = 1;
}

void BM_EndAborted(uint32_t code) {
    if (!g_bm.active) return;
    g_bm.aborted = 1;
    g_bm.abort_code = code;
    g_bm.done_pending = 1;
}

static int _starts_with(const char* s, const char* pfx) {
    while (*pfx) {
        if (*s++ != *pfx++) return 0;
    }
    return 1;
}

int BM_HandleHostLine(const char* line) {
    if (!line) return 0;
    if (TB_HandleHostLine(line)) return 1;

    if (_starts_with(line, "ACK_COMPLETE")) {
        const char* p = strstr(line, "burst_id=");
        if (!p) {
            if (g_bm.active && g_bm.waiting_ack_complete) {
                g_bm.waiting_ack_complete = 0;
                g_bm.active = 0;
                TB_EndBurst();
                return 1;
            }
        } else {
            uint32_t bid = 0;
            if (api_parse_u32(p + 9, &bid)) {
                if (g_bm.active && g_bm.burst_id == bid) {
                    g_bm.waiting_ack_complete = 0;
                    g_bm.active = 0;
                    TB_EndBurst();
                    return 1;
                }
            }
        }
    }
    return 0;
}

uint8_t BM_IsActive(void) { return g_bm.active; }
uint8_t BM_IsWaitingAckComplete(void) { return g_bm.waiting_ack_complete; }


// =================================================================================
// SECTION 2: HIGH-LEVEL APPLICATION LOGIC (from refactoring)
// =================================================================================

#define SAMPLES_PER_BURST 8000
enum {
  BM_CTX_MAX_BLOCKS = (SAMPLES_PER_BURST + PROTO_BLOCK_LINES_DEFAULT - 1u) / PROTO_BLOCK_LINES_DEFAULT
};

typedef struct {
    uint16_t base;
    AppContext_t* ctx;
} BurstGenCtx_t;

static int16_t burst_data_x[SAMPLES_PER_BURST];
static int16_t burst_data_y[SAMPLES_PER_BURST];
static int16_t burst_data_z[SAMPLES_PER_BURST];
static uint32_t burst_timestamps[SAMPLES_PER_BURST];
static uint16_t samples_collected_in_burst = 0;
static int16_t g_median_buf[SAMPLES_PER_BURST];

static DataKind_t g_current_kind = KIND_UNKNOWN;
static uint32_t g_current_burst_id = 0;
static uint32_t g_burst_id_counter = 0;
static uint32_t g_active_burst_ms = 0;
static OpMode_t g_mode_before_burst = OP_MODE_IDLE;
static uint32_t last_sample_ms_burst = 0;

static uint16_t g_burst_tx_next_block = 0;
static uint16_t g_burst_tx_total_blocks = 0;
static bool g_burst_tx_ended = false;
static BurstGenCtx_t g_burst_gen_ctxs[BM_CTX_MAX_BLOCKS];

static bool g_burst_after_countdown = false;
static DataKind_t g_pending_burst_kind = KIND_UNKNOWN;
static uint32_t g_pending_burst_id = 0;
static uint32_t g_burst_param_seconds = 0;
static uint32_t g_burst_param_cycles = 0;

static void ProcessAndTransmitBurstData(AppContext_t* ctx);
static int GenDataLine(uint16_t index, char *out, size_t out_sz, void *user);
static float calculate_mean_int16(const int16_t *data, int size);
static void swap_int16(int16_t *a, int16_t *b);
static int16_t quickselect_int16(int16_t *arr, int low, int high, int k);
static int16_t calculate_median_int16(int16_t *data, int size);

void BurstManager_Init(AppContext_t* ctx) {
    (void)ctx;
    BurstManager_Reset(NULL);
    g_burst_id_counter = 0;
}

void BurstManager_Reset(AppContext_t* ctx) {
    (void)ctx;
    samples_collected_in_burst = 0;
    g_current_kind = KIND_UNKNOWN;
    g_current_burst_id = 0;
    g_active_burst_ms = 0;
    g_mode_before_burst = OP_MODE_IDLE;
    g_burst_tx_next_block = 0;
    g_burst_tx_total_blocks = 0;
    g_burst_tx_ended = false;
    g_burst_after_countdown = false;
    g_pending_burst_kind = KIND_UNKNOWN;
    g_pending_burst_id = 0;
    g_burst_param_seconds = 0;
    g_burst_param_cycles = 0;
}

void BurstManager_Configure(AppContext_t* ctx, DataKind_t kind, uint32_t seconds, uint32_t cycles) {
    g_pending_burst_kind = kind;
    g_burst_param_seconds = seconds;
    g_burst_param_cycles = cycles;
    g_burst_after_countdown = true;
    g_pending_burst_id = ++g_burst_id_counter;
    g_mode_before_burst = ctx->op_mode;
    ctx->is_dumping = true;
}

void BurstManager_Start(AppContext_t* ctx, DataKind_t kind, uint32_t burst_id, uint32_t duration_ms) {
    g_current_burst_id = burst_id;
    g_current_kind = kind;
    g_active_burst_ms = duration_ms;
    samples_collected_in_burst = 0;
    last_sample_ms_burst = HAL_GetTick();
    ctx->diag.i2c_fail = 0;
    ctx->diag.ring_ovf = 0;
    if (!ctx->is_dumping) {
        ctx->is_dumping = true;
        ctx->diag.hb_pauses++;
    }
    Sensor_StartSampling(ctx);
    AppContext_SetOpMode(ctx, OP_MODE_BURST);
}

uint32_t BurstManager_GetNextBurstId(AppContext_t* ctx) {
    (void)ctx;
    return ++g_burst_id_counter;
}

DataKind_t BurstManager_GetCurrentKind(AppContext_t* ctx) {
    (void)ctx;
    return g_current_kind;
}

void BurstManager_Pump(AppContext_t* ctx) {
    uint32_t current_tick_ms = HAL_GetTick();

    // Always pump the low-level transport state machine if it's active
    if (BM_IsActive()) {
        BM_Pump();
    }

    switch (ctx->op_mode) {
        case OP_MODE_BURST: {
            uint32_t use_ms = (g_active_burst_ms > 0U) ? g_active_burst_ms : ctx->cfg.burst_ms;
            uint32_t target_samples = (uint32_t)((use_ms * ctx->cfg.odr_hz) / 1000U);
            if (target_samples == 0U) target_samples = 1U;
            if (target_samples > SAMPLES_PER_BURST) target_samples = SAMPLES_PER_BURST;

            uint16_t samples_before_drain = samples_collected_in_burst;
            while (samples_collected_in_burst < target_samples) {
                Sample_t sample;
                if (!Sensor_GetSample(&sample)) break;
                burst_data_x[samples_collected_in_burst] = sample.x;
                burst_data_y[samples_collected_in_burst] = sample.y;
                burst_data_z[samples_collected_in_burst] = sample.z;
                burst_timestamps[samples_collected_in_burst] = sample.timestamp;
                samples_collected_in_burst++;
            }
            if (samples_collected_in_burst > samples_before_drain) {
                last_sample_ms_burst = HAL_GetTick();
            }

            bool time_up = ((current_tick_ms - ctx->state_timer_start_ms) >= use_ms);

            // Stall detection: if sampling stops mid-burst, abort.
            if (!time_up && samples_collected_in_burst < target_samples) {
                if (samples_collected_in_burst > 0 && (current_tick_ms - last_sample_ms_burst) > 500) {
                    Telemetry_SendERROR("BURST", 500, "sampling_stalled");
                    Sensor_StopSampling(ctx);
                    if (BM_IsActive()) {
                        BM_EndAborted(999);
                        ctx->burst_abort_pending = true;
                    } else {
                        BurstManager_Reset(ctx);
                        AppContext_SetOpMode(ctx, g_mode_before_burst);
                    }
                    return;
                }
            }

            if ((samples_collected_in_burst >= target_samples) || time_up) {
                Sensor_StopSampling(ctx);
                ProcessAndTransmitBurstData(ctx);
            }
            break;
        }
        case OP_MODE_BURST_SENDING: {
            if (g_burst_tx_next_block < g_burst_tx_total_blocks) {
                uint16_t lines_in_block = (g_burst_tx_next_block == g_burst_tx_total_blocks - 1)
                    ? (samples_collected_in_burst - g_burst_tx_next_block * PROTO_BLOCK_LINES_DEFAULT)
                    : PROTO_BLOCK_LINES_DEFAULT;
                TB_BlockGen block_generator = { GenDataLine, &g_burst_gen_ctxs[g_burst_tx_next_block], lines_in_block };
                if (BM_Enqueue(&block_generator)) {
                    g_burst_tx_next_block++;
                }
            } else if (!g_burst_tx_ended) {
                BM_EndOk();
                g_burst_tx_ended = true;
            }
            if (!BM_IsActive() && !ctx->burst_abort_pending) {
                BurstManager_Reset(ctx);
                OpMode_t next_mode = (g_current_kind == KIND_DAMP_TRG) ? OP_MODE_WAIT_ARM : g_mode_before_burst;
                if (next_mode == OP_MODE_IDLE && g_mode_before_burst != OP_MODE_IDLE) next_mode = g_mode_before_burst;
                else if (next_mode != OP_MODE_WAIT_ARM) next_mode = OP_MODE_IDLE;
                AppContext_SetOpMode(ctx, next_mode);
            }
            HAL_Delay(5);
            break;
        }
        case OP_MODE_COUNTDOWN: {
            if (!Countdown_IsActive()) {
                if (g_burst_after_countdown) {
                    g_burst_after_countdown = false;
                    uint32_t id = g_pending_burst_id;
                    DataKind_t kind = g_pending_burst_kind;
                    g_pending_burst_id = 0;
                    g_pending_burst_kind = KIND_UNKNOWN;
                    if (kind == KIND_WEIGHT) {
                        if (g_burst_param_cycles > 0 && ctx->cfg.odr_hz > 0) g_active_burst_ms = (uint32_t)((((uint64_t)g_burst_param_cycles * 1000U) / ctx->cfg.odr_hz) + 5);
                        else g_active_burst_ms = 0;
                    } else if (kind == KIND_DAMP_CD) {
                        g_active_burst_ms = (g_burst_param_seconds > 0) ? (g_burst_param_seconds * 1000U) : ctx->cfg.burst_ms;
                    }
                    BurstManager_Start(ctx, kind, id, g_active_burst_ms);
                } else {
                    AppContext_SetOpMode(ctx, OP_MODE_IDLE);
                }
            }
            break;
        }
        default: break;
    }

    if (ctx->burst_abort_pending) {
        if (!BM_IsActive() && COMM_TxIsIdle()) {
            ctx->burst_abort_pending = false;
            BurstManager_Reset(ctx);
            OpMode_t next_mode = (g_current_kind == KIND_DAMP_TRG) ? OP_MODE_WAIT_ARM : g_mode_before_burst;
            if (next_mode != OP_MODE_WAIT_ARM) next_mode = OP_MODE_IDLE;
            AppContext_SetOpMode(ctx, next_mode);
        }
    }
}

static void ProcessAndTransmitBurstData(AppContext_t* ctx) {
    if (!ctx->is_dumping) {
        ctx->is_dumping = true;
        ctx->diag.hb_pauses++;
    }
    const uint16_t samples = samples_collected_in_burst;
    if (g_current_kind == KIND_WEIGHT) {
        if (samples > 0) {
            float mean_ax_raw = calculate_mean_int16(burst_data_x, samples);
            memcpy(g_median_buf, burst_data_x, samples * sizeof(int16_t));
            int16_t median_ax_raw = calculate_median_int16(g_median_buf, samples);
            float mean_ms2 = 0.0f, std_ms2 = 0.0f;
            double sum_ms2 = 0.0, sum2_ms2 = 0.0;
            for(uint16_t i=0; i<samples; ++i) {
                float ax, ay, az;
                Sample_t s = {.x=burst_data_x[i], .y=burst_data_y[i], .z=burst_data_z[i]};
                Sensor_ConvertToMps2(ctx, &s, &ax, &ay, &az);
                float mag_ms2 = sqrtf(ax*ax + ay*ay + az*az);
                sum_ms2 += mag_ms2;
                sum2_ms2 += (double)mag_ms2 * (double)mag_ms2;
            }
            mean_ms2 = (samples > 0) ? (float)(sum_ms2 / samples) : 0.0f;
            float var_ms2 = (samples > 0) ? (float)((sum2_ms2 / samples) - (double)mean_ms2 * (double)mean_ms2) : 0.0f;
            std_ms2 = (var_ms2 > 0.f) ? sqrtf(var_ms2) : 0.f;
            COMM_SendfBlocking(MSG_SUMMARY ",mean_ax_raw=%d,median_ax_raw=%d,mean_ms2=%.3f,std_ms2=%.3f,delta_vinkel_deg=%.3f" PROTO_EOL,
                (int)mean_ax_raw, median_ax_raw, mean_ms2, std_ms2, 0.0f);
        }
        AppContext_SetOpMode(ctx, g_mode_before_burst != OP_MODE_IDLE ? g_mode_before_burst : OP_MODE_IDLE);
        BurstManager_Reset(ctx);
        return;
    }
    uint16_t total_blocks = (samples + PROTO_BLOCK_LINES_DEFAULT - 1) / PROTO_BLOCK_LINES_DEFAULT;
    if (total_blocks > BM_CTX_MAX_BLOCKS) total_blocks = BM_CTX_MAX_BLOCKS;
    for (uint16_t i = 0; i < total_blocks; ++i) {
        g_burst_gen_ctxs[i].base = i * PROTO_BLOCK_LINES_DEFAULT;
        g_burst_gen_ctxs[i].ctx = ctx;
    }
    g_burst_tx_total_blocks = total_blocks;
    g_burst_tx_next_block = 0;
    g_burst_tx_ended = false;
    BM_Type bm_type = (g_current_kind == KIND_DAMP_CD) ? BM_TYPE_DAMP_CD : BM_TYPE_DAMP_TRG;
    AppContext_SetOpMode(ctx, OP_MODE_BURST_SENDING);
    BM_Begin(bm_type, g_current_burst_id, 0, samples, ctx->cfg.odr_hz);
    if (samples == 0) {
        BM_EndOk();
        g_burst_tx_ended = true;
    }
}

static int GenDataLine(uint16_t index, char *out, size_t out_sz, void *user) {
    const BurstGenCtx_t *gen_ctx = (const BurstGenCtx_t *)user;
    AppContext_t* app_ctx = gen_ctx->ctx;
    const uint16_t i = (uint16_t)(gen_ctx->base + index);
    if (i >= SAMPLES_PER_BURST) return -1;
    float ax_mps2, ay_mps2, az_mps2;
    Sample_t s = {.x = burst_data_x[i], .y = burst_data_y[i], .z = burst_data_z[i]};
    Sensor_ConvertToMps2(app_ctx, &s, &ax_mps2, &ay_mps2, &az_mps2);
    int ret = snprintf(out, out_sz, "DATA,%lu,%.3f,%.3f,%.3f,%.3f" PROTO_EOL,
        (unsigned long)Sensor_TicksToUs(app_ctx, burst_timestamps[i]),
        ax_mps2, ay_mps2, az_mps2, 0.0f);
    if (ret < 0 || (size_t)ret >= out_sz) return -1;
    return ret;
}

static float calculate_mean_int16(const int16_t *data, int size) {
  if (size == 0) return 0.0f;
  long sum = 0;
  for (int i = 0; i < size; i++) sum += data[i];
  return (float)sum / size;
}

static void swap_int16(int16_t *a, int16_t *b) {
  int16_t temp = *a; *a = *b; *b = temp;
}

static int16_t quickselect_int16(int16_t *arr, int low, int high, int k) {
  while (low <= high) {
    if (low == high) return arr[low];
    int pivot_idx = low + (high - low) / 2;
    int16_t pivot = arr[pivot_idx];
    swap_int16(&arr[pivot_idx], &arr[high]);
    int i = low;
    for (int j = low; j < high; j++) {
      if (arr[j] < pivot) {
        swap_int16(&arr[i], &arr[j]);
        i++;
      }
    }
    swap_int16(&arr[i], &arr[high]);
    if (k == i) return arr[i];
    else if (k < i) high = i - 1;
    else low = i + 1;
  }
  return INT16_MIN;
}

static int16_t calculate_median_int16(int16_t *data, int size) {
  if (size == 0) return 0;
  if (size == 1) return data[0];
  if (size % 2 == 1) {
    return quickselect_int16(data, 0, size - 1, size / 2);
  } else {
    int mid1_idx = size / 2 - 1;
    int mid2_idx = size / 2;
    int16_t mid1 = quickselect_int16(data, 0, size - 1, mid1_idx);
    int16_t mid2 = quickselect_int16(data, mid1_idx + 1, size - 1, mid2_idx);
    return (int16_t)(((int32_t)mid1 + (int32_t)mid2) / 2);
  }
}
