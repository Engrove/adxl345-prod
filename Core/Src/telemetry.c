/*
PATCH SUMMARY:
- Root cause: The `STATUS` and `TRG_SETTINGS` messages sent by the firmware contained extra, non-normative fields (`th`, `hy` in STATUS; `win_ms` in TRG_SETTINGS). This was identified in `technical_debt.json` and confirmed by the test logs.
- Fix approach: The `snprintf` format strings in `Telemetry_SendStatus` and `Telemetry_SendTrgSettings` have been modified to remove the non-normative fields. The corrected formats now strictly adhere to the definitions in API v12.1, sections §11.1 and §13.1.
- Spec traceability: (list § and bullets)
  - §11.1 Status: The `STATUS` message format is now `STATUS,op=<STATE>,trg=<idle|armed|holdoff>,axis=MAG`, removing the `th` and `hy` fields.
  - §13.1 Läs och sätt trigger: The `TRG_SETTINGS` message format is now `TRG_SETTINGS,k_mult=<f32>,hold_ms=<u32>`, removing the `win_ms` field.
- Non-blocking & power management notes: N/A.
- Risks and mitigations: No risk. This is a trivial string format correction to align with the specification.
- Tests to run: 17.1 (Handshake och grundinställning) to verify the corrected `STATUS` and `TRG_SETTINGS` formats.
*/
/* filename: Core/Src/telemetry.c */
#include "telemetry.h"
#include "comm.h"
#include "api_schema.h"
#include "sensor_hal.h" // For Sensor_TicksToUs and preview data
#include "streaming.h"  // For Streaming_GetDivider
#include "burst_mgr.h"  // For BM_IsActive
#include "transport_blocks.h" // For TB_GetQueueCount, etc.

#include <stdio.h>
#include <math.h> // For atan2f in preview calculation

// --- Static variables ---
static uint32_t g_hb_last_ms = 0;

// --- Static Function Prototypes ---
static const char *op_mode_to_str(OpMode_t mode);
static const char *trg_state_to_str(TrgState_t state);
static float theta_deg_from_ms2(float ax_ms2, float ay_ms2);

// --- Public Functions ---

void Telemetry_Init(AppContext_t* ctx) {
    (void)ctx; // No specific initialization needed for this module yet
    g_hb_last_ms = 0;
}

void Telemetry_Pump(AppContext_t* ctx) {
    uint32_t current_tick_ms = HAL_GetTick();

    // Per API v12.1 §4.1, HB is gated (paused) only during active BLOCKS transfers.
    if (!BM_IsActive() && ctx->cfg.hb_ms > 0 && (current_tick_ms - g_hb_last_ms) >= ctx->cfg.hb_ms) {
        g_hb_last_ms = current_tick_ms;
        uint64_t host_ms = 0;
        if (ctx->tsync.has_sync) {
            uint32_t now_ticks = __HAL_TIM_GET_COUNTER(ctx->htim2);
            uint32_t dt_ticks = now_ticks - ctx->tsync.tick_at_sync;
            uint32_t delta_us = Sensor_TicksToUs(ctx, dt_ticks);
            host_ms = ctx->tsync.host_ms_at_sync + (uint64_t)(delta_us / 1000u);
            uint32_t host_hi = (uint32_t)(host_ms >> 32);
            uint32_t host_lo = (uint32_t)(host_ms & 0xFFFFFFFFu);
            COMM_Sendf(MSG_HB ",tick=%lu,host_hi=%lu,host_lo=%lu,tx_free=%u,tx_drop=%lu" PROTO_EOL,
                       (unsigned long)current_tick_ms, host_hi, host_lo,
                       (unsigned)COMM_TxFree(), (unsigned long)COMM_TxDropCount());
        } else {
            COMM_Sendf(MSG_HB ",tick=%lu,tx_free=%u,tx_drop=%lu" PROTO_EOL,
                       (unsigned long)current_tick_ms, (unsigned)COMM_TxFree(),
                       (unsigned long)COMM_TxDropCount());
        }
    }
}

void Telemetry_SendStatus(AppContext_t* ctx) {
    // Avoid sending STATUS during a block transfer; a correct status will be sent after ACK_COMPLETE.
    if (ctx->op_mode == OP_MODE_BURST_SENDING && BM_IsActive()) {
        return;
    }
    COMM_Sendf(MSG_STATUS ",op=%s,trg=%s,axis=MAG" PROTO_EOL,
               op_mode_to_str(ctx->op_mode), trg_state_to_str(ctx->trg_state));
}

void Telemetry_SendCfg(AppContext_t* ctx) {
    COMM_Sendf(MSG_CFG ",odr_hz=%lu,burst_ms=%lu,hb_ms=%lu,stream_rate_hz=%lu" PROTO_EOL,
               ctx->cfg.odr_hz, ctx->cfg.burst_ms, ctx->cfg.hb_ms, ctx->cfg.stream_rate_hz);
}

void Telemetry_SendTrgSettings(AppContext_t* ctx) {
    COMM_Sendf(MSG_TRG_SETTINGS ",k_mult=%.3f,hold_ms=%lu" PROTO_EOL,
               ctx->trigger_settings.k_mult,
               (unsigned long)ctx->trigger_settings.hold_ms);
}

void Telemetry_SendACK(const char* subject) {
    COMM_Sendf(MSG_ACK ",SUBJECT=%s" PROTO_EOL, subject);
}

void Telemetry_SendNACK(const char* subject, const char* reason, uint32_t code) {
    COMM_Sendf(MSG_NACK ",SUBJECT=%s,reason=%s,code=%lu" PROTO_EOL, subject, reason, code);
}

void Telemetry_SendERROR(const char* src, uint32_t code, const char* msg) {
    COMM_Sendf(MSG_ERROR ",src=%s,code=%lu,msg=\"%s\"" PROTO_EOL, src, code, msg);
}

void Telemetry_SendStreamStartACK(AppContext_t* ctx) {
    COMM_Sendf(MSG_ACK ",SUBJECT=" CMD_STREAM_START ",rate_hz=%lu,div=%lu" PROTO_EOL,
               ctx->cfg.stream_rate_hz, Streaming_GetDivider(ctx));
}

void Telemetry_SendCalInfo(AppContext_t* ctx) {
    (void)ctx;
    COMM_Sendf(MSG_CAL_INFO ",status=hold_zero,duration_ms=%u,instr_id=HOLD_ZERO" PROTO_EOL,
               REF_CAPTURE_DURATION_MS);
}

void Telemetry_SendDiag(AppContext_t* ctx) {
#if defined(RXTX_DEBUG) && RXTX_DEBUG > 0
    (void)ctx;
    Telemetry_SendACK(CMD_GET_DIAG);
    COMM_SendfBlocking("[DEBUG] DIAG_STATS: tx_drops=%lu, rx_ovf=%lu\r\n",
                       COMM_TxDropCount(), COMM_RxOverflowCount());
    COMM_SendfBlocking("[DEBUG] DIAG_BUFS: rx_ring=%u/%u, tx_ring=%u/%u\r\n",
                       COMM_RxRingUsage(), RX_RING_BUFFER_SIZE,
                       COMM_TxRingUsage(), COMM_TX_RING_SIZE);
    uint8_t q_count = TB_GetQueueCount();
    uint8_t inflight_count = TB_GetInflightCount();
    COMM_SendfBlocking("[DEBUG] DIAG_BLOCKS: queue=%u, inflight=%u\r\n",
                       q_count, inflight_count);
#else
    (void)ctx;
    Telemetry_SendNACK(CMD_GET_DIAG, "not_supported", 900);
#endif
}

void Telemetry_SendPreview(AppContext_t* ctx) {
    const PreviewSnap_t* preview = Sensor_GetPreviewSnapshot(ctx);
    if (!preview) return;

    COMM_Sendf(MSG_PREVIEW_HEADER ",samples=%u" PROTO_EOL, preview->count);
    for (uint16_t i = 0; i < preview->count; i++) {
        float ax_mps2, ay_mps2, az_mps2;
        Sensor_ConvertToMps2(ctx, &preview->buf[i], &ax_mps2, &ay_mps2, &az_mps2);
        
        float theta_deg = theta_deg_from_ms2(ax_mps2, ay_mps2);
        
        COMM_Sendf(MSG_PREVIEW ",ts_us=%lu,ax=%.3f,ay=%.3f,az=%.3f,theta=%.3f" PROTO_EOL,
                   (unsigned long)Sensor_TicksToUs(ctx, preview->buf[i].timestamp),
                   ax_mps2, ay_mps2, az_mps2, theta_deg);
    }
    COMM_Send(MSG_PREVIEW_END PROTO_EOL);
}

void Telemetry_UpdateLED(AppContext_t* ctx) {
    uint32_t tick = HAL_GetTick();
    switch (ctx->op_mode) {
    case OP_MODE_INIT:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        break;
    case OP_MODE_IDLE:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        break;
    case OP_MODE_TRG_CAL_ZERO:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (tick / 500) % 2);
        break;
    case OP_MODE_BURST:
    case OP_MODE_BURST_SENDING:
    case OP_MODE_COUNTDOWN:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (tick / 100) % 2);
        break;
    case OP_MODE_ARMED:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (tick / 2000) % 2);
        break;
    case OP_MODE_WAIT_ARM:
    case OP_MODE_WAIT_CAL_ZERO:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (tick % 2000) < 100);
        break;
    default:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        break;
    }
}

void Telemetry_Flush(void) {
    uint32_t t0 = HAL_GetTick();
    while (!COMM_TxIsIdle() && (HAL_GetTick() - t0) < 50) {
        HAL_Delay(1);
    }
}

// --- Static Helper Functions ---

static const char *op_mode_to_str(OpMode_t m) {
    switch (m) {
    case OP_MODE_INIT: return "INIT";
    case OP_MODE_IDLE: return "IDLE";
    case OP_MODE_WAIT_CAL_ZERO: return "WAIT_CAL_ZERO";
    case OP_MODE_TRG_CAL_ZERO: return "TRG_CAL_ZERO";
    case OP_MODE_WAIT_ARM: return "WAIT_ARM";
    case OP_MODE_ARMED: return "ARMED";
    case OP_MODE_COUNTDOWN: return "COUNTDOWN";
    case OP_MODE_BURST: return "BURST";
    case OP_MODE_BURST_SENDING: return "BURST_SENDING";
    case OP_MODE_STATIC_RUN: return "STATIC_RUN";
    case OP_MODE_STREAMING: return "STREAMING";
    case OP_MODE_ERROR: return "ERROR";
    default: return "UNKNOWN";
    }
}

static const char *trg_state_to_str(TrgState_t s) {
    switch (s) {
    case TRG_STATE_ARMED: return "armed";
    case TRG_STATE_IN_HOLDOFF: return "holdoff";
    default: return "idle";
    }
}

static float theta_deg_from_ms2(float ax_ms2, float ay_ms2) {
    const float RAD2DEG = 57.29577951308232f;
    float theta = atan2f(ay_ms2, ax_ms2) * RAD2DEG;
    return (theta >= 0.0f) ? theta : -theta;
}