/* filename: Core/Src/streaming.c */
#include "streaming.h"
#include "sensor_hal.h"
#include "comm.h"
#include "api_schema.h"

// --- Streaming State (private to this module) ---
static volatile bool g_stream_enabled = false;
static volatile uint32_t g_stream_div = 8;
static volatile uint32_t g_stream_decim = 0;
static volatile uint32_t g_stream_seq = 0;
static bool g_stream_suspended = false;
static bool g_stream_owns_timer = false;

// --- Handoff buffer from ISR to main loop ---
static volatile bool g_live_ready = false;
static struct {
  uint32_t seq;
  int16_t  x, y, z;
  uint32_t ts_us;
} g_live_buf;


// --- Public Functions ---

void Streaming_Init(AppContext_t* ctx) {
    (void)ctx;
    g_stream_enabled = false;
    g_stream_suspended = false;
    g_stream_owns_timer = false;
    g_stream_seq = 0;
    g_stream_decim = 0;
    g_live_ready = false;
    g_stream_div = (DEFAULT_ODR_HZ / DEFAULT_STREAM_HZ > 1) ? (DEFAULT_ODR_HZ / DEFAULT_STREAM_HZ) : 1;
}

void Streaming_Start(AppContext_t* ctx) {
    __disable_irq();
    g_stream_seq = 0;
    g_stream_decim = 0;
    __enable_irq();

    Streaming_UpdateDivider(ctx);

    if (ctx->op_mode == OP_MODE_IDLE) {
        Sensor_StartSampling(ctx);
        g_stream_owns_timer = true;
    } else {
        g_stream_owns_timer = false;
    }
    g_stream_enabled = true;
    ctx->is_dumping = true;
    ctx->diag.hb_pauses++;
}

void Streaming_Stop(AppContext_t* ctx) {
    g_stream_enabled = false;
    if (ctx->op_mode == OP_MODE_IDLE && g_stream_owns_timer) {
        Sensor_StopSampling(ctx);
        g_stream_owns_timer = false;
    }
    // Set is_dumping to true to allow any final messages to clear
    // before heartbeats resume. It will be cleared by the main loop logic.
    ctx->is_dumping = true; 
}

void Streaming_Pump(AppContext_t* ctx) {
    if (!g_live_ready) return;
    if (!g_stream_enabled) {
        __disable_irq();
        g_live_ready = false;
        __enable_irq();
        return;
    }

    // Check if there is enough space in the TX buffer to avoid blocking.
    if (COMM_TxFree() > 128U) {
        // Atomically copy the data from the handoff buffer
        __disable_irq();
        uint32_t seq = g_live_buf.seq;
        uint32_t ts = g_live_buf.ts_us;
        int16_t x = g_live_buf.x, y = g_live_buf.y, z = g_live_buf.z;
        g_live_ready = false;
        __enable_irq();

        // Send the formatted message
        COMM_Sendf(MSG_LIVE ",seq=%lu,ax=%d,ay=%d,az=%d,ts_us=%lu" PROTO_EOL,
                   seq, (int)x, (int)y, (int)z, (unsigned long)ts);
    }
}

void Streaming_ProcessSampleFromISR(AppContext_t* ctx, const Sample_t* s) {
    if (!g_stream_enabled) {
        return;
    }

    g_stream_decim++;
    if (g_stream_decim >= g_stream_div) {
        g_stream_decim = 0;

        // Produce a new LIVE line only if the previous one has been consumed
        // and there's ample TX buffer space to prevent ISR-level churn.
        if (!g_live_ready && COMM_TxFree() > 128U) {
            g_live_buf.seq   = g_stream_seq++;
            g_live_buf.x     = s->x;
            g_live_buf.y     = s->y;
            g_live_buf.z     = s->z;
            g_live_buf.ts_us = Sensor_TicksToUs(ctx, s->timestamp);
            g_live_ready = true;
        } else {
            ctx->diag.live_drops++;
        }
    }
}

void Streaming_Reconfigure(AppContext_t* ctx) {
    if (g_stream_enabled) {
        if (g_stream_owns_timer && ctx->op_mode == OP_MODE_IDLE) {
            Sensor_StopSampling(ctx);
        }
        g_stream_enabled = false;
        g_stream_owns_timer = false;
        g_stream_suspended = true;
        __disable_irq();
        g_stream_decim = 0;
        __enable_irq();
    }
}

void Streaming_UpdateDivider(AppContext_t* ctx) {
    g_stream_div = (ctx->cfg.stream_rate_hz > 0U)
                     ? (ctx->cfg.odr_hz / ctx->cfg.stream_rate_hz)
                     : 1U;
    if (g_stream_div == 0U) {
        g_stream_div = 1U;
    }
}

uint32_t Streaming_GetDivider(AppContext_t* ctx) {
    (void)ctx;
    return g_stream_div;
}