/*
 * Filename: Core/Src/transport_blocks.c
 * Antagande: Denna fil är en del av ett STM32CubeIDE-projekt och
 *            interagerar med andra moduler som comm.c och burst_mgr.c.
 */
/* filename: Core/Src/transport_blocks.c */
#include "transport_blocks.h"
#include "stm32f4xx_hal.h"
#include "burst_mgr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dev_telemetry.h" // Inkludera ny debug-header

_Static_assert(PROTO_EOL_LEN == 2, "EOL length assumption is invalid");
_Static_assert(PROTO_MAX_LINE >= 256, "Line buffer smaller than spec requirement");

#ifndef TB_MAX_INFLIGHT
#define TB_MAX_INFLIGHT 8u
#endif
#ifndef TB_MAX_QUEUE
#define TB_MAX_QUEUE 16u
#endif

typedef struct {
    TB_GenLineFn gen;
    void*        user;
    uint16_t     lines;
    uint16_t     blk;
    uint16_t     crc16;
    uint8_t      retries;
    uint8_t      sent;
    uint32_t     t_last_tx_ms;
    uint8_t      inflight;
    uint8_t      done;
} tb_entry_t;

static struct {
    uint16_t window;
    uint16_t blk_lines;
    uint8_t  max_retries;
    uint32_t burst_id;
    uint16_t next_blk;
    uint8_t  burst_active;
    tb_entry_t queue[TB_MAX_QUEUE];
    uint8_t q_head, q_tail, q_count;
    tb_entry_t* inflight[TB_MAX_INFLIGHT];
    uint8_t     inflight_count;
} g_tb;

static inline int _queue_full(void)  { return g_tb.q_count >= TB_MAX_QUEUE; }
static inline int _queue_empty(void) { return g_tb.q_count == 0; }

static tb_entry_t* _queue_push(void) {
    if (_queue_full()) return NULL;
    tb_entry_t* e = &g_tb.queue[g_tb.q_tail];
    memset(e, 0, sizeof(*e));
    g_tb.q_tail = (uint8_t)((g_tb.q_tail + 1u) % TB_MAX_QUEUE);
    g_tb.q_count++;
    return e;
}

static tb_entry_t* _queue_pop(void) {
    if (_queue_empty()) return NULL;
    tb_entry_t* e = &g_tb.queue[g_tb.q_head];
    g_tb.q_head = (uint8_t)((g_tb.q_head + 1u) % TB_MAX_QUEUE);
    g_tb.q_count--;
    return e;
}

static void _inflight_add(tb_entry_t* e) {
    if (g_tb.inflight_count < TB_MAX_INFLIGHT) {
        g_tb.inflight[g_tb.inflight_count++] = e;
        e->inflight = 1;
    }
}

static void _inflight_remove_index(uint8_t idx) {
    if (idx >= g_tb.inflight_count) return;
    for (uint8_t i = (uint8_t)(idx + 1u); i < g_tb.inflight_count; ++i) {
        g_tb.inflight[i - 1u] = g_tb.inflight[i];
    }
    g_tb.inflight_count--;
}

static int _inflight_find_blk(uint16_t blk, uint8_t* out_idx) {
    for (uint8_t i = 0; i < g_tb.inflight_count; ++i) {
        if (g_tb.inflight[i]->blk == blk && g_tb.inflight[i]->inflight) {
            if (out_idx) *out_idx = i;
            return 1;
        }
    }
    return 0;
}

static void _abort_all(void) {
    for (uint8_t i = 0; i < g_tb.inflight_count; ++i) {
        g_tb.inflight[i]->inflight = 0;
        g_tb.inflight[i]->done = 1;
    }
    g_tb.inflight_count = 0;
    while (!_queue_empty()) { (void)_queue_pop(); }
    g_tb.burst_active = 0;
}

void TB_Init(uint16_t window, uint16_t blk_lines, uint8_t max_retries) {
    if (window == 0 || window > TB_MAX_INFLIGHT) window = TB_MAX_INFLIGHT;
    if (blk_lines == 0) blk_lines = PROTO_BLOCK_LINES_DEFAULT;
    if (max_retries == 0) max_retries = PROTO_MAX_RETRIES;
    g_tb.window = window;
    g_tb.blk_lines = blk_lines;
    g_tb.max_retries = max_retries;
    g_tb.burst_id = 0;
    g_tb.next_blk = 0;
    g_tb.burst_active = 0;
    g_tb.q_head = g_tb.q_tail = g_tb.q_count = 0;
    g_tb.inflight_count = 0;
}

void TB_SetWindow(uint16_t window) { g_tb.window = (window == 0) ? 1u : window; }
void TB_SetBlockLines(uint16_t blk_lines) { g_tb.blk_lines = (blk_lines == 0) ? 1u : blk_lines; }
void TB_SetMaxRetries(uint8_t max_retries) { g_tb.max_retries = (max_retries == 0) ? 1u : max_retries; }

void TB_BeginBurst(uint32_t burst_id) {
    g_tb.burst_id = burst_id;
    g_tb.next_blk = 1;
    g_tb.burst_active = 1u;
    g_tb.q_head = g_tb.q_tail = g_tb.q_count = 0;
    g_tb.inflight_count = 0;
}

void TB_EndBurst(void) {
    g_tb.burst_active = 0;
}

static uint16_t _compute_crc16(TB_GenLineFn gen, void* user, uint16_t lines) {
    proto_crc16_t c; proto_crc16_init(&c);
    char line[PROTO_MAX_LINE];
    for (uint16_t i = 0; i < lines; ++i) {
        int n = gen(i, line, sizeof line, user);
        if (n <= 0) return 0;
        proto_crc16_update(&c, line, (size_t)n); /* inkluderar CRLF */
    }
    return proto_crc16_final(&c);
}

static void _send_block(tb_entry_t* e) {
    COMM_SendfBlocking("BLOCK_HEADER,burst_id=%lu,blk=%u,lines=%u,crc16=%u" PROTO_EOL,
               (unsigned long)g_tb.burst_id, (unsigned)e->blk, (unsigned)e->lines, (unsigned)e->crc16);
    char line[PROTO_MAX_LINE];
    for (uint16_t i = 0; i < e->lines; ++i) {
        int n = e->gen(i, line, sizeof line, e->user);
        if (n > 0) {
            Telemetry_WriteBlocking(line, (size_t)n);
        }
    }
    COMM_SendfBlocking("BLOCK_END,blk=%u,crc16=%u" PROTO_EOL, (unsigned)e->blk, (unsigned)e->crc16);
    e->t_last_tx_ms = HAL_GetTick();
    e->sent = 1;
    e->inflight = 1;
}

int TB_EnqueueBlock(const TB_BlockGen* blk) {
    if (!g_tb.burst_active || blk == NULL || blk->gen == NULL || blk->lines == 0 || blk->lines > g_tb.blk_lines) {
        return 0;
    }
    if (_queue_full()) return 0;
    tb_entry_t* e = _queue_push();
    e->gen = blk->gen;
    e->user = blk->user;
    e->lines = blk->lines;
    e->blk = g_tb.next_blk++;
    e->crc16 = _compute_crc16(e->gen, e->user, e->lines);
    e->retries = 0;
    e->sent = 0;
    e->inflight = 0;
    e->done = 0;
    return 1;
}

static void _pump_send(void) {
    while (g_tb.inflight_count < g_tb.window && !_queue_empty()) {
        tb_entry_t* e = _queue_pop();
        _send_block(e);
        _inflight_add(e);
    }
}

static void _pump_timeouts(void) {
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < g_tb.inflight_count; ) {
        tb_entry_t* e = g_tb.inflight[i];
        if (!e->inflight) { _inflight_remove_index(i); continue; }
        if ((uint32_t)(now - e->t_last_tx_ms) >= PROTO_BLOCK_TIMEOUT_MS) {
            if (e->retries < g_tb.max_retries) {
                e->retries++;
                _send_block(e);
                i++;
            } else {
                _abort_all();
                BM_EndAborted(400u);
            }
        } else {
            i++;
        }
    }
}

void TB_Pump(void) {
    if (!g_tb.burst_active) return;

    #if RXTX_DEBUG > 0
    uint8_t q_before = g_tb.q_count;
    uint8_t i_before = g_tb.inflight_count;
    #endif

    _pump_send();
    _pump_timeouts();

    #if RXTX_DEBUG > 0
    // Logga endast om status har ändrats för att minska spam
    if (q_before != g_tb.q_count || i_before != g_tb.inflight_count) {
        DevTel_LogTbStatus(g_tb.q_count, TB_MAX_QUEUE, g_tb.inflight_count, g_tb.window);
    }
    #endif
}


int TB_IsIdle(void) {
    /* Idle = inga inflight-block och tom kö */
    return (g_tb.inflight_count == 0) && _queue_empty();
}

void TB_OnAckBlk(uint16_t blk) {
    uint8_t idx;
    if (_inflight_find_blk(blk, &idx)) {
        tb_entry_t* e = g_tb.inflight[idx];
        e->done = 1;
        e->inflight = 0;
        _inflight_remove_index(idx);
    }
}

void TB_OnNackBlk(uint16_t blk, uint32_t code) {
    (void)code; /* felkod mappas högre upp i felmodellen */
    uint8_t idx;
    if (_inflight_find_blk(blk, &idx)) {
        tb_entry_t* e = g_tb.inflight[idx];
        if (e->retries < g_tb.max_retries) {
            e->retries++;
            _send_block(e);
        } else {
            _abort_all();
            BM_EndAborted((code != 0u) ? code : 400u);
        }
    }
}

static int _parse_u16(const char* s, uint16_t* out) {
    unsigned long v = 0;
    if (sscanf(s, "%lu", &v) == 1) { *out = (uint16_t)v; return 1; }
    return 0;
}
static int _parse_u32(const char* s, uint32_t* out) {
    unsigned long v = 0;
    if (sscanf(s, "%lu", &v) == 1) { *out = (uint32_t)v; return 1; }
    return 0;
}

int TB_HandleHostLine(const char* line) {
    if (!line) return 0;
    if (strncmp(line, "ACK_BLK", 7) == 0) {
        const char* p = strstr(line, "blk=");
        uint16_t blk = 0;
        if (p && _parse_u16(p + 4, &blk)) {
            TB_OnAckBlk(blk);
            return 1;
        }
    } else if (strncmp(line, "NACK_BLK", 8) == 0) {
        const char* p = strstr(line, "blk=");
        const char* q = strstr(line, "code=");
        uint16_t blk = 0; uint32_t code = 0;
        if (p && _parse_u16(p + 4, &blk)) {
            if (q) _parse_u32(q + 5, &code);
            TB_OnNackBlk(blk, code);
            return 1;
        }
    }
    return 0;
}

uint8_t TB_GetQueueCount(void)
{
    return g_tb.q_count;
}

uint8_t TB_GetInflightCount(void)
{
    return g_tb.inflight_count;
}
