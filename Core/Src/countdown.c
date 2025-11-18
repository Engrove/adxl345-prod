/* Core/Src/countdown.c */
#include "main.h"
#include "countdown.h"
#include "comm.h"
#include "api_schema.h"
#include <stdbool.h>
#include <string.h>

typedef struct {
    bool active;
    uint8_t step;
    uint32_t last_ms;
} cd_ctx_t;

static cd_ctx_t g_cd;

void Countdown_Init(void) {
    memset(&g_cd, 0, sizeof(g_cd));
}

void Countdown_Start(uint8_t seconds) {
    if (seconds < 5) seconds = 5;
    if (seconds > 10) seconds = 10;
    g_cd.active  = true;
    g_cd.step    = seconds;
    g_cd.last_ms = HAL_GetTick();
    COMM_Sendf(MSG_COUNTDOWN_ID ",id=%u" PROTO_EOL, g_cd.step);
}

void Countdown_Stop(void) {
    if (!g_cd.active) return;
    g_cd.active = false;
    /* v3.0.7: id=0 endast vid avbrutet förlopp */
    COMM_Send(MSG_COUNTDOWN_ID ",id=0" PROTO_EOL);
}

bool Countdown_IsActive(void) { return g_cd.active; }

void Countdown_Tick(void) {
    if (!g_cd.active) return;
    uint32_t now = HAL_GetTick();
    if ((now - g_cd.last_ms) >= 1000U) {
        g_cd.last_ms = now;
        if (g_cd.step > 1) {
            g_cd.step--;
            COMM_Sendf(MSG_COUNTDOWN_ID ",id=%u" PROTO_EOL, g_cd.step);
        } else {
            /* Normal completion: inget id=0 */
            g_cd.active = false;
        }
    }
}
/* Core/Src/countdown.c */