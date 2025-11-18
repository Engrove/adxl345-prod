/*
 * FIL: Core/Src/cmd_handler.c
 * ANM: Patchad för att göra _TEST_FORCE_TRIGGER-kommandot permanent tillgängligt.
 */
/*
PATCH SUMMARY:
- Fix: Lade till hantering för det nya diagnostikkommandot `DIAG_HW_TEST` (via en test-hook) för att möjliggöra lågnivåfelsökning av ADXL345/DMA/EXTI-problemet.
- Key functions / ISRs touched: process_command, handle_diag_hw_test
*/
/* filename: Core/Src/cmd_handler.c */
#include "cmd_handler.h"
#include "app_context.h"
#include "comm.h"
#include "telemetry.h"
#include "api_parse.h"
#include "api_schema.h"
#include "trigger_logic.h"
#include "burst_mgr.h"
#include "streaming.h"
#include "countdown.h"
#include "sensor_hal.h"
#include "dev_diagnostics.h" // Inkludera den nya diagnostikfilen

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// --- Static variables ---
// A static pointer to the global application context.
// This is necessary because the legacy comm.c calls process_command() without a context pointer.
static AppContext_t* s_ctx;
// A static flag to prevent re-entrancy in process_command.
static volatile bool g_is_processing_command = false;

// --- Static Function Prototypes ---
static void Parse_SetCfg(const char *line);
static void Parse_SetTrg(const char *line);
static void Parse_TimeSync(const char *line);
static void Parse_StreamStart(const char *line);
static void Parse_StartBurstWeight(const char *line);
static void Parse_StartBurstDamping(const char *line);
static void Parse_Mode(const char *line);
static void Parse_CalReady(const char *line);
static void Parse_HB(const char *line);
static void Parse_AdxlSt(const char *line);
static inline int cmd_exact(const char *line, const char *cmd);
static void handle_diag_hw_test(AppContext_t* ctx, const char* line); // Ny prototyp

// --- Public Functions ---

void CmdHandler_Init(AppContext_t* ctx) {
    s_ctx = ctx;
}

void CmdHandler_ProcessInput(AppContext_t* ctx) {
    // This function orchestrates the processing of incoming bytes from the comms buffer.
    // It calls COMM_Process_Budgeted, which in turn will call the global process_command function below.
    // The context is passed via the static s_ctx pointer set during Init.
    (void)ctx; // The context is used via the static pointer s_ctx.
    COMM_Process_Budgeted();
}

void CmdHandler_HandleStop(AppContext_t* ctx) {
    OpMode_t prev_mode = ctx->op_mode;

    Telemetry_SendACK(CMD_STOP);
    Telemetry_Flush();

    if (prev_mode == OP_MODE_BURST || prev_mode == OP_MODE_BURST_SENDING) {
        Sensor_StopSampling(ctx);

        if (BM_IsActive()) {
            BM_EndAborted(0);
            ctx->burst_abort_pending = true;
        } else {
            BurstManager_Reset(ctx);
            Trigger_Reset(ctx);
            OpMode_t next_mode = OP_MODE_IDLE;
            if (BurstManager_GetCurrentKind(ctx) == KIND_DAMP_TRG) {
                next_mode = OP_MODE_WAIT_ARM;
            }
            AppContext_SetOpMode(ctx, next_mode);
        }
    } else {
        Sensor_StopSampling(ctx);
        Streaming_Stop(ctx);

        if (prev_mode == OP_MODE_COUNTDOWN) {
            Countdown_Stop();
        }

        Trigger_Reset(ctx);
        BurstManager_Reset(ctx);
        ctx->is_dumping = false;

        OpMode_t next_idle_mode = (prev_mode == OP_MODE_ARMED) ? OP_MODE_WAIT_ARM : OP_MODE_IDLE;
        AppContext_SetOpMode(ctx, next_idle_mode);
    }
}

// --- Command Processing Logic (moved from main.c) ---

/**
 * @brief Main command dispatcher.
 * @note This function has global scope to be callable by the existing comm.c module.
 *       It uses a static context pointer `s_ctx` set by `CmdHandler_Init`.
 *       The instruction to make all functions static was overridden for this function
 *       to maintain compatibility with the unmodified comm.c module.
 */
void process_command(char *line) {
    g_is_processing_command = true;
    if (!line || !*line || !s_ctx) {
        g_is_processing_command = false;
        return;
    }
    while (*line == ' ')
        line++;

    if (BM_HandleHostLine(line)) {
        g_is_processing_command = false;
        return;
    }

    if (cmd_exact(line, CMD_HELLO)) {
        memset(&s_ctx->diag, 0, sizeof(s_ctx->diag));
        s_ctx->tsync.has_sync = false;
        s_ctx->stop_flag = false;
        COMM_Sendf(MSG_HELLO_ACK ",fw=\"%s\",proto=%s,win=%u,blk_lines=%u" PROTO_EOL,
                   FW_VERSION, "3.3.3", (unsigned)PROTO_WINDOW_DEFAULT,
                   (unsigned)PROTO_BLOCK_LINES_DEFAULT);
        AppContext_SetOpMode(s_ctx, OP_MODE_IDLE);
    } else if (cmd_exact(line, CMD_GET_STATUS)) {
        Telemetry_SendStatus(s_ctx);
    } else if (cmd_exact(line, CMD_GET_CFG)) {
        Telemetry_SendCfg(s_ctx);
    } else if (cmd_exact(line, CMD_SET_CFG)) {
        Parse_SetCfg(line);
    } else if (cmd_exact(line, CMD_HB)) {
        Parse_HB(line);
    } else if (cmd_exact(line, CMD_TIME_SYNC)) {
        Parse_TimeSync(line);
    } else if (cmd_exact(line, CMD_STREAM_START)) {
        Parse_StreamStart(line);
    } else if (cmd_exact(line, CMD_STREAM_STOP)) {
        Streaming_Stop(s_ctx);
        Telemetry_SendACK(CMD_STREAM_STOP);
    } else if (cmd_exact(line, CMD_GET_TRG)) {
        Telemetry_SendTrgSettings(s_ctx);
    } else if (cmd_exact(line, CMD_SET_TRG)) {
        Parse_SetTrg(line);
    } else if (cmd_exact(line, CMD_MODE)) {
        Parse_Mode(line);
    } else if (cmd_exact(line, CMD_CAL_READY)) {
        Parse_CalReady(line);
    } else if (cmd_exact(line, CMD_ARM)) {
        if (s_ctx->op_mode == OP_MODE_WAIT_ARM) {
            if (!Trigger_IsZeroCalibrated(s_ctx)) {
                Telemetry_SendNACK(CMD_ARM, "zero_not_calibrated", 104);
                g_is_processing_command = false;
                return;
            }
            Telemetry_SendACK(CMD_ARM);
            Telemetry_Flush();
            s_ctx->is_dumping = true;
            s_ctx->diag.hb_pauses++;

            Sensor_StartSampling(s_ctx);
            Trigger_Arm(s_ctx);
            Sensor_StartSampling(s_ctx); // Ensure sampling is on for monitoring
            s_ctx->trg_state = TRG_STATE_ARMED;
            AppContext_SetOpMode(s_ctx, OP_MODE_ARMED);

        } else if (s_ctx->op_mode == OP_MODE_ARMED) {
            Telemetry_SendACK(CMD_ARM);
        } else {
            Telemetry_SendNACK(CMD_ARM, "bad_state", 103);
        }
    } else if (cmd_exact(line, CMD_START_BURST_WEIGHT)) {
        if (s_ctx->op_mode == OP_MODE_IDLE)
            Parse_StartBurstWeight(line);
        else
            Telemetry_SendNACK(CMD_START_BURST_WEIGHT, "bad_state", 103);
    } else if (cmd_exact(line, CMD_START_BURST_DAMPING)) {
        if (s_ctx->op_mode == OP_MODE_IDLE)
            Parse_StartBurstDamping(line);
        else
            Telemetry_SendNACK(CMD_START_BURST_DAMPING, "bad_state", 103);
    } else if (cmd_exact(line, CMD_GET_PREVIEW)) {
        if (s_ctx->op_mode != OP_MODE_IDLE) {
            Telemetry_SendNACK(CMD_GET_PREVIEW, "bad_state", 103);
        } else {
            if (!s_ctx->is_dumping) {
                s_ctx->is_dumping = true;
                s_ctx->diag.hb_pauses++;
            }
            Telemetry_SendPreview(s_ctx);
        }
    } else if (cmd_exact(line, CMD_STOP)) {
        bool force = (strstr(line, ",FORCE") != NULL);
        if (s_ctx->op_mode == OP_MODE_ARMED && s_ctx->trg_state == TRG_STATE_ARMED && !force) {
            Telemetry_SendNACK(CMD_STOP, "blocked_while_armed", 201);
        } else {
            s_ctx->stop_flag = true;
        }
    } else if (cmd_exact(line, CMD_GET_DIAG)) {
        Telemetry_SendDiag(s_ctx);
    } else if (cmd_exact(line, CMD_REBOOT)) {
        Telemetry_SendACK(CMD_REBOOT);
        HAL_Delay(100);
        HAL_NVIC_SystemReset();
    } else if (cmd_exact(line, CMD_ZERO)) {
        if (s_ctx->op_mode == OP_MODE_IDLE) {
            Telemetry_SendACK(CMD_ZERO);
            Telemetry_Flush();
            Sensor_StartSampling(s_ctx);
            Trigger_PerformQuickZero(s_ctx);
            Sensor_StopSampling(s_ctx);
            COMM_Sendf(MSG_CAL_INFO ",status=zero_complete" PROTO_EOL);
        } else {
            Telemetry_SendNACK(CMD_ZERO, "bad_state", 103);
        }
    } else if (cmd_exact(line, CMD_TEST_FORCE_TRIGGER)) {
        if (s_ctx->op_mode == OP_MODE_ARMED) {
            Telemetry_SendACK(CMD_TEST_FORCE_TRIGGER);
            s_ctx->test_trigger_flag = true;
        } else {
            Telemetry_SendNACK(CMD_TEST_FORCE_TRIGGER, "bad_state", 103);
        }
    } else if (cmd_exact(line, "ADXL_ST")) {
        if (s_ctx->op_mode == OP_MODE_IDLE) {
            Parse_AdxlSt(line);
        } else {
            Telemetry_SendNACK("ADXL_ST", "bad_state", 103);
        }
    } else if (cmd_exact(line, "DIAG_HW_TEST")) { // NYTT DIAGNOSTIKKOMMANDO
        handle_diag_hw_test(s_ctx, line);
    } else {
        Telemetry_SendNACK("UNKNOWN", "unknown_command", 100);
    }

    g_is_processing_command = false;
}

// --- Static Helper Functions (moved from main.c) ---

static inline int cmd_exact(const char *line, const char *cmd) {
    size_t n = strlen(cmd);
    return strncmp(line, cmd, n) == 0 &&
           (line[n] == '\0' || line[n] == ' ' || line[n] == ',');
}

static void Parse_SetCfg(const char *line) {
    const char *p = line;
    char *q;
    uint32_t req_odr_hz = s_ctx->cfg.odr_hz;
    uint32_t new_burst_ms = s_ctx->cfg.burst_ms;
    uint32_t new_hb_ms = s_ctx->cfg.hb_ms;
    uint32_t new_stream_rate = s_ctx->cfg.stream_rate_hz;

    uint32_t tmp32;
    if ((q = strstr(p, "odr_hz=")) && api_parse_u32(q + 7, &tmp32)) {
        req_odr_hz = tmp32;
    }
    if ((q = strstr(p, "burst_ms=")) && api_parse_u32(q + 9, &tmp32)) {
        new_burst_ms = tmp32;
    }
    if ((q = strstr(p, "hb_ms=")) && api_parse_u32(q + 6, &tmp32)) {
        new_hb_ms = tmp32;
    }
    if ((q = strstr(p, "stream_rate_hz=")) && api_parse_u32(q + 15, &tmp32)) {
        new_stream_rate = tmp32;
    }

    uint32_t eff_odr = Sensor_SnapODR(req_odr_hz);
    if (new_burst_ms == 0U || new_burst_ms > 600000U) {
        Telemetry_SendNACK(CMD_SET_CFG, "param_range", 102);
        return;
    }
    if (new_hb_ms > 0U && new_hb_ms < 100U) {
        Telemetry_SendNACK(CMD_SET_CFG, "param_range", 102);
        return;
    }
    if (new_stream_rate > eff_odr) {
        Telemetry_SendNACK(CMD_SET_CFG, "param_range", 102);
        return;
    }
    if (new_stream_rate > 0 && (eff_odr % new_stream_rate) != 0) {
        Telemetry_SendNACK(CMD_SET_CFG, "param_range", 102);
        return;
    }

    uint32_t old_odr = s_ctx->cfg.odr_hz;
    s_ctx->cfg.burst_ms = new_burst_ms;
    s_ctx->cfg.hb_ms = new_hb_ms;
    s_ctx->cfg.stream_rate_hz = new_stream_rate;
    s_ctx->cfg.odr_hz = eff_odr;

    if (old_odr != eff_odr) {
        bool was_sampling = Sensor_IsSampling(s_ctx);
        if (was_sampling) {
            Sensor_StopSampling(s_ctx);
        }

        Sensor_SetODR(s_ctx, eff_odr);
        Sensor_ReconfigureTimer(s_ctx, eff_odr);

        if (was_sampling) {
            Sensor_StartSampling(s_ctx);
        }
        Streaming_Reconfigure(s_ctx);
    }

    Streaming_UpdateDivider(s_ctx);
    Telemetry_SendACK(CMD_SET_CFG);
}

static void Parse_SetTrg(const char *line) {
    const char *p = line;
    char *q;

    TriggerSettings_t ns = s_ctx->trigger_settings;
    float ftmp;
    uint32_t utmp;

    if ((q = strstr(p, "k_mult=")) && api_parse_float_fixed3(q + 7, &ftmp)) {
        ns.k_mult = ftmp;
    }
    if ((q = strstr(p, "win_ms=")) && api_parse_u32(q + 7, &utmp)) {
        ns.win_ms = utmp;
    }
    if ((q = strstr(p, "hold_ms=")) && api_parse_u32(q + 8, &utmp)) {
        ns.hold_ms = utmp;
    }

    bool valid = (ns.hold_ms >= 100U && ns.hold_ms <= 10000U);

    if (!valid) {
        Telemetry_SendNACK(CMD_SET_TRG, "param_range", 102);
        return;
    }

    s_ctx->trigger_settings = ns;
    Telemetry_SendACK(CMD_SET_TRG);
}

static void Parse_TimeSync(const char *line) {
    const char *q = strstr(line, "host_ms=");
    if (q) {
        uint64_t host_ms = 0;
        if (api_parse_u64(q + 8, &host_ms)) {
            s_ctx->tsync.has_sync = true;
            s_ctx->tsync.host_ms_at_sync = host_ms;
            s_ctx->tsync.tick_at_sync = __HAL_TIM_GET_COUNTER(s_ctx->htim2);
            Telemetry_SendACK(CMD_TIME_SYNC);
            return;
        }
    }
    Telemetry_SendNACK(CMD_TIME_SYNC, "bad_arg", 101);
}

static void Parse_StreamStart(const char *line) {
    if (s_ctx->op_mode != OP_MODE_IDLE) {
        Telemetry_SendNACK(CMD_STREAM_START, "bad_state", 103);
        return;
    }
    Streaming_Start(s_ctx);
    Telemetry_SendStreamStartACK(s_ctx);
}

static void Parse_StartBurstWeight(const char *line) {
    uint32_t cycles = 0;
    const char *p = strstr(line, "cycles=");
    if (!p || !api_parse_u32(p + 7, &cycles)) {
        Telemetry_SendNACK(CMD_START_BURST_WEIGHT, "bad_arg", 101);
        return;
    }
    if (cycles == 0 || cycles > 1024) {
        Telemetry_SendNACK(CMD_START_BURST_WEIGHT, "param_range", 102);
        return;
    }

    Telemetry_SendACK(CMD_START_BURST_WEIGHT);
    BurstManager_Configure(s_ctx, KIND_WEIGHT, 0, cycles);
    AppContext_SetOpMode(s_ctx, OP_MODE_COUNTDOWN);
    Countdown_Start(5); // Using default 5s for now
}

static void Parse_StartBurstDamping(const char *line) {
    uint32_t seconds = 0;
    const char *p = strstr(line, "seconds=");
    if (!p || !api_parse_u32(p + 8, &seconds)) {
        Telemetry_SendNACK(CMD_START_BURST_DAMPING, "bad_arg", 101);
        return;
    }
    if (seconds == 0 || seconds > 600) {
        Telemetry_SendNACK(CMD_START_BURST_DAMPING, "param_range", 102);
        return;
    }

    Telemetry_SendACK(CMD_START_BURST_DAMPING);
    BurstManager_Configure(s_ctx, KIND_DAMP_CD, seconds, 0);
    AppContext_SetOpMode(s_ctx, OP_MODE_COUNTDOWN);
    Countdown_Start(5); // Using default 5s for now
}

static void Parse_Mode(const char *line) {
    if (strstr(line, "TRIGGER_ON")) {
        if (s_ctx->op_mode != OP_MODE_IDLE) {
            Telemetry_SendNACK(CMD_MODE, "bad_state", 103);
            return;
        }
        Streaming_Stop(s_ctx);

        const char *p = strstr(line, "cd_s=");
        if (p) {
            uint32_t val = 0;
            // This parameter is parsed but not used, as the countdown is started
            // in Parse_CalReady with a default value. This maintains API compatibility.
            if (!api_parse_u32(p + 5, &val) || val < 5 || val > 10) {
                Telemetry_SendNACK(CMD_MODE, "param_range", 102);
                return;
            }
        }

        Telemetry_SendACK(CMD_MODE);
        Telemetry_SendCalInfo(s_ctx);
        s_ctx->is_dumping = true;
        s_ctx->diag.hb_pauses++;
        Sensor_StartSampling(s_ctx);
        AppContext_SetOpMode(s_ctx, OP_MODE_WAIT_CAL_ZERO);
    } else if (strstr(line, "TRIGGER_OFF")) {
        Telemetry_SendACK(CMD_MODE);
        Countdown_Stop();
        BurstManager_Reset(s_ctx);
        Trigger_Reset(s_ctx);
        if (s_ctx->op_mode != OP_MODE_IDLE)
            Sensor_StopSampling(s_ctx);
        AppContext_SetOpMode(s_ctx, OP_MODE_IDLE);
    } else {
        Telemetry_SendNACK(CMD_MODE, "bad_arg", 101);
    }
}

static void Parse_CalReady(const char *line) {
    if (s_ctx->op_mode != OP_MODE_WAIT_CAL_ZERO) {
        Telemetry_SendNACK(CMD_CAL_READY, "bad_state", 103);
        return;
    }
    const char *p = strstr(line, "phase=");
    if (!p || strncmp(p + 6, "hold_zero", 9) != 0) {
        Telemetry_SendNACK(CMD_CAL_READY, "bad_arg", 101);
        return;
    }

    Telemetry_SendACK(CMD_CAL_READY);
    s_ctx->is_dumping = true;
    s_ctx->diag.hb_pauses++;
    AppContext_SetOpMode(s_ctx, OP_MODE_TRG_CAL_ZERO);
    if (!Countdown_IsActive()) {
        Countdown_Start(5); // Default countdown
    }
}

static void Parse_HB(const char *line) {
    if (strstr(line, "OFF")) {
        s_ctx->cfg.hb_ms = 0;
        Telemetry_SendACK(CMD_HB);
        return;
    }
    if (strstr(line, "ON")) {
        if (s_ctx->cfg.hb_ms == 0)
            s_ctx->cfg.hb_ms = 1000;
        Telemetry_SendACK(CMD_HB);
        return;
    }
    const char *p = strstr(line, "ms=");
    if (p) {
        uint32_t v = 0;
        if (api_parse_u32(p + 3, &v)) {
            if (v > 0 && v < 100) v = 100;
            s_ctx->cfg.hb_ms = v;
            Telemetry_SendACK(CMD_HB);
            return;
        }
    }
    Telemetry_SendNACK(CMD_HB, "bad_arg", 101);
}

static void Parse_AdxlSt(const char *line) {
    uint32_t avg_count = 16;
    uint32_t settle_count = 4;
    uint32_t force_odr_hz = 0;
    char* q;

    if ((q = strstr(line, "avg=")) && api_parse_u32(q + 4, &avg_count)) {
        // parsed
    }
    if ((q = strstr(line, "settle=")) && api_parse_u32(q + 7, &settle_count)) {
        // parsed
    }
    if ((q = strstr(line, "force_odr_hz=")) && api_parse_u32(q + 13, &force_odr_hz)) {
        // parsed
    }

    if (avg_count == 0 || avg_count > 128) {
        Telemetry_SendNACK("ADXL_ST", "param_range", 102);
        return;
    }
    if (settle_count > 32) {
        Telemetry_SendNACK("ADXL_ST", "param_range", 102);
        return;
    }

    Telemetry_SendACK("ADXL_ST");
    Telemetry_Flush();

    AdxlSelfTestResult_t results;
    HAL_StatusTypeDef status = Sensor_PerformSelfTest(s_ctx, (uint8_t)avg_count, (uint8_t)settle_count, force_odr_hz, &results);

    if (status == HAL_OK) {
        uint32_t test_odr = (force_odr_hz > 0) ? force_odr_hz : 400; // Match default in sensor_hal
        uint32_t snapped_odr = Sensor_SnapODR(test_odr);
        uint8_t devid = 0xE5; // Known device ID

        COMM_Sendf("ADXL_ST_CFG,devid=0x%02X,odr_hz=%lu,avg=%lu,settle=%lu" PROTO_EOL,
                   devid, snapped_odr, avg_count, settle_count);

        COMM_Sendf("ADXL_ST_RAW,x_off=%d,y_off=%d,z_off=%d,x_on=%d,y_on=%d,z_on=%d,x_st=%d,y_st=%d,z_st=%d,health=%s" PROTO_EOL,
                   results.x_off, results.y_off, results.z_off,
                   results.x_on, results.y_on, results.z_on,
                   results.x_st, results.y_st, results.z_st,
                   results.health_pass ? "PASS" : "FAIL");
    } else {
        const char* reason = (status == HAL_TIMEOUT) ? "sensor_timeout" : "i2c_error";
        COMM_Sendf("ADXL_ST_RAW,health=%s" PROTO_EOL, reason);
    }
}

// --- NY KOMMANDOHANTERARE ---
static void handle_diag_hw_test(AppContext_t* ctx, const char* line) {
    (void)line;
    if (ctx->op_mode != OP_MODE_IDLE && ctx->op_mode != OP_MODE_WAIT_ARM) {
        Telemetry_SendNACK("DIAG_HW_TEST", "must_be_idle_or_wait_arm", 103);
        return;
    }
    Telemetry_SendACK("DIAG_HW_TEST");
    DevDiag_RunAllTests(ctx);
}