/* PATCH SUMMARY:
- Task: Align the firmware's protocol constants with the normative API specification.
- Root cause addressed: The PROTO_BLOCK_LINES_DEFAULT value (32) did not match the value advertised in the normative HELLO_ACK example (128) in the specification.
- Fix approach: Changed the `#define` for `PROTO_BLOCK_LINES_DEFAULT` from `32u` to `128u`.
- Spec traceability: This change aligns the firmware constant with the example provided in API specification §10 (Handshake).
*/
/* Core/Inc/api_schema.h */
#ifndef API_SCHEMA_H_
#define API_SCHEMA_H_

/*
 * api_schema.h
 * Version: 3.3.2
 * Date: 2025-10-20 14:42:42+03:00 [Europe/Helsinki]
 *
 * Purpose: Centralized definition of the normalized, key-value ASCII API protocol.
 * This file provides the canonical tokens and constants for firmware implementation.
 *
 * Version History:
 * v3.3.2 | MrPerfect | MAJOR: Replaced edge-based trigger with variance-based method. Removed CAL_ARM/VT_ACK states and commands.
 * |      |           | - DEPRECATED: SET_TRG now uses k_mult/win_ms, not thresh_ms2/hyst_ms2/axis.
 * |      |           | - REMOVED: Obsolete commands CMD_VT_ACK, CMD_VT_DEC and message MSG_VT_REQ.
 * v3.3.0 | Frankensteen | Initial version for BLOCKS transport, FSM, and guided calibration.
 * v3.0.x | Frankensteen | Various updates for v3.0.x API compliance.
 */

#ifdef __cplusplus
extern "C" {
#endif

// --- Protocol Version ---
/* 2025-10-13 13:00:00+03:00 [Europe/Helsinki] */
#define PROTO_VERSION_MAJOR 3
#define PROTO_VERSION_MINOR 3
#define PROTO_VERSION_PATCH 3
#define PROTO_VERSION_STR "3.3.3"

// --- Protocol Structural Constants ---
#define PROTO_DELIM ','         // Field delimiter for positional data
#define PROTO_KV_SEP '='          // Key-Value separator
#ifndef PROTO_EOL
#define PROTO_EOL "\r\n"        // End of Line sequence
#endif
#ifndef PROTO_EOL_LEN
#define PROTO_EOL_LEN 2         // Length of EOL string
#endif
#define PROTO_MAX_LINE 256      // Maximum command/message line length, including EOL

/* --- v3.3.0 BLOCKS/CRC constants --- */
#ifndef PROTO_BLOCK_TIMEOUT_MS
#define PROTO_BLOCK_TIMEOUT_MS 1000u   /* ACK/NACK timeout per block */
#endif
#ifndef PROTO_MAX_RETRIES
#define PROTO_MAX_RETRIES 3u          /* Retries per block on NACK/timeout */
#endif
#ifndef PROTO_WINDOW_DEFAULT
#define PROTO_WINDOW_DEFAULT 4u       /* Default in-flight block window */
#endif
#ifndef PROTO_BLOCK_LINES_DEFAULT
#define PROTO_BLOCK_LINES_DEFAULT 128u /* Default DATA lines per block */
#endif
#ifndef PROTO_CRC16_POLY
#define PROTO_CRC16_POLY 0x1021u      /* CCITT-FALSE polynomial */
#endif
#ifndef PROTO_CRC16_INIT
#define PROTO_CRC16_INIT 0xFFFFu      /* CCITT-FALSE initial value */
#endif

/* API v3.3.0 ABNF-konform inputpolicy */
#ifndef PROTO_FLOAT_DEC_MAX
#define PROTO_FLOAT_DEC_MAX 3   /* högst tre decimaler i indata */
#endif
#ifndef PROTO_INPUT_FORBID_SCI
#define PROTO_INPUT_FORBID_SCI 1 /* vetenskaplig notation förbjuden i indata */
#endif

/*
 * Implementation Requirement:
 * The C translation unit(s) responsible for formatting and parsing lines based on this
 * schema MUST include compile-time assertions to guarantee buffer safety.
 * Specifically, it must verify:
 * - _Static_assert(PROTO_EOL_LEN == 2, "EOL length assumption is invalid.");
 * - _Static_assert(PROTO_MAX_LINE >= 256, "Line buffer is smaller than spec requirement.");
 * - That all string formatting buffers are sized to at least PROTO_MAX_LINE.
 * - That all formatters explicitly use the PROTO_EOL constant for termination.
 */

// --- MCU -> PC Message Prefixes ---
#define MSG_HELLO_ACK       "HELLO_ACK"
#define MSG_ACK             "ACK"
#define MSG_NACK            "NACK"
#define MSG_ERROR           "ERROR"
#define MSG_STATUS          "STATUS"
#define MSG_CFG             "CFG"
#define MSG_HB              "HB" // Format: HB,tick=<u32>[,host_hi=<u32>,host_lo=<u32>],tx_free=<u>,tx_drop=<u32>
#define MSG_TRG_SETTINGS    "TRG_SETTINGS"
#define MSG_TRIGGER_EDGE    "TRIGGER_EDGE"
#define MSG_DATA_HEADER     "DATA_HEADER"
#define MSG_DATA            "DATA"
#define MSG_COMPLETE        "COMPLETE"
#define MSG_COUNTDOWN_ID    "COUNTDOWN_ID"
#define MSG_CAL_ACTION      "CAL_ACTION"
#define MSG_PREVIEW_HEADER  "PREVIEW_HEADER"
#define MSG_PREVIEW         "PREVIEW"
#define MSG_PREVIEW_END     "PREVIEW_END"
#define MSG_LIVE            "LIVE"
#define MSG_SUMMARY         "SUMMARY"
#define MSG_CAL_INFO        "CAL_INFO"
#define MSG_USER_BTN        "USER_BTN"

// --- PC -> MCU Command Prefixes ---
#define CMD_HELLO               "HELLO"
#define CMD_GET_STATUS          "GET_STATUS"
#define CMD_GET_CFG             "GET_CFG"
#define CMD_SET_CFG             "SET_CFG"
#define CMD_HB                  "HB"         // HB,OFF | HB,ON | HB,ms=<u32>
#define CMD_TIME_SYNC           "TIME_SYNC" // Format: TIME_SYNC,host_ms=<u64>
#define CMD_STREAM_START        "STREAM_START"
#define CMD_STREAM_STOP         "STREAM_STOP"
#define CMD_GET_TRG             "GET_TRG"
#define CMD_SET_TRG             "SET_TRG"
#define CMD_MODE                "MODE"
#define CMD_CAL_READY           "CAL_READY"  // CAL_READY,phase=<hold_zero|hold_arm>
#define CMD_ARM                 "ARM"
#define CMD_START_BURST_WEIGHT  "START_BURST_WEIGHT"
#define CMD_START_BURST_DAMPING "START_BURST_DAMPING"
#define CMD_GET_PREVIEW         "GET_PREVIEW"
#define CMD_GET_DIAG            "GET_DIAG"
#define CMD_REBOOT              "REBOOT"
#define CMD_STOP                "STOP"
#define CMD_ZERO                "ZERO"
#define CMD_TEST_FORCE_TRIGGER  "_TEST_FORCE_TRIGGER"

/*
 * NOTE on acknowledgements:
 * - "ACK": Sent from MCU to PC to confirm receipt and acceptance of a command.
 * - PC does not send a generic "ACK" command. Responses from PC to an MCU "REQ"
 * are context-specific and defined by the sequence (e.g., VT_ACK, VT_DEC).
 */


#ifdef __cplusplus
}
#endif

#endif /* API_SCHEMA_H_ */
