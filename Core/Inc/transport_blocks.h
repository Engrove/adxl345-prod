/* filename: Core/Inc/transport_blocks.h */
#ifndef TRANSPORT_BLOCKS_H_
#define TRANSPORT_BLOCKS_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "api_schema.h"
#include "protocol_crc16.h"
#include "comm.h"
#include "dev_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* v3.3.0 BLOCKS-transport */
typedef int (*TB_GenLineFn)(uint16_t index, char* out, size_t out_sz, void* user);

void TB_Init(uint16_t window, uint16_t blk_lines, uint8_t max_retries);
void TB_SetWindow(uint16_t window);
void TB_SetBlockLines(uint16_t blk_lines);
void TB_SetMaxRetries(uint8_t max_retries);

void TB_BeginBurst(uint32_t burst_id);
void TB_EndBurst(void);

typedef struct {
    TB_GenLineFn gen;
    void*        user;
    uint16_t     lines;
} TB_BlockGen;

int  TB_EnqueueBlock(const TB_BlockGen* blk);
void TB_Pump(void);
int  TB_IsIdle(void);

void TB_OnAckBlk(uint16_t blk);
void TB_OnNackBlk(uint16_t blk, uint32_t code);
int  TB_HandleHostLine(const char* line);

uint8_t TB_GetQueueCount(void);
uint8_t TB_GetInflightCount(void);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_BLOCKS_H_ */
/* filename: Core/Inc/transport_blocks.h */
