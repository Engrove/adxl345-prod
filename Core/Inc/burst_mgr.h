/* filename: Core/Inc/burst_mgr.h */
#ifndef BURST_MGR_H_
#define BURST_MGR_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "transport_blocks.h"
#include "api_schema.h"
#include "comm.h"
#include "app_context.h"

#ifdef __cplusplus
extern "C" {
#endif

// This enum was missing from the header, causing build errors.
typedef enum {
    BM_TYPE_WEIGHT = 0,
    BM_TYPE_DAMP_TRG = 1,
    BM_TYPE_DAMP_CD  = 2
} BM_Type;

/* Init underlying BLOCKS-transport and burst manager state. */
void BM_Init(uint16_t window, uint16_t blk_lines, uint8_t max_retries);
void BurstManager_Init(AppContext_t* ctx);

/* Starta ny burst: sänder DATA_HEADER och primar BLOCKS. */
void BM_Begin(BM_Type type, uint32_t burst_id, uint32_t ts0_us, uint16_t samples, uint32_t odr_hz);
void BurstManager_Start(AppContext_t* ctx, DataKind_t kind, uint32_t burst_id, uint32_t duration_ms);

/* Köa block för aktuell burst. 'lines' måste vara <= konfigurerad blk_lines. */
int  BM_Enqueue(const TB_BlockGen* blk);

/* Anropas regelbundet i main-loop. Sköter sändning, timeouts och ACK/NACK. */
void BM_Pump(void);
void BurstManager_Pump(AppContext_t* ctx);

/* Hook för värdrader. Returnerar 1 om hanterad. Delger ACK_BLK/NACK_BLK och ACK_COMPLETE. */
int  BM_HandleHostLine(const char* line);

/* Avsluta bursten. COMPLETE skickas med reason=ok eller aborted. */
void BM_EndOk(void);
void BM_EndAborted(uint32_t code);

/* Tillståndsfrågor för gating av HB/LIVE och sekvensering. */
uint8_t BM_IsActive(void);
uint8_t BM_IsWaitingAckComplete(void);

/* New functions to manage state from other modules */
void BurstManager_Reset(AppContext_t* ctx);
void BurstManager_Configure(AppContext_t* ctx, DataKind_t kind, uint32_t seconds, uint32_t cycles);
uint32_t BurstManager_GetNextBurstId(AppContext_t* ctx);
DataKind_t BurstManager_GetCurrentKind(AppContext_t* ctx);


#ifdef __cplusplus
}
#endif

#endif /* BURST_MGR_H_ */
