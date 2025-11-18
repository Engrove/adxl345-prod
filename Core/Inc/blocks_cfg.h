/* filename: Core/Inc/blocks_cfg.h */
#ifndef BLOCKS_CFG_H_
#define BLOCKS_CFG_H_

#include <stdint.h>
#include "api_schema.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t window;
    uint16_t lines;
    uint8_t  retries;
} BlocksCfg_t;

void        BlocksCfg_Init(uint16_t def_window, uint16_t def_lines, uint8_t def_retries);
BlocksCfg_t BlocksCfg_Get(void);
/* Returnerar 1 vid OK, 0 vid valideringsfel. Tillämpar direkt på BLOCKS-transporten. */
int         BlocksCfg_Set(uint16_t window, uint16_t lines, uint8_t retries);

#ifdef __cplusplus
}
#endif

#endif /* BLOCKS_CFG_H_ */
/* filename: Core/Inc/blocks_cfg.h */