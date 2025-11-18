/* filename: Core/Src/blocks_cfg.c */
#include "blocks_cfg.h"
#include "transport_blocks.h"

static BlocksCfg_t g_bc;

void BlocksCfg_Init(uint16_t def_window, uint16_t def_lines, uint8_t def_retries)
{
    /* Clamp per API v3.3.0 (§16): window[1..8], lines[32..512], retries>=1 */
    if (def_window < 1u) def_window = 1u; else if (def_window > 8u)  def_window = 8u;
    if (def_lines  < 32u) def_lines  = 32u; else if (def_lines  > 512u) def_lines  = 512u;
    if (def_retries < 1u) def_retries = 1u;

    g_bc.window  = def_window;
    g_bc.lines   = def_lines;
    g_bc.retries = def_retries;
    TB_SetWindow(g_bc.window);
    TB_SetBlockLines(g_bc.lines);
    TB_SetMaxRetries(g_bc.retries);
}

BlocksCfg_t BlocksCfg_Get(void)
{
    return g_bc;
}

int BlocksCfg_Set(uint16_t window, uint16_t lines, uint8_t retries)
{
    /* Validate and clamp to safe protocol ranges. */
    if (window < 1u) window = 1u; else if (window > 8u)  window = 8u;
    if (lines  < 32u) lines  = 32u; else if (lines  > 512u) lines  = 512u;
    if (retries < 1u) retries = 1u;

    g_bc.window  = window;
    g_bc.lines   = lines;
    g_bc.retries = retries;

    TB_SetWindow(g_bc.window);
    TB_SetBlockLines(g_bc.lines);
    TB_SetMaxRetries(g_bc.retries);
    return 1;
}
/* filename: Core/Src/blocks_cfg.c */