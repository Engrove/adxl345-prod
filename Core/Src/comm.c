/*
 * Filename: Core/Src/comm.c
 * Version: 3.3.2-repatched
 *
 * ==============================================================================
 * ÄNDRINGSLOGG (jmf. med föregående patch)
 * ==============================================================================
 * 1. RC-201 (HÖG): Korrigerade en TOCTOU (Time-of-check to time-of-use) race
 *    condition i Telemetry_Write() och Telemetry_WriteBlocking().
 *
 *    - Kontrollen av ledigt utrymme i TX-ringbufferten (`_rb_free()`) har
 *      flyttats in i den kritiska sektionen (`__disable_irq()`) för att
 *      garantera att en preemption från en ISR inte kan fylla bufferten
 *      mellan kontrollen och den faktiska skrivningen.
 *
 *    - Detta eliminerar risken för buffer overflows och datakorruption när
 *      både huvudloopen och en ISR försöker sända data samtidigt under
 *      hög systemlast.
 *
 *    - Den blockerande varianten använder nu en säker loop-mekanism som
 *      släpper låset mellan försök, vilket förhindrar deadlock.
 * ==============================================================================
 */

#include "comm.h"
#include "usart.h"
#include "api_schema.h" // For PROTO_MAX_LINE
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <limits.h>
#include "dev_telemetry.h" // Inkludera ny debug-header

 // Public function prototypes from main.c
void process_command(char* line);

// --- Private Defines ---
#define UART_RX_DMA_BUFFER_SIZE 256
// RX_RING_BUFFER_SIZE är nu definierad i comm.h
#define LINE_BUFFER_SIZE PROTO_MAX_LINE // Synchronized with API schema

// Budget for non-blocking processing
#define COMM_MAX_LINES_PER_CALL   8
#define COMM_MAX_MS_PER_CALL      2

// --- Private Variables ---
static uint8_t uart_rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE];

static uint8_t rx_ring_buffer[RX_RING_BUFFER_SIZE];
static volatile uint16_t rx_ring_head = 0;
static volatile uint16_t rx_ring_tail = 0;
static volatile uint32_t s_rx_overflow_count = 0;

static uint8_t tx_ring_buffer[COMM_TX_RING_SIZE];
static volatile uint16_t tx_ring_head = 0;
static volatile uint16_t tx_ring_tail = 0;
static volatile uint8_t tx_dma_busy = 0;
static uint8_t tx_dma_buffer[COMM_TX_DMA_SIZE];
static volatile uint16_t tx_dma_active_len = 0;
static volatile uint32_t s_tx_drop_count = 0;

static char line_buffer[LINE_BUFFER_SIZE];
static uint16_t line_len = 0;
static uint8_t line_truncated = 0; /* flag for over-long line */

// --- Compile-time checks ---
#ifdef PROTO_EOL_LEN
_Static_assert(LINE_BUFFER_SIZE >= (PROTO_MAX_LINE - PROTO_EOL_LEN + 1),
               "line_buffer must be able to hold the max payload plus a NUL terminator");
#else
_Static_assert(LINE_BUFFER_SIZE >= PROTO_MAX_LINE,
               "line_buffer must be at least PROTO_MAX_LINE when EOL length is unknown");
#endif
_Static_assert(COMM_TX_DMA_SIZE <= COMM_TX_RING_SIZE,
               "COMM_TX_DMA_SIZE must not exceed COMM_TX_RING_SIZE");
_Static_assert(COMM_TX_DMA_SIZE >= PROTO_MAX_LINE,
               "DMA chunk must cover one full protocol line");
_Static_assert(COMM_TX_RING_SIZE >= (PROTO_MAX_LINE * 4),
               "TX ring should hold at least 4 protocol lines for burst sequences");


// --- Private Function Prototypes ---
static void StartDmaTx(void);
static inline uint16_t _rb_free(void);
static inline uint16_t _tx_rb_usage(void);
static inline uint16_t _rx_rb_usage(void);


// --- Public Functions ---

void COMM_Init(void)
{
    rx_ring_head = 0;
    rx_ring_tail = 0;
    tx_ring_head = 0;
    tx_ring_tail = 0;
    tx_dma_busy = 0;
    s_rx_overflow_count = 0;
    s_tx_drop_count = 0;
    tx_dma_active_len = 0;
    line_len = 0;
    line_truncated = 0;
}

void COMM_StartRx(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE);
}

void COMM_Process(void)
{
    while (rx_ring_head != rx_ring_tail)
    {
        char c = rx_ring_buffer[rx_ring_tail];
        rx_ring_tail = (rx_ring_tail + 1) % RX_RING_BUFFER_SIZE;

        if (c == '\r' || c == '\n')
        {
            if (line_len > 0)
            {
                line_buffer[line_len] = '\0';
                process_command(line_buffer);
            }
            else if (line_truncated)
            {
                COMM_Sendf(MSG_NACK ",SUBJECT=UNKNOWN,reason=line_too_long,code=%u" PROTO_EOL, 300U);
            }
            line_len = 0;
            line_truncated = 0;
        }
        else if (!line_truncated)
        {
            if (line_len < (LINE_BUFFER_SIZE - 1)) {
                line_buffer[line_len++] = c;
            }
            else {
                line_truncated = 1;
                line_len = 0;
            }
        }
    }
}

void COMM_Process_Budgeted(void)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t lines_processed = 0;

    while (rx_ring_head != rx_ring_tail)
    {
        char c = rx_ring_buffer[rx_ring_tail];
        rx_ring_tail = (rx_ring_tail + 1) % RX_RING_BUFFER_SIZE;

        if (c == '\r' || c == '\n')
        {
            if (line_len > 0)
            {
                line_buffer[line_len] = '\0';
                process_command(line_buffer);
                lines_processed++;
            }
            else if (line_truncated)
            {
                COMM_Sendf(MSG_NACK ",SUBJECT=UNKNOWN,reason=line_too_long,code=%u" PROTO_EOL, 300U);
            }
            line_len = 0;
            line_truncated = 0;
        }
        else if (!line_truncated)
        {
            if (line_len < (LINE_BUFFER_SIZE - 1)) {
                line_buffer[line_len++] = c;
            }
            else
            {
                line_truncated = 1;
                line_len = 0;
            }
        }

        if (lines_processed >= COMM_MAX_LINES_PER_CALL) {
            break;
        }
        if ((HAL_GetTick() - start_ms) >= COMM_MAX_MS_PER_CALL) {
            break;
        }
    }
}

void COMM_Send(const char* s)
{
    if (s == NULL) return;
    size_t len = strlen(s);
    if (len > 0) {
        Telemetry_Write(s, len);
    }
}

int COMM_Sendf(const char* fmt, ...)
{
    char temp_buf[PROTO_MAX_LINE + 1];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(temp_buf, sizeof(temp_buf), fmt, args);
    va_end(args);

    if (len <= 0) { return 0; }
    if (len >= (int)sizeof(temp_buf)) {
        __disable_irq();
        uint32_t drops = (uint32_t)len;
        if (s_tx_drop_count <= UINT32_MAX - drops) { s_tx_drop_count += drops; }
        else { s_tx_drop_count = UINT32_MAX; }
        __enable_irq();
        return 0;
    }

    if (Telemetry_Write(temp_buf, (size_t)len) == (size_t)len) {
        return len;
    }

    return 0;
}

/* PATCH RC-201: Hela funktionen är omstrukturerad för att flytta
 * friutrymmes-kontrollen in i den kritiska sektionen. */
size_t Telemetry_Write(const char* data, size_t len)
{
    if (!data || !len) return 0;

    uint8_t start_tx = 0;
    bool dropped = false;

    __disable_irq();
    uint16_t free_space = _rb_free(); // Kontrollera utrymme under lås

    if (len > free_space) {
        // Räkna tappade bytes och släpp låset
        if (s_tx_drop_count <= UINT32_MAX - (uint32_t)len) {
            s_tx_drop_count += (uint32_t)len;
        } else {
            s_tx_drop_count = UINT32_MAX;
        }
        __enable_irq();
        dropped = true;
    } else {
        // Det finns plats, fortsätt under lås
        uint16_t head = tx_ring_head;
        uint16_t first_chunk_len = COMM_TX_RING_SIZE - head;
        if (len < first_chunk_len) {
            first_chunk_len = len;
        }
        memcpy(&tx_ring_buffer[head], data, first_chunk_len);
        head = (head + first_chunk_len) % COMM_TX_RING_SIZE;

        size_t remaining_len = len - first_chunk_len;
        if (remaining_len > 0) {
            memcpy(&tx_ring_buffer[0], data + first_chunk_len, remaining_len);
            head = (head + remaining_len);
        }
        tx_ring_head = head;
        if (!tx_dma_busy) { start_tx = 1; }
        __enable_irq(); // Släpp låset
    }

    #if RXTX_DEBUG > 0
    // Anropa debug-telemetri utanför låset
    DevTel_TxEnqueue(len, free_space, dropped);
    #endif

    if (dropped) {
        return 0;
    }

    if (start_tx) {
        StartDmaTx();
    }

    return len;
}

/* PATCH RC-201: Implementerar en säker blockerande väntan som inte
 * håller låset medan den väntar. */
size_t Telemetry_WriteBlocking(const char* data, size_t len)
{
    if (!data || !len) return 0;
    uint8_t start_tx = 0;

    for (;;) {
        __disable_irq();
        if (_rb_free() >= len) {
            // Det finns plats, låset är aktivt, bryt loopen för att skriva
            break;
        }
        // Släpp låset och loopa igen för att låta ISR:er köra
        __enable_irq();
    }

    // Nu är vi inne i den kritiska sektionen med garanterat utrymme
    uint16_t head = tx_ring_head;
    size_t first_chunk_len = (size_t)(COMM_TX_RING_SIZE - head);
    if (len < first_chunk_len) { first_chunk_len = len; }
    memcpy(&tx_ring_buffer[head], data, first_chunk_len);
    head = (uint16_t)((head + first_chunk_len) % COMM_TX_RING_SIZE);
    
    size_t remaining_len = len - first_chunk_len;
    if (remaining_len > 0) {
        memcpy(&tx_ring_buffer[0], data + first_chunk_len, remaining_len);
        head = (uint16_t)(remaining_len);
    }
    tx_ring_head = head;
    if (!tx_dma_busy) { start_tx = 1; }
    __enable_irq(); // Släpp låset

    if (start_tx) { StartDmaTx(); }
    return len;
}


int COMM_SendfBlocking(const char* fmt, ...)
{
    char temp_buf[PROTO_MAX_LINE + 1];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(temp_buf, sizeof(temp_buf), fmt, args);
    va_end(args);
    if (len <= 0 || len >= (int)sizeof(temp_buf)) return 0;
    
    return (int)Telemetry_WriteBlocking(temp_buf, (size_t)len);
}


// --- Private Functions ---

static inline uint16_t _rb_free(void)
{
    uint16_t head = tx_ring_head;
    uint16_t tail = tx_ring_tail;
    if (head >= tail) {
        return (COMM_TX_RING_SIZE - 1) - (head - tail);
    }
    return tail - head - 1;
}

static void StartDmaTx(void)
{
    __disable_irq();

    if (tx_dma_busy) {
        __enable_irq();
        return;
    }

    if (tx_ring_head == tx_ring_tail) {
        tx_dma_active_len = 0;
        __enable_irq();
        return;
    }

    tx_dma_busy = 1;

    uint16_t len;
    if (tx_ring_head > tx_ring_tail) {
        len = tx_ring_head - tx_ring_tail;
    }
    else {
        len = COMM_TX_RING_SIZE - tx_ring_tail;
    }

    if (len > COMM_TX_DMA_SIZE) {
        len = COMM_TX_DMA_SIZE;
    }

    memcpy(tx_dma_buffer, &tx_ring_buffer[tx_ring_tail], len);
    tx_dma_active_len = len;

    __enable_irq();
        
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart2, tx_dma_buffer, len);
    if (st != HAL_OK) {
        __disable_irq();
        tx_dma_busy = 0;
        tx_dma_active_len = 0;
        __enable_irq();
    }
}

// --- HAL Callback Redirection ---

void COMM_OnRxEvent(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        uint16_t local_head = rx_ring_head;
        uint16_t local_tail;
        __disable_irq();
        local_tail = rx_ring_tail;
        __enable_irq();

        for (uint16_t i = 0; i < Size; i++)
        {
            uint16_t next_head = (local_head + 1) % RX_RING_BUFFER_SIZE;
            if (next_head != local_tail)
            {
                rx_ring_buffer[local_head] = uart_rx_dma_buffer[i];
                local_head = next_head;
            }
            else
            {
                s_rx_overflow_count++;
            }
        }
        rx_ring_head = local_head;
        
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE);
    }
}

void COMM_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance != USART2) { return; }
    
    uint16_t done;
    __disable_irq();
    done = tx_dma_active_len;
    if (done) {
        tx_ring_tail = (uint16_t)((tx_ring_tail + done) % COMM_TX_RING_SIZE);
    }
    tx_dma_active_len = 0;
    
    tx_dma_busy = 0;

    __enable_irq();
    
    StartDmaTx();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    COMM_OnRxEvent(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    COMM_TxCpltCallback(huart);
}

uint8_t COMM_TxIsIdle(void)
{
    uint8_t idle;
    __disable_irq();
    idle = (tx_dma_busy == 0) && (tx_ring_head == tx_ring_tail);
    __enable_irq();
    return idle;
}

uint16_t COMM_TxFree(void)
{
    uint16_t free_val;
    __disable_irq();
    free_val = _rb_free();
    __enable_irq();
    return free_val;
}

static inline uint16_t _tx_rb_usage(void) {
    uint16_t head = tx_ring_head;
    uint16_t tail = tx_ring_tail;
    if (head >= tail) return head - tail;
    return COMM_TX_RING_SIZE - (tail - head);
}

static inline uint16_t _rx_rb_usage(void) {
    uint16_t head = rx_ring_head;
    uint16_t tail = rx_ring_tail;
    if (head >= tail) return head - tail;
    return RX_RING_BUFFER_SIZE - (tail - head);
}

uint16_t COMM_TxRingUsage(void) {
    uint16_t usage;
    __disable_irq();
    usage = _tx_rb_usage();
    __enable_irq();
    return usage;
}

uint16_t COMM_RxRingUsage(void) {
    uint16_t usage;
    __disable_irq();
    usage = _rx_rb_usage();
    __enable_irq();
    return usage;
}

uint32_t COMM_TxDropCount(void) { return s_tx_drop_count; }
uint32_t COMM_RxOverflowCount(void) { return s_rx_overflow_count; }

void COMM_SendLine(const char* s)
{
    if (!s) return;
    COMM_Sendf("%s" PROTO_EOL, s);
}

int COMM_SendfLine(const char* fmt, ...)
{
    char buf[PROTO_MAX_LINE];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n <= 0) return 0;
    return COMM_Sendf("%s" PROTO_EOL, buf);
}
