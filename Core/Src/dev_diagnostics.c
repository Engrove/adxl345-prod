/* filename: Core/Src/dev_diagnostics.c */
#include "dev_diagnostics.h"
#include "comm.h"
#include "telemetry.h"
#include "api_schema.h"
#include "sensor_hal.h" // För Sensor_StartSampling, Sensor_StopSampling, sample_ring_head/tail, OCH I2CState_t
#include "gpio.h"       // För ADXL345_INT1_GPIO_Port, ADXL345_INT1_Pin
#include "tim.h"        // För TIM3_IRQn
#include "usart.h"      // För USART2_IRQn, DMA1_Stream6_IRQn
// #include "i2c.h"     // FIX: Borttagen, felaktig include. Definitionen finns i sensor_hal.h

#include <stdio.h>
#include <string.h>

// --- ADXL345 Register Constants (för felsökning) ---
#define ACCEL_SENSOR_ADDR (0x53 << 1)
#define ACCEL_REG_DEVID 0x00
#define ACCEL_REG_DATA_FORMAT 0x31
#define ACCEL_REG_INT_ENABLE  0x2E
#define ACCEL_REG_INT_MAP     0x2F
#define ACCEL_REG_INT_SOURCE 0x30
#define ACCEL_REG_BW_RATE     0x2C
#define ACCEL_REG_FIFO_CTL    0x38 // FIX: Lade till denna definition som saknades
#define ACCEL_REG_FIFO_STATUS 0x39
#define ACCEL_REG_POWER_CTL 0x2D

// --- Private Function Prototypes ---
static void Test_I2C_DevID(AppContext_t* ctx);
static void Test_ADXL_Configuration(AppContext_t* ctx); // Omdöpt och utökad
static void Test_EXTI_State(AppContext_t* ctx);
static void Test_DMA_State(AppContext_t* ctx);
static void Test_Sampling_Integrity(AppContext_t* ctx);
static void Test_NVIC_Priorities(AppContext_t* ctx); // NYTT
static void Diagnostics_Callback_Chain(AppContext_t* ctx); // NYTT (DEBUG)

// --- Public Main Test Runner ---\n
void DevDiag_RunAllTests(AppContext_t* ctx) {
    COMM_SendLine("DIAG_START,msg=\"Running low-level hardware diagnostics\"");
    
    Test_I2C_DevID(ctx);
    Test_ADXL_Configuration(ctx);
    Test_EXTI_State(ctx);
    Test_DMA_State(ctx);
    Test_NVIC_Priorities(ctx); // NYTT TEST
    Test_Sampling_Integrity(ctx);
    Diagnostics_Callback_Chain(ctx); // NYTT (DEBUG)

    COMM_SendLine("DIAG_END,msg=\"Diagnostics complete\"");
}

// --- Private Test Implementations ---

static void Test_I2C_DevID(AppContext_t* ctx) {
    uint8_t devid = 0;
    char val_str[16];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_DEVID, I2C_MEMADD_SIZE_8BIT, &devid, 1, 100);
    
    snprintf(val_str, sizeof(val_str), "0x%02X", devid);
    bool pass = (status == HAL_OK) && (devid == 0xE5);
    
    DIAG_SEND_RESULT("I2C_DEVID", "ADXL345 Device ID (0xE5)", val_str, pass);
    
    snprintf(val_str, sizeof(val_str), "%u", status);
    DIAG_SEND_RESULT("I2C_STATUS", "HAL_I2C_Mem_Read Status", val_str, status == HAL_OK);
}

static void Test_ADXL_Configuration(AppContext_t* ctx) {
    uint8_t reg_val[5];
    char val_str[32];
    HAL_StatusTypeDef status;
    
    // 1. DATA_FORMAT (0x31) - Förväntas vara 0x0B (FULL_RES=1, INT_INVERT=1, Range=16g)
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_DATA_FORMAT, I2C_MEMADD_SIZE_8BIT, &reg_val[0], 1, 100);
    snprintf(val_str, sizeof(val_str), "0x%02X", reg_val[0]);
    bool pass_df = (status == HAL_OK) && (reg_val[0] == 0x0B);
    DIAG_SEND_RESULT("ADXL_DF", "DATA_FORMAT (0x0B expected)", val_str, pass_df);

    // 2. BW_RATE (0x2C) - Förväntas vara 0x0D (800 Hz)
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_BW_RATE, I2C_MEMADD_SIZE_8BIT, &reg_val[1], 1, 100);
    snprintf(val_str, sizeof(val_str), "0x%02X", reg_val[1]);
    bool pass_br = (status == HAL_OK) && (reg_val[1] == 0x0D);
    DIAG_SEND_RESULT("ADXL_BR", "BW_RATE (0x0D for 800Hz expected)", val_str, pass_br);

    // 3. INT_ENABLE (0x2E) - Förväntas vara 0x02 (WATERMARK enabled)
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &reg_val[2], 1, 100);
    snprintf(val_str, sizeof(val_str), "0x%02X", reg_val[2]);
    bool pass_ie = (status == HAL_OK) && (reg_val[2] == 0x02);
    DIAG_SEND_RESULT("ADXL_IE", "INT_ENABLE (0x02 for WATERMARK expected)", val_str, pass_ie);

    // 4. FIFO_CTL (0x38) - Förväntas vara 0x9F (Stream Mode, Watermark=31)
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_FIFO_CTL, I2C_MEMADD_SIZE_8BIT, &reg_val[3], 1, 100);
    snprintf(val_str, sizeof(val_str), "0x%02X", reg_val[3]);
    bool pass_fc = (status == HAL_OK) && (reg_val[3] == 0x9F);
    DIAG_SEND_RESULT("ADXL_FC", "FIFO_CTL (0x9F for Stream/WM=31 expected)", val_str, pass_fc);
    
    // 5. POWER_CTL (0x2D) - Förväntas vara 0x08 (Measure=1)
    status = HAL_I2C_Mem_Read(ctx->hi2c1, ACCEL_SENSOR_ADDR, ACCEL_REG_POWER_CTL, I2C_MEMADD_SIZE_8BIT, &reg_val[4], 1, 100);
    snprintf(val_str, sizeof(val_str), "0x%02X", reg_val[4]);
    bool pass_pc = (status == HAL_OK) && ((reg_val[4] & 0x08) == 0x08);
    DIAG_SEND_RESULT("ADXL_PC", "POWER_CTL (Measure=1 expected)", val_str, pass_pc);
}

static void Test_EXTI_State(AppContext_t* ctx) {
    (void)ctx;
    // Läs GPIO-pinne (PA7) tillstånd. Förväntas vara HÖG (eftersom INT_INVERT=1 och interrupt rensad)
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(ADXL345_INT1_GPIO_Port, ADXL345_INT1_Pin);
    char val_str[16];
    snprintf(val_str, sizeof(val_str), "%s", (pin_state == GPIO_PIN_SET) ? "HIGH" : "LOW");
    
    // Efter initiering och rensning (Active Low), ska pinne vara HÖG
    bool pass = (pin_state == GPIO_PIN_SET);
    DIAG_SEND_RESULT("EXTI_PIN", "ADXL_INT1 Pin State (HIGH expected)", val_str, pass);
    
    // Kontrollera EXTI Pending Register (PR) för ADXL345_INT1_Pin (PA7 -> EXTI7)
    // EXTI->PR är 32-bitars register, bit 7 är för EXTI7
    uint32_t exti_pr = EXTI->PR;
    uint32_t exti_mask = (1U << 7);
    
    snprintf(val_str, sizeof(val_str), "0x%08lX", (unsigned long)exti_pr);
    bool pass_pr = (exti_pr & exti_mask) == 0;
    DIAG_SEND_RESULT("EXTI_PR", "EXTI PR (Bit 7 cleared)", val_str, pass_pr);
}

static void Test_DMA_State(AppContext_t* ctx) {
    char val_str[32];
    
    // Kontrollera I2C DMA RX-tillstånd (Stream 2/3)
    // Antar att hi2c1->hdmarx är det korrekta handtaget
    // OBS: hi2c1->hdmarx är inte definierad i i2c.h/i2c.c, men vi antar att det är länkat i HAL_I2C_MspInit
    if (ctx->hi2c1->hdmarx == NULL) {
        DIAG_SEND_RESULT("I2C_DMA_RX", "I2C DMA RX Handle", "NULL", false);
        return;
    }
    
    HAL_DMA_StateTypeDef dma_state = HAL_DMA_GetState(ctx->hi2c1->hdmarx);
    
    switch (dma_state) {
        case HAL_DMA_STATE_READY: snprintf(val_str, sizeof(val_str), "READY"); break;
        case HAL_DMA_STATE_BUSY: snprintf(val_str, sizeof(val_str), "BUSY"); break;
        case HAL_DMA_STATE_TIMEOUT: snprintf(val_str, sizeof(val_str), "TIMEOUT"); break;
        default: snprintf(val_str, sizeof(val_str), "OTHER (%u)", (unsigned int)dma_state); break;
    }
    
    // Förväntas vara READY i IDLE-läge
    bool pass = (dma_state == HAL_DMA_STATE_READY);
    DIAG_SEND_RESULT("I2C_DMA_RX", "I2C DMA RX State (READY expected)", val_str, pass);
}


static void Test_Sampling_Integrity(AppContext_t* ctx) {
    // Test: Starta sampling, vänta 100ms, kontrollera om ringbufferten har data
    
    // 1. Rensa ringbuffert och starta sampling
    Sensor_StartSampling(ctx);
    
    // 2. Vänta 100ms (borde ge 80 samples vid 800Hz)
    HAL_Delay(100);
    
    // 3. Stoppa sampling
    Sensor_StopSampling(ctx);
    
    // 4. Kontrollera ringbufferten
    uint16_t samples_in_rb = 0;
    __disable_irq();
    // sample_ring_head och sample_ring_tail är nu globala (ej static)
    if (sample_ring_head >= sample_ring_tail) {
        samples_in_rb = sample_ring_head - sample_ring_tail;
    } else {
        samples_in_rb = SAMPLE_RING_BUFFER_SIZE - (sample_ring_tail - sample_ring_head);
    }
    // Återställ ringbuffert
    sample_ring_head = 0;
    sample_ring_tail = 0;
    __enable_irq();
    
    char val_str[32];
    snprintf(val_str, sizeof(val_str), "%u", samples_in_rb);
    
    // Förväntas ha > 10 samples för att vara säker
    bool pass = (samples_in_rb > 10);
    DIAG_SEND_RESULT("RB_SAMPLES", "Samples in RingBuffer (Expected > 10)", val_str, pass);
}

static void Test_NVIC_Priorities(AppContext_t* ctx) {
    (void)ctx;
    char val_str[32];
    //bool overall_pass = true; // Kommenteras bort då den orsakade 'unused variable' varning
    
    // Hämta Priority Grouping för att kunna läsa ut prioriteterna korrekt
    uint32_t priority_group = HAL_NVIC_GetPriorityGrouping();
    uint32_t preempt_prio, sub_prio;
    
    // 1. TIM3_IRQn (Sampling Timer) - Förväntad prioritet: 5
    // HAL_NVIC_GetPriority kräver 4 argument i denna HAL-version.
    HAL_NVIC_GetPriority(TIM3_IRQn, priority_group, &preempt_prio, &sub_prio);
    uint32_t tim3_prio = preempt_prio; // Vi bryr oss bara om preemption priority
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)tim3_prio);
    bool pass_tim3 = (tim3_prio == 5);
    DIAG_SEND_RESULT("NVIC_TIM3", "TIM3 IRQ Priority (Expected 5)", val_str, pass_tim3);
    //if (!pass_tim3) overall_pass = false;

    // 2. USART2_IRQn (UART RX/TX) - Förväntad prioritet: 6
    HAL_NVIC_GetPriority(USART2_IRQn, priority_group, &preempt_prio, &sub_prio);
    uint32_t usart2_prio = preempt_prio;
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)usart2_prio);
    bool pass_usart2 = (usart2_prio == 6);
    DIAG_SEND_RESULT("NVIC_USART2", "USART2 IRQ Priority (Expected 6)", val_str, pass_usart2);
    //if (!pass_usart2) overall_pass = false;

    // 3. DMA1_Stream6_IRQn (UART TX DMA) - Förväntad prioritet: 6
    HAL_NVIC_GetPriority(DMA1_Stream6_IRQn, priority_group, &preempt_prio, &sub_prio);
    uint32_t dma_tx_prio = preempt_prio;
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)dma_tx_prio);
    bool pass_dma_tx = (dma_tx_prio == 6);
    DIAG_SEND_RESULT("NVIC_DMA_TX", "DMA1_Stream6 IRQ Priority (Expected 6)", val_str, pass_dma_tx);
    //if (!pass_dma_tx) overall_pass = false;
    
    // 4. Jämförelse: Sampling (5) ska vara högre prioritet än Kommunikation (6)
    snprintf(val_str, sizeof(val_str), "TIM3=%lu, COMM=%lu", (unsigned long)tim3_prio, (unsigned long)usart2_prio);
    bool pass_hierarchy = (tim3_prio < usart2_prio) && (tim3_prio < dma_tx_prio);
    DIAG_SEND_RESULT("NVIC_HIERARCHY", "TIM3 < COMM Priority", val_str, pass_hierarchy);
    
    // --- START: New critical tests (ISR-001) ---
    // CRITICAL: Verify ISR-001 deployment
    
    // 5. I2C1_EV_IRQn (I2C Event) - Förväntad prioritet: 3
    HAL_NVIC_GetPriority(I2C1_EV_IRQn, priority_group, &preempt_prio, &sub_prio);
    uint32_t prio_i2c_ev = preempt_prio;
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)prio_i2c_ev);
    bool pass_i2c_ev = (prio_i2c_ev == 3);
    DIAG_SEND_RESULT("NVIC_I2C_EV", "I2C1_EV IRQ Priority (Expected 3)", val_str, pass_i2c_ev);
    //if (!pass_i2c_ev) overall_pass = false;

    // 6. I2C1_ER_IRQn (I2C Error) - Förväntad prioritet: 3
    HAL_NVIC_GetPriority(I2C1_ER_IRQn, priority_group, &preempt_prio, &sub_prio);
    uint32_t prio_i2c_er = preempt_prio;
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)prio_i2c_er);
    bool pass_i2c_er = (prio_i2c_er == 3);
    DIAG_SEND_RESULT("NVIC_I2C_ER", "I2C1_ER IRQ Priority (Expected 3)", val_str, pass_i2c_er);
    //if (!pass_i2c_er) overall_pass = false;

    // 7. DMA1_Stream0_IRQn (I2C RX DMA) - Förväntad prioritet: 3
    HAL_NVIC_GetPriority(DMA1_Stream0_IRQn, priority_group, &preempt_prio, &sub_prio);
    uint32_t prio_dma_rx = preempt_prio;
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)prio_dma_rx);
    bool pass_dma_rx = (prio_dma_rx == 3);
    DIAG_SEND_RESULT("NVIC_DMA_I2C_RX", "DMA_I2C_RX IRQ Priority (Expected 3)", val_str, pass_dma_rx);
    //if (!pass_dma_rx) overall_pass = false;

    // 8. EXTI9_5_IRQn (ADXL Interrupt, PA7) - Förväntad prioritet: 4
    HAL_NVIC_GetPriority(EXTI9_5_IRQn, priority_group, &preempt_prio, &sub_prio);
    uint32_t prio_exti = preempt_prio;
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)prio_exti);
    bool pass_exti = (prio_exti == 4);
    DIAG_SEND_RESULT("NVIC_EXTI9_5", "EXTI9_5 IRQ Priority (Expected 4)", val_str, pass_exti);
    //if (!pass_exti) overall_pass = false;

    // 9. SENSOR_HIERARCHY Jämförelse: I2C/DMA (3) < EXTI (4)
    snprintf(val_str, sizeof(val_str), "I2C/DMA=%lu, EXTI=%lu", (unsigned long)prio_i2c_ev, (unsigned long)prio_exti);
    bool pass_sensor_hierarchy = (prio_i2c_ev == 3) && (prio_i2c_er == 3) && (prio_dma_rx == 3) && (prio_exti == 4);
    DIAG_SEND_RESULT("SENSOR_HIERARCHY", "I2C/DMA (3) < EXTI (4) Priority", val_str, pass_sensor_hierarchy);
    
    // --- END: New critical tests ---
}

/**
 * @brief Kör diagnostik för callback-kedjan (EXTI -> DMA)
 * @note Förlitar sig på externa debug-räknare från sensor_hal.c
 */
static void Diagnostics_Callback_Chain(AppContext_t* ctx) {
    char val_str[32]; // FIX: Buffer för att formatera numeriska värden
    
    // FIX: Rensat bort lokala extern-deklarationer.
    // Alla variabler (g_debug_..., g_i2c_state, g_sampling_active)
    // inkluderas nu korrekt via sensor_hal.h.
    
    // EXTI callback diagnostics
    // FIX: Byt ut Telemetry_SendDIAG_RES mot COMM_SendfLine för att stödja "INFO"/"WARN"
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_debug_exti_callback_count);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                   "DEBUG_EXTI_TOTAL", 
                   "EXTI Callback Total Count", 
                   val_str,
                   (g_debug_exti_callback_count > 0) ? "PASS" : "FAIL");
    
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_debug_exti_rejected_sampling);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_EXTI_REJ_SAMP", 
                           "EXTI Rejected (sampling=false)", 
                           val_str,
                           "INFO");
    
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_debug_exti_rejected_context);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_EXTI_REJ_CTX", 
                           "EXTI Rejected (context=NULL)", 
                           val_str,
                           "INFO");
    
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_debug_exti_rejected_state);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_EXTI_REJ_STATE", 
                           "EXTI Rejected (state!=IDLE)", 
                           val_str,
                           "INFO");
    
    // DMA start diagnostics
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_debug_dma_start_ok);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_DMA_START_OK", 
                           "DMA Start Success Count", 
                           val_str,
                           (g_debug_dma_start_ok > 0) ? "PASS" : "FAIL");
    
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_debug_dma_start_fail);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_DMA_START_FAIL", 
                           "DMA Start Failure Count", 
                           val_str,
                           (g_debug_dma_start_fail == 0) ? "PASS" : "FAIL");
    
    // DMA completion diagnostics
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_debug_dma_complete_count);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_DMA_COMPLETE", 
                           "DMA Complete Callback Count", 
                           val_str,
                           (g_debug_dma_complete_count > 0) ? "PASS" : "FAIL");
    
    // Sample processing diagnostics
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_debug_samples_processed);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_SAMPLES_PROC", 
                           "Samples Processed Count", 
                           val_str,
                           (g_debug_samples_processed > 0) ? "PASS" : "FAIL");
    
    // State machine diagnostic
    const char* state_str;
    switch(g_i2c_state) {
        case I2C_STATE_IDLE: state_str = "IDLE"; break;
        case I2C_STATE_WAIT_FIFO_DATA: state_str = "WAIT_FIFO"; break;
        case I2C_STATE_DRAIN_STATUS: state_str = "DRAIN_STATUS"; break;
        case I2C_STATE_CLEAR_INT_SOURCE: state_str = "CLEAR_INT"; break;
        default: state_str = "UNKNOWN"; break;
    }
    
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_I2C_STATE", 
                           "I2C State Machine", 
                           state_str,
                           (g_i2c_state == I2C_STATE_IDLE) ? "PASS" : "WARN");
    
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DEBUG_SAMPLING_ACTIVE", 
                           "Sampling Active Flag", 
                           g_sampling_active ? "true" : "false",
                           g_sampling_active ? "PASS" : "INFO");
    
    // Existing diagnostic counters
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)ctx->diag.i2c_fail);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DIAG_I2C_FAIL", 
                           "I2C Failure Count", 
                           val_str,
                           (ctx->diag.i2c_fail == 0) ? "PASS" : "FAIL");
    
    snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)ctx->diag.ring_ovf);
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", 
                           "DIAG_RING_OVF", 
                           "Ring Buffer Overflow Count", 
                           val_str,
                           (ctx->diag.ring_ovf == 0) ? "PASS" : "WARN");
}