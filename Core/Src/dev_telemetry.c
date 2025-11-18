/*
 * Filename: Core/Src/dev_telemetry.c
 * Version: 3.3.2-patched
 *
 * ==============================================================================
 * ÄNDRINGSLOGG (jmf. med ursprunglig fil)
 * ==============================================================================
 * 1. RC-005 (LÅG, men potentiell deadlock): Åtgärdade en latent deadlock-risk
 *    genom att byta ut anropet till den blockerande `Telemetry_WriteBlocking`
 *    mot dess icke-blockerande motsvarighet `Telemetry_Write`.
 *
 *    - Detta garanterar att ett anrop till `DevTel_SendMessage` från en ISR-
 *      kontext aldrig kan fastna i en spin-wait.
 *
 *    - Om TX-bufferten är full kommer debug-meddelandet nu att kastas istället
 *      för att blockera systemet, vilket är det korrekta och säkra beteendet
 *      för icke-kritisk telemetri.
 *
 * 2. Allmän kodkvalitet: Justerade kommentarerna för att tydligt reflektera
 *    det nya, icke-blockerande beteendet. Den gamla hänvisningen till den
 *    blockerande funktionen är borttagen.
 *
 * 3. RC-204 (LÅG): Lade till en kritisk sektion vid rensning av debug-låset
 *    för att förhindra ett mikroskopiskt race-fönster där ett re-entrant
 *    anrop skulle kunna passera.
 * ==============================================================================
 */

#include "dev_telemetry.h"

#if RXTX_DEBUG > 0

#include "api_schema.h"
#include "comm.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Låsflagga för att förhindra re-entrancy och data-korruption om flera
// debug-anrop sker nära inpå varandra.
static volatile bool is_sending_debug_message = false;

/**
 * @brief Säker, ISR-skyddad sändningsfunktion för debug-meddelanden.
 *
 * Denna funktion använder en atomisk låsmekanism. Om en debug-sändning redan
 * pågår kommer funktionen omedelbart att returnera och kasta bort det nya
 * meddelandet. Detta förhindrar köbildning och deadlock.
 *
 * Sändningen omdirigeras nu till Telemetry_Write, vilket garanterar att
 * all kommunikation går genom den DMA-drivna kön i comm.c. Detta eliminerar
 * race conditions och förhindrar deadlock vid anrop från en ISR.
 */
static void DevTel_SendMessage(const char *fmt, ...) {
  // 1. Atomisk "test-and-set" av låset.
  __disable_irq();
  if (is_sending_debug_message) {
    __enable_irq();
    return; // Kasta meddelandet om en sändning redan pågår.
  }
  is_sending_debug_message = true; // Ta låset.
  __enable_irq();

  // 2. Formatera strängen i en lokal buffert.
  char temp_buf[PROTO_MAX_LINE];
  int total_len = 0;

  // Använd snprintf för att säkert bygga prefixet.
  total_len = snprintf(temp_buf, sizeof(temp_buf), "[DEBUG] ");

  if (total_len > 0 && total_len < (int)sizeof(temp_buf)) {
    va_list args;
    va_start(args, fmt);
    // Lägg till resten av meddelandet efter prefixet.
    int len =
        vsnprintf(temp_buf + total_len, sizeof(temp_buf) - total_len, fmt, args);
    va_end(args);

    if (len > 0) {
      total_len += len;
      // Säkerhetskontroll för att inte skicka en trunkerad, felaktig sträng.
      if (total_len >= (int)sizeof(temp_buf)) {
        // Skicka inte om bufferten blev full, kan leda till korrupt meddelande.
      } else {
        /*
         * PATCH RC-005: Använd den Icke-blockerande sändningen.
         * Om bufferten är full (Telemetry_Write returnerar 0),
         * kommer meddelandet att kastas, vilket är korrekt
         * beteende för debug-loggning från en ISR och förhindrar deadlock.
         */
        Telemetry_Write(temp_buf, (size_t)total_len);
      }
    }
  }

  // 4. Släpp låset.
  /* PATCH RC-204: Add critical section for lock clear */
  __disable_irq();
  is_sending_debug_message = false;
  __enable_irq();
}

void DevTel_RxDmaEvent(uint16_t size, uint16_t ring_usage,
                       uint16_t ring_size) {
  // Kallas från ISR. Loggning inaktiverad för maximal stabilitet.
  (void)size;
  (void)ring_usage;
  (void)ring_size;
}

void DevTel_TxEnqueue(size_t len, uint16_t free_before, bool dropped) {
  if (dropped) {
    DevTel_SendMessage("TX_ENQ: DROP, req=%u, free=%u\r\n", (unsigned)len,
                       free_before);
  }
}

void DevTel_TxDmaStart(uint16_t len, uint16_t ring_usage, uint16_t ring_size) {
  // Kallas från ISR-kontext. Loggning inaktiverad för maximal stabilitet.
  (void)len;
  (void)ring_usage;
  (void)ring_size;
}

void DevTel_TxDmaComplete(uint16_t len_sent, uint16_t ring_usage,
                          uint16_t ring_size) {
  // Kallas från ISR-kontext. Loggning inaktiverad för maximal stabilitet.
  (void)len_sent;
  (void)ring_usage;
  (void)ring_size;
}

void DevTel_LogTbStatus(uint8_t q_count, uint8_t q_size,
                        uint8_t inflight_count, uint16_t window_size) {
  DevTel_SendMessage("TB_STATUS: queue=%u/%u, inflight=%u/%u\r\n", q_count,
                     q_size, inflight_count, window_size);
}

#endif // RXTX_DEBUG > 0
