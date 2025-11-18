/* FILENAME: Core/Inc/dev_telemetry.h */
#ifndef DEV_TELEMETRY_H_
#define DEV_TELEMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * ============================================================================
 * Aktivera/Avaktivera RX/TX Debug Telemetry
 * ============================================================================
 * Sätt till 1 för att aktivera detaljerad loggning av RX/TX-händelser.
 * Sätt till 0 för att helt kompilera bort all debug-kod för noll overhead.
 */
#define RXTX_DEBUG 1

#if RXTX_DEBUG > 0

/**
 * @brief Loggar en DMA RX-händelse (när data tas emot från UART).
 * @param size Antal mottagna byte.
 * @param ring_usage Antal byte i RX-ringbufferten EFTER att ny data lagts till.
 * @param ring_size Total storlek på RX-ringbufferten.
 */
void DevTel_RxDmaEvent(uint16_t size, uint16_t ring_usage, uint16_t ring_size);

/**
 * @brief Loggar ett försök att köa data för sändning.
 * @param len Antal byte som försöker köas.
 * @param free_before Ledigt utrymme i TX-ringbufferten FÖRE försöket.
 * @param dropped true om paketet tappades, annars false.
 */
void DevTel_TxEnqueue(size_t len, uint16_t free_before, bool dropped);

/**
 * @brief Loggar starten av en ny DMA TX-överföring.
 * @param len Antal byte som laddas i DMA-bufferten.
 * @param ring_usage Antal byte kvar i TX-ringbufferten FÖRE denna sändning.
 * @param ring_size Total storlek på TX-ringbufferten.
 */
void DevTel_TxDmaStart(uint16_t len, uint16_t ring_usage, uint16_t ring_size);

/**
 * @brief Loggar att en DMA TX-överföring är slutförd.
 * @param len_sent Antal byte som just skickats.
 * @param ring_usage Antal byte kvar i TX-ringbufferten EFTER att denna sändning är klar.
 * @param ring_size Total storlek på TX-ringbufferten.
 */
void DevTel_TxDmaComplete(uint16_t len_sent, uint16_t ring_usage, uint16_t ring_size);

/**
 * @brief Loggar status för BLOCKS transportlager.
 * @param q_count Antal block som väntar i kön.
 * @param q_size Total storlek på kön.
 * @param inflight_count Antal block som är "in-flight" (väntar på ACK).
 * @param window_size Nuvarande fönsterstorlek.
 */
void DevTel_LogTbStatus(uint8_t q_count, uint8_t q_size, uint8_t inflight_count, uint16_t window_size);

#endif // RXTX_DEBUG > 0

#ifdef __cplusplus
}
#endif

#endif /* DEV_TELEMETRY_H_ */
