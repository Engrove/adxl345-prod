/* filename: Core/Inc/protocol_crc16.h */
#ifndef PROTOCOL_CRC16_H_
#define PROTOCOL_CRC16_H_

#include <stdint.h>
#include <stddef.h>
#include "api_schema.h" /* för PROTO_CRC16_POLY/INIT och PROTO_MAX_LINE */

/*
 * CRC-16/CCITT-FALSE
 *  - Poly: 0x1021
 *  - Init: 0xFFFF
 *  - RefIn/RefOut: false
 *  - XorOut: 0x0000
 *  - Datainkludering: alla DATA-rader inkl. CRLF (BLOCK_HEADER/END ingår inte).
 */

#ifndef PROTO_CRC16_POLY
#define PROTO_CRC16_POLY 0x1021u
#endif
#ifndef PROTO_CRC16_INIT
#define PROTO_CRC16_INIT 0xFFFFu
#endif

_Static_assert(PROTO_CRC16_POLY == 0x1021u, "CRC16 poly måste vara 0x1021 (CCITT-FALSE)");
_Static_assert(PROTO_CRC16_INIT == 0xFFFFu, "CRC16 init måste vara 0xFFFF (CCITT-FALSE)");

/* Inkrementell ackumulator för block-CRC över DATA+CRLF. */
typedef struct {
    uint16_t crc;
} proto_crc16_t;

static inline void proto_crc16_init(proto_crc16_t* ctx) {
    ctx->crc = (uint16_t)PROTO_CRC16_INIT;
}

/* Bitvis uppdatering, oflätad rad (ingen reflektion). */
static inline uint16_t proto_crc16_update_byte(uint16_t crc, uint8_t b) {
    crc ^= ((uint16_t)b) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ PROTO_CRC16_POLY);
        else               crc = (uint16_t)(crc << 1);
    }
    return crc;
}

static inline void proto_crc16_update(proto_crc16_t* ctx, const void* data, size_t len) {
    const uint8_t* p = (const uint8_t*)data;
    for (size_t i = 0; i < len; ++i) {
        ctx->crc = proto_crc16_update_byte(ctx->crc, p[i]);
    }
}

static inline uint16_t proto_crc16_final(const proto_crc16_t* ctx) {
    return ctx->crc; /* CCITT-FALSE har XorOut=0x0000 */
}

/*
 * Hjälpare för hel rad: mata exakt de bytes som ingår i CRC enligt spec.
 *  - För CSV-"DATA"-raden: hela strängen inklusive avslutande \r\n.
 */
static inline uint16_t proto_crc16_line(const char* line_with_crlf) {
    proto_crc16_t c; proto_crc16_init(&c);
    const uint8_t* p = (const uint8_t*)line_with_crlf;
    while (*p) { c.crc = proto_crc16_update_byte(c.crc, *p++); }
    return c.crc;
}

/*
 * Säker hjälpare för buffertar som kan innehålla NUL: mata exakt len bytes.
 */
static inline uint16_t proto_crc16_buf(const void* data, size_t len) {
    proto_crc16_t c; proto_crc16_init(&c);
    proto_crc16_update(&c, data, len);
    return proto_crc16_final(&c);
}

#endif /* PROTOCOL_CRC16_H_ */
/* filename: Core/Inc/protocol_crc16.h */