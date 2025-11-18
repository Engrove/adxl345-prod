/* filename: Core/Src/api_parse.c */
/*
 * Project: EngoveVibraNUCLEO-F446RE_2
 * FW: 3.3.0
 * Module: Core/Src/api_parse.c
 * Purpose: Strict, lightweight parsers for API v3.3.0 protocol primitives.
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Local helpers: no locale, no libc parsing. */
static inline const char* skip_ws(const char* s) {
    while (*s == ' ' || *s == '\t') { ++s; }
    return s;
}
static inline int is_term_or_eol(char c) {
    return (c == 0) || (c == ',') || (c == ' ') || (c == '\t') || (c == '\r') || (c == '\n');
}

/* Parse unsigned 32-bit integer. Strict decimal. No sign. */
int api_parse_u32(const char* s, uint32_t* out) {
    if (!s || !out) return 0;
    s = skip_ws(s);
    uint32_t acc = 0; int nd = 0;
    while (*s >= '0' && *s <= '9') {
        uint32_t d = (uint32_t)(*s - '0');
        /* overflow check: acc*10 + d <= 0xFFFFFFFF */
        if (acc > 429496729U || (acc == 429496729U && d > 5U)) return 0;
        acc = acc * 10U + d;
        ++s; ++nd;
    }
    if (nd == 0) return 0;
    s = skip_ws(s);
    if (!is_term_or_eol(*s)) return 0;
    *out = acc;
    return 1;
}

/* Parse unsigned 64-bit integer. Strict decimal. No sign. */
int api_parse_u64(const char* s, uint64_t* out) {
    if (!s || !out) return 0;
    s = skip_ws(s);
    uint64_t acc = 0; int nd = 0;
    while (*s >= '0' && *s <= '9') {
        uint64_t d = (uint64_t)(*s - '0');
        /* overflow check: acc*10 + d <= 0xFFFFFFFFFFFFFFFF */
        if (acc > 1844674407370955161ULL || (acc == 1844674407370955161ULL && d > 5ULL)) return 0;
        acc = acc * 10ULL + d;
        ++s; ++nd;
    }
    if (nd == 0) return 0;
    s = skip_ws(s);
    if (!is_term_or_eol(*s)) return 0;
    *out = acc;
    return 1;
}

/* Parse fixed-point float with up to 3 decimals. No exponent. Optional leading sign. */
int api_parse_float_fixed3(const char* s, float* out) {
    if (!s || !out) return 0;
    s = skip_ws(s);
    int neg = 0;
    if (*s == '+' || *s == '-') { neg = (*s == '-'); ++s; }

    /* integer part */
    uint32_t ip = 0; int nd_int = 0;
    while (*s >= '0' && *s <= '9') { ip = ip * 10U + (uint32_t)(*s - '0'); ++s; ++nd_int; }

    /* fractional part (optional) */
    uint32_t fp = 0; int nd_frac = 0;
    if (*s == '.') {
        ++s;
        while (*s >= '0' && *s <= '9') {
            if (nd_frac < 3) { fp = fp * 10U + (uint32_t)(*s - '0'); ++nd_frac; ++s; }
            else {
                /* more than 3 decimals is not allowed by strict spec */
                return 0;
            }
        }
    }

    if (nd_int == 0 && nd_frac == 0) return 0; /* no digits at all */

    /* allow spaces before terminator */
    s = skip_ws(s);
    if (!is_term_or_eol(*s)) return 0;

    /* scale fractional to 3 digits by padding with zeros */
    while (nd_frac < 3) { fp *= 10U; ++nd_frac; }

    /* combine: value = (ip*1000 + fp) / 1000.0 with sign */
    uint32_t scaled = ip;
    /* protect from overflow in ip*1000 + fp: ip fits in 32-bit. */
    if (scaled > 4294966U) {
        /* 4,294,966 * 1000 ~= 4.29e9 fits, next would overflow 32-bit. Use double path. */
        double v = (double)ip + (double)fp / 1000.0;
        if (neg) v = -v;
        *out = (float)v;
        return 1;
    }
    scaled = scaled * 1000U + fp;
    float v = (float)scaled / 1000.0f;
    if (neg) v = -v;
    *out = v;
    return 1;
}

/* Optional: 16-bit helper if needed elsewhere. */
int api_parse_u16(const char* s, uint16_t* out) {
    if (!s || !out) return 0;
    s = skip_ws(s);
    uint32_t acc = 0; int nd = 0;
    while (*s >= '0' && *s <= '9') {
        acc = acc * 10U + (uint32_t)(*s - '0');
        if (acc > 0xFFFFU) return 0;
        ++s; ++nd;
    }
    if (nd == 0) return 0;
    s = skip_ws(s);
    if (!is_term_or_eol(*s)) return 0;
    *out = (uint16_t)acc;
    return 1;
}

/* filename: Core/Src/api_parse.c */