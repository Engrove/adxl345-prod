/* Core/Inc/api_parse.h */
/* 2025-10-13 13:00:00+03:00 [Europe/Helsinki] */
#ifndef API_PARSE_H_
#define API_PARSE_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ABNF (v3.3.0):
 * float = ["-"] 1*DIGIT "." 1*3DIGIT ; ingen exponent
 */
int api_parse_u32(const char *s, uint32_t *out);
int api_parse_u64(const char *s, uint64_t *out);
int api_parse_float_fixed3(const char *s, float *out);
int api_parse_qstring(const char *s, char *out, size_t out_sz);
int api_parse_u16(const char* s, uint16_t* out);

#ifdef __cplusplus
}
#endif

#endif /* API_PARSE_H_ */
/* Core/Inc/api_parse.h */