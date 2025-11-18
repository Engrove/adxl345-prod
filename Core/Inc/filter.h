/* filename: Core/Inc/filter.h */
#ifndef FILTER_H_
#define FILTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 2nd-order Butterworth low-pass biquad.
 * a0 is normalized to 1.0.
 */
typedef struct {
    float b0, b1, b2;   /* feedforward */
    float a1, a2;       /* feedback (a0 == 1) */
    float x1, x2;       /* input history */
    float y1, y2;       /* output history */
} iir_filter_t;

/* Initialize with cutoff fc_hz and sample rate fs_hz. */
void  Filter_Init(iir_filter_t* f, float fc_hz, float fs_hz);

/* Zero the internal state. */
void  Filter_Reset(iir_filter_t* f);

/* Process one sample and return the filtered value. */
float Filter_Update(iir_filter_t* f, float input);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H_ */