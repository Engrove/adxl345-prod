/* filename: Core/Src/filter.c */
#include "filter.h"
#include <math.h>
#include <string.h>

/*
 * 2nd-order Butterworth low-pass biquad implemented with bilinear transform.
 * Difference equation:
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 * Coefficients are normalized so a0 = 1.
 */

static inline float _fabsf(float v) { return v < 0.0f ? -v : v; }

void Filter_Reset(iir_filter_t* f)
{
    if (!f) return;
    f->x1 = 0.0f; f->x2 = 0.0f;
    f->y1 = 0.0f; f->y2 = 0.0f;
}

void Filter_Init(iir_filter_t* f, float fc_hz, float fs_hz)
{
    if (!f) return;

    /* Guard and clamp inputs */
    if (!(fs_hz > 0.0f)) {
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f; f->a1 = 0.0f; f->a2 = 0.0f;
        Filter_Reset(f);
        return;
    }
    if (fc_hz < 0.0f) fc_hz = 0.0f;
    /* Nyquist clamp, keep strictly below fs/2 to avoid k -> inf */
//    float nyq = 0.5f * fs_hz;
    if (fc_hz > 0.49f * fs_hz) fc_hz = 0.49f * fs_hz;

    /* Degenerate case: dc passthrough if cutoff is ~0 */
    if (fc_hz <= 0.0f) {
        f->b0 = 0.0f; f->b1 = 0.0f; f->b2 = 0.0f; f->a1 = 0.0f; f->a2 = 0.0f;
        Filter_Reset(f);
        return;
    }

    /* Bilinear transform with pre-warp. Butterworth Q = 1/sqrt(2). */
    const float PI = 3.14159265358979323846f;
    const float k = tanf(PI * fc_hz / fs_hz);
    const float k2 = k * k;
    const float sqrt2 = 1.4142135623730951f;

    const float norm = 1.0f / (1.0f + sqrt2 * k + k2);

    /* Low-pass 2nd order Butterworth */
    f->b0 =  k2 * norm;
    f->b1 =  2.0f * k2 * norm;
    f->b2 =  k2 * norm;
    f->a1 =  2.0f * (k2 - 1.0f) * norm;         /* note: used with minus in update */
    f->a2 =  (1.0f - sqrt2 * k + k2) * norm;     /* note: used with minus in update */

    Filter_Reset(f);
}

float Filter_Update(iir_filter_t* f, float input)
{
    if (!f) return input;

    /* Direct Form I implementation using stored histories */
    float y =
        f->b0 * input +
        f->b1 * f->x1 +
        f->b2 * f->x2 -
        f->a1 * f->y1 -
        f->a2 * f->y2;

    /* Optional flush-to-zero for denormals */
    if (_fabsf(y) < 1e-30f) y = 0.0f;

    /* Shift history */
    f->x2 = f->x1; f->x1 = input;
    f->y2 = f->y1; f->y1 = y;

    return y;
}

/* filename: Core/Src/filter.c */
