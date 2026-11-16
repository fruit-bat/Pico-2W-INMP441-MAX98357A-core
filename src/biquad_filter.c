#include <math.h>
#include "biquad_filter.h"
#include "pico/stdlib.h"

void biquad_filter_init_highpass(
    BiquadFilter_t *f,
    float32_t sample_rate,
    float32_t cutoff_hz,
    float32_t q
) {
    // Bilinear-transform high-pass (R. Bristow-Johnson's audio EQ cookbook),
    // computed in double and normalised so that a0 = 1
    const double w = 2.0 * M_PI * (double)cutoff_hz / (double)sample_rate;
    const double alpha = sin(w) / (2.0 * (double)q);
    const double c = cos(w);
    const double a0 = 1.0 + alpha;

    f->b0 = (float32_t)((1.0 + c) / 2.0 / a0);
    f->b1 = (float32_t)(-(1.0 + c) / a0);
    f->b2 = (float32_t)((1.0 + c) / 2.0 / a0);
    f->a1 = (float32_t)(-2.0 * c / a0);
    f->a2 = (float32_t)((1.0 - alpha) / a0);
    biquad_filter_reset(f);
}

void biquad_filter_reset(BiquadFilter_t *f) {
    f->s1 = 0.0f;
    f->s2 = 0.0f;
}

void __not_in_flash_func(biquad_filter_process)(BiquadFilter_t *f, float32_t *samples, uint32_t count) {
    const float32_t b0 = f->b0, b1 = f->b1, b2 = f->b2, a1 = f->a1, a2 = f->a2;
    float32_t s1 = f->s1, s2 = f->s2;
    for (uint32_t i = 0; i < count; i++) {
        const float32_t x = samples[i];
        const float32_t y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        samples[i] = y;
    }
    f->s1 = s1;
    f->s2 = s2;
}
