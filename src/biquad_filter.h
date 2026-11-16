#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

/**
 * @file biquad_filter.h
 *
 * A single second-order IIR section (biquad), run in place over blocks of a
 * continuous sample stream. The state carries over from one block to the
 * next, so a stream filtered block by block is identical to one filtered in
 * a single call.
 *
 * Used on the microphone stream before the receive window: the INMP441's DC
 * offset and low-frequency noise are far stronger than anything in the chirp
 * band, and where a frame's two ends sit at different low-frequency levels
 * the decoder's FFT band-pass leaks that jump into the band. After de-chirping
 * it lands on the same few bins every frame (folded bins ~3..8), which at low
 * signal levels win against the real symbol. A high-pass well below the
 * band removes it.
 *
 * Direct form II transposed:
 *   y  = b0*x + s1
 *   s1 = b1*x - a1*y + s2
 *   s2 = b2*x - a2*y
 *
 * No memory is allocated.
 */

#include <stdint.h>
#include "arm_math.h"

typedef struct {
    float32_t b0, b1, b2; // Feed-forward coefficients
    float32_t a1, a2;     // Feedback coefficients (a0 normalised to 1)
    float32_t s1, s2;     // State
} BiquadFilter_t;

// Second-order high-pass with cutoff (-3 dB for q = 1/sqrt(2), Butterworth)
// at cutoff_hz. Clears the state.
void biquad_filter_init_highpass(
    BiquadFilter_t *f,
    float32_t sample_rate,
    float32_t cutoff_hz,
    float32_t q
);

// Clears the state (as if the input had been silent)
void biquad_filter_reset(BiquadFilter_t *f);

// Filters count samples in place, continuing from the previous call
void biquad_filter_process(BiquadFilter_t *f, float32_t *samples, uint32_t count);

#endif // BIQUAD_FILTER_H
