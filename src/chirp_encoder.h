#ifndef CHIRP_ENCODER_H
#define CHIRP_ENCODER_H

/**
 * @file chirp_encoder.h
 *
 * Generates the transmitted audio for a chirp symbol.
 *
 * Symbol s is the base up-chirp cyclically shifted by
 * s * samples_per_symbol / number_of_symbols samples, which after de-chirping
 * at the receiver moves its energy by a whole number of FFT bins.
 *
 * The encoder does not own the chirp table; it reads the real part of an
 * interleaved complex chirp such as the decoder's de-chirp vector (whose real
 * part, cos(phase), is the same for the chirp and its conjugate).
 */

#include <stdint.h>
#include "arm_math.h"

typedef struct {
    const float32_t *complex_chirp; // Interleaved complex base chirp (length = samples_per_symbol * 2)
    uint32_t samples_per_symbol;    // Samples in one symbol (the FFT size)
    uint32_t number_of_symbols;     // Number of distinct symbols
    float32_t volume;               // Output amplitude scale
} ChirpEncoder_t;

/**
 * Initializes the encoder.
 *
 * @param enc                Target encoder state.
 * @param complex_chirp      Interleaved complex base chirp (length = samples_per_symbol * 2).
 * @param samples_per_symbol Samples in one symbol.
 * @param number_of_symbols  Number of distinct symbols; should divide samples_per_symbol.
 * @param volume             Output amplitude scale.
 */
void chirp_encoder_init(
    ChirpEncoder_t *enc,
    const float32_t *complex_chirp,
    uint32_t samples_per_symbol,
    uint32_t number_of_symbols,
    float32_t volume
);

/**
 * Writes the samples_per_symbol audio samples for symbol into out.
 * Symbols are taken modulo number_of_symbols.
 */
void chirp_encoder_symbol_to_samples(const ChirpEncoder_t *enc, uint32_t symbol, float32_t *out);

#endif // CHIRP_ENCODER_H
