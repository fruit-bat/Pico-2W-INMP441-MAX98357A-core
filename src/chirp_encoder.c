#include "chirp_encoder.h"
#include "pico/stdlib.h"

void chirp_encoder_init(
    ChirpEncoder_t *enc,
    const float32_t *complex_chirp,
    uint32_t samples_per_symbol,
    uint32_t number_of_symbols,
    float32_t volume
) {
    enc->complex_chirp = complex_chirp;
    enc->samples_per_symbol = samples_per_symbol;
    enc->number_of_symbols = number_of_symbols;
    enc->volume = volume;
}

void __not_in_flash_func(chirp_encoder_symbol_to_samples)(const ChirpEncoder_t *enc, uint32_t symbol, float32_t *out) {
    const uint32_t n = enc->samples_per_symbol;
    const uint32_t shift = (symbol % enc->number_of_symbols) * n / enc->number_of_symbols;
    const float32_t *chirp = enc->complex_chirp;

    // out[i] = Re(chirp[(i + shift) % n]) * volume, split into two runs to avoid the modulo
    const uint32_t first_run = n - shift;
    for (uint32_t i = 0; i < first_run; i++) {
        out[i] = chirp[2 * (i + shift)] * enc->volume;
    }
    for (uint32_t i = first_run; i < n; i++) {
        out[i] = chirp[2 * (i - first_run)] * enc->volume;
    }
}
