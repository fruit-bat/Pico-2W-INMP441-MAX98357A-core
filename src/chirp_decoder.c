#include <math.h>
#include <string.h>
#include "chirp_decoder.h"
#include "pico/stdlib.h"

//
// Conjugate up-chirp, optionally with an extra frequency shift of
// shift_bins FFT bins (the half-bin vector uses 0.5: multiplying by
// exp(-j*2*pi*0.5*n/N) moves every bin down by half a bin, so the FFT's bin k
// then holds the spectrum at k + 1/2).
//
static void generate_complex_dechirp_vector(
    float32_t *complex_dechirp_vector,
    uint32_t fft_size,
    const ChirpParams_t *params,
    float32_t shift_bins
) {
    for (uint32_t n = 0; n < fft_size; n++) {
        const float32_t t = (float32_t)n / params->sample_rate;
        // The standard Up-Chirp phase equation, plus the shift
        const float32_t phase = 2.0f * PI * (params->f0 * t + 0.5f * params->chirp_rate * t * t)
                              + 2.0f * PI * shift_bins * (float32_t)n / (float32_t)fft_size;

        // Complex Conjugate: [Cos(phase), -Sin(phase)]
        complex_dechirp_vector[2 * n]     = cosf(phase);
        complex_dechirp_vector[2 * n + 1] = -sinf(phase);
    }
}

void chirp_decoder_init(
    ChirpDecoderState_t *state,
    const ChirpParams_t *params,
    uint32_t fft_size,
    const arm_cfft_instance_f32 *cfft_instance,
    uint32_t power_exponent,
    uint32_t number_of_bins,
    uint32_t peak_half_width,
    float32_t *complex_dechirp_vector,
    float32_t *complex_dechirp_half_vector,
    float32_t *fft_half_buffer,
    float32_t *fft_output_buffer,
    float32_t *fft_magnitude_buffer,
    float32_t *folded_power_buffer,
    float32_t *fine_power_buffer,
    float32_t *cos_table,
    float32_t *sin_table
) {
    state->fft_size = fft_size;
    state->cfft_instance = cfft_instance;
    state->complex_dechirp_vector = complex_dechirp_vector;
    state->fft_output_buffer = fft_output_buffer;
    state->fft_magnitude_buffer = fft_magnitude_buffer;
    state->folded_power_buffer = folded_power_buffer;
    state->complex_dechirp_half_vector = complex_dechirp_half_vector;
    state->fft_half_buffer = fft_half_buffer;
    state->fine_power_buffer = fine_power_buffer;
    state->peak_half_width = peak_half_width;

    generate_complex_dechirp_vector(complex_dechirp_vector, fft_size, params, 0.0f);
    generate_complex_dechirp_vector(complex_dechirp_half_vector, fft_size, params, 0.5f);

    // Band-pass: the FFT bins covering the chirp band f0 .. f0 + bandwidth
    const double bin_hz = (double)params->sample_rate / (double)fft_size;
    const double bandwidth = (double)params->chirp_rate * (double)fft_size / (double)params->sample_rate;
    uint32_t last = (uint32_t)ceil(((double)params->f0 + bandwidth) / bin_hz);
    if (last > fft_size / 2u - 1u) last = fft_size / 2u - 1u;
    state->band_first_bin = (uint32_t)floor((double)params->f0 / bin_hz);
    state->band_last_bin = last;

    // Fold rotation. With phase(v) = 2*pi*(f0*v/fs + 0.5*chirp_rate*(v/fs)^2), a
    // chirp cyclically shifted to land on bin p de-chirps into a tone at bin p
    // (before the wrap) and one at p - B (after it), B = bandwidth in bins. For
    // the two to join up into one continuous tone the wrapped piece needs
    // rotating by exp(j*2*pi*(f0*N/fs - B/2 + f)), where f is the frequency
    // (in bins) the spectrum is evaluated at. On whole bins the f term drops
    // out; half way between bins it is exp(j*pi) = -1, so the half-bin
    // spectrum uses -R. Computed in double: f0*N/fs is several hundred cycles
    // and only its fractional part matters.
    const double bins_in_band = bandwidth / bin_hz;
    double turns = (double)params->f0 * (double)fft_size / (double)params->sample_rate - 0.5 * bins_in_band;
    turns -= floor(turns);
    state->fold_rotation_re = (float32_t)cos(2.0 * M_PI * turns);
    state->fold_rotation_im = (float32_t)sin(2.0 * M_PI * turns);

    cyclic_weighted_centroid_init(
        &state->cwc,
        power_exponent,
        number_of_bins,
        cos_table,
        sin_table
    );
}

//
// Folds a de-chirped spectrum x (fft_size complex bins) into number_of_bins
// powers, out[k * stride] = |x[k] + R * x[fft_size - number_of_bins + k]|^2:
// the symbol's wrapped piece (bin k - number_of_bins) is rotated into phase
// with the first piece and added.
//
static void __not_in_flash_func(fold_power)(
    const ChirpDecoderState_t *state,
    const float32_t *x,
    float32_t rr,
    float32_t ri,
    float32_t *out,
    uint32_t stride
) {
    const uint32_t size = (uint32_t)state->cwc.size;
    for (uint32_t k = 0; k < size; k++) {
        const uint32_t m = state->fft_size - size + k;
        const float32_t br = x[2 * m], bi = x[2 * m + 1];
        const float32_t re = x[2 * k] + (rr * br - ri * bi);
        const float32_t im = x[2 * k + 1] + (rr * bi + ri * br);
        out[k * stride] = re * re + im * im;
    }
}

//
// De-chirp a complex signal z (may be fft_output_buffer itself), FFT and fold:
//   - fft_magnitude_buffer: plain power of every de-chirped bin
//   - folded_power_buffer:  folded power on whole bins (returned)
//   - fine_power_buffer:    folded power every half bin
//
static float32_t* __not_in_flash_func(dechirp_and_fold)(
    ChirpDecoderState_t *state,
    const float32_t *z
) {
    const uint32_t n = state->fft_size;
    const uint32_t size = (uint32_t)state->cwc.size;

    // Half-bin path first, as the whole-bin de-chirp may overwrite z
    arm_cmplx_mult_cmplx_f32(z, state->complex_dechirp_half_vector, state->fft_half_buffer, n);
    arm_cmplx_mult_cmplx_f32(z, state->complex_dechirp_vector, state->fft_output_buffer, n);

    // Whole bins
    arm_cfft_f32(state->cfft_instance, state->fft_output_buffer, 0, 1);
    arm_cmplx_mag_squared_f32(state->fft_output_buffer, state->fft_magnitude_buffer, n);
    fold_power(state, state->fft_output_buffer,
        state->fold_rotation_re, state->fold_rotation_im, state->folded_power_buffer, 1);
    for (uint32_t k = 0; k < size; k++) {
        state->fine_power_buffer[2 * k] = state->folded_power_buffer[k];
    }

    // Half way between bins, folded with -R
    arm_cfft_f32(state->cfft_instance, state->fft_half_buffer, 0, 1);
    fold_power(state, state->fft_half_buffer,
        -state->fold_rotation_re, -state->fold_rotation_im, &state->fine_power_buffer[1], 2);

    return state->folded_power_buffer;
}

static float32_t* __not_in_flash_func(chirp_decoder_complex_chirp_to_magnitudes)(
    ChirpDecoderState_t *state,
    const float32_t* complex_input_buffer // Interleaved complex input: [Real, Imag, Real, Imag...]
) {
    return dechirp_and_fold(state, complex_input_buffer);
}

static float32_t* __not_in_flash_func(chirp_decoder_real_chirp_to_magnitudes)(
    ChirpDecoderState_t *state,
    const float32_t* real_input_buffer // Real input samples (length = fft_size)
) {
    const uint32_t n = state->fft_size;
    float32_t *buf = state->fft_output_buffer;

    // Stage the real input as complex
    for (uint32_t i = 0; i < n; i++) {
        buf[2 * i] = real_input_buffer[i];
        buf[2 * i + 1] = 0.0f;
    }

    // Band-pass: keep only the chirp band's positive-frequency bins. The
    // inverse FFT (which scales by 1/n) then gives the band-limited analytic
    // signal, roughly half the input amplitude times exp(j*phase).
    arm_cfft_f32(state->cfft_instance, buf, 0, 1);
    const uint32_t first = state->band_first_bin;
    const uint32_t last = state->band_last_bin;
    memset(buf, 0, 2u * first * sizeof(*buf));
    memset(&buf[2u * (last + 1u)], 0, 2u * (n - last - 1u) * sizeof(*buf));
    arm_cfft_f32(state->cfft_instance, buf, 1, 1);

    return dechirp_and_fold(state, buf);
}

static ChirpSymbolRx_t* __not_in_flash_func(chirp_decoder_magnitudes_to_symbol)(
    ChirpDecoderState_t *state,
    const float32_t* magnitude_buffer, // Folded power spectrum (length = number_of_bins)
    const ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
    ChirpSymbolRx_t *symbol_rx // Output structure to hold the decoded symbol properties
) {
    // Calculate the centroid of the FFT magnitude spectrum; only its magnitude
    // and strength are used, as measures of how clear the symbol is
    CyclicWeightedCentroidVector_t cwc_vector;
    cyclic_weighted_centroid_init_vector(&cwc_vector);
    cyclic_weighted_centroid_accumulate(
        &state->cwc,
        magnitude_buffer,
        state->cwc.power_exponent,
        &cwc_vector
    );

    CyclicWeightedCentroidResult_t cwc_result;
    cyclic_weighted_centroid_finalize(&state->cwc, &cwc_vector, &cwc_result);

    // Decide the symbol from the strongest folded bin; in noise this is far
    // more reliable than the centroid angle
    const uint32_t peak_bin = cyclic_weighted_centroid_peak_bin(&state->cwc, magnitude_buffer);

    // Refine the peak position to sub-bin precision from the energy around
    // it, on the half-bin spectrum (2 points per bin); this is what makes
    // small timing errors measurable, even when echoes spread the peak over
    // several bins
    const float32_t angle = cyclic_weighted_centroid_refine_peak(
        state->fine_power_buffer, 2u * state->cwc.size, 2u * peak_bin, 2u * state->peak_half_width);

    // The symbol is the strongest bin (bins map onto symbols evenly; with one
    // bin per symbol they are the same); the refined angle gives the timing
    const uint32_t bins = (uint32_t)state->cwc.size;
    const uint32_t symbol = (peak_bin * config->number_of_symbols + bins / 2u) / bins;
    chirp_symbol_rx_set(config, symbol_rx, symbol, angle, cwc_result.magnitude, cwc_result.strength);

    return symbol_rx;
}

ChirpSymbolRx_t* __not_in_flash_func(chirp_decoder_complex_chirp_to_symbol)(
    ChirpDecoderState_t *state,
    const float32_t* complex_input_buffer, // Interleaved complex input: [Real, Imag, Real, Imag...]
    const ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
    ChirpSymbolRx_t *symbol_rx // Output structure to hold the decoded symbol properties
) {
    // Convert the complex chirp signal to its magnitude spectrum
    const float32_t* magnitude_buffer = chirp_decoder_complex_chirp_to_magnitudes(state, complex_input_buffer);

    // Decode the symbol from the magnitude spectrum
    return chirp_decoder_magnitudes_to_symbol(state, magnitude_buffer, config, symbol_rx);
}

ChirpSymbolRx_t* __not_in_flash_func(chirp_decoder_real_chirp_to_symbol)(
    ChirpDecoderState_t *state,
    const float32_t* real_input_buffer, // Real input samples (length = fft_size)
    const ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
    ChirpSymbolRx_t *symbol_rx // Output structure to hold the decoded symbol properties
) {
    // Convert the real chirp signal to its magnitude spectrum
    const float32_t* magnitude_buffer = chirp_decoder_real_chirp_to_magnitudes(state, real_input_buffer);

    // Decode the symbol from the magnitude spectrum
    return chirp_decoder_magnitudes_to_symbol(state, magnitude_buffer, config, symbol_rx);
}
