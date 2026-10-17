#include "chirp_decoder.h"
#include "pico/stdlib.h"

static void generate_complex_dechirp_vector(
    float32_t *complex_dechirp_vector,
    uint32_t fft_size,
    const ChirpParams_t *params
) {
    for (uint32_t n = 0; n < fft_size; n++) {
        const float32_t t = (float32_t)n / params->sample_rate;
        // The standard Up-Chirp phase equation
        const float32_t phase = 2.0f * PI * (params->f0 * t + 0.5f * params->chirp_rate * t * t);

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
    float32_t power_exponent,
    uint32_t number_of_bins,
    float32_t *complex_dechirp_vector,
    float32_t *fft_output_buffer,
    float32_t *fft_magnitude_buffer,
    float32_t *cos_table,
    float32_t *sin_table
) {
    state->fft_size = fft_size;
    state->cfft_instance = cfft_instance;
    state->complex_dechirp_vector = complex_dechirp_vector;
    state->fft_output_buffer = fft_output_buffer;
    state->fft_magnitude_buffer = fft_magnitude_buffer;

    generate_complex_dechirp_vector(complex_dechirp_vector, fft_size, params);

    cyclic_weighted_centroid_init(
        &state->cwc,
        fft_size,
        power_exponent,
        number_of_bins,
        cos_table,
        sin_table
    );
}

// FFT of the de-chirped signal already in fft_output_buffer -> magnitudes
static float32_t* __not_in_flash_func(dechirped_to_magnitudes)(
    ChirpDecoderState_t *state
) {
    // Execute the Complex FFT.
    // Note: arm_cfft_f32 processes data IN-PLACE.
    arm_cfft_f32(
        state->cfft_instance,
        state->fft_output_buffer,
        0,                          // '0' means Forward FFT (1 would mean Inverse)
        1                           // The bit reversal flag is hardcoded to 1 in modern CMSIS-DSP
    );

    // Calculate the squared magnitudes for all bins.
    arm_cmplx_mag_squared_f32(state->fft_output_buffer, state->fft_magnitude_buffer, state->fft_size);

    return &state->fft_magnitude_buffer[0];
}

float32_t* __not_in_flash_func(chirp_decoder_complex_chirp_to_magnitudes)(
    ChirpDecoderState_t *state,
    const float32_t* complex_input_buffer // Interleaved complex input: [Real, Imag, Real, Imag...]
) {
    // Complex Element-wise Multiplication (De-chirp).
    // Multiplies complex_input by complex_dechirp and saves to fft_output_buffer.
    arm_cmplx_mult_cmplx_f32(
        complex_input_buffer,
        state->complex_dechirp_vector,
        state->fft_output_buffer,
        state->fft_size);

    return dechirped_to_magnitudes(state);
}

float32_t* __not_in_flash_func(chirp_decoder_real_chirp_to_magnitudes)(
    ChirpDecoderState_t *state,
    const float32_t* real_input_buffer // Real input samples (length = fft_size)
) {
    // Complex-by-Real Element-wise Multiplication (De-chirp).
    // Scales each complex de-chirp sample by the real input and saves to fft_output_buffer.
    arm_cmplx_mult_real_f32(
        state->complex_dechirp_vector,
        real_input_buffer,
        state->fft_output_buffer,
        state->fft_size);

    return dechirped_to_magnitudes(state);
}

ChirpSymbolRx_t* __not_in_flash_func(chirp_decoder_magnitudes_to_symbol)(
    ChirpDecoderState_t *state,
    const float32_t* magnitude_buffer, // Input magnitude spectrum (length = fft_size)
    const ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
    ChirpSymbolRx_t *symbol_rx // Output structure to hold the decoded symbol properties
) {
    // Calculate the centroid of the FFT magnitude spectrum
    CyclicWeightedCentroidVector_t cwc_vector;
    cyclic_weighted_centroid_init_vector(&cwc_vector);
    cyclic_weighted_centroid_accumulate_forwards(
        &state->cwc,
        magnitude_buffer,
        state->cwc.power_exponent,
        &cwc_vector
    );
    cyclic_weighted_centroid_accumulate_backwards(
        &state->cwc,
        magnitude_buffer,
        state->cwc.power_exponent,
        &cwc_vector
    );

    CyclicWeightedCentroidResult_t cwc_result;
    cyclic_weighted_centroid_finalize(&state->cwc, &cwc_vector, &cwc_result);

    // Set the properties of the received chirp symbol based on the calculated centroid
    chirp_symbol_rx_set(config, symbol_rx, cwc_result.angle, cwc_result.magnitude, cwc_result.strength);

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
