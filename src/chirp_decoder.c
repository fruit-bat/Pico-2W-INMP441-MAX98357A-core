#include "chirp_decoder.h"
#include "pico/stdlib.h"

float32_t* __not_in_flash_func(chirp_decoder_complex_chirp_to_magnitudes)(
    ChirpDecoderState_t *state, 
    float32_t* complex_input_buffer // Interleaved complex input: [Real, Imag, Real, Imag...] 
) {
    // Complex Element-wise Multiplication (De-chirp).
    // Multiplies complex_input by complex_dechirp and saves to fft_output_buffer.
    arm_cmplx_mult_cmplx_f32(
        complex_input_buffer, 
        state->complex_dechirp_vector, 
        state->fft_output_buffer, 
        state->fft_size);

    // Execute the Complex FFT.
    // Note: arm_cfft_f32 processes data IN-PLACE. 
    arm_cfft_f32(
        state->cfft_instance, 
        state->fft_output_buffer, 
        0,                          // '0' means Forward FFT (1 would mean Inverse)
        1                           // The bit reversal flag is hardcoded to 1 in modern CMSIS-DSP
    );

    // Calculate the squared magnitudes for all 1024 bins.
    arm_cmplx_mag_squared_f32(state->fft_output_buffer, state->fft_magnitude_buffer, state->fft_size);

    return &state->fft_magnitude_buffer[0];
}

ChirpSymbolRx_t* chirp_decoder_magnitudes_to_symbol(
    ChirpDecoderState_t *state,
    float32_t* magnitude_buffer, // Input magnitude spectrum (length = fft_size)
    ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
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

ChirpSymbolRx_t* chirp_decoder_complex_chirp_to_symbol(
    ChirpDecoderState_t *state,
    float32_t* complex_input_buffer, // Interleaved complex input: [Real, Imag, Real, Imag...] 
    ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
    ChirpSymbolRx_t *symbol_rx // Output structure to hold the decoded symbol properties
) {
    // Convert the complex chirp signal to its magnitude spectrum
    float32_t* magnitude_buffer = chirp_decoder_complex_chirp_to_magnitudes(state, complex_input_buffer);

    // Decode the symbol from the magnitude spectrum
    return chirp_decoder_magnitudes_to_symbol(state, magnitude_buffer, config, symbol_rx);
}

