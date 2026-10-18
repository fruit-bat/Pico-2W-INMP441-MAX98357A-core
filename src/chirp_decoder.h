#ifndef CHIRP_DECODER_H
#define CHIRP_DECODER_H

#include <stdint.h>
#include "arm_math.h"
#include "chirp_symbol_rx.h"
#include "cyclic_weighted_centroid.h"

//
// The decoder never allocates memory. The caller provides all buffers (typically
// as static arrays) and the decoder state only holds pointers to them.
//
// Required buffer sizes, in float32_t elements:
//
//   complex_dechirp_vector : CHIRP_DECODER_COMPLEX_LEN(fft_size)
//   fft_output_buffer      : CHIRP_DECODER_COMPLEX_LEN(fft_size)
//   fft_magnitude_buffer   : fft_size
//   cos_table, sin_table   : number_of_bins (bins spanned by the chirp bandwidth)
//
#define CHIRP_DECODER_COMPLEX_LEN(fft_size) ((fft_size) * 2u)

typedef struct {
    uint32_t fft_size;
    float32_t *fft_magnitude_buffer;        // Preallocated buffer for FFT magnitude results (length = fft_size)
    float32_t *fft_output_buffer;           // Preallocated buffer for FFT complex output (length = fft_size * 2, interleaved real/imag)
    float32_t *complex_dechirp_vector;      // Preallocated buffer for the complex de-chirp vector (length = fft_size * 2, interleaved real/imag)
    const arm_cfft_instance_f32 *cfft_instance; // Pointer to the CMSIS-DSP CFFT instance for the specified FFT size
    CyclicWeightedCentroid_t cwc;           // Centroid configuration for symbol decoding
    uint32_t peak_half_width;               // Bins either side of the peak used to refine its position
} ChirpDecoderState_t;

//
// Parameters describing the up-chirp the decoder correlates against.
// The chirp phase is 2*pi*(f0*t + 0.5*chirp_rate*t^2).
//
typedef struct {
    float32_t sample_rate;  // Hz
    float32_t f0;           // Start frequency, Hz
    float32_t chirp_rate;   // Hz per second
} ChirpParams_t;

//
// Initializes the decoder state, wiring in the caller-provided buffers and
// generating the complex de-chirp vector and centroid tables.
//
void chirp_decoder_init(
    ChirpDecoderState_t *state,
    const ChirpParams_t *params,
    uint32_t fft_size,
    const arm_cfft_instance_f32 *cfft_instance,
    uint32_t power_exponent,               // Centroid weighting exponent, w = p^k
    uint32_t number_of_bins,               // Number of FFT bins spanned by the chirp bandwidth
    uint32_t peak_half_width,              // Bins either side of the peak used to refine its position (e.g. 3)
    float32_t *complex_dechirp_vector,     // length = fft_size * 2
    float32_t *fft_output_buffer,          // length = fft_size * 2
    float32_t *fft_magnitude_buffer,       // length = fft_size
    float32_t *cos_table,                  // length = number_of_bins
    float32_t *sin_table                   // length = number_of_bins
);

// Complex chirp -> Magnitudes -> ChirpSymbolRx_t
ChirpSymbolRx_t* chirp_decoder_complex_chirp_to_symbol(
    ChirpDecoderState_t *state,
    const float32_t* complex_input_buffer, // Interleaved complex input: [Real, Imag, Real, Imag...]
    const ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
    ChirpSymbolRx_t *symbol_rx // Output structure to hold the decoded symbol properties
);

// Real chirp -> Magnitudes -> ChirpSymbolRx_t
// Equivalent to the complex version with all imaginary parts zero, but avoids
// staging an interleaved complex input buffer.
ChirpSymbolRx_t* chirp_decoder_real_chirp_to_symbol(
    ChirpDecoderState_t *state,
    const float32_t* real_input_buffer, // Real input samples (length = fft_size)
    const ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
    ChirpSymbolRx_t *symbol_rx // Output structure to hold the decoded symbol properties
);

#endif // CHIRP_DECODER_H
