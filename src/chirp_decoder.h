#ifndef CHIRP_DECODER_H
#define CHIRP_DECODER_H

#include <stdint.h>
#include "arm_math.h"
#include "chirp_symbol_rx.h"
#include "cyclic_weighted_centroid.h"

typedef struct {
    uint32_t fft_size;
    float32_t *fft_magnitude_buffer;        // Preallocated buffer for FFT magnitude results (length = fft_size)
    float32_t *fft_output_buffer;           // Preallocated buffer for FFT complex output (length = fft_size * 2, interleaved real/imag)
    float32_t *complex_dechirp_vector;      // Preallocated buffer for the complex de-chirp vector (length = fft_size * 2, interleaved real/imag)    
    arm_cfft_instance_f32 *cfft_instance;   // Pointer to the CMSIS-DSP CFFT instance for the specified FFT size
    CyclicWeightedCentroid_t cwc;           // Centroid configuration for symbol decoding
} ChirpDecoderState_t;

// Complex chirp -> Magnitudes -> ChirpSymbolRx_t
ChirpSymbolRx_t* chirp_decoder_complex_chirp_to_symbol(
    ChirpDecoderState_t *state,
    float32_t* complex_input_buffer, // Interleaved complex input: [Real, Imag, Real, Imag...] 
    ChirpSymbolRxConfig_t *config, // Configuration for symbol decoding
    ChirpSymbolRx_t *symbol_rx // Output structure to hold the decoded symbol properties
);

#endif // CHIRP_DECODER_H
