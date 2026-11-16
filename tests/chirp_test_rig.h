#ifndef CHIRP_TEST_RIG_H
#define CHIRP_TEST_RIG_H

/**
 * @file chirp_test_rig.h
 *
 * Shared set-up for the host tests: an encoder, decoder and channel model
 * configured with the same parameters as the firmware (see main.c).
 */

#include "arm_math.h"
#include "chirp_decoder.h"
#include "chirp_encoder.h"
#include "chirp_symbol_rx.h"
#include "chirp_channel.h"

// Firmware parameters (main.c)
#define RIG_N   1024u                    // FFT_SIZE: samples per symbol
#define RIG_BINS 64u                     // CHIRP_BINS: FFT bins swept by the chirp
#define RIG_M   64u                      // NUM_SYMBOLS at the default spacing of 1 bin per symbol
#define RIG_SAMPLES_PER_STEP (RIG_N / RIG_M) // Cyclic shift between adjacent symbols (default spacing)
#define RIG_FS  44100.0
#define RIG_F0  15000.0
#define RIG_VOLUME 0.005

extern ChirpDecoderState_t rig_decoder;
extern ChirpEncoder_t rig_encoder;
extern ChirpSymbolRxConfig_t rig_rx_config;
extern ChirpChannelTx_t rig_tx_model;

// Symbol spacing in use (BINS_PER_SYMBOL) and the resulting number of symbols
extern uint32_t rig_bins_per_symbol;
extern uint32_t rig_m;

// Initializes everything above at 1 bin per symbol (RIG_M symbols); call once before use
void rig_setup(void);

// As rig_setup(), with symbols spaced bins_per_symbol bins apart
// (RIG_BINS / bins_per_symbol symbols); may be called again to change it
void rig_setup_spacing(uint32_t bins_per_symbol);

// Decodes one frame of RIG_N real samples
void rig_decode(const float *samples, ChirpSymbolRx_t *rx);

// Decoded position relative to the nearest symbol, in samples. Positive means
// the receive window started late (as main.c: increase the delay to correct).
double rig_timing_error_samples(const ChirpSymbolRx_t *rx);

#endif // CHIRP_TEST_RIG_H
