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
#define RIG_M   64u                      // MAX_SYMBOLS
#define RIG_SAMPLES_PER_STEP (RIG_N / RIG_M) // Cyclic shift between adjacent symbols
#define RIG_FS  44100.0
#define RIG_F0  15000.0
#define RIG_VOLUME 0.005

extern ChirpDecoderState_t rig_decoder;
extern ChirpEncoder_t rig_encoder;
extern ChirpSymbolRxConfig_t rig_rx_config;
extern ChirpChannelTx_t rig_tx_model;

// Initializes everything above; call once before use
void rig_setup(void);

// Decodes one frame of RIG_N real samples
void rig_decode(const float *samples, ChirpSymbolRx_t *rx);

// Decoded position relative to the nearest symbol, in samples. Positive means
// the receive window started late (as main.c: increase the delay to correct).
double rig_timing_error_samples(const ChirpSymbolRx_t *rx);

#endif // CHIRP_TEST_RIG_H
