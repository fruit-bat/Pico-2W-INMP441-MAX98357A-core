#include "arm_const_structs.h"
#include "chirp_test_rig.h"

static float32_t complex_dechirp_vector[CHIRP_DECODER_COMPLEX_LEN(RIG_N)];
static float32_t complex_dechirp_half_vector[CHIRP_DECODER_COMPLEX_LEN(RIG_N)];
static float32_t fft_half_buffer[CHIRP_DECODER_COMPLEX_LEN(RIG_N)];
static float32_t fft_output_buffer[CHIRP_DECODER_COMPLEX_LEN(RIG_N)];
static float32_t fft_magnitude_buffer[RIG_N];
static float32_t folded_power_buffer[RIG_BINS];
static float32_t fine_power_buffer[CHIRP_DECODER_FINE_LEN(RIG_BINS)];
static float32_t cos_table[RIG_BINS];
static float32_t sin_table[RIG_BINS];

ChirpDecoderState_t rig_decoder;
ChirpEncoder_t rig_encoder;
ChirpSymbolRxConfig_t rig_rx_config;
ChirpChannelTx_t rig_tx_model;
uint32_t rig_bins_per_symbol;
uint32_t rig_m;

void rig_setup(void) {
    rig_setup_spacing(1);
}

void rig_setup_spacing(uint32_t bins_per_symbol) {
    rig_bins_per_symbol = bins_per_symbol;
    rig_m = RIG_BINS / bins_per_symbol;
    const double bw = RIG_FS / RIG_N * RIG_BINS; // The chirp sweeps RIG_BINS FFT bins
    const double t = (double)RIG_N / RIG_FS;   // Symbol duration
    const ChirpParams_t params = {
        .sample_rate = (float32_t)RIG_FS,
        .f0 = (float32_t)RIG_F0,
        .chirp_rate = (float32_t)(bw / t),
    };
    chirp_decoder_init(&rig_decoder, &params, RIG_N, &arm_cfft_sR_f32_len1024,
        4, RIG_BINS, 3,
        complex_dechirp_vector, complex_dechirp_half_vector, fft_half_buffer,
        fft_output_buffer, fft_magnitude_buffer, folded_power_buffer, fine_power_buffer,
        cos_table, sin_table);
    chirp_encoder_init(&rig_encoder, rig_decoder.complex_dechirp_vector, RIG_N, rig_m, (float32_t)RIG_VOLUME);
    chirp_symbol_rx_config_init(&rig_rx_config, rig_m);

    rig_tx_model = (ChirpChannelTx_t){
        .sample_rate = RIG_FS,
        .f0 = RIG_F0,
        .chirp_rate = bw / t,
        .samples_per_symbol = RIG_N,
        .number_of_symbols = rig_m,
        .volume = RIG_VOLUME,
    };
}

void rig_decode(const float *samples, ChirpSymbolRx_t *rx) {
    chirp_decoder_real_chirp_to_symbol(&rig_decoder, samples, &rig_rx_config, rx);
}

double rig_timing_error_samples(const ChirpSymbolRx_t *rx) {
    return rx->error_angle * rig_rx_config.symbols_per_radian * (double)(RIG_N / rig_m);
}
