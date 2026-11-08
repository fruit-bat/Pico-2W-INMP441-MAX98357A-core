#ifndef CHIRP_CHANNEL_H
#define CHIRP_CHANNEL_H

/**
 * @file chirp_channel.h
 *
 * Simulated transmitter and channel for the host tests.
 *
 * The transmitted signal is computed analytically (in double precision) from
 * the chirp formula rather than from the encoder's table, so it can be
 * evaluated at any real-valued time. That allows fractional timing offsets and
 * clock-rate mismatch without interpolation artefacts.
 *
 * Time is measured in transmitter samples. Symbol k of the stream occupies
 * [k*N, (k+1)*N). Symbol s is the base up-chirp cyclically shifted by
 * s*N/M samples, matching chirp_encoder.c:
 *
 *   x(u) = volume * cos(phase((u_local + s*N/M) mod N))
 *   phase(v) = 2*pi*(f0*v/fs + 0.5*chirp_rate*(v/fs)^2)
 *
 * The receiver's sample j of frame k is taken at transmitter time
 *
 *   u = (k*N + j) * clock_scale + offset
 *
 * so offset > 0 means the receive window starts late, and clock_scale > 1
 * means the receiver samples more slowly than the transmitter.
 *
 * No memory is allocated; the caller provides all buffers.
 */

#include <stdint.h>

typedef struct {
    double sample_rate;          // Hz
    double f0;                   // Chirp start frequency, Hz
    double chirp_rate;           // Hz per second
    uint32_t samples_per_symbol; // N
    uint32_t number_of_symbols;  // M
    double volume;               // Amplitude
} ChirpChannelTx_t;

typedef struct {
    double offset;       // Receive window start relative to the transmitter, in TX samples
    double clock_scale;  // TX samples per RX sample (1.0 = matched clocks)
    double noise_rms;    // Standard deviation of added white Gaussian noise
} ChirpChannel_t;

/**
 * Value of the transmitted signal at transmitter time u (in samples), for a
 * symbol stream. Times before 0 or beyond the stream give 0 (silence).
 */
double chirp_channel_tx_sample(
    const ChirpChannelTx_t *tx,
    const uint32_t *symbols,
    uint32_t symbol_count,
    double u
);

/**
 * Fills out[0 .. N-1] with receive frame 'frame' as seen through the channel.
 */
void chirp_channel_rx_frame(
    const ChirpChannelTx_t *tx,
    const ChirpChannel_t *channel,
    const uint32_t *symbols,
    uint32_t symbol_count,
    uint32_t frame,
    float *out
);

/**
 * Deterministic random numbers, so test runs are repeatable.
 */
void chirp_channel_seed(uint64_t seed);
uint32_t chirp_channel_rand_u32(void);
double chirp_channel_gaussian(void); // Zero mean, unit variance

#endif // CHIRP_CHANNEL_H
