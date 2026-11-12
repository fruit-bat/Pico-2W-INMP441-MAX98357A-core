#include <math.h>
#include "chirp_channel.h"

static uint64_t rng_state = 0x9E3779B97F4A7C15ull;

void chirp_channel_seed(uint64_t seed) {
    rng_state = seed ? seed : 0x9E3779B97F4A7C15ull;
}

// xorshift64*
uint32_t chirp_channel_rand_u32(void) {
    rng_state ^= rng_state >> 12;
    rng_state ^= rng_state << 25;
    rng_state ^= rng_state >> 27;
    return (uint32_t)((rng_state * 0x2545F4914F6CDD1Dull) >> 32);
}

// Box-Muller
double chirp_channel_gaussian(void) {
    double u1;
    do {
        u1 = (chirp_channel_rand_u32() + 0.5) / 4294967296.0;
    } while (u1 <= 0.0);
    const double u2 = (chirp_channel_rand_u32() + 0.5) / 4294967296.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

double chirp_channel_tx_sample(
    const ChirpChannelTx_t *tx,
    const uint32_t *symbols,
    uint32_t symbol_count,
    double u
) {
    const double n = (double)tx->samples_per_symbol;
    if (u < 0.0) return 0.0;
    const double k = floor(u / n);
    if (k >= (double)symbol_count) return 0.0;

    const uint32_t symbol = symbols[(uint32_t)k] % tx->number_of_symbols;
    const double shift = (double)symbol * n / (double)tx->number_of_symbols;
    const double v = fmod((u - k * n) + shift, n);
    const double t = v / tx->sample_rate;
    const double phase = 2.0 * M_PI * (tx->f0 * t + 0.5 * tx->chirp_rate * t * t);
    return tx->volume * cos(phase);
}

void chirp_channel_rx_frame(
    const ChirpChannelTx_t *tx,
    const ChirpChannel_t *channel,
    const uint32_t *symbols,
    uint32_t symbol_count,
    uint32_t frame,
    float *out
) {
    const uint32_t n = tx->samples_per_symbol;
    for (uint32_t j = 0; j < n; j++) {
        const double rx_index = (double)frame * (double)n + (double)j;
        const double u = rx_index * channel->clock_scale + channel->offset;
        double x = chirp_channel_tx_sample(tx, symbols, symbol_count, u);
        for (uint32_t e = 0; e < channel->echo_count; e++) {
            x += channel->echoes[e].gain *
                 chirp_channel_tx_sample(tx, symbols, symbol_count, u - channel->echoes[e].delay);
        }
        if (channel->noise_rms > 0.0) {
            x += channel->noise_rms * chirp_channel_gaussian();
        }
        out[j] = (float)x;
    }
}
