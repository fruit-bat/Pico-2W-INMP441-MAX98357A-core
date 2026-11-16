/**
 * @file test_filter.c
 *
 * Tests for the microphone high-pass (biquad_filter.c): its frequency
 * response, block-by-block continuity, and what it does for the decoder when
 * the input carries strong low-frequency noise, as the INMP441's does.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "biquad_filter.h"
#include "chirp_test_rig.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

#define N RIG_N
#define M RIG_M

// Firmware settings (main.c)
#define HIGHPASS_HZ 10000.0f
#define HIGHPASS_Q  0.70710678f

static BiquadFilter_t filter;

static void init_filter(void) {
    biquad_filter_init_highpass(&filter, (float32_t)RIG_FS, HIGHPASS_HZ, HIGHPASS_Q);
}

// Steady-state gain in dB for a sine at freq_hz (DC when freq_hz is 0)
static double gain_db(double freq_hz) {
    enum { SETTLE = 8192, MEASURE = 8192 };
    static float32_t buf[SETTLE + MEASURE];
    for (uint32_t n = 0; n < SETTLE + MEASURE; n++) {
        buf[n] = freq_hz == 0.0 ? 1.0f : (float32_t)sin(2.0 * M_PI * freq_hz * n / RIG_FS);
    }
    init_filter();
    biquad_filter_process(&filter, buf, SETTLE + MEASURE);
    double in = 0.0, out = 0.0;
    for (uint32_t n = SETTLE; n < SETTLE + MEASURE; n++) {
        const double x = freq_hz == 0.0 ? 1.0 : sin(2.0 * M_PI * freq_hz * n / RIG_FS);
        in += x * x;
        out += (double)buf[n] * buf[n];
    }
    return 10.0 * log10(out / in + 1e-30);
}

// Blocks DC and low frequencies, -3 dB at the cutoff, passes the chirp band
static void test_highpass_response(void) {
    const double freqs[] = { 0.0, 50.0, 1000.0, 5000.0, 10000.0, 15000.0, 16400.0, 17800.0 };
    printf("  freq Hz   gain dB\n");
    for (unsigned i = 0; i < sizeof freqs / sizeof freqs[0]; i++) {
        printf("  %7.0f   %8.2f\n", freqs[i], gain_db(freqs[i]));
    }
    CHECK(gain_db(0.0) < -100.0);
    CHECK(gain_db(50.0) < -80.0);
    CHECK(gain_db(1000.0) < -35.0);
    CHECK_NEAR(gain_db(10000.0), -3.0, 0.5);
    CHECK(gain_db(15000.0) > -1.0);
    CHECK(gain_db(17800.0) > -0.5);
}

// Filtering block by block (as the firmware does, one I2S block at a time)
// gives exactly the same output as one call over the whole stream
static void test_block_continuity(void) {
    enum { LEN = 4 * N };
    static float32_t whole[LEN], blocks[LEN];
    chirp_channel_seed(11);
    for (uint32_t n = 0; n < LEN; n++) whole[n] = blocks[n] = (float32_t)chirp_channel_gaussian();

    init_filter();
    biquad_filter_process(&filter, whole, LEN);
    init_filter();
    for (uint32_t b = 0; b < LEN / N; b++) biquad_filter_process(&filter, &blocks[b * N], N);

    CHECK(memcmp(whole, blocks, sizeof whole) == 0);
}

// ---------------------------------------------------------------------------
// The decoder with strong low-frequency noise on the input
// ---------------------------------------------------------------------------

// Low-frequency noise: leaky integrated (brown) noise, about lf_rms
// overall, plus a DC offset, like a MEMS mic's output below the chirp band
typedef struct {
    double level;
    double step;
    double dc;
} LowFrequencyNoise_t;

#define LF_LEAK 0.9995

static void lf_init(LowFrequencyNoise_t *lf, double lf_rms) {
    lf->level = 0.0;
    lf->step = lf_rms * sqrt(1.0 - LF_LEAK * LF_LEAK);
    lf->dc = lf_rms;
    for (int n = 0; n < 20000; n++) lf->level = LF_LEAK * lf->level + lf->step * chirp_channel_gaussian();
}

static void lf_add(LowFrequencyNoise_t *lf, float32_t *frame) {
    for (uint32_t n = 0; n < N; n++) {
        lf->level = LF_LEAK * lf->level + lf->step * chirp_channel_gaussian();
        frame[n] += (float32_t)(lf->level + lf->dc);
    }
}

// Low-frequency noise ~30 dB above the in-band (white) noise, as estimated
// from the hardware's silent-speaker logs
#define LF_OVER_WHITE_DB 30.0

// With the speaker silent, the folded spectrum should be flat. Unfiltered,
// the low-frequency noise leaks in at the frame edges and piles up on folded
// bins ~3..8, which then win against weak symbols.
//
// Baseline: bins 3..8 average 1.23x the median unfiltered, 1.02x filtered
// (on the desk, silent speaker, unfiltered: 1.3..1.6x).
static void test_low_frequency_noise_floor(void) {
    enum { FRAMES = 300 };
    static float32_t frame[N];
    const double white_rms = RIG_VOLUME;
    for (int filtered = 0; filtered <= 1; filtered++) {
        double mean[RIG_BINS] = { 0 };
        chirp_channel_seed(21);
        LowFrequencyNoise_t lf;
        lf_init(&lf, white_rms * pow(10.0, LF_OVER_WHITE_DB / 20.0));
        init_filter();
        for (uint32_t f = 0; f < FRAMES; f++) {
            for (uint32_t n = 0; n < N; n++) frame[n] = (float32_t)(white_rms * chirp_channel_gaussian());
            lf_add(&lf, frame);
            if (filtered) biquad_filter_process(&filter, frame, N);
            if (f < 4) continue; // Let the filter settle
            ChirpSymbolRx_t rx;
            rig_decode(frame, &rx);
            for (uint32_t k = 0; k < RIG_BINS; k++) mean[k] += rig_decoder.folded_power_buffer[k];
        }
        double sorted[RIG_BINS];
        memcpy(sorted, mean, sizeof sorted);
        for (uint32_t i = 1; i < RIG_BINS; i++) {
            for (uint32_t j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
                const double t = sorted[j]; sorted[j] = sorted[j - 1]; sorted[j - 1] = t;
            }
        }
        const double median = 0.5 * (sorted[RIG_BINS / 2 - 1] + sorted[RIG_BINS / 2]);
        double bump = 0.0;
        for (uint32_t k = 3; k <= 8; k++) bump += mean[k];
        bump /= 6.0 * median;
        printf("  %-10s bins 3..8 / median = %.2f\n", filtered ? "filtered" : "unfiltered", bump);
        if (filtered) CHECK(bump < 1.1);
        else CHECK(bump > 1.15); // Shows the problem is reproduced
    }
}

// Symbol errors with chirps at low SNR on top of the low-frequency noise
//
// Baseline (494 frames): -9 dB: unfiltered 5 errors, filtered 0; -12 dB:
// unfiltered 45, filtered 30 (13 with no low-frequency noise at all, see
// test_noise in test_chirp_codec.c).
static void test_low_frequency_noise_errors(void) {
    enum { COUNT = 500 };
    static uint32_t stream[COUNT];
    static float32_t frame[N];
    const int snrs[] = { -6, -9, -12 };
    printf("  SNR dB  unfiltered  filtered   (errors/%u)\n", COUNT - 6);
    for (unsigned s = 0; s < sizeof snrs / sizeof snrs[0]; s++) {
        uint32_t errors[2];
        for (int filtered = 0; filtered <= 1; filtered++) {
            chirp_channel_seed(5);
            for (uint32_t i = 0; i < COUNT; i++) stream[i] = chirp_channel_rand_u32() % M;
            const double signal_rms = RIG_VOLUME / sqrt(2.0);
            const ChirpChannel_t ch = {
                .offset = 0.0, .clock_scale = 1.0,
                .noise_rms = signal_rms / pow(10.0, snrs[s] / 20.0),
            };
            LowFrequencyNoise_t lf;
            lf_init(&lf, ch.noise_rms * pow(10.0, LF_OVER_WHITE_DB / 20.0));
            init_filter();
            errors[filtered] = 0;
            for (uint32_t k = 1; k < COUNT - 1; k++) {
                chirp_channel_rx_frame(&rig_tx_model, &ch, stream, COUNT, k, frame);
                lf_add(&lf, frame);
                if (filtered) biquad_filter_process(&filter, frame, N);
                if (k < 5) continue; // Let the filter settle
                ChirpSymbolRx_t rx;
                rig_decode(frame, &rx);
                if (rx.symbol_index != stream[k]) errors[filtered]++;
            }
        }
        printf("  %+4d    %6u      %6u\n", snrs[s], errors[0], errors[1]);
        CHECK(errors[1] <= errors[0]);
        if (snrs[s] >= -9) CHECK_EQ_U(errors[1], 0);
    }
}

int main(void) {
    rig_setup();

    RUN_TEST(test_highpass_response);
    RUN_TEST(test_block_continuity);
    RUN_TEST(test_low_frequency_noise_floor);
    RUN_TEST(test_low_frequency_noise_errors);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
