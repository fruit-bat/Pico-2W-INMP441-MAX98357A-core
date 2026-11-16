/**
 * @file test_spacing.c
 *
 * The receive path at each symbol spacing (BINS_PER_SYMBOL in main.c): 1, 2
 * and 4 FFT bins per symbol, i.e. 64, 32 and 16 symbols over the same chirp.
 *
 * For each spacing: every symbol round-trips, the timing estimate is linear
 * over +/- half a symbol step, symbol errors against SNR, and the tracking
 * loop on a desk-like channel (two near-equal paths 32 samples apart, which
 * smears each symbol into lobes a bin either side, as measured on hardware).
 */

#include <stdio.h>
#include <math.h>
#include "chirp_test_rig.h"
#include "rx_sample_window.h"
#include "symbol_timing_tracker.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

#define N RIG_N

// Firmware tracking settings (main.c)
#define FW_ALPHA          0.25f
#define FW_BETA           0.0f
#define FW_MAX_RATE       0.5f
#define FW_RATE_WINDOW    4.0f
#define FW_MIN_MAGNITUDE  0.8f

static const uint32_t spacings[] = { 1, 2, 4 };
#define SPACING_COUNT (sizeof spacings / sizeof spacings[0])

// Desk-like channel: a second path 32 samples (2 bins) later, nearly as strong
static const ChirpEcho_t desk_echo[] = { { 32.0, 0.9 } };

// Every symbol decodes when aligned, with a small timing error
static void test_round_trip(void) {
    static float32_t frame[N];
    static uint32_t stream[64];
    for (unsigned i = 0; i < SPACING_COUNT; i++) {
        rig_setup_spacing(spacings[i]);
        for (uint32_t s = 0; s < rig_m; s++) stream[s] = s;
        const ChirpChannel_t ch = { .offset = 0.0, .clock_scale = 1.0 };
        uint32_t errors = 0;
        double worst = 0.0;
        for (uint32_t s = 0; s < rig_m; s++) {
            chirp_channel_rx_frame(&rig_tx_model, &ch, stream, rig_m, s, frame);
            ChirpSymbolRx_t rx;
            rig_decode(frame, &rx);
            if (rx.symbol_index != s) errors++;
            const double e = fabs(rig_timing_error_samples(&rx));
            if (e > worst) worst = e;
        }
        printf("  %u bin(s)/symbol, %2u symbols: errors %u, worst |timing| %.3f samples\n",
            spacings[i], rig_m, errors, worst);
        CHECK_EQ_U(errors, 0);
        CHECK(worst < 0.2);
    }
}

// A window offset by d samples reads as timing error d (as main.c: positive
// means the window started late), with the right symbol, for any d within
// half a symbol step. Wider spacing gives a wider range.
//
// Baseline worst |measured - offset|: 0.28 (1 bin, +/-7), 0.52 (2 bins,
// +/-15), 0.44 (4 bins, +/-31) samples; the error grows a little far from
// the symbol, which the tracking loop removes anyway.
static void test_timing_range(void) {
    static float32_t frame[N];
    static uint32_t stream[64];
    for (unsigned i = 0; i < SPACING_COUNT; i++) {
        rig_setup_spacing(spacings[i]);
        const double step = (double)N / rig_m;
        for (uint32_t s = 0; s < rig_m; s++) stream[s] = s;
        uint32_t errors = 0;
        double worst = 0.0;
        for (double d = -(step / 2.0 - 1.0); d <= step / 2.0 - 1.0; d += 1.0) {
            const ChirpChannel_t ch = { .offset = d, .clock_scale = 1.0 };
            for (uint32_t s = 0; s < rig_m; s += 3) {
                chirp_channel_rx_frame(&rig_tx_model, &ch, stream, rig_m, s, frame);
                ChirpSymbolRx_t rx;
                rig_decode(frame, &rx);
                if (rx.symbol_index != s) { errors++; continue; }
                const double e = fabs(rig_timing_error_samples(&rx) - d);
                if (e > worst) worst = e;
            }
        }
        printf("  %u bin(s)/symbol: offsets +/-%.0f samples, errors %u, worst |measured - offset| %.3f\n",
            spacings[i], step / 2.0 - 1.0, errors, worst);
        CHECK_EQ_U(errors, 0);
        CHECK(worst < 0.6);
    }
}

// Symbol errors against SNR (as test_noise in test_chirp_codec.c)
//
// Baseline (errors/498 at -12 / -15 / -18 dB): 1 bin 14 / 73 / 204,
// 2 bins 0 / 13 / 111, 4 bins 0 / 3 / 84. All are error free to -9 dB.
static void test_noise(void) {
    enum { COUNT = 500 };
    static uint32_t stream[COUNT];
    static float32_t frame[N];
    printf("  SNR dB");
    for (unsigned i = 0; i < SPACING_COUNT; i++) printf("   %u bin(s)", spacings[i]);
    printf("   (errors/%u)\n", COUNT - 2);
    for (int snr_db = -6; snr_db >= -18; snr_db -= 3) {
        printf("  %+4d  ", snr_db);
        for (unsigned i = 0; i < SPACING_COUNT; i++) {
            rig_setup_spacing(spacings[i]);
            chirp_channel_seed(2);
            for (uint32_t k = 0; k < COUNT; k++) stream[k] = chirp_channel_rand_u32() % rig_m;
            const ChirpChannel_t ch = {
                .offset = 0.0, .clock_scale = 1.0,
                .noise_rms = (RIG_VOLUME / sqrt(2.0)) / pow(10.0, snr_db / 20.0),
            };
            uint32_t errors = 0;
            for (uint32_t k = 1; k < COUNT - 1; k++) {
                chirp_channel_rx_frame(&rig_tx_model, &ch, stream, COUNT, k, frame);
                ChirpSymbolRx_t rx;
                rig_decode(frame, &rx);
                if (rx.symbol_index != stream[k]) errors++;
            }
            printf("   %6u  ", errors);
            if (snr_db >= -9) CHECK_EQ_U(errors, 0);
        }
        printf("\n");
    }
}

// Closed loop (channel -> window -> decoder -> tracker) on the desk-like
// channel, starting the window off by start_offset samples. Returns the
// symbol errors in the second half, after removing a constant rotation
// (decision-directed tracking can lock a whole symbol over; a sync word
// resolves that), and the measured timing error's rms there.
static uint32_t desk_loop(double start_offset, double snr_db, double *rms_out) {
    enum { STREAM = 220, BLOCKS = 200 };
    static uint32_t stream[STREAM];
    static float32_t window_storage[RX_SAMPLE_WINDOW_STORAGE_LEN(N)];
    static float32_t frame[N];
    static int32_t sent[BLOCKS], decoded[BLOCKS];
    static double measured[BLOCKS];

    chirp_channel_seed(9);
    for (uint32_t k = 0; k < STREAM; k++) stream[k] = chirp_channel_rand_u32() % rig_m;
    const ChirpChannel_t ch = {
        .offset = start_offset, .clock_scale = 1.0,
        .noise_rms = (RIG_VOLUME / sqrt(2.0)) / pow(10.0, snr_db / 20.0),
        .echoes = desk_echo, .echo_count = 1,
    };

    RxSampleWindow_t window;
    SymbolTimingTracker_t tracker;
    rx_sample_window_init(&window, window_storage, N, 0);
    symbol_timing_tracker_init(&tracker, N, FW_ALPHA, FW_BETA, FW_MAX_RATE, FW_RATE_WINDOW, 0);

    for (uint32_t b = 0; b < BLOCKS; b++) {
        chirp_channel_rx_frame(&rig_tx_model, &ch, stream, STREAM, b, rx_sample_window_next_block(&window));
        sent[b] = -1;
        if (b == 0) continue;
        const uint32_t delay = rx_sample_window_get_delay(&window);
        rx_sample_window_read(&window, frame);
        ChirpSymbolRx_t rx;
        rig_decode(frame, &rx);
        // The symbol this frame mostly covers
        const double start = (double)b * N - delay + start_offset;
        const double k = floor(start / N + 0.5);
        if (k >= 0 && k < STREAM) sent[b] = (int32_t)stream[(uint32_t)k];
        decoded[b] = (int32_t)rx.symbol_index;
        measured[b] = rig_timing_error_samples(&rx);
        rx_sample_window_set_delay(&window,
            symbol_timing_tracker_update(&tracker, delay, (float)measured[b], rx.magnitude >= FW_MIN_MAGNITUDE));
    }

    const int32_t m = (int32_t)rig_m;
    uint32_t hist[64] = { 0 };
    for (uint32_t b = BLOCKS / 2; b < BLOCKS; b++) {
        if (sent[b] >= 0) hist[(uint32_t)((decoded[b] - sent[b] + m) % m)]++;
    }
    uint32_t rotation = 0;
    for (uint32_t r = 1; r < rig_m; r++) if (hist[r] > hist[rotation]) rotation = r;
    uint32_t errors = 0;
    double sum_sq = 0.0;
    uint32_t n = 0;
    for (uint32_t b = BLOCKS / 2; b < BLOCKS; b++) {
        if (sent[b] < 0) continue;
        if ((decoded[b] - sent[b] - (int32_t)rotation + 2 * m) % m != 0) errors++;
        sum_sq += measured[b] * measured[b];
        n++;
    }
    *rms_out = n ? sqrt(sum_sq / n) : 0.0;
    return errors;
}

// Wider spacing tolerates the desk channel's smear better, and pulls in from
// further off (half a symbol step: 8, 16, 32 samples)
//
// Baseline (errors/100 at 0 / -6 / -9 dB): 1 bin 0 / 0 / 1, 2 and 4 bins
// 0 / 0 / 0, from either start.
static void test_desk_channel_loop(void) {
    const double snrs[] = { 0.0, -6.0, -9.0 };
    printf("  spacing   start   errors at 0 / -6 / -9 dB   timing rms at -9 dB\n");
    for (unsigned i = 0; i < SPACING_COUNT; i++) {
        rig_setup_spacing(spacings[i]);
        const double step = (double)N / rig_m;
        const double starts[] = { -(step / 2.0 - 2.0), step / 2.0 - 2.0 };
        for (unsigned j = 0; j < 2; j++) {
            uint32_t errors[3];
            double rms = 0.0;
            for (unsigned s = 0; s < 3; s++) errors[s] = desk_loop(starts[j], snrs[s], &rms);
            printf("  %u bin(s)  %+6.0f   %3u / %3u / %3u               %.2f\n",
                spacings[i], starts[j], errors[0], errors[1], errors[2], rms);
            CHECK_EQ_U(errors[0], 0);
            if (spacings[i] >= 2) CHECK_EQ_U(errors[1], 0);
        }
    }
}

int main(void) {
    RUN_TEST(test_round_trip);
    RUN_TEST(test_timing_range);
    RUN_TEST(test_noise);
    RUN_TEST(test_desk_channel_loop);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
