/**
 * @file test_timing.c
 *
 * Host tests for symbol timing:
 *
 *  1. Unit tests of symbol_timing_tracker on its own (no DSP).
 *  2. Closed-loop tests that mirror the firmware's receive path:
 *       channel -> rx_sample_window -> decoder -> timing error -> tracker -> window delay
 *     with matched clocks, crystal-sized clock mismatch and motion-sized
 *     (Doppler) time scaling.
 *
 * Note on time scaling: in an acoustic link a sample-clock mismatch and
 * Doppler from constant relative motion are the same thing, a time scale of
 * (1 + eps). The channel models it exactly with clock_scale, frequency shift
 * included. For an up-chirp that frequency shift looks like an extra timing
 * offset of roughly eps * (f0 + BW) / bin_width bins, so the loop settles where
 * the *measured* error is zero, which is not exactly the true symbol boundary.
 * If that bias passes half a symbol step the loop locks one step over and
 * every symbol decodes rotated by one. Decision-directed tracking cannot tell
 * the difference; resolving it is the job of a packet's sync word. So the
 * closed-loop tests check the measured error and the symbol errors after
 * removing a constant rotation, and print the true offset and rotation.
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
#define M RIG_M

// Firmware tracking parameters (main.c)
#define FW_ALPHA          0.25f
#define FW_BETA           0.0f
#define FW_MAX_RATE       0.5f
#define FW_RATE_WINDOW    4.0f
#define FW_MIN_MAGNITUDE  0.8f

// ---------------------------------------------------------------------------
// 1. Tracker unit tests
//
// A simulated target delay D (possibly drifting); each frame the error fed to
// the tracker is D minus the delay the frame was received with.
// ---------------------------------------------------------------------------

static void test_tracker_first_order_converges(void) {
    SymbolTimingTracker_t t;
    symbol_timing_tracker_init(&t, N, 0.25f, 0.0f, 0.5f, 4.0f, 90);
    uint32_t out = 90;
    for (int i = 0; i < 40; i++) {
        out = symbol_timing_tracker_update(&t, out, (float)(100.0 - out), true);
    }
    CHECK_EQ_U(out, 100);
    CHECK_NEAR(t.rate, 0.0, 1e-6);
}

static void test_tracker_negative_error(void) {
    SymbolTimingTracker_t t;
    symbol_timing_tracker_init(&t, N, 0.25f, 0.0f, 0.5f, 4.0f, 110);
    uint32_t out = 110;
    for (int i = 0; i < 40; i++) {
        out = symbol_timing_tracker_update(&t, out, (float)(100.0 - out), true);
    }
    CHECK_EQ_U(out, 100);
}

// Steady drift: first order lags by rate/alpha; second order learns the rate
static void test_tracker_drift(void) {
    const double drift = 0.3; // Samples per frame
    for (int order = 1; order <= 2; order++) {
        SymbolTimingTracker_t t;
        const float beta = order == 1 ? 0.0f : 0.01f;
        symbol_timing_tracker_init(&t, N, 0.25f, beta, 0.5f, 4.0f, 100);
        uint32_t out = 100;
        double target = 100.0, lag_sum = 0.0;
        const int frames = 600, averaged = 100;
        for (int i = 0; i < frames; i++) {
            const double err = target - out;
            if (i >= frames - averaged) lag_sum += err;
            out = symbol_timing_tracker_update(&t, out, (float)err, true);
            target += drift;
        }
        const double lag = lag_sum / averaged;
        printf("  order %d: mean lag %.3f samples, rate %.4f\n", order, lag, t.rate);
        if (order == 1) {
            CHECK_NEAR(lag, drift / 0.25 + drift, 0.6); // Error measured before the update
            CHECK_NEAR(t.rate, 0.0, 1e-6);
        } else {
            CHECK_NEAR(t.rate, drift, 0.02);
            CHECK_NEAR(lag, 0.0, 0.6);
        }
    }
}

static void test_tracker_rate_clamped(void) {
    SymbolTimingTracker_t t;
    symbol_timing_tracker_init(&t, N, 0.25f, 0.01f, 0.5f, 100.0f, 100);
    uint32_t out = 100;
    double target = 100.0;
    for (int i = 0; i < 300; i++) {
        out = symbol_timing_tracker_update(&t, out, (float)(target - out), true);
        target += 1.0;
    }
    CHECK_NEAR(t.rate, 0.5, 1e-6);
}

static void test_tracker_rate_window(void) {
    SymbolTimingTracker_t t;
    symbol_timing_tracker_init(&t, N, 0.25f, 0.01f, 0.5f, 4.0f, 100);
    symbol_timing_tracker_update(&t, 100, 5.0f, true); // Outside the window: no rate update
    CHECK_NEAR(t.rate, 0.0, 1e-9);
    symbol_timing_tracker_update(&t, t.output, 3.0f, true); // Inside
    CHECK_NEAR(t.rate, 0.03, 1e-6);
}

// Invalid frames ignore the error but keep coasting at the learned rate
static void test_tracker_invalid_coasts(void) {
    SymbolTimingTracker_t t;
    symbol_timing_tracker_init(&t, N, 0.25f, 0.0f, 0.5f, 4.0f, 100);
    t.rate = 0.5f;
    uint32_t out = 100;
    for (int i = 0; i < 10; i++) {
        out = symbol_timing_tracker_update(&t, out, 1000.0f, false);
    }
    CHECK_EQ_U(out, 105);
}

static void test_tracker_wraps(void) {
    SymbolTimingTracker_t t;
    symbol_timing_tracker_init(&t, N, 1.0f, 0.0f, 0.5f, 4.0f, N - 1);
    CHECK_EQ_U(symbol_timing_tracker_update(&t, N - 1, 3.0f, true), 2);
    CHECK_EQ_U(symbol_timing_tracker_update(&t, 2, -5.0f, true), N - 3);
}

// A delay changed elsewhere (e.g. by the user) restarts tracking from it
static void test_tracker_external_change(void) {
    SymbolTimingTracker_t t;
    symbol_timing_tracker_init(&t, N, 0.25f, 0.0f, 0.5f, 4.0f, 100);
    t.rate = 0.4f;
    const uint32_t out = symbol_timing_tracker_update(&t, 500, 0.0f, true);
    CHECK_EQ_U(out, 500);
    CHECK_NEAR(t.rate, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 2. Closed-loop tests
// ---------------------------------------------------------------------------

#define STREAM_LEN 420u
#define MAX_BLOCKS 400u

typedef struct {
    double clock_scale;   // TX samples per RX sample
    double offset0;       // TX time seen by RX sample 0 (positive: RX started late)
    double noise_rms;
    float alpha, beta, max_rate, rate_window;
    float initial_rate;   // Tracker rate at the start (e.g. as estimated from a preamble)
    uint32_t blocks;
    const uint32_t *stream; // STREAM_LEN symbols
} LoopConfig_t;

typedef struct {
    int converged_block;   // First block after which the true offset stays within 1 sample of its final mean (-1: never)
    double measured_mean;  // Steady state (last half): measured timing error, samples
    double measured_rms;
    double true_mean;      // Steady state: true offset of the window from the nearest symbol boundary
    int rotation;          // Most common (decoded - sent) mod M in steady state
    uint32_t symbol_errors; // Steady state, after removing the rotation
    uint32_t steady_frames;
    float rate;            // Tracker's final rate, samples per frame
} LoopResult_t;

static float window_storage[RX_SAMPLE_WINDOW_STORAGE_LEN(N)];
static float frame[N];

static void run_loop(const LoopConfig_t *cfg, LoopResult_t *res) {
    RxSampleWindow_t window;
    SymbolTimingTracker_t tracker;
    rx_sample_window_init(&window, window_storage, N, 0);
    symbol_timing_tracker_init(&tracker, N, cfg->alpha, cfg->beta, cfg->max_rate, cfg->rate_window, 0);
    tracker.rate = cfg->initial_rate;
    const ChirpChannel_t ch = { .offset = cfg->offset0, .clock_scale = cfg->clock_scale, .noise_rms = cfg->noise_rms };

    static double measured[MAX_BLOCKS], true_off[MAX_BLOCKS];
    static int32_t sent[MAX_BLOCKS], decoded[MAX_BLOCKS];

    for (uint32_t b = 0; b < cfg->blocks; b++) {
        chirp_channel_rx_frame(&rig_tx_model, &ch, cfg->stream, STREAM_LEN, b, rx_sample_window_next_block(&window));
        measured[b] = 0.0;
        sent[b] = -1;
        if (b == 0) continue; // The window's older block is still empty

        const uint32_t delay = rx_sample_window_get_delay(&window);
        rx_sample_window_read(&window, frame);
        ChirpSymbolRx_t rx;
        rig_decode(frame, &rx);

        // Ground truth: where this frame really started in TX time
        const double start = ((double)b * N - delay) * cfg->clock_scale + cfg->offset0;
        const double k = floor(start / N + 0.5);
        true_off[b] = start - k * N;
        if (k >= 0 && k < STREAM_LEN) sent[b] = (int32_t)cfg->stream[(uint32_t)k];
        decoded[b] = (int32_t)rx.symbol_index;

        const double err = rig_timing_error_samples(&rx);
        measured[b] = err;
        const bool valid = rx.magnitude >= FW_MIN_MAGNITUDE;
        rx_sample_window_set_delay(&window,
            symbol_timing_tracker_update(&tracker, delay, (float)err, valid));
    }

    const uint32_t first = cfg->blocks / 2;
    uint32_t hist[M] = { 0 };
    for (uint32_t b = first; b < cfg->blocks; b++) {
        if (sent[b] >= 0) hist[(uint32_t)((decoded[b] - sent[b] + (int32_t)M) % (int32_t)M)]++;
    }
    res->rotation = 0;
    for (uint32_t r = 1; r < M; r++) if (hist[r] > hist[res->rotation]) res->rotation = (int)r;
    if (res->rotation > (int)M / 2) res->rotation -= (int)M;

    double ms = 0, ms2 = 0, ts = 0;
    res->symbol_errors = 0;
    res->steady_frames = 0;
    for (uint32_t b = first; b < cfg->blocks; b++) {
        if (sent[b] < 0) continue;
        res->steady_frames++;
        ms += measured[b];
        ms2 += measured[b] * measured[b];
        ts += true_off[b];
        if ((decoded[b] - sent[b] - res->rotation + 2 * (int32_t)M) % (int32_t)M != 0) res->symbol_errors++;
    }
    const double n = res->steady_frames ? res->steady_frames : 1;
    res->measured_mean = ms / n;
    res->measured_rms = sqrt(ms2 / n);
    res->true_mean = ts / n;
    res->rate = tracker.rate;

    res->converged_block = 1;
    for (int b = (int)cfg->blocks - 1; b >= 1; b--) {
        if (fabs(true_off[b] - res->true_mean) > 1.0) { res->converged_block = b + 1; break; }
    }
    if (res->converged_block >= (int)cfg->blocks) res->converged_block = -1;
}

static void print_result(const char *label, const LoopResult_t *r) {
    printf("  %-28s converged@%4d  measured mean %+6.3f rms %5.3f  true %+7.3f  rot %+d  errors %u/%u  rate %+.4f\n",
        label, r->converged_block, r->measured_mean, r->measured_rms, r->true_mean,
        r->rotation, r->symbol_errors, r->steady_frames, r->rate);
}

static uint32_t random_stream[STREAM_LEN];
static uint32_t zero_stream[STREAM_LEN];

static LoopConfig_t firmware_config(void) {
    return (LoopConfig_t){
        .clock_scale = 1.0, .offset0 = 0.0, .noise_rms = 0.0,
        .alpha = FW_ALPHA, .beta = FW_BETA, .max_rate = FW_MAX_RATE, .rate_window = FW_RATE_WINDOW,
        .blocks = 200, .stream = random_stream,
    };
}

// Matched clocks, window starting late or early: locks and decodes cleanly
static void test_loop_acquire_matched_clocks(void) {
    const double offsets[] = { -7.0, -3.0, 3.0, 7.0 };
    for (unsigned i = 0; i < sizeof offsets / sizeof offsets[0]; i++) {
        LoopConfig_t cfg = firmware_config();
        cfg.offset0 = offsets[i];
        LoopResult_t r;
        run_loop(&cfg, &r);
        char label[40];
        snprintf(label, sizeof label, "offset %+.0f", offsets[i]);
        print_result(label, &r);
        CHECK(r.converged_block >= 0 && r.converged_block < 50);
        CHECK_NEAR(r.measured_mean, 0.0, 0.5);
        CHECK_EQ_U(r.symbol_errors, 0);
        CHECK(r.rotation == 0);
    }
}

// Same, but with a stream of zeros (a preamble). Known limitation: symbol 0
// barely reports small offsets (see test_chirp_codec.c), so the loop stalls
// well short of the true boundary. Only the decisions are checked.
static void test_loop_acquire_zero_preamble(void) {
    const double offsets[] = { -7.0, -3.0, 3.0, 7.0 };
    for (unsigned i = 0; i < sizeof offsets / sizeof offsets[0]; i++) {
        LoopConfig_t cfg = firmware_config();
        cfg.offset0 = offsets[i];
        cfg.stream = zero_stream;
        LoopResult_t r;
        run_loop(&cfg, &r);
        char label[40];
        snprintf(label, sizeof label, "zeros, offset %+.0f", offsets[i]);
        print_result(label, &r);
        CHECK_EQ_U(r.symbol_errors, 0);
    }
}

// Crystal-sized clock mismatch (+/-100 ppm, about 0.1 samples per frame)
static void test_loop_crystal_drift(void) {
    const double ppm[] = { -100.0, 100.0 };
    for (int order = 1; order <= 2; order++) {
        for (unsigned i = 0; i < 2; i++) {
            LoopConfig_t cfg = firmware_config();
            cfg.clock_scale = 1.0 + ppm[i] * 1e-6;
            cfg.blocks = 400;
            if (order == 2) cfg.beta = 0.01f;
            LoopResult_t r;
            run_loop(&cfg, &r);
            char label[40];
            snprintf(label, sizeof label, "%+.0f ppm, %s", ppm[i], order == 1 ? "1st order" : "2nd order");
            print_result(label, &r);
            CHECK_EQ_U(r.symbol_errors, 0);
            CHECK_NEAR(r.measured_mean, 0.0, order == 1 ? 1.0 : 0.3);
            if (order == 2) CHECK_NEAR(r.rate, ppm[i] * 1e-6 * N, 0.03);
        }
    }
}

// Motion-sized time scaling: 0.2% is about 0.7 m/s, a drift of ~2 samples
// per frame. Starting from zero rate the loop cannot pull in: the lag passes
// half a symbol step before the rate is learned (and the firmware's max_rate of
// 0.5 samples/frame, ~490 ppm or ~0.17 m/s, is too low anyway). That case is
// only printed. With the rate seeded, as a preamble-based estimate would do,
// it tracks; the frequency shift makes it lock one symbol step over (rot +/-1),
// which a sync word would resolve.
static void test_loop_motion(void) {
    const double eps[] = { -0.002, 0.002 };
    for (int seeded = 0; seeded <= 1; seeded++) {
        for (unsigned i = 0; i < 2; i++) {
            LoopConfig_t cfg = firmware_config();
            cfg.clock_scale = 1.0 + eps[i];
            cfg.blocks = 400;
            cfg.beta = 0.01f;
            if (seeded) {
                cfg.max_rate = 4.0f;
                cfg.initial_rate = (float)(eps[i] * N);
            }
            LoopResult_t r;
            run_loop(&cfg, &r);
            char label[40];
            snprintf(label, sizeof label, "%+.1f%%, %s", eps[i] * 100.0, seeded ? "rate seeded" : "firmware");
            print_result(label, &r);
            if (seeded) {
                CHECK(r.converged_block >= 0 && r.converged_block < 100);
                CHECK_EQ_U(r.symbol_errors, 0);
                CHECK_NEAR(r.measured_mean, 0.0, 0.5);
                CHECK_NEAR(r.rate, eps[i] * N, 0.1);
            }
        }
    }
}

int main(void) {
    rig_setup();
    chirp_channel_seed(3);
    for (uint32_t i = 0; i < STREAM_LEN; i++) {
        random_stream[i] = chirp_channel_rand_u32() % M;
        zero_stream[i] = 0;
    }

    RUN_TEST(test_tracker_first_order_converges);
    RUN_TEST(test_tracker_negative_error);
    RUN_TEST(test_tracker_drift);
    RUN_TEST(test_tracker_rate_clamped);
    RUN_TEST(test_tracker_rate_window);
    RUN_TEST(test_tracker_invalid_coasts);
    RUN_TEST(test_tracker_wraps);
    RUN_TEST(test_tracker_external_change);

    RUN_TEST(test_loop_acquire_matched_clocks);
    RUN_TEST(test_loop_acquire_zero_preamble);
    RUN_TEST(test_loop_crystal_drift);
    RUN_TEST(test_loop_motion);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
