/**
 * @file test_channel_template.c
 *
 * Tests for the channel-matched decision (channel_template.c), on a channel
 * modelled on the two-board measurements: the desk's two-path smear plus
 * room reflections 240 and 672 samples late, as strong as the direct path.
 * Data changes every frame, so the late reflections also carry part of the
 * previous symbol into each frame.
 */

#include <stdio.h>
#include <math.h>
#include "channel_template.h"
#include "chirp_test_rig.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

#define N RIG_N
#define BINS RIG_BINS

static float32_t template_storage[CHANNEL_TEMPLATE_STORAGE_LEN(BINS)];
static ChannelTemplate_t tmpl;

static const ChirpEcho_t room_echoes[] = { { 32.0, 0.9 }, { 240.0, 1.0 }, { 672.0, 1.0 } };

enum { STREAM = 460, TRAIN_FROM = 2, DATA_FROM = 40 };
static uint32_t stream[STREAM];

typedef struct {
    uint32_t frames;        // Data frames decided
    uint32_t strongest;     // Errors deciding from the decoder's symbol (refined centre of the strongest bin),
                            // after removing its most common offset (which a preamble would)
    uint32_t matched;       // Errors with the channel-matched decision
} Result_t;

// Sends stream[] through the channel with the window offset by 'offset'
// samples (0: aligned with the transmitter's frames), learns from 'train'
// frames, then decides the data frames both ways. Training frames carry the
// stream's own symbols, or symbol 0 throughout if train_on_zeros.
static Result_t run(const ChirpEcho_t *echoes, uint32_t echo_count, double snr_db,
                    double offset, uint32_t train, int train_on_zeros) {
    static float32_t frame[N];
    chirp_channel_seed(4);
    for (uint32_t k = 0; k < STREAM; k++) {
        stream[k] = (train_on_zeros && k < DATA_FROM) ? 0 : chirp_channel_rand_u32() % rig_m;
    }
    const ChirpChannel_t ch = {
        .offset = offset, .clock_scale = 1.0,
        .noise_rms = snr_db > 90.0 ? 0.0 : (RIG_VOLUME / sqrt(2.0)) / pow(10.0, snr_db / 20.0),
        .echoes = echoes, .echo_count = echo_count,
    };
    // With the window offset by whole symbol steps, symbol s lands on bin
    // (s * bins_per_symbol - offset / 16); the receiver gets this rotation
    // from the preamble
    const int32_t shift = (int32_t)lround(offset / (N / BINS));
    const uint32_t rotation = (uint32_t)(((-shift) % (int32_t)BINS + (int32_t)BINS) % (int32_t)BINS);

    channel_template_reset(&tmpl);
    Result_t r = { 0, 0, 0 };
    uint32_t offsets[64] = { 0 }; // decoder symbol - sent, mod rig_m
    for (uint32_t k = TRAIN_FROM; k < STREAM - 1; k++) {
        chirp_channel_rx_frame(&rig_tx_model, &ch, stream, STREAM, k, frame);
        ChirpSymbolRx_t rx;
        rig_decode(frame, &rx);
        const uint32_t sent = stream[k];
        if (k < TRAIN_FROM + train) {
            channel_template_learn(&tmpl, rig_decoder.folded_power_buffer, (sent * rig_bins_per_symbol + rotation) % BINS);
            continue;
        }
        if (k < DATA_FROM) continue;
        r.frames++;
        offsets[(rx.symbol_index + rig_m - sent) % rig_m]++;
        const uint32_t s = channel_template_decide(&tmpl, rig_decoder.folded_power_buffer,
            rotation, rig_bins_per_symbol, rig_m, NULL);
        if (s != sent) r.matched++;
    }
    uint32_t most = 0;
    for (uint32_t i = 0; i < rig_m; i++) if (offsets[i] > most) most = offsets[i];
    r.strongest = r.frames - most;
    return r;
}

static void print_result(const char *label, Result_t r) {
    printf("  %-44s strongest %3u  matched %3u   (errors/%u)\n", label, r.strongest, r.matched, r.frames);
}

// With nothing learned, the decision is the strongest candidate bin, so on a
// clean channel it already decodes everything
static void test_untrained(void) {
    rig_setup_spacing(1);
    const Result_t r = run(NULL, 0, 100.0, 0.0, 0, 0);
    print_result("clean, nothing learned", r);
    CHECK_EQ_U(r.matched, 0);
    CHECK_EQ_U(r.strongest, 0);
}

// On a clean channel learning changes nothing
static void test_clean_channel(void) {
    rig_setup_spacing(1);
    const Result_t r = run(NULL, 0, -9.0, 0.0, 4, 0);
    print_result("clean, -9 dB, 4 frames learned", r);
    CHECK_EQ_U(r.matched, 0);
}

// The room channel: the strongest bin is often a reflection, and the
// matched decision collects every path.
//
// Baseline (errors/419, strongest after removing its constant offset /
// matched): no noise 59 / 0, 0 dB 60 / 0, -6 dB 77 / 0; -12 dB 134 / 0
// (test_training_length).
static void test_room_channel(void) {
    rig_setup_spacing(1);
    const double snrs[] = { 100.0, 0.0, -6.0 };
    for (unsigned i = 0; i < 3; i++) {
        const Result_t r = run(room_echoes, 3, snrs[i], 0.0, 8, 0);
        char label[64];
        if (snrs[i] > 90.0) snprintf(label, sizeof label, "room, no noise, 8 frames learned");
        else snprintf(label, sizeof label, "room, %+.0f dB, 8 frames learned", snrs[i]);
        print_result(label, r);
        CHECK(r.matched * 10 < r.strongest);      // Much better than the strongest bin
        CHECK(r.matched * 100 <= r.frames);       // And at most 1% wrong
    }
}

// How many learned frames are enough (at -12 dB, where it starts to matter)
static void test_training_length(void) {
    rig_setup_spacing(1);
    const uint32_t counts[] = { 1, 2, 4, 8, 16 };
    for (unsigned i = 0; i < 5; i++) {
        const Result_t r = run(room_echoes, 3, -12.0, 0.0, counts[i], 0);
        char label[64];
        snprintf(label, sizeof label, "room, -12 dB, %u frame(s) learned", counts[i]);
        print_result(label, r);
        if (counts[i] >= 4) CHECK(r.matched * 100 <= r.frames);
    }
}

// Learning from a repeated symbol (as a preamble of symbol 0 would give):
// the reflections then carry the same symbol into each frame and add fully,
// which is not what they do with changing data
static void test_train_on_repeated_symbol(void) {
    rig_setup_spacing(1);
    const Result_t varying = run(room_echoes, 3, 0.0, 0.0, 8, 0);
    const Result_t zeros = run(room_echoes, 3, 0.0, 0.0, 8, 1);
    print_result("room, 0 dB, learned from changing symbols", varying);
    print_result("room, 0 dB, learned from symbol 0 repeated", zeros);
    CHECK(varying.matched <= zeros.matched);
}

// The window must be aligned with the transmitter's frames. Offset by half a
// frame (32 symbol steps, still on the symbol grid), a frame holds equal
// halves of two symbols and cannot be decided; shown on the desk channel
// alone. With the room's late reflections, which carry more of the earlier
// symbol into each frame, the model still decodes a misaligned window; on
// the boards, misaligned runs got 45..54% right against 99% for an aligned
// run at similar SNR, so the receiver aligns to the transmitter's frames.
//
// Baseline (errors/419, matched): desk aligned 0, 4 steps off 0, half a
// frame off ~200; room half a frame off 0.
static void test_alignment_matters(void) {
    rig_setup_spacing(1);
    const Result_t aligned = run(room_echoes, 1, 0.0, 0.0, 8, 0);
    const Result_t step = run(room_echoes, 1, 0.0, 64.0, 8, 0);
    const Result_t half = run(room_echoes, 1, 0.0, 512.0, 8, 0);
    const Result_t room_half = run(room_echoes, 3, 0.0, 512.0, 8, 0);
    print_result("desk, 0 dB, window aligned", aligned);
    print_result("desk, 0 dB, window 4 steps (64 samples) off", step);
    print_result("desk, 0 dB, window half a frame off", half);
    print_result("room, 0 dB, window half a frame off", room_half);
    CHECK_EQ_U(aligned.matched, 0);
    CHECK(half.matched * 4 > half.frames); // Over a quarter wrong
}

// Wider symbol spacing: candidates every 2 or 4 bins
static void test_spacing(void) {
    const uint32_t spacings[] = { 2, 4 };
    for (unsigned i = 0; i < 2; i++) {
        rig_setup_spacing(spacings[i]);
        const Result_t r = run(room_echoes, 3, -6.0, 0.0, 8, 0);
        char label[64];
        snprintf(label, sizeof label, "room, -6 dB, %u bins/symbol", spacings[i]);
        print_result(label, r);
        CHECK(r.matched * 100 <= r.frames);
    }
}

int main(void) {
    channel_template_init(&tmpl, BINS, template_storage);

    RUN_TEST(test_untrained);
    RUN_TEST(test_clean_channel);
    RUN_TEST(test_room_channel);
    RUN_TEST(test_training_length);
    RUN_TEST(test_train_on_repeated_symbol);
    RUN_TEST(test_alignment_matters);
    RUN_TEST(test_spacing);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
