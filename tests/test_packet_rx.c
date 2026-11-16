/**
 * @file test_packet_rx.c
 *
 * Closed-loop tests of the packet receiver's first stage (packet_rx.c):
 * packets from packet_tx go through the channel model, with silence around
 * them, and the receiver must find the preamble, place B on the right frame
 * and learn the channel, whatever the window's offset; and must not be
 * fooled by noise or a steady tone.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "chirp_test_rig.h"
#include "packet_rx.h"
#include "packet_tx.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

#define N RIG_N
#define BINS RIG_BINS

static float32_t rx_storage[PACKET_RX_STORAGE_LEN(BINS)];
static float32_t template_storage[CHANNEL_TEMPLATE_STORAGE_LEN(BINS)];
static ChannelTemplate_t tmpl;
static PacketRx_t prx;
static PacketTx_t ptx;

static const ChirpEcho_t room_echoes[] = { { 32.0, 0.9 }, { 240.0, 1.0 }, { 672.0, 1.0 } };

enum { MAX_STREAM = 2000, LEAD = 12 };
static uint32_t stream[MAX_STREAM];

// Builds: LEAD silent frames, a packet with a 16-byte payload, then silence
// up to 'length' frames. Returns the frame where B starts.
static uint32_t build_packet_stream(uint32_t length) {
    static const uint8_t payload[16] = "packet rx test!";
    for (uint32_t k = 0; k < length; k++) stream[k] = CHIRP_CHANNEL_SILENT;
    packet_tx_init(&ptx, (uint32_t)log2(rig_m));
    packet_tx_start(&ptx, payload, sizeof payload);
    uint32_t k = LEAD, s;
    while (packet_tx_next_symbol(&ptx, &s) && k < length) stream[k++] = s;
    return LEAD + PACKET_A_LEN;
}

typedef struct {
    uint32_t locked, trained, failed, tone;
    PacketRxStatus_t last_trained;
} Events_t;

// Runs 'length' frames of the stream through the channel and the receiver
static Events_t run(uint32_t length, double offset, double snr_db, const ChirpEcho_t *echoes, uint32_t echo_count) {
    static float32_t frame[N];
    const ChirpChannel_t ch = {
        .offset = offset, .clock_scale = 1.0,
        .noise_rms = snr_db > 90.0 ? 0.0 : (RIG_VOLUME / sqrt(2.0)) / pow(10.0, snr_db / 20.0),
        .echoes = echoes, .echo_count = echo_count,
    };
    packet_rx_init(&prx, BINS, (uint32_t)log2(rig_m), rx_storage, &tmpl);
    Events_t e;
    memset(&e, 0, sizeof e);
    for (uint32_t k = 0; k < length; k++) {
        chirp_channel_rx_frame(&rig_tx_model, &ch, stream, length, k, frame);
        ChirpSymbolRx_t rx;
        rig_decode(frame, &rx);
        const PacketRxStatus_t *st = packet_rx_frame(&prx, rig_decoder.folded_power_buffer);
        switch (st->event) {
            case PACKET_RX_EVENT_LOCKED: e.locked++; break;
            case PACKET_RX_EVENT_TRAINED: e.trained++; e.last_trained = *st; break;
            case PACKET_RX_EVENT_TRAIN_FAILED: e.failed++; break;
            case PACKET_RX_EVENT_TONE: e.tone++; break;
            default: break;
        }
    }
    return e;
}

static int32_t wrap_offset(int32_t o) {
    o %= (int32_t)BINS;
    if (o >= (int32_t)BINS / 2) o -= (int32_t)BINS;
    if (o < -(int32_t)BINS / 2) o += (int32_t)BINS;
    return o;
}

static void print_events(const char *label, Events_t e) {
    printf("  %-38s locked %u trained %u failed %u tone %u", label, e.locked, e.trained, e.failed, e.tone);
    if (e.trained) {
        printf(" | B at %u (adj %+d) coherence %.2f strongest %+d earliest %+d",
            e.last_trained.b_start_frame, e.last_trained.start_adjust, e.last_trained.coherence,
            e.last_trained.strongest_offset, e.last_trained.earliest_offset);
    }
    printf("\n");
}

// Clean channel, window at various offsets from the transmitter's frames:
// B is placed on the frame that holds most of B0, and the learned shape's
// peak is where the window sits relative to the symbols (a window d samples
// late sees every symbol d/16 bins high). Coherence falls as the window
// moves off, as each frame mixes two B symbols; near half a frame B's start
// is ambiguous by one frame (see packet_rx.h), so it is only checked within
// +/-400 samples.
//
// Baseline coherence: 1.00 (0), 0.98 (+37), 0.78 (-200), 0.64 (+300), 0.47 (+500).
static void test_offsets(void) {
    rig_setup_spacing(1);
    const double offsets[] = { 0.0, 37.0, -200.0, 300.0, 500.0 };
    for (unsigned i = 0; i < 5; i++) {
        const uint32_t b_frame = build_packet_stream(80);
        const Events_t e = run(80, offsets[i], 100.0, NULL, 0);
        char label[48];
        snprintf(label, sizeof label, "clean, window %+.0f samples", offsets[i]);
        print_events(label, e);
        CHECK_EQ_U(e.trained, 1);
        CHECK_EQ_U(e.failed, 0);
        if (e.trained) {
            if (fabs(offsets[i]) < 400.0) CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
            if (fabs(offsets[i]) < 50.0) CHECK(e.last_trained.coherence > 0.9);
            else CHECK(e.last_trained.coherence > 0.4);
            const int32_t expect = wrap_offset((int32_t)lround(offsets[i] / (N / BINS)));
            CHECK(abs(e.last_trained.strongest_offset - expect) <= 1);
        }
    }
}

// Noise
static void test_noise(void) {
    rig_setup_spacing(1);
    const double snrs[] = { 0.0, -6.0, -9.0, -12.0 };
    for (unsigned i = 0; i < 4; i++) {
        chirp_channel_seed(30 + i);
        const uint32_t b_frame = build_packet_stream(80);
        const Events_t e = run(80, 100.0, snrs[i], NULL, 0);
        char label[48];
        snprintf(label, sizeof label, "clean, %+.0f dB", snrs[i]);
        print_events(label, e);
        {
            CHECK_EQ_U(e.trained, 1);
            if (e.trained) CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
        }
    }
}

// The room: B still placed right; the strongest path may be a reflection,
// the earliest strong one is the direct path (offset 0 with the window aligned)
static void test_room(void) {
    rig_setup_spacing(1);
    const double snrs[] = { 100.0, 0.0, -6.0 };
    for (unsigned i = 0; i < 3; i++) {
        chirp_channel_seed(40 + i);
        const uint32_t b_frame = build_packet_stream(80);
        const Events_t e = run(80, 0.0, snrs[i], room_echoes, 3);
        char label[48];
        snprintf(label, sizeof label, "room, %s", snrs[i] > 90.0 ? "no noise" : (snrs[i] == 0.0 ? "0 dB" : "-6 dB"));
        print_events(label, e);
        CHECK_EQ_U(e.trained, 1);
        if (e.trained) {
            CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
            CHECK(abs(e.last_trained.earliest_offset) <= 1);
        }
    }
}

// Wider symbol spacing: B is the same sound
static void test_spacing(void) {
    rig_setup_spacing(2);
    const uint32_t b_frame = build_packet_stream(80);
    const Events_t e = run(80, 100.0, -6.0, room_echoes, 3);
    print_events("room, -6 dB, 2 bins/symbol", e);
    CHECK_EQ_U(e.trained, 1);
    if (e.trained) {
        CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
        CHECK(abs(e.last_trained.earliest_offset - 6) <= 1); // Window 100 samples late: direct path at +6
    }
    rig_setup_spacing(1);
}

// Noise alone never locks
static void test_noise_only(void) {
    rig_setup_spacing(1);
    for (uint32_t k = 0; k < 2000; k++) stream[k] = CHIRP_CHANNEL_SILENT;
    chirp_channel_seed(50);
    const Events_t e = run(2000, 0.0, 0.0, NULL, 0); // Noise at the level of a 0 dB signal
    print_events("noise only, 2000 frames", e);
    CHECK_EQ_U(e.locked, 0);
    CHECK_EQ_U(e.trained, 0);
}

// A steady tone (chirp test mode) is ignored; a packet after it is found
static void test_steady_tone(void) {
    rig_setup_spacing(1);
    const uint32_t b_frame = build_packet_stream(200) + 100;
    // Shift the packet back by 100 frames and put a held symbol 20 in front
    for (int32_t k = 199; k >= 100; k--) stream[k] = stream[k - 100];
    for (uint32_t k = 0; k < 100; k++) stream[k] = k < 90 ? 20 : CHIRP_CHANNEL_SILENT;
    const Events_t e = run(200, 0.0, -3.0, NULL, 0);
    print_events("90 frames of tone, then a packet", e);
    CHECK(e.tone >= 1);
    CHECK_EQ_U(e.trained, 1);
    if (e.trained) CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
}

int main(void) {
    channel_template_init(&tmpl, BINS, template_storage);

    RUN_TEST(test_offsets);
    RUN_TEST(test_noise);
    RUN_TEST(test_room);
    RUN_TEST(test_spacing);
    RUN_TEST(test_noise_only);
    RUN_TEST(test_steady_tone);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
