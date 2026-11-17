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
// The same delays at about the strengths the two boards learned (2026-11-17)
static const ChirpEcho_t measured_echoes[] = { { 32.0, 0.6 }, { 240.0, 0.4 }, { 672.0, 0.35 } };

enum { MAX_STREAM = 2000, LEAD = 12 };
static uint32_t stream[MAX_STREAM];

static const uint8_t test_payload[16] = "packet rx test!";
static const uint8_t *sent_payload = test_payload;
static uint32_t sent_length = sizeof test_payload;

// Builds: LEAD silent frames, a packet, then silence up to 'length' frames.
// Returns the frame where B starts.
static uint32_t build_packet_stream_with(uint32_t length, const uint8_t *payload, uint32_t payload_length) {
    sent_payload = payload;
    sent_length = payload_length;
    for (uint32_t k = 0; k < length; k++) stream[k] = CHIRP_CHANNEL_SILENT;
    packet_tx_init(&ptx, (uint32_t)log2(rig_m));
    packet_tx_start(&ptx, payload, payload_length);
    uint32_t k = LEAD, s;
    while (packet_tx_next_symbol(&ptx, &s) && k < length) stream[k++] = s;
    return LEAD + PACKET_A_LEN;
}

// A packet with the 16-byte test payload
static uint32_t build_packet_stream(uint32_t length) {
    return build_packet_stream_with(length, test_payload, sizeof test_payload);
}

typedef struct {
    uint32_t locked, trained, failed, tone;
    uint32_t header_failed, packets, good;   // good: CRC-16 ok and the payload is what was sent
    PacketRxStatus_t last_packet;
    int32_t last_align;           // align_bins at the last LOCKED
    double final_offset;          // Window offset at the end (after any moves)
    PacketRxStatus_t last_trained;
} Events_t;

// Runs 'length' frames of the stream through the channel and the receiver.
// With 'align', the window is moved when the receiver locks, as main.c does.
static Events_t run(uint32_t length, double offset, double snr_db, const ChirpEcho_t *echoes, uint32_t echo_count, bool align) {
    static float32_t frame[N];
    ChirpChannel_t ch = {
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
        const float32_t centre = rx.angle * (float32_t)BINS / (2.0f * (float32_t)M_PI);
        const PacketRxStatus_t *st = packet_rx_frame(&prx, rig_decoder.folded_power_buffer, centre);
        switch (st->event) {
            case PACKET_RX_EVENT_LOCKED:
                e.locked++;
                e.last_align = st->align_bins;
                if (align) {
                    ch.offset -= round((st->align_bins + st->align_fraction) * (double)(N / BINS)); // Earlier
                    packet_rx_window_moved(&prx, st->align_bins);
                }
                break;
            case PACKET_RX_EVENT_TRAINED: e.trained++; e.last_trained = *st; break;
            case PACKET_RX_EVENT_TRAIN_FAILED: e.failed++; break;
            case PACKET_RX_EVENT_TONE: e.tone++; break;
            case PACKET_RX_EVENT_HEADER_FAILED: e.header_failed++; break;
            case PACKET_RX_EVENT_PACKET:
                e.packets++;
                e.last_packet = *st;
                if (st->crc_ok && st->length == sent_length
                        && memcmp(packet_rx_payload(&prx), sent_payload, sent_length) == 0) e.good++;
                break;
            default: break;
        }
    }
    e.final_offset = ch.offset;
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
// moves off, as each frame mixes two B symbols. Near half a frame out B is
// unreliable without the window move (see packet_rx.h; whether it is found
// depends on the data after it), so that case is left to test_alignment.
//
// Baseline coherence: 1.00 (0), 0.98 (+37), 0.78 (-200), 0.64 (+300).
static void test_offsets(void) {
    rig_setup_spacing(1);
    const double offsets[] = { 0.0, 37.0, -200.0, 300.0 };
    for (unsigned i = 0; i < 4; i++) {
        const uint32_t b_frame = build_packet_stream(80);
        const Events_t e = run(80, offsets[i], 100.0, NULL, 0, false);
        char label[48];
        snprintf(label, sizeof label, "clean, window %+.0f samples", offsets[i]);
        print_events(label, e);
        CHECK_EQ_U(e.trained, 1);
        CHECK_EQ_U(e.failed, 0);
        if (e.trained) {
            CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
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
        const Events_t e = run(80, 100.0, snrs[i], NULL, 0, false);
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
        const Events_t e = run(80, 0.0, snrs[i], room_echoes, 3, false);
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
    const Events_t e = run(80, 100.0, -6.0, room_echoes, 3, false);
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
    const Events_t e = run(2000, 0.0, 0.0, NULL, 0, false); // Noise at the level of a 0 dB signal
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
    const Events_t e = run(200, 0.0, -3.0, NULL, 0, false);
    print_events("90 frames of tone, then a packet", e);
    CHECK(e.tone >= 1);
    CHECK_EQ_U(e.trained, 1);
    if (e.trained) CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
}

// Window alignment from A: whatever the window's offset, after the move it
// sits on the earliest path (whole bins from the path shape, the fraction
// from the decoder's refined centre: within 1 sample on a clean channel), B
// lands on the right frame with one symbol per frame, and the learned paths
// sit at 0
static void test_alignment(void) {
    rig_setup_spacing(1);
    const double offsets[] = { 37.0, -200.0, 300.0, 500.0, -500.0 };
    for (unsigned i = 0; i < 5; i++) {
        const uint32_t b_frame = build_packet_stream(80);
        const Events_t e = run(80, offsets[i], 100.0, NULL, 0, true);
        char label[48];
        snprintf(label, sizeof label, "aligned, clean, window %+.0f", offsets[i]);
        print_events(label, e);
        printf("    moved %+d bins, window now %+.0f samples\n", e.last_align, e.final_offset);
        CHECK(fabs(e.final_offset) <= 1.0);
        CHECK_EQ_U(e.trained, 1);
        CHECK_EQ_U(e.failed, 0);
        if (e.trained) {
            CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
            CHECK(e.last_trained.coherence > 0.9);
            CHECK(abs(e.last_trained.strongest_offset) <= 1);
        }
    }
    // The room, noisy: aligns on the direct path, not the strongest echo
    const double snrs[] = { 0.0, -6.0 };
    for (unsigned i = 0; i < 2; i++) {
        chirp_channel_seed(60 + i);
        const uint32_t b_frame = build_packet_stream(80);
        const Events_t e = run(80, 300.0, snrs[i], room_echoes, 3, true);
        char label[48];
        snprintf(label, sizeof label, "aligned, room, window +300, %+.0f dB", snrs[i]);
        print_events(label, e);
        printf("    moved %+d bins, window now %+.0f samples\n", e.last_align, e.final_offset);
        CHECK(fabs(e.final_offset) <= 24.0);
        CHECK_EQ_U(e.trained, 1);
        if (e.trained) {
            CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
            CHECK(abs(e.last_trained.earliest_offset) <= 1);
        }
    }
}

static void print_packet(const char *label, Events_t e) {
    printf("  %-38s trained %u header failed %u packets %u good %u", label, e.trained, e.header_failed, e.packets, e.good);
    if (e.packets) {
        printf(" | %u data symbols, min margin %.2f (symbol %u), %u weak",
            e.last_packet.data_count, e.last_packet.min_margin, e.last_packet.weakest_symbol, e.last_packet.weak_symbols);
    }
    printf("\n");
}

// Whole packets, window aligned from A, decided with the learned shape:
// the payload comes through intact on clean and room channels with noise,
// at 1 and 2 bins per symbol
static void test_packets(void) {
    static const struct { uint32_t spacing; double offset, snr; bool room; } cases[] = {
        { 1, 0.0, 100.0, false }, { 1, 300.0, 0.0, false }, { 1, -450.0, -6.0, false },
        { 1, 120.0, 100.0, true }, { 1, 120.0, 0.0, true }, { 1, -260.0, -3.0, true },
        { 2, 200.0, -6.0, true },
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        rig_setup_spacing(cases[i].spacing);
        chirp_channel_seed(70 + i);
        build_packet_stream(80);
        const Events_t e = run(80, cases[i].offset, cases[i].snr,
            cases[i].room ? room_echoes : NULL, cases[i].room ? 3 : 0, true);
        char label[64];
        snprintf(label, sizeof label, "%s, %u bin, window %+.0f, %s", cases[i].room ? "room" : "clean",
            cases[i].spacing, cases[i].offset, cases[i].snr > 90.0 ? "no noise" : (cases[i].snr == 0.0 ? "0 dB" : (cases[i].snr == -3.0 ? "-3 dB" : "-6 dB")));
        print_packet(label, e);
        CHECK_EQ_U(e.packets, 1);
        CHECK_EQ_U(e.good, 1);
    }
    rig_setup_spacing(1);
}

// The longest payload: 8 packets, windows -400 .. +300 samples, in the
// room at the path strengths measured between the boards, and in the harsh
// room (echoes as strong as the direct path): all intact even at -6 dB.
// With whole-bin alignment only, the harsh room lost 2 of 8 at every noise
// level (inter-symbol interference with the energy between bins).
//
// Baseline: 8/8 everywhere; min margin 1.04 (measured, -6 dB) .. 1.16 (harsh, no noise).
static void test_long_packets(void) {
    static uint8_t payload[PACKET_MAX_PAYLOAD];
    rig_setup_spacing(1);
    const uint32_t frames = LEAD + packet_symbol_count(sizeof payload, 6) + 10;
    for (int harsh = 0; harsh < 2; harsh++) {
        for (int i = 0; i < 3; i++) {
            const double snr = i == 0 ? 100.0 : (i == 1 ? 0.0 : -6.0);
            uint32_t good = 0, weak = 0;
            float32_t min_margin = 1e9f;
            for (uint32_t r = 0; r < 8; r++) {
                chirp_channel_seed(200 + r);
                for (uint32_t k = 0; k < sizeof payload; k++) payload[k] = (uint8_t)(k * 7u + r * 31u);
                build_packet_stream_with(frames, payload, sizeof payload);
                const Events_t e = run(frames, -400.0 + 100.0 * r, snr, harsh ? room_echoes : measured_echoes, 3, true);
                good += e.good;
                weak += e.last_packet.weak_symbols;
                if (e.packets && e.last_packet.min_margin < min_margin) min_margin = e.last_packet.min_margin;
            }
            printf("  %s room, 255 bytes, %-8s %u/8 intact, %u weak decisions, min margin %.2f\n",
                harsh ? "harsh   " : "measured", i == 0 ? "no noise" : (i == 1 ? "0 dB" : "-6 dB"), good, weak, min_margin);
            CHECK_EQ_U(good, 8);
        }
    }
}

// A corrupted packet is reported with a bad CRC, not dropped or accepted:
// one data symbol (in the payload) is changed on the air
static void test_bad_crc(void) {
    rig_setup_spacing(1);
    build_packet_stream(80);
    stream[LEAD + PACKET_PREAMBLE_LEN + 10] ^= 0x15;
    const Events_t e = run(80, 0.0, 100.0, NULL, 0, true);
    print_packet("one symbol changed", e);
    CHECK_EQ_U(e.packets, 1);
    CHECK_EQ_U(e.good, 0);
    if (e.packets) CHECK(!e.last_packet.crc_ok);
}

// A corrupted header is caught by its CRC-8
static void test_bad_header(void) {
    rig_setup_spacing(1);
    build_packet_stream(80);
    stream[LEAD + PACKET_PREAMBLE_LEN] ^= 0x20;
    const Events_t e = run(80, 0.0, 100.0, NULL, 0, true);
    print_packet("first header symbol changed", e);
    CHECK_EQ_U(e.header_failed, 1);
    CHECK_EQ_U(e.packets, 0);
}

// Two packets back to back with a short gap: both received
static void test_two_packets(void) {
    rig_setup_spacing(1);
    const uint32_t len = 43u + 4u;
    build_packet_stream(LEAD + 2 * len + 10);
    for (uint32_t k = 0; k < len; k++) stream[LEAD + len + k] = stream[LEAD + k];
    const Events_t e = run(LEAD + 2 * len + 10, 210.0, 0.0, room_echoes, 3, true);
    print_packet("two packets, 4 silent frames apart", e);
    CHECK_EQ_U(e.packets, 2);
    CHECK_EQ_U(e.good, 2);
}

int main(void) {
    channel_template_init(&tmpl, BINS, template_storage);

    RUN_TEST(test_offsets);
    RUN_TEST(test_noise);
    RUN_TEST(test_room);
    RUN_TEST(test_spacing);
    RUN_TEST(test_noise_only);
    RUN_TEST(test_steady_tone);
    RUN_TEST(test_alignment);
    RUN_TEST(test_packets);
    RUN_TEST(test_long_packets);
    RUN_TEST(test_bad_crc);
    RUN_TEST(test_bad_header);
    RUN_TEST(test_two_packets);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
