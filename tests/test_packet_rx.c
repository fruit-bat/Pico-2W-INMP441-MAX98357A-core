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
#include "logged_ghost_lock.h"

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
static uint32_t sent_ecc = PACKET_ECC_RS2; // Error correction on packets built (as the firmware's default)

// Builds: LEAD silent frames, a packet, then silence up to 'length' frames.
// Returns the frame where B starts.
static uint32_t build_packet_stream_with(uint32_t length, const uint8_t *payload, uint32_t payload_length) {
    sent_payload = payload;
    sent_length = payload_length;
    for (uint32_t k = 0; k < length; k++) stream[k] = CHIRP_CHANNEL_SILENT;
    packet_tx_init(&ptx, (uint32_t)log2(rig_m));
    packet_tx_start(&ptx, payload, payload_length, sent_ecc);
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
    uint32_t lost;
    PacketRxStatus_t last_lost;
    float32_t min_quality;                  // Lowest data frame quality seen
    PacketRxStatus_t last_packet;
    int32_t last_align;           // align_bins at the last LOCKED
    int32_t realigned;            // realign_bins at the last TRAINED (0: none)
    double final_offset;          // Window offset at the end (after any moves)
    PacketRxStatus_t last_trained;
    uint32_t decisions_ok;        // Packets whose kept decisions are all the symbols sent
    PacketRxStatus_t last_header; // At the last HEADER_FAILED
} Events_t;

// A ghost for the lock to fall for: with 'align', the window is moved this
// many bins further than the receiver asked (as when A's earliest path is a
// long echo's wrapped image)
static int32_t wrong_lock_move = 0;

// Runs 'length' frames of the stream through the channel and the receiver.
// With 'align', the window is moved when the receiver locks, and back if
// training says that move was wrong, as main.c does.
static Events_t run(uint32_t length, double offset, double snr_db, const ChirpEcho_t *echoes, uint32_t echo_count, bool align) {
    static float32_t frame[N];
    ChirpChannel_t ch = {
        .offset = offset, .clock_scale = 1.0,
        .noise_rms = snr_db > 90.0 ? 0.0 : (RIG_VOLUME / sqrt(2.0)) / pow(10.0, snr_db / 20.0),
        .echoes = echoes, .echo_count = echo_count,
    };
    packet_rx_init(&prx, BINS, rx_storage, &tmpl);
    Events_t e;
    memset(&e, 0, sizeof e);
    e.min_quality = 1.0f;
    for (uint32_t k = 0; k < length; k++) {
        chirp_channel_rx_frame(&rig_tx_model, &ch, stream, length, k, frame);
        ChirpSymbolRx_t rx;
        rig_decode(frame, &rx);
        const float32_t centre = rx.angle * (float32_t)BINS / (2.0f * (float32_t)M_PI);
        const PacketRxStatus_t *st = packet_rx_frame(&prx, rig_decoder.folded_power_buffer, centre);
        if ((st->state == PACKET_RX_HEADER || st->state == PACKET_RX_PAYLOAD || st->event == PACKET_RX_EVENT_PACKET)
                && st->data_count > 0 && st->data_quality < e.min_quality) e.min_quality = st->data_quality;
        switch (st->event) {
            case PACKET_RX_EVENT_LOCKED:
                e.locked++;
                e.last_align = st->align_bins;
                if (align) {
                    const int32_t bins = st->align_bins + wrong_lock_move;
                    ch.offset -= round((bins + st->align_fraction) * (double)(N / BINS)); // Earlier
                    packet_rx_window_moved(&prx, bins);
                }
                break;
            case PACKET_RX_EVENT_TRAINED:
                e.trained++;
                e.last_trained = *st;
                e.realigned = st->realign_bins;
                if (align && st->realign_bins != 0) {
                    ch.offset -= (double)(st->realign_bins * (int32_t)(N / BINS));
                    packet_rx_window_moved(&prx, st->realign_bins);
                }
                break;
            case PACKET_RX_EVENT_TRAIN_FAILED: e.failed++; break;
            case PACKET_RX_EVENT_TONE: e.tone++; break;
            case PACKET_RX_EVENT_HEADER_FAILED: e.header_failed++; e.last_header = *st; break;
            case PACKET_RX_EVENT_LOST: e.lost++; e.last_lost = *st; break;
            case PACKET_RX_EVENT_PACKET:
                e.packets++;
                e.last_packet = *st;
                if (st->crc_ok && st->length == sent_length
                        && memcmp(packet_rx_payload(&prx), sent_payload, sent_length) == 0) e.good++;
                {
                    const PacketRxDecision_t *d = packet_rx_decisions(&prx);
                    const uint32_t b = (uint32_t)log2(rig_m);
                    bool same = st->data_count == packet_symbol_count(sent_length, sent_ecc, b) - PACKET_PREAMBLE_LEN;
                    for (uint32_t i = 0; same && i < st->data_count; i++) {
                        // Header symbols are kept at spacing 8 (the stream has them at the rig's)
                        const uint32_t sent = stream[LEAD + PACKET_PREAMBLE_LEN + i];
                        const uint32_t want = i < PACKET_HEADER_SYMBOLS ? sent >> (b - PACKET_HEADER_BITS_PER_SYMBOL) : sent;
                        same = d[i].symbol == want && d[i].second != d[i].symbol && d[i].margin >= 1000u;
                    }
                    if (same) e.decisions_ok++;
                }
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
        CHECK_EQ_U(e.realigned, 0);
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
        CHECK_EQ_U(e.realigned, 0);
        if (e.trained) {
            CHECK_EQ_U(e.last_trained.b_start_frame, b_frame);
            CHECK(abs(e.last_trained.earliest_offset) <= 1);
        }
    }
}

static void print_packet(const char *label, Events_t e) {
    printf("  %-38s trained %u header failed %u lost %u packets %u good %u", label, e.trained, e.header_failed, e.lost, e.packets, e.good);
    if (e.packets) {
        printf(" | %u data symbols, min margin %.2f (symbol %u), %u weak, min quality %.2f",
            e.last_packet.data_count, e.last_packet.min_margin, e.last_packet.weakest_symbol, e.last_packet.weak_symbols,
            e.min_quality);
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
        CHECK_EQ_U(e.lost, 0);
        CHECK_EQ_U(e.realigned, 0); // A right lock is left alone, even with echoes as strong as the direct path
        CHECK_EQ_U(e.decisions_ok, 1); // The decisions kept for rx.symbols are the symbols sent
    }
    rig_setup_spacing(1);
}

// The lock falls for a ghost (soak test 2026-11-19: a long echo's wrapped
// image, ~19 bins above the true path): the window is moved that much too
// far. Training sees the strongest path back where the window was, the
// whole move is undone before the header (the window goes back where it
// was: a fair start, as the window was right for the last packet), and the
// packet comes through. Up to
// PACKET_RX_START_SEARCH + 2 data frames are decided before the window
// moves back, through the misplaced window: at 4 bins per symbol (the
// default) the margin absorbs them; it's checked at 1 and 2 as well.
static void test_wrong_lock_move(void) {
    static const struct { uint32_t spacing; int32_t wrong; double offset, snr; bool room; } cases[] = {
        { 4, +19, 0.0, 100.0, false }, { 4, -19, 0.0, 100.0, false }, { 4, +19, 30.0, 0.0, true },
        { 4, +12, -40.0, -3.0, true }, { 4, -10, 0.0, 0.0, true }, { 2, +19, 0.0, 0.0, true },
        { 1, +19, 0.0, 100.0, true },
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        rig_setup_spacing(cases[i].spacing);
        chirp_channel_seed(90 + i);
        build_packet_stream(80);
        wrong_lock_move = cases[i].wrong;
        const Events_t e = run(80, cases[i].offset, cases[i].snr,
            cases[i].room ? measured_echoes : NULL, cases[i].room ? 3 : 0, true);
        wrong_lock_move = 0;
        char label[64];
        snprintf(label, sizeof label, "%s, %u bin, lock moved %+d too far", cases[i].room ? "room" : "clean",
            cases[i].spacing, cases[i].wrong);
        print_packet(label, e);
        printf("    lock %+d, training strongest %+d, moved back %+d, window now %+.0f samples\n",
            e.last_align, e.last_trained.strongest_offset, e.realigned, e.final_offset);
        CHECK_EQ_U(e.trained, 1);
        CHECK_EQ_U((uint32_t)(e.realigned + 64), (uint32_t)(-(e.last_align + cases[i].wrong) + 64));
        // The whole move is undone: the window is back where it was (less the lock's fraction of a bin)
        CHECK(fabs(e.final_offset - cases[i].offset) <= 8.0);
        CHECK_EQ_U(e.good, 1);
    }
    // A wrong move below PACKET_RX_UNDO_MIN_BINS is left for the template to absorb
    rig_setup_spacing(4);
    build_packet_stream(80);
    wrong_lock_move = 5;
    const Events_t e = run(80, 0.0, 100.0, NULL, 0, true);
    wrong_lock_move = 0;
    CHECK_EQ_U(e.realigned, 0);
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
    const uint32_t frames = LEAD + packet_symbol_count(sizeof payload, sent_ecc, 6) + 10;
    for (int harsh = 0; harsh < 2; harsh++) {
        for (int i = 0; i < 3; i++) {
            const double snr = i == 0 ? 100.0 : (i == 1 ? 0.0 : -6.0);
            uint32_t good = 0, weak = 0, lost = 0;
            float32_t min_margin = 1e9f;
            for (uint32_t r = 0; r < 8; r++) {
                chirp_channel_seed(200 + r);
                for (uint32_t k = 0; k < sizeof payload; k++) payload[k] = (uint8_t)(k * 7u + r * 31u);
                build_packet_stream_with(frames, payload, sizeof payload);
                const Events_t e = run(frames, -400.0 + 100.0 * r, snr, harsh ? room_echoes : measured_echoes, 3, true);
                good += e.good;
                lost += e.lost;
                weak += e.last_packet.weak_symbols;
                if (e.packets && e.last_packet.min_margin < min_margin) min_margin = e.last_packet.min_margin;
            }
            printf("  %s room, %u bytes, %-8s %u/8 intact, %u weak decisions, min margin %.2f\n",
                harsh ? "harsh   " : "measured", (unsigned)sizeof payload, i == 0 ? "no noise" : (i == 1 ? "0 dB" : "-6 dB"), good, weak, min_margin);
            CHECK_EQ_U(good, 8);
            CHECK_EQ_U(lost, 0);
        }
    }
}

// Without error correction a corrupted packet is reported with a bad CRC,
// not dropped or accepted: one data symbol (in the payload) is changed on
// the air
static void test_bad_crc(void) {
    rig_setup_spacing(1);
    sent_ecc = PACKET_ECC_NONE;
    build_packet_stream(80);
    stream[LEAD + PACKET_PREAMBLE_LEN + 10] ^= 0x15;
    const Events_t e = run(80, 0.0, 100.0, NULL, 0, true);
    print_packet("one symbol changed, no error correction", e);
    CHECK_EQ_U(e.packets, 1);
    CHECK_EQ_U(e.good, 0);
    if (e.packets) {
        CHECK(!e.last_packet.crc_ok);
        CHECK_EQ_U(e.last_packet.ecc, PACKET_ECC_NONE);
        CHECK_EQ_U(e.last_packet.ecc_fixed, 0);
    }
    sent_ecc = PACKET_ECC_RS2;
}

// With rs2, wrong bytes after the header are corrected, up to 2 of them
// (in the check bytes too); 3 are not, and the packet ends with a bad CRC.
// At 6 bits/symbol, payload symbols 0, 4, 8, 12 and 28 each lie within one
// byte (0, 3, 6, 9 and 21, the last check byte)
static void test_payload_correction(void) {
    static const struct { uint32_t count, symbols[3]; int32_t fixed; } cases[] = {
        { 1, { 4 }, 1 },
        { 1, { 28 }, 1 },
        { 2, { 4, 12 }, 2 },
        { 2, { 0, 28 }, 2 },
        { 3, { 0, 4, 12 }, -1 },
        { 3, { 0, 8, 28 }, -1 },
    };
    rig_setup_spacing(1);
    for (unsigned k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        build_packet_stream(90);
        for (uint32_t c = 0; c < cases[k].count; c++) {
            stream[LEAD + PACKET_PREAMBLE_LEN + PACKET_HEADER_SYMBOLS + cases[k].symbols[c]] ^= 0x15;
        }
        const Events_t e = run(90, 0.0, 100.0, NULL, 0, true);
        char label[48];
        snprintf(label, sizeof label, "%u wrong bytes, rs2", cases[k].count);
        print_packet(label, e);
        CHECK_EQ_U(e.packets, 1);
        if (!e.packets) continue;
        CHECK_EQ_U(e.last_packet.ecc, PACKET_ECC_RS2);
        if (cases[k].fixed >= 0) {
            CHECK_EQ_U(e.good, 1);
            CHECK_EQ_U(e.last_packet.ecc_fixed, cases[k].fixed);
        } else {
            // Not corrected: the code says so, or (rarely) "corrects" to
            // another codeword, which the CRC catches
            CHECK_EQ_U(e.good, 0);
            CHECK(!e.last_packet.crc_ok);
            printf("    ecc_fixed %d\n", (int)e.last_packet.ecc_fixed);
        }
    }
}

// Changes header symbol i (at spacing 1, as the rig) so its 3-bit value has
// these bits flipped
static void corrupt_header_symbol(uint32_t i, uint32_t flip) {
    uint32_t *s = &stream[LEAD + PACKET_PREAMBLE_LEN + i];
    const uint32_t s8 = *s >> 3;
    *s = packet_gray_symbol(packet_gray_value(s8) ^ flip) << 3;
}

// Header errors: up to 3 wrong bits are corrected (the packet arrives, and
// says how many); 4 are caught, and the packet is given up
static void test_bad_header(void) {
    static const struct { uint32_t symbol, flip; } cases[][2] = {
        { { 0, 4 }, { 0, 0 } },  // One bit
        { { 0, 7 }, { 0, 0 } },  // A wild symbol: all 3 bits
        { { 2, 1 }, { 7, 3 } },  // 3 bits in 2 symbols
        { { 0, 7 }, { 5, 2 } },  // 4 bits: too many
    };
    static const uint32_t fixed[] = { 1, 3, 3, 0 };
    rig_setup_spacing(1);
    for (unsigned k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        build_packet_stream(80);
        for (unsigned c = 0; c < 2; c++) if (cases[k][c].flip) corrupt_header_symbol(cases[k][c].symbol, cases[k][c].flip);
        const Events_t e = run(80, 0.0, 100.0, NULL, 0, true);
        char label[48];
        snprintf(label, sizeof label, "header: %u bits wrong", k < 3 ? fixed[k] : 4u);
        print_packet(label, e);
        if (k < 3) {
            CHECK_EQ_U(e.good, 1);
            CHECK_EQ_U(e.last_packet.header_fixed, fixed[k]);
            CHECK_EQ_U(e.last_packet.spacing, 1);
        } else {
            CHECK_EQ_U(e.header_failed, 1);
            CHECK_EQ_U(e.packets, 0);
            CHECK_EQ_U(e.last_header.header_result, PACKET_RX_HEADER_CHECK);
        }
    }
}

// A header from a newer format (here: error correction 2, reserved) is
// decoded, and refused
static void test_unknown_header(void) {
    rig_setup_spacing(4);
    build_packet_stream(80);
    const PacketHeader_t h = { PACKET_VERSION, 4, 2, sizeof test_payload };
    const uint32_t code = packet_golay_encode(packet_header_pack(&h));
    for (uint32_t i = 0; i < PACKET_HEADER_SYMBOLS; i++) {
        const uint32_t v = (code >> (21u - 3u * i)) & 7u;
        stream[LEAD + PACKET_PREAMBLE_LEN + i] = packet_gray_symbol(v) << 1; // Spacing 8 at 4 bits/symbol
    }
    const Events_t e = run(80, 0.0, 100.0, NULL, 0, true);
    print_packet("header with error correction 2", e);
    CHECK_EQ_U(e.header_failed, 1);
    CHECK_EQ_U(e.last_header.header_result, PACKET_RX_HEADER_FORMAT);
    rig_setup_spacing(1);
}

// The receiver follows the header's spacing and error correction: packets
// at 1, 2 and 4 bins per symbol, with and without rs2, one after another,
// all with the same receiver set-up
static void test_mixed_spacings(void) {
    static const uint32_t spacings[] = { 4, 1, 2, 4 };
    static const uint32_t eccs[] = { PACKET_ECC_RS2, PACKET_ECC_NONE, PACKET_ECC_RS2, PACKET_ECC_NONE };
    for (unsigned k = 0; k < 4; k++) {
        rig_setup_spacing(spacings[k]);
        sent_ecc = eccs[k];
        chirp_channel_seed(90 + k);
        build_packet_stream(100);
        const Events_t e = run(100, 150.0, 0.0, measured_echoes, 3, true);
        char label[48];
        snprintf(label, sizeof label, "spacing %u, %s, room, 0 dB", spacings[k], eccs[k] ? "rs2" : "no ecc");
        print_packet(label, e);
        CHECK_EQ_U(e.good, 1);
        CHECK_EQ_U(e.last_packet.spacing, spacings[k]);
        CHECK_EQ_U(e.last_packet.ecc, eccs[k]);
    }
    sent_ecc = PACKET_ECC_RS2;
    rig_setup_spacing(1);
}

// Two packets back to back with a short gap: both received
static void test_two_packets(void) {
    rig_setup_spacing(1);
    const uint32_t len = packet_symbol_count(sizeof test_payload, sent_ecc, 6) + 4u;
    build_packet_stream(LEAD + 2 * len + 10);
    for (uint32_t k = 0; k < len; k++) stream[LEAD + len + k] = stream[LEAD + k];
    const Events_t e = run(LEAD + 2 * len + 10, 210.0, 0.0, room_echoes, 3, true);
    print_packet("two packets, 4 silent frames apart", e);
    CHECK_EQ_U(e.packets, 2);
    CHECK_EQ_U(e.good, 2);
}

// The transmitter stops partway through the payload (noise goes on): the
// packet is abandoned within a few frames, not decoded to the end, and a
// packet straight after it is received
static void test_lost_signal(void) {
    rig_setup_spacing(1);
    // 16-byte packet; stop after the header and 4 payload symbols
    const uint32_t len = packet_symbol_count(sizeof test_payload, sent_ecc, 6), cut = PACKET_PREAMBLE_LEN + PACKET_HEADER_SYMBOLS + 4u;
    const uint32_t frames = LEAD + len + 6u + len + 10u;
    build_packet_stream(frames);
    for (uint32_t k = 0; k < len; k++) stream[LEAD + len + 6u + k] = stream[LEAD + k]; // A second, whole packet
    for (uint32_t k = cut; k < len; k++) stream[LEAD + k] = CHIRP_CHANNEL_SILENT;
    for (int i = 0; i < 2; i++) {
        chirp_channel_seed(90 + i);
        const Events_t e = run(frames, 150.0, i ? -6.0 : 0.0, measured_echoes, 3, true);
        print_packet(i ? "cut off, then a packet, -6 dB" : "cut off, then a packet, 0 dB", e);
        CHECK_EQ_U(e.lost, 1);
        if (e.lost) {
            printf("    lost after %u data symbols\n", e.last_lost.data_count);
            CHECK(e.last_lost.data_count <= cut - PACKET_PREAMBLE_LEN + PACKET_RX_LOST_FRAMES + 1u);
        }
        CHECK_EQ_U(e.packets, 1);
        CHECK_EQ_U(e.good, 1);
    }
}

// Board 2, 2026-11-18 (frames 27128..27138 of a log): noise, then the first frames of A
// with the window 30 bins out. Folded power, and the decoder's centre in bins
static const float32_t logged_folded[11][64] = {
    { 6.512e-06f, 1.271e-06f, 2.230e-06f, 3.176e-06f, 1.709e-06f, 5.764e-07f, 7.295e-06f, 7.451e-06f, 3.001e-07f, 2.632e-05f, 8.186e-06f, 3.365e-06f, 1.434e-06f, 8.095e-06f, 1.929e-06f, 1.053e-08f, 1.535e-06f, 7.822e-07f, 5.638e-06f, 1.468e-06f, 5.008e-06f, 2.714e-06f, 8.828e-06f, 6.730e-06f, 4.148e-07f, 3.729e-06f, 6.044e-07f, 9.458e-06f, 1.987e-05f, 1.705e-06f, 2.604e-06f, 1.337e-06f, 8.908e-06f, 2.127e-07f, 3.467e-06f, 9.273e-06f, 6.937e-06f, 2.697e-06f, 1.679e-06f, 4.971e-06f, 1.320e-05f, 1.154e-06f, 3.695e-07f, 1.430e-05f, 7.526e-06f, 7.587e-06f, 3.382e-06f, 1.830e-05f, 4.183e-06f, 5.789e-06f, 3.753e-06f, 1.030e-06f, 7.048e-06f, 8.831e-06f, 6.746e-06f, 4.005e-06f, 1.134e-05f, 5.058e-07f, 1.320e-06f, 1.241e-06f, 1.158e-06f, 2.347e-06f, 2.152e-06f, 1.220e-06f },
    { 3.611e-08f, 1.274e-05f, 1.666e-07f, 2.286e-05f, 1.113e-05f, 2.031e-07f, 3.674e-06f, 1.172e-05f, 2.370e-07f, 1.274e-06f, 3.451e-06f, 4.245e-06f, 4.986e-06f, 3.750e-05f, 1.309e-06f, 1.648e-06f, 1.474e-07f, 2.809e-05f, 1.236e-05f, 1.089e-05f, 3.593e-06f, 2.299e-05f, 4.536e-06f, 1.115e-05f, 1.879e-06f, 4.240e-06f, 1.198e-05f, 6.841e-06f, 9.567e-06f, 4.279e-06f, 6.450e-06f, 7.080e-07f, 8.024e-06f, 4.693e-06f, 1.169e-06f, 3.459e-08f, 0.997e-05f, 3.656e-06f, 5.822e-06f, 1.045e-06f, 3.019e-06f, 1.014e-05f, 6.872e-06f, 1.969e-06f, 2.148e-06f, 3.326e-06f, 8.034e-07f, 2.962e-06f, 6.793e-07f, 2.937e-06f, 3.875e-06f, 4.846e-06f, 4.379e-06f, 1.308e-05f, 2.621e-06f, 4.223e-06f, 1.678e-06f, 2.053e-06f, 1.501e-06f, 7.677e-06f, 2.417e-06f, 7.302e-06f, 2.827e-06f, 1.199e-05f },
    { 1.507e-05f, 8.646e-06f, 7.140e-06f, 5.966e-06f, 1.302e-05f, 1.524e-06f, 1.064e-05f, 1.255e-05f, 9.430e-07f, 2.440e-06f, 2.213e-06f, 4.068e-06f, 7.996e-06f, 6.818e-06f, 1.819e-05f, 3.118e-06f, 2.018e-05f, 7.287e-06f, 8.680e-06f, 3.590e-06f, 1.563e-06f, 2.006e-06f, 2.410e-06f, 2.205e-05f, 1.414e-05f, 1.121e-06f, 2.615e-06f, 1.059e-05f, 1.479e-07f, 1.844e-05f, 0.981e-06f, 2.978e-05f, 1.312e-06f, 6.081e-06f, 8.147e-06f, 8.772e-06f, 2.478e-07f, 2.074e-05f, 2.010e-06f, 1.111e-06f, 2.490e-06f, 1.209e-06f, 4.007e-06f, 3.078e-05f, 4.641e-07f, 1.695e-05f, 1.536e-07f, 6.245e-05f, 1.118e-05f, 5.110e-06f, 9.264e-06f, 8.559e-07f, 4.727e-06f, 3.409e-06f, 1.382e-06f, 1.154e-05f, 4.829e-06f, 1.164e-05f, 1.251e-05f, 4.131e-06f, 1.142e-05f, 1.848e-05f, 5.824e-06f, 1.942e-05f },
    { 4.533e-06f, 5.091e-06f, 2.305e-06f, 1.720e-05f, 7.669e-06f, 8.726e-06f, 2.661e-06f, 4.073e-06f, 8.539e-07f, 7.738e-06f, 7.092e-06f, 3.904e-06f, 6.528e-06f, 2.740e-05f, 4.813e-08f, 1.535e-05f, 5.291e-06f, 2.424e-06f, 3.374e-05f, 2.736e-06f, 3.152e-06f, 7.366e-06f, 9.392e-06f, 5.580e-06f, 5.438e-06f, 1.928e-05f, 6.337e-06f, 1.267e-06f, 2.485e-06f, 4.314e-06f, 7.913e-06f, 8.590e-06f, 3.480e-06f, 1.509e-05f, 8.029e-06f, 1.241e-05f, 3.916e-06f, 2.745e-06f, 4.141e-06f, 1.626e-05f, 1.698e-05f, 4.281e-06f, 1.514e-06f, 1.172e-05f, 1.927e-06f, 1.022e-05f, 1.382e-05f, 4.182e-06f, 3.713e-05f, 2.668e-06f, 4.239e-06f, 3.175e-06f, 8.856e-06f, 2.406e-05f, 1.789e-05f, 3.292e-07f, 4.296e-07f, 2.756e-06f, 1.210e-07f, 3.281e-06f, 1.014e-05f, 2.582e-06f, 3.893e-06f, 5.980e-07f },
    { 1.472e-06f, 2.988e-06f, 2.330e-06f, 3.223e-06f, 8.005e-06f, 2.968e-06f, 2.420e-06f, 3.303e-06f, 4.068e-06f, 1.690e-06f, 1.309e-05f, 5.893e-06f, 2.075e-05f, 5.332e-06f, 5.679e-06f, 1.036e-05f, 1.786e-05f, 8.437e-06f, 2.223e-05f, 9.098e-06f, 2.935e-06f, 1.337e-05f, 5.454e-08f, 2.177e-06f, 1.357e-05f, 8.870e-06f, 2.221e-05f, 6.885e-06f, 1.282e-05f, 9.734e-06f, 3.913e-06f, 2.530e-06f, 1.173e-05f, 3.188e-06f, 6.287e-06f, 1.546e-05f, 3.841e-06f, 3.025e-06f, 6.878e-06f, 0.995e-05f, 8.805e-07f, 1.324e-05f, 7.920e-06f, 1.219e-06f, 8.219e-06f, 1.574e-05f, 1.307e-05f, 5.924e-06f, 1.859e-05f, 3.337e-06f, 1.926e-05f, 1.032e-05f, 1.060e-05f, 7.076e-07f, 1.426e-06f, 1.086e-05f, 9.255e-07f, 2.011e-06f, 1.050e-05f, 6.464e-08f, 5.218e-07f, 2.421e-06f, 2.344e-05f, 2.567e-05f },
    { 3.329e-06f, 7.785e-06f, 1.163e-05f, 1.841e-05f, 1.536e-05f, 6.847e-06f, 2.162e-06f, 7.487e-06f, 5.103e-07f, 1.110e-05f, 1.076e-05f, 4.373e-06f, 1.449e-06f, 3.697e-06f, 1.216e-05f, 2.286e-05f, 1.497e-06f, 3.580e-06f, 2.259e-06f, 6.899e-06f, 9.117e-07f, 6.523e-06f, 0.959e-06f, 8.748e-06f, 0.999e-05f, 5.690e-06f, 6.484e-07f, 1.681e-05f, 1.641e-06f, 1.191e-05f, 2.738e-06f, 2.602e-07f, 1.100e-05f, 2.634e-06f, 1.084e-05f, 1.543e-05f, 1.100e-05f, 9.003e-07f, 3.519e-07f, 3.772e-06f, 1.163e-05f, 2.857e-05f, 1.344e-05f, 1.287e-05f, 3.157e-06f, 7.244e-06f, 2.648e-05f, 3.283e-06f, 1.933e-05f, 1.475e-06f, 6.436e-06f, 2.511e-06f, 6.512e-06f, 5.047e-06f, 2.438e-06f, 3.604e-06f, 9.012e-07f, 2.711e-06f, 9.533e-06f, 1.145e-06f, 2.457e-06f, 5.591e-06f, 6.501e-06f, 4.829e-06f },
    { 1.812e-05f, 4.027e-06f, 5.888e-06f, 2.088e-05f, 4.449e-06f, 3.831e-07f, 7.457e-06f, 2.296e-05f, 8.782e-06f, 6.402e-07f, 2.742e-07f, 2.664e-06f, 1.022e-05f, 6.363e-06f, 6.385e-06f, 1.530e-05f, 2.851e-06f, 7.858e-06f, 1.161e-07f, 5.748e-06f, 2.821e-06f, 3.074e-06f, 6.893e-06f, 1.246e-05f, 3.285e-06f, 1.282e-05f, 1.539e-05f, 1.087e-05f, 1.674e-05f, 3.715e-06f, 3.934e-07f, 5.194e-06f, 5.797e-06f, 5.158e-06f, 1.626e-06f, 2.263e-05f, 5.365e-05f, 5.294e-06f, 1.477e-05f, 2.410e-05f, 2.366e-06f, 1.843e-06f, 1.895e-06f, 3.658e-06f, 1.326e-05f, 1.135e-05f, 1.079e-05f, 1.365e-06f, 4.884e-06f, 5.418e-06f, 2.979e-06f, 5.981e-06f, 6.277e-06f, 1.714e-06f, 2.315e-05f, 5.175e-06f, 1.950e-05f, 1.582e-05f, 3.316e-06f, 2.016e-06f, 9.368e-07f, 2.734e-06f, 2.703e-05f, 1.731e-05f },
    { 4.658e-05f, 2.015e-07f, 1.371e-05f, 5.676e-05f, 9.443e-05f, 1.442e-04f, 1.243e-04f, 3.540e-05f, 9.188e-05f, 1.239e-04f, 1.382e-04f, 4.704e-04f, 4.854e-04f, 7.316e-04f, 1.044e-03f, 1.329e-03f, 2.045e-03f, 2.351e-03f, 1.499e-03f, 5.783e-04f, 3.330e-04f, 1.325e-03f, 1.436e-03f, 6.408e-04f, 3.957e-05f, 6.451e-05f, 1.692e-04f, 4.514e-05f, 4.885e-05f, 3.771e-04f, 1.287e-03f, 2.031e-03f, 1.034e-03f, 1.755e-03f, 5.662e-03f, 2.200e-03f, 1.551e-04f, 4.427e-04f, 3.075e-04f, 3.499e-04f, 6.390e-05f, 4.919e-05f, 2.650e-05f, 7.590e-05f, 3.665e-05f, 1.109e-04f, 3.320e-06f, 1.638e-04f, 3.199e-04f, 3.551e-04f, 1.166e-04f, 3.732e-05f, 4.710e-04f, 9.151e-04f, 2.992e-03f, 3.257e-03f, 1.583e-04f, 5.780e-05f, 2.982e-06f, 6.692e-05f, 3.227e-05f, 8.695e-05f, 5.747e-05f, 3.694e-05f },
    { 3.600e-04f, 3.366e-04f, 5.785e-04f, 1.931e-03f, 1.179e-03f, 8.291e-04f, 6.373e-04f, 2.429e-03f, 2.295e-04f, 1.092e-03f, 2.100e-04f, 2.734e-04f, 2.093e-04f, 1.236e-03f, 3.218e-03f, 1.672e-03f, 6.149e-04f, 1.019e-02f, 2.591e-03f, 6.990e-04f, 1.104e-03f, 9.432e-04f, 1.378e-03f, 5.912e-05f, 4.713e-04f, 6.542e-05f, 2.594e-04f, 1.159e-04f, 6.885e-04f, 1.072e-03f, 3.294e-04f, 5.520e-03f, 6.823e-04f, 1.628e-02f, 2.140e-02f, 1.829e-03f, 2.632e-03f, 1.987e-03f, 1.332e-03f, 6.610e-04f, 3.220e-04f, 1.397e-04f, 1.460e-05f, 1.557e-03f, 1.583e-03f, 9.487e-04f, 8.011e-05f, 1.521e-03f, 1.595e-03f, 4.904e-04f, 1.266e-03f, 1.449e-04f, 1.487e-03f, 1.389e-04f, 2.645e-03f, 3.348e-03f, 2.288e-03f, 4.591e-04f, 2.843e-05f, 1.059e-03f, 7.253e-04f, 1.015e-03f, 1.258e-03f, 4.726e-04f },
    { 5.360e-04f, 2.158e-04f, 3.350e-04f, 8.452e-04f, 8.674e-04f, 1.758e-03f, 1.017e-03f, 3.430e-03f, 3.758e-06f, 9.183e-04f, 2.814e-05f, 3.319e-04f, 5.260e-04f, 0.974e-03f, 3.317e-03f, 1.733e-03f, 0.972e-03f, 1.480e-02f, 1.896e-03f, 1.492e-03f, 6.229e-04f, 1.010e-03f, 2.060e-03f, 3.426e-05f, 4.057e-04f, 1.143e-04f, 2.992e-04f, 6.574e-04f, 1.160e-04f, 4.646e-04f, 1.051e-04f, 5.828e-03f, 5.299e-04f, 1.550e-02f, 1.763e-02f, 1.717e-03f, 1.947e-03f, 1.638e-03f, 6.322e-04f, 2.558e-04f, 2.900e-04f, 9.747e-05f, 3.850e-04f, 2.345e-03f, 1.122e-03f, 1.023e-03f, 5.979e-05f, 1.660e-03f, 1.050e-03f, 1.881e-03f, 1.979e-03f, 3.982e-04f, 2.274e-03f, 1.148e-04f, 3.444e-03f, 4.173e-03f, 3.031e-03f, 3.377e-04f, 2.806e-04f, 1.096e-03f, 1.878e-03f, 9.491e-04f, 1.395e-03f, 8.796e-04f },
    { 7.295e-04f, 3.670e-04f, 5.694e-04f, 6.260e-04f, 1.086e-03f, 2.014e-03f, 4.865e-04f, 3.158e-03f, 2.189e-05f, 9.596e-04f, 1.413e-05f, 5.055e-04f, 6.652e-04f, 3.899e-04f, 2.985e-03f, 3.061e-03f, 1.230e-03f, 1.242e-02f, 1.649e-03f, 8.604e-04f, 4.613e-04f, 8.249e-04f, 2.075e-03f, 9.403e-05f, 4.368e-04f, 2.538e-05f, 3.392e-04f, 4.843e-04f, 1.292e-04f, 4.037e-04f, 2.350e-04f, 5.843e-03f, 3.834e-04f, 1.607e-02f, 2.075e-02f, 3.358e-03f, 1.897e-03f, 8.251e-04f, 4.060e-04f, 4.728e-04f, 2.395e-04f, 1.414e-04f, 4.231e-04f, 2.378e-03f, 9.264e-04f, 0.972e-03f, 2.251e-04f, 2.003e-03f, 1.207e-03f, 1.163e-03f, 1.180e-03f, 2.434e-04f, 2.106e-03f, 1.658e-04f, 4.463e-03f, 3.914e-03f, 2.781e-03f, 3.491e-04f, 9.244e-04f, 1.079e-03f, 0.989e-03f, 9.230e-04f, 1.873e-03f, 4.234e-04f },
};
static const float32_t logged_centre[11] = { 8.5176f, 12.9065f, 47.1189f, 47.4285f, 62.8775f, 41.3242f, 36.5603f, 33.3356f, 33.3123f, 33.2387f, 33.3216f };

// Measured on the boards (2026-11-18), frames above: with the window about
// half a frame out, the first frame to overlap A (27135) held only part of
// it, with stray energy 20 bins above A's peak. The board locked on 27138
// (the third agreement), averaged 27135..27138, took the stray energy for
// the earliest path, and moved the window 10 bins instead of 30: B's
// coherence was 0.42 and the packet failed. Now the lock waits for a fourth
// agreement and measures from the four frames that agreed, all whole A
// frames. The board moved its window after 27138, so the frame that would
// have followed is stood in for by a copy of 27137 (another whole A frame).
static void test_partial_first_frame(void) {
    channel_template_init(&tmpl, BINS, template_storage);
    packet_rx_init(&prx, BINS, rx_storage, &tmpl);
    const uint32_t logged = sizeof logged_centre / sizeof logged_centre[0];
    const uint32_t count = logged + 1u;
    const PacketRxStatus_t *st = NULL;
    uint32_t locked_at = 0;
    for (uint32_t f = 0; f < count; f++) {
        const uint32_t g = f < logged ? f : logged - 2u;
        st = packet_rx_frame(&prx, logged_folded[g], logged_centre[g]);
        if (st->event == PACKET_RX_EVENT_LOCKED) locked_at = f;
    }
    CHECK_EQ_U(locked_at, count - 1u); // A frame after the board did
    printf("  lock: window %+ld bins %+.2f out (A's peak is at bin 34, i.e. -30)\n",
        (long)st->align_bins, (double)st->align_fraction);
    CHECK_EQ_U((uint32_t)(st->align_bins + 64), (uint32_t)(-30 + 64));
    // The fraction from the same four frames' centres
    float c = 0.0f, sn = 0.0f;
    for (uint32_t f = count - 4u; f < count; f++) {
        const uint32_t g = f < logged ? f : logged - 2u;
        c += cosf(6.2831853f * logged_centre[g]);
        sn += sinf(6.2831853f * logged_centre[g]);
    }
    CHECK_NEAR(st->align_fraction, atan2f(sn, c) / 6.2831853f, 0.001);
}

// The soak test's failure, replayed (logged_ghost_lock.h): the lock moves
// the window 16 bins the wrong way, as the board did, and training must
// then ask for the move to be undone. (What follows can't be replayed: the
// board's later frames were taken through the wrong window.)
static void test_ghost_lock(void) {
    channel_template_init(&tmpl, BINS, template_storage);
    packet_rx_init(&prx, BINS, rx_storage, &tmpl);
    int32_t locked_at = -1, trained_at = -1;
    PacketRxStatus_t lock, trained;
    memset(&lock, 0, sizeof lock);
    memset(&trained, 0, sizeof trained);
    for (uint32_t f = 0; f < LOGGED_GHOST_FRAMES; f++) {
        const PacketRxStatus_t *st = packet_rx_frame(&prx, logged_ghost_folded[f], logged_ghost_centre[f]);
        if (st->event == PACKET_RX_EVENT_LOCKED) {
            locked_at = (int32_t)f;
            lock = *st;
            packet_rx_window_moved(&prx, st->align_bins); // As the board did
        }
        if (st->event == PACKET_RX_EVENT_TRAINED) {
            trained_at = (int32_t)f;
            trained = *st;
        }
    }
    printf("  lock on %d: align %+ld; trained on %d: coherence %.2f, strongest %+ld, earliest %+ld, realign %+ld\n",
        (int)locked_at, (long)lock.align_bins, (int)trained_at, (double)trained.coherence,
        (long)trained.strongest_offset, (long)trained.earliest_offset, (long)trained.realign_bins);
    CHECK_EQ_U((uint32_t)locked_at, LOGGED_GHOST_LOCK_FRAME);       // Where the board locked...
    CHECK_EQ_U((uint32_t)(lock.align_bins + 64), (uint32_t)(16 + 64)); // ...and the ghost it took
    CHECK_EQ_U((uint32_t)trained_at, LOGGED_GHOST_TRAINED_FRAME);
    CHECK_EQ_U((uint32_t)(trained.strongest_offset + 64), (uint32_t)(-18 + 64)); // As the board reported
    CHECK_EQ_U((uint32_t)(trained.realign_bins + 64), (uint32_t)(-16 + 64));    // Undo the whole move
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
    RUN_TEST(test_partial_first_frame);
    RUN_TEST(test_ghost_lock);
    RUN_TEST(test_packets);
    RUN_TEST(test_wrong_lock_move);
    RUN_TEST(test_long_packets);
    RUN_TEST(test_bad_crc);
    RUN_TEST(test_payload_correction);
    RUN_TEST(test_bad_header);
    RUN_TEST(test_unknown_header);
    RUN_TEST(test_mixed_spacings);
    RUN_TEST(test_two_packets);
    RUN_TEST(test_lost_signal);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
