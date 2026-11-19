/**
 * @file fake_board.c
 *
 * A pretend board for the protocol tests; see fake_board.h.
 */

#include "fake_board.h"

#include <string.h>

#define SAMPLE_RATE 44100u
#define FFT_SIZE 1024u
#define LOCK_FRAME 3u     // Frames into a packet when the receiver locks (A is 8 frames)
#define TRAIN_FRAME 18u   // ... and finds B (B ends at 16; it decides 2 frames later)

static uint32_t bits_per_symbol(const FakeBoard_t *fb) {
    return (uint32_t)__builtin_ctz(PROTOCOL_CHIRP_BINS / (uint32_t)fb->settings.value[SETTING_LINK_SPACING]);
}

uint32_t fake_board_packet_frames(const FakeBoard_t *fb, uint32_t length) {
    const uint32_t bits = 16u + 8u * length + 16u; // Header, payload, CRC-16
    const uint32_t b = bits_per_symbol(fb);
    return 16u + (bits + b - 1u) / b;
}

// Made-up spectra: low noise, with the symbol's bin standing out while a
// packet is being sent (the board hears itself)
static float folded[PROTOCOL_CHIRP_BINS];
static float band[2u * PROTOCOL_CHIRP_BINS];
static float shape[PROTOCOL_CHIRP_BINS];
static float spectrum[FFT_SIZE];

static float noise(uint32_t *seed) {
    *seed = *seed * 1664525u + 1013904223u;
    return 1e-4f * (0.5f + (float)(*seed >> 8) / 16777216.0f);
}

static void report_frame(FakeBoard_t *fb, bool learned) {
    const uint32_t spacing = (uint32_t)fb->settings.value[SETTING_LINK_SPACING];
    const uint32_t symbols = PROTOCOL_CHIRP_BINS / spacing;
    // Chirp mode sends test.symbol all the time; packets are heard while being sent
    const bool chirp = fb->settings.value[SETTING_TEST_MODE] == 3;
    const bool on_air = chirp || (fb->sending && fb->settings.value[SETTING_RX_DURING_TX] != 0);
    const uint32_t symbol = chirp ? (uint32_t)fb->settings.value[SETTING_TEST_SYMBOL] % symbols
                          : on_air ? fb->sent_frames * 7u % symbols : 0u;
    uint32_t seed = fb->frame;
    for (uint32_t i = 0; i < PROTOCOL_CHIRP_BINS; i++) folded[i] = noise(&seed);
    for (uint32_t i = 0; i < 2u * PROTOCOL_CHIRP_BINS; i++) band[i] = noise(&seed);
    for (uint32_t i = 0; i < FFT_SIZE; i++) spectrum[i] = noise(&seed);
    if (on_air) {
        folded[symbol * spacing] = 1.0f;
        band[PROTOCOL_CHIRP_BINS + symbol * spacing] = 1.0f;
        spectrum[symbol * spacing] = 1.0f;
    }
    if (learned) {
        for (uint32_t i = 0; i < PROTOCOL_CHIRP_BINS; i++) shape[i] = noise(&seed);
        shape[0] = 1.0f;
        shape[PROTOCOL_CHIRP_BINS - 1u] = 0.3f; // A path a bin early
    }
    const ProtocolFrame_t f = {
        .frame = fb->frame,
        .tx_symbol = symbol,
        .delay = fb->status.rx_delay,
        .rx_symbol = symbol,
        .angle = 2.0f * 3.14159265f * (float)(symbol * spacing) / (float)PROTOCOL_CHIRP_BINS,
        .error_angle = on_air ? 0.01f : 0.2f,
        .magnitude = on_air ? 0.95f : 0.3f,
        .strength = on_air ? 2.5e-3f : 1.0e-6f,
        .peak = on_air ? 0.31f : 0.002f,
        .timing_error = on_air ? 0.2f : 3.0f,
        .timing_rate = 0.0f,
        .state = fb->status.rx_state,
        .corr = on_air ? 0.9f : 0.3f,
        .data_symbol = symbol,
        .data_margin = on_air ? 2.1f : 0.0f,
        .data_quality = on_air ? 0.85f : 0.3f,
        .rx_missed = fb->status.rx_missed,
        .tx_missed = fb->status.tx_missed,
        .records_dropped = fb->status.records_dropped,
        .folded = folded,
        .band = band,
        .shape = shape,
        .shape_changed = learned,
        .spectrum = spectrum,
        .spectrum_len = FFT_SIZE,
    };
    protocol_frame(fb->proto, &f);
}

static void event(FakeBoard_t *fb, ProtocolEvent_t e) {
    e.frame = fb->frame;
    protocol_event(fb->proto, &e);
}

static void one_frame(FakeBoard_t *fb) {
    bool learned = false;
    fb->frame++;
    fb->status.frames = fb->frame;
    const bool hear = fb->settings.value[SETTING_RX_DURING_TX] != 0;

    // As main.c: a spacing change cuts the packet being sent short, and gives
    // up the one being received; so does "reset rx" (both on the next pass)
    const bool spacing = fb->abandon && fb->sending;
    if (spacing) {
        fb->sending = false;
        ProtocolEvent_t done = { .type = PROTO_EV_TX_DONE };
        done.u.tx.id = fb->current.id;
        done.u.tx.abandoned = true;
        event(fb, done);
    }
    if ((spacing || fb->reset_rx) && fb->listening) {
        fb->listening = false;
        fb->status.rx_state = PROTO_RX_SEARCH;
        fb->status.lost++;
        ProtocolEvent_t lost = { .type = PROTO_EV_RX_LOST };
        lost.u.lost.symbols = fb->sent_frames > 16u ? fb->sent_frames - 16u : 0u;
        lost.u.lost.reason = spacing ? PROTO_LOST_SPACING : PROTO_LOST_RESET;
        event(fb, lost);
    }
    if (fb->reset_rx) fb->status.rx_state = PROTO_RX_SEARCH;
    fb->abandon = false;
    fb->reset_rx = false;
    if (fb->sending) {
        fb->sent_frames++;
        if (hear && fb->sent_frames == LOCK_FRAME) {
            fb->listening = true;
            fb->status.rx_state = PROTO_RX_LOCK;
            ProtocolEvent_t e = { .type = PROTO_EV_RX_LOCKED };
            e.u.locked.align = 0;
            e.u.locked.fraction = 0.12f;
            e.u.locked.moved = fb->settings.value[SETTING_RX_ALIGN] ? 2 : 0;
            fb->status.lock_align = e.u.locked.align;
            fb->status.lock_fraction = e.u.locked.fraction;
            fb->status.lock_moved = e.u.locked.moved;
            fb->status.rx_delay = (fb->status.rx_delay + (uint32_t)e.u.locked.moved) % FFT_SIZE;
            event(fb, e);
        }
        if (fb->listening && fb->sent_frames == TRAIN_FRAME) {
            fb->status.rx_state = PROTO_RX_HEADER;
            ProtocolEvent_t e = { .type = PROTO_EV_RX_TRAINED };
            e.u.trained.b_start = fb->frame - 10u;
            e.u.trained.coherence = 0.83f;
            e.u.trained.strongest = -1;
            e.u.trained.earliest = 0;
            fb->status.train_frame = e.u.trained.b_start;
            fb->status.train_coherence = e.u.trained.coherence;
            fb->status.train_strongest = e.u.trained.strongest;
            fb->status.train_earliest = e.u.trained.earliest;
            fb->status.train_moved = e.u.trained.moved;
            event(fb, e);
            learned = true;
        }
        if (fb->sent_frames == fb->total_frames) {
            fb->sending = false;
            ProtocolEvent_t done = { .type = PROTO_EV_TX_DONE };
            done.u.tx.id = fb->current.id;
            event(fb, done);
            if (fb->listening) {
                fb->listening = false;
                fb->status.rx_state = PROTO_RX_SEARCH;
                fb->status.good++;
                ProtocolEvent_t e = { .type = PROTO_EV_RX_PACKET };
                e.u.packet.crc_ok = true;
                e.u.packet.length = fb->current.length;
                e.u.packet.min_margin = 2.0f;
                e.u.packet.weakest = 0;
                e.u.packet.weak = 0;
                e.u.packet.payload = fb->current.payload;
                event(fb, e);
            }
        }
    }

    // Start the next packet (packet mode only, as main.c)
    if (!fb->sending && fb->queued > 0 && fb->settings.value[SETTING_TEST_MODE] == 5) {
        fb->current = fb->queue[0];
        memmove(&fb->queue[0], &fb->queue[1], (fb->queued - 1u) * sizeof fb->queue[0]);
        fb->queued--;
        fb->sending = true;
        fb->sent_frames = 0;
        fb->total_frames = fake_board_packet_frames(fb, fb->current.length);
        ProtocolEvent_t e = { .type = PROTO_EV_TX_START };
        e.u.tx.id = fb->current.id;
        event(fb, e);
    }
    fb->status.tx_queued = fb->queued;
    report_frame(fb, learned);
}

void fake_board_advance_us(FakeBoard_t *fb, uint64_t us) {
    const uint64_t end = fb->now_us + us;
    for (;;) {
        // Frame n ends at n * FFT_SIZE / SAMPLE_RATE seconds
        const uint64_t next = ((uint64_t)(fb->frame + 1u) * FFT_SIZE * 1000000u + SAMPLE_RATE - 1u) / SAMPLE_RATE;
        if (next > end) break;
        fb->now_us = next;
        one_frame(fb);
    }
    fb->now_us = end;
}

void fake_board_advance_frames(FakeBoard_t *fb, uint32_t frames) {
    for (uint32_t i = 0; i < frames; i++) {
        const uint64_t next = ((uint64_t)(fb->frame + 1u) * FFT_SIZE * 1000000u + SAMPLE_RATE - 1u) / SAMPLE_RATE;
        fake_board_advance_us(fb, next - fb->now_us);
    }
}

// ---------------------------------------------------------------------------
// The board interface
// ---------------------------------------------------------------------------
static void fb_apply(void *ctx, const ProtocolSettings_t *s, uint32_t changed) {
    FakeBoard_t *fb = ctx;
    if ((changed & (1u << SETTING_LINK_SPACING)) && fb->sending) fb->abandon = true;
    fb->settings = *s;
    fb->apply_calls++;
    fb->last_changed = changed;
}

static void fb_get_status(void *ctx, ProtocolStatus_t *st) {
    const FakeBoard_t *fb = ctx;
    *st = fb->status;
}

static int fb_send(void *ctx, const uint8_t *payload, uint32_t length, uint32_t id) {
    FakeBoard_t *fb = ctx;
    if (fb->queued == FAKE_TX_QUEUE) return PROTO_ERR_QUEUE_FULL;
    FakePacket_t *q = &fb->queue[fb->queued++];
    q->id = id;
    q->length = length;
    memcpy(q->payload, payload, length);
    fb->status.tx_queued = fb->queued;
    return 0;
}

static uint32_t fb_nudge(void *ctx, int32_t samples) {
    FakeBoard_t *fb = ctx;
    fb->status.rx_delay = (uint32_t)(((int32_t)fb->status.rx_delay + samples) % (int32_t)FFT_SIZE + (int32_t)FFT_SIZE) % FFT_SIZE;
    return fb->status.rx_delay;
}

static void fb_reset_counters(void *ctx) {
    FakeBoard_t *fb = ctx;
    ProtocolStatus_t *s = &fb->status;
    s->rx_missed = s->tx_missed = s->records_dropped = 0;
    s->good = s->bad = s->header_failed = s->lost = s->rx_dropped = 0;
}

static void fb_reset_rx(void *ctx) {
    FakeBoard_t *fb = ctx;
    fb->reset_rx = true; // As main.c: core 1 does it on its next pass
}

static bool fb_save(void *ctx, const ProtocolSettings_t *s) {
    FakeBoard_t *fb = ctx;
    if (!fb->save_enabled) return false;
    fb->saved = *s;
    fb->have_saved = true;
    return true;
}

static bool fb_load(void *ctx, ProtocolSettings_t *s) {
    FakeBoard_t *fb = ctx;
    if (!fb->have_saved) return false;
    *s = fb->saved;
    return true;
}

void fake_board_init(FakeBoard_t *fb, Protocol_t *proto) {
    memset(fb, 0, sizeof *fb);
    fb->proto = proto;
    protocol_default_settings(&fb->settings);
    fb->board = (ProtocolBoard_t){
        .ctx = fb,
        .version = "test",
        .board_id = "0011223344556677",
        .apply = fb_apply,
        .get_status = fb_get_status,
        .send = fb_send,
        .nudge = fb_nudge,
        .reset_counters = fb_reset_counters,
        .reset_rx = fb_reset_rx,
        .save = fb_save,
        .load = fb_load,
    };
    ProtocolStatus_t *s = &fb->status;
    s->sample_rate = 44100.0f;
    s->fft_size = FFT_SIZE;
    s->bins = PROTOCOL_CHIRP_BINS;
    s->f0 = 15000.0f;
    s->bandwidth = 44100.0f / FFT_SIZE * PROTOCOL_CHIRP_BINS;
    s->highpass_hz = 10000.0f;
    s->track_alpha = 0.25f;
    s->track_max_rate = 0.5f;
    s->track_rate_window = 4.0f;
    s->track_min_magnitude = 0.8f;
    s->rx_delay = 960;
}
