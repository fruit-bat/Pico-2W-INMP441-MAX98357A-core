#include <math.h>
#include <string.h>
#include "packet_rx.h"
#include "packet_format.h"
#include "pico/stdlib.h"

void packet_rx_init(
    PacketRx_t *rx,
    uint32_t bins,
    uint32_t bits_per_symbol,
    float32_t *storage,
    ChannelTemplate_t *tmpl
) {
    rx->bins = bins;
    rx->bits_per_symbol = bits_per_symbol;
    rx->bins_per_symbol = bins >> bits_per_symbol;
    rx->history = storage;
    rx->previous = storage + PACKET_RX_HISTORY * bins;
    rx->reference = rx->previous + bins;
    rx->shape_vector = rx->reference + bins;
    rx->tmpl = tmpl;
    memset(storage, 0, PACKET_RX_STORAGE_LEN(bins) * sizeof(*storage));
    memset(&rx->status, 0, sizeof(rx->status));
    packet_rx_reset(rx);
}

void packet_rx_reset(PacketRx_t *rx) {
    rx->status.state = PACKET_RX_SEARCH;
    rx->agree = 0;
    rx->lock_frames = 0;
    rx->tone = false;
}

static float32_t *history_frame(const PacketRx_t *rx, uint32_t frame) {
    return &rx->history[(frame % PACKET_RX_HISTORY) * rx->bins];
}

// Square-root power with the mean removed, scaled to unit length (all zero
// if the frame is flat)
static void __not_in_flash_func(correlation_vector)(const PacketRx_t *rx, const float32_t *power, float32_t *v) {
    const uint32_t n = rx->bins;
    float32_t mean = 0.0f;
    for (uint32_t k = 0; k < n; k++) {
        float32_t a;
        arm_sqrt_f32(power[k] > 0.0f ? power[k] : 0.0f, &a);
        v[k] = a;
        mean += a;
    }
    mean /= (float32_t)n;
    float32_t sum_sq = 0.0f;
    for (uint32_t k = 0; k < n; k++) {
        v[k] -= mean;
        sum_sq += v[k] * v[k];
    }
    float32_t norm;
    arm_sqrt_f32(sum_sq, &norm);
    const float32_t scale = norm > 0.0f ? 1.0f / norm : 0.0f;
    for (uint32_t k = 0; k < n; k++) v[k] *= scale;
}

static float32_t dot(const float32_t *a, const float32_t *b, uint32_t n) {
    float32_t s = 0.0f;
    for (uint32_t k = 0; k < n; k++) s += a[k] * b[k];
    return s;
}

// FFT bin of B symbol i
static uint32_t b_bin(const PacketRx_t *rx, uint32_t i) {
    return packet_b_symbol(i, rx->bits_per_symbol) * rx->bins_per_symbol;
}

// How well the B frames starting at 'start' agree once each is moved round
// so its known symbol sits at bin 0: |sum of unit vectors|^2 / count^2
static float32_t __not_in_flash_func(b_coherence)(const PacketRx_t *rx, uint32_t start) {
    const uint32_t n = rx->bins;
    float32_t sum[64];
    float32_t v[64];
    if (n > 64) return 0.0f;
    memset(sum, 0, n * sizeof(*sum));
    for (uint32_t i = 0; i < PACKET_B_LEN; i++) {
        correlation_vector(rx, history_frame(rx, start + i), v);
        const uint32_t bin = b_bin(rx, i);
        for (uint32_t o = 0; o < n; o++) sum[o] += v[(bin + o) % n];
    }
    return dot(sum, sum, n) / (float32_t)(PACKET_B_LEN * PACKET_B_LEN);
}

static int32_t signed_offset(uint32_t o, uint32_t n) {
    return o >= n / 2u ? (int32_t)o - (int32_t)n : (int32_t)o;
}

// Paths in a shape (power by bin offset from the window): the strongest,
// and the earliest within 6 dB of it. A later path lands at a lower bin, so
// the earliest is the highest one above the strongest, looked for only
// within PACKET_RX_EARLIEST_SPAN bins (the shape wraps, so further up it
// would be a long echo instead). Signed offsets, -bins/2 .. bins/2-1.
static void find_paths(const PacketRx_t *rx, const float32_t *shape, int32_t *strongest_out, int32_t *earliest_out) {
    const uint32_t n = rx->bins;
    uint32_t strongest = 0;
    for (uint32_t o = 1; o < n; o++) if (shape[o] > shape[strongest]) strongest = o;
    uint32_t earliest = strongest;
    for (uint32_t up = 1; up <= (uint32_t)PACKET_RX_EARLIEST_SPAN && up < n; up++) {
        const uint32_t o = (strongest + up) % n;
        if (shape[o] >= PACKET_RX_PATH_FRACTION * shape[strongest]) earliest = o;
    }
    *strongest_out = signed_offset(strongest, n);
    *earliest_out = signed_offset(earliest, n);
}

// Entering LOCK: the path shape of the agreeing A frames, each normalised as
// channel_template does, and how far the window is out. The agreeing frames
// are this one and the PACKET_RX_LOCK_FRAMES - 1 before it: each agreed with
// the frame before. The frame before them (what the first agreed with) is
// left out: with the window well out it holds only part of A, and its stray
// energy was once taken for the earliest path (board 2, 2026-11-18: moved
// 10 bins instead of 30, and the packet failed). Waiting for a fourth
// agreement keeps four frames in the average (packet_format.md)
static void __not_in_flash_func(measure_alignment)(PacketRx_t *rx, uint32_t frame) {
    const uint32_t n = rx->bins;
    float32_t shape[64];
    memset(shape, 0, n * sizeof(*shape));
    for (uint32_t i = 0; i < PACKET_RX_LOCK_FRAMES; i++) {
        const float32_t *p = history_frame(rx, frame - i);
        float32_t total = 0.0f;
        for (uint32_t k = 0; k < n; k++) total += p[k];
        if (total <= 0.0f) continue;
        for (uint32_t k = 0; k < n; k++) shape[k] += p[k] / total;
    }
    int32_t strongest, earliest;
    find_paths(rx, shape, &strongest, &earliest);
    rx->status.align_bins = earliest;

    // Fraction of a bin: mean of the centres on a one-bin circle, so
    // values either side of a bin edge (+0.49, -0.49) average correctly
    const float32_t two_pi = 6.28318531f;
    float32_t c = 0.0f, s = 0.0f;
    for (uint32_t i = 0; i < PACKET_RX_LOCK_FRAMES; i++) {
        const float32_t x = rx->centre[(frame - i) % PACKET_RX_HISTORY];
        c += cosf(two_pi * x);
        s += sinf(two_pi * x);
    }
    rx->status.align_fraction = atan2f(s, c) / two_pi;
}

// Moves a vector's contents 'bins' bins lower (what was at k goes to k - bins)
static void rotate_down(float32_t *v, uint32_t n, int32_t bins) {
    float32_t t[64];
    const uint32_t s = (uint32_t)(((bins % (int32_t)n) + (int32_t)n) % (int32_t)n);
    for (uint32_t k = 0; k < n; k++) t[k] = v[(k + s) % n];
    memcpy(v, t, n * sizeof(*v));
}

void packet_rx_window_moved(PacketRx_t *rx, int32_t bins) {
    if (rx->bins > 64) return;
    rotate_down(rx->reference, rx->bins, bins);
    rotate_down(rx->previous, rx->bins, bins);
}

// B is complete: find its start, learn the channel, report the paths
static void __not_in_flash_func(finish_training)(PacketRx_t *rx) {
    PacketRxStatus_t *st = &rx->status;
    float32_t best = -1.0f;
    int32_t best_adjust = 0;
    for (int32_t s = -PACKET_RX_START_SEARCH; s <= PACKET_RX_START_SEARCH; s++) {
        const float32_t c = b_coherence(rx, (uint32_t)((int32_t)rx->change_frame + s));
        if (c > best) {
            best = c;
            best_adjust = s;
        }
    }
    if (best < PACKET_RX_MIN_COHERENCE) {
        st->event = PACKET_RX_EVENT_TRAIN_FAILED;
        st->coherence = best;
        return;
    }

    const uint32_t start = (uint32_t)((int32_t)rx->change_frame + best_adjust);
    channel_template_reset(rx->tmpl);
    for (uint32_t i = 0; i < PACKET_B_LEN; i++) {
        channel_template_learn(rx->tmpl, history_frame(rx, start + i), b_bin(rx, i));
    }

    int32_t strongest, earliest;
    find_paths(rx, rx->tmpl->shape, &strongest, &earliest);

    st->event = PACKET_RX_EVENT_TRAINED;
    st->coherence = best;
    st->start_adjust = best_adjust;
    st->b_start_frame = start;
    st->strongest_offset = strongest;
    st->earliest_offset = earliest;

    // The shape as a correlation vector, for the data frames' quality
    // (correlation_vector takes power and square-roots it; the shape is
    // power too)
    correlation_vector(rx, rx->tmpl->shape, rx->shape_vector);

    // The data symbols follow B
    st->state = PACKET_RX_HEADER;
    rx->low_quality = 0;
    rx->next_data_frame = start + PACKET_B_LEN;
    rx->bit_count = 0;
    memset(rx->bytes, 0, sizeof(rx->bytes));
    st->data_count = 0;
    st->min_margin = 1e9f;
    st->weakest_symbol = 0;
    st->weak_symbols = 0;
}

const uint8_t *packet_rx_payload(const PacketRx_t *rx) {
    return &rx->bytes[PACKET_HEADER_BYTES];
}

static void back_to_search(PacketRx_t *rx) {
    rx->status.state = PACKET_RX_SEARCH;
    rx->agree = 0;
    rx->tone = false;
}

// How much a frame looks like a symbol: the best correlation, over the
// candidate symbols, of its correlation vector with the shape's
static float32_t __not_in_flash_func(frame_quality)(const PacketRx_t *rx, const float32_t *folded_power) {
    const uint32_t n = rx->bins;
    float32_t v[64];
    correlation_vector(rx, folded_power, v);
    float32_t best = -1.0f;
    for (uint32_t c = 0; c < n; c += rx->bins_per_symbol) {
        float32_t q = 0.0f;
        for (uint32_t o = 0; o < n; o++) q += v[(c + o) % n] * rx->shape_vector[o];
        if (q > best) best = q;
    }
    return best;
}

// Decides the data symbol in one frame and adds its bits (MSB first); then
// checks the header or finishes the packet when enough are in
static void __not_in_flash_func(decide_data)(PacketRx_t *rx, const float32_t *folded_power) {
    PacketRxStatus_t *st = &rx->status;
    const uint32_t b = rx->bits_per_symbol;
    float32_t margin;
    const uint32_t symbol = channel_template_decide(
        rx->tmpl, folded_power, 0, rx->bins_per_symbol, 1u << b, &margin);
    st->data_symbol = symbol;
    st->data_margin = margin;
    st->data_quality = frame_quality(rx, folded_power);
    rx->low_quality = st->data_quality < PACKET_RX_MIN_QUALITY ? rx->low_quality + 1u : 0u;
    if (rx->low_quality >= PACKET_RX_LOST_FRAMES) {
        st->event = PACKET_RX_EVENT_LOST;
        back_to_search(rx);
        return;
    }
    if (margin < st->min_margin) {
        st->min_margin = margin;
        st->weakest_symbol = st->data_count;
    }
    if (margin < PACKET_RX_WEAK_MARGIN) st->weak_symbols++;
    st->data_count++;
    const uint32_t value = packet_gray_value(symbol);
    for (uint32_t k = b; k-- > 0;) {
        const uint32_t byte = rx->bit_count / 8u;
        if (byte < PACKET_MAX_BYTES) rx->bytes[byte] |= (uint8_t)(((value >> k) & 1u) << (7u - rx->bit_count % 8u));
        rx->bit_count++;
    }

    if (st->state == PACKET_RX_HEADER && rx->bit_count >= 8u * PACKET_HEADER_BYTES) {
        st->length = rx->bytes[0];
        if (packet_crc8(rx->bytes, 1) != rx->bytes[1]) {
            st->event = PACKET_RX_EVENT_HEADER_FAILED;
            back_to_search(rx);
            return;
        }
        st->state = PACKET_RX_PAYLOAD;
        rx->data_total = packet_symbol_count(st->length, b) - PACKET_PREAMBLE_LEN;
    }
    if (st->state == PACKET_RX_PAYLOAD && st->data_count >= rx->data_total) {
        const uint32_t len = st->length;
        packet_scramble(&rx->bytes[PACKET_HEADER_BYTES], len);
        const uint16_t crc = packet_crc16(rx->bytes, PACKET_HEADER_BYTES + len);
        const uint16_t sent = (uint16_t)((rx->bytes[PACKET_HEADER_BYTES + len] << 8) | rx->bytes[PACKET_HEADER_BYTES + len + 1u]);
        st->crc_ok = crc == sent;
        st->event = PACKET_RX_EVENT_PACKET;
        back_to_search(rx);
    }
}

const PacketRxStatus_t *__not_in_flash_func(packet_rx_frame)(PacketRx_t *rx, const float32_t *folded_power, float32_t centre_bins) {
    PacketRxStatus_t *st = &rx->status;
    const uint32_t n = rx->bins;
    const uint32_t frame = st->frame;
    st->event = PACKET_RX_EVENT_NONE;

    memcpy(history_frame(rx, frame), folded_power, n * sizeof(*folded_power));
    rx->centre[frame % PACKET_RX_HISTORY] = centre_bins;
    float32_t v[64];
    if (n > 64) return st;
    correlation_vector(rx, folded_power, v);

    switch (st->state) {
        case PACKET_RX_SEARCH:
            st->correlation = dot(v, rx->previous, n);
            if (rx->tone) {
                // Ignoring a steady tone until it changes
                if (st->correlation < PACKET_RX_CHANGE_CORRELATION) rx->tone = false;
                break;
            }
            rx->agree = st->correlation >= PACKET_RX_LOCK_CORRELATION ? rx->agree + 1u : 0u;
            if (rx->agree >= PACKET_RX_LOCK_FRAMES) {
                st->state = PACKET_RX_LOCK;
                st->event = PACKET_RX_EVENT_LOCKED;
                memcpy(rx->reference, v, n * sizeof(*v));
                measure_alignment(rx, frame);
                rx->lock_frames = 0;
            }
            break;

        case PACKET_RX_LOCK:
            st->correlation = dot(v, rx->reference, n);
            if (st->correlation >= PACKET_RX_CHANGE_CORRELATION) {
                // Still A: fold this frame into the reference
                float32_t sum_sq = 0.0f;
                for (uint32_t k = 0; k < n; k++) {
                    rx->reference[k] += v[k];
                    sum_sq += rx->reference[k] * rx->reference[k];
                }
                float32_t norm;
                arm_sqrt_f32(sum_sq, &norm);
                if (norm > 0.0f) for (uint32_t k = 0; k < n; k++) rx->reference[k] /= norm;
                if (++rx->lock_frames > PACKET_RX_MAX_LOCK_FRAMES) {
                    st->event = PACKET_RX_EVENT_TONE;
                    st->state = PACKET_RX_SEARCH;
                    rx->tone = true;
                    rx->agree = 0;
                }
            } else {
                st->state = PACKET_RX_TRAIN;
                rx->change_frame = frame;
            }
            break;

        case PACKET_RX_TRAIN:
            st->correlation = dot(v, rx->previous, n);
            if (frame >= rx->change_frame + PACKET_B_LEN - 1u + (uint32_t)PACKET_RX_START_SEARCH) {
                finish_training(rx);
                if (st->state == PACKET_RX_TRAIN) back_to_search(rx); // B not found
            }
            break;

        case PACKET_RX_HEADER:
        case PACKET_RX_PAYLOAD:
            st->correlation = dot(v, rx->previous, n);
            break;
    }

    // Data frames, including any that arrived while B was being placed
    while ((st->state == PACKET_RX_HEADER || st->state == PACKET_RX_PAYLOAD) && rx->next_data_frame <= frame) {
        decide_data(rx, history_frame(rx, rx->next_data_frame));
        rx->next_data_frame++;
    }

    memcpy(rx->previous, v, n * sizeof(*v));
    st->frame = frame + 1u;
    return st;
}
