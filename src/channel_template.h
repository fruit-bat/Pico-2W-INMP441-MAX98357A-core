#ifndef CHANNEL_TEMPLATE_H
#define CHANNEL_TEMPLATE_H

/**
 * @file channel_template.h
 *
 * Channel-matched symbol decision.
 *
 * In a room, each transmitted symbol reaches the receiver along several
 * paths. After de-chirping and folding, every path shows up as a copy of the
 * symbol's peak at a fixed bin offset (a path d samples later lands d/16
 * bins lower), so the folded spectrum of every symbol has the same shape,
 * just moved round to that symbol's bin. Measured between two boards 2 m
 * apart, the copies were as strong as the direct path, and deciding from the
 * strongest bin got only 30..74% right.
 *
 * The template learns that shape from frames whose symbol is known (e.g. a
 * preamble): each frame's folded power, normalised and moved round so the
 * known symbol sits at bin 0, is added up. A frame is then decided by
 * correlating the square root of its folded power with the template at each
 * candidate symbol bin, so every path's energy counts towards the right
 * symbol. On the same recordings this got 99.9..100% right, learning from as
 * few as 2..4 frames.
 *
 * The receive window must be aligned with the transmitter's frames (not just
 * the symbol grid) for this to work: a misaligned window mixes two symbols
 * into each frame.
 *
 * Until any frame has been learned, the template is a single bin at offset
 * 0, which makes the decision the strongest candidate bin.
 *
 * Cost per decision: candidates x bins multiply-adds (64 x 64 = 4096) plus
 * bins square roots. No memory is allocated; the caller provides storage of
 * CHANNEL_TEMPLATE_STORAGE_LEN(bins) floats.
 */

#include <stdint.h>
#include "arm_math.h"

#define CHANNEL_TEMPLATE_STORAGE_LEN(bins) (2u * (bins))

typedef struct {
    uint32_t bins;       // Folded spectrum length (bins in the chirp band)
    float32_t *shape;    // Learned shape, by offset from the symbol's bin (bins)
    float32_t *work;     // Scratch for a frame's amplitudes (bins)
    uint32_t frames;     // Frames learned since the last reset
} ChannelTemplate_t;

void channel_template_init(ChannelTemplate_t *t, uint32_t bins, float32_t *storage);

// Forgets everything learned
void channel_template_reset(ChannelTemplate_t *t);

// Learns from one frame's folded power whose transmitted symbol is known to
// sit at known_bin
void channel_template_learn(ChannelTemplate_t *t, const float32_t *folded_power, uint32_t known_bin);

// Decides between count candidate bins first_bin + i * step (mod bins),
// i = 0 .. count - 1, returning the best i. If margin is not NULL it
// receives the best score over the second best (>= 1; near 1 means unsure),
// and if second is not NULL it receives the second best i (the runner-up).
uint32_t channel_template_decide(
    const ChannelTemplate_t *t,
    const float32_t *folded_power,
    uint32_t first_bin,
    uint32_t step,
    uint32_t count,
    float32_t *margin,
    uint32_t *second
);

#endif // CHANNEL_TEMPLATE_H
