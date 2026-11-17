#ifndef PACKET_RX_H
#define PACKET_RX_H

/**
 * @file packet_rx.h
 *
 * Packet receiver, first stage: finds a packet's preamble (packet_format.h)
 * and learns the channel from it. Runs once per received frame on the
 * decoder's folded power spectrum. States so far:
 *
 *   SEARCH  waiting for a held symbol (preamble A): consecutive spectra
 *           that look alike
 *   LOCK    in A; waiting for the spectrum to change (the start of B)
 *   TRAIN   collecting B; then finds exactly which frame B started on, and
 *           learns the channel's shape from the B frames (channel_template)
 *
 * It works from spectra, not decoded symbols, because in a room the
 * strongest bin is often a reflection: "looks alike" is the correlation of
 * two frames' square-root folded power with the mean removed (noise gives
 * ~0, a held symbol close to 1, whichever paths are strong). B's start is
 * the candidate frame where the B frames, each moved round by its known
 * symbol, agree best (the "coherence", 0..1); a wrong start mixes
 * different shapes because B's neighbour differences are all distinct.
 *
 * A held symbol that lasts much longer than A (e.g. chirp test mode) is
 * reported as a steady tone and ignored until it changes.
 *
 * Any constant offset between the transmitter's and receiver's frames is
 * absorbed into the learned shape: the shape's peaks are the arrival paths,
 * at bin offsets from where the receiver's window currently sits. A window
 * d samples late shows a path d/16 bins higher; a path d samples later than
 * another shows d/16 bins lower. Shapes wrap every bins (64 bins = a frame),
 * so a path more than half a frame later looks earlier instead; the
 * "earliest" path is therefore only looked for within
 * PACKET_RX_EARLIEST_SPAN bins above the strongest.
 *
 * Window alignment: A's frames all hold the same symbol, so however far the
 * window is out they don't mix two symbols. On entering LOCK the receiver
 * learns the path shape from the agreeing A frames and reports, with
 * PACKET_RX_EVENT_LOCKED, how far the window is out (align_bins: the
 * earliest strong path's offset). The caller may move its window by that
 * many bins (later paths keep their place relative to it) and must then
 * call packet_rx_window_moved() so the receiver's reference follows. B is
 * then read with the window in line, so each B frame holds one symbol.
 * Without the move, near half a frame out B's start is ambiguous by one
 * frame (every B frame mixes two symbols).
 *
 * No memory is allocated; the caller provides PACKET_RX_STORAGE_LEN(bins)
 * floats and a channel template.
 */

#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"
#include "channel_template.h"

#define PACKET_RX_HISTORY 16u // Frames of folded power kept (B plus a margin either side)
#define PACKET_RX_STORAGE_LEN(bins) ((PACKET_RX_HISTORY + 2u) * (bins))

// Thresholds on the frame-to-frame correlation and on B's coherence
#define PACKET_RX_LOCK_CORRELATION   0.5f  // SEARCH -> LOCK when this many consecutive frames agree...
#define PACKET_RX_LOCK_FRAMES        3u    // ...this many times in a row
#define PACKET_RX_CHANGE_CORRELATION 0.4f  // LOCK -> TRAIN when a frame agrees with A less than this
#define PACKET_RX_MAX_LOCK_FRAMES    16u   // Longer than this in LOCK: a steady tone, not a packet
#define PACKET_RX_START_SEARCH       2     // B's start is looked for this many frames either side of the change
#define PACKET_RX_MIN_COHERENCE      0.3f  // Below this B is not accepted
#define PACKET_RX_PATH_FRACTION      0.25f // Paths within 6 dB of the strongest count as "strong"
#define PACKET_RX_EARLIEST_SPAN      20    // The earliest strong path is looked for this many bins above the strongest

typedef enum {
    PACKET_RX_SEARCH = 0,
    PACKET_RX_LOCK = 1,
    PACKET_RX_TRAIN = 2,
} PacketRxState_t;

typedef enum {
    PACKET_RX_EVENT_NONE = 0,
    PACKET_RX_EVENT_LOCKED,       // Entered LOCK (a held symbol); see align_bins
    PACKET_RX_EVENT_TRAINED,      // B found and learned; see the status fields
    PACKET_RX_EVENT_TRAIN_FAILED, // The change after A was not B
    PACKET_RX_EVENT_TONE,         // A held symbol went on too long; ignored until it changes
} PacketRxEvent_t;

typedef struct {
    PacketRxState_t state;
    PacketRxEvent_t event;        // What happened on this frame
    uint32_t frame;               // Frames processed so far
    float32_t correlation;        // This frame against the previous one (SEARCH) or A (LOCK)
    // Set with PACKET_RX_EVENT_LOCKED (and kept until the next one):
    int32_t align_bins;           // Earliest strong path in A, bins from the window's position: the
                                  // window is this many bins late (move it earlier to correct)
    // Set with PACKET_RX_EVENT_TRAINED (and kept until the next one):
    float32_t coherence;          // How well the B frames agreed at the chosen start (0..1)
    int32_t start_adjust;         // Chosen B start relative to the frame where the change was seen
    uint32_t b_start_frame;       // Frame number of B's first frame
    int32_t strongest_offset;     // Strongest path, bins from the window's position (-bins/2 .. bins/2-1)
    int32_t earliest_offset;      // Earliest path within 6 dB of the strongest (and within
                                  // PACKET_RX_EARLIEST_SPAN bins above it), same units
} PacketRxStatus_t;

typedef struct {
    uint32_t bins;                // Folded spectrum length
    uint32_t bits_per_symbol;     // 4 .. 6
    uint32_t bins_per_symbol;
    float32_t *history;           // PACKET_RX_HISTORY frames of folded power (ring)
    float32_t *previous;          // Previous frame, as a correlation vector
    float32_t *reference;         // A's average, as a correlation vector
    ChannelTemplate_t *tmpl;      // Learned shape (from B)
    uint32_t agree;               // SEARCH: consecutive agreeing frames
    uint32_t lock_frames;         // LOCK: frames so far
    bool tone;                    // Ignoring a steady tone until it changes
    uint32_t change_frame;        // Frame where LOCK saw the change
    PacketRxStatus_t status;
} PacketRx_t;

void packet_rx_init(
    PacketRx_t *rx,
    uint32_t bins,
    uint32_t bits_per_symbol,
    float32_t *storage,
    ChannelTemplate_t *tmpl
);

// Back to SEARCH (keeps the frame count)
void packet_rx_reset(PacketRx_t *rx);

// Processes one frame's folded power spectrum (bins values)
const PacketRxStatus_t *packet_rx_frame(PacketRx_t *rx, const float32_t *folded_power);

// The caller has moved its window 'bins' bins earlier (bins * frame / bins
// samples) from the next frame on: what was at bin k is now at bin k - bins
void packet_rx_window_moved(PacketRx_t *rx, int32_t bins);

#endif // PACKET_RX_H
