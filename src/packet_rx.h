#ifndef PACKET_RX_H
#define PACKET_RX_H

/**
 * @file packet_rx.h
 *
 * Packet receiver, first stage: finds a packet's preamble (packet_format.h)
 * and learns the channel from it. Runs once per received frame on the
 * decoder's folded power spectrum. States:
 *
 *   SEARCH  waiting for a held symbol (preamble A): consecutive spectra
 *           that look alike
 *   LOCK    in A; waiting for the spectrum to change (the start of B)
 *   TRAIN   collecting B; then finds exactly which frame B started on, and
 *           learns the channel's shape from the B frames (channel_template)
 *   HEADER  decides data symbols with the learned shape, until the header's
 *           16 bits are in; checks its CRC-8 (bad: back to SEARCH)
 *   PAYLOAD decides the rest; when the last symbol is in, descrambles the
 *           payload, checks the CRC-16 and reports the packet (good or bad)
 *           with PACKET_RX_EVENT_PACKET; back to SEARCH
 *
 * In HEADER and PAYLOAD each frame's "quality" is checked: the correlation
 * (mean removed, 0..1) of its square-root power with the learned shape at
 * the best candidate symbol. A symbol scores about 0.85 between the boards,
 * noise about 0.3, independent of level. PACKET_RX_LOST_FRAMES frames in a
 * row below PACKET_RX_MIN_QUALITY abandon the packet (PACKET_RX_EVENT_LOST),
 * so a header that passed its CRC-8 by chance, or a transmitter that
 * stopped, doesn't hold the receiver for the rest of a long payload.
 *
 * Data symbols are decided by channel_template_decide: each candidate
 * symbol's score is the correlation of the frame with the learned shape
 * moved to that symbol, so reflections count for the symbol rather than
 * against it. The margin is best / second-best score; low margins mark the
 * decisions most likely to be wrong.
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
 * PACKET_RX_EVENT_LOCKED, how far the window is out: align_bins, the
 * earliest strong path's offset in whole bins, plus align_fraction, where
 * the energy sits within a bin (the decoder's refined centre, averaged over
 * the same frames). The caller may move its window by the sum and must then
 * call packet_rx_window_moved() with the whole bins so the receiver's
 * reference follows. The fraction matters: with the energy half a bin off a
 * bin centre it splits between neighbouring bins, and between the boards
 * nearly every data decision then came down to two adjacent symbols. B is
 * then read with the window in line, so each B frame holds one symbol.
 * Without the move, near half a frame out B's start is ambiguous by one
 * frame (every B frame mixes two symbols).
 *
 * Undoing a wrong move: a held symbol can't tell an early path from a long
 * echo, because the shape wraps. In the room an echo ~44 bins (~700
 * samples) late sits 20 bins above the strongest, the edge of
 * PACKET_RX_EARLIEST_SPAN, and now and then reaches the -6 dB threshold:
 * the lock then moves the window ~19 bins the wrong way and the packet
 * fails (soak test 2026-11-19: all 14 such packets). B's symbols change,
 * so training sees it: the strongest path is back where the window was
 * before the move. When the lock's move was at least
 * PACKET_RX_UNDO_MIN_BINS and training's strongest path is within
 * PACKET_RX_UNDO_TOLERANCE bins of undoing it, PACKET_RX_EVENT_TRAINED
 * comes with realign_bins = minus that move; the caller moves the window
 * back (whole bins: the lock's fraction still holds) and calls
 * packet_rx_window_moved() again, which also moves the learned shape. Data
 * frames that came before the move (up to PACKET_RX_START_SEARCH + 2) were
 * decided already, with the window where it was. The test is that narrow
 * on purpose: training's strongest path alone is no guide (a later path
 * can be the strongest on a window that is right), and training's
 * earliest wraps just as A's does.
 *
 * No memory is allocated; the caller provides PACKET_RX_STORAGE_LEN(bins)
 * floats and a channel template.
 */

#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"
#include "channel_template.h"
#include "packet_format.h"

#define PACKET_RX_HISTORY 16u // Frames of folded power kept (B plus a margin either side)
#define PACKET_RX_STORAGE_LEN(bins) ((PACKET_RX_HISTORY + 3u) * (bins))

// Thresholds on the frame-to-frame correlation and on B's coherence
#define PACKET_RX_LOCK_CORRELATION   0.5f  // SEARCH -> LOCK when this many consecutive frames agree...
#define PACKET_RX_LOCK_FRAMES        4u    // ...this many times in a row (A is 8 frames: the window still moves before B)
#define PACKET_RX_CHANGE_CORRELATION 0.4f  // LOCK -> TRAIN when a frame agrees with A less than this
#define PACKET_RX_MAX_LOCK_FRAMES    16u   // Longer than this in LOCK: a steady tone, not a packet
#define PACKET_RX_START_SEARCH       2     // B's start is looked for this many frames either side of the change
#define PACKET_RX_MIN_COHERENCE      0.3f  // Below this B is not accepted
#define PACKET_RX_PATH_FRACTION      0.25f // Paths within 6 dB of the strongest count as "strong"
#define PACKET_RX_EARLIEST_SPAN      20    // The earliest strong path is looked for this many bins above the strongest
#define PACKET_RX_UNDO_MIN_BINS      8     // A lock move at least this big is undone after B if...
#define PACKET_RX_UNDO_TOLERANCE     3     // ...training's strongest path is within this many bins of undoing it
#define PACKET_RX_WEAK_MARGIN        1.2f  // Data decisions with a margin below this are counted as weak
#define PACKET_RX_MIN_QUALITY        0.55f // Data frames below this don't look like a symbol...
#define PACKET_RX_LOST_FRAMES        8u    // ...and this many in a row (~190 ms) abandon the packet. Not
                                           // fewer: loud bursts of room noise drop the quality for 2-3
                                           // frames while the signal is still there

typedef enum {
    PACKET_RX_SEARCH = 0,
    PACKET_RX_LOCK = 1,
    PACKET_RX_TRAIN = 2,
    PACKET_RX_HEADER = 3,
    PACKET_RX_PAYLOAD = 4,
} PacketRxState_t;

typedef enum {
    PACKET_RX_EVENT_NONE = 0,
    PACKET_RX_EVENT_LOCKED,       // Entered LOCK (a held symbol); see align_bins
    PACKET_RX_EVENT_TRAINED,      // B found and learned; see the status fields
    PACKET_RX_EVENT_TRAIN_FAILED, // The change after A was not B
    PACKET_RX_EVENT_TONE,         // A held symbol went on too long; ignored until it changes
    PACKET_RX_EVENT_HEADER_FAILED, // The header's CRC-8 was wrong
    PACKET_RX_EVENT_PACKET,       // A packet is complete; see the packet fields and packet_rx_payload()
    PACKET_RX_EVENT_LOST,         // The signal went during the header or payload; abandoned
} PacketRxEvent_t;

typedef struct {
    PacketRxState_t state;
    PacketRxEvent_t event;        // What happened on this frame
    uint32_t frame;               // Frames processed so far
    float32_t correlation;        // This frame against the previous one (SEARCH) or A (LOCK)
    // Set with PACKET_RX_EVENT_LOCKED (and kept until the next one):
    int32_t align_bins;           // Earliest strong path in A, bins from the window's position: the
                                  // window is this many bins late (move it earlier to correct)
    float32_t align_fraction;     // Plus this much of a bin (-0.5 .. 0.5): where A's energy sits
                                  // relative to the nearest bin centre
    // Set with PACKET_RX_EVENT_TRAINED (and kept until the next one):
    float32_t coherence;          // How well the B frames agreed at the chosen start (0..1)
    int32_t start_adjust;         // Chosen B start relative to the frame where the change was seen
    uint32_t b_start_frame;       // Frame number of B's first frame
    int32_t strongest_offset;     // Strongest path, bins from the window's position (-bins/2 .. bins/2-1)
    int32_t earliest_offset;      // Earliest path within 6 dB of the strongest (and within
                                  // PACKET_RX_EARLIEST_SPAN bins above it), same units
    int32_t realign_bins;         // 0, or the lock's move was wrong: the window is this many bins
                                  // late, as align_bins (move it, whole bins only)
    // In HEADER and PAYLOAD: the last data symbol decided (on this frame or
    // just before), and its margin
    uint32_t data_symbol;
    float32_t data_margin;
    float32_t data_quality;       // Its quality (see above)
    uint32_t data_count;          // Data symbols decided so far in this packet
    // Set with PACKET_RX_EVENT_HEADER_FAILED / PACKET (and kept until the next one):
    uint32_t length;              // Payload length from the header
    bool crc_ok;                  // CRC-16 good (PACKET only)
    float32_t min_margin;         // Lowest data decision margin in the packet
    uint32_t weakest_symbol;      // Which data symbol it was (0 = first after B)
    uint32_t weak_symbols;        // Data decisions with a margin below PACKET_RX_WEAK_MARGIN
} PacketRxStatus_t;

// Data symbols in the longest packet (header, payload and CRC at the fewest
// bits per symbol)
#define PACKET_RX_MAX_DATA_SYMBOLS ((8u * PACKET_MAX_BYTES + PACKET_MIN_BITS_PER_SYMBOL - 1u) / PACKET_MIN_BITS_PER_SYMBOL)

// One data symbol's decision, kept for measurement (how wrong decisions
// look, for choosing error correction)
typedef struct {
    uint8_t symbol;               // The symbol decided (before Gray decoding)
    uint8_t second;               // The runner-up
    uint16_t margin;              // Its margin in thousandths (1000 = a tie), at most 65535
} PacketRxDecision_t;

typedef struct {
    uint32_t bins;                // Folded spectrum length
    uint32_t bits_per_symbol;     // 4 .. 6
    uint32_t bins_per_symbol;
    float32_t *history;           // PACKET_RX_HISTORY frames of folded power (ring)
    float32_t *previous;          // Previous frame, as a correlation vector
    float32_t *reference;         // A's average, as a correlation vector
    float32_t *shape_vector;      // The learned shape, as a correlation vector (for the quality)
    ChannelTemplate_t *tmpl;      // Learned shape (from B)
    uint32_t agree;               // SEARCH: consecutive agreeing frames
    uint32_t lock_frames;         // LOCK: frames so far
    bool tone;                    // Ignoring a steady tone until it changes
    uint32_t change_frame;        // Frame where LOCK saw the change
    int32_t lock_moved;           // Bins the caller moved the window since the last lock
    uint32_t next_data_frame;     // HEADER, PAYLOAD: next frame to decide
    float32_t centre[PACKET_RX_HISTORY]; // The decoder's refined centre per frame (bins, ring)
    uint32_t data_total;          // PAYLOAD: data symbols in this packet
    uint32_t bit_count;           // Bits collected into bytes
    uint32_t low_quality;         // Consecutive data frames below PACKET_RX_MIN_QUALITY
    uint8_t bytes[PACKET_MAX_BYTES]; // Header, payload (descrambled once complete), CRC-16
    PacketRxDecision_t decisions[PACKET_RX_MAX_DATA_SYMBOLS]; // This packet's data symbols so far (status.data_count)
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

// Processes one frame's folded power spectrum (bins values). centre_bins:
// the decoder's refined centre for the frame (its angle), in bins; only its
// fraction of a bin is used
const PacketRxStatus_t *packet_rx_frame(PacketRx_t *rx, const float32_t *folded_power, float32_t centre_bins);

// After PACKET_RX_EVENT_PACKET: the payload (status.length bytes), valid
// until the next frame is processed
const uint8_t *packet_rx_payload(const PacketRx_t *rx);

// After PACKET_RX_EVENT_HEADER_FAILED or PACKET: the packet's data symbol
// decisions (status.data_count of them, header first), valid until the
// next frame is processed
const PacketRxDecision_t *packet_rx_decisions(const PacketRx_t *rx);

// The caller has moved its window 'bins' bins earlier (bins * frame / bins
// samples) from the next frame on: what was at bin k is now at bin k - bins.
// The receiver's reference, previous frame and learned shape follow
// (after PACKET_RX_EVENT_LOCKED, and after TRAINED with realign_bins)
void packet_rx_window_moved(PacketRx_t *rx, int32_t bins);

#endif // PACKET_RX_H
