#ifndef RX_SAMPLE_WINDOW_H
#define RX_SAMPLE_WINDOW_H

/**
 * @file rx_sample_window.h
 *
 * Double-buffered receive window used to align received frames with the
 * transmitted symbol timing.
 *
 * Incoming blocks of frame_size samples are written alternately into two
 * buffers. A frame can then be read starting 'delay' samples before the start
 * of the newest block:
 *
 *   frame = older[frame_size - delay .. frame_size - 1] ++ newer[0 .. frame_size - delay - 1]
 *
 * With delay = 0 the frame is the newest block unchanged.
 *
 * No memory is allocated; the caller provides the storage.
 */

#include <stdint.h>
#include "arm_math.h"

// Required storage length, in float32_t elements, for a given frame size.
#define RX_SAMPLE_WINDOW_STORAGE_LEN(frame_size) ((frame_size) * 2u)

typedef struct {
    float32_t *storage;      // Two consecutive blocks (length = frame_size * 2)
    uint32_t frame_size;     // Samples per block and per frame
    uint32_t newest;         // Index (0 or 1) of the most recently written block
    volatile uint32_t delay; // Alignment offset in samples, in [0, frame_size); may be changed from another core
} RxSampleWindow_t;

/**
 * Initializes the window and clears the storage.
 *
 * @param w          Target window state.
 * @param storage    Caller-provided storage of RX_SAMPLE_WINDOW_STORAGE_LEN(frame_size) elements.
 * @param frame_size Samples per block and per frame.
 * @param delay      Initial alignment offset in samples (wrapped into [0, frame_size)).
 */
void rx_sample_window_init(
    RxSampleWindow_t *w,
    float32_t *storage,
    uint32_t frame_size,
    uint32_t delay
);

/**
 * Advances to the next block and returns where to write its frame_size samples.
 * The returned block becomes the newest block.
 */
float32_t* rx_sample_window_next_block(RxSampleWindow_t *w);

/**
 * Sets the alignment offset in samples (wrapped into [0, frame_size)).
 */
void rx_sample_window_set_delay(RxSampleWindow_t *w, uint32_t delay);

/**
 * Returns the current alignment offset in samples.
 */
uint32_t rx_sample_window_get_delay(const RxSampleWindow_t *w);

/**
 * Copies the aligned frame into frame_out.
 *
 * @param w         Window state.
 * @param frame_out Destination for frame_size samples.
 */
void rx_sample_window_read(const RxSampleWindow_t *w, float32_t *frame_out);

#endif // RX_SAMPLE_WINDOW_H
