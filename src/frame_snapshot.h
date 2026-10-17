#ifndef FRAME_SNAPSHOT_H
#define FRAME_SNAPSHOT_H

/**
 * @file frame_snapshot.h
 *
 * Request/ready handshake for taking a consistent copy of a frame of samples
 * (e.g. an FFT magnitude spectrum) from one core for use on the other.
 *
 * The consumer requests a snapshot; the producer copies its next frame into
 * the snapshot buffer only when one has been requested, so the copy costs
 * nothing when nobody is looking. The consumer then reads the buffer at its
 * leisure and releases it, which requests the next snapshot.
 *
 *   consumer                 producer
 *   --------                 --------
 *   (initially REQUESTED)
 *                            frame_snapshot_offer()   -> READY
 *   frame_snapshot_acquire() (non-NULL while READY)
 *   frame_snapshot_release() -> REQUESTED
 *
 * No memory is allocated; the caller provides the buffer.
 */

#include <stdatomic.h>
#include <stdint.h>
#include "arm_math.h"

typedef struct {
    float32_t *buffer;  // Snapshot storage (length = length)
    uint32_t length;    // Number of samples per frame
    atomic_uint state;  // FRAME_SNAPSHOT_REQUESTED or FRAME_SNAPSHOT_READY
} FrameSnapshot_t;

enum {
    FRAME_SNAPSHOT_REQUESTED = 0,
    FRAME_SNAPSHOT_READY = 1,
};

/**
 * Initializes the snapshot with a request already pending.
 *
 * @param snapshot Target snapshot state.
 * @param buffer   Caller-provided storage for length samples.
 * @param length   Number of samples per frame.
 */
void frame_snapshot_init(FrameSnapshot_t *snapshot, float32_t *buffer, uint32_t length);

/**
 * Producer: copies frame into the snapshot if one has been requested.
 *
 * @param snapshot Snapshot state.
 * @param frame    Source frame of length samples.
 */
void frame_snapshot_offer(FrameSnapshot_t *snapshot, const float32_t *frame);

/**
 * Consumer: returns the snapshot buffer if a snapshot is ready, otherwise NULL.
 * The buffer remains valid until frame_snapshot_release() is called.
 */
const float32_t* frame_snapshot_acquire(FrameSnapshot_t *snapshot);

/**
 * Consumer: finishes with the current snapshot and requests the next one.
 */
void frame_snapshot_release(FrameSnapshot_t *snapshot);

#endif // FRAME_SNAPSHOT_H
