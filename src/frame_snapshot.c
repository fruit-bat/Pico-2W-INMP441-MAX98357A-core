#include <string.h>
#include "frame_snapshot.h"

void frame_snapshot_init(FrameSnapshot_t *snapshot, float32_t *buffer, uint32_t length) {
    snapshot->buffer = buffer;
    snapshot->length = length;
    atomic_init(&snapshot->state, FRAME_SNAPSHOT_REQUESTED);
}

void frame_snapshot_offer(FrameSnapshot_t *snapshot, const float32_t *frame) {
    if (atomic_load_explicit(&snapshot->state, memory_order_acquire) != FRAME_SNAPSHOT_REQUESTED) {
        return;
    }

    memcpy(snapshot->buffer, frame, snapshot->length * sizeof(*frame));

    // Publish the copy only after it has been fully written
    atomic_store_explicit(&snapshot->state, FRAME_SNAPSHOT_READY, memory_order_release);
}

const float32_t* frame_snapshot_acquire(FrameSnapshot_t *snapshot) {
    if (atomic_load_explicit(&snapshot->state, memory_order_acquire) != FRAME_SNAPSHOT_READY) {
        return NULL;
    }
    return snapshot->buffer;
}

void frame_snapshot_release(FrameSnapshot_t *snapshot) {
    atomic_store_explicit(&snapshot->state, FRAME_SNAPSHOT_REQUESTED, memory_order_release);
}
