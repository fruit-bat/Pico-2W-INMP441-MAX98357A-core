#include <assert.h>
#include <string.h>
#include "spsc_ring.h"
#include "pico/stdlib.h"

void spsc_ring_init(SpscRing_t *ring, void *storage, uint32_t element_size, uint32_t capacity) {
    assert(capacity > 0 && (capacity & (capacity - 1)) == 0);

    ring->storage = storage;
    ring->element_size = element_size;
    ring->capacity = capacity;
    atomic_init(&ring->head, 0);
    atomic_init(&ring->tail, 0);
}

bool __not_in_flash_func(spsc_ring_push)(SpscRing_t *ring, const void *element) {
    const unsigned head = atomic_load_explicit(&ring->head, memory_order_relaxed);
    const unsigned tail = atomic_load_explicit(&ring->tail, memory_order_acquire);
    if (head - tail >= ring->capacity) {
        return false;
    }

    memcpy(&ring->storage[(head & (ring->capacity - 1)) * ring->element_size], element, ring->element_size);

    // Publish the element only after it has been fully written
    atomic_store_explicit(&ring->head, head + 1, memory_order_release);
    return true;
}

bool spsc_ring_pop(SpscRing_t *ring, void *element) {
    const unsigned tail = atomic_load_explicit(&ring->tail, memory_order_relaxed);
    const unsigned head = atomic_load_explicit(&ring->head, memory_order_acquire);
    if (head == tail) {
        return false;
    }

    memcpy(element, &ring->storage[(tail & (ring->capacity - 1)) * ring->element_size], ring->element_size);

    // Hand the slot back to the producer only after it has been fully read
    atomic_store_explicit(&ring->tail, tail + 1, memory_order_release);
    return true;
}
