#ifndef SPSC_RING_H
#define SPSC_RING_H

/**
 * @file spsc_ring.h
 *
 * Lock-free single-producer / single-consumer ring buffer of fixed-size
 * elements, safe for passing data between the two RP2350 cores.
 *
 * Exactly one context may push and exactly one context may pop. Neither side
 * ever blocks: push fails when the ring is full and pop fails when it is
 * empty, so the real-time producer is never held up by a slow consumer.
 *
 * No memory is allocated; the caller provides the storage.
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

// Required storage length, in bytes, for capacity elements of element_size bytes.
#define SPSC_RING_STORAGE_LEN(element_size, capacity) ((element_size) * (capacity))

typedef struct {
    uint8_t *storage;      // capacity * element_size bytes
    uint32_t element_size; // Size of one element in bytes
    uint32_t capacity;     // Number of elements; must be a power of two
    atomic_uint head;      // Free-running count of pushed elements (written by producer)
    atomic_uint tail;      // Free-running count of popped elements (written by consumer)
} SpscRing_t;

/**
 * Initializes an empty ring.
 *
 * @param ring         Target ring state.
 * @param storage      Caller-provided storage of SPSC_RING_STORAGE_LEN(element_size, capacity) bytes.
 * @param element_size Size of one element in bytes.
 * @param capacity     Number of elements; must be a power of two.
 */
void spsc_ring_init(SpscRing_t *ring, void *storage, uint32_t element_size, uint32_t capacity);

/**
 * Producer: copies one element into the ring.
 *
 * @return true on success, false if the ring was full (element not added).
 */
bool spsc_ring_push(SpscRing_t *ring, const void *element);

/**
 * Consumer: copies the oldest element out of the ring.
 *
 * @return true on success, false if the ring was empty.
 */
bool spsc_ring_pop(SpscRing_t *ring, void *element);

#endif // SPSC_RING_H
