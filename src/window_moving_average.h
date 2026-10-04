#ifndef WINDOW_MOVING_AVERAGE_H
#define WINDOW_MOVING_AVERAGE_H

/**
 * @file window_moving_average.h
 *
 * Circular-buffer moving-average accumulator for streaming sensor data.
 *
 * The implementation keeps a fixed-size window of the most recent samples and a
 * running sum so the mean over the window can be computed in O(1) time each time
 * a new value arrives.
 */

#include <stddef.h>
#include <string.h>
#include "arm_math.h"

/**
 * State for a fixed-size sliding window average.
 *
 * buffer: storage for the last N samples in circular-buffer order.
 * buffer_size: number of samples retained in the window.
 * head: index where the next sample will be written.
 * count: number of valid samples currently in the window.
 * sum: running total of the samples currently in the window.
 */
typedef struct {
    float32_t *buffer;
    size_t buffer_size;
    size_t head;
    size_t count;
    float32_t sum;
} WindowMovingAverage_t;

/**
 * Initializes the sliding-window accumulator.
 *
 * @param ma Pointer to the accumulator state.
 * @param buffer Caller-provided storage for the window samples.
 * @param buffer_size Number of samples retained in the window.
 */
static inline void window_moving_average_init(WindowMovingAverage_t *ma, float32_t *buffer, size_t buffer_size) {
    ma->buffer = buffer;
    ma->buffer_size = buffer_size;
    ma->head = 0;
    ma->count = 0;
    ma->sum = 0.0f;

    if (buffer != NULL && buffer_size > 0U) {
        memset(buffer, 0, buffer_size * sizeof(*buffer));
    }
}

/**
 * Resets the windowed moving average to its initial empty state.
 *
 * @param ma Pointer to the accumulator state to clear.
 */
static inline void window_moving_average_reset(WindowMovingAverage_t *ma) {
    ma->head = 0;
    ma->count = 0;
    ma->sum = 0.0f;

    if (ma->buffer != NULL && ma->buffer_size > 0U) {
        memset(ma->buffer, 0, ma->buffer_size * sizeof(*ma->buffer));
    }
}

/**
 * Adds a new sample to the windowed moving average and returns the updated mean.
 *
 * The oldest sample is evicted when the buffer is full, keeping the window
 * length fixed.
 *
 * @param ma Pointer to the accumulator state.
 * @param new_value Sample value to include in the windowed mean.
 * @return Updated windowed moving average, or 0.0f if the buffer is not configured.
 */
static inline float32_t window_moving_average_add_value(WindowMovingAverage_t *ma, const float32_t new_value) {
    if (ma == NULL || ma->buffer == NULL || ma->buffer_size == 0U) {
        return 0.0f;
    }

    if (ma->count == ma->buffer_size) {
        ma->sum -= ma->buffer[ma->head];
    } else {
        ma->count++;
    }

    ma->buffer[ma->head] = new_value;
    ma->sum += new_value;
    ma->head = (ma->head + 1U) % ma->buffer_size;

    return ma->count == 0U ? 0.0f : ma->sum / (float32_t)ma->count;
}

#endif // WINDOW_MOVING_AVERAGE_H
