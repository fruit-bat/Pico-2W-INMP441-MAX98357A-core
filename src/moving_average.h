#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

/**
 * @file moving_average.h
 *
 * Simple running-average accumulator for streaming sensor data.
 *
 * This implementation uses the standard numerically stable update formula:
 *
 *   average = average + (new_value - average) / count
 *
 * which avoids the accumulation error that would occur if you summed every
 * sample and divided at the end.
 */

#include <stddef.h>
#include "arm_math.h"

/**
 * State for a running mean.
 *
 * average: current exponentially-updated average.
 * count: number of samples incorporated so far.
 */
typedef struct {
    float32_t average;
    size_t count;
} MovingAverage_t;

/**
 * Resets the running average to its initial empty state.
 *
 * @param ma Pointer to the accumulator state to clear.
 */
static inline void reset_moving_average(MovingAverage_t *ma) {
    ma->average = 0.0f;
    ma->count = 0;
}

/**
 * Adds a new sample to the running average and returns the updated mean.
 *
 * The accumulator is intentionally mutable because it stores the running state.
 * A caller passing a const-qualified pointer would silently fail to compile or
 * would incorrectly imply that no state change occurs.
 *
 * @param ma Pointer to the accumulator state.
 * @param new_value Sample value to include in the running mean.
 * @return Updated moving average.
 */
static inline float32_t add_moving_average_value(MovingAverage_t *ma, const float32_t new_value) {
    ma->count++;
    // Numerically stable formula to prevent precision drift across long runs.
    ma->average += (new_value - ma->average) / (float32_t)ma->count;
    return ma->average;
}

#endif // MOVING_AVERAGE_H
