#ifndef RUNNING_MOVING_AVERAGE_H
#define RUNNING_MOVING_AVERAGE_H

/**
 * @file running_moving_average.h
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
} RunningMovingAverage_t;

/**
 * Resets the running average to its initial empty state.
 *
 * @param ma Pointer to the accumulator state to clear.
 */
static inline void running_moving_average_reset(RunningMovingAverage_t *ma) {
    ma->average = 0.0f;
    ma->count = 0;
}

/**
 * Adds a new sample to the running average and returns the updated mean.
 *
 * @param ma Pointer to the accumulator state.
 * @param new_value Sample value to include in the running mean.
 * @return Updated moving average.
 */
static inline float32_t running_moving_average_add_value(RunningMovingAverage_t *ma, const float32_t new_value) {
    ma->count++;
    // Numerically stable formula to prevent precision drift across long runs.
    ma->average += (new_value - ma->average) / (float32_t)ma->count;
    return ma->average;
}

#endif // RUNNING_MOVING_AVERAGE_H
