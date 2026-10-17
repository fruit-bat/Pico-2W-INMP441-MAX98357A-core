#include <math.h>
#include "symbol_timing_tracker.h"
#include "pico/stdlib.h"

// Wraps x into [0, n)
static float32_t __not_in_flash_func(wrap)(float32_t x, float32_t n) {
    x = fmodf(x, n);
    return x < 0.0f ? x + n : x;
}

// Nearest integer delay in [0, frame_size)
static uint32_t __not_in_flash_func(to_output)(const SymbolTimingTracker_t *t) {
    return (uint32_t)lroundf(t->delay) % t->frame_size;
}

void symbol_timing_tracker_init(
    SymbolTimingTracker_t *t,
    uint32_t frame_size,
    float32_t alpha,
    float32_t beta,
    float32_t max_rate,
    float32_t rate_window,
    uint32_t delay
) {
    t->frame_size = frame_size;
    t->alpha = alpha;
    t->beta = beta;
    t->max_rate = max_rate;
    t->rate_window = rate_window;
    symbol_timing_tracker_reset(t, delay);
}

void __not_in_flash_func(symbol_timing_tracker_reset)(SymbolTimingTracker_t *t, uint32_t delay) {
    t->output = delay % t->frame_size;
    t->delay = (float32_t)t->output;
    t->rate = 0.0f;
}

uint32_t __not_in_flash_func(symbol_timing_tracker_update)(
    SymbolTimingTracker_t *t,
    uint32_t current_delay,
    float32_t error,
    bool valid
) {
    if (current_delay != t->output) {
        // Changed externally; carry on from the new delay
        symbol_timing_tracker_reset(t, current_delay);
    }

    if (valid) {
        if (fabsf(error) <= t->rate_window) {
            t->rate += t->beta * error;
            if (t->rate > t->max_rate) t->rate = t->max_rate;
            if (t->rate < -t->max_rate) t->rate = -t->max_rate;
        }
        t->delay += t->alpha * error;
    }
    t->delay = wrap(t->delay + t->rate, (float32_t)t->frame_size);

    t->output = to_output(t);
    return t->output;
}
