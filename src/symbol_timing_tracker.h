#ifndef SYMBOL_TIMING_TRACKER_H
#define SYMBOL_TIMING_TRACKER_H

/**
 * @file symbol_timing_tracker.h
 *
 * Second-order tracking loop that keeps the receive window aligned with the
 * incoming symbols.
 *
 * Each frame the caller supplies a timing error in samples (positive meaning
 * the window delay should increase). The tracker keeps a fractional delay
 * estimate and a drift rate (samples per frame, e.g. from a clock mismatch
 * between transmitter and receiver):
 *
 *   rate  += beta  * error        (only when |error| <= rate_window; clamped to +/- max_rate)
 *   delay += alpha * error + rate (wrapped into [0, frame_size))
 *
 * The rate is only learned once the loop is close to lock, so large errors
 * during acquisition do not wind it up. With beta = 0 it is a simple
 * first-order loop. Frames marked invalid (e.g.
 * no signal) do not update the estimate, but the delay keeps coasting at the
 * current rate.
 *
 * If the delay is changed externally (e.g. by the user), the tracker resyncs
 * to the new value and carries on from there.
 *
 * No memory is allocated.
 */

#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"

typedef struct {
    uint32_t frame_size;  // Delay wraps modulo this many samples
    float32_t alpha;      // Proportional gain, typically 0.1 .. 0.5
    float32_t beta;       // Rate gain, typically 0 .. 0.01; 0 disables rate tracking
    float32_t max_rate;   // Limit on |rate|, samples per frame
    float32_t rate_window; // Rate is only updated when |error| is within this many samples
    float32_t delay;      // Delay estimate in samples, in [0, frame_size)
    float32_t rate;       // Drift estimate, samples per frame
    uint32_t output;      // Integer delay most recently returned
} SymbolTimingTracker_t;

/**
 * Initializes the tracker.
 *
 * @param t          Target tracker state.
 * @param frame_size Frame length in samples; delays wrap modulo this.
 * @param alpha      Proportional gain.
 * @param beta       Rate gain (0 for first-order tracking).
 * @param max_rate   Limit on |rate| in samples per frame.
 * @param rate_window Rate is only updated when |error| is at most this many samples.
 * @param delay      Initial delay in samples.
 */
void symbol_timing_tracker_init(
    SymbolTimingTracker_t *t,
    uint32_t frame_size,
    float32_t alpha,
    float32_t beta,
    float32_t max_rate,
    float32_t rate_window,
    uint32_t delay
);

/**
 * Restarts tracking from the given delay with zero drift.
 */
void symbol_timing_tracker_reset(SymbolTimingTracker_t *t, uint32_t delay);

/**
 * Updates the tracker with one frame's measurement.
 *
 * @param t             Tracker state.
 * @param current_delay Delay in samples the frame was actually received with.
 *                      If it differs from the tracker's last output (changed
 *                      externally), the tracker resyncs to it first.
 * @param error         Timing error in samples; positive means increase the delay.
 * @param valid         false to ignore error for this frame (e.g. no signal).
 * @return New delay in samples, in [0, frame_size).
 */
uint32_t symbol_timing_tracker_update(
    SymbolTimingTracker_t *t,
    uint32_t current_delay,
    float32_t error,
    bool valid
);

#endif // SYMBOL_TIMING_TRACKER_H
