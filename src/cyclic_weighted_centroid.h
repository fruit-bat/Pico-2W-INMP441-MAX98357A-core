#ifndef CYCLIC_WEIGHTED_CENTROID_H
#define CYCLIC_WEIGHTED_CENTROID_H

/**
 * @file cyclic_weighted_centroid.h
 *
 * Circular weighted centroid estimator for a narrow band of an FFT power spectrum.
 *
 * The spectrum is treated as a set of vectors on the unit circle. For a bin at
 * angle theta_i, the vector contribution is:
 *
 *   w_i = p_i^k
 *   v_i = w_i * [cos(theta_i), sin(theta_i)]
 *
 * where p_i is the power-spectral value in that bin, k is the chosen
 * power_exponent, and theta_i is the phase angle for that bin.
 *
 * The centroid is the normalized average of these vectors:
 *
 *   R = sum_i w_i * [cos(theta_i), sin(theta_i)]
 *   C = R / sum_i w_i
 *   angle = atan2(C_y, C_x)
 *   magnitude = |C|
 *
 * This is a circular mean, not a raw weighted sum. Normalizing by the total
 * weight keeps the magnitude meaningful when the exponent is changed.
 *
 * For an FFT, the spectral bins are mirrored about DC, so the same occupied
 * band may appear on both the forward and mirrored negative-frequency side. The
 * API therefore supports accumulation in either direction, using the reflected
 * negative-frequency bins when needed.
 *
 * A good configuration is to choose a bandwidth that is an integer number of FFT
 * bins so a narrowband signal stays centered and does not split across adjacent
 * bins.
 */

#include <stddef.h>
#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, atan2f, sqrtf)

/**
 * Configuration for the centroid estimator.
 *
 * The caller prepares a table of sin/cos values for the angular positions to use
 * during accumulation. These values are generated for the selected bin span and
 * reused to avoid repeated trigonometric work.
 */
typedef struct {
    size_t fft_size;       // FFT length used to generate the source power spectrum
    size_t size;           // Number of bins included in this centroid window
    float32_t *cos_table;  // Precomputed cos(theta_i) values for the selected bins
    float32_t *sin_table;  // Precomputed sin(theta_i) values for the selected bins
    float32_t power_exponent; // Power used to emphasize stronger bins: w_i = p_i^k
} CyclicWeightedCentroid_t;

/**
 * Accumulated vector state for the centroid sum.
 *
 * x and y hold the accumulated Cartesian components of the weighted bin vectors.
 * weight_sum holds the total weight used for normalization.
 */
typedef struct {
    float32_t x;
    float32_t y;
    float32_t weight_sum;
} CyclicWeightedCentroidVector_t;

/**
 * Final centroid result.
 *
 * angle: angular position of the centroid in radians, normalized into [0, 2*pi)
 * magnitude: length of the normalized centroid vector; a value in [0, 1]
 * bin: estimated bin index corresponding to the centroid angle
 */
typedef struct {
    float32_t angle;
    float32_t magnitude;
    float32_t bin;
} CyclicWeightedCentroidResult_t;

/**
 * Initializes the centroid calculator.
 *
 * @param cwc          Target configuration object.
 * @param fft_size     FFT length used to interpret the spectrum indexing.
 * @param power_exponent Exponent used for weighting, where weight = p^k.
 * @param size         Number of bins to process in the centroid window.
 * @param cos_table    Preallocated cosine table of length at least size.
 * @param sin_table    Preallocated sine table of length at least size.
 */
void cyclic_weighted_centroid_init(
    CyclicWeightedCentroid_t *cwc,
    size_t fft_size,
    float32_t power_exponent,
    size_t size,
    float32_t *cos_table,
    float32_t *sin_table
);

/**
 * Resets a running vector accumulator.
 *
 * @param vector The accumulator to clear.
 */
void cyclic_weighted_centroid_init_vector(
    CyclicWeightedCentroidVector_t *vector
);

/**
 * Accumulates the mirrored negative-frequency portion of the spectrum.
 *
 * The caller passes a pointer into the FFT magnitude/power spectrum and the
 * function walks the mirrored portion from the upper end of the array toward the
 * center. This matches the convention where negative-frequency bins are located
 * near the end of the FFT buffer.
 *
 * @param cwc             Centroid configuration containing the bin mapping.
 * @param power_spectrum  Source power spectrum values.
 * @param power_exponent  Exponent used for the current accumulation pass.
 * @param result          Vector accumulator to add the weighted bin contributions to.
 */
void cyclic_weighted_centroid_accumulate_backwards(
    CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
);

/**
 * Accumulates the forward (positive-frequency) part of the spectrum.
 *
 * @param cwc             Centroid configuration containing the bin mapping.
 * @param power_spectrum  Source power spectrum values.
 * @param power_exponent  Exponent used for the current accumulation pass.
 * @param result          Vector accumulator to add the weighted bin contributions to.
 */
void cyclic_weighted_centroid_accumulate_forwards(
    CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
);

/**
 * Finalizes the accumulated vector into a centroid angle and magnitude.
 *
 * The accumulated vector is:
 *
 *   R = sum_i w_i * [cos(theta_i), sin(theta_i)]
 *
 * and the normalized centroid is:
 *
 *   C = R / sum_i w_i
 *
 * The final angle is computed using atan2(C_y, C_x) so the correct quadrant is
 * preserved even when the vector is near the axes. The magnitude is then:
 *
 *   |C| = sqrt(C_x^2 + C_y^2)
 *
 * This gives a value in the range [0, 1] for a unit-circle interpretation,
 * which remains meaningful even if power_exponent changes.
 *
 * @param cwc     The centroid configuration.
 * @param vector  Accumulated centroid vector before normalization.
 * @param result  Output result structure containing the finalized angle, magnitude,
 *                and corresponding bin estimate.
 */
void cyclic_weighted_centroid_finalize(
    CyclicWeightedCentroid_t *cwc,
    CyclicWeightedCentroidVector_t *vector,
    CyclicWeightedCentroidResult_t *result
);

#endif // CYCLIC_WEIGHTED_CENTROID_H
