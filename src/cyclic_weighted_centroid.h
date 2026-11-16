#ifndef CYCLIC_WEIGHTED_CENTROID_H
#define CYCLIC_WEIGHTED_CENTROID_H

/**
 * @file cyclic_weighted_centroid.h
 *
 * Circular weighted centroid estimator for a circular power spectrum of
 * 'size' bins, such as the folded spectrum produced by the chirp decoder
 * (bin size - 1 is adjacent to bin 0).
 *
 * The spectrum is treated as a set of vectors on the unit circle. For a bin at
 * angle theta_i = 2*pi*i/size, the vector contribution is:
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
 *   angle = atan2(R_y, R_x)
 *   magnitude = |R|^2 / (sum_i w_i)^2
 *   strength = (|R|^2)^(1/(2 + k))
 *
 * This is a circular mean, not a raw weighted sum.
 *
 * Performance: power_exponent is an unsigned integer so the weights are
 * computed by multiplication only (k = 1..4 have dedicated loops, larger values
 * use repeated squaring). The choice is made once per accumulate call, not per
 * bin. k = 0 gives every bin equal weight.
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
    size_t size;           // Number of bins in the circular spectrum
    float32_t *cos_table;  // Precomputed cos(theta_i) values for the selected bins
    float32_t *sin_table;  // Precomputed sin(theta_i) values for the selected bins
    uint32_t power_exponent;  // Power used to emphasize stronger bins: w_i = p_i^k
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
 * magnitude: squared length of the normalized centroid vector; a value in [0, 1].
 *            This measures directional concentration, not signal level.
 * strength:  power-adjusted size of the unnormalized resultant. This retains
 *            information about how much weighted energy is present and is
 *            therefore more useful than magnitude for deciding whether a
 *            signal is being received. Its threshold depends on FFT scaling,
 *            microphone gain, and the noise floor.
 * bin: estimated bin index corresponding to the centroid angle
 */
typedef struct {
    float32_t angle;
    float32_t magnitude;
    float32_t strength; // Power-adjusted size of the unnormalized resultant
    float32_t bin;
    uint32_t ubin; // The unsigned integer bin index corresponding to the centroid angle
} CyclicWeightedCentroidResult_t;

/**
 * Initializes the centroid calculator.
 *
 * @param cwc          Target configuration object.
 * @param power_exponent Exponent used for weighting, where weight = p^k.
 * @param size         Number of bins in the circular spectrum.
 * @param cos_table    Preallocated cosine table of length at least size.
 * @param sin_table    Preallocated sine table of length at least size.
 */
void cyclic_weighted_centroid_init(
    CyclicWeightedCentroid_t *cwc,
    uint32_t power_exponent,
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
 * Accumulates the weighted bin vectors of a spectrum.
 *
 * @param cwc             Centroid configuration containing the bin mapping.
 * @param power_spectrum  Source power spectrum values (length = size).
 * @param power_exponent  Exponent used for the current accumulation pass.
 * @param result          Vector accumulator to add the weighted bin contributions to.
 */
void cyclic_weighted_centroid_accumulate(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    uint32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
);

/**
 * Finalizes the accumulated vector into a centroid angle, magnitude, and
 * strength.
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
 * preserved even when the vector is near the axes. The magnitude is the
 * squared length of the normalized centroid (the square root is skipped to
 * save cycles; ordering is preserved, but thresholds must be squared too):
 *
 *   |C|^2 = C_x^2 + C_y^2 = (R_x^2 + R_y^2) / (sum_i w_i)^2
 *
 * This gives a value in the range [0, 1] for a unit-circle interpretation,
 * which remains meaningful even if power_exponent changes.
 *
 * If the total weight is zero (e.g. silence), every result field is set to 0.
 *
 * Strength is calculated from the unnormalized resultant before normalization:
 *
 *   strength = (R_x^2 + R_y^2)^(1 / (2 + power_exponent))
 *
 * Unlike magnitude, strength reflects the total weighted energy in the window.
 * Use it for signal-presence decisions after establishing a threshold against
 * the measured noise floor.
 *
 * @param cwc     The centroid configuration.
 * @param vector  Accumulated centroid vector before normalization.
 * @param result  Output result structure containing the finalized angle, magnitude,
 *                strength, and corresponding bin estimate.
 */
void cyclic_weighted_centroid_finalize(
    CyclicWeightedCentroid_t *cwc,
    CyclicWeightedCentroidVector_t *vector,
    CyclicWeightedCentroidResult_t *result
);

/**
 * Finds the strongest bin of the spectrum.
 *
 * As a symbol decision this beats the centroid
 * angle in noise: the centroid is pulled off the peak by the noise floor in
 * every other bin, while the maximum is not.
 *
 * @param cwc             The centroid configuration.
 * @param power_spectrum  Source power spectrum values (length = size).
 * @return Bin index in [0, size).
 */
uint32_t cyclic_weighted_centroid_peak_bin(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum
);

/**
 * Refines a peak to sub-bin precision using the energy in a window around it.
 *
 * Picking the strongest bin is robust but insensitive to offsets of less than
 * about half a bin. This function starts from a peak bin (e.g. from
 * cyclic_weighted_centroid_peak_bin) and repeatedly moves
 * a window of +/- half_width bins to the power-weighted mean position
 * (exponent 1) of the bins inside it (a mean shift). The window slides
 * continuously, with bins partly inside it weighted proportionally, so the
 * result changes smoothly with the spectrum and does not depend on which of
 * several similar bins happened to be strongest.
 *
 * This copes with spread-out peaks such as an acoustic channel with echoes,
 * where energy arrives over several bins; a 3-point peak fit is unstable there.
 * The window wraps around the circular spectrum.
 *
 * The spectrum need not be the one the centroid uses: for a clean tone the
 * mean of whole-bin samples barely follows small shifts (nearly all the
 * energy sits in one bin), so the chirp decoder passes a spectrum sampled
 * every half bin (size = 2 * bins), which makes the estimate close to linear.
 *
 * @param power_spectrum  Source power spectrum values (length = size).
 * @param size            Number of points in the circular spectrum.
 * @param start_bin       Point to start from, in [0, size).
 * @param half_width      Window half-width in points (e.g. 3 bins = 6 points at half-bin spacing).
 * @return Refined angle in radians, in [0, 2*pi), with size points per turn.
 *         With no power in the window the angle of start_bin is returned.
 */
float32_t cyclic_weighted_centroid_refine_peak(
    const float32_t *power_spectrum,
    size_t size,
    uint32_t start_bin,
    uint32_t half_width
);

#endif // CYCLIC_WEIGHTED_CENTROID_H
