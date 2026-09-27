#ifndef CYCLIC_WEIGHTED_CENTROID_H
#define CYCLIC_WEIGHTED_CENTROID_H
/**
 * @file cyclic_weighted_centroid.h
 * 
 * The idea of these functions is to take the power spectrum of a signal and calculate the cyclic-weighted centroid 
 * of a small portion of the spectrum.
 * 
 * We are looking for a weighted peak within the transmitters allocated bandwidth. 
 * 
 * Imagine the power spectrum warped around a circle and each power value now pointing out from the center of the circle. 
 * The centroid is the average of all these vectors.
 * 
 * The power values can optionally be raised to a higher power to emphasize the peak and reduce the influence of noise.
 * 
 * As an aside, for this to work well we really want the bandwidth to fit an integer number of FFT bins.
 * 
 *   band_width_per_bin = sample_rate / fft_size
 *   signal_bandwidth = band_width_per_bin * n
 * 
 * where n is an integer. This way the signal will be centered in the bins and not split across two bins.
 * 
 * The signal is (sort of) reflected around the DC bin, so we need to be able run the calculation forward and backward.
 */

#include <stddef.h>
#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, etc.)

typedef struct {
    size_t fft_size; // Size of the FFT used to generate the power spectrum
    size_t size; // Number of bins to consider for the centroid calculation
    float32_t *cos_table; // Precomputed cosine values for the bins
    float32_t *sin_table; // Precomputed sine values for the bins
    float32_t power_exponent; // Exponent to raise the power spectrum values to
} CyclicWeightedCentroid_t;

typedef struct {
    float32_t x;
    float32_t y;
    float32_t weight_sum;
} CyclicWeightedCentroidVector_t;

typedef struct {
    float32_t angle; // The angle of the centroid in radians
    float32_t magnitude; // The magnitude of the centroid
    float32_t bin; // The bin index of the centroid
} CyclicWeightedCentroidResult_t;

void cyclic_weighted_centroid_init(
    CyclicWeightedCentroid_t *cwc,
    size_t fft_size,
    float32_t power_exponent,
    size_t size,
    float32_t *cos_table,
    float32_t *sin_table
);

void cyclic_weighted_centroid_init_vector(
    CyclicWeightedCentroidVector_t *vector
);

void cyclic_weighted_centroid_accumulate_backwards(
    CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
);

void cyclic_weighted_centroid_accumulate_forwards(
    CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
);

void cyclic_weighted_centroid_finalize(
    CyclicWeightedCentroid_t *cwc,
    CyclicWeightedCentroidVector_t *vector,
    CyclicWeightedCentroidResult_t *result
);

#endif // CYCLIC_WEIGHTED_CENTROID_H
