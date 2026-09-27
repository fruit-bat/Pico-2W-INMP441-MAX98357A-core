
#include "cyclic_weighted_centroid.h"

/**
 * @file cyclic_weighted_centroid.c
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

void cyclic_weighted_centroid_init(
    CyclicWeightedCentroid_t *cwc, 
    float32_t power_exponent,
    size_t size,
    float32_t *cos_table,
    float32_t *sin_table
) {
    cwc->size = size;
    cwc->cos_table = cos_table;
    cwc->sin_table = sin_table;
    cwc->power_exponent = power_exponent;

    for (size_t i = 0; i < size; i++) {
        const float32_t angle = 2.0f * PI * ((float32_t)i / (float32_t)size);
        cwc->cos_table[i] = cosf(angle);
        cwc->sin_table[i] = sinf(angle);
    }
}

void cyclic_weighted_centroid_init_vector(
    CyclicWeightedCentroidVector_t *vector
) {
    vector->x = 0.0f;
    vector->y = 0.0f;
}

void cyclic_weighted_centroid_accumulate_backwards(
    CyclicWeightedCentroid_t *cwc, 
    const float32_t *power_spectrum,  // pointer to the end of the power spectrum array for backwards accumulation
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
) {
    for (size_t i = 0; i < cwc->size; i++) {
        const size_t index = cwc->size - 1 - i; // Reverse index for backwards accumulation
        float32_t weight = powf(power_spectrum[-i], power_exponent);
        result->x += weight * cwc->cos_table[index];
        result->y += weight * cwc->sin_table[index];
    }
}

void cyclic_weighted_centroid_accumulate_forwards(
    CyclicWeightedCentroid_t *cwc, 
    const float32_t *power_spectrum, 
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
) {
    for (size_t i = 0; i < cwc->size; i++) {
        float32_t weight = powf(power_spectrum[i], power_exponent);
        result->x += weight * cwc->cos_table[i];
        result->y += weight * cwc->sin_table[i];
    }
}

void cyclic_weighted_centroid_finalize(
    CyclicWeightedCentroid_t *cwc,
    CyclicWeightedCentroidVector_t *vector, 
    CyclicWeightedCentroidResult_t *result
) {
    result->angle = atan2f(vector->y, vector->x);
    result->magnitude = powf(vector->x * vector->x + vector->y * vector->y, 1.0f /(2.0f + cwc->power_exponent));
}

