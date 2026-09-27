#ifndef CYCLIC_WEIGHTED_CENTROID_H
#define CYCLIC_WEIGHTED_CENTROID_H

#include <stddef.h>
#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, etc.)

typedef struct {
    size_t size; // FFT bin span for the centroid calculation
    float32_t *cos_table;
    float32_t *sin_table;
    float32_t power_exponent; // Exponent to raise the power spectrum values to
} CyclicWeightedCentroid_t;

typedef struct {
    float32_t x;
    float32_t y;
} CyclicWeightedCentroidVector_t;

typedef struct {
    float32_t angle; // The angle of the centroid in radians
    float32_t magnitude; // The magnitude of the centroid
} CyclicWeightedCentroidResult_t;

void cyclic_weighted_centroid_init(
    CyclicWeightedCentroid_t *cwc,
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
