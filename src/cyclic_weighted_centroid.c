
#include "cyclic_weighted_centroid.h"

void cyclic_weighted_centroid_init(
    CyclicWeightedCentroid_t *cwc,
    size_t fft_size,
    float32_t power_exponent,
    size_t size,
    float32_t *cos_table,
    float32_t *sin_table
) {
    cwc->size = size;
    cwc->fft_size = fft_size;
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
    vector->weight_sum = 0.0f;
}

void cyclic_weighted_centroid_accumulate_backwards(
    CyclicWeightedCentroid_t *cwc, 
    const float32_t *power_spectrum,
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
) {
    for (size_t i = 0; i < cwc->size; i++) {
        const float32_t value = power_spectrum[cwc->fft_size - 1 - i];
        const float32_t weight = powf(value > 0.0f ? value : 0.0f, power_exponent);
        const size_t index = cwc->size - 1 - i;
        result->x += weight * cwc->cos_table[index];
        result->y += weight * cwc->sin_table[index];
        result->weight_sum += weight;
    }
}

void cyclic_weighted_centroid_accumulate_forwards(
    CyclicWeightedCentroid_t *cwc, 
    const float32_t *power_spectrum, 
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
) {
    for (size_t i = 0; i < cwc->size; i++) {
        const float32_t value = power_spectrum[i];
        const float32_t weight = powf(value > 0.0f ? value : 0.0f, power_exponent);
        result->x += weight * cwc->cos_table[i];
        result->y += weight * cwc->sin_table[i];
        result->weight_sum += weight;
    }
}

void cyclic_weighted_centroid_finalize(
    CyclicWeightedCentroid_t *cwc,
    CyclicWeightedCentroidVector_t *vector, 
    CyclicWeightedCentroidResult_t *result
) {
    if (vector->weight_sum <= 0.0f) {
        result->angle = 0.0f;
        result->magnitude = 0.0f;
        result->bin = 0.0f;
        return;
    }

    const float32_t x = vector->x / vector->weight_sum;
    const float32_t y = vector->y / vector->weight_sum;
    result->angle = atan2f(y, x);
    if (result->angle < 0.0f) {
        result->angle += 2.0f * PI;
    }
    result->magnitude = sqrtf(x * x + y * y);
    result->bin = (result->angle / (2.0f * PI)) * (float32_t)cwc->size;
}

