
#include <stdbool.h>
#include "cyclic_weighted_centroid.h"
#include "pico/stdlib.h"

// Largest exponent handled by integer multiplication rather than powf
#define CWC_MAX_INTEGER_EXPONENT 32u

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

void __not_in_flash_func(cyclic_weighted_centroid_init_vector)(
    CyclicWeightedCentroidVector_t *vector
) {
    vector->x = 0.0f;
    vector->y = 0.0f;
    vector->weight_sum = 0.0f;
}

//
// Returns power_exponent as an integer in [1, CWC_MAX_INTEGER_EXPONENT] if it
// is a whole number in that range, otherwise 0 (meaning: use powf).
//
static uint32_t __not_in_flash_func(integer_exponent)(float32_t power_exponent) {
    if (power_exponent >= 1.0f &&
        power_exponent <= (float32_t)CWC_MAX_INTEGER_EXPONENT &&
        power_exponent == (float32_t)(uint32_t)power_exponent) {
        return (uint32_t)power_exponent;
    }
    return 0;
}

// value^k by repeated squaring: about log2(k) multiplications
static inline float32_t __not_in_flash_func(int_pow)(float32_t value, uint32_t k) {
    float32_t result = 1.0f;
    while (k != 0) {
        if (k & 1u) result *= value;
        value *= value;
        k >>= 1;
    }
    return result;
}

//
// Weight for one bin. k is the integer exponent from integer_exponent(), or 0
// to use powf with the real-valued power_exponent.
//
static inline float32_t weight_of(float32_t value, uint32_t k, float32_t power_exponent) {
    switch (k) {
        case 0:  return powf(value, power_exponent);
        case 1:  return value;
        case 2:  return value * value;
        case 3:  return value * value * value;
        case 4:  { const float32_t v2 = value * value; return v2 * v2; }
        default: return int_pow(value, k);
    }
}

//
// Accumulation loop. Always inlined so that each call below, with constant
// k and direction, compiles to its own loop with no per-bin branching.
//
static inline __attribute__((always_inline)) void accumulate(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    uint32_t k,
    float32_t power_exponent,
    bool backwards,
    CyclicWeightedCentroidVector_t *result
) {
    float32_t x = result->x;
    float32_t y = result->y;
    float32_t weight_sum = result->weight_sum;

    for (size_t i = 0; i < cwc->size; i++) {
        const float32_t value = backwards ? power_spectrum[cwc->fft_size - 1 - i] : power_spectrum[i];
        const size_t index = backwards ? cwc->size - 1 - i : i;
        const float32_t weight = weight_of(value > 0.0f ? value : 0.0f, k, power_exponent);
        x += weight * cwc->cos_table[index];
        y += weight * cwc->sin_table[index];
        weight_sum += weight;
    }

    result->x = x;
    result->y = y;
    result->weight_sum = weight_sum;
}

//
// Chooses the weighting method once per call rather than once per bin.
//
static void __not_in_flash_func(accumulate_dispatch)(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    float32_t power_exponent,
    bool backwards,
    CyclicWeightedCentroidVector_t *result
) {
    const uint32_t k = integer_exponent(power_exponent);
    switch (k) {
        case 1:  accumulate(cwc, power_spectrum, 1, power_exponent, backwards, result); break;
        case 2:  accumulate(cwc, power_spectrum, 2, power_exponent, backwards, result); break;
        case 3:  accumulate(cwc, power_spectrum, 3, power_exponent, backwards, result); break;
        case 4:  accumulate(cwc, power_spectrum, 4, power_exponent, backwards, result); break;
        default: accumulate(cwc, power_spectrum, k, power_exponent, backwards, result); break; // int_pow, or powf when k == 0
    }
}

void __not_in_flash_func(cyclic_weighted_centroid_accumulate_backwards)(
    CyclicWeightedCentroid_t *cwc, 
    const float32_t *power_spectrum,
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
) {
    accumulate_dispatch(cwc, power_spectrum, power_exponent, true, result);
}

void __not_in_flash_func(cyclic_weighted_centroid_accumulate_forwards)(
    CyclicWeightedCentroid_t *cwc, 
    const float32_t *power_spectrum, 
    float32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
) {
    accumulate_dispatch(cwc, power_spectrum, power_exponent, false, result);
}

void __not_in_flash_func(cyclic_weighted_centroid_finalize)(
    CyclicWeightedCentroid_t *cwc,
    CyclicWeightedCentroidVector_t *vector, 
    CyclicWeightedCentroidResult_t *result
) {
    if (vector->weight_sum <= 0.0f) {
        result->angle = 0.0f;
        result->magnitude = 0.0f;
        result->strength = 0.0f;
        result->bin = 0.0f;
        result->ubin = 0;
        return;
    }

    const float32_t x = vector->x;
    const float32_t y = vector->y;
    result->angle = atan2f(y, x);
    if (result->angle < 0.0f) {
        result->angle += 2.0f * PI;
    }
    result->magnitude = (x * x + y * y) / (vector->weight_sum * vector->weight_sum);
    result->strength = powf(x * x + y * y, 1.0f / (2.0f + cwc->power_exponent)); // Adjust magnitude calculation based on power exponent
    result->bin = (result->angle / (2.0f * PI)) * (float32_t)cwc->size;
    const long int ubin = lroundf(result->bin) % (long int)cwc->size;
    result->ubin = ubin < 0 ? (uint32_t)(ubin + cwc->size) : (uint32_t)ubin;
}

