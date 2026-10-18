
#include <stdbool.h>
#include "cyclic_weighted_centroid.h"
#include "pico/stdlib.h"

void cyclic_weighted_centroid_init(
    CyclicWeightedCentroid_t *cwc,
    size_t fft_size,
    uint32_t power_exponent,
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
// Weight for one bin: value^k.
//
static inline float32_t weight_of(float32_t value, uint32_t k) {
    switch (k) {
        case 0:  return 1.0f;
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
    bool backwards,
    CyclicWeightedCentroidVector_t *result
) {
    float32_t x = result->x;
    float32_t y = result->y;
    float32_t weight_sum = result->weight_sum;

    for (size_t i = 0; i < cwc->size; i++) {
        const float32_t value = backwards ? power_spectrum[cwc->fft_size - 1 - i] : power_spectrum[i];
        const size_t index = backwards ? cwc->size - 1 - i : i;
        const float32_t weight = weight_of(value > 0.0f ? value : 0.0f, k);
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
    uint32_t power_exponent,
    bool backwards,
    CyclicWeightedCentroidVector_t *result
) {
    switch (power_exponent) {
        case 1:  accumulate(cwc, power_spectrum, 1, backwards, result); break;
        case 2:  accumulate(cwc, power_spectrum, 2, backwards, result); break;
        case 3:  accumulate(cwc, power_spectrum, 3, backwards, result); break;
        case 4:  accumulate(cwc, power_spectrum, 4, backwards, result); break;
        default: accumulate(cwc, power_spectrum, power_exponent, backwards, result); break; // int_pow
    }
}

void __not_in_flash_func(cyclic_weighted_centroid_accumulate_backwards)(
    CyclicWeightedCentroid_t *cwc, 
    const float32_t *power_spectrum,
    uint32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
) {
    accumulate_dispatch(cwc, power_spectrum, power_exponent, true, result);
}

void __not_in_flash_func(cyclic_weighted_centroid_accumulate_forwards)(
    CyclicWeightedCentroid_t *cwc, 
    const float32_t *power_spectrum, 
    uint32_t power_exponent,
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
    result->strength = powf(x * x + y * y, 1.0f / (2.0f + (float32_t)cwc->power_exponent)); // Adjust magnitude calculation based on power exponent
    result->bin = (result->angle / (2.0f * PI)) * (float32_t)cwc->size;
    const long int ubin = lroundf(result->bin) % (long int)cwc->size;
    result->ubin = ubin < 0 ? (uint32_t)(ubin + cwc->size) : (uint32_t)ubin;
}


// Power of bin index (mod size), combined with its mirrored negative-frequency bin
static inline float32_t folded_power(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    int32_t index
) {
    const int32_t size = (int32_t)cwc->size;
    const uint32_t i = (uint32_t)(((index % size) + size) % size);
    return power_spectrum[i] + power_spectrum[cwc->fft_size - cwc->size + i];
}

// Power-weighted mean position, in bins relative to centre, of the folded bins
// within +/- half_width of centre. Returns 0 if the window holds no power.
static float32_t __not_in_flash_func(window_offset)(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    int32_t centre,
    int32_t half_width
) {
    float32_t moment = 0.0f;
    float32_t total = 0.0f;
    for (int32_t k = -half_width; k <= half_width; k++) {
        const float32_t p = folded_power(cwc, power_spectrum, centre + k);
        moment += (float32_t)k * p;
        total += p;
    }
    return total > 0.0f ? moment / total : 0.0f;
}

float32_t __not_in_flash_func(cyclic_weighted_centroid_refine_peak)(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    const CyclicWeightedCentroidResult_t *result,
    uint32_t half_width
) {
    const int32_t size = (int32_t)cwc->size;
    const int32_t w = (int32_t)half_width;

    // Re-centre on the energy, then measure around the new centre
    int32_t centre = (int32_t)result->ubin;
    centre += (int32_t)lroundf(window_offset(cwc, power_spectrum, centre, w));
    centre = ((centre % size) + size) % size;
    const float32_t offset = window_offset(cwc, power_spectrum, centre, w);

    float32_t angle = ((float32_t)centre + offset) * (2.0f * PI / (float32_t)size);
    if (angle < 0.0f) angle += 2.0f * PI;
    if (angle >= 2.0f * PI) angle -= 2.0f * PI;
    return angle;
}
