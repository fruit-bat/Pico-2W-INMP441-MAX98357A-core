
#include "cyclic_weighted_centroid.h"
#include "pico/stdlib.h"

void cyclic_weighted_centroid_init(
    CyclicWeightedCentroid_t *cwc,
    uint32_t power_exponent,
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
// k, compiles to its own loop with no per-bin branching.
//
static inline __attribute__((always_inline)) void accumulate(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    uint32_t k,
    CyclicWeightedCentroidVector_t *result
) {
    float32_t x = result->x;
    float32_t y = result->y;
    float32_t weight_sum = result->weight_sum;

    for (size_t i = 0; i < cwc->size; i++) {
        const float32_t value = power_spectrum[i];
        const float32_t weight = weight_of(value > 0.0f ? value : 0.0f, k);
        x += weight * cwc->cos_table[i];
        y += weight * cwc->sin_table[i];
        weight_sum += weight;
    }

    result->x = x;
    result->y = y;
    result->weight_sum = weight_sum;
}

//
// Chooses the weighting method once per call rather than once per bin.
//
void __not_in_flash_func(cyclic_weighted_centroid_accumulate)(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum,
    uint32_t power_exponent,
    CyclicWeightedCentroidVector_t *result
) {
    switch (power_exponent) {
        case 1:  accumulate(cwc, power_spectrum, 1, result); break;
        case 2:  accumulate(cwc, power_spectrum, 2, result); break;
        case 3:  accumulate(cwc, power_spectrum, 3, result); break;
        case 4:  accumulate(cwc, power_spectrum, 4, result); break;
        default: accumulate(cwc, power_spectrum, power_exponent, result); break; // int_pow
    }
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


// Power of point index, taken mod size (the spectrum is circular)
static inline float32_t circular_power(
    const float32_t *power_spectrum,
    int32_t size,
    int32_t index
) {
    return power_spectrum[((index % size) + size) % size];
}

uint32_t __not_in_flash_func(cyclic_weighted_centroid_peak_bin)(
    const CyclicWeightedCentroid_t *cwc,
    const float32_t *power_spectrum
) {
    uint32_t peak_bin = 0;
    float32_t peak_power = power_spectrum[0];
    for (uint32_t i = 1; i < cwc->size; i++) {
        const float32_t p = power_spectrum[i];
        if (p > peak_power) {
            peak_power = p;
            peak_bin = i;
        }
    }
    return peak_bin;
}

// Number of times the window is moved to its own power-weighted mean
#define CWC_REFINE_ITERATIONS 4

//
// Power-weighted mean position (in points) of the circular spectrum inside the
// window [centre - half_width, centre + half_width]. The window slides
// continuously: bins only partly inside it get a proportional weight, so the
// result changes smoothly with centre. Returns centre if the window holds no power.
//
static float32_t __not_in_flash_func(window_mean)(
    const float32_t *power_spectrum,
    int32_t size,
    float32_t centre,
    float32_t half_width
) {
    const float32_t lo = centre - half_width;
    const float32_t hi = centre + half_width;
    float32_t moment = 0.0f;
    float32_t total = 0.0f;
    for (int32_t k = (int32_t)floorf(lo); k <= (int32_t)ceilf(hi); k++) {
        float32_t weight = 1.0f;
        if ((float32_t)k < lo) weight = 1.0f - (lo - (float32_t)k);
        if ((float32_t)k > hi) weight = 1.0f - ((float32_t)k - hi);
        if (weight <= 0.0f) continue;
        const float32_t p = weight * circular_power(power_spectrum, size, k);
        moment += (float32_t)k * p;
        total += p;
    }
    return total > 0.0f ? moment / total : centre;
}

float32_t __not_in_flash_func(cyclic_weighted_centroid_refine_peak)(
    const float32_t *power_spectrum,
    size_t size,
    uint32_t start_bin,
    uint32_t half_width
) {
    // Mean shift: start at the peak bin and repeatedly move the window to the
    // power-weighted mean of what it contains
    float32_t centre = (float32_t)start_bin;
    for (uint32_t i = 0; i < CWC_REFINE_ITERATIONS; i++) {
        centre = window_mean(power_spectrum, (int32_t)size, centre, (float32_t)half_width);
    }

    float32_t angle = centre * (2.0f * PI / (float32_t)size);
    angle = fmodf(angle, 2.0f * PI);
    if (angle < 0.0f) angle += 2.0f * PI;
    return angle;
}
