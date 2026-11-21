#include "reed_solomon.h"
#include "pico/stdlib.h"

// GF(256) with x^8 + x^4 + x^3 + x^2 + 1. gf_exp is doubled so a product's
// logs can be added without reducing mod 255
static uint8_t gf_exp[512];
static uint8_t gf_log[256];
static uint8_t generator[RS_PARITY + 1u]; // Highest power first; generator[0] = 1
static bool ready;

static inline uint8_t gf_mul(uint8_t a, uint8_t b) {
    return (a && b) ? gf_exp[gf_log[a] + gf_log[b]] : 0u;
}

static inline uint8_t gf_div(uint8_t a, uint8_t b) { // b != 0
    return a ? gf_exp[gf_log[a] + 255u - gf_log[b]] : 0u;
}

// 2^e, any e >= 0
static inline uint8_t gf_pow2(uint32_t e) {
    return gf_exp[e % 255u];
}

void reed_solomon_init(void) {
    if (ready) return;
    uint32_t x = 1;
    for (uint32_t i = 0; i < 255u; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100u) x ^= 0x11Du;
    }
    for (uint32_t i = 255; i < 512u; i++) gf_exp[i] = gf_exp[i - 255u];

    // generator = (x - 2^0)(x - 2^1)...(x - 2^(RS_PARITY-1))
    generator[0] = 1;
    for (uint32_t j = 0; j < RS_PARITY; j++) {
        const uint8_t root = gf_pow2(j);
        generator[j + 1u] = 0;
        for (uint32_t k = j + 1u; k > 0; k--) generator[k] ^= gf_mul(generator[k - 1u], root);
    }
    ready = true;
}

void __not_in_flash_func(reed_solomon_encode)(const uint8_t *data, uint32_t length, uint8_t *parity) {
    // The remainder of data(x) * x^RS_PARITY divided by the generator, by
    // long division one data byte at a time
    for (uint32_t k = 0; k < RS_PARITY; k++) parity[k] = 0;
    for (uint32_t i = 0; i < length; i++) {
        const uint8_t feedback = data[i] ^ parity[0];
        for (uint32_t k = 0; k + 1u < RS_PARITY; k++) parity[k] = parity[k + 1u] ^ gf_mul(generator[k + 1u], feedback);
        parity[RS_PARITY - 1u] = gf_mul(generator[RS_PARITY], feedback);
    }
}

// The received polynomial at 2^j for j = 0 .. RS_PARITY-1; true if all zero
static bool __not_in_flash_func(syndromes)(const uint8_t *codeword, uint32_t length, uint8_t *s) {
    bool clean = true;
    for (uint32_t j = 0; j < RS_PARITY; j++) {
        const uint8_t x = gf_pow2(j);
        uint8_t v = 0;
        for (uint32_t i = 0; i < length; i++) v = gf_mul(v, x) ^ codeword[i];
        s[j] = v;
        if (v) clean = false;
    }
    return clean;
}

int32_t __not_in_flash_func(reed_solomon_decode)(uint8_t *codeword, uint32_t length) {
    if (length <= RS_PARITY || length > RS_MAX_CODEWORD) return -1;
    uint8_t s[RS_PARITY];
    if (syndromes(codeword, length, s)) return 0;

    // Berlekamp-Massey: the shortest error locator lambda(x) (lowest power
    // first, lambda[0] = 1) whose roots are the inverses of the error positions
    uint8_t lambda[RS_PARITY + 1u] = { 1 };
    uint8_t prev[RS_PARITY + 1u] = { 1 };
    uint32_t errors = 0, shift = 1;
    uint8_t prev_discrepancy = 1;
    for (uint32_t n = 0; n < RS_PARITY; n++) {
        uint8_t d = s[n];
        for (uint32_t i = 1; i <= errors; i++) d ^= gf_mul(lambda[i], s[n - i]);
        if (d == 0) {
            shift++;
            continue;
        }
        const uint8_t scale = gf_div(d, prev_discrepancy);
        uint8_t updated[RS_PARITY + 1u];
        for (uint32_t i = 0; i <= RS_PARITY; i++) {
            updated[i] = lambda[i] ^ (i >= shift ? gf_mul(scale, prev[i - shift]) : 0u);
        }
        if (2u * errors <= n) {
            for (uint32_t i = 0; i <= RS_PARITY; i++) prev[i] = lambda[i];
            errors = n + 1u - errors;
            prev_discrepancy = d;
            shift = 1;
        } else {
            shift++;
        }
        for (uint32_t i = 0; i <= RS_PARITY; i++) lambda[i] = updated[i];
    }
    if (errors > RS_MAX_FIXED) return -1;

    // omega(x) = s(x) lambda(x) mod x^RS_PARITY, the error evaluator
    uint8_t omega[RS_PARITY] = { 0 };
    for (uint32_t i = 0; i < RS_PARITY; i++) {
        for (uint32_t k = 0; k <= i; k++) omega[i] ^= gf_mul(s[i - k], lambda[k]);
    }

    // Chien search over the codeword's positions (byte i is power
    // length-1-i), with Forney's formula for each error's value:
    // e = X * omega(1/X) / lambda'(1/X), X = 2^power (first root 2^0)
    uint32_t where[RS_MAX_FIXED];
    uint8_t value[RS_MAX_FIXED];
    uint32_t found = 0;
    for (uint32_t i = 0; i < length; i++) {
        const uint32_t power = length - 1u - i;
        const uint32_t inv = (255u - power % 255u) % 255u; // log of 1/X
        uint8_t l = 0, dl = 0, o = 0;
        for (uint32_t k = 0; k <= errors; k++) {
            const uint8_t t = gf_mul(lambda[k], gf_pow2(inv * k));
            l ^= t;
            if (k & 1u) dl ^= gf_mul(lambda[k], gf_pow2(inv * (k - 1u))); // Odd terms make the derivative
        }
        if (l != 0) continue;
        if (found >= errors || dl == 0) return -1;
        for (uint32_t k = 0; k < RS_PARITY; k++) o ^= gf_mul(omega[k], gf_pow2(inv * k));
        where[found] = i;
        value[found] = gf_mul(gf_pow2(power), gf_div(o, dl));
        found++;
    }
    if (found != errors) return -1; // Roots outside the codeword: too many errors

    for (uint32_t k = 0; k < found; k++) codeword[where[k]] ^= value[k];
    if (!syndromes(codeword, length, s)) {
        for (uint32_t k = 0; k < found; k++) codeword[where[k]] ^= value[k];
        return -1;
    }
    return (int32_t)found;
}
