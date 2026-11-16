/**
 * @file test_sequence.c
 *
 * Tests for the link-test symbol sequence (symbol_sequence.c). The first
 * values are pinned so that the firmware and the tools that check its logs
 * (which reimplement the LFSR) stay in step.
 */

#include <stdio.h>
#include <string.h>
#include "symbol_sequence.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

// First symbols at 6, 5 and 4 bits per symbol (64, 32, 16 symbols), from an
// independent implementation of the definition in symbol_sequence.h
static void test_reference_values(void) {
    static const uint32_t ref6[] = { 33, 50, 13, 6, 55, 11, 3, 29, 59, 47, 35, 61 };
    static const uint32_t ref5[] = { 16, 28, 17, 20, 13, 23, 5, 16, 27, 23, 23, 15 };
    static const uint32_t ref4[] = { 8, 7, 2, 3, 4, 6, 13, 12, 11, 0, 13, 13 };
    const struct { uint32_t bits; const uint32_t *ref; } cases[] = { { 6, ref6 }, { 5, ref5 }, { 4, ref4 } };
    for (unsigned c = 0; c < 3; c++) {
        SymbolSequence_t s;
        symbol_sequence_init(&s, cases[c].bits);
        for (unsigned i = 0; i < 12; i++) CHECK_EQ_U(symbol_sequence_next(&s), cases[c].ref[i]);
    }
}

// The LFSR is maximal length: its state returns to the seed after exactly
// 65535 bits, and not before
static void test_period(void) {
    SymbolSequence_t s;
    symbol_sequence_init(&s, 1);
    uint32_t period = 0;
    do {
        symbol_sequence_next(&s);
        period++;
    } while (s.lfsr != SYMBOL_SEQUENCE_SEED && period < 70000);
    printf("  period %u bits\n", period);
    CHECK_EQ_U(period, 65535);
}

// Every symbol value turns up about equally often
static void test_uniform(void) {
    enum { COUNT = 6400 };
    uint32_t hist[64];
    memset(hist, 0, sizeof hist);
    SymbolSequence_t s;
    symbol_sequence_init(&s, 6);
    for (unsigned i = 0; i < COUNT; i++) {
        const uint32_t v = symbol_sequence_next(&s);
        CHECK(v < 64);
        if (v < 64) hist[v]++;
    }
    uint32_t lo = COUNT, hi = 0;
    for (unsigned k = 0; k < 64; k++) {
        if (hist[k] < lo) lo = hist[k];
        if (hist[k] > hi) hi = hist[k];
    }
    printf("  %u symbols: each value %u .. %u times (expected 100)\n", COUNT, lo, hi);
    CHECK(lo > 60 && hi < 140);
}

int main(void) {
    RUN_TEST(test_reference_values);
    RUN_TEST(test_period);
    RUN_TEST(test_uniform);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
