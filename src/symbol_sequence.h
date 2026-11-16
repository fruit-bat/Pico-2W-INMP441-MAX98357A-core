#ifndef SYMBOL_SEQUENCE_H
#define SYMBOL_SEQUENCE_H

/**
 * @file symbol_sequence.h
 *
 * A known pseudo-random sequence of symbols, for link testing: the
 * transmitter sends one symbol per frame and the receiver's log is compared
 * against the same sequence afterwards.
 *
 * Bits come from a 16-bit maximal-length Galois LFSR (polynomial
 * x^16 + x^14 + x^13 + x^11 + 1, mask 0xB400, period 65535 bits) started
 * from SYMBOL_SEQUENCE_SEED. Each symbol takes the next bits_per_symbol
 * output bits, first bit most significant:
 *
 *   for each bit: out = lfsr & 1; lfsr >>= 1; if (out) lfsr ^= 0xB400;
 *                 symbol = (symbol << 1) | out
 *
 * (tools that check a log reproduce exactly this).
 *
 * No memory is allocated.
 */

#include <stdint.h>

#define SYMBOL_SEQUENCE_SEED 0xACE1u
#define SYMBOL_SEQUENCE_MASK 0xB400u

typedef struct {
    uint16_t lfsr;
    uint32_t bits_per_symbol;
} SymbolSequence_t;

// Starts the sequence from the beginning; symbols are bits_per_symbol bits
// (1 .. 16), i.e. 0 .. 2^bits_per_symbol - 1
void symbol_sequence_init(SymbolSequence_t *s, uint32_t bits_per_symbol);

// Returns the next symbol
uint32_t symbol_sequence_next(SymbolSequence_t *s);

#endif // SYMBOL_SEQUENCE_H
