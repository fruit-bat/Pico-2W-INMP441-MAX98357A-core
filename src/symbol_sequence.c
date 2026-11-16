#include "symbol_sequence.h"
#include "pico/stdlib.h"

void symbol_sequence_init(SymbolSequence_t *s, uint32_t bits_per_symbol) {
    s->lfsr = SYMBOL_SEQUENCE_SEED;
    s->bits_per_symbol = bits_per_symbol;
}

uint32_t __not_in_flash_func(symbol_sequence_next)(SymbolSequence_t *s) {
    uint32_t symbol = 0;
    for (uint32_t i = 0; i < s->bits_per_symbol; i++) {
        const uint32_t out = s->lfsr & 1u;
        s->lfsr >>= 1;
        if (out) s->lfsr ^= SYMBOL_SEQUENCE_MASK;
        symbol = (symbol << 1) | out;
    }
    return symbol;
}
