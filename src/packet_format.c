#include "packet_format.h"
#include "symbol_sequence.h"
#include "pico/stdlib.h"

const uint8_t packet_b_grid16[PACKET_B_LEN] = { 12, 6, 15, 6, 1, 7, 15, 4 };

uint32_t packet_b_symbol(uint32_t i, uint32_t bits_per_symbol) {
    return (uint32_t)packet_b_grid16[i % PACKET_B_LEN] << (bits_per_symbol - 4u);
}

uint16_t packet_crc16(const uint8_t *data, uint32_t length) {
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void packet_scramble(uint8_t *data, uint32_t length) {
    SymbolSequence_t s;
    symbol_sequence_init(&s, 8);
    for (uint32_t i = 0; i < length; i++) {
        data[i] ^= (uint8_t)symbol_sequence_next(&s);
    }
}

uint32_t packet_gray_symbol(uint32_t value) {
    uint32_t symbol = value;
    for (uint32_t shift = value >> 1; shift != 0; shift >>= 1) symbol ^= shift;
    return symbol;
}

uint32_t packet_coded_bytes(uint32_t payload_length, uint32_t ecc) {
    return payload_length + PACKET_CRC_BYTES + (ecc == PACKET_ECC_RS2 ? RS_PARITY : 0u);
}

uint32_t packet_payload_symbol_count(uint32_t payload_length, uint32_t ecc, uint32_t bits_per_symbol) {
    const uint32_t bits = 8u * packet_coded_bytes(payload_length, ecc);
    return (bits + bits_per_symbol - 1u) / bits_per_symbol;
}

uint32_t packet_symbol_count(uint32_t payload_length, uint32_t ecc, uint32_t bits_per_symbol) {
    return PACKET_PREAMBLE_LEN + PACKET_HEADER_SYMBOLS + packet_payload_symbol_count(payload_length, ecc, bits_per_symbol);
}

uint32_t packet_header_pack(const PacketHeader_t *h) {
    const uint32_t code = h->spacing == 1u ? 0u : (h->spacing == 2u ? 1u : 2u);
    return ((h->version & 3u) << 10) | (code << 8) | ((h->ecc & 3u) << 6) | (h->length & 63u);
}

bool packet_header_unpack(uint32_t bits, PacketHeader_t *h) {
    static const uint8_t SPACINGS[4] = { 1, 2, 4, 0 };
    h->version = (bits >> 10) & 3u;
    h->spacing = SPACINGS[(bits >> 8) & 3u];
    h->ecc = (bits >> 6) & 3u;
    h->length = bits & 63u;
    return h->version == PACKET_VERSION && h->spacing != 0u && h->ecc <= PACKET_ECC_RS2;
}

// The (23,12) Golay code's generator, x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1
#define GOLAY_GENERATOR 0xC75u

uint32_t __not_in_flash_func(packet_golay_encode)(uint32_t bits) {
    // Systematic (23,12): the 11 check bits are bits * x^11 mod g(x)
    uint32_t r = (bits & 0xFFFu) << 11;
    for (int k = 22; k >= 11; k--) {
        if (r & (1u << k)) r ^= GOLAY_GENERATOR << (k - 11);
    }
    const uint32_t word = ((bits & 0xFFFu) << 11) | r;
    // Extended: one more bit makes the weight even
    return (word << 1) | ((uint32_t)__builtin_popcount(word) & 1u);
}

bool __not_in_flash_func(packet_golay_decode)(uint32_t received, uint32_t *bits, uint32_t *fixed) {
    // Nearest of the 4096 codewords. Only once per packet, so simply all of
    // them, in Gray code order: each step flips one message bit, so the next
    // codeword is the last one XOR that bit's row (the code is linear)
    uint32_t row[12];
    for (uint32_t i = 0; i < 12u; i++) row[i] = packet_golay_encode(1u << i);
    received &= 0xFFFFFFu;
    uint32_t word = 0, m = 0;
    uint32_t best = 0, best_distance = (uint32_t)__builtin_popcount(received);
    for (uint32_t k = 1; k < 4096u && best_distance > 0u; k++) {
        const uint32_t bit = (uint32_t)__builtin_ctz(k); // Gray code k-1 -> k flips this bit
        word ^= row[bit];
        m ^= 1u << bit;
        const uint32_t d = (uint32_t)__builtin_popcount(word ^ received);
        if (d < best_distance) {
            best_distance = d;
            best = m;
        }
    }
    *bits = best;
    *fixed = best_distance;
    return best_distance <= PACKET_HEADER_MAX_FIXED;
}
