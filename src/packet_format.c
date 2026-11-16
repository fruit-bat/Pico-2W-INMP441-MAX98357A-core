#include "packet_format.h"
#include "symbol_sequence.h"

const uint8_t packet_b_grid16[PACKET_B_LEN] = { 12, 6, 15, 6, 1, 7, 15, 4 };

uint32_t packet_b_symbol(uint32_t i, uint32_t bits_per_symbol) {
    return (uint32_t)packet_b_grid16[i % PACKET_B_LEN] << (bits_per_symbol - 4u);
}

uint8_t packet_crc8(const uint8_t *data, uint32_t length) {
    uint8_t crc = 0;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
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

uint32_t packet_symbol_count(uint32_t payload_length, uint32_t bits_per_symbol) {
    const uint32_t bits = 8u * (PACKET_HEADER_BYTES + payload_length + PACKET_CRC_BYTES);
    return PACKET_PREAMBLE_LEN + (bits + bits_per_symbol - 1u) / bits_per_symbol;
}
