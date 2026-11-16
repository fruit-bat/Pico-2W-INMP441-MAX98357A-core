#include <string.h>
#include "packet_tx.h"
#include "pico/stdlib.h"

void packet_tx_init(PacketTx_t *tx, uint32_t bits_per_symbol) {
    memset(tx, 0, sizeof(*tx));
    tx->bits_per_symbol = bits_per_symbol;
}

bool packet_tx_start(PacketTx_t *tx, const uint8_t *payload, uint32_t length) {
    if (tx->busy || length > PACKET_MAX_PAYLOAD) return false;

    uint8_t *b = tx->bytes;
    b[0] = (uint8_t)length;
    b[1] = packet_crc8(b, 1);
    memcpy(&b[PACKET_HEADER_BYTES], payload, length);

    // CRC over the header and the payload as given, then scramble the payload
    const uint16_t crc = packet_crc16(b, PACKET_HEADER_BYTES + length);
    packet_scramble(&b[PACKET_HEADER_BYTES], length);
    b[PACKET_HEADER_BYTES + length] = (uint8_t)(crc >> 8);
    b[PACKET_HEADER_BYTES + length + 1u] = (uint8_t)crc;

    tx->byte_count = PACKET_HEADER_BYTES + length + PACKET_CRC_BYTES;
    tx->symbol_count = packet_symbol_count(length, tx->bits_per_symbol);
    tx->next = 0;
    tx->busy = true;
    return true;
}

// Data symbol j: bits j*b .. j*b + b - 1 of the byte stream, MSB first,
// zero beyond the end
static uint32_t __not_in_flash_func(data_symbol)(const PacketTx_t *tx, uint32_t j) {
    const uint32_t nbits = tx->bits_per_symbol;
    uint32_t symbol = 0;
    for (uint32_t k = 0; k < nbits; k++) {
        const uint32_t bit = j * nbits + k;
        const uint32_t byte = bit / 8u;
        const uint32_t value = byte < tx->byte_count ? (tx->bytes[byte] >> (7u - bit % 8u)) & 1u : 0u;
        symbol = (symbol << 1) | value;
    }
    return symbol;
}

bool __not_in_flash_func(packet_tx_next_symbol)(PacketTx_t *tx, uint32_t *symbol) {
    if (!tx->busy) return false;
    const uint32_t i = tx->next++;
    if (i < PACKET_A_LEN) {
        *symbol = 0;
    } else if (i < PACKET_PREAMBLE_LEN) {
        *symbol = packet_b_symbol(i - PACKET_A_LEN, tx->bits_per_symbol);
    } else {
        *symbol = data_symbol(tx, i - PACKET_PREAMBLE_LEN);
    }
    if (tx->next >= tx->symbol_count) tx->busy = false;
    return true;
}
