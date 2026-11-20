#include <string.h>
#include "packet_tx.h"
#include "pico/stdlib.h"

void packet_tx_init(PacketTx_t *tx, uint32_t bits_per_symbol) {
    memset(tx, 0, sizeof(*tx));
    tx->bits_per_symbol = bits_per_symbol;
}

bool packet_tx_start(PacketTx_t *tx, const uint8_t *payload, uint32_t length) {
    if (tx->busy || length > PACKET_MAX_PAYLOAD) return false;

    const PacketHeader_t h = {
        .version = PACKET_VERSION,
        .spacing = 64u >> tx->bits_per_symbol,
        .ecc = PACKET_ECC_NONE,
        .length = length,
    };
    const uint32_t header = packet_header_pack(&h);
    tx->header_code = packet_golay_encode(header);

    // CRC over the header (2 bytes) and the payload as given, then scramble the payload
    uint8_t *b = tx->bytes;
    b[0] = (uint8_t)(header >> 8);
    b[1] = (uint8_t)header;
    memcpy(&b[2], payload, length);
    const uint16_t crc = packet_crc16(b, 2u + length);
    memmove(b, &b[2], length);
    packet_scramble(b, length);
    b[length] = (uint8_t)(crc >> 8);
    b[length + 1u] = (uint8_t)crc;

    tx->byte_count = length + PACKET_CRC_BYTES;
    tx->symbol_count = packet_symbol_count(length, tx->bits_per_symbol);
    tx->next = 0;
    tx->busy = true;
    return true;
}

// Data symbol j: bits j*b .. j*b + b - 1 of the byte stream, MSB first,
// zero beyond the end, Gray coded
static uint32_t __not_in_flash_func(data_symbol)(const PacketTx_t *tx, uint32_t j) {
    const uint32_t nbits = tx->bits_per_symbol;
    uint32_t symbol = 0;
    for (uint32_t k = 0; k < nbits; k++) {
        const uint32_t bit = j * nbits + k;
        const uint32_t byte = bit / 8u;
        const uint32_t value = byte < tx->byte_count ? (tx->bytes[byte] >> (7u - bit % 8u)) & 1u : 0u;
        symbol = (symbol << 1) | value;
    }
    return packet_gray_symbol(symbol);
}

bool __not_in_flash_func(packet_tx_next_symbol)(PacketTx_t *tx, uint32_t *symbol) {
    if (!tx->busy) return false;
    const uint32_t i = tx->next++;
    if (i < PACKET_A_LEN) {
        *symbol = 0;
    } else if (i < PACKET_PREAMBLE_LEN) {
        *symbol = packet_b_symbol(i - PACKET_A_LEN, tx->bits_per_symbol);
    } else if (i < PACKET_PREAMBLE_LEN + PACKET_HEADER_SYMBOLS) {
        // Header symbol j: 3 codeword bits, MSB first, Gray coded, at spacing 8
        const uint32_t j = i - PACKET_PREAMBLE_LEN;
        const uint32_t shift = PACKET_HEADER_CODE_BITS - PACKET_HEADER_BITS_PER_SYMBOL * (j + 1u);
        const uint32_t value = (tx->header_code >> shift) & ((1u << PACKET_HEADER_BITS_PER_SYMBOL) - 1u);
        *symbol = packet_gray_symbol(value) << (tx->bits_per_symbol - PACKET_HEADER_BITS_PER_SYMBOL);
    } else {
        *symbol = data_symbol(tx, i - PACKET_PREAMBLE_LEN - PACKET_HEADER_SYMBOLS);
    }
    if (tx->next >= tx->symbol_count) tx->busy = false;
    return true;
}
