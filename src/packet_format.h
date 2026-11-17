#ifndef PACKET_FORMAT_H
#define PACKET_FORMAT_H

/**
 * @file packet_format.h
 *
 * The packet format shared by the transmitter and receiver (see
 * packet_format.md for the reasoning). One symbol per frame:
 *
 *   | A: 8 x symbol 0 | B: 8 known symbols | header | payload | CRC-16 |
 *
 *   A       detection: a held symbol, easy to spot and lock onto
 *   B       training and alignment: learn the channel's shape, and mark
 *           where the data starts (it replaces a sync word)
 *   header  payload length (1 byte) and a CRC-8 of it (1 byte)
 *   payload 0..255 bytes, scrambled (XOR with an LFSR sequence) so that
 *           repetitive data looks like random data
 *   CRC-16  CRC-16/CCITT-FALSE of the header and the unscrambled payload
 *
 * The header, payload and CRC bytes are sent most significant bit first,
 * bits_per_symbol bits per symbol; the last symbol is padded with zeros.
 *
 * B is defined on a 16-symbol grid and scaled to the symbol size, so it is
 * the same sound at any spacing (4, 5 or 6 bits per symbol). It was chosen
 * by search: no two neighbours within 5 of 16 of each other, all neighbour
 * differences distinct, and no misplaced copy (against itself or the zeros
 * of A before it) matches any of them.
 *
 * Up-chirps only; no down-chirp sync.
 */

#include <stdint.h>

#define PACKET_A_LEN 8u
#define PACKET_B_LEN 8u
#define PACKET_PREAMBLE_LEN (PACKET_A_LEN + PACKET_B_LEN)
#define PACKET_MAX_PAYLOAD 255u
#define PACKET_HEADER_BYTES 2u
#define PACKET_CRC_BYTES 2u
#define PACKET_MAX_BYTES (PACKET_HEADER_BYTES + PACKET_MAX_PAYLOAD + PACKET_CRC_BYTES)
#define PACKET_MIN_BITS_PER_SYMBOL 4u
#define PACKET_MAX_BITS_PER_SYMBOL 6u

// B on the 16-symbol grid
extern const uint8_t packet_b_grid16[PACKET_B_LEN];

// B symbol i (0 .. PACKET_B_LEN - 1) at bits_per_symbol (4 .. 6)
uint32_t packet_b_symbol(uint32_t i, uint32_t bits_per_symbol);

// CRC-8 (polynomial 0x07, initial value 0; "123456789" gives 0xF4)
uint8_t packet_crc8(const uint8_t *data, uint32_t length);

// CRC-16/CCITT-FALSE (polynomial 0x1021, initial value 0xFFFF; "123456789" gives 0x29B1)
uint16_t packet_crc16(const uint8_t *data, uint32_t length);

// XORs data with the scrambling sequence (the 8-bit symbols of the
// symbol_sequence LFSR from its seed); applying it twice restores the data
void packet_scramble(uint8_t *data, uint32_t length);

// Gray coding of data symbols: a data value v (bits_per_symbol bits) is sent
// as the symbol whose Gray code is v, so neighbouring symbols (including the
// wrap from the last to the first) carry values one bit apart, and a slip to
// a neighbouring symbol, the commonest error, damages only one bit.
static inline uint32_t packet_gray_value(uint32_t symbol) { return symbol ^ (symbol >> 1); } // Symbol -> value
uint32_t packet_gray_symbol(uint32_t value);                                                   // Value -> symbol

// Symbols in a whole packet (preamble included) with this payload length
uint32_t packet_symbol_count(uint32_t payload_length, uint32_t bits_per_symbol);

#endif // PACKET_FORMAT_H
