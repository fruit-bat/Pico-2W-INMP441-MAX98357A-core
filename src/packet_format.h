#ifndef PACKET_FORMAT_H
#define PACKET_FORMAT_H

/**
 * @file packet_format.h
 *
 * The packet format shared by the transmitter and receiver (see
 * packet_format.md for the reasoning). One symbol per frame:
 *
 *   | A: 8 x symbol 0 | B: 8 known symbols | header: 8 symbols | payload | CRC-16 | RS check |
 *
 *   A       detection: a held symbol, easy to spot and lock onto
 *   B       training and alignment: learn the channel's shape, and mark
 *           where the data starts (it replaces a sync word)
 *   header  12 bits (version, payload spacing, error correction, payload
 *           length) in an extended Golay (24,12) code, which corrects any 3
 *           wrong bits; always sent at spacing 8 (3 bits per symbol, the
 *           most robust), whatever the payload's spacing
 *   payload 0..63 bytes, scrambled (XOR with an LFSR sequence) so that
 *           repetitive data looks like random data, at the spacing the
 *           header gives
 *   CRC-16  CRC-16/CCITT-FALSE of the header (2 bytes, big-endian) and the
 *           unscrambled payload
 *   RS check with error correction PACKET_ECC_RS2 (the header says): 4
 *           Reed-Solomon check bytes over the scrambled payload and the CRC
 *           (reed_solomon.h), which correct any 2 wrong bytes; the CRC then
 *           catches a wrong correction
 *
 * Header, payload, CRC and check bits are sent most significant bit first, Gray
 * coded, bits_per_symbol bits per symbol; the last symbol is padded with
 * zeros. The receiver needs no spacing setting: B is the same sound at
 * every spacing, and the header (at a fixed spacing) gives the payload's.
 *
 * B is defined on a 16-symbol grid and scaled to the symbol size, so it is
 * the same sound at any spacing. It was chosen by search: no two neighbours
 * within 5 of 16 of each other, all neighbour differences distinct, and no
 * misplaced copy (against itself or the zeros of A before it) matches any
 * of them.
 *
 * Up-chirps only; no down-chirp sync.
 */

#include <stdbool.h>
#include <stdint.h>
#include "reed_solomon.h"

#define PACKET_A_LEN 8u
#define PACKET_B_LEN 8u
#define PACKET_PREAMBLE_LEN (PACKET_A_LEN + PACKET_B_LEN)
#define PACKET_MAX_PAYLOAD 63u
#define PACKET_CRC_BYTES 2u
#define PACKET_MAX_BYTES (PACKET_MAX_PAYLOAD + PACKET_CRC_BYTES + RS_PARITY) // Payload, CRC, check bytes (the header is separate)
#define PACKET_MIN_BITS_PER_SYMBOL 4u
#define PACKET_MAX_BITS_PER_SYMBOL 6u

// The header: 12 bits, sent as a 24-bit Golay codeword in 8 symbols of 3 bits
#define PACKET_HEADER_BITS 12u
#define PACKET_HEADER_CODE_BITS 24u
#define PACKET_HEADER_BITS_PER_SYMBOL 3u
#define PACKET_HEADER_SPACING 8u // Bins between header symbols
#define PACKET_HEADER_SYMBOLS (PACKET_HEADER_CODE_BITS / PACKET_HEADER_BITS_PER_SYMBOL)
#define PACKET_HEADER_MAX_FIXED 3u // Wrong bits the code corrects

#define PACKET_VERSION 0u // Header format version
#define PACKET_ECC_NONE 0u // Payload error correction: none
#define PACKET_ECC_RS2 1u  // Reed-Solomon, 4 check bytes, corrects 2 bytes (2, 3 reserved)

typedef struct {
    uint32_t version;     // 0..3: PACKET_VERSION
    uint32_t spacing;     // Payload bins per symbol: 1, 2 or 4 (code 3 is reserved: 0 here)
    uint32_t ecc;         // 0..3: PACKET_ECC_NONE or PACKET_ECC_RS2
    uint32_t length;      // Payload bytes, 0..63
} PacketHeader_t;

// The header's 12 bits: version (2) | spacing code (2: 0 = 1 bin, 1 = 2,
// 2 = 4, 3 reserved) | error correction (2) | length (6), MSB first
uint32_t packet_header_pack(const PacketHeader_t *h);
// Unpacks 12 bits; false if the format is not one this code knows (other
// version, reserved spacing code, reserved error correction)
bool packet_header_unpack(uint32_t bits, PacketHeader_t *h);

// Extended Golay (24,12): the 24-bit codeword for 12 bits (the bits, then
// 12 check bits); every two codewords differ in at least 8 bits
uint32_t packet_golay_encode(uint32_t bits);
// Decodes 24 received bits: the nearest codeword's 12 bits and how many bits
// differed. False if more than PACKET_HEADER_MAX_FIXED differ (then the
// nearest codeword is at least 4 away, so 4 wrong bits are always detected)
bool packet_golay_decode(uint32_t received, uint32_t *bits, uint32_t *fixed);

// B on the 16-symbol grid
extern const uint8_t packet_b_grid16[PACKET_B_LEN];

// B symbol i (0 .. PACKET_B_LEN - 1) at bits_per_symbol (4 .. 6)
uint32_t packet_b_symbol(uint32_t i, uint32_t bits_per_symbol);

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

// Bytes after the header (payload, CRC, any check bytes)
uint32_t packet_coded_bytes(uint32_t payload_length, uint32_t ecc);

// Symbols after the header (payload, CRC, any check bytes)
uint32_t packet_payload_symbol_count(uint32_t payload_length, uint32_t ecc, uint32_t bits_per_symbol);

// Symbols in a whole packet (preamble and header included)
uint32_t packet_symbol_count(uint32_t payload_length, uint32_t ecc, uint32_t bits_per_symbol);

#endif // PACKET_FORMAT_H
