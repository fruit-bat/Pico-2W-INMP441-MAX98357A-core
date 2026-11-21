#ifndef PACKET_TX_H
#define PACKET_TX_H

/**
 * @file packet_tx.h
 *
 * Packet transmitter: turns a payload into the symbol stream of
 * packet_format.h, one symbol per call (one per frame). The packet is
 * framed when it is started (header codeword, scrambled payload and CRC kept
 * in the transmitter's own storage); symbols are then produced on the fly.
 *
 * Symbols are numbered at the transmitter's spacing (bits_per_symbol), as
 * the chirp encoder takes them; the header's spacing-8 symbols come out
 * multiplied up to that (as B's do).
 *
 * No memory is allocated.
 */

#include <stdbool.h>
#include <stdint.h>
#include "packet_format.h"

typedef struct {
    uint32_t bits_per_symbol;           // 4 .. 6 (the payload's, and the symbol numbering)
    uint32_t header_code;               // The header's 24-bit Golay codeword
    uint8_t bytes[PACKET_MAX_BYTES];    // Scrambled payload, CRC-16, any check bytes
    uint32_t byte_count;
    uint32_t symbol_count;              // Whole packet, preamble included
    uint32_t next;                      // Index of the next symbol to send
    bool busy;
} PacketTx_t;

void packet_tx_init(PacketTx_t *tx, uint32_t bits_per_symbol);

// Frames a packet and starts sending it, its payload at this transmitter's
// spacing with error correction ecc (PACKET_ECC_NONE or PACKET_ECC_RS2).
// Returns false (and does nothing) while a packet is still being sent, if
// length > PACKET_MAX_PAYLOAD, or if ecc is not one of those.
bool packet_tx_start(PacketTx_t *tx, const uint8_t *payload, uint32_t length, uint32_t ecc);

// The next symbol to send. Returns false when there is nothing to send
// (idle, or the last packet has finished), in which case the frame should
// be silent.
bool packet_tx_next_symbol(PacketTx_t *tx, uint32_t *symbol);

static inline bool packet_tx_busy(const PacketTx_t *tx) { return tx->busy; }

#endif // PACKET_TX_H
