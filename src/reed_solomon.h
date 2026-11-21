#ifndef REED_SOLOMON_H
#define REED_SOLOMON_H

/**
 * @file reed_solomon.h
 *
 * Reed-Solomon code over GF(256) with RS_PARITY check bytes: corrects up to
 * RS_PARITY / 2 wrong bytes anywhere in a codeword of up to 255 bytes
 * (shorter codewords are the full-length code with leading zeros left out).
 *
 * The usual conventions (as the Python reedsolo package's defaults, and
 * tools/chirp_soak.py): field polynomial x^8 + x^4 + x^3 + x^2 + 1 (0x11D),
 * primitive element 2, generator roots 2^0 .. 2^(RS_PARITY-1). A codeword
 * is the data bytes then the check bytes; byte 0 is the highest power.
 *
 * The packet's payload code (packet_format.h): wrong decisions are almost
 * always single symbols, so with 4..6 bits per symbol one wrong decision
 * damages at most 2 bytes, and 2 correctable bytes cover the commonest
 * failures (soak tests 2026-11-20/21).
 *
 * No memory is allocated; the field tables are built by reed_solomon_init()
 * (static, 768 bytes).
 */

#include <stdbool.h>
#include <stdint.h>

#define RS_PARITY 4u           // Check bytes
#define RS_MAX_FIXED (RS_PARITY / 2u) // Wrong bytes corrected
#define RS_MAX_CODEWORD 255u

// Builds the field tables (once; later calls do nothing)
void reed_solomon_init(void);

// The RS_PARITY check bytes for 'length' data bytes (length + RS_PARITY <= 255)
void reed_solomon_encode(const uint8_t *data, uint32_t length, uint8_t *parity);

// Corrects a received codeword of 'length' bytes (data then check bytes) in
// place. Returns the number of bytes corrected (0 .. RS_MAX_FIXED), or -1 if
// it can't be corrected (more than RS_MAX_FIXED wrong bytes, as far as the
// code can tell; then the codeword is left as it was). More wrong bytes than
// that can also be "corrected" into a different codeword: a check over the
// data (the packet's CRC) catches that.
int32_t reed_solomon_decode(uint8_t *codeword, uint32_t length);

#endif // REED_SOLOMON_H
