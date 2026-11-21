/**
 * @file test_packet.c
 *
 * Tests for the packet format and transmitter (packet_format.c,
 * packet_tx.c, reed_solomon.c): CRC check values, scrambling, the B
 * sequence, the header and its Golay code, the payload's Reed-Solomon code,
 * and framing a payload into symbols and back.
 */

#include <stdio.h>
#include <string.h>
#include "packet_format.h"
#include "packet_tx.h"
#include "reed_solomon.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

static void test_crc_check_values(void) {
    const uint8_t s[] = "123456789";
    CHECK_EQ_U(packet_crc16(s, 9), 0x29B1);
}

static void test_scramble(void) {
    uint8_t zeros[16] = { 0 }, data[16];
    memcpy(data, zeros, sizeof data);
    packet_scramble(data, sizeof data);
    uint32_t nonzero = 0;
    for (unsigned i = 0; i < sizeof data; i++) nonzero += data[i] != 0;
    printf("  16 zero bytes scramble to %u non-zero bytes\n", nonzero);
    CHECK(nonzero >= 14);
    packet_scramble(data, sizeof data);
    CHECK(memcmp(data, zeros, sizeof data) == 0);
}

// B sits on the same FFT bins at every spacing (symbol x bins per symbol),
// and keeps its neighbours well apart
static void test_b_sequence(void) {
    for (uint32_t i = 0; i < PACKET_B_LEN; i++) {
        const uint32_t bin6 = packet_b_symbol(i, 6) * 1u;
        const uint32_t bin5 = packet_b_symbol(i, 5) * 2u;
        const uint32_t bin4 = packet_b_symbol(i, 4) * 4u;
        CHECK_EQ_U(bin5, bin6);
        CHECK_EQ_U(bin4, bin6);
        if (i + 1 < PACKET_B_LEN) {
            const uint32_t d = (packet_b_grid16[i + 1] - packet_b_grid16[i] + 16u) % 16u;
            CHECK(d >= 5 && d <= 11);
        }
    }
    CHECK(packet_b_grid16[0] != 0);
}

// Gray coding: a round trip for every value, and neighbouring symbols
// (including the wrap round) always carry values exactly one bit apart
static void test_gray(void) {
    for (uint32_t b = 4; b <= 6; b++) {
        const uint32_t m = 1u << b;
        uint32_t bad = 0;
        for (uint32_t v = 0; v < m; v++) {
            const uint32_t s = packet_gray_symbol(v);
            bad += s >= m || packet_gray_value(s) != v;
            const uint32_t diff = packet_gray_value(s) ^ packet_gray_value((s + 1) % m);
            bad += __builtin_popcount(diff) != 1;
        }
        CHECK_EQ_U(bad, 0);
    }
}

// Independent unpacker: symbols (Gray coded, MSB first, b bits each) back to bytes
static void unpack(const uint32_t *symbols, uint32_t count, uint32_t b, uint8_t *bytes, uint32_t nbytes) {
    memset(bytes, 0, nbytes);
    for (uint32_t bit = 0; bit < count * b && bit / 8 < nbytes; bit++) {
        const uint32_t s = symbols[bit / b];
        const uint32_t v = ((s ^ (s >> 1)) >> (b - 1 - bit % b)) & 1u;
        bytes[bit / 8] |= (uint8_t)(v << (7 - bit % 8));
    }
}

// The Golay code: every two codewords differ in at least 8 bits, so any 3
// wrong bits are corrected and any 4 detected (never "corrected" wrongly)
static void test_golay(void) {
    // The code is linear: the minimum distance is the lowest nonzero weight
    uint32_t min_weight = 24, weights[25] = { 0 };
    for (uint32_t m = 0; m < 4096u; m++) {
        const uint32_t c = packet_golay_encode(m);
        CHECK_EQ_U(c >> 12, m); // Systematic: the 12 bits first
        const uint32_t w = (uint32_t)__builtin_popcount(c);
        weights[w]++;
        if (m && w < min_weight) min_weight = w;
    }
    printf("  minimum distance %u; weights 0:%u 8:%u 12:%u 16:%u 24:%u\n", min_weight,
        weights[0], weights[8], weights[12], weights[16], weights[24]);
    CHECK_EQ_U(min_weight, 8);
    CHECK_EQ_U(weights[8], 759); // The extended Golay code's weight distribution
    CHECK_EQ_U(weights[12], 2576);

    // Every pattern of up to 3 wrong bits, on a few headers
    static const uint32_t messages[] = { 0x000, 0xFFF, 0x010, 0x2A5, 0x93C };
    uint32_t bad = 0, patterns = 0;
    for (unsigned k = 0; k < sizeof messages / sizeof messages[0]; k++) {
        const uint32_t c = packet_golay_encode(messages[k]);
        for (uint32_t a = 0; a < 24; a++) for (uint32_t b = a; b < 24; b++) for (uint32_t d = b; d < 24; d++) {
            const uint32_t e = (1u << a) | (1u << b) | (1u << d); // 1, 2 or 3 bits
            uint32_t m, fixed;
            const bool ok = packet_golay_decode(c ^ e, &m, &fixed);
            bad += !ok || m != messages[k] || fixed != (uint32_t)__builtin_popcount(e);
            patterns++;
        }
        uint32_t m, fixed;
        bad += !packet_golay_decode(c, &m, &fixed) || m != messages[k] || fixed != 0;
    }
    printf("  %u patterns of 1-3 wrong bits: %u not corrected\n", patterns, bad);
    CHECK_EQ_U(bad, 0);

    // Every pattern of 4 wrong bits on one header: always detected
    const uint32_t c = packet_golay_encode(0x2A5);
    uint32_t wrong = 0;
    patterns = 0;
    for (uint32_t a = 0; a < 24; a++) for (uint32_t b = a + 1; b < 24; b++)
        for (uint32_t d = b + 1; d < 24; d++) for (uint32_t f = d + 1; f < 24; f++) {
            uint32_t m, fixed;
            wrong += packet_golay_decode(c ^ (1u << a) ^ (1u << b) ^ (1u << d) ^ (1u << f), &m, &fixed);
            patterns++;
        }
    printf("  %u patterns of 4 wrong bits: %u accepted\n", patterns, wrong);
    CHECK_EQ_U(wrong, 0);
}

static void test_header(void) {
    static const uint32_t spacings[] = { 1, 2, 4 };
    for (unsigned k = 0; k < 3; k++) {
        for (uint32_t len = 0; len <= PACKET_MAX_PAYLOAD; len += 21) {
            const PacketHeader_t h = { PACKET_VERSION, spacings[k], PACKET_ECC_NONE, len };
            const uint32_t bits = packet_header_pack(&h);
            CHECK(bits < 4096u);
            PacketHeader_t u;
            CHECK(packet_header_unpack(bits, &u));
            CHECK(u.version == h.version && u.spacing == h.spacing && u.ecc == h.ecc && u.length == h.length);
        }
    }
    const PacketHeader_t h = { PACKET_VERSION, 4, PACKET_ECC_NONE, 16 };
    CHECK_EQ_U(packet_header_pack(&h), 0x210); // version 0 | spacing code 2 | ecc 0 | length 16
    PacketHeader_t u;
    CHECK(!packet_header_unpack(0x310, &u));  // Reserved spacing code
    CHECK(packet_header_unpack(0x250, &u));   // Error correction 1: rs2
    CHECK_EQ_U(u.ecc, PACKET_ECC_RS2);
    CHECK(!packet_header_unpack(0x290, &u));  // Error correction 2 (reserved)
    CHECK(!packet_header_unpack(0xA10, &u));  // Version 2
}

static void test_framing(void) {
    static PacketTx_t tx;
    static uint32_t symbols[400];
    const uint8_t payload[] = "Hello, chirp!";
    const uint32_t len = sizeof payload - 1;
    for (uint32_t step = 0; step < 6; step++) {
        const uint32_t b = 4u + step % 3u, ecc = step / 3u;
        packet_tx_init(&tx, b);
        CHECK(packet_tx_start(&tx, payload, len, ecc));
        CHECK(!packet_tx_start(&tx, payload, len, ecc)); // Busy
        uint32_t n = 0, s;
        while (packet_tx_next_symbol(&tx, &s) && n < 400) symbols[n++] = s;
        CHECK(!packet_tx_busy(&tx));
        CHECK_EQ_U(n, packet_symbol_count(len, ecc, b));
        printf("  %u bits/symbol, %s: %u-byte payload -> %u symbols\n", b, ecc ? "rs2" : "no ecc", len, n);

        for (uint32_t i = 0; i < PACKET_A_LEN; i++) CHECK_EQ_U(symbols[i], 0);
        for (uint32_t i = 0; i < PACKET_B_LEN; i++) CHECK_EQ_U(symbols[PACKET_A_LEN + i], packet_b_symbol(i, b));
        for (uint32_t i = 0; i < n; i++) CHECK(symbols[i] < (1u << b));

        // The header: 8 symbols on the spacing-8 grid, 3 bits each
        uint32_t code = 0;
        for (uint32_t i = 0; i < PACKET_HEADER_SYMBOLS; i++) {
            const uint32_t s8 = symbols[PACKET_PREAMBLE_LEN + i] >> (b - 3u);
            CHECK_EQ_U(symbols[PACKET_PREAMBLE_LEN + i], s8 << (b - 3u));
            code = (code << 3) | (s8 ^ (s8 >> 1));
        }
        uint32_t bits, fixed;
        PacketHeader_t h;
        CHECK(packet_golay_decode(code, &bits, &fixed));
        CHECK_EQ_U(fixed, 0);
        CHECK(packet_header_unpack(bits, &h));
        CHECK_EQ_U(h.spacing, 64u >> b);
        CHECK_EQ_U(h.ecc, ecc);
        CHECK_EQ_U(h.length, len);

        // The payload, CRC and any check bytes at the payload's spacing
        const uint32_t first = PACKET_PREAMBLE_LEN + PACKET_HEADER_SYMBOLS;
        const uint32_t coded = packet_coded_bytes(len, ecc);
        CHECK_EQ_U(coded, len + PACKET_CRC_BYTES + (ecc ? RS_PARITY : 0u));
        uint8_t bytes[2 + PACKET_MAX_BYTES];
        bytes[0] = (uint8_t)(bits >> 8);
        bytes[1] = (uint8_t)bits;
        unpack(&symbols[first], n - first, b, &bytes[2], coded);
        if (ecc) {
            uint8_t parity[RS_PARITY];
            reed_solomon_encode(&bytes[2], len + PACKET_CRC_BYTES, parity);
            CHECK(memcmp(parity, &bytes[2 + len + PACKET_CRC_BYTES], RS_PARITY) == 0);
            CHECK_EQ_U(reed_solomon_decode(&bytes[2], coded), 0);
        }
        packet_scramble(&bytes[2], len);
        CHECK(memcmp(&bytes[2], payload, len) == 0);
        const uint16_t crc = packet_crc16(bytes, 2 + len);
        CHECK_EQ_U(bytes[2 + len], crc >> 8);
        CHECK_EQ_U(bytes[3 + len], crc & 0xFF);
    }
    // Too long
    packet_tx_init(&tx, 6);
    static uint8_t big[PACKET_MAX_PAYLOAD + 1];
    CHECK(!packet_tx_start(&tx, big, sizeof big, PACKET_ECC_RS2));
    CHECK(!packet_tx_start(&tx, big, 4, 2)); // Reserved error correction
    CHECK(packet_tx_start(&tx, big, PACKET_MAX_PAYLOAD, PACKET_ECC_RS2));
}

// The symbols sent for "Hi" with rs2, as tools/chirp_soak.py rebuilds them
// (test_client.py pins the same lists; header symbols at spacing 8)
static void test_hi_symbols(void) {
    static const uint32_t want4[] = { 1, 1, 0, 3, 6, 5, 1, 5, 8, 10, 7, 12, 4, 2, 15, 10, 1, 8, 0, 2, 2, 12, 7, 8 };
    static const uint32_t want5[] = { 0, 6, 0, 3, 1, 7, 3, 2, 17, 22, 6, 10, 31, 24, 22, 10, 29, 0, 22, 28, 7 };
    static PacketTx_t tx;
    for (uint32_t b = 4; b <= 5; b++) {
        const uint32_t *want = b == 4 ? want4 : want5;
        const uint32_t count = b == 4 ? sizeof want4 / sizeof want4[0] : sizeof want5 / sizeof want5[0];
        packet_tx_init(&tx, b);
        CHECK(packet_tx_start(&tx, (const uint8_t *)"Hi", 2, PACKET_ECC_RS2));
        uint32_t i = 0, s, bad = 0;
        while (packet_tx_next_symbol(&tx, &s)) {
            if (i >= PACKET_PREAMBLE_LEN) {
                const uint32_t j = i - PACKET_PREAMBLE_LEN;
                const uint32_t got = j < PACKET_HEADER_SYMBOLS ? s >> (b - PACKET_HEADER_BITS_PER_SYMBOL) : s;
                bad += j >= count || got != want[j];
            }
            i++;
        }
        CHECK_EQ_U(i, PACKET_PREAMBLE_LEN + count);
        CHECK_EQ_U(bad, 0);
    }
}

// Reed-Solomon (4 check bytes): check bytes from an independent
// implementation, then every way of getting 1 or 2 bytes wrong in a short
// codeword is corrected, random ones in the longest, and 3 wrong bytes are
// never passed off as clean (but are "corrected" into another codeword about
// as often as a random word lies within 2 bytes of one: 0.4% at 22 bytes,
// 3.6% at 69; the packet's CRC-16 then catches all but 1 in 65536)
static uint32_t lcg_state = 12345u;
static uint32_t lcg(void) {
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return lcg_state >> 8;
}

static void test_reed_solomon(void) {
    reed_solomon_init();
    // tools/chirp_soak.py rs_parity, which gives the QR code's published
    // check bytes with 10 of them (test_client.py)
    uint8_t parity[RS_PARITY];
    reed_solomon_encode((const uint8_t *)"Hello, chirp!", 13, parity);
    CHECK(parity[0] == 0xfe && parity[1] == 0x46 && parity[2] == 0xec && parity[3] == 0x5b);
    uint8_t ramp[20];
    for (unsigned i = 0; i < sizeof ramp; i++) ramp[i] = (uint8_t)i;
    reed_solomon_encode(ramp, sizeof ramp, parity);
    CHECK(parity[0] == 0xfa && parity[1] == 0x1d && parity[2] == 0x5b && parity[3] == 0xbc);

    // A 16-byte payload's codeword: 22 bytes
    enum { SHORT = 22 };
    uint8_t clean[PACKET_MAX_BYTES], word[PACKET_MAX_BYTES];
    for (unsigned i = 0; i < SHORT - RS_PARITY; i++) clean[i] = (uint8_t)lcg();
    reed_solomon_encode(clean, SHORT - RS_PARITY, &clean[SHORT - RS_PARITY]);
    memcpy(word, clean, SHORT);
    CHECK_EQ_U(reed_solomon_decode(word, SHORT), 0);

    uint32_t tried = 0, bad = 0;
    for (unsigned i = 0; i < SHORT; i++) {
        for (unsigned v = 1; v < 256; v++) {
            memcpy(word, clean, SHORT);
            word[i] ^= (uint8_t)v;
            bad += reed_solomon_decode(word, SHORT) != 1 || memcmp(word, clean, SHORT) != 0;
            tried++;
        }
    }
    for (unsigned i = 0; i < SHORT; i++) {
        for (unsigned j = i + 1; j < SHORT; j++) {
            for (unsigned k = 0; k < 16; k++) {
                memcpy(word, clean, SHORT);
                word[i] ^= (uint8_t)(1u + lcg() % 255u);
                word[j] ^= (uint8_t)(1u + lcg() % 255u);
                bad += reed_solomon_decode(word, SHORT) != 2 || memcmp(word, clean, SHORT) != 0;
                tried++;
            }
        }
    }
    printf("  22 bytes: %u patterns of 1-2 wrong bytes, %u not corrected\n", tried, bad);
    CHECK_EQ_U(bad, 0);

    // The longest packet's codeword: 69 bytes
    for (unsigned i = 0; i < PACKET_MAX_BYTES - RS_PARITY; i++) clean[i] = (uint8_t)lcg();
    reed_solomon_encode(clean, PACKET_MAX_BYTES - RS_PARITY, &clean[PACKET_MAX_BYTES - RS_PARITY]);
    tried = bad = 0;
    for (unsigned t = 0; t < 20000; t++) {
        memcpy(word, clean, PACKET_MAX_BYTES);
        const unsigned i = lcg() % PACKET_MAX_BYTES;
        unsigned j = lcg() % PACKET_MAX_BYTES;
        if (j == i) j = (j + 1u) % PACKET_MAX_BYTES;
        word[i] ^= (uint8_t)(1u + lcg() % 255u);
        word[j] ^= (uint8_t)(1u + lcg() % 255u);
        bad += reed_solomon_decode(word, PACKET_MAX_BYTES) != 2 || memcmp(word, clean, PACKET_MAX_BYTES) != 0;
        tried++;
    }
    printf("  69 bytes: %u random pairs of wrong bytes, %u not corrected\n", tried, bad);
    CHECK_EQ_U(bad, 0);

    // 3 wrong bytes: refused, or (rarely) "corrected" to another codeword
    // (the packet's CRC catches that); never the original, never clean
    for (unsigned n = SHORT; n <= PACKET_MAX_BYTES; n += PACKET_MAX_BYTES - SHORT) {
        for (unsigned i = 0; i < n - RS_PARITY; i++) clean[i] = (uint8_t)lcg();
        reed_solomon_encode(clean, n - RS_PARITY, &clean[n - RS_PARITY]);
        uint32_t refused = 0, miscorrected = 0, wrong = 0;
        for (unsigned t = 0; t < 20000; t++) {
            memcpy(word, clean, n);
            unsigned pos[3];
            for (unsigned c = 0; c < 3; c++) {
                bool again;
                do {
                    pos[c] = lcg() % n;
                    again = false;
                    for (unsigned d = 0; d < c; d++) again |= pos[d] == pos[c];
                } while (again);
                word[pos[c]] ^= (uint8_t)(1u + lcg() % 255u);
            }
            uint8_t before[PACKET_MAX_BYTES];
            memcpy(before, word, n);
            const int32_t r = reed_solomon_decode(word, n);
            if (r < 0) {
                refused++;
                wrong += memcmp(word, before, n) != 0; // Must be left as it was
            } else {
                miscorrected++;
                wrong += r == 0 || memcmp(word, clean, n) == 0;
            }
        }
        // A random word is within 2 bytes of some codeword with chance
        // (words within 2 bytes of one codeword) / 256^4
        const double ball = 1.0 + 255.0 * n + 255.0 * 255.0 * n * (n - 1u) / 2.0;
        const double expected = ball / 4294967296.0;
        printf("  %u bytes, 3 wrong: %u refused, %u corrected to another codeword (%.2f%%; random words %.2f%%)\n",
            n, refused, miscorrected, 100.0 * miscorrected / 20000.0, 100.0 * expected);
        CHECK_EQ_U(wrong, 0);
        CHECK(miscorrected < 1.25 * expected * 20000.0);
    }
    CHECK_EQ_U(reed_solomon_decode(word, RS_PARITY), -1); // Too short to hold data
}

// Packet lengths quoted in packet_format.md
static void test_airtime(void) {
    CHECK_EQ_U(packet_symbol_count(0, PACKET_ECC_NONE, 4), 28);
    CHECK_EQ_U(packet_symbol_count(16, PACKET_ECC_NONE, 4), 60);
    CHECK_EQ_U(packet_symbol_count(63, PACKET_ECC_NONE, 4), 154);
    CHECK_EQ_U(packet_symbol_count(16, PACKET_ECC_NONE, 6), 48);
    CHECK_EQ_U(packet_symbol_count(63, PACKET_ECC_NONE, 6), 111);
    CHECK_EQ_U(packet_symbol_count(0, PACKET_ECC_RS2, 4), 36);
    CHECK_EQ_U(packet_symbol_count(16, PACKET_ECC_RS2, 4), 68);
    CHECK_EQ_U(packet_symbol_count(16, PACKET_ECC_RS2, 5), 60);
    CHECK_EQ_U(packet_symbol_count(63, PACKET_ECC_RS2, 4), 162);
    CHECK_EQ_U(packet_symbol_count(16, PACKET_ECC_RS2, 6), 54);
    CHECK_EQ_U(packet_symbol_count(63, PACKET_ECC_RS2, 6), 116);
}

int main(void) {
    RUN_TEST(test_crc_check_values);
    RUN_TEST(test_scramble);
    RUN_TEST(test_b_sequence);
    RUN_TEST(test_gray);
    RUN_TEST(test_golay);
    RUN_TEST(test_header);
    RUN_TEST(test_framing);
    RUN_TEST(test_hi_symbols);
    RUN_TEST(test_reed_solomon);
    RUN_TEST(test_airtime);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
