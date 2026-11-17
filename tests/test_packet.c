/**
 * @file test_packet.c
 *
 * Tests for the packet format and transmitter (packet_format.c,
 * packet_tx.c): CRC check values, scrambling, the B sequence, and framing a
 * payload into symbols and back.
 */

#include <stdio.h>
#include <string.h>
#include "packet_format.h"
#include "packet_tx.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

static void test_crc_check_values(void) {
    const uint8_t s[] = "123456789";
    CHECK_EQ_U(packet_crc8(s, 9), 0xF4);
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

static void test_framing(void) {
    static PacketTx_t tx;
    static uint32_t symbols[400];
    const uint8_t payload[] = "Hello, chirp!";
    const uint32_t len = sizeof payload - 1;
    for (uint32_t b = 4; b <= 6; b++) {
        packet_tx_init(&tx, b);
        CHECK(packet_tx_start(&tx, payload, len));
        CHECK(!packet_tx_start(&tx, payload, len)); // Busy
        uint32_t n = 0, s;
        while (packet_tx_next_symbol(&tx, &s) && n < 400) symbols[n++] = s;
        CHECK(!packet_tx_busy(&tx));
        CHECK_EQ_U(n, packet_symbol_count(len, b));
        printf("  %u bits/symbol: %u-byte payload -> %u symbols\n", b, len, n);

        for (uint32_t i = 0; i < PACKET_A_LEN; i++) CHECK_EQ_U(symbols[i], 0);
        for (uint32_t i = 0; i < PACKET_B_LEN; i++) CHECK_EQ_U(symbols[PACKET_A_LEN + i], packet_b_symbol(i, b));
        for (uint32_t i = 0; i < n; i++) CHECK(symbols[i] < (1u << b));

        uint8_t bytes[PACKET_MAX_BYTES];
        const uint32_t nbytes = PACKET_HEADER_BYTES + len + PACKET_CRC_BYTES;
        unpack(&symbols[PACKET_PREAMBLE_LEN], n - PACKET_PREAMBLE_LEN, b, bytes, nbytes);
        CHECK_EQ_U(bytes[0], len);
        CHECK_EQ_U(bytes[1], packet_crc8(bytes, 1));
        packet_scramble(&bytes[2], len);
        CHECK(memcmp(&bytes[2], payload, len) == 0);
        const uint16_t crc = packet_crc16(bytes, 2 + len);
        CHECK_EQ_U(bytes[2 + len], crc >> 8);
        CHECK_EQ_U(bytes[3 + len], crc & 0xFF);
    }
    // Too long
    packet_tx_init(&tx, 6);
    static uint8_t big[PACKET_MAX_PAYLOAD + 1];
    CHECK(!packet_tx_start(&tx, big, sizeof big));
    CHECK(packet_tx_start(&tx, big, PACKET_MAX_PAYLOAD));
}

// Packet lengths quoted in packet_format.md
static void test_airtime(void) {
    CHECK_EQ_U(packet_symbol_count(0, 6), 22);
    CHECK_EQ_U(packet_symbol_count(16, 6), 43);
    CHECK_EQ_U(packet_symbol_count(64, 6), 107);
    CHECK_EQ_U(packet_symbol_count(255, 6), 362);
}

int main(void) {
    RUN_TEST(test_crc_check_values);
    RUN_TEST(test_scramble);
    RUN_TEST(test_b_sequence);
    RUN_TEST(test_gray);
    RUN_TEST(test_framing);
    RUN_TEST(test_airtime);

    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
