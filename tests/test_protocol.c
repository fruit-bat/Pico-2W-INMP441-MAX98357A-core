/**
 * @file test_protocol.c
 *
 * Scenario tests for the host protocol engine (src/protocol.c) against a
 * pretend board (fake_board.c) on a simulated clock. Each test feeds the
 * engine host lines and checks the exact text that comes back.
 *
 * Every line both ways carries a check ("XXXX " in front). The helpers add
 * it to what the host sends and verify and remove it from what the board
 * sends, so most tests read as plain text; test_checks covers the check
 * itself.
 */

#include <stdio.h>
#include <string.h>
#include "fake_board.h"
#include "protocol.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

static Protocol_t proto;
static FakeBoard_t fb;
static char out[65536];  // Decoded output
static char raw[65536];  // As sent
static char enc[65536];  // Encoded input

#define CHECK_STR(actual, expected) do { \
    test_checks++; \
    const char *a_ = (actual), *e_ = (expected); \
    if (strcmp(a_, e_) != 0) { \
        test_failures++; \
        printf("  FAIL %s:%d: got\n----\n%s----\n  expected\n----\n%s----\n", __FILE__, __LINE__, a_, e_); \
    } \
} while (0)

// Adds a check to each line of plain text, as a host would. Blank lines
// (or only spaces) stay as they are; a CR before the LF is sent but not checked
static const char *encode(const char *plain) {
    char *o = enc;
    for (const char *line = plain; *line;) {
        const char *nl = strchr(line, '\n');
        const size_t n = nl ? (size_t)(nl - line) : strlen(line);
        size_t text = n;
        if (text > 0 && line[text - 1] == '\r') text--;
        bool blank = true;
        for (size_t i = 0; i < text; i++) blank = blank && line[i] == ' ';
        if (!blank) o += sprintf(o, "%04X ", protocol_crc16(line, text));
        memcpy(o, line, n);
        o += n;
        if (!nl) break;
        *o++ = '\n';
        line = nl + 1;
    }
    *o = 0;
    return enc;
}

// Checks and removes each line's check, as a host would
static const char *decode(const char *in) {
    char *o = out;
    for (const char *line = in; *line;) {
        const char *nl = strchr(line, '\n');
        CHECK(nl != NULL); // Whole lines only
        if (!nl) break;
        const size_t n = (size_t)(nl - line);
        unsigned crc = 0;
        const bool ok = n >= 5 && line[4] == ' ' && sscanf(line, "%4X", &crc) == 1
            && crc == protocol_crc16(line + 5, n - 5);
        test_checks++;
        if (!ok) { test_failures++; printf("  FAIL bad check on line: %.*s\n", (int)n, line); }
        if (n >= 5) { memcpy(o, line + 5, n - 5); o += n - 5; }
        *o++ = '\n';
        line = nl + 1;
    }
    *o = 0;
    return out;
}

static const char *drain_raw(void) {
    const size_t n = protocol_output(&proto, raw, sizeof raw - 1);
    raw[n] = 0;
    return raw;
}

// Everything waiting for the host, checked and without the checks
static const char *drain(void) {
    return decode(drain_raw());
}

// Host sends exactly these bytes; returns what the board sent (decoded)
static const char *talk_raw(const char *bytes) {
    const size_t n = protocol_input(&proto, bytes, strlen(bytes));
    CHECK_EQ_U(n, strlen(bytes));
    return drain();
}

// Host sends plain lines, each with its check added
static const char *talk(const char *text) {
    return talk_raw(encode(text));
}

// A fresh board and engine, with the start-up "* hello" read and checked
static void setup(void) {
    fake_board_init(&fb, &proto);
    protocol_init(&proto, &fb.board);
    protocol_start(&proto);
    CHECK_STR(drain(), "* hello seq=0 proto=1 version=test id=0011223344556677\n");
}

static uint32_t count_lines(const char *s) {
    uint32_t n = 0;
    for (; *s; s++) n += *s == '\n';
    return n;
}

// Every line of s starting with prefix comes in one unbroken run
static bool lines_together(const char *s, const char *prefix) {
    int runs = 0;
    bool in_run = false;
    for (const char *line = s; *line;) {
        const bool match = strncmp(line, prefix, strlen(prefix)) == 0;
        if (match && !in_run) runs++;
        in_run = match;
        const char *nl = strchr(line, '\n');
        if (!nl) break;
        line = nl + 1;
    }
    return runs == 1;
}

static void inject_packet(uint32_t length) {
    static uint8_t payload[PROTOCOL_PAYLOAD_MAX];
    for (uint32_t i = 0; i < length; i++) payload[i] = (uint8_t)i;
    ProtocolEvent_t e = { .type = PROTO_EV_RX_PACKET, .frame = 7 };
    e.u.packet.crc_ok = true;
    e.u.packet.length = length;
    e.u.packet.min_margin = 1.5f;
    e.u.packet.payload = payload;
    protocol_event(&proto, &e);
}

// ---------------------------------------------------------------------------

static void test_start(void) {
    setup();
    CHECK_EQ_U(fb.apply_calls, 1);                        // All settings pushed at start-up
    CHECK_EQ_U(fb.last_changed, (1u << SETTING_COUNT) - 1u);
    CHECK_EQ_U(fb.settings.value[SETTING_TX_VOLUME], -28);
    CHECK_STR(talk("a1 hello\n"), "a1 proto=1 version=test id=0011223344556677\na1 ok\n");
}

static void test_get(void) {
    setup();
    CHECK_STR(talk("a2 get tx.volume\n"), "a2 tx.volume=-28\na2 ok\n");
    CHECK_STR(talk("a3 get rx.state rx.delay test.mode\n"),
        "a3 rx.state=search\na3 rx.delay=960\na3 test.mode=packet\na3 ok\n");
    CHECK_STR(talk("a4 get nothing.here\n"), "a4 err 502 unknown name nothing.here\n");
    CHECK_STR(talk("a5 get tx.volume nothing.here\n"), "a5 err 502 unknown name nothing.here\n");
    CHECK_STR(talk("a6 get link.*\n"), "a6 link.spacing=4\na6 link.ecc=rs2\na6 link.symbols=16\na6 link.bits=4\na6 ok\n");

    const char *all = talk("a7 get\n");
    printf("  get (all): %lu lines, %lu bytes\n", (unsigned long)count_lines(all), (unsigned long)strlen(all));
    CHECK_EQ_U(count_lines(all), SETTING_COUNT + 36u + 1u);
    CHECK(strstr(all, "a7 info.f0=15000.0\n") != NULL);
    CHECK(strstr(all, "a7 count.events_dropped=0\n") != NULL);
    CHECK(strcmp(all + strlen(all) - 6, "a7 ok\n") == 0);

    const char *rx = talk("a8 get rx.*\n");
    CHECK_EQ_U(count_lines(rx), 4u + 2u + 3u + 5u + 1u);
}

static void test_list(void) {
    setup();
    CHECK_STR(talk("l1 list tx.volume\n"),
        "l1 name=tx.volume kind=setting type=int range=-55..-1 default=-28 units=dBFS help=Transmit level\nl1 ok\n");
    CHECK_STR(talk("l2 list rx.tracking link.spacing\n"),
        "l2 name=rx.tracking kind=setting type=choice values=off,decision,pilot default=off help=Symbol timing tracking\n"
        "l2 name=link.spacing kind=setting type=int values=1,2,4 default=4 units=bins "
        "help=Bins between symbols in packets sent and the test modes (4 bits/symbol at 4, 5 at 2, 6 at 1); "
        "packet receivers follow the header\nl2 ok\n");
    CHECK_STR(talk("l3 list count.good\n"),
        "l3 name=count.good kind=status type=int help=Packets received with a good CRC\nl3 ok\n");
    const char *all = talk("l4 list\n");
    printf("  list (all): %lu bytes with checks (reply limit %u)\n", (unsigned long)strlen(raw), PROTOCOL_REPLY_MAX);
    CHECK(strcmp(all + strlen(all) - 6, "l4 ok\n") == 0);
    const char *help = talk("l6 help\n");
    CHECK(strstr(help, "l6 command=send help=send hex=<bytes>") != NULL);
    CHECK(strcmp(help + strlen(help) - 6, "l6 ok\n") == 0);
}

static void test_set(void) {
    setup();
    CHECK_STR(talk("s1 set tx.volume=-31\n"), "s1 ok\n");
    CHECK_EQ_U(fb.settings.value[SETTING_TX_VOLUME], -31);
    CHECK_EQ_U(fb.last_changed, 1u << SETTING_TX_VOLUME);

    // Rejected, not clamped; and nothing changes
    const uint32_t calls = fb.apply_calls;
    CHECK_STR(talk("s2 set tx.volume=-60\n"), "s2 err 503 tx.volume out of range -55..-1\n");
    CHECK_STR(talk("s3 set tx.volume=-40 link.spacing=3\n"), "s3 err 503 link.spacing must be one of 1,2,4\n");
    CHECK_EQ_U(fb.settings.value[SETTING_TX_VOLUME], -31);
    CHECK_EQ_U(fb.apply_calls, calls);

    // test.symbol's range follows the spacing; a spacing change wraps it
    CHECK_STR(talk("s4 set link.spacing=1 test.symbol=40\n"), "s4 ok\n");
    CHECK_STR(talk("s5 set link.spacing=4\n"), "s5 ok test.symbol=8\n");
    CHECK_STR(talk("s6 set test.symbol=16\n"), "s6 err 503 test.symbol out of range 0..15\n");
    CHECK_STR(talk("s7 set test.symbol=15\n"), "s7 ok\n");

    // Choices
    CHECK_STR(talk("s8 set rx.tracking=pilot rx.align=off\n"), "s8 ok\n");
    CHECK_EQ_U(fb.settings.value[SETTING_RX_TRACKING], 2);
    CHECK_EQ_U(fb.settings.value[SETTING_RX_ALIGN], 0);
    CHECK_STR(talk("s9 set rx.align=maybe\n"), "s9 err 503 rx.align must be one of off,on\n");

    // Other mistakes
    CHECK_STR(talk("t1 set rx.delay=5\n"), "t1 err 504 rx.delay is read only\n");
    CHECK_STR(talk("t2 set tx.loudness=5\n"), "t2 err 502 unknown setting tx.loudness\n");
    CHECK_STR(talk("t3 set tx.volume=-3 tx.volume=-4\n"), "t3 err 505 tx.volume given twice\n");
    CHECK_STR(talk("t4 set\n"), "t4 err 505 set needs name=value\n");
    CHECK_STR(talk("t5 set tx.volume\n"), "t5 err 505 expected name=value, not tx.volume\n");
    CHECK_STR(talk("t6 set tx.volume=loud\n"), "t6 err 503 tx.volume needs an integer, not 'loud'\n");
    CHECK_STR(talk("t7 set tx.volume=-3x\n"), "t7 err 503 tx.volume needs an integer, not '-3x'\n");

    // Setting the same value again doesn't disturb the board
    const uint32_t before = fb.apply_calls;
    CHECK_STR(talk("t8 set tx.volume=-31\n"), "t8 ok\n");
    CHECK_EQ_U(fb.apply_calls, before);
    CHECK_STR(talk("t9 get tx.volume link.spacing test.symbol\n"),
        "t9 tx.volume=-31\nt9 link.spacing=4\nt9 test.symbol=15\nt9 ok\n");
}

static void test_send(void) {
    setup();
    CHECK_STR(talk("p1 send hex=4869\n"), "p1 ok id=1\n");
    CHECK_STR(talk("p2 get tx.queued\n"), "p2 tx.queued=1\np2 ok\n");
    fake_board_advance_frames(&fb, 1);
    CHECK_STR(drain(), "* tx.start seq=1 frame=1 id=1\n");

    // 2 bytes at 4 bits/symbol with rs2 (the default): 16 preamble + 8 header
    // + (16 payload + 16 CRC + 32 check bits) / 4 = 40 frames
    const uint32_t frames = fake_board_packet_frames(&fb, 2);
    CHECK_EQ_U(frames, 40);
    fake_board_advance_frames(&fb, frames);
    CHECK_STR(drain(),
        "* rx.locked seq=2 frame=4 align=+0 fraction=+0.120 moved=+2\n"
        "* rx.trained seq=3 frame=19 b_start=9 coherence=0.830 strongest=-1 earliest=+0 moved=+0\n"
        "* tx.done seq=4 frame=41 id=1 result=sent\n"
        "* rx.packet seq=5 frame=41 crc=ok len=2 spacing=4 fixed=0 ecc=rs2 corrected=0 min_margin=2.000 weakest=0 weak=0 hex=4869\n");
    CHECK_STR(talk("p3 get count.good rx.delay rx.last_train.coherence\n"),
        "p3 count.good=1\np3 rx.delay=962\np3 rx.last_train.coherence=0.830\np3 ok\n");

    // The queue holds 2: a third is refused, and doesn't use up an id
    CHECK_STR(talk("p4 send hex=01\np5 send hex=02\np6 send hex=03\n"),
        "p4 ok id=2\np5 ok id=3\np6 err 450 transmit queue full\n");
    fake_board_advance_frames(&fb, 1);
    CHECK_STR(talk("p7 send hex=04\n"), "* tx.start seq=6 frame=42 id=2\np7 ok id=4\n");

    // Real time: 63 bytes at 4 bits/symbol with rs2 take 16 + 8 + 138 frames, 3.76 s
    setup();
    CHECK_STR(talk("p8 set rx.during_tx=off\n"), "p8 ok\n");
    char line[600] = "p9 send hex=";
    for (int i = 0; i < 63; i++) strcat(line, "ab");
    strcat(line, "\n");
    CHECK_STR(talk(line), "p9 ok id=1\n");
    fake_board_advance_us(&fb, 3700000u);
    CHECK_STR(drain(), "* tx.start seq=1 frame=1 id=1\n");
    fake_board_advance_us(&fb, 100000u);
    CHECK_STR(drain(), "* tx.done seq=2 frame=163 id=1 result=sent\n");

    // Bad hex
    CHECK_STR(talk("q1 send hex=486\n"), "q1 err 506 odd number of hex digits\n");
    CHECK_STR(talk("q2 send hex=zz\n"), "q2 err 506 not hex\n");
    strcpy(line, "q3 send hex=");
    for (int i = 0; i < 64; i++) strcat(line, "00");
    strcat(line, "\n");
    CHECK_STR(talk(line), "q3 err 506 more than 63 bytes\n");
    CHECK_STR(talk("q4 send\n"), "q4 err 505 send takes hex=<bytes>\n");
    CHECK_STR(talk("q5 send text=hi\n"), "q5 err 505 send takes hex=<bytes>\n");
    CHECK_STR(talk("q6 send hex=\n"), "q6 ok id=2\n"); // An empty packet is allowed
    CHECK_STR(talk("q7 send hex=4A4b\n"), "q7 ok id=3\n"); // Either case
}

// rx.symbols=on: each packet is followed by its symbol decisions
static void test_symbols(void) {
    setup();
    CHECK_STR(talk("r1 send hex=4869\n"), "r1 ok id=1\n");
    fake_board_advance_frames(&fb, 30);
    CHECK(strstr(drain(), "* rx.symbols") == NULL); // Off by default
    CHECK_STR(talk("r2 set rx.symbols=on\nr3 send hex=4869\n"), "r2 ok\nr3 ok id=2\n");
    fake_board_advance_frames(&fb, 30);
    const char *s = drain();
    CHECK(strstr(s, "hex=4869\n* rx.symbols seq=") != NULL);
    CHECK(strstr(s, " count=3 symbols=013f07 second=020006 margins=1500,1002,65535\n") != NULL);
}

// Events can come before or after a reply, but lines are whole, a reply's
// lines stay together, and a reply comes before what it causes
static void test_ordering(void) {
    setup();

    // An event half-sent when the command arrives is finished first
    inject_packet(4);
    char all[512];
    CHECK_EQ_U(protocol_output(&proto, all, 10), 10);
    const char *cmd1 = encode("e1 get tx.volume\n");
    CHECK_EQ_U(protocol_input(&proto, cmd1, strlen(cmd1)), strlen(cmd1));
    strcpy(all + 10, drain_raw());
    CHECK_STR(decode(all),
        "* rx.packet seq=1 frame=7 crc=ok len=4 spacing=0 fixed=0 ecc=none corrected=0 min_margin=1.500 weakest=0 weak=0 hex=00010203\n"
        "e1 tx.volume=-28\ne1 ok\n");

    // Events around a long reply never land inside it
    inject_packet(1);
    const char *cmd2 = encode("e2 get rx.*\n");
    protocol_input(&proto, cmd2, strlen(cmd2));
    inject_packet(2);
    const char *s = drain();
    CHECK(lines_together(s, "e2 "));
    CHECK(strncmp(s, "* rx.packet", 11) == 0);
    CHECK(strstr(s, "e2 ok\n* rx.packet") != NULL);

    // The reply to send comes before tx.start and tx.done, even with
    // the clock running between input bytes
    const char *cmd = encode("e3 send hex=00\n");
    for (const char *c = cmd; *c; c++) {
        protocol_input(&proto, c, 1);
        fake_board_advance_frames(&fb, 1);
    }
    fake_board_advance_frames(&fb, 40);
    s = drain();
    const char *ok = strstr(s, "e3 ok id=1"), *start = strstr(s, "* tx.start"), *done = strstr(s, "* tx.done");
    CHECK(ok && start && done && ok < start && start < done);
}

static void test_input(void) {
    setup();
    CHECK_STR(talk("c1 hello\r\n"), "c1 proto=1 version=test id=0011223344556677\nc1 ok\n");
    CHECK_STR(talk("\n\r\n   \n"), "");
    CHECK_STR(talk("c2   get    tx.volume  \n"), "c2 tx.volume=-28\nc2 ok\n"); // Extra spaces are forgiven

    // Too long: it can't be checked, so not even its tag is trusted; the
    // answer is an event (with a sequence number), and the next line is fine
    static char line[1600];
    strcpy(line, "c3 send hex=");
    for (int i = 0; i < 1500; i++) strcat(line, "0");
    strcat(line, "\n");
    CHECK_STR(talk(line), "* err seq=1 500 line too long\n");
    CHECK_STR(talk("c4 get tx.volume\n"), "c4 tx.volume=-28\nc4 ok\n");

    // No usable tag (checked lines, so the tag is genuinely missing or bad)
    CHECK_STR(talk("*bad thing\n"), "* err seq=2 500 line has no valid tag\n");
    CHECK_STR(talk("abcdefghi hello\n"), "* err seq=3 500 line has no valid tag\n");
    CHECK_STR(talk("a-1 hello\n"), "* err seq=4 500 line has no valid tag\n");

    CHECK_STR(talk("c5\n"), "c5 err 501 no command\n");
    CHECK_STR(talk("c6 dance\n"), "c6 err 501 unknown command dance\n");
    CHECK_STR(talk("c7 HELLO\n"), "c7 err 501 unknown command HELLO\n");
    CHECK_STR(talk("c8 get tx.volume\x80\n"), "c8 err 500 characters other than printable ASCII\n");
    CHECK_STR(talk("c9 get\ttx.volume\n"), "c9 err 500 characters other than printable ASCII\n");
    CHECK_STR(talk("d1 hello there\n"), "d1 err 505 hello takes no arguments\n");

    // A tag may be reused; lines may arrive a byte at a time
    CHECK_STR(talk("x hello\nx hello\n"),
        "x proto=1 version=test id=0011223344556677\nx ok\nx proto=1 version=test id=0011223344556677\nx ok\n");
    const char *cmd = encode("d2 get tx.volume\n");
    for (const char *c = cmd; *c; c++) protocol_input(&proto, c, 1);
    CHECK_STR(drain(), "d2 tx.volume=-28\nd2 ok\n");
}

static void test_nudge(void) {
    setup();
    CHECK_STR(talk("n1 nudge samples=-8\n"), "n1 ok delay=952\n");
    CHECK_STR(talk("n2 nudge samples=100\n"), "n2 ok delay=28\n"); // Wraps round the frame
    CHECK_STR(talk("n3 nudge samples=2000\n"), "n3 err 503 samples out of range -1023..1023\n");
    CHECK_STR(talk("n4 nudge\n"), "n4 err 505 nudge takes samples=<n>\n");
    CHECK_STR(talk("n5 get rx.delay\n"), "n5 rx.delay=28\nn5 ok\n");
}

static void test_save_load(void) {
    setup();
    CHECK_STR(talk("v1 save\n"), "v1 err 452 saving settings is not available yet\n");
    CHECK_STR(talk("v2 load\n"), "v2 ok\n"); // Nothing saved: the defaults

    fb.save_enabled = true;
    CHECK_STR(talk("v3 set tx.volume=-40 link.spacing=2\n"), "v3 ok\n");
    CHECK_STR(talk("v4 save\n"), "v4 ok\n");
    CHECK_STR(talk("v5 set tx.volume=-10\n"), "v5 ok\n");
    CHECK_STR(talk("v6 load\n"), "v6 ok\n");
    CHECK_EQ_U(fb.settings.value[SETTING_TX_VOLUME], -40);
    CHECK_STR(talk("v7 reset settings\n"), "v7 ok\n");
    CHECK_STR(talk("v8 get tx.volume link.spacing\n"), "v8 tx.volume=-28\nv8 link.spacing=4\nv8 ok\n");
    CHECK_STR(talk("v9 load\n"), "v9 ok\n"); // The saved ones were kept
    CHECK_EQ_U(fb.settings.value[SETTING_TX_VOLUME], -40);
    CHECK_EQ_U(fb.settings.value[SETTING_LINK_SPACING], 2);

    // A restart starts from the saved settings
    protocol_init(&proto, &fb.board);
    protocol_start(&proto);
    drain();
    CHECK_STR(talk("w1 get tx.volume\n"), "w1 tx.volume=-40\nw1 ok\n");

    // Saved settings that don't make sense are ignored
    fb.saved.value[SETTING_TX_VOLUME] = 7;
    protocol_init(&proto, &fb.board);
    protocol_start(&proto);
    drain();
    CHECK_STR(talk("w2 get tx.volume\n"), "w2 tx.volume=-28\nw2 ok\n");

    // reset counters, reset rx
    fb.status.good = 5;
    fb.status.rx_state = PROTO_RX_PAYLOAD;
    CHECK_STR(talk("w3 reset counters\nw4 reset rx\n"), "w3 ok\nw4 ok\n");
    fake_board_advance_frames(&fb, 1); // Core 1 resets the receiver on its next pass
    CHECK_STR(talk("w5 get count.good rx.state\n"), "w5 count.good=0\nw5 rx.state=search\nw5 ok\n");
    CHECK_STR(talk("w6 reset everything\n"), "w6 err 503 reset takes counters, rx or settings\n");
}

// Every line both ways starts with "XXXX ": CRC-16/CCITT of the rest
static void test_checks_on_lines(void) {
    CHECK_EQ_U(protocol_crc16("123456789", 9), 0x29B1); // The standard check value
    setup();

    // What the board sends: upper-case hex, a space, the line
    char expect[64];
    const char *k1 = encode("k1 get tx.volume\n");
    protocol_input(&proto, k1, strlen(k1));
    drain_raw();
    snprintf(expect, sizeof expect, "%04X k1 tx.volume=-28\n%04X k1 ok\n",
        protocol_crc16("k1 tx.volume=-28", 16), protocol_crc16("k1 ok", 5));
    CHECK_STR(raw, expect);

    // Lines that fail: no check, a damaged one, one character changed,
    // no space after the check, and a 4-letter tag mistaken for a check
    CHECK_STR(talk_raw("k2 hello\n"), "* err seq=1 500 line failed its check\n");
    snprintf(expect, sizeof expect, "%04X k3 hello\n", protocol_crc16("k3 hello", 8) ^ 0x0100u);
    CHECK_STR(talk_raw(expect), "* err seq=2 500 line failed its check\n");
    snprintf(expect, sizeof expect, "%04X k4 hello\n", protocol_crc16("k4 hellp", 8));
    CHECK_STR(talk_raw(expect), "* err seq=3 500 line failed its check\n");
    snprintf(expect, sizeof expect, "%04Xk5 hello\n", protocol_crc16("k5 hello", 8));
    CHECK_STR(talk_raw(expect), "* err seq=4 500 line failed its check\n");
    CHECK_STR(talk_raw("abcd get tx.volume\n"), "* err seq=5 500 line failed its check\n");
    CHECK_STR(talk("k6 get count.link_errors\n"), "k6 count.link_errors=5\nk6 ok\n");

    // Lower case is accepted; blank lines need no check
    snprintf(expect, sizeof expect, "%04x k7 get tx.volume\n\n", protocol_crc16("k7 get tx.volume", 16));
    CHECK_STR(talk_raw(expect), "k7 tx.volume=-28\nk7 ok\n");

    // Rubbish (as at power-up) between good lines costs only itself
    snprintf(expect, sizeof expect, "\x13\xfe\x80garbage\n%04X k8 hello\n", protocol_crc16("k8 hello", 8));
    CHECK_STR(talk_raw(expect),
        "* err seq=6 500 line failed its check\nk8 proto=1 version=test id=0011223344556677\nk8 ok\n");

    CHECK_STR(talk("k9 reset counters\nk10 get count.link_errors\n"), "k9 ok\nk10 count.link_errors=0\nk10 ok\n");
}

// ---------------------------------------------------------------------------
// Streams
// ---------------------------------------------------------------------------

static bool starts(const char *s, const char *prefix) {
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static uint32_t count_char(const char *s, char c) {
    uint32_t n = 0;
    for (; *s; s++) n += *s == c;
    return n;
}

static void test_sub_commands(void) {
    setup();
    CHECK_STR(talk("s1 sub\n"), "s1 ok\n"); // None yet
    CHECK_STR(talk("s2 sub stream=frame fields=delay,rx_symbol every=4\n"), "s2 ok\n");
    CHECK_STR(talk("s3 sub stream=folded units=power\n"), "s3 ok\n");
    CHECK_STR(talk("s4 sub stream=spectrum from=10 to=19\n"), "s4 ok\n");
    CHECK_STR(talk("s5 sub stream=shape\n"), "s5 ok\n");
    const char *list =
        "s6 stream=frame every=4 fields=delay,rx_symbol\n"
        "s6 stream=folded every=1 units=power\n"
        "s6 stream=shape units=db\n"
        "s6 stream=spectrum every=1 units=db from=10 to=19\n"
        "s6 ok\n";
    CHECK_STR(talk("s6 sub\n"), list);

    CHECK_STR(talk("e1 sub stream=video\n"), "e1 err 507 unknown stream video\n");
    CHECK_STR(talk("e2 sub stream=frame fields=delay,colour\n"), "e2 err 507 frame has no field colour\n");
    CHECK_STR(talk("e3 sub stream=folded fields=delay\n"), "e3 err 505 folded has no fields\n");
    CHECK_STR(talk("e4 sub stream=frame units=db\n"), "e4 err 505 frame has no units\n");
    CHECK_STR(talk("e5 sub stream=band units=loud\n"), "e5 err 503 units must be one of db,power\n");
    CHECK_STR(talk("e6 sub stream=frame every=0\n"), "e6 err 503 every out of range 1..1000\n");
    CHECK_STR(talk("e7 sub stream=shape every=2\n"), "e7 err 505 shape is sent when it changes; no every\n");
    CHECK_STR(talk("e8 sub stream=band from=1\n"), "e8 err 505 only spectrum takes from and to\n");
    const char *range = "err 503 from..to must be within 0..1023, at most 512 bins\n";
    char expect[128];
    snprintf(expect, sizeof expect, "e9 %s", range);
    CHECK_STR(talk("e9 sub stream=spectrum from=0 to=512\n"), expect);
    snprintf(expect, sizeof expect, "f1 %s", range);
    CHECK_STR(talk("f1 sub stream=spectrum from=600 to=1024\n"), expect);
    snprintf(expect, sizeof expect, "f2 %s", range);
    CHECK_STR(talk("f2 sub stream=spectrum from=9 to=5\n"), expect);
    CHECK_STR(talk("f3 sub every=2\n"), "f3 err 505 sub needs stream=<name>\n");
    CHECK_STR(talk("f4 sub stream=frame stream=band\n"), "f4 err 505 stream given twice\n");
    CHECK_STR(talk("f5 sub stream=frame colour=red\n"), "f5 err 505 sub has no argument colour\n");
    CHECK_STR(talk("f6 sub frame\n"), "f6 err 505 expected name=value, not frame\n");
    // A failed sub leaves things as they were
    CHECK_STR(talk("s6 sub\n"), list);

    CHECK_STR(talk("u1 unsub stream=folded\n"), "u1 ok\n");
    CHECK_STR(talk("u2 unsub stream=folded\n"), "u2 ok\n"); // Safe to repeat
    CHECK_STR(talk("u3 unsub stream=video\n"), "u3 err 507 unknown stream video\n");
    CHECK_STR(talk("u4 unsub\n"), "u4 err 505 unsub takes stream=<name> or all\n");
    CHECK_STR(talk("u5 unsub all\n"), "u5 ok\n");
    CHECK_STR(talk("u6 sub\n"), "u6 ok\n");
}

static void test_stream_lines(void) {
    setup();
    // One frame in 4, chosen fields; the frame number always comes first
    CHECK_STR(talk("a1 sub stream=frame fields=delay,state every=4\n"), "a1 ok\n");
    fake_board_advance_frames(&fb, 8);
    CHECK_STR(drain(), "* frame seq=1 frame=1 delay=960 state=search\n* frame seq=2 frame=5 delay=960 state=search\n");

    // A new sub replaces the old one: every field
    CHECK_STR(talk("a2 sub stream=frame\n"), "a2 ok\n");
    fake_board_advance_frames(&fb, 1);
    CHECK_STR(drain(), "* frame seq=3 frame=9 tx_symbol=0 delay=960 rx_symbol=0 angle=0.00000 error_angle=+0.20000 "
        "magnitude=0.3000 strength=1.0000e-06 peak=0.0020 timing_error=+3.000 timing_rate=+0.0000 state=search "
        "corr=+0.300 data_symbol=0 data_margin=0.000 data_quality=0.300 rx_missed=0 tx_missed=0 records_dropped=0\n");

    // Spectra, in dB and as raw power
    CHECK_STR(talk("a3 unsub all\na4 sub stream=folded\n"), "a3 ok\na4 ok\n");
    static float folded[PROTOCOL_CHIRP_BINS], band[2u * PROTOCOL_CHIRP_BINS], spectrum[1024];
    for (uint32_t i = 0; i < PROTOCOL_CHIRP_BINS; i++) folded[i] = 1e-3f;
    folded[0] = 1.0f; folded[1] = 0.5f; folded[2] = 0.0f; folded[3] = 100.0f; folded[4] = 2.0f;
    for (uint32_t i = 0; i < 2u * PROTOCOL_CHIRP_BINS; i++) band[i] = 1.0f;
    for (uint32_t i = 0; i < 1024u; i++) spectrum[i] = 1.25e-5f * (float)(i + 1u);
    ProtocolFrame_t f = { .frame = 500, .folded = folded, .band = band, .spectrum = spectrum, .spectrum_len = 1024 };
    protocol_frame(&proto, &f);
    const char *line = drain();
    CHECK(starts(line, "* folded seq=4 frame=500 units=db values=0.0,-3.0,-200.0,20.0,3.0,-30.0,-30.0,"));
    CHECK_EQ_U(count_char(line, ','), PROTOCOL_CHIRP_BINS - 1u);
    CHECK_STR(talk("a5 sub stream=folded units=power\n"), "a5 ok\n");
    protocol_frame(&proto, &f);
    line = drain();
    CHECK(starts(line, "* folded seq=5 frame=500 units=power values=1.000e+00,5.000e-01,0.000e+00,1.000e+02,"));

    CHECK_STR(talk("a6 unsub all\na7 sub stream=band\n"), "a6 ok\na7 ok\n");
    protocol_frame(&proto, &f);
    line = drain();
    CHECK(starts(line, "* band seq=6 frame=500 units=db values=0.0,0.0,"));
    CHECK_EQ_U(count_char(line, ','), 2u * PROTOCOL_CHIRP_BINS - 1u);

    CHECK_STR(talk("a8 unsub all\na9 sub stream=spectrum from=10 to=19 units=power\n"), "a8 ok\na9 ok\n");
    protocol_frame(&proto, &f);
    CHECK_STR(drain(), "* spectrum seq=7 frame=500 units=power from=10 values="
        "1.375e-04,1.500e-04,1.625e-04,1.750e-04,1.875e-04,2.000e-04,2.125e-04,2.250e-04,2.375e-04,2.500e-04\n");

    // The longest line there is: 512 bins as raw power
    CHECK_STR(talk("b1 sub stream=spectrum from=512 to=1023 units=power\n"), "b1 ok\n");
    protocol_frame(&proto, &f);
    line = drain_raw();
    printf("  512-bin spectrum as power: %lu bytes (limit %u)\n", (unsigned long)strlen(line), PROTOCOL_OUT_LINE_MAX);
    CHECK(strlen(line) <= PROTOCOL_OUT_LINE_MAX + 1u);
    line = decode(line);
    CHECK_EQ_U(count_char(line, ','), 511);
    CHECK(strstr(line, "from=512 values=6.413e-03,") != NULL);
    CHECK(strstr(line, ",1.280e-02\n") != NULL);

    // The shape: the current one on subscribing, then only when one is
    // learned, here from the board's own packet
    CHECK_STR(talk("b2 unsub all\nb3 sub stream=shape\nb4 send hex=00\n"), "b2 ok\nb3 ok\nb4 ok id=1\n");
    fake_board_advance_frames(&fb, 40);
    line = drain();
    CHECK(starts(line, "* tx.start seq=")); // Then the current shape, on the same frame
    CHECK(strstr(line, "\n* shape seq=") != NULL && strstr(line, " units=db values=") != NULL);
    line = strchr(strchr(line, '\n') + 1, '\n') + 1; // Past both
    CHECK_EQ_U(count_char(line, '\n'), 5); // rx.locked, rx.trained, shape, tx.done, rx.packet
    const char *shape = strstr(line, "* shape seq=");
    const char *trained = strstr(line, "* rx.trained");
    CHECK(shape && trained && trained < shape && strstr(shape, " values=0.0,") != NULL);
    CHECK(shape && strstr(shape, ",-5.2\n") != NULL); // The early path, 0.3 = -5.2 dB
}

// A spacing change cuts a packet short; the host still hears how it ended,
// for the packet being sent and the one being received (here the same one:
// the board hears itself). So does "reset rx". Every rx.locked ends in one
// conclusion
static void test_abandoned(void) {
    setup();
    CHECK_STR(talk("x1 set rx.during_tx=off\nx2 send hex=0102030405\n"), "x1 ok\nx2 ok id=1\n");
    fake_board_advance_frames(&fb, 10);
    CHECK_STR(drain(), "* tx.start seq=1 frame=1 id=1\n");
    CHECK_STR(talk("x3 set link.spacing=2\n"), "x3 ok\n"); // The reply comes first...
    fake_board_advance_frames(&fb, 1);
    CHECK_STR(drain(), "* tx.done seq=2 frame=11 id=1 result=abandoned\n"); // ...then what it caused

    // Heard, mid-payload
    CHECK_STR(talk("x4 set rx.during_tx=on\nx5 send hex=0102030405\n"), "x4 ok\nx5 ok id=2\n");
    fake_board_advance_frames(&fb, 24);
    CHECK_STR(talk("x6 set link.spacing=4\n"), "* tx.start seq=3 frame=12 id=2\n"
        "* rx.locked seq=4 frame=15 align=+0 fraction=+0.120 moved=+2\n"
        "* rx.trained seq=5 frame=30 b_start=20 coherence=0.830 strongest=-1 earliest=+0 moved=+0\nx6 ok\n");
    fake_board_advance_frames(&fb, 1);
    CHECK_STR(drain(), "* tx.done seq=6 frame=36 id=2 result=abandoned\n"
        "* rx.lost seq=7 frame=36 symbols=7 reason=signal\n");

    // reset rx while hearing: rx.lost, and nothing more from that packet
    CHECK_STR(talk("x7 send hex=0102030405\n"), "x7 ok id=3\n");
    fake_board_advance_frames(&fb, 20);
    drain();
    CHECK_STR(talk("x8 reset rx\n"), "x8 ok\n");
    fake_board_advance_frames(&fb, 40);
    const char *s = drain();
    CHECK(starts(s, "* rx.lost seq=") && strstr(s, " reason=reset\n") != NULL);
    CHECK(strstr(s, "* tx.done") != NULL && strstr(s, "result=sent") != NULL);
    CHECK(strstr(s, "* rx.packet") == NULL);
    CHECK_STR(talk("x9 get count.lost rx.state\n"), "x9 count.lost=2\nx9 rx.state=search\nx9 ok\n");
    // Nothing being received: no rx.lost
    CHECK_STR(talk("y1 reset rx\n"), "y1 ok\n");
    fake_board_advance_frames(&fb, 2);
    CHECK_STR(drain(), "");
}

// A slow host: streams are dropped long before replies or events suffer,
// and seq still runs without a gap
static void test_stream_priority(void) {
    setup();
    CHECK_STR(talk("p1 sub stream=frame\np2 sub stream=band\np3 sub stream=spectrum units=power\n"),
        "p1 ok\np2 ok\np3 ok\n");
    fake_board_advance_frames(&fb, 200); // 4.6 s, and the host reads nothing
    const uint32_t queued = protocol_output_pending(&proto);
    printf("  200 frames unread: %lu bytes queued, %lu stream lines dropped\n",
        (unsigned long)queued, (unsigned long)proto.stream_dropped);
    CHECK(proto.stream_dropped > 0);
    CHECK(queued <= PROTOCOL_OUT_SIZE - PROTOCOL_REPLY_MAX - PROTOCOL_EVENT_RESERVE);

    // Events still get in, and a command is answered at once
    for (int i = 0; i < 3; i++) inject_packet(PROTOCOL_PAYLOAD_MAX);
    CHECK_EQ_U(proto.events_dropped, 0);
    const char *cmd = encode("p4 get count.stream_dropped count.events_dropped\n");
    CHECK_EQ_U(protocol_input(&proto, cmd, strlen(cmd)), strlen(cmd));

    const char *all = drain();
    CHECK(strstr(all, "p4 ok\n") != NULL);
    CHECK_EQ_U(count_char(all, '\n') > 0, 1);
    // seq: one more on every event line, frames: gaps where lines were dropped
    uint32_t last_seq = 0, last_frame = 0, frame_gaps = 0, packets = 0;
    bool seq_ok = true;
    for (const char *l = all; *l; l = strchr(l, '\n') + 1) {
        if (*l != '*') continue;
        unsigned long seq = 0, frame = 0;
        const char *q = strstr(l, " seq=");
        seq_ok = seq_ok && q && sscanf(q, " seq=%lu", &seq) == 1 && seq == last_seq + 1u;
        last_seq = (uint32_t)seq;
        if (strncmp(l, "* frame ", 8) == 0 && sscanf(strstr(l, " frame=") , " frame=%lu", &frame) == 1) {
            if (last_frame && frame != last_frame + 1u) frame_gaps++;
            last_frame = (uint32_t)frame;
        }
        if (strncmp(l, "* rx.packet ", 12) == 0) packets++;
    }
    CHECK(seq_ok);
    CHECK_EQ_U(frame_gaps, 0); // What got through came in order...
    CHECK_EQ_U(packets, 3);
    printf("  delivered seq 1..%lu without a gap; frame lines skipped frames %lu times\n",
        (unsigned long)last_seq, (unsigned long)frame_gaps);

    // ...and reading again, the streams flow again from the current frame:
    // the frame numbers show what was dropped, and seq carries straight on
    fake_board_advance_frames(&fb, 1);
    all = drain();
    unsigned long seq = 0, frame = 0;
    CHECK(sscanf(all, "* frame seq=%lu frame=%lu", &seq, &frame) == 2);
    CHECK_EQ_U(seq, last_seq + 1u);
    CHECK_EQ_U(frame, 201);
    printf("  frame lines resumed at frame %lu after frame %lu\n", frame, (unsigned long)last_frame);
    CHECK(strstr(all, "* spectrum seq=") != NULL);
}

// The host stops reading: events are dropped and counted, commands wait,
// and replies are never lost
static void test_flow(void) {
    enum { FLOOD = 300 }; // Enough longest-packet events to overflow the output queue
    setup();
    for (int i = 0; i < FLOOD; i++) inject_packet(PROTOCOL_PAYLOAD_MAX);
    const uint32_t queued = protocol_output_pending(&proto);
    printf("  %d long packet events: %lu bytes queued, %lu dropped\n",
        FLOOD, (unsigned long)queued, (unsigned long)proto.events_dropped);
    CHECK(proto.events_dropped > 0);
    CHECK(queued <= PROTOCOL_OUT_SIZE);

    // The first command is taken in but waits; the next isn't taken in yet
    static char cmds[128];
    strcpy(cmds, encode("f1 get count.events_dropped\nf2 hello\n"));
    const size_t first = (size_t)(strchr(cmds, '\n') + 1 - cmds);
    size_t used = protocol_input(&proto, cmds, strlen(cmds));
    CHECK_EQ_U(used, first);
    CHECK_EQ_U(protocol_input(&proto, cmds + used, strlen(cmds) - used), 0);

    // The host reads again, a little at a time, as a slow link would
    static char seen[65536];
    size_t total = 0;
    while (used < strlen(cmds) || protocol_output_pending(&proto) > 0) {
        total += protocol_output(&proto, seen + total, 700);
        protocol_poll(&proto);
        used += protocol_input(&proto, cmds + used, strlen(cmds) - used);
    }
    seen[total] = 0;
    decode(seen);
    strcpy(seen, out);
    char expect[64];
    snprintf(expect, sizeof expect, "f1 count.events_dropped=%lu\nf1 ok\n", (unsigned long)proto.events_dropped);
    CHECK(strstr(seen, expect) != NULL);
    CHECK(strstr(seen, "f2 ok\n") != NULL);
    // The notice follows the packets (seq 1..FLOOD); the dropped ones
    // left a gap in the sequence numbers just before it
    snprintf(expect, sizeof expect, "* overflow seq=%d dropped=%lu\n", FLOOD + 1, (unsigned long)proto.events_dropped);
    CHECK(strstr(seen, expect) != NULL);
    const uint32_t delivered = FLOOD - proto.events_dropped;
    snprintf(expect, sizeof expect, "* rx.packet seq=%lu ", (unsigned long)delivered);
    CHECK(strstr(seen, expect) != NULL);
    snprintf(expect, sizeof expect, "* rx.packet seq=%lu ", (unsigned long)delivered + 1u);
    CHECK(strstr(seen, expect) == NULL);
    // Every event line that got through is whole
    uint32_t packets = 0;
    for (const char *s = seen; (s = strstr(s, "* rx.packet")) != NULL; s++) {
        const char *nl = strchr(s, '\n');
        CHECK(nl && strstr(s, " hex=") && nl - strstr(s, " hex=") == 5 + 2 * PROTOCOL_PAYLOAD_MAX);
        packets++;
    }
    CHECK_EQ_U(packets + proto.events_dropped, FLOOD);

    // Afterwards events flow again, and reset counters clears the count
    inject_packet(1);
    char next[32];
    snprintf(next, sizeof next, "* rx.packet seq=%d ", FLOOD + 2);
    CHECK(strncmp(drain(), next, strlen(next)) == 0);
    CHECK_STR(talk("f3 reset counters\nf4 get count.events_dropped\n"), "f3 ok\nf4 count.events_dropped=0\nf4 ok\n");
}

int main(void) {
    RUN_TEST(test_start);
    RUN_TEST(test_get);
    RUN_TEST(test_list);
    RUN_TEST(test_set);
    RUN_TEST(test_send);
    RUN_TEST(test_symbols);
    RUN_TEST(test_ordering);
    RUN_TEST(test_abandoned);
    RUN_TEST(test_input);
    RUN_TEST(test_nudge);
    RUN_TEST(test_save_load);
    RUN_TEST(test_checks_on_lines);
    RUN_TEST(test_sub_commands);
    RUN_TEST(test_stream_lines);
    RUN_TEST(test_stream_priority);
    RUN_TEST(test_flow);
    printf("%d checks, %d failures\n", test_checks, test_failures);
    return test_failures != 0;
}
