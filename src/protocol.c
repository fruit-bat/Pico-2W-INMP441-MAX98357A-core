/**
 * @file protocol.c
 *
 * Host protocol engine; see protocol.h and protocol.md.
 *
 * Ordering rules, and how they are kept:
 * 1. Lines are whole: replies and events go into one byte queue as complete
 *    lines, and the queue is the only thing written to the port.
 * 2. Replies and events in no fixed order: events go in when they happen.
 * 3. A reply's lines stay together: a command's reply is built in a
 *    separate buffer and copied into the queue in one piece.
 * 4. A reply comes before anything it causes: the reply is queued before
 *    control returns to the board, which reports later happenings.
 * 5. Commands are answered in order: one line at a time, and a command is
 *    only run when the queue has room for the longest possible reply, so
 *    nothing is half-done when the queue is full (the line waits instead).
 */

#include "protocol.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BIT(id) (1u << (id))
#define MAX_ARGS 32u

// ---------------------------------------------------------------------------
// The settings table
// ---------------------------------------------------------------------------
static const char *const OFF_ON[] = { "off", "on", NULL };
static const char *const TRACKING[] = { "off", "decision", "pilot", NULL };
static const char *const MODES[] = { "loopback", "mic", "tone", "chirp", "sequence", "packet", NULL };
static const char *const RX_STATES[] = { "search", "lock", "train", "header", "payload", NULL };
static const int32_t SPACINGS[] = { 1, 2, 4 };

typedef struct {
    const char *name;
    const char *const *choices;   // NULL: an integer
    int32_t min, max;             // Integer range (test.symbol's max follows link.spacing)
    const int32_t *allowed;       // Optional: only these integers
    uint32_t allowed_count;
    int32_t def;
    const char *units;            // "" if none
    const char *help;
} SettingDef_t;

static const SettingDef_t SETTINGS[SETTING_COUNT] = {
    [SETTING_LINK_SPACING] = { "link.spacing", NULL, 1, 4, SPACINGS, 3, 4, "bins",
                               "Bins between symbols (4 bits/symbol at 4, 5 at 2, 6 at 1); both boards must match" },
    [SETTING_TX_VOLUME]    = { "tx.volume", NULL, -55, -1, NULL, 0, -28, "dBFS", "Transmit level" },
    [SETTING_RX_ALIGN]     = { "rx.align", OFF_ON, 0, 1, NULL, 0, 1, "",
                               "Move the receive window onto the earliest path when a preamble is found" },
    [SETTING_RX_TRACKING]  = { "rx.tracking", TRACKING, 0, 2, NULL, 0, 0, "", "Symbol timing tracking" },
    [SETTING_RX_DURING_TX] = { "rx.during_tx", OFF_ON, 0, 1, NULL, 0, 1, "",
                               "Receive while transmitting (a board hears its own packets)" },
    [SETTING_TEST_MODE]    = { "test.mode", MODES, 0, 5, NULL, 0, 5, "", "What the transmitter does" },
    [SETTING_TEST_SYMBOL]  = { "test.symbol", NULL, 0, (int32_t)PROTOCOL_CHIRP_BINS - 1, NULL, 0, 0, "",
                               "Symbol sent in chirp mode; below link.symbols" },
};

// ---------------------------------------------------------------------------
// The status table
// ---------------------------------------------------------------------------
typedef enum { ST_U32, ST_I32, ST_FLOAT } StatusType_t;

typedef struct {
    const char *name;
    StatusType_t type;
    size_t offset;
    const char *fmt;              // printf format for the value
    const char *const *choices;   // ST_U32 shown as a word
    const char *help;
} StatusDef_t;

#define S_U(n, f, h)     { n, ST_U32, offsetof(ProtocolStatus_t, f), "%lu", NULL, h }
#define S_I(n, f, h)     { n, ST_I32, offsetof(ProtocolStatus_t, f), "%+ld", NULL, h }
#define S_F(n, f, x, h)  { n, ST_FLOAT, offsetof(ProtocolStatus_t, f), x, NULL, h }
#define S_W(n, f, c, h)  { n, ST_U32, offsetof(ProtocolStatus_t, f), NULL, c, h }

static const StatusDef_t STATUS[] = {
    S_F("info.sample_rate", sample_rate, "%.1f", "Sample rate, Hz"),
    S_U("info.fft_size", fft_size, "Samples per frame"),
    S_U("info.bins", bins, "FFT bins the chirp spans"),
    S_F("info.f0", f0, "%.1f", "Chirp start frequency, Hz"),
    S_F("info.bandwidth", bandwidth, "%.2f", "Chirp bandwidth, Hz"),
    S_F("info.highpass_hz", highpass_hz, "%g", "Microphone high-pass corner, Hz"),
    S_F("info.track_alpha", track_alpha, "%g", "Timing tracker gain"),
    S_F("info.track_beta", track_beta, "%g", "Timing tracker drift gain"),
    S_F("info.track_max_rate", track_max_rate, "%g", "Timing tracker drift limit, samples/frame"),
    S_F("info.track_rate_window", track_rate_window, "%g", "Timing tracker learns drift within this error, samples"),
    S_F("info.track_min_magnitude", track_min_magnitude, "%g", "Timing tracker ignores weaker frames"),
    S_U("link.symbols", link_symbols, "Symbols at this spacing"),
    S_U("link.bits", link_bits, "Bits per symbol at this spacing"),
    S_U("count.frames", frames, "Frames processed"),
    S_U("count.rx_missed", rx_missed, "Receive audio blocks processed late"),
    S_U("count.tx_missed", tx_missed, "Transmit audio blocks filled late"),
    S_U("count.records_dropped", records_dropped, "Per-frame records lost between the cores"),
    S_U("count.good", good, "Packets received with a good CRC"),
    S_U("count.bad", bad, "Packets received with a bad CRC"),
    S_U("count.header_failed", header_failed, "Headers with a bad CRC-8"),
    S_U("count.lost", lost, "Packets abandoned when the signal went"),
    S_U("count.rx_dropped", rx_dropped, "Received packets lost between the cores"),
    S_U("count.events_dropped", events_dropped, "Events dropped because the host was not reading"),
    S_U("tx.queued", tx_queued, "Packets waiting to be sent"),
    S_W("rx.state", rx_state, RX_STATES, "Packet receiver state"),
    S_U("rx.delay", rx_delay, "Receive window delay, samples (moved by alignment, tracking and nudge)"),
    S_I("rx.last_lock.align", lock_align, "Last lock: whole bins the window was late"),
    S_F("rx.last_lock.fraction", lock_fraction, "%+.3f", "Last lock: and this fraction of a bin"),
    S_I("rx.last_lock.moved", lock_moved, "Last lock: samples the window was moved"),
    S_U("rx.last_train.frame", train_frame, "Last training: frame where B started"),
    S_F("rx.last_train.coherence", train_coherence, "%.3f", "Last training: how well B's frames agreed (0..1)"),
    S_I("rx.last_train.strongest", train_strongest, "Last training: strongest path, bins"),
    S_I("rx.last_train.earliest", train_earliest, "Last training: earliest strong path, bins"),
};
#define STATUS_COUNT (sizeof STATUS / sizeof STATUS[0])

// ---------------------------------------------------------------------------
// Settings helpers
// ---------------------------------------------------------------------------
void protocol_default_settings(ProtocolSettings_t *s) {
    for (uint32_t i = 0; i < SETTING_COUNT; i++) s->value[i] = SETTINGS[i].def;
}

const ProtocolSettings_t *protocol_settings(const Protocol_t *p) {
    return &p->settings;
}

static uint32_t symbols_for(int32_t spacing) {
    return PROTOCOL_CHIRP_BINS / (uint32_t)spacing;
}

static int32_t setting_max(const ProtocolSettings_t *s, uint32_t id) {
    if (id == SETTING_TEST_SYMBOL) return (int32_t)symbols_for(s->value[SETTING_LINK_SPACING]) - 1;
    return SETTINGS[id].max;
}

static bool setting_valid(const ProtocolSettings_t *s, uint32_t id) {
    const SettingDef_t *d = &SETTINGS[id];
    const int32_t v = s->value[id];
    if (v < d->min || v > setting_max(s, id)) return false;
    if (d->allowed) {
        for (uint32_t i = 0; i < d->allowed_count; i++) {
            if (d->allowed[i] == v) return true;
        }
        return false;
    }
    return true;
}

static bool settings_valid(const ProtocolSettings_t *s) {
    // Spacing first: test.symbol's range depends on it
    if (!setting_valid(s, SETTING_LINK_SPACING)) return false;
    for (uint32_t i = 0; i < SETTING_COUNT; i++) {
        if (!setting_valid(s, i)) return false;
    }
    return true;
}

static uint32_t settings_diff(const ProtocolSettings_t *a, const ProtocolSettings_t *b) {
    uint32_t changed = 0;
    for (uint32_t i = 0; i < SETTING_COUNT; i++) {
        if (a->value[i] != b->value[i]) changed |= BIT(i);
    }
    return changed;
}

static void apply_settings(Protocol_t *p, const ProtocolSettings_t *s) {
    const uint32_t changed = settings_diff(&p->settings, s);
    p->settings = *s;
    if (changed && p->board->apply) p->board->apply(p->board->ctx, &p->settings, changed);
}

// ---------------------------------------------------------------------------
// Output queue
// ---------------------------------------------------------------------------
static uint32_t out_free(const Protocol_t *p) {
    return PROTOCOL_OUT_SIZE - (p->out_head - p->out_tail);
}

uint32_t protocol_output_pending(const Protocol_t *p) {
    return p->out_head - p->out_tail;
}

static void out_put(Protocol_t *p, const char *data, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        p->out[p->out_head++ % PROTOCOL_OUT_SIZE] = data[i];
    }
}

size_t protocol_output(Protocol_t *p, char *buf, size_t max) {
    size_t n = 0;
    while (n < max && p->out_tail != p->out_head) {
        buf[n++] = p->out[p->out_tail++ % PROTOCOL_OUT_SIZE];
    }
    return n;
}

// ---------------------------------------------------------------------------
// Building a reply
// ---------------------------------------------------------------------------
static void reply_vadd(Protocol_t *p, const char *fmt, va_list ap) {
    if (p->reply_overflow) return;
    const size_t room = PROTOCOL_REPLY_MAX - p->reply_len;
    const int n = vsnprintf(p->reply + p->reply_len, room, fmt, ap);
    if (n < 0 || (size_t)n >= room) {
        p->reply_overflow = true;
    } else {
        p->reply_len += (uint32_t)n;
    }
}

static void reply_add(Protocol_t *p, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    reply_vadd(p, fmt, ap);
    va_end(ap);
}

// One whole reply line: "<tag> ...\n"
static void reply_line(Protocol_t *p, const char *tag, const char *fmt, ...) {
    reply_add(p, "%s ", tag);
    va_list ap;
    va_start(ap, fmt);
    reply_vadd(p, fmt, ap);
    va_end(ap);
    reply_add(p, "\n");
}

// Replaces anything built so far with "<tag> err <code> <text>"
static void reply_err(Protocol_t *p, const char *tag, int code, const char *fmt, ...) {
    p->reply_len = 0;
    p->reply_overflow = false;
    reply_add(p, "%s err %d ", tag, code);
    va_list ap;
    va_start(ap, fmt);
    reply_vadd(p, fmt, ap);
    va_end(ap);
    reply_add(p, "\n");
}

static void reply_ok(Protocol_t *p, const char *tag) {
    reply_line(p, tag, "ok");
}

// ---------------------------------------------------------------------------
// Values
// ---------------------------------------------------------------------------
static void add_setting_value(Protocol_t *p, uint32_t id, int32_t v) {
    const SettingDef_t *d = &SETTINGS[id];
    if (d->choices) reply_add(p, "%s", d->choices[v]);
    else reply_add(p, "%ld", (long)v);
}

static void add_status_value(Protocol_t *p, const StatusDef_t *d, const ProtocolStatus_t *st) {
    const void *f = (const uint8_t *)st + d->offset;
    switch (d->type) {
        case ST_U32: {
            const uint32_t v = *(const uint32_t *)f;
            if (d->choices) reply_add(p, "%s", d->choices[v]);
            else reply_add(p, d->fmt, (unsigned long)v);
            break;
        }
        case ST_I32: reply_add(p, d->fmt, (long)*(const int32_t *)f); break;
        case ST_FLOAT: reply_add(p, d->fmt, (double)*(const float *)f); break;
    }
}

static bool parse_int(const char *s, int32_t *out) {
    if (*s == 0) return false;
    char *end;
    const long v = strtol(s, &end, 10);
    if (*end != 0 || v < INT32_MIN || v > INT32_MAX) return false;
    *out = (int32_t)v;
    return true;
}

static int choice_index(const char *const *choices, const char *word) {
    for (int i = 0; choices[i]; i++) {
        if (strcmp(choices[i], word) == 0) return i;
    }
    return -1;
}

static void fill_status(Protocol_t *p, ProtocolStatus_t *st) {
    memset(st, 0, sizeof *st);
    if (p->board->get_status) p->board->get_status(p->board->ctx, st);
    st->link_symbols = symbols_for(p->settings.value[SETTING_LINK_SPACING]);
    st->link_bits = (uint32_t)__builtin_ctz(st->link_symbols);
    st->events_dropped = p->events_dropped;
}

// "name" matches itself; "prefix.*" and "*" match by prefix
static bool name_matches(const char *pattern, const char *name) {
    const size_t n = strlen(pattern);
    if (n > 0 && pattern[n - 1] == '*') return strncmp(pattern, name, n - 1) == 0;
    return strcmp(pattern, name) == 0;
}

static int find_setting(const char *name) {
    for (uint32_t i = 0; i < SETTING_COUNT; i++) {
        if (strcmp(SETTINGS[i].name, name) == 0) return (int)i;
    }
    return -1;
}

static int find_status(const char *name) {
    for (uint32_t i = 0; i < STATUS_COUNT; i++) {
        if (strcmp(STATUS[i].name, name) == 0) return (int)i;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
typedef void (*Handler_t)(Protocol_t *p, const char *tag, uint32_t argc, char **argv);

static void cmd_hello(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    (void)argv;
    if (argc != 0) { reply_err(p, tag, PROTO_ERR_ARGUMENT, "hello takes no arguments"); return; }
    reply_line(p, tag, "proto=%d version=%s id=%s", PROTOCOL_VERSION, p->board->version, p->board->board_id);
    reply_ok(p, tag);
}

// get and list: the items named by argv (all if none), in table order
static void get_or_list(Protocol_t *p, const char *tag, uint32_t argc, char **argv, bool list) {
    static char *const all[] = { "*" };
    if (argc == 0) { argc = 1; argv = (char **)all; }

    // Every pattern must match something
    for (uint32_t a = 0; a < argc; a++) {
        bool any = false;
        for (uint32_t i = 0; i < SETTING_COUNT && !any; i++) any = name_matches(argv[a], SETTINGS[i].name);
        for (uint32_t i = 0; i < STATUS_COUNT && !any; i++) any = name_matches(argv[a], STATUS[i].name);
        if (!any) { reply_err(p, tag, PROTO_ERR_NAME, "unknown name %s", argv[a]); return; }
    }

    ProtocolStatus_t st;
    fill_status(p, &st);
    for (uint32_t a = 0; a < argc; a++) {
        for (uint32_t i = 0; i < SETTING_COUNT; i++) {
            const SettingDef_t *d = &SETTINGS[i];
            if (!name_matches(argv[a], d->name)) continue;
            if (!list) {
                reply_add(p, "%s %s=", tag, d->name);
                add_setting_value(p, i, p->settings.value[i]);
                reply_add(p, "\n");
                continue;
            }
            reply_add(p, "%s name=%s kind=setting", tag, d->name);
            if (d->choices) {
                reply_add(p, " type=choice values=");
                for (uint32_t c = 0; d->choices[c]; c++) reply_add(p, "%s%s", c ? "," : "", d->choices[c]);
            } else if (d->allowed) {
                reply_add(p, " type=int values=");
                for (uint32_t c = 0; c < d->allowed_count; c++) reply_add(p, "%s%ld", c ? "," : "", (long)d->allowed[c]);
            } else {
                reply_add(p, " type=int range=%ld..%ld", (long)d->min, (long)d->max);
            }
            reply_add(p, " default=");
            add_setting_value(p, i, d->def);
            if (d->units[0]) reply_add(p, " units=%s", d->units);
            reply_add(p, " help=%s\n", d->help);
        }
        for (uint32_t i = 0; i < STATUS_COUNT; i++) {
            const StatusDef_t *d = &STATUS[i];
            if (!name_matches(argv[a], d->name)) continue;
            if (!list) {
                reply_add(p, "%s %s=", tag, d->name);
                add_status_value(p, d, &st);
                reply_add(p, "\n");
            } else {
                reply_add(p, "%s name=%s kind=status type=%s help=%s\n", tag, d->name,
                    d->choices ? "choice" : d->type == ST_FLOAT ? "real" : "int", d->help);
            }
        }
    }
    reply_ok(p, tag);
}

static void cmd_get(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    get_or_list(p, tag, argc, argv, false);
}

static void cmd_list(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    get_or_list(p, tag, argc, argv, true);
}

static void range_err(Protocol_t *p, const char *tag, const ProtocolSettings_t *s, uint32_t id) {
    const SettingDef_t *d = &SETTINGS[id];
    if (d->choices) {
        reply_err(p, tag, PROTO_ERR_RANGE, "%s must be one of", d->name);
        p->reply_len--; // Before the newline
        for (uint32_t c = 0; d->choices[c]; c++) reply_add(p, "%s%s", c ? "," : " ", d->choices[c]);
        reply_add(p, "\n");
    } else if (d->allowed) {
        reply_err(p, tag, PROTO_ERR_RANGE, "%s must be one of", d->name);
        p->reply_len--;
        for (uint32_t c = 0; c < d->allowed_count; c++) reply_add(p, "%s%ld", c ? "," : " ", (long)d->allowed[c]);
        reply_add(p, "\n");
    } else {
        reply_err(p, tag, PROTO_ERR_RANGE, "%s out of range %ld..%ld", d->name, (long)d->min, (long)setting_max(s, id));
    }
}

static void cmd_set(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    if (argc == 0) { reply_err(p, tag, PROTO_ERR_ARGUMENT, "set needs name=value"); return; }
    ProtocolSettings_t s = p->settings;
    uint32_t assigned = 0;
    for (uint32_t a = 0; a < argc; a++) {
        char *eq = strchr(argv[a], '=');
        if (!eq) { reply_err(p, tag, PROTO_ERR_ARGUMENT, "expected name=value, not %s", argv[a]); return; }
        *eq = 0;
        const char *name = argv[a], *value = eq + 1;
        const int id = find_setting(name);
        if (id < 0) {
            if (find_status(name) >= 0) reply_err(p, tag, PROTO_ERR_READ_ONLY, "%s is read only", name);
            else reply_err(p, tag, PROTO_ERR_NAME, "unknown setting %s", name);
            return;
        }
        if (assigned & BIT(id)) { reply_err(p, tag, PROTO_ERR_ARGUMENT, "%s given twice", name); return; }
        assigned |= BIT(id);
        int32_t v;
        if (SETTINGS[id].choices) {
            v = choice_index(SETTINGS[id].choices, value);
            if (v < 0) { range_err(p, tag, &s, (uint32_t)id); return; }
        } else if (!parse_int(value, &v)) {
            reply_err(p, tag, PROTO_ERR_RANGE, "%s needs an integer, not '%s'", name, value);
            return;
        }
        s.value[id] = v;
    }
    // Check against the final values (test.symbol's range follows link.spacing)
    if (!setting_valid(&s, SETTING_LINK_SPACING)) { range_err(p, tag, &s, SETTING_LINK_SPACING); return; }
    for (uint32_t i = 0; i < SETTING_COUNT; i++) {
        if ((assigned & BIT(i)) && !setting_valid(&s, i)) { range_err(p, tag, &s, i); return; }
    }
    // Side effects: a symbol no longer in range wraps (as key 'b' did)
    if (!(assigned & BIT(SETTING_TEST_SYMBOL))) {
        const int32_t n = (int32_t)symbols_for(s.value[SETTING_LINK_SPACING]);
        s.value[SETTING_TEST_SYMBOL] %= n;
    }
    const uint32_t side = settings_diff(&p->settings, &s) & ~assigned;
    apply_settings(p, &s);

    reply_add(p, "%s ok", tag);
    for (uint32_t i = 0; i < SETTING_COUNT; i++) {
        if (!(side & BIT(i))) continue;
        reply_add(p, " %s=", SETTINGS[i].name);
        add_setting_value(p, i, s.value[i]);
    }
    reply_add(p, "\n");
}

static int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void cmd_send(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    if (argc != 1 || strncmp(argv[0], "hex=", 4) != 0) {
        reply_err(p, tag, PROTO_ERR_ARGUMENT, "send takes hex=<bytes>");
        return;
    }
    const char *hex = argv[0] + 4;
    const size_t n = strlen(hex);
    if (n % 2 != 0) { reply_err(p, tag, PROTO_ERR_HEX, "odd number of hex digits"); return; }
    if (n / 2 > PROTOCOL_PAYLOAD_MAX) { reply_err(p, tag, PROTO_ERR_HEX, "more than %u bytes", PROTOCOL_PAYLOAD_MAX); return; }
    uint8_t payload[PROTOCOL_PAYLOAD_MAX];
    for (size_t i = 0; i < n / 2; i++) {
        const int hi = hex_digit(hex[2 * i]), lo = hex_digit(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) { reply_err(p, tag, PROTO_ERR_HEX, "not hex"); return; }
        payload[i] = (uint8_t)(hi << 4 | lo);
    }
    const uint32_t id = p->next_id;
    const int r = p->board->send ? p->board->send(p->board->ctx, payload, (uint32_t)(n / 2), id) : PROTO_ERR_NOT_YET;
    if (r == PROTO_ERR_QUEUE_FULL) { reply_err(p, tag, r, "transmit queue full"); return; }
    if (r != 0) { reply_err(p, tag, r, "cannot send"); return; }
    p->next_id++;
    reply_line(p, tag, "ok id=%lu", (unsigned long)id);
}

static void cmd_nudge(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    int32_t samples;
    if (argc != 1 || strncmp(argv[0], "samples=", 8) != 0) {
        reply_err(p, tag, PROTO_ERR_ARGUMENT, "nudge takes samples=<n>");
        return;
    }
    if (!parse_int(argv[0] + 8, &samples) || samples < -1023 || samples > 1023) {
        reply_err(p, tag, PROTO_ERR_RANGE, "samples out of range -1023..1023");
        return;
    }
    if (!p->board->nudge) { reply_err(p, tag, PROTO_ERR_NOT_YET, "nudge not available"); return; }
    const uint32_t delay = p->board->nudge(p->board->ctx, samples);
    reply_line(p, tag, "ok delay=%lu", (unsigned long)delay);
}

static void cmd_reset(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    if (argc != 1) { reply_err(p, tag, PROTO_ERR_ARGUMENT, "reset takes counters, rx or settings"); return; }
    if (strcmp(argv[0], "counters") == 0) {
        if (p->board->reset_counters) p->board->reset_counters(p->board->ctx);
        p->events_dropped = 0;
    } else if (strcmp(argv[0], "rx") == 0) {
        if (p->board->reset_rx) p->board->reset_rx(p->board->ctx);
    } else if (strcmp(argv[0], "settings") == 0) {
        ProtocolSettings_t s;
        protocol_default_settings(&s);
        apply_settings(p, &s);
    } else {
        reply_err(p, tag, PROTO_ERR_RANGE, "reset takes counters, rx or settings");
        return;
    }
    reply_ok(p, tag);
}

static void cmd_save(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    (void)argv;
    if (argc != 0) { reply_err(p, tag, PROTO_ERR_ARGUMENT, "save takes no arguments"); return; }
    if (!p->board->save || !p->board->save(p->board->ctx, &p->settings)) {
        reply_err(p, tag, PROTO_ERR_NOT_YET, "saving settings is not available yet");
        return;
    }
    reply_ok(p, tag);
}

// The saved settings if there are any (and they are valid), else the defaults
static void load_settings(Protocol_t *p, ProtocolSettings_t *s) {
    if (!p->board->load || !p->board->load(p->board->ctx, s) || !settings_valid(s)) {
        protocol_default_settings(s);
    }
}

static void cmd_load(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    (void)argv;
    if (argc != 0) { reply_err(p, tag, PROTO_ERR_ARGUMENT, "load takes no arguments"); return; }
    ProtocolSettings_t s;
    load_settings(p, &s);
    apply_settings(p, &s);
    reply_ok(p, tag);
}

static void cmd_help(Protocol_t *p, const char *tag, uint32_t argc, char **argv);

static const struct {
    const char *name;
    Handler_t handler;
    const char *help;
} COMMANDS[] = {
    { "hello", cmd_hello, "hello: protocol version, firmware version and board id" },
    { "get",   cmd_get,   "get [name|prefix.* ...]: settings and status (all if none named)" },
    { "set",   cmd_set,   "set name=value ...: change settings (all or nothing); ok lists side effects" },
    { "list",  cmd_list,  "list [name|prefix.* ...]: describe settings and status" },
    { "send",  cmd_send,  "send hex=<bytes>: queue a packet; ok id=<n>, then * tx.start and * tx.done" },
    { "nudge", cmd_nudge, "nudge samples=<n>: move the receive window (positive = later)" },
    { "reset", cmd_reset, "reset counters|rx|settings: clear counters, abandon reception, or restore defaults" },
    { "save",  cmd_save,  "save: keep the current settings for start-up" },
    { "load",  cmd_load,  "load: go back to the saved settings (the defaults if none)" },
    { "help",  cmd_help,  "help: this list" },
};
#define COMMAND_COUNT (sizeof COMMANDS / sizeof COMMANDS[0])

static void cmd_help(Protocol_t *p, const char *tag, uint32_t argc, char **argv) {
    (void)argc; (void)argv;
    for (uint32_t i = 0; i < COMMAND_COUNT; i++) {
        reply_line(p, tag, "command=%s help=%s", COMMANDS[i].name, COMMANDS[i].help);
    }
    reply_ok(p, tag);
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------
static bool valid_tag(const char *t) {
    const size_t n = strlen(t);
    if (n == 0 || n > PROTOCOL_TAG_MAX) return false;
    for (size_t i = 0; i < n; i++) {
        const char c = t[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'))) return false;
    }
    return true;
}

static void reset_line(Protocol_t *p) {
    p->line_len = 0;
    p->line_too_long = false;
    p->line_held = false;
}

// Runs the complete line in p->line. False if there was no room for the
// reply yet: the line is held and nothing has been done.
static bool run_line(Protocol_t *p) {
    if (out_free(p) < PROTOCOL_REPLY_MAX) {
        p->line_held = true;
        return false;
    }
    p->line[p->line_len] = 0;
    if (p->line_len > 0 && p->line[p->line_len - 1] == '\r') p->line[--p->line_len] = 0;

    p->reply_len = 0;
    p->reply_overflow = false;

    // Split into words (runs of spaces count as one)
    char *argv[MAX_ARGS + 2];
    uint32_t argc = 0;
    bool too_many = false, bad_char = false;
    for (uint32_t i = 0; i < p->line_len; i++) {
        const char c = p->line[i];
        if (c < 0x20 || c > 0x7E) bad_char = true;
    }
    for (char *s = p->line; *s;) {
        while (*s == ' ') *s++ = 0;
        if (!*s) break;
        if (argc == MAX_ARGS + 2) { too_many = true; break; }
        argv[argc++] = s;
        while (*s && *s != ' ') s++;
    }

    if (argc == 0 && !p->line_too_long) {
        reset_line(p); // Empty line: ignored
        return true;
    }
    if (argc == 0 || !valid_tag(argv[0])) {
        reply_add(p, "* err %d line has no valid tag\n", PROTO_ERR_SYNTAX);
    } else if (p->line_too_long) {
        reply_err(p, argv[0], PROTO_ERR_SYNTAX, "line longer than %u characters", PROTOCOL_LINE_MAX);
    } else if (bad_char) {
        reply_err(p, argv[0], PROTO_ERR_SYNTAX, "characters other than printable ASCII");
    } else if (argc == 1) {
        reply_err(p, argv[0], PROTO_ERR_COMMAND, "no command");
    } else if (too_many) {
        reply_err(p, argv[0], PROTO_ERR_ARGUMENT, "more than %u arguments", MAX_ARGS);
    } else {
        uint32_t c = 0;
        while (c < COMMAND_COUNT && strcmp(COMMANDS[c].name, argv[1]) != 0) c++;
        if (c == COMMAND_COUNT) {
            reply_err(p, argv[0], PROTO_ERR_COMMAND, "unknown command %s", argv[1]);
        } else {
            char tag[PROTOCOL_TAG_MAX + 1];
            strcpy(tag, argv[0]);
            COMMANDS[c].handler(p, tag, argc - 2, argv + 2);
            if (p->reply_overflow) reply_err(p, tag, PROTO_ERR_SYNTAX, "reply too long; ask for less");
        }
    }
    out_put(p, p->reply, p->reply_len);
    reset_line(p);
    return true;
}

size_t protocol_input(Protocol_t *p, const char *data, size_t length) {
    if (p->line_held && !run_line(p)) return 0;
    size_t used = 0;
    while (used < length) {
        const char c = data[used++];
        if (c == '\n') {
            if (!run_line(p)) return used; // Held: offer the rest later
            continue;
        }
        if (p->line_len < PROTOCOL_LINE_MAX) p->line[p->line_len++] = c;
        else p->line_too_long = true;
    }
    return used;
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------
// Reports dropped events once there is room again: for the notice and the
// longest event after it (so one overflow gives one notice, not a notice
// squeezed in between drops)
static void report_overflow(Protocol_t *p) {
    if (p->events_dropped_pending == 0) return;
    char line[48];
    const int n = snprintf(line, sizeof line, "* overflow dropped=%lu\n", (unsigned long)p->events_dropped_pending);
    if (out_free(p) >= (uint32_t)n + PROTOCOL_LINE_MAX + 1u) {
        out_put(p, line, (uint32_t)n);
        p->events_dropped_pending = 0;
    }
}

// Queues a complete event line, or drops (and counts) it
static void emit(Protocol_t *p, const char *line, uint32_t n) {
    report_overflow(p);
    if (p->events_dropped_pending == 0 && out_free(p) >= n) {
        out_put(p, line, n);
    } else {
        p->events_dropped++;
        p->events_dropped_pending++;
    }
}

void protocol_event(Protocol_t *p, const ProtocolEvent_t *e) {
    static const char HEX[] = "0123456789abcdef";
    char *line = p->event_line;
    const size_t size = sizeof p->event_line;
    const unsigned long frame = (unsigned long)e->frame;
    int n = 0;
    switch (e->type) {
        case PROTO_EV_RX_LOCKED:
            n = snprintf(line, size, "* rx.locked frame=%lu align=%+ld fraction=%+.3f moved=%+ld\n", frame,
                (long)e->u.locked.align, (double)e->u.locked.fraction, (long)e->u.locked.moved);
            break;
        case PROTO_EV_RX_TRAINED:
            n = snprintf(line, size, "* rx.trained frame=%lu b_start=%lu coherence=%.3f strongest=%+ld earliest=%+ld\n",
                frame, (unsigned long)e->u.trained.b_start, (double)e->u.trained.coherence,
                (long)e->u.trained.strongest, (long)e->u.trained.earliest);
            break;
        case PROTO_EV_RX_TRAIN_FAILED:
            n = snprintf(line, size, "* rx.train_failed frame=%lu\n", frame);
            break;
        case PROTO_EV_RX_TONE:
            n = snprintf(line, size, "* rx.tone frame=%lu\n", frame);
            break;
        case PROTO_EV_RX_HEADER_FAILED:
            n = snprintf(line, size, "* rx.header_failed frame=%lu length=%lu\n", frame,
                (unsigned long)e->u.header_failed.length);
            break;
        case PROTO_EV_RX_LOST:
            n = snprintf(line, size, "* rx.lost frame=%lu symbols=%lu\n", frame, (unsigned long)e->u.lost.symbols);
            break;
        case PROTO_EV_RX_PACKET: {
            uint32_t len = e->u.packet.length;
            if (len > PROTOCOL_PAYLOAD_MAX) len = PROTOCOL_PAYLOAD_MAX;
            n = snprintf(line, size, "* rx.packet frame=%lu crc=%s len=%lu min_margin=%.3f weakest=%lu weak=%lu hex=",
                frame, e->u.packet.crc_ok ? "ok" : "bad", (unsigned long)len, (double)e->u.packet.min_margin,
                (unsigned long)e->u.packet.weakest, (unsigned long)e->u.packet.weak);
            for (uint32_t i = 0; i < len; i++) {
                line[n++] = HEX[e->u.packet.payload[i] >> 4];
                line[n++] = HEX[e->u.packet.payload[i] & 15];
            }
            line[n++] = '\n';
            break;
        }
        case PROTO_EV_TX_START:
        case PROTO_EV_TX_DONE:
            n = snprintf(line, size, "* tx.%s frame=%lu id=%lu\n", e->type == PROTO_EV_TX_START ? "start" : "done",
                frame, (unsigned long)e->u.tx.id);
            break;
    }
    if (n > 0) emit(p, line, (uint32_t)n);
}

void protocol_poll(Protocol_t *p) {
    if (p->line_held) protocol_input(p, NULL, 0);
    report_overflow(p);
}

// ---------------------------------------------------------------------------
// Start-up
// ---------------------------------------------------------------------------
void protocol_init(Protocol_t *p, const ProtocolBoard_t *board) {
    memset(p, 0, sizeof *p);
    p->board = board;
    p->next_id = 1;
    protocol_default_settings(&p->settings);
}

void protocol_start(Protocol_t *p) {
    ProtocolSettings_t s;
    load_settings(p, &s);
    p->settings = s;
    if (p->board->apply) p->board->apply(p->board->ctx, &p->settings, BIT(SETTING_COUNT) - 1u);
    char line[160];
    const int n = snprintf(line, sizeof line, "* hello proto=%d version=%s id=%s\n",
        PROTOCOL_VERSION, p->board->version, p->board->board_id);
    emit(p, line, (uint32_t)n);
}
