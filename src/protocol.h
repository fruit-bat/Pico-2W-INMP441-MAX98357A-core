#ifndef PROTOCOL_H
#define PROTOCOL_H

/**
 * @file protocol.h
 *
 * Host protocol engine (see protocol.md): tagged text commands in, tagged
 * replies and '*' events out, over any byte stream.
 *
 * Plain C, no Pico calls and no dynamic memory, so the same code runs on the
 * board (core 0) and in the PC tests. Everything the protocol needs from the
 * board goes through ProtocolBoard_t; the board reports what happens
 * (packets, locks, transmissions) with protocol_event().
 *
 * Use, from one thread:
 *   protocol_init(), then protocol_start() (sends "* hello")
 *   loop:
 *     n = protocol_input(p, bytes, len)   // bytes from the host; n consumed
 *     protocol_event(p, &event)           // whenever something happens
 *     protocol_poll(p)                    // retry a held command / overflow notice
 *     m = protocol_output(p, buf, max)    // bytes for the host
 *
 * Every line both ways starts with a check: "XXXX " (CRC-16 of the rest of
 * the line, in hex, then a space). A line from the host that fails it is
 * answered with an event, "* err seq=<n> 500 line failed its check".
 *
 * protocol_input() may consume fewer bytes than offered: it holds a command
 * until its whole reply fits in the output queue (replies are never
 * dropped), and the caller offers the rest again later.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_VERSION 1
#define PROTOCOL_LINE_MAX 1024u    // Longest line either way, with its check, without the LF
#define PROTOCOL_CHECK_LEN 5u      // "XXXX ": CRC-16 in hex and a space, at the start of every line
#define PROTOCOL_OUT_LINE_MAX 6144u // Longest line from the board, with its check (a 512-bin spectrum as raw power)
#define PROTOCOL_OUT_SIZE 24576u   // Output queue, bytes
#define PROTOCOL_REPLY_MAX 6144u   // Longest reply to one command, bytes, with its checks ("list" of everything is about 4 KB)
#define PROTOCOL_TAG_MAX 8u
#define PROTOCOL_PAYLOAD_MAX 255u  // Bytes in a packet (as packet_format.h)
#define PROTOCOL_CHIRP_BINS 64u    // FFT bins the chirp spans (CHIRP_BINS in main.c)
#define PROTOCOL_FFT_SIZE 1024u    // Bins in the spectrum stream's source
#define PROTOCOL_SPECTRUM_MAX 512u // Most spectrum bins one subscription may ask for
#define PROTOCOL_EVENT_RESERVE 2048u // Queue space streams leave for events (beyond a reply's worth)

// Error codes (protocol.md, "Error codes")
enum {
    PROTO_ERR_QUEUE_FULL   = 450,
    PROTO_ERR_BUSY         = 451,
    PROTO_ERR_NOT_YET      = 452,
    PROTO_ERR_SYNTAX       = 500,
    PROTO_ERR_COMMAND      = 501,
    PROTO_ERR_NAME         = 502,
    PROTO_ERR_RANGE        = 503,
    PROTO_ERR_READ_ONLY    = 504,
    PROTO_ERR_ARGUMENT     = 505,
    PROTO_ERR_HEX          = 506,
    PROTO_ERR_STREAM       = 507,
};

// ---------------------------------------------------------------------------
// Settings: one table (protocol.c) gives names, ranges, defaults and help.
// Values are integers; a choice is stored as the index of its word.
// ---------------------------------------------------------------------------
typedef enum {
    SETTING_LINK_SPACING = 0, // 1, 2, 4 bins per symbol
    SETTING_TX_VOLUME,        // dBFS
    SETTING_RX_ALIGN,         // off, on
    SETTING_RX_TRACKING,      // off, decision, pilot (as enum tracking_mode in main.c)
    SETTING_RX_DURING_TX,     // off, on
    SETTING_TEST_MODE,        // loopback, mic, tone, chirp, sequence, packet (as enum test_mode)
    SETTING_TEST_SYMBOL,      // 0 .. symbols-1
    SETTING_COUNT
} ProtocolSettingId_t;

typedef struct {
    int32_t value[SETTING_COUNT];
} ProtocolSettings_t;

// The receiver states, as PacketRxState_t
enum { PROTO_RX_SEARCH, PROTO_RX_LOCK, PROTO_RX_TRAIN, PROTO_RX_HEADER, PROTO_RX_PAYLOAD };

// Read-only status, filled in by the board on request. The protocol fills
// link_symbols, link_bits (from the settings) and events_dropped itself.
typedef struct {
    // Constants (info.*)
    float sample_rate, f0, bandwidth, highpass_hz;
    uint32_t fft_size, bins;
    float track_alpha, track_beta, track_max_rate, track_rate_window, track_min_magnitude;
    // From the settings
    uint32_t link_symbols, link_bits;
    // Counters (count.*)
    uint32_t frames, rx_missed, tx_missed, records_dropped;
    uint32_t good, bad, header_failed, lost, rx_dropped, events_dropped, link_errors, stream_dropped;
    // Now
    uint32_t tx_queued;
    uint32_t rx_state;  // PROTO_RX_*
    uint32_t rx_delay;  // Samples
    // rx.last_lock.*
    int32_t lock_align;
    float lock_fraction;
    int32_t lock_moved;
    // rx.last_train.*
    uint32_t train_frame;
    float train_coherence;
    int32_t train_strongest, train_earliest;
} ProtocolStatus_t;

// ---------------------------------------------------------------------------
// Events the board reports
// ---------------------------------------------------------------------------
typedef enum {
    PROTO_EV_RX_LOCKED,
    PROTO_EV_RX_TRAINED,
    PROTO_EV_RX_TRAIN_FAILED,
    PROTO_EV_RX_TONE,
    PROTO_EV_RX_HEADER_FAILED,
    PROTO_EV_RX_LOST,
    PROTO_EV_RX_PACKET,
    PROTO_EV_TX_START,
    PROTO_EV_TX_DONE,
} ProtocolEventType_t;

typedef struct {
    ProtocolEventType_t type;
    uint32_t frame;
    union {
        struct { int32_t align; float fraction; int32_t moved; } locked;
        struct { uint32_t b_start; float coherence; int32_t strongest, earliest; } trained;
        struct { uint32_t length; } header_failed;
        struct { uint32_t symbols; } lost;
        struct {
            bool crc_ok;
            uint32_t length;
            float min_margin;
            uint32_t weakest, weak;
            const uint8_t *payload; // Only needs to last for the protocol_event() call
        } packet;
        struct { uint32_t id; } tx; // TX_START, TX_DONE
    } u;
} ProtocolEvent_t;

// ---------------------------------------------------------------------------
// One received frame, for the streams (protocol_frame). Arrays are power
// (|X|^2) and need only last for the call; NULL if not available.
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t frame, tx_symbol, delay, rx_symbol;
    float angle, error_angle, magnitude, strength, peak, timing_error, timing_rate;
    uint32_t state;              // PROTO_RX_*
    float corr;
    uint32_t data_symbol;
    float data_margin, data_quality;
    const float *folded;         // PROTOCOL_CHIRP_BINS: what the decoder decides from
    const float *band;           // 2 * PROTOCOL_CHIRP_BINS: bins -bins .. bins-1 of the de-chirped FFT
    const float *shape;          // PROTOCOL_CHIRP_BINS: the learned channel shape
    bool shape_changed;          // shape was learned on this frame
    const float *spectrum;       // spectrum_len bins of the de-chirped FFT
    uint32_t spectrum_len;
} ProtocolFrame_t;

typedef enum {
    PROTO_STREAM_FRAME,
    PROTO_STREAM_FOLDED,
    PROTO_STREAM_BAND,
    PROTO_STREAM_SHAPE,
    PROTO_STREAM_SPECTRUM,
    PROTO_STREAM_COUNT
} ProtocolStream_t;

typedef struct {
    bool on;
    bool power;          // units=power (else db)
    uint32_t every;      // Send one frame in this many
    uint32_t count;      // Frames since the last one sent
    uint32_t fields;     // frame: bit per field
    uint32_t from, to;   // spectrum: bins (inclusive)
} ProtocolSub_t;

// ---------------------------------------------------------------------------
// What the protocol needs from the board. Optional hooks may be NULL.
// ---------------------------------------------------------------------------
typedef struct {
    void *ctx;
    const char *version;  // Firmware version, no spaces
    const char *board_id; // Unique id (hex), no spaces

    // New settings; changed has bit (1 << id) set for each setting that changed
    void (*apply)(void *ctx, const ProtocolSettings_t *settings, uint32_t changed);
    void (*get_status)(void *ctx, ProtocolStatus_t *status);
    // Queue a packet with this id: 0, or PROTO_ERR_QUEUE_FULL
    int (*send)(void *ctx, const uint8_t *payload, uint32_t length, uint32_t id);
    // Move the receive window by samples (positive = later); returns the new delay
    uint32_t (*nudge)(void *ctx, int32_t samples);
    void (*reset_counters)(void *ctx);
    void (*reset_rx)(void *ctx);
    // Optional: keep / fetch saved settings. load returns false if none saved
    bool (*save)(void *ctx, const ProtocolSettings_t *settings);
    bool (*load)(void *ctx, ProtocolSettings_t *settings);
} ProtocolBoard_t;

typedef struct {
    const ProtocolBoard_t *board;
    ProtocolSettings_t settings;
    uint32_t next_id;

    // Input: the line being collected
    char line[PROTOCOL_LINE_MAX + 1];
    uint32_t line_len;
    bool line_too_long;  // Discarding up to the LF
    bool line_held;      // A complete line waiting for room in the output queue

    // Output queue (byte ring) and the reply being built
    char out[PROTOCOL_OUT_SIZE];
    uint32_t out_head, out_tail; // Free-running
    char reply[PROTOCOL_REPLY_MAX];
    uint32_t reply_len;
    uint32_t reply_lines;  // Each will get a check in front
    bool reply_overflow;
    char event_line[PROTOCOL_OUT_LINE_MAX + 2]; // An event being formatted (not on core 0's small stack)

    ProtocolSub_t subs[PROTO_STREAM_COUNT];
    uint32_t stream_dropped;

    uint32_t next_seq;               // Every event gets the next one (0 = "* hello" at start-up)
    uint32_t link_errors;            // Lines that failed their check
    uint32_t events_dropped;         // Total
    uint32_t events_dropped_pending; // Not yet reported with "* overflow"
} Protocol_t;

void protocol_init(Protocol_t *p, const ProtocolBoard_t *board);

// Sends "* hello ..." (call once the board is running)
void protocol_start(Protocol_t *p);

// Bytes from the host. Returns how many were consumed (see above)
size_t protocol_input(Protocol_t *p, const char *data, size_t length);

// Something happened on the board
void protocol_event(Protocol_t *p, const ProtocolEvent_t *event);

// A frame was received: sends whatever the streams ask for. Stream lines
// are dropped (and counted) before they would crowd out replies or events
void protocol_frame(Protocol_t *p, const ProtocolFrame_t *frame);

// Retries a held command and reports dropped events, once there is room
void protocol_poll(Protocol_t *p);

// Copies up to max bytes of output for the host; returns the count
size_t protocol_output(Protocol_t *p, char *buf, size_t max);

// Bytes waiting in the output queue
uint32_t protocol_output_pending(const Protocol_t *p);

// CRC-16/CCITT (poly 0x1021, init 0xFFFF) of a line's text: every line,
// both ways, is "XXXX text" with this in upper-case hex
uint16_t protocol_crc16(const char *data, size_t length);

// The built-in defaults
void protocol_default_settings(ProtocolSettings_t *settings);

// Current settings (as last applied)
const ProtocolSettings_t *protocol_settings(const Protocol_t *p);

#endif // PROTOCOL_H
