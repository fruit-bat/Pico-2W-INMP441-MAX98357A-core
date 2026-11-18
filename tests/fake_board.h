#ifndef FAKE_BOARD_H
#define FAKE_BOARD_H

/**
 * @file fake_board.h
 *
 * A pretend board behind the protocol engine, for the PC tests: plausible,
 * made-up data on a simulated clock (one frame per 1024 samples at 44.1 kHz).
 *
 * Packets queued with send are transmitted for as many frames as a real one
 * would take at the current spacing (preamble 16 frames, then header,
 * payload and CRC). With rx.during_tx on, the board also "hears" its own
 * packet, as the real one does: rx.locked a few frames in, rx.trained after
 * the preamble, and rx.packet when it ends.
 *
 * Tests may also call protocol_event() directly to inject events at awkward
 * moments.
 */

#include "protocol.h"

#define FAKE_TX_QUEUE 2u // As TX_PACKET_QUEUE_CAPACITY in main.c

typedef struct {
    uint32_t id, length;
    uint8_t payload[PROTOCOL_PAYLOAD_MAX];
} FakePacket_t;

typedef struct {
    Protocol_t *proto;          // Where events go
    ProtocolBoard_t board;      // The interface handed to the protocol

    // Simulated time
    uint64_t now_us;
    uint32_t frame;

    // What the protocol told us
    ProtocolSettings_t settings;
    uint32_t apply_calls, last_changed;

    // Pretend state
    ProtocolStatus_t status;
    FakePacket_t queue[FAKE_TX_QUEUE];
    uint32_t queued;
    bool sending;
    FakePacket_t current;
    uint32_t sent_frames, total_frames;
    bool abandon;               // Spacing changed mid-packet: report it on the next frame
    bool listening;             // Hearing its own packet (since rx.locked, until it concludes)
    bool reset_rx;              // "reset rx" asked for: done on the next frame

    // Saved settings (RAM only); save fails unless save_enabled
    bool save_enabled, have_saved;
    ProtocolSettings_t saved;
} FakeBoard_t;

void fake_board_init(FakeBoard_t *fb, Protocol_t *proto);

// Runs the simulated clock forward; frames that complete report their events
void fake_board_advance_us(FakeBoard_t *fb, uint64_t us);
void fake_board_advance_frames(FakeBoard_t *fb, uint32_t frames);

// Frames a packet of this many bytes takes at the current spacing
uint32_t fake_board_packet_frames(const FakeBoard_t *fb, uint32_t length);

#endif // FAKE_BOARD_H
