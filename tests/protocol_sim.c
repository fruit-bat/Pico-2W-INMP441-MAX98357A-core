/**
 * @file protocol_sim.c
 *
 * The protocol engine and the pretend board (fake_board.c) as a program:
 * protocol lines on stdin and stdout, the board's clock running in real
 * time (or faster). For trying out hosts, such as tools/chirp_host.py,
 * without a board:
 *
 *   tests/build/protocol_sim [--speed <x>] [--save]
 *
 *   --speed x  run the board's clock x times faster than real time
 *   --save     let "save" work (in memory); the real board can't yet
 *
 * Ends when stdin closes. Output the host isn't reading waits in the
 * engine's queue (stdout is non-blocking), as it would on the board.
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "fake_board.h"
#include "protocol.h"

static Protocol_t proto;
static FakeBoard_t fb;

static double now_s(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
    double speed = 1.0;
    bool save = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc) {
            speed = atof(argv[++i]);
        } else if (strcmp(argv[i], "--save") == 0) {
            save = true;
        } else {
            fprintf(stderr, "usage: %s [--speed <x>] [--save]\n", argv[0]);
            return 2;
        }
    }
    if (speed <= 0.0) speed = 1.0;

    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    fcntl(STDOUT_FILENO, F_SETFL, fcntl(STDOUT_FILENO, F_GETFL) | O_NONBLOCK);

    fake_board_init(&fb, &proto);
    fb.save_enabled = save;
    protocol_init(&proto, &fb.board);
    protocol_start(&proto);

    static char in[4096];
    size_t in_len = 0;     // Bytes read but not yet taken by the engine
    static char out[4096];
    size_t out_len = 0, out_pos = 0;
    bool eof = false;
    const double start = now_s();
    double simulated_us = 0.0;

    while (!eof || in_len > 0) {
        struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };
        poll(&pfd, 1, 2); // A couple of ms: well under a frame

        // Input: only read more once the engine has taken what we have
        if (!eof && in_len == 0) {
            const ssize_t n = read(STDIN_FILENO, in, sizeof in);
            if (n > 0) in_len = (size_t)n;
            else if (n == 0) eof = true;
            else if (errno != EAGAIN && errno != EWOULDBLOCK) eof = true;
        }
        if (in_len > 0) {
            const size_t used = protocol_input(&proto, in, in_len);
            memmove(in, in + used, in_len - used);
            in_len -= used;
        }

        // The board's clock
        const double target_us = (now_s() - start) * 1e6 * speed;
        if (target_us > simulated_us) {
            fake_board_advance_us(&fb, (uint64_t)(target_us - simulated_us));
            simulated_us += (double)(uint64_t)(target_us - simulated_us);
        }
        protocol_poll(&proto);

        // Output, as much as the host will take
        for (;;) {
            if (out_pos == out_len) {
                out_len = protocol_output(&proto, out, sizeof out);
                out_pos = 0;
                if (out_len == 0) break;
            }
            const ssize_t n = write(STDOUT_FILENO, out + out_pos, out_len - out_pos);
            if (n <= 0) break; // Host not reading: try again later
            out_pos += (size_t)n;
        }
    }
    // Last words
    for (;;) {
        if (out_pos == out_len) {
            out_len = protocol_output(&proto, out, sizeof out);
            out_pos = 0;
            if (out_len == 0) break;
        }
        const ssize_t n = write(STDOUT_FILENO, out + out_pos, out_len - out_pos);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { poll(NULL, 0, 2); continue; }
        if (n <= 0) break;
        out_pos += (size_t)n;
    }
    return 0;
}
