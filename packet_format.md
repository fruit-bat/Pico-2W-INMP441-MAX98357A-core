# Packet format and receiver states — DRAFT for review

Status: the transmitter (`packet_tx`) and receiver (`packet_rx`: all the
states below) are implemented and tested in the host model and between two
boards: 30/30 16-byte packets intact in each direction (2026-11-17), with
the window aligned from A to within about a sample.
Numbers in *italics* are still guesses.

## Background: what the measurements say

| Finding | Source | Consequence |
|---|---|---|
| Decision tracking locks the window to the 16-sample symbol grid from any start in 4–7 frames | host closed-loop tests, one board | No search over 1024 offsets is needed |
| It does not find the transmitter's *frame* boundary; a window half a frame out mixed two symbols per frame and ruined decoding | two boards, sequence mode | The preamble must tell the receiver where frames start |
| A room spreads each symbol into several copies (seen: 0, +1/+2, −13/−14, ±28 bins), often as strong as the direct path | two boards 2 m apart | Decide by matching against a learned channel shape (`channel_template`), not the strongest bin: 30–74% → 99.9–100% |
| 2–4 known frames are enough to learn that shape | two boards, offline | Training costs little airtime |
| The two crystals agree to +0.6 ppm | two boards | No clock-rate learning needed for these boards; motion would need it |
| Errors from the previous symbol were negligible once aligned | two boards | No guard interval for now |

Constraints carried forward: up-chirps only (no down-chirp sync, see
`ip-caution` notes); no dynamic memory; core 1 stays real time.

## Timing and sizes

One symbol per frame: N = 1024 samples at 44.1 kHz = 23.2 ms, 43.07
symbols/s. `BINS_PER_SYMBOL` sets the symbol size b:

| Bins/symbol | Symbols | b (bits/symbol) | Raw rate |
|---|---|---|---|
| 1 | 64 | 6 | 258 bit/s |
| 2 | 32 | 5 | 215 bit/s |
| 4 | 16 | 4 | 172 bit/s |

The format is defined in symbols and bits so it works at any spacing.

## Packet layout

```
| A: detect   | B: train + align   | H: header  | P: payload           | C: CRC-16 |
| 8 x sym 0   | 8 known symbols    | 16 bits    | 0..255 bytes         | 16 bits   |
```

### A — detect (*8 frames* of symbol 0)

Repeated symbol 0 gives the receiver frames that all decode the same,
which is easy to detect, and lets the tracker lock to the symbol grid. The
decoded value *r* is the offset between the receiver's and transmitter's
frames, in symbol steps (window delay off by about r × N/symbols samples). It
may point at a reflection rather than the direct path; B refines it.

### B — train and align (*8 frames*, known, all different)

B is **12 6 15 6 1 7 15 4** on a 16-symbol grid, multiplied up to the
symbol size (×4 at 6 bits, ×2 at 5), so it is the same sound at every
spacing. Found by search: B₀ ≠ 0, neighbours at least 5 of 16 apart, all
neighbour differences distinct, and no misplaced copy (against itself or
the zeros of A) matches any of them.

Uses:
- **Start of data.** The receiver tries B's start a couple of frames
  either side of where A ended, moves each frame's spectrum round by its
  known B symbol, and keeps the start where they agree best (the
  *coherence*). A wrong start mixes different shapes because B's
  differences are all distinct. This replaces a separate sync word and does
  not rely on decoded symbols, which in a room are often a reflection.
- **Frame offset.** The learned shape's peaks are the arrival paths, at bin
  offsets from where the window sits (a window d samples late shows them
  d/16 bins high).
- **Channel training.** Each B frame, with its known symbol, goes into
  `channel_template_learn`.

Learning from changing symbols matches what data looks like (in a room the
late reflections carry part of the previous symbol); a held symbol would
not, though in the model the difference was small.

### H — header (16 bits)

| Bits | Field |
|---|---|
| 8 | Payload length in bytes, 0..255 |
| 8 | CRC-8 of the length byte (so a bad length is caught before waiting for a payload that is not coming) |

At 6 bits/symbol that is 3 symbols (2 spare bits, sent as 0).

### P — payload and C — CRC

Payload bytes, then CRC-16/CCITT (poly 0x1021, init 0xFFFF) over the header
and payload. The bit stream (H, P, C) is packed most significant bit first
into b-bit symbols, the last symbol padded with zeros.

Two optional, cheap steps on the bit stream (I'd include both):
- **Scrambling**: XOR the payload bits with an LFSR sequence, so that
  repetitive data (e.g. all zeros) cannot look like preamble and symbols stay
  evenly spread. A single scrambler is a generic, old technique.
- **Gray coding** of each symbol, so a slip to the neighbouring symbol flips
  one bit. It changes nothing for error detection; it helps if forward error
  correction is added later.

### Airtime examples (6 bits/symbol)

| Payload | Symbols (A+B, H, P+C) | Time |
|---|---|---|
| 0 bytes | 16 + 3 + 3 | 0.51 s |
| 16 bytes | 16 + 3 + 24 | 1.00 s |
| 64 bytes | 16 + 3 + 88 | 2.48 s |
| 255 bytes | 16 + 3 + 343 | 8.4 s |

The preamble is ~0.4 s; long payloads make it cheap, short ones do not.

## Receiver states (core 1, one step per frame)

```
SEARCH --detect--> LOCK --A ends--> TRAIN --B₇ seen--> HEADER --CRC-8 ok--> PAYLOAD --last symbol--> CHECK
   ^                 |                 |                  |                                            |
   +----timeout------+-----timeout-----+-----bad CRC-8----+----------- CRC-16 good or bad -------------+
```

| State | Each frame | Leaves when |
|---|---|---|
| SEARCH | correlate this frame's spectrum (square-root power, mean removed) with the previous one | 0.5 or more on 3 frames in a row → LOCK, measuring how far the window is out from the agreeing A frames (main.c then moves it; key `a` turns that off) |
| LOCK | correlate with A's average spectrum | below 0.4 → TRAIN (B has started); over 16 frames → a steady tone, ignored until it changes |
| TRAIN | keep frames | 2 frames after B should have ended: place B (best coherence, at least 0.3), learn the shape, report the strongest and earliest paths → HEADER (deciding any data frames already in); else → SEARCH |
| HEADER | matched decisions (`channel_template_decide`); collect 16 bits; check CRC-8 | CRC-8 good → PAYLOAD; bad → SEARCH |
| PAYLOAD | matched decisions; collect the payload and CRC bits | all symbols received → CHECK |
| CHECK | descramble, CRC-16 (done in the same frame as the last symbol) | report the packet (good or bad) with its lowest margin, which symbol that was, and how many were below 1.2; main.c puts the payload on its own queue to core 0 → SEARCH |

No timing tracking during the data yet: the crystals agree to 0.6 ppm, about
0.2 samples over the longest packet, and each packet re-aligns from its A.
Core 0 shows good/bad counts and the last payload on the display, and
writes a `# packet ...` comment line into the log for each packet.

Measured so far: in the model, preambles are found down to −12 dB, and
2000 frames of noise never lock. Between the boards the coherence was
0.5–0.63 with the windows 224–352 samples out of line (0.99 for one board
hearing itself), and the learned shapes repeated packet after packet.

**Window alignment (done):** A's frames all hold the same symbol, so
however far the window is out they don't mix two symbols, and A's path
shape is clean. On entering LOCK the receiver takes the earliest strong path
from the agreeing A frames and main.c moves the window that many bins
(16 samples each) so it sits on it, from the next frame, before B starts.
Without the move, near half a frame out B's start was ambiguous by one
frame. In the model every offset (±500 samples) now ends within 8 samples
of the direct path, with B's coherence 1.00 on a clean channel (0.47 at
+500 unaligned).

Between the boards (2026-11-17), 10 packets each way:

| | First lock moved the window | Later locks | B coherence | Learned paths |
|---|---|---|---|---|
| Board 1 → 2 | −19 bins (304 samples) | all 0 | 0.81–0.84 (was 0.60–0.63) | 0 strongest, −1/−2 at a third, +21 at a tenth |
| Board 2 → 1 | +28 bins (448 samples) | 0, one −1 trim | 0.81–0.86 (was 0.49–0.54) | −1/−2 strongest, earliest 0 |

The one-bin trim came from a path near the 6 dB threshold flipping in and
out; hysteresis could stop that if it matters.

**Within a bin too.** Whole-bin alignment wasn't enough: between the
boards, board 2 → 1 lost 10 of 20 packets, every error a ±1 symbol slip.
Nudging the window showed margins collapsing every 16 samples: with the
energy about half a bin off a bin centre it splits between two bins and
neighbouring symbols look alike. The receiver now also averages the
decoder's refined centre over the A frames (on a one-bin circle) and the
window is moved by that fraction too. In the model the window then lands
within 1 sample (clean), long packets in the harsh room went from 6/8 to
8/8 intact, and the clean-channel margin rose from about 2.2 to 5.9.

| Window nudge, board 2 → 1 | Intact | Lowest margin | Decoder error in A (samples) |
|---|---|---|---|
| +8 | 8/8 | 1.18 | +0.6 |
| −8 | 8/8 | 1.33 | −1.2 |
| −24 | 8/8 | 1.44 | −3.6 |
| −40 | 8/8 | 1.52 | −5.9 |
| 0 | 8/8 | 1.13 | +6.7 |
| −16 | 6/8 | 1.01 | +5.9 |
| −32 | 4/8 | 1.00 | +3.4 |

(The pattern drifts slowly between runs: 0.6 ppm is about 1.6 samples a
minute.) With sub-bin alignment, 30 packets each way: all intact; lowest
margin 1.005 (2 → 1) and 1.17 (1 → 2). For 2 → 1 the margins were best with
the residual slightly negative, so its best target is a few samples off the
bin centre; the target could be tuned per channel later.

**Which path to align to:** aligning to the earliest strong copy (direct
path) means late copies carry a little of the previous symbol; the other
way round, early copies carry some of the *next* symbol. `packet_rx`
reports the earliest copy within 6 dB of the strongest. Because shapes wrap
every 64 bins, a copy more than half a frame late looks early, so it only
looks up to 20 bins (320 samples) above the strongest.

## Transmitter (core 1, one symbol per frame)

```
IDLE (silence) --packet queued--> A (8) --> B (8) --> H, P, C --> IDLE
```

Core 0 queues a packet (bytes) on a single-producer single-consumer ring,
like the record queue. Core 1 builds the symbols as it goes, so no symbol
buffer is needed beyond a few bytes of state.

**Half duplex:** a board hears its own speaker far louder than the other
board, so its receiver ignores frames while it is transmitting (and *2
frames* after, for the room to quieten).

## Questions for you

1. **Default spacing**: 1 bin/symbol (most rate) or 2 (more tolerant of
   smearing)? With the matched decision, 1 got 99.9–100% between the boards.
2. **Maximum payload**: 255 bytes (8 s of airtime) or smaller?
3. **Scrambling and Gray coding**: include now (cheap), or leave out until
   needed?
4. **First implementation step**: I'd propose the transmitter plus a
   receiver that runs SEARCH → LOCK → TRAIN and logs what it finds (offset,
   learned shape, B position), tested in the host model and between the
   boards, before adding HEADER/PAYLOAD/CHECK.

IP note: everything here is up-chirps, generic CRCs, a scrambler and Gray
coding; it avoids the down-chirp sync. Patent details remain unverified.
