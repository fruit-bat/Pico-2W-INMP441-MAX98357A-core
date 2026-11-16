# Packet format and receiver states — DRAFT for review

Status: the transmitter (`packet_tx`, all of the layout below) and the
receiver's first stage (`packet_rx`: SEARCH, LOCK, TRAIN) are implemented
and tested in the host model and between two boards (10/10 packets found in
each direction). HEADER, PAYLOAD and CHECK, and window alignment, are next.
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
| SEARCH | correlate this frame's spectrum (square-root power, mean removed) with the previous one | 0.5 or more on 3 frames in a row → LOCK |
| LOCK | correlate with A's average spectrum | below 0.4 → TRAIN (B has started); over 16 frames → a steady tone, ignored until it changes |
| TRAIN | keep frames | 2 frames after B should have ended: place B (best coherence, at least 0.3), learn the shape, report the strongest and earliest paths → (for now) SEARCH; else → SEARCH |
| HEADER | matched decisions; collect 16 bits; check CRC-8 | CRC-8 good → PAYLOAD; bad → SEARCH |
| PAYLOAD | matched decisions; collect the payload and CRC bits | all symbols received → CHECK |
| CHECK | CRC-16 | publish the packet (good or bad, with per-symbol margins) on the record queue to core 0; → SEARCH |

Timing tracking continues through HEADER and PAYLOAD (decision directed, on
the matched decisions).

Measured so far: in the model, preambles are found down to −12 dB, and
2000 frames of noise never lock. Between the boards the coherence was
0.5–0.63 with the windows 224–352 samples out of line (0.99 for one board
hearing itself), and the learned shapes repeated packet after packet.

**Known limitation:** near half a frame out of line, B's start is ambiguous
by one frame. Aligning the window before B (option (a) below) cures it.

**Order problem to solve:** the window should be frame-aligned *before* B is
learned, but the full offset is only certain after B. Options: (a) align
on A's estimate of *r* when entering TRAIN, then correct any residual from
B (moving the learned shape round by the same amount); (b) learn B at
whatever alignment and move the window after B. I'd start with (a).

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
