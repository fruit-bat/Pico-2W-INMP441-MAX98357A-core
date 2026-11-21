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
| A: detect   | B: train + align   | H: header           | P: payload    | C: CRC-16 | R: RS check  |
| 8 x sym 0   | 8 known symbols    | 8 symbols at        | 0..63 bytes   | 16 bits   | 4 bytes, if  |
|             |                    | spacing 8 (Golay)   | at the payload's spacing  | H says rs2   |
```

The receiver needs no spacing or error correction setting: B is the same
sound at every spacing, and H, at a fixed spacing, says what P, C and R use.

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

### H — header (12 bits in an extended Golay code, 2026-11-20)

| Bits | Field |
|---|---|
| 2 | Format version: 0 |
| 2 | Payload spacing: 0 = 1 bin (6 bits/symbol), 1 = 2 (5), 2 = 4 (4), 3 = reserved (for 8) |
| 2 | Payload error correction: 0 = none; 1 = rs2 (R below); 2, 3 reserved |
| 6 | Payload length in bytes, 0..63 |

The 12 bits (MSB first, in that order) are the message of an extended Golay
(24,12) code: the 12 bits, then 12 check bits (the (23,12) code's 11, with
generator x¹¹+x¹⁰+x⁶+x⁵+x⁴+x²+1, and an overall parity bit). Every two
codewords differ in at least 8 bits, so the receiver corrects any 3 wrong
bits and detects any 4. The 24 bits go out as **8 symbols at spacing 8**
(3 bits each, Gray coded, MSB first), whatever the payload's spacing: the
transmitter numbers them at its own spacing (×2 at 4 bits, ×8 at 6), as it
does B. Not scrambled.

Why (soak 3, 2026-11-20, `rx.symbols`): of 48 packets that failed with
symbols decided, 36 had one wrong symbol and 44 at most two; every one of
the 21 failed headers had 1 or 2. 61% of wrong symbols were one step off
(one bit, with Gray coding). Spacing 8 puts the header symbols twice as far
apart as the default payload's, and the code then corrects 3 one-step slips,
or one symbol wrong in all 3 bits. It replaces the CRC-8, which could only
detect.

The receiver decodes by trying all 4096 codewords (in Gray-code order, one
XOR and a bit count each: well under a millisecond, once per packet).
**Too many wrong bits** (`reason=check`) or **a format it doesn't know**
(`reason=format`: another version, the reserved spacing, a reserved error
correction) ends the packet. `rx.packet` reports `fixed=`, the bits
corrected.

Trade-off: a stronger code accepts more noise. 24 random bits land within 3
bits of some codeword 57% of the time (CRC-8: 0.4%); the format check then
rejects all but 5%, so about 3% of junk headers get through, to a bad CRC-16
up to 3.6 s later. A junk header needs a false lock and a B that trains
first, and soaks have seen no false locks; if that changes, the version bits
could go to a fixed pattern, or correction could stop at 2 bits (7%).

Cost: 8 symbols (0.19 s) instead of 4 (0.09 s) at the default spacing.

### P — payload and C — CRC

Payload bytes, then CRC-16/CCITT (poly 0x1021, init 0xFFFF) over the
header's 12 bits as two bytes (big-endian) and the payload. The bit stream
(P, C) is packed most significant bit first into b-bit symbols at the
payload's spacing, the last symbol padded with zeros.

Two optional, cheap steps on the bit stream (I'd include both):
- **Scrambling**: XOR the payload bits with an LFSR sequence, so that
  repetitive data (e.g. all zeros) cannot look like preamble and symbols stay
  evenly spread. A single scrambler is a generic, old technique.
- **Gray coding** of each data symbol (done): value v is sent as the symbol
  whose Gray code is v (`packet_gray_symbol` / `packet_gray_value`), so a
  slip to the neighbouring symbol, including the wrap round, flips one bit.
  It changes nothing for error detection; it helps when forward error
  correction is added. The preamble is not Gray coded.

### R — Reed–Solomon check bytes (`link.ecc=rs2`, the default; 2026-11-21)

With error correction 1 in the header, 4 check bytes follow the CRC: a
Reed–Solomon code over GF(256) on the bytes as sent (scrambled payload and
CRC), so the receiver corrects **any 2 wrong bytes** in P, C and R before it
descrambles and checks the CRC. The CRC then catches a wrong correction.
Usual conventions (`reed_solomon.h`): field polynomial 0x11D, generator
roots 2⁰..2³, check bytes last, the code shortened to the packet (at most 69
bytes of the 255 it could cover). The receiver decodes once per packet
(Berlekamp–Massey, Chien search, Forney; tens of microseconds on core 1).
`rx.packet` reports `ecc=` and `corrected=` (bytes, or `failed`).

Why (soak 4, 2026-11-21, `rx.symbols`): with the Golay header no packet
failed in its header; of the 9 that failed their CRC, 8 had a single wrong
symbol and 1 a run of 3. A symbol of 4–6 bits lies in one or two bytes, so
one wrong symbol costs at most 2 bytes: 2 correctable bytes cover a wrong
symbol wherever it falls, or two that fall within single bytes.

Limits: 3 or more wrong bytes are refused, or now and then "corrected" into
another codeword: about as often as a random word lies within 2 bytes of
one, 0.4% for a 16-byte payload, 3.6% for 63 (`test_reed_solomon`); the
CRC-16 then lets through 1 in 65536 of those. At 5 bits a symbol straddles
a byte boundary 4 times in 5, so a single wrong symbol often uses up both
bytes (2 of 5 corrections in the first board check). Marking the least
certain decisions as erasures (the decoder then fixes up to 4 marked bytes)
is the next step if the soaks show 2-byte failures.

Cost: 32 bits, 8 symbols at 4 bits per symbol (0.19 s).

### Airtime examples

| Payload | Symbols (A+B, H, P+C) at 4 bits | Time | At 6 bits | With rs2 (+R) at 4 bits | At 6 bits |
|---|---|---|---|---|---|
| 0 bytes | 16 + 8 + 4 | 0.65 s | 0.63 s | 16 + 8 + 12: 0.84 s | 0.77 s |
| 16 bytes | 16 + 8 + 36 | 1.39 s | 1.11 s | 16 + 8 + 44: 1.58 s | 1.25 s |
| 63 bytes | 16 + 8 + 130 | 3.58 s | 2.58 s | 16 + 8 + 138: 3.76 s | 2.69 s |

The preamble is ~0.4 s; long payloads make it cheap, short ones do not.

## Receiver states (core 1, one step per frame)

```
SEARCH --detect--> LOCK --A ends--> TRAIN --B₇ seen--> HEADER --decoded--> PAYLOAD --last symbol--> CHECK
   ^                 |                 |                  |                                          |
   +----timeout------+-----timeout-----+--check/format----+---------- CRC-16 good or bad ------------+
```

| State | Each frame | Leaves when |
|---|---|---|
| SEARCH | correlate this frame's spectrum (square-root power, mean removed) with the previous one | 0.5 or more on 4 frames in a row → LOCK, measuring how far the window is out from those 4 agreeing A frames (not the frame the first agreed with: see below) (main.c then moves it; `set rx.align=off` turns that off) |
| LOCK | correlate with A's average spectrum | below 0.4 → TRAIN (B has started); over 16 frames → a steady tone, ignored until it changes |
| TRAIN | keep frames | 2 frames after B should have ended: place B (best coherence, at least 0.3), learn the shape, report the strongest and earliest paths, and whether the lock's move was wrong (main.c then moves the window back) → HEADER (deciding any data frames already in); else → SEARCH |
| HEADER | matched decisions (`channel_template_decide`) among the 8 spacing-8 symbols; collect 24 bits; Golay decode | at most 3 bits wrong and a known format → PAYLOAD, at the spacing it gives; else → SEARCH (`rx.header_failed reason=check\|format`) |
| PAYLOAD | matched decisions; collect the payload, CRC and any check bytes | all symbols received → CHECK |
| CHECK | with rs2, Reed–Solomon correction; then descramble, CRC-16 (done in the same frame as the last symbol) | report the packet (good or bad) with its lowest margin, which symbol that was, and how many were below 1.2; main.c puts the payload on its own queue to core 0 → SEARCH |

No timing tracking during the data yet: the crystals agree to 0.6 ppm, about
0.2 samples over the longest packet, and each packet re-aligns from its A.
Core 0 reports each packet to the host as an `* rx.packet` event (good or
bad, with its payload), and counts them (`count.good`, `count.bad`).

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

**Only whole A frames (2026-11-18).** The lock used to wait for 3
agreements and measure from 4 frames: the 3 agreeing ones and the frame the
first of them agreed with. With the window about half a frame out, that
oldest frame holds only part of A. On board 2 it once held stray energy
exactly 20 bins above A's peak, the edge of the earliest-path search. The
lock moved the window 10 bins instead of 30, B's coherence fell to 0.42, and
the packet failed. That happened twice in about 50 first packets, and was
reproduced by nudging the window there after the board had heard itself.
Now the lock waits for a 4th agreement and measures from the 4 agreeing
frames, all whole A frames. A is 8 frames, so the window still moves before
B. `test_partial_first_frame` replays the logged frames.

| Packets intact of 40 (model) | Old (3 agreements + the frame before) | Measure the 3 agreeing only | Now: 4 agreements, measure those |
|---|---|---|---|
| 4 bins/symbol, measured room, 0 / −6 dB | 40 / 37 | 40 / 37 | 40 / 40 |
| 4 bins/symbol, harsh room, 0 / −6 dB | 40 / 40 | 40 / 40 | 40 / 40 |
| 1 bin/symbol, measured room, 0 / −6 dB | 40 / 37 | 40 / 37 | 40 / 40 |
| 1 bin/symbol, harsh room, 0 / −6 dB | 40 / 40 | 38 / 35 | 38 / 36 |

The cost is in the model's harsh room at 1 bin/symbol. Its echo 672
samples late is as strong as the direct path, and from whole A frames it
looks just as "early". The old partial frame came before that echo
arrived, so it happened to favour the direct path. On the boards
afterwards: 30/30 with the window moved −394 to −501 samples (the band that
failed), and 20/20 in normal traffic.

**Which path to align to:** aligning to the earliest strong copy (direct
path) means late copies carry a little of the previous symbol; the other
way round, early copies carry some of the *next* symbol. `packet_rx`
reports the earliest copy within 6 dB of the strongest. Because shapes wrap
every 64 bins, a copy more than half a frame late looks early, so it only
looks up to 20 bins (320 samples) above the strongest.

**Undoing a wrong move (2026-11-19).** A soak test (`tools/chirp_soak.py`,
600 packets, both directions, spacing 2 and 4, −28 to −40 dB) failed 63
packets. Every one of the 5 failures at −28 and −34 dB, and 14 in all, had
the same story: the lock moved the window about 19 bins (~300 samples), and
training then found the strongest path 19 bins back, where the window had
been. A logged capture showed why. In this room an echo about 44 bins
(~700 samples) late lands, wrapped, 20 bins above the strongest path: the
edge of the earliest-path search. It is usually 7–12 dB down and now and
then reaches −6 dB, and A, a held symbol, can't tell it from an early path.
B can, because its symbols change.

So after training: if the lock moved the window at least 8 bins and
training's strongest path is within 3 bins of where the window was before,
`packet_rx` reports `realign_bins` and main.c moves the window back
(`rx.trained moved=`), and the learned shape moves with it. The rule is
narrow on purpose. In the soak, training's strongest path was −5..+1 for
every normal packet and −17..−20 after every bad move, but in a room where
a later path is the strongest (the model's harsh room) "move to the
strongest" would move a right window. Training's earliest path is no
better: the same wrapped echo turned up there too (+1, +2). Up to 4 data
frames (the first header symbols) have been decided through the wrong
window by then. `test_ghost_lock` replays the logged frames, and
`test_wrong_lock_move` checks the whole packet in the model with the lock
sent 10–19 bins the wrong way.

## Transmitter (core 1, one symbol per frame)

```
IDLE (silence) --packet queued--> A (8) --> B (8) --> H, P, C, R --> IDLE
```

Core 0 queues a packet (bytes) on a single-producer single-consumer ring,
like the record queue. Core 1 builds the symbols as it goes, so no symbol
buffer is needed beyond a few bytes of state.

**Half duplex:** a board hears its own speaker far louder than the other
board, so its receiver ignores frames while it is transmitting (and *2
frames* after, for the room to quieten).

## Spacing: 4 bins per symbol by default

Since the Golay header (2026-11-20) `link.spacing` sets only what this board
sends (and the test modes); a receiver follows each packet's header, so
boards no longer need to match, and changing it no longer abandons a packet
being received.


Long (255-byte) packets at 1 bin per symbol, 2026-11-17: board 1 → 2 10/10
intact, board 2 → 1 only 2/10. The errors were ±1 slips at the same symbols
in most packets, set by the previous symbol (paths 16–48 samples late carry
its tail into the next frame). At 2 bins per symbol: 2 → 1 8/8, 9/9 and 4/4,
1 → 2 5/5; at 4: 3/3 each way, then 6/6 each way with room noise and no
close decisions at all. So the default is now 4 (4 bits per symbol, 12.4 s
for 255 bytes; 2 gives 5 bits, 10.0 s); `set link.spacing=1|2|4` changes it while running (both boards
must match; a packet in progress is abandoned), and the log records the
spacing on every row.

## Questions answered (kept for the record)

1. **Default spacing**: 1 bin/symbol (most rate) or 2 (more tolerant of
   smearing)? With the matched decision, 1 got 99.9–100% between the boards.
   *Now 4; see above.*
2. **Maximum payload**: 255 bytes (8 s of airtime) or smaller? *63 since
   2026-11-20: a 6-bit length in the coded header.*
3. **Scrambling and Gray coding**: include now (cheap), or leave out until
   needed?
4. **First implementation step**: I'd propose the transmitter plus a
   receiver that runs SEARCH → LOCK → TRAIN and logs what it finds (offset,
   learned shape, B position), tested in the host model and between the
   boards, before adding HEADER/PAYLOAD/CHECK.

IP note: everything here is up-chirps, generic CRCs, a scrambler, Gray
coding, a classic Golay code and a textbook Reed–Solomon code (not LoRa's
Hamming coding chain or interleaver); it avoids the down-chirp sync. Patent details remain unverified.
