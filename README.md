# RP2350 (Pico 2 W) Chirp Audio Transceiver

An acoustic data link experiment for the Raspberry Pi Pico 2 W (RP2350), using an
**INMP441** I2S microphone and a **MAX98357A** I2S amplifier. Data symbols are
sent as frequency-shifted chirps (chirp spread spectrum) in the 15–17.8 kHz band
and decoded with a de-chirp + FFT receiver.

The project is a work in progress: the I2S engine, symbol encoder, symbol
decoder and symbol timing tracking work; packet framing is still to come. A
terminal display is provided to see what the receiver is doing.

All code avoids dynamic memory allocation; every module takes caller-provided
(typically static) storage.

See [i2s_audio_circuit_guide.md](i2s_audio_circuit_guide.md) for wiring.

---

## 1. Build

Requires the Pico SDK (tested with 2.3.0). CMSIS-DSP v1.15.0 is fetched
automatically.

```sh
cmake -S . -B build -DPICO_BOARD=pico2 -DPICO_SDK_PATH=/path/to/pico-sdk -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Copy `build/Pico-2W-INMP441-MAX98357A-core-test.uf2` to the board in BOOTSEL mode.

Notes:

* The board is configured as `pico2`, which is sufficient because the wireless
  chip is not used. Use `pico2_w` if Wi-Fi or the on-board LED (which is driven
  through the wireless chip) is needed.
* The program is built as `copy_to_ram`: everything is copied from flash to RAM
  at boot so the real-time path never waits on flash. If RAM becomes short,
  [CMakeLists.txt](CMakeLists.txt) documents how to run from flash and move
  only the hot library code into RAM instead.

---

## 2. Using It

Connect to the USB serial port (e.g. `minicom -D /dev/ttyACM0`). The display
shows a page of the FFT spectrum of the received (de-chirped) signal, followed
by the transmitted and decoded symbols and receiver statistics.

| Key | Action |
| :--- | :--- |
| `1` | Loopback: generated chirps are fed straight to the receiver (speaker silent) |
| `2` | Mic level: speaker silent, receive from the microphone |
| `3` | 440 Hz test tone (start-up default, to check the speaker path) |
| `4` | Chirp: transmit chirps through the speaker, receive from the microphone |
| `o` / `p` | Previous / next transmitted symbol |
| `z` / `x` | Decrease / increase the receive alignment delay by one sample |
| `t` | Cycle timing tracking: off → decision → pilot (see below) |
| `c` / `v` | Previous / next page of the spectrum display |
| `l` | Toggle CSV logging (replaces the display; see below) |

The status lines show:

* **TX symbol / RX symbol** – what is being sent and what was decoded.
* **centroid / error** – the decoded position in symbol units and its offset
  from the nearest symbol in radians.
* **Tracking / timing error / drift** – the tracking mode, the measured timing
  error in samples and the tracker's drift estimate. The timing error is shown
  even with tracking off, as a guide for adjusting the delay by hand.
* **magnitude** – how clearly one tone stands out (0–1); tracking ignores
  frames below 0.8.
* **peak** – peak received level as a percentage of full scale.
* **missed rx / tx** – DMA blocks the processing core failed to service in time
  (should stay at 0).
* **dropped records** – receiver results lost because the display core fell
  behind.

### Timing tracking

Each frame must start at a symbol boundary. A window that is *d* samples off
shifts the decoded position by *d*/16 of a symbol, so the decoder's angle error
gives the timing error directly (one sample ≈ 2π/1024 rad).

Each FFT bin after de-chirping corresponds to 16 samples of delay, so the
spectrum around a symbol is effectively the channel's impulse response.
Through the air, echoes spread it over about three bins, so the decoder
measures the symbol's position as the energy centroid of a ±3-bin window
around the peak, sliding the window to its own centroid a few times (a mean
shift) rather than fitting the single strongest bin. Logged sweeps showed this
agrees across symbols to within about 2–3 samples, with no frame-to-frame
flipping when echoes give the peak two humps.

`symbol_timing_tracker` turns the error into delay corrections each frame.
It can also learn a steady drift (e.g. clock mismatch between two boards), but
that is disabled (`TRACK_BETA` = 0) while one board both transmits and receives,
as there is no drift to learn and it only picked up noise. Frames without a
clear tone are ignored, and pressing `z`/`x` while tracking simply moves it to
the new delay, from where it re-locks.

* **decision** – uses the error from the nearest symbol, as a real receiver
  must. It aligns to the symbol grid (±8 samples pull-in), but because a
  16-sample shift looks exactly like the next symbol, decoded symbols may be
  offset from the transmitted ones by a constant. A preamble will resolve this.
* **pilot** – uses the error from the known transmitted symbol (testing only).
  It locks from any delay with the correct symbol labels.

The gains are `TRACK_*` constants near the top of `main.c`.

### Logging

Pressing `l` replaces the display with one CSV line per received frame
(about 43 lines per second), starting with `#` comment lines describing the
configuration and a column header. Each line holds the transmitted symbol, the
decoded symbol (angle, error, magnitude, strength), the tracker state, the
missed/dropped counters, and the FFT power of the 128 bins either side of DC
(`b-64` … `b63`) that the decoder uses.

[tools/capture_log.py](tools/capture_log.py) captures a log without needing a
terminal program (standard library only):

```sh
# chirp mode, pilot tracking, 30 s
python3 tools/capture_log.py --keys 4tt --seconds 30 pilot.csv

# chirp mode, tracking off, step through all 64 symbols for 1 s each
python3 tools/capture_log.py --keys 4 --sweep 1 sweep.csv
```

It reports gaps in the frame numbers; if the board cannot keep up, the
`dropped` column will also increase.

[tools/analyse_log.py](tools/analyse_log.py) summarises a log per transmitted
symbol (skipping the frames just after each symbol change): decode accuracy,
the decoded position relative to the transmitted symbol in samples, the
decoder's error, and magnitude.

```sh
python3 tools/analyse_log.py sweep.csv
```

---

## 3. Signal Parameters

| Parameter | Value |
| :--- | :--- |
| Sample rate | 44.1 kHz, 32-bit I2S |
| Frame / symbol length | 1024 samples (≈ 23.2 ms) |
| FFT | 1024-point complex (CMSIS-DSP) |
| Symbols | 64 (6 bits per frame, ≈ 258 bit/s raw) |
| Band | 15000 Hz to ≈ 17756 Hz (64 FFT bins of ≈ 43.1 Hz) |

Symbol *s* is the base up-chirp cyclically shifted by 16·*s* samples. After
multiplying by the conjugate chirp, this concentrates its energy at FFT bin *s*,
whose position is estimated with a circular weighted centroid.

---

## 4. Architecture

### Hardware

* **One PIO state machine** (on `pio0`) generates BCLK and WS, shifts data out to
  the amplifier, and samples the microphone on the same clock edges, so TX and RX
  are locked together. Mic samples arrive in the same state machine's RX FIFO.
* **Four DMA channels** (two chained channels per direction) move 1024-sample
  blocks between ping-pong buffers and the PIO FIFOs. When one channel finishes
  its buffer the hardware immediately starts the other, so the I2S stream has no
  gaps. The DMA interrupt (DMA_IRQ_0, on core 0) rewinds the finished channel,
  marks its buffer as ready for core code and advances a block counter.
* The system clock runs at 200 MHz.

### Cores

* **Core 1 – real-time DSP.** Polls the DMA block counters. For each TX block it
  generates audio; for each RX block it aligns the frame, decodes a symbol and
  publishes the result. It never blocks and does no I/O.
* **Core 0 – user interface.** Handles keys and the terminal display. It reads
  the receiver's results from a lock-free queue and the spectrum from a
  snapshot handshake, so it always sees complete, consistent data.

### Receive chain (core 1)

```text
I2S RX DMA block (q31)
  → float conversion
  → rx_sample_window      align the frame to symbol timing (delay)
  → chirp_decoder         de-chirp × FFT → magnitudes → centroid → energy-refined peak
  → chirp_symbol_rx       angle → symbol index + error
  → symbol_timing_tracker timing error → next frame's delay
  → RxFrameRecord_t       pushed to spsc_ring → core 0
```

Each `RxFrameRecord_t` carries the frame number, delay, transmitted symbol, peak
level, decoded symbol and timing tracking state. This queue is the receiver's output: today it feeds
the display; it is intended to feed logging to a host and packet decoding later.

### Source files

| File | Purpose |
| :--- | :--- |
| `main.c` | Application: mode handling, core 1 audio loop, core 0 display |
| `i2s_core.c/.h` | Chained DMA ping-pong buffers, interrupt handler, block counters, pin definitions |
| `audio_i2s_setup.c/.h`, `audio_i2s.pio` | PIO program and state machine setup |
| `chirp_encoder.c/.h` | Symbol → transmitted chirp samples |
| `chirp_decoder.c/.h` | Received samples → symbol (de-chirp, FFT, centroid) |
| `chirp_symbol_rx.c/.h` | Symbol index and error from a centroid angle |
| `cyclic_weighted_centroid.c/.h` | Circular weighted centroid of a spectrum band, with energy-window peak refinement |
| `rx_sample_window.c/.h` | Double-buffered receive window with adjustable alignment delay |
| `symbol_timing_tracker.c/.h` | Second-order loop keeping the receive window aligned to the symbols |
| `spsc_ring.c/.h` | Lock-free single-producer/single-consumer queue between cores |
| `frame_snapshot.c/.h` | Request/ready handshake for copying a frame between cores |
| `fft_visualise.c/.h` | Paged terminal bar graph of a spectrum |
| `running_moving_average.h`, `window_moving_average.h` | Averaging helpers |
| `audio_i2s_stereo.pio` | Alternative PIO program (not currently built) |
| `tools/capture_log.py` | Captures the CSV log over USB serial, optionally sweeping all symbols |
| `tools/analyse_log.py` | Per-symbol statistics from a CSV log |

---

## 5. Signal Detection Metrics

The receiver's cyclic weighted centroid reports two different measures:

* **Magnitude** measures how concentrated the weighted spectrum is around one
    direction on the centroid circle. It is normalized and is useful for judging
    directional consistency, but it does not indicate how much signal energy is
    present.
* **Strength** is derived from the unnormalized resultant before normalization.
    It preserves information about the total weighted energy in the selected FFT
    window, so it is the preferred metric for deciding whether a signal is being
    received.

Use `strength` with a threshold established from the actual microphone gain,
FFT scaling, and measured noise floor. A high `magnitude` with low `strength`
can still represent a consistent but very weak or noise-level signal.

---

## 6. Timing Budget

Each block is 1024 samples at 44.1 kHz, so core 1 has about **23.2 ms** to
generate a TX block and decode an RX block before the next one completes. If it
overruns, the `missed rx` / `missed tx` counters increase and receive alignment
is lost for that block.

Because the DMA channels are chained, the interrupt handler only has to rewind
the finished channel before its partner completes, so it too has a full block
period; the I2S stream does not depend on interrupt latency.
