# RP2350 (Pico 2 W) I2S Audio Core Driver (DMA & Interrupts)

This system provides a full-duplex, dual-peripheral I2S engine optimized for the RP2350 (Pico 2 W). 
It uses both hardware PIO blocks (`pio0` and `pio1`) synchronized to the same master clock pins, 
operating via zero-CPU ring-buffered Direct Memory Access (DMA) and hardware interrupts.

---

## 1. Project Directory Structure

```text
audio_core_project/
├── CMakeLists.txt
├── pico_sdk_import.cmake
└── src/
    ├── main.c
    ├── i2s_core.c
    └── i2s_core.h
```

---

## 2. Buid

```sh
cmake -S . -B build -DPICO_BOARD=pico2 -DPICO_SDK_PATH=/home/neo/fruit-bat/pico/pico/pico-sdk -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

### 3. Compile the source code using all available CPU threads
```sh
cmake --build . -j
```
---

## 2. Hardware Allocation Architecture

To guarantee strict multi-core or interrupt stability on the RP2350, resources are explicitly isolated:

* **PIO Blocks**: `pio0` is dedicated to I2S Transmit (TX / Amplifier). `pio1` is dedicated to I2S Receive (RX / Microphone).
* **DMA Channels**: Channel A manages the ping-pong peripheral write stream. Channel B manages the ping-pong peripheral read stream.
* **Interrupt Handlers**: Tied natively to DMA IRQ0 to process buffer switches instantly with zero jitter.

---

## 4. Operational Validation and Test Strategy

Before implementing Chirp Spread Spectrum modulation, verify the base engine layer using these validation criteria:

1. **Phase and Timing Verification:** Ensure that the Master Bit Clock does not slip phase under full-duplex operation. If the microphone and speaker sample rates drift, double-buffering asserts underflows.
2. **DMA Inter-lock Check:** The system uses dual ring buffers. Monitor execution using standard visual indicators. Ensure that `i2s_callback_rx_ready` processing time does not exceed $32\,	ext{ms}$ (the duration of a $512$-sample buffer at $16\,	ext{kHz}$), otherwise subsequent data packets will overwrite values before processing finishes.
