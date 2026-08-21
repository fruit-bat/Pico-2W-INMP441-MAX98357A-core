# I2S Audio Prototype Circuit Reference: Pico 2 W (RP2350)

This document provides the hardware wiring configurations and architectural details for connecting an **INMP441 Digital Microphone** and a **MAX98357A Class-D Amplifier** simultaneously to a **Raspberry Pi Pico 2 W (RP2350)**.

---

## Circuit Architecture Overview

The Integrated Inter-IC Sound (I2S) protocol relies on a shared bus architecture. Because the Pico 2 W acts as the master clock generator, it can drive both the input (RX) and output (TX) pipelines using a single pair of clock lines. 

* **Clock Sharing:** The Serial Clock (**SCK/BCLK**) and Word Select (**WS/LRC**) lines are multiplexed. Both components listen to these identical pins simultaneously to remain in perfect temporal lockstep.
* **Independent Data Paths:** Data collision is prevented by routing the microphone's input stream (**SD**) and the amplifier's output stream (**DIN**) over entirely distinct, dedicated GPIO pins.
* **Power Isolation:** The microphone runs on clean **3.3V** to protect its sensitive digital logic. The amplifier runs on **5V (VBUS)** to maximize acoustic headroom and prevent speaker transient spikes from causing brownouts on the Pico's main 3.3V regulator.

---

## 1. INMP441 Microphone Wiring

The INMP441 is a high-performance, low-power, digital-output omnidirectional microphone with an integrated bottom-port element. 

*Tying the **L/R** pin to Ground configures the microphone to output its data exclusively on the Left I2S channel slot.*

| INMP441 Pin | Pico 2 W Pin | Physical Pin ID | Description | Bus Status |
| :--- | :--- | :--- | :--- | :--- |
| **VDD** | 3V3 | Pin 36 | 3.3V Main Power Supply | Dedicated |
| **GND** | GND | Pin 38 | System Ground | Shared |
| **L/R** | GND | Pin 38 | Left/Right Channel Select (Low = Left) | Dedicated |
| **SCK** | GP13 | Pin 17 | Serial Clock (Bit Clock / BCLK) | **Shared Clock** |
| **WS** | GP14 | Pin 19 | Word Select (Left-Right Clock / LRCLK) | **Shared Clock** |
| **SD** | GP11 | Pin 15 | Serial Data Output (Microphone to Pico) | Dedicated Input |

---

## 2. MAX98357A Audio Amplifier Wiring

The MAX98357A is a digital pulse-code modulation (PCM) input Class-D power amplifier that offers native I2S flexibility.

*Leaving **GAIN** and **SD** floating configures the chip to use its default 9dB internal gain step and automatically downmixes stereo I2S streams into a combined mono speaker output.*

| MAX98357A Pin | Pico 2 W Pin | Physical Pin ID | Description | Bus Status |
| :--- | :--- | :--- | :--- | :--- |
| **Vin** | VBUS | Pin 40 | 5V Main Power Supply (USB Raw Power) | Dedicated |
| **GND** | GND | Pin 8 | System Ground | Shared |
| **BCLK** | GP13 | Pin 17 | Bit Clock Input | **Shared Clock** |
| **LRC** | GP14 | Pin 19 | Left-Right Clock Input | **Shared Clock** |
| **DIN** | GP12 | Pin 16 | Data Input (Pico to Amplifier) | Dedicated Output |
| **GAIN** | *Unconnected* | N/A | Default Gain Setting (9dB) | Floating |
| **SD** | *Unconnected* | N/A | Shutdown / Channel Select (Mono Mix) | Floating |

---
