#ifndef AUDIO_I2S_SETUP_H
#define AUDIO_I2S_SETUP_H

#include <stdint.h>
#include "hardware/pio.h"

//
// Loads the I2S PIO program and configures a single state machine that clocks
// the bus (BCLK/WS), drives the amplifier (DIN) and samples the microphone (SD).
// TX data goes to the state machine's TX FIFO and mic data arrives in its RX FIFO.
//
void audio_i2s_hardware_init(uint32_t sample_rate);

// The PIO block and state machine claimed by audio_i2s_hardware_init().
PIO i2s_get_pio(void);
uint i2s_get_sm(void);

#endif // AUDIO_I2S_SETUP_H
