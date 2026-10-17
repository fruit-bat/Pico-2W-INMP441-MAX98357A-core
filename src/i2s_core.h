#ifndef I2S_CORE_H
#define I2S_CORE_H

#include "pico/stdlib.h"

#define I2S_SAMPLE_RATE  (44100)
#define I2S_BUFFER_SIZE  1024 // Number of 32-bit samples per buffer slice

// Pin Configurations matching your prototype
#define I2S_PIN_BCLK     13
#define I2S_PIN_WS       14
#define I2S_PIN_MIC_SD   11
#define I2S_PIN_AMP_DIN  12

// Core Driver Functions
void i2s_core_init(void);
void i2s_core_start(void);
void i2s_core_stop(void);

// Returns the buffer core code should fill (TX) or read (RX) next.
// Call immediately after the corresponding block count has changed; the
// buffer must then be filled (TX) or consumed (RX) within one block period
// (I2S_BUFFER_SIZE / I2S_SAMPLE_RATE, about 23 ms), before DMA reuses it.
int32_t *get_tx_buffer(void);
int32_t *get_rx_buffer(void);

// Free-running counts of completed DMA blocks. A change means a new buffer is
// available; a jump of more than one means blocks were missed. The count is
// updated after the buffer index, so reading the count and then the buffer
// is safe from another core.
uint32_t i2s_core_get_tx_block_count(void);
uint32_t i2s_core_get_rx_block_count(void);

#endif // I2S_CORE_H
