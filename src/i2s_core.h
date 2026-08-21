#ifndef I2S_CORE_H
#define I2S_CORE_H

#include "pico/stdlib.h"

#define I2S_SAMPLE_RATE  44100
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

int32_t *get_tx_buffer(void);
int32_t *get_rx_buffer(void);

#endif // I2S_CORE_H
