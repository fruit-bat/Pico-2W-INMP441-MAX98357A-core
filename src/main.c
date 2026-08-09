#include <stdio.h>
#include "pico/stdlib.h"
#include "i2s_core.h"

// Diagnostic state variables
static volatile bool loopback_enabled = true;
static int32_t shared_dsp_buffer[I2S_BUFFER_SIZE];

// Interrupt Hook: Invoked automatically when the INMP441 fills a memory chunk
void i2s_callback_rx_ready(const int32_t *buffer, size_t size) {
    if (loopback_enabled) {
        // Core diagnostic loopback copy (Mic -> Speaker)
        // High 24-bit alignment configuration handles components natively
        for(size_t i = 0; i < size; i++) {
            shared_dsp_buffer[i] = buffer[i];
        }
    }
}

// Interrupt Hook: Invoked automatically when the MAX98357A requests audio bytes
void i2s_callback_tx_demanded(int32_t *buffer, size_t size) {
    if (loopback_enabled) {
        // Output the captured microphone data instantly
        for(size_t i = 0; i < size; i++) {
            buffer[i] = shared_dsp_buffer[i];
        }
    } else {
        // Mute state
        for(size_t i = 0; i < size; i++) {
            buffer[i] = 0;
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Allow hardware lines to settle post-connection
    
    printf("=== Pico 2 W (RP2350) I2S Audio Core Test System ===\n");
    printf("[CORE] Initializing DMA and PIO peripherals...\n");
    
    i2s_core_init();
    
    printf("[CORE] Dynamic full-duplex loops running... Speak into Mic!\n");
    i2s_core_start();
    
    // Standard execution loop for monitoring system state via UART/USB
    while (true) {
        tight_loop_contents();
    }
    
    return 0;
}
