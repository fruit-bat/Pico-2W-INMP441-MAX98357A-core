#include "i2s_core.h"
#include "audio_i2s_setup.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <string.h>

// Completed DMA block counts (see i2s_core_get_tx/rx_block_count)
static volatile uint32_t g_i2s_tx_dma_count = 0;
static volatile uint32_t g_i2s_rx_dma_count = 0;

// PIO FIFOs the DMA channels service. Both directions use the same state
// machine (see audio_i2s_setup.h); they are kept separate here for clarity.
static PIO tx_pio;
static PIO rx_pio;
static uint tx_sm;
static uint rx_sm;

// Two chained DMA channels per direction. Channel [i] always transfers
// buffer [i] and, when it finishes, the hardware immediately triggers the
// other channel, so the PIO FIFOs are fed without any gap. The interrupt only
// has to rewind the finished channel before the other one completes.
static uint dma_tx_chan[2];
static uint dma_rx_chan[2];
static bool i2s_initialized = false;

// Ping-Pong Double Buffers to ensure zero data-loss during processing
static int32_t tx_buffers[2][I2S_BUFFER_SIZE];
static int32_t rx_buffers[2][I2S_BUFFER_SIZE];

// Index of the buffer most recently released by DMA: free to refill (TX) or
// holding a complete block (RX)
static volatile uint32_t tx_buf_idx = 0;
static volatile uint32_t rx_buf_idx = 0;

int32_t *__not_in_flash_func(get_tx_buffer)(void) {
    return tx_buffers[tx_buf_idx];
}

int32_t *__not_in_flash_func(get_rx_buffer)(void) {
    return rx_buffers[rx_buf_idx];
}

uint32_t __not_in_flash_func(i2s_core_get_tx_block_count)(void) {
    return g_i2s_tx_dma_count;
}

uint32_t __not_in_flash_func(i2s_core_get_rx_block_count)(void) {
    return g_i2s_rx_dma_count;
}

// Internal DMA IRQ Handler
static void __not_in_flash_func(i2s_dma_irq_handler)() {
    const uint32_t ints = dma_hw->ints0;

    for (uint32_t i = 0; i < 2; i++) {
        const uint32_t tx_mask = 1u << dma_tx_chan[i];
        if (ints & tx_mask) {
            dma_hw->ints0 = tx_mask; // Clear interrupt flag

            // Channel i has played tx_buffers[i] and chained to the other channel.
            // Rewind it for its next turn; the transfer count reloads on trigger.
            dma_channel_set_read_addr(dma_tx_chan[i], tx_buffers[i], false);
            tx_buf_idx = i;

            // Publish the new buffer index before the count that announces it
            __dmb();
            g_i2s_tx_dma_count++;
        }

        const uint32_t rx_mask = 1u << dma_rx_chan[i];
        if (ints & rx_mask) {
            dma_hw->ints0 = rx_mask; // Clear interrupt flag

            // Channel i has filled rx_buffers[i] and chained to the other channel.
            dma_channel_set_write_addr(dma_rx_chan[i], rx_buffers[i], false);
            rx_buf_idx = i;

            // Publish the new buffer index before the count that announces it
            __dmb();
            g_i2s_rx_dma_count++;
        }
    }
}

void i2s_core_init(void) {
    if (i2s_initialized) {
        printf("[I2S] Already initialized; skipping DMA/PIO setup.\n");
        return;
    }

    memset(tx_buffers, 0, sizeof(tx_buffers));
    memset(rx_buffers, 0, sizeof(rx_buffers));
    tx_buf_idx = 0;
    rx_buf_idx = 0;
    g_i2s_tx_dma_count = 0;
    g_i2s_rx_dma_count = 0;

    printf("[I2S] Initializing PIO clocking and DMA at %lu Hz...\n", (unsigned long)I2S_SAMPLE_RATE);

    // 1. PIO Engine Configuration Setup
    audio_i2s_hardware_init(I2S_SAMPLE_RATE);
    // A single state machine drives the speaker and samples the mic, so RX
    // reads from the same state machine's RX FIFO.
    tx_pio = i2s_get_pio();
    rx_pio = i2s_get_pio();
    tx_sm = i2s_get_sm();
    rx_sm = i2s_get_sm();
    printf("[I2S] PIO config: tx_pio=%p sm=%u rx_pio=%p sm=%u\n", (void *)tx_pio, tx_sm, (void *)rx_pio, rx_sm);

    // 2. Allocate DMA Channels
    for (uint32_t i = 0; i < 2; i++) {
        dma_tx_chan[i] = (uint)dma_claim_unused_channel(true);
        dma_rx_chan[i] = (uint)dma_claim_unused_channel(true);
    }
    printf("[I2S] Allocated DMA channels: TX=%u,%u RX=%u,%u\n",
        dma_tx_chan[0], dma_tx_chan[1], dma_rx_chan[0], dma_rx_chan[1]);

    for (uint32_t i = 0; i < 2; i++) {
        // 3. Configure TX DMA (Memory to Peripheral), chained to the other TX channel
        dma_channel_config c_tx = dma_channel_get_default_config(dma_tx_chan[i]);
        channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_32);
        channel_config_set_read_increment(&c_tx, true);
        channel_config_set_write_increment(&c_tx, false);
        channel_config_set_dreq(&c_tx, pio_get_dreq(tx_pio, tx_sm, true));
        channel_config_set_chain_to(&c_tx, dma_tx_chan[1 - i]);

        dma_channel_configure(
            dma_tx_chan[i], &c_tx,
            &tx_pio->txf[tx_sm], // Write address: PIO FIFO
            tx_buffers[i],        // Read address: this channel's buffer
            I2S_BUFFER_SIZE,      // Transfer count (reloaded on every trigger)
            false                 // Do not start yet
        );

        // 4. Configure RX DMA (Peripheral to Memory), chained to the other RX channel
        dma_channel_config c_rx = dma_channel_get_default_config(dma_rx_chan[i]);
        channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_32);
        channel_config_set_read_increment(&c_rx, false);
        channel_config_set_write_increment(&c_rx, true);
        channel_config_set_dreq(&c_rx, pio_get_dreq(rx_pio, rx_sm, false));
        channel_config_set_chain_to(&c_rx, dma_rx_chan[1 - i]);

        dma_channel_configure(
            dma_rx_chan[i], &c_rx,
            rx_buffers[i],        // Write address: this channel's buffer
            &rx_pio->rxf[rx_sm], // Read address: PIO FIFO
            I2S_BUFFER_SIZE,      // Transfer count (reloaded on every trigger)
            false                 // Do not start yet
        );

        // 5. Interrupt on completion of every channel
        dma_channel_set_irq0_enabled(dma_tx_chan[i], true);
        dma_channel_set_irq0_enabled(dma_rx_chan[i], true);
    }

    // 6. Wire Handlers to RP2350 Nested Vectored Interrupt Controller (NVIC)
    irq_set_exclusive_handler(DMA_IRQ_0, i2s_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    i2s_initialized = true;
    printf("[I2S] DMA/PIO initialization complete.\n");
}

void i2s_core_start(void) {
    if (!i2s_initialized) {
        printf("[I2S] Start requested before initialization; initializing now.\n");
        i2s_core_init();
    }

    printf("[I2S] Starting I2S TX/RX state machines...\n");

    pio_enable_sm_mask_in_sync(tx_pio, 1u << tx_sm);

    // Start the first channel of each direction together; chaining does the rest
    dma_start_channel_mask((1u << dma_tx_chan[0]) | (1u << dma_rx_chan[0]));

    printf("[I2S] TX and RX DMA started.\n");
}

void i2s_core_stop(void) {
    irq_set_enabled(DMA_IRQ_0, false);

    // RP2350-E5: clear the enable bit of every chained channel before aborting,
    // otherwise an aborted channel can still trigger its partner.
    uint32_t mask = 0;
    for (uint32_t i = 0; i < 2; i++) {
        const uint chans[2] = { dma_tx_chan[i], dma_rx_chan[i] };
        for (uint32_t j = 0; j < 2; j++) {
            hw_clear_bits(&dma_hw->ch[chans[j]].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
            mask |= 1u << chans[j];
        }
    }

    dma_hw->abort = mask;
    while (dma_hw->abort & mask) {
        tight_loop_contents();
    }

    // Acknowledge anything raised while stopping
    dma_hw->ints0 = mask;
}
