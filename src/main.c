#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "i2s_core.h"

#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, etc.)
#include "arm_const_structs.h"

#include "chirp_decoder.h"
#include "chirp_encoder.h"
#include "chirp_symbol_rx.h"
#include "fft_visualise.h"
#include "frame_snapshot.h"
#include "rx_sample_window.h"
#include "spsc_ring.h"

#define MAX_SYMBOLS 64u
// TODO Should we consider separating 'max symbols' from 'bins in bandwidth' ?
// TODO perhaps hwe need 'bins per symbol' and 'bins per bandwidth' as separate parameters.

// Currently must be 1024
#define FFT_SIZE I2S_BUFFER_SIZE

static const float32_t FS = I2S_SAMPLE_RATE;
static const float32_t BPB = FS / (float32_t)FFT_SIZE; // Bandwidth per FFT bin
static const float32_t BW = BPB * MAX_SYMBOLS; // Total bandwidth for the chirp signal
static const float32_t F0 = 15000.0f;
static const float32_t T  = (float32_t)FFT_SIZE / FS;
static const float32_t chirp_rate = BW / T;
static const float32_t chirp_vol = 0.005f;

#define DISPLAY_PERIOD_MS 100
#define TONE_STARTUP_STEPS 32
#define RX_RECORD_QUEUE_CAPACITY 64u // Must be a power of two

enum test_mode {
    MODE_LOOPBACK = 0,
    MODE_MIC_LEVEL = 1,
    MODE_TONE_440 = 2,
    MODE_TONE_CHIRP = 3,
};

// ---------------------------------------------------------------------------
// Shared between cores.
// Core 0 (UI) writes these single-word controls; core 1 (DSP) reads them.
// ---------------------------------------------------------------------------
static volatile enum test_mode current_mode = MODE_TONE_440;
static volatile uint32_t tx_symbol = 0;        // Symbol transmitted in chirp modes
static volatile uint32_t tone_step_index = 0;  // Reset by core 0 to restart the tone ramp

// Core 1 writes these counters; core 0 reads them.
static volatile uint32_t rx_blocks_missed = 0;  // RX blocks not processed in time
static volatile uint32_t tx_blocks_missed = 0;  // TX blocks not refilled in time
static volatile uint32_t rx_records_dropped = 0; // Records lost because the queue was full

//
// One record per received frame, passed from core 1 to core 0.
// This is the receiver's output: today it is displayed, later it can be
// streamed to a host or fed to a packet decoder.
//
typedef struct {
    uint32_t frame;          // RX block count when this frame was decoded
    uint32_t delay;          // Alignment delay in samples used for this frame
    uint32_t tx_symbol;      // Symbol being transmitted at the time (useful for loopback/testing)
    float32_t peak;          // Peak absolute sample value in the frame, [0, 1]
    ChirpSymbolRx_t symbol;  // Decoded symbol
} RxFrameRecord_t;

static uint8_t rx_record_storage[SPSC_RING_STORAGE_LEN(sizeof(RxFrameRecord_t), RX_RECORD_QUEUE_CAPACITY)];
static SpscRing_t rx_record_queue;

// Consistent copy of the latest FFT magnitude spectrum for display
static float32_t spectrum_snapshot_buffer[FFT_SIZE];
static FrameSnapshot_t spectrum_snapshot;

// Double-buffered mic input so we can phase align with the input signal.
// Core 1 writes samples; core 0 may adjust the delay.
static float32_t rx_window_storage[RX_SAMPLE_WINDOW_STORAGE_LEN(FFT_SIZE)];
static RxSampleWindow_t rx_window;

// ---------------------------------------------------------------------------
// Core 1 only.
// ---------------------------------------------------------------------------

// The aligned frame passed to the decoder
static float32_t rx_frame[FFT_SIZE];

// Statically allocated storage for the chirp decoder (no malloc)
static float32_t complex_dechirp_vector[CHIRP_DECODER_COMPLEX_LEN(FFT_SIZE)];
static float32_t fft_output_buffer[CHIRP_DECODER_COMPLEX_LEN(FFT_SIZE)];
static float32_t fft_magnitude_buffer[FFT_SIZE];
static float32_t cos_table[MAX_SYMBOLS];
static float32_t sin_table[MAX_SYMBOLS];

static ChirpDecoderState_t chirp_decoder;
static ChirpEncoder_t chirp_encoder;
static ChirpSymbolRxConfig_t symbol_rx_config;

static float tone_phase = 0.0f;

// ---------------------------------------------------------------------------
// Core 0 only.
// ---------------------------------------------------------------------------
static FftVisualise_t fft_visualise;

static void init_audio_system(void) {
    const ChirpParams_t chirp_params = {
        .sample_rate = FS,
        .f0 = F0,
        .chirp_rate = chirp_rate,
    };

    chirp_decoder_init(
        &chirp_decoder,
        &chirp_params,
        FFT_SIZE,
        &arm_cfft_sR_f32_len1024, // The 1024-point complex instance is globally defined by CMSIS-DSP
        4, // Power exponent for weighting
        MAX_SYMBOLS,
        complex_dechirp_vector,
        fft_output_buffer,
        fft_magnitude_buffer,
        cos_table,
        sin_table
    );

    // The transmitter shares the decoder's chirp table
    chirp_encoder_init(
        &chirp_encoder,
        chirp_decoder.complex_dechirp_vector,
        FFT_SIZE,
        MAX_SYMBOLS,
        chirp_vol
    );

    chirp_symbol_rx_config_init(&symbol_rx_config, MAX_SYMBOLS);
}

static const char *mode_name(enum test_mode mode) {
    switch (mode) {
        case MODE_LOOPBACK:   return "loopback";
        case MODE_MIC_LEVEL:  return "mic-level";
        case MODE_TONE_440:   return "tone-440";
        case MODE_TONE_CHIRP: return "chirp";
    }
    return "unknown";
}

static void set_mode(enum test_mode mode) {
    current_mode = mode;
    tone_step_index = 0;
    printf("[MODE] Selected mode: %s\n", mode_name(mode));
}

static float32_t __not_in_flash_func(peak_abs)(const float32_t *buffer, uint32_t size) {
    float32_t peak = 0.0f;
    for (uint32_t i = 0; i < size; ++i) {
        const float32_t mag = fabsf(buffer[i]);
        if (mag > peak) {
            peak = mag;
        }
    }
    return peak;
}

static void __not_in_flash_func(fill_tone_buffer)(float *buffer) {
    const float phase_step = (2.0f * (float)M_PI * 440.0f) / (float)I2S_SAMPLE_RATE;
    const float two_pi = 2.0f * (float)M_PI;
    float gain = 0.1f;

    const uint32_t step = tone_step_index;
    if (step < TONE_STARTUP_STEPS) {
        gain *= (float)step / (float)TONE_STARTUP_STEPS;
        tone_step_index = step + 1;
    }

    for (size_t i = 0; i < I2S_BUFFER_SIZE; ++i) {
        float sine = sinf(tone_phase);
        float sample = sine * gain; // Scale down to avoid clipping
        buffer[i] = sample;
        tone_phase += phase_step;
        if (tone_phase >= two_pi) {
            tone_phase -= two_pi;
        }
    }
}

// Core 1: decode the newest block in the RX window and publish the result.
static void __not_in_flash_func(process_rx_block)(uint32_t frame) {
    rx_sample_window_read(&rx_window, rx_frame);

    RxFrameRecord_t record = {
        .frame = frame,
        .delay = rx_sample_window_get_delay(&rx_window),
        .tx_symbol = tx_symbol,
        .peak = peak_abs(rx_frame, FFT_SIZE),
    };

    chirp_decoder_real_chirp_to_symbol(
        &chirp_decoder,
        rx_frame,
        &symbol_rx_config,
        &record.symbol
    );

    if (!spsc_ring_push(&rx_record_queue, &record)) {
        rx_records_dropped++;
    }

    frame_snapshot_offer(&spectrum_snapshot, chirp_decoder.fft_magnitude_buffer);
}

// Core 1: produce the next block of audio to transmit.
static void __not_in_flash_func(fill_tx_block)(float32_t *buffer) {
    switch (current_mode) {
        case MODE_LOOPBACK:
        case MODE_TONE_CHIRP:
            chirp_encoder_symbol_to_samples(&chirp_encoder, tx_symbol, buffer);
            break;

        case MODE_MIC_LEVEL:
            memset(buffer, 0, I2S_BUFFER_SIZE * sizeof(*buffer));
            break;

        case MODE_TONE_440:
            fill_tone_buffer(buffer);
            break;
    }
}

// Core 1: real-time audio loop. Services each DMA block as it completes.
static void __not_in_flash_func(core1_main)(void) {
    static float32_t tx_block[I2S_BUFFER_SIZE];

    init_audio_system();

    uint32_t last_tx_count = i2s_core_get_tx_block_count();
    uint32_t last_rx_count = i2s_core_get_rx_block_count();

    while (1) {
        const uint32_t tx_count = i2s_core_get_tx_block_count();
        if (tx_count != last_tx_count) {
            tx_blocks_missed += tx_count - last_tx_count - 1;
            last_tx_count = tx_count;

            int32_t *tx_buffer = get_tx_buffer();
            fill_tx_block(tx_block);

            if (current_mode == MODE_LOOPBACK) {
                // Send silence and feed the generated audio straight to the receiver
                memset(tx_buffer, 0, I2S_BUFFER_SIZE * sizeof(*tx_buffer));
                memcpy(rx_sample_window_next_block(&rx_window), tx_block, I2S_BUFFER_SIZE * sizeof(*tx_block));
                process_rx_block(tx_count);
            } else {
                // fast vector conversion using optimized CMSIS assembly loops
                arm_float_to_q31(tx_block, (q31_t *)tx_buffer, I2S_BUFFER_SIZE);
            }
        }

        const uint32_t rx_count = i2s_core_get_rx_block_count();
        if (rx_count != last_rx_count) {
            rx_blocks_missed += rx_count - last_rx_count - 1;
            last_rx_count = rx_count;

            if (current_mode != MODE_LOOPBACK) {
                // fast vector conversion using optimized CMSIS assembly loops
                arm_q31_to_float(
                    (q31_t *)get_rx_buffer(),
                    rx_sample_window_next_block(&rx_window),
                    I2S_BUFFER_SIZE);
                process_rx_block(rx_count);
            }
        }
    }
}

// Core 0: redraw the terminal display. Always prints the same number of rows.
static void draw_display(const float32_t *spectrum, const RxFrameRecord_t *record, uint32_t records_received) {
    fft_visualise_draw(&fft_visualise, spectrum);

    printf("Mode %-9s  TX symbol %3lu of %lu  RX delay %4lu  peak %5.1f%%          \n",
        mode_name(current_mode),
        (unsigned long)tx_symbol,
        (unsigned long)MAX_SYMBOLS,
        (unsigned long)record->delay,
        record->peak * 100.0f);
    printf("RX symbol %3lu  centroid %8.4f  error %+.4f rad  strength %e          \n",
        (unsigned long)record->symbol.symbol_index,
        record->symbol.angle * symbol_rx_config.symbols_per_radian,
        record->symbol.error_angle,
        record->symbol.strength);

    for (uint32_t i = 0; i < MAX_SYMBOLS; ++i) {
        putchar(i == record->symbol.symbol_index ? '*' : '.');
    }
    putchar('\n');

    printf("Frames %lu  missed rx %lu tx %lu  dropped records %lu          \n",
        (unsigned long)records_received,
        (unsigned long)rx_blocks_missed,
        (unsigned long)tx_blocks_missed,
        (unsigned long)rx_records_dropped);
    printf("Keys: 1-4 mode  o/p symbol  z/x delay  c/v page\n");
}

static void handle_key(int ch) {
    switch (ch) {
        case '1':
            set_mode(MODE_LOOPBACK);
            break;
        case '2':
            set_mode(MODE_MIC_LEVEL);
            break;
        case '3':
            set_mode(MODE_TONE_440);
            break;
        case '4':
            set_mode(MODE_TONE_CHIRP);
            break;
        case 'p':
            tx_symbol = (tx_symbol + 1) % MAX_SYMBOLS;
            break;
        case 'o':
            tx_symbol = (tx_symbol == 0) ? (MAX_SYMBOLS - 1) : (tx_symbol - 1);
            break;
        case 'z': {
            const uint32_t delay = rx_sample_window_get_delay(&rx_window);
            rx_sample_window_set_delay(&rx_window, delay > 0 ? delay - 1 : (FFT_SIZE - 1));
            break;
        }
        case 'x': {
            const uint32_t delay = rx_sample_window_get_delay(&rx_window);
            rx_sample_window_set_delay(&rx_window, delay < (FFT_SIZE - 1) ? delay + 1 : 0);
            break;
        }
        case 'c':
            fft_visualise_page_prev(&fft_visualise);
            break;
        case 'v':
            fft_visualise_page_next(&fft_visualise);
            break;
        default:
            break;
    }
}

int main() {

    // Set clock to 200 MHz (200,000 kHz)
    set_sys_clock_khz(200000, true);

    stdio_init_all();
    sleep_ms(2000);

    printf("=== Pico 2 W (RP2350) I2S Audio Core Test System ===\n");
    printf("[CORE] Available modes:\n");
    printf("  1 = loopback test\n");
    printf("  2 = mic level meter\n");
    printf("  3 = 440Hz sine tone\n");
    printf("  4 = chirp tone\n");
    printf("[CORE] Initializing DMA and PIO peripherals...\n");

    i2s_core_init();

    // Everything shared between the cores is initialized before core 1 starts
    spsc_ring_init(&rx_record_queue, rx_record_storage, sizeof(RxFrameRecord_t), RX_RECORD_QUEUE_CAPACITY);
    frame_snapshot_init(&spectrum_snapshot, spectrum_snapshot_buffer, FFT_SIZE);
    rx_sample_window_init(
        &rx_window,
        rx_window_storage,
        FFT_SIZE,
        967 // Initial delay in samples to phase align with the input signal
    );

    fft_visualise_init(
        &fft_visualise,
        FFT_SIZE,
        FS,
        50,      // Bins per page
        1500.0f, // Magnitude to bar length; tune for microphone gain
        120      // Max bar width in characters
    );

    set_mode(MODE_TONE_440);
    printf("[CORE] Starting in tone validation mode to isolate the speaker output path.\n");
    i2s_core_start();

    // Start core 1
    multicore_launch_core1(core1_main);

    RxFrameRecord_t latest_record = {0};
    uint32_t records_received = 0;
    uint32_t last_display_ms = 0;

    while (true) {
        int ch = getchar_timeout_us(500);
        if (ch != PICO_ERROR_TIMEOUT) {
            handle_key(ch);
        }

        // Drain every record from the receiver, keeping the latest for display
        RxFrameRecord_t record;
        while (spsc_ring_pop(&rx_record_queue, &record)) {
            latest_record = record;
            records_received++;
        }

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms - last_display_ms >= DISPLAY_PERIOD_MS) {
            const float32_t *spectrum = frame_snapshot_acquire(&spectrum_snapshot);
            if (spectrum != NULL) {
                draw_display(spectrum, &latest_record, records_received);
                frame_snapshot_release(&spectrum_snapshot);
                last_display_ms = now_ms;
            }
        }

        tight_loop_contents();
    }

    return 0;
}
