#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "i2s_core.h"

#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, etc.)
#include "arm_const_structs.h"

#include "chirp_decoder.h"
#include "chirp_symbol_rx.h"
#include "fft_visualise.h"
#include "rx_sample_window.h"

#define MAX_SYMBOLS 64u
// TODO Should we consider separating 'max symbols' from 'bins in bandwidth' ?
// TODO perhaps hwe need 'bins per symbol' and 'bins per bandwidth' as separate parameters.

const float32_t FS = I2S_SAMPLE_RATE;
const float32_t BPB = FS / (float32_t)I2S_BUFFER_SIZE; // Bandwidth per FFT bin
const float32_t BW = BPB * MAX_SYMBOLS; // Total bandwidth for the chirp signal
const float32_t F0 = 15000.0f;
const float32_t F1 = F0 + BW;
const float32_t T  = (float32_t)I2S_BUFFER_SIZE / FS;
const float32_t chirp_rate = BW / T;
const float32_t chirp_vol = 0.005f;

#define MIC_LEVEL_PRINT_PERIOD_MS 100
#define BAR_WIDTH 100
#define TONE_STARTUP_STEPS 32

enum test_mode {
    MODE_LOOPBACK = 0,
    MODE_MIC_LEVEL = 1,
    MODE_TONE_440 = 2,
    MODE_TONE_CHIRP = 3,
};

extern volatile uint32_t g_i2s_tx_dma_count;
extern volatile uint32_t g_i2s_rx_dma_count;

static volatile enum test_mode current_mode = MODE_TONE_440;
static volatile bool loopback_enabled = true;
static int32_t shared_dsp_buffer[I2S_BUFFER_SIZE];
static int32_t rx_dsp_buffer[I2S_BUFFER_SIZE];
static volatile float mic_peak_value = 0;
static float tone_phase = 0.0f;
static uint32_t tone_step_index = 0;

// Currently must be 1024
#define FFT_SIZE I2S_BUFFER_SIZE

// Double-buffered mic input so we can phase align with the input signal
static float32_t rx_window_storage[RX_SAMPLE_WINDOW_STORAGE_LEN(FFT_SIZE)];
static RxSampleWindow_t rx_window;
// The aligned frame passed to the decoder
static float32_t rx_frame[FFT_SIZE];

// Statically allocated storage for the chirp decoder (no malloc)
static float32_t complex_dechirp_vector[CHIRP_DECODER_COMPLEX_LEN(FFT_SIZE)];
static float32_t fft_output_buffer[CHIRP_DECODER_COMPLEX_LEN(FFT_SIZE)];
static float32_t fft_magnitude_buffer[FFT_SIZE];
static float32_t cos_table[MAX_SYMBOLS];
static float32_t sin_table[MAX_SYMBOLS];

static ChirpDecoderState_t chirp_decoder;
static ChirpSymbolRxConfig_t symbol_rx_config;
static ChirpSymbolRx_t symbol_rx;

void generate_modulated_chirp(float32_t *tx_audio_buffer, uint32_t symbol_val) {
    const float32_t *dechirp = chirp_decoder.complex_dechirp_vector;
    uint32_t shift = (float32_t)symbol_val * FS / BW; // Shift in samples for the symbol value
    for (int n = 0; n < I2S_BUFFER_SIZE; n++) {
        tx_audio_buffer[n] = dechirp[((n + shift) % I2S_BUFFER_SIZE) * 2] * chirp_vol; // Apply the complex de-chirp and scale by volume
    }
}

void init_audio_system(void) {
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
        4.0f, // Power exponent for weighting
        MAX_SYMBOLS,
        complex_dechirp_vector,
        fft_output_buffer,
        fft_magnitude_buffer,
        cos_table,
        sin_table
    );

    chirp_symbol_rx_config_init(&symbol_rx_config, MAX_SYMBOLS);
}

static FftVisualise_t fft_visualise;



static void print_bar(float percent) {
    int count = (percent * BAR_WIDTH) / 100;
    printf("[MIC] %3.2f%% |", percent);
    for (int i = 0; i < BAR_WIDTH; ++i) {
        putchar(i < count ? '#' : ' ');
    }
    printf("|\n");
}

static void __not_in_flash_func(update_mic_level_stats)(const float *buffer) {
    float sum_sq = 0.0f;
    float peak = 0;

    for (size_t i = 0; i < I2S_BUFFER_SIZE; ++i) {
        float sample = buffer[i];
        float mag = fabsf(sample);
        if (mag > peak) {
            peak = mag;
        }
    }
    mic_peak_value = peak;
}

static void set_mode(enum test_mode mode) {
    current_mode = mode;
    loopback_enabled = (mode == MODE_LOOPBACK);
    printf("[MODE] Selected mode: %s\n",
           mode == MODE_LOOPBACK ? "loopback" :
           mode == MODE_MIC_LEVEL ? "mic-level" : "tone-440");
}

static void __not_in_flash_func(fill_tone_buffer)(float *buffer) {
    const float phase_step = (2.0f * (float)M_PI * 440.0f) / (float)I2S_SAMPLE_RATE;
    const float two_pi = 2.0f * (float)M_PI;
    float gain = 0.1f;

    if (tone_step_index < TONE_STARTUP_STEPS) {
        gain *= (float)tone_step_index / (float)TONE_STARTUP_STEPS;
        tone_step_index++;
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


// Interrupt Hook: Invoked automatically when the INMP441 fills a memory chunk
void __not_in_flash_func(i2s_callback_rx_ready)() {
    //update_mic_level_stats(buffer);
    rx_sample_window_read(&rx_window, rx_frame);

    chirp_decoder_real_chirp_to_symbol(
        &chirp_decoder,
        rx_frame,
        &symbol_rx_config,
        &symbol_rx
    );
}

static uint32_t symbol_index = 0; // Current symbol index for chirp modulation

// Interrupt Hook: Invoked automatically when the MAX98357A requests audio bytes
void __not_in_flash_func(i2s_callback_tx_demanded)(float *buffer) {
    const size_t size = I2S_BUFFER_SIZE;
    switch (current_mode) {

        case MODE_LOOPBACK:
            generate_modulated_chirp(buffer, symbol_index);
            break;

        case MODE_MIC_LEVEL:
            for (size_t i = 0; i < size; ++i) {
                buffer[i] = 0.0f;
            }
            break;

        case MODE_TONE_440:
            fill_tone_buffer(buffer);
            break;

        case MODE_TONE_CHIRP:
            generate_modulated_chirp(buffer, symbol_index);
            break;
    }
}


void core1_main() {
    static int32_t *tx_buffer = NULL;
    static int32_t *rx_buffer = NULL;
    static float float_buf[I2S_BUFFER_SIZE];

    init_audio_system(); // Initialize the FFT instance once at startup

    while(1) {
        int32_t *t_tx_buffer = get_tx_buffer();
        int32_t *t_rx_buffer = get_rx_buffer();

        switch (current_mode) {
            case MODE_LOOPBACK: {

                if (tx_buffer != t_tx_buffer) {
                    i2s_callback_tx_demanded(float_buf);
                    // arm_float_to_q31((float32_t *)float_buf, (q31_t *)t_tx_buffer, I2S_BUFFER_SIZE);
                    memset(t_tx_buffer, 0, I2S_BUFFER_SIZE * sizeof(int32_t)); // Clear TX buffer for loopback
                    tx_buffer = t_tx_buffer;
                    // Copy TX buffer to RX buffer for loopback
                    memcpy(
                        rx_sample_window_next_block(&rx_window),
                        (float32_t *)float_buf, 
                        I2S_BUFFER_SIZE * sizeof(float32_t));
                    i2s_callback_rx_ready();
                    rx_buffer = t_rx_buffer;
                }

                break;
            }
            default: {

                if (tx_buffer != t_tx_buffer) {
                    i2s_callback_tx_demanded(float_buf);
                    // fast vector conversion using optimized CMSIS assembly loops
                    arm_float_to_q31((float32_t *)float_buf, (q31_t *)t_tx_buffer, I2S_BUFFER_SIZE);
                    tx_buffer = t_tx_buffer;
                }

                if (rx_buffer != t_rx_buffer) {
                    // fast vector conversion using optimized CMSIS assembly loops
                    arm_q31_to_float(
                        (q31_t *)t_rx_buffer, 
                        rx_sample_window_next_block(&rx_window),
                        I2S_BUFFER_SIZE);

                    i2s_callback_rx_ready();
                    rx_buffer = t_rx_buffer;
                }

                break;
            }
        }
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
    printf("[CORE] Tone output uses a 440Hz sine wave with soft startup ramp.\n");

    i2s_core_init();

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

    uint32_t last_print_ms = 0;
    uint32_t last_status_ms = 0;
    while (true) {
        int ch = getchar_timeout_us(500);
        if (ch != PICO_ERROR_TIMEOUT) {
            switch (ch) {
                case '1':
                    set_mode(MODE_LOOPBACK);
                    tone_step_index = 0;
                    break;
                case '2':
                    set_mode(MODE_MIC_LEVEL);
                    break;
                case '3':
                    set_mode(MODE_TONE_440);
                    tone_step_index = 0;
                    break;
                case '4':
                    set_mode(MODE_TONE_CHIRP);
                    tone_step_index = 0;
                    break;
                case 'p':
                    symbol_index = (symbol_index + 1) % MAX_SYMBOLS;
                    break;
                case 'o':
                    symbol_index = (symbol_index == 0) ? (MAX_SYMBOLS - 1) : (symbol_index - 1);
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

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms - last_print_ms >= MIC_LEVEL_PRINT_PERIOD_MS) {
            float percent = mic_peak_value  * 100.0f;
            if (percent < 0.0) percent = 0.0;
            if (percent > 100.0) {
                printf("[MIC] Warning: Peak value exceeds 100%% (%3.2f) of full scale!\n", percent);
                percent = 100.0f;
            }
            fft_visualise_draw(&fft_visualise, fft_magnitude_buffer);
            //print_bar(percent);
            printf("Symbol %3lu of %lu RX delay %4ld RX symbol centroid: %3.4f Rx symbol strength: %ef     \n", 
                symbol_index, 
                MAX_SYMBOLS,
                rx_sample_window_get_delay(&rx_window),
                symbol_rx.angle * symbol_rx_config.symbols_per_radian,
                symbol_rx.strength
            );
            uint32_t rsym = symbol_rx.symbol_index;
            if (rsym >= MAX_SYMBOLS) rsym = 0;
            for (int i = 0; i < MAX_SYMBOLS; ++i) {
                putchar(i == rsym  ? '*' : '.');
            }
            last_print_ms = now_ms;
        }
/*
        if (now_ms - last_status_ms >= 1000U) {
            const char *mode_name = (current_mode == MODE_LOOPBACK) ? "loopback" :
                                    (current_mode == MODE_MIC_LEVEL) ? "mic-level" : "tone-440";
            printf("[STATUS] running=%s peak=%3.2f tx_dma=%lu rx_dma=%lu\n",
                   mode_name,
                   mic_peak_value * 100.0f,
                   (unsigned long)g_i2s_tx_dma_count,
                   (unsigned long)g_i2s_rx_dma_count);
            last_status_ms = now_ms;
        }
*/
        tight_loop_contents();
    }

    return 0;
}
