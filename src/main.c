#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "i2s_core.h"
#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, etc.)

#define MIC_LEVEL_PRINT_PERIOD_MS 100
#define BAR_WIDTH 100
#define TONE_AMPLITUDE 100000000
#define TONE_STARTUP_STEPS 32

enum test_mode {
    MODE_LOOPBACK = 0,
    MODE_MIC_LEVEL = 1,
    MODE_TONE_440 = 2,
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

// Must be a supported power of 2 (32 to 4096)
#define FFT_SIZE I2S_BUFFER_SIZE
float32_t fft_output_buffer[FFT_SIZE];
float32_t fft_magnitude_buffer[FFT_SIZE / 2];

float32_t* __not_in_flash_func(fft_mic_input_buffer)(float32_t* input_buffer) {
    // 1. Create and initialize the Real FFT instance
    arm_rfft_fast_instance_f32 fft_instance;
    arm_status status = arm_rfft_fast_init_1024_f32(&fft_instance);
    
    if (status != ARM_MATH_SUCCESS) {
        // Initialization error handling (e.g., unsupported FFT size)
        printf("Error initializing FFT: %d\n    ", status);
        return &fft_magnitude_buffer[0]; // Return empty buffer on error
    }

    // 3. Execute the Forward FFT
    // The last parameter '0' indicates a forward transform (1 would mean inverse)
    arm_rfft_fast_f32(&fft_instance, input_buffer, fft_output_buffer, 0);

    // 4. Calculate the frequency magnitudes 
    // This processes the interleaved complex data into real amplitudes
    arm_cmplx_mag_f32(fft_output_buffer, fft_magnitude_buffer, FFT_SIZE / 2);
    return &fft_magnitude_buffer[0];
}

static void print_bar(float percent) {
    int count = (percent * BAR_WIDTH) / 100;
    printf("[MIC] %3.2f%% |", percent);
    for (int i = 0; i < BAR_WIDTH; ++i) {
        putchar(i < count ? '#' : ' ');
    }
    printf("|\n");
}

static void __not_in_flash_func(update_mic_level_stats)(const float *buffer, size_t size) {
    float sum_sq = 0.0f;
    float peak = 0;

    for (size_t i = 0; i < size; ++i) {
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

static void __not_in_flash_func(fill_tone_buffer)(int32_t *buffer, size_t size) {
    const float phase_step = (2.0f * (float)M_PI * 440.0f) / (float)I2S_SAMPLE_RATE;
    const float two_pi = 2.0f * (float)M_PI;
    float gain = 1.0f;

    if (tone_step_index < TONE_STARTUP_STEPS) {
        gain = (float)tone_step_index / (float)TONE_STARTUP_STEPS;
        tone_step_index++;
    }

    for (size_t i = 0; i < size; ++i) {
        float sine = sinf(tone_phase);
        int32_t sample = (int32_t)(sine * (float)TONE_AMPLITUDE * gain);
        buffer[i] = sample;
        tone_phase += phase_step;
        if (tone_phase >= two_pi) {
            tone_phase -= two_pi;
        }
    }
}



// Interrupt Hook: Invoked automatically when the INMP441 fills a memory chunk
void __not_in_flash_func(i2s_callback_rx_ready)(const float *buffer, size_t size) {
    update_mic_level_stats(buffer, size);
    fft_mic_input_buffer((float32_t *)buffer);
}

// Interrupt Hook: Invoked automatically when the MAX98357A requests audio bytes
void __not_in_flash_func(i2s_callback_tx_demanded)(int32_t *buffer, size_t size) {
    switch (current_mode) {
        case MODE_LOOPBACK:
            if (loopback_enabled) {
                for (size_t i = 0; i < size; ++i) {
                    buffer[i] = shared_dsp_buffer[i];
                }
            } else {
                for (size_t i = 0; i < size; ++i) {
                    buffer[i] = 0;
                }
            }
            break;

        case MODE_MIC_LEVEL:
            for (size_t i = 0; i < size; ++i) {
                buffer[i] = 0;
            }
            break;

        case MODE_TONE_440:
            fill_tone_buffer(buffer, size);
            break;
    }
}

float* __not_in_flash_func(rx_buffer_int32_to_float)(int32_t* int32_buf, uint32_t size) {
    static float float_buf[I2S_BUFFER_SIZE];   // Just re use the shared_dsp_buffer for float conversion to avoid dynamic allocation
    for (uint32_t i = 0; i < size; ++i) {
        float_buf[i] = ((float)(int32_buf[i])) / 2147483648.0f; // Convert to float in range [-1.0, 1.0]
    }
    return float_buf;
}

void core1_main() {
    static int32_t *tx_buffer = NULL;
    static int32_t *rx_buffer = NULL;

    while(1) {
        int32_t *t_tx_buffer = get_tx_buffer();
        int32_t *t_rx_buffer = get_rx_buffer();

        if (tx_buffer != t_tx_buffer) {
            tx_buffer = t_tx_buffer;
            i2s_callback_tx_demanded(tx_buffer, I2S_BUFFER_SIZE);
        }
        if (rx_buffer != t_rx_buffer) {
            rx_buffer = t_rx_buffer;
            float *float_buffer = rx_buffer_int32_to_float(rx_buffer, I2S_BUFFER_SIZE);
            i2s_callback_rx_ready(float_buffer, I2S_BUFFER_SIZE);
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
    printf("[CORE] Initializing DMA and PIO peripherals...\n");
    printf("[CORE] Tone output uses a 440Hz sine wave with soft startup ramp.\n");

    i2s_core_init();

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
            print_bar(percent);
            last_print_ms = now_ms;
        }

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

        tight_loop_contents();
    }

    return 0;
}
