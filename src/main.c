#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "i2s_core.h"
#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, etc.)




const float32_t FS = I2S_SAMPLE_RATE;
const float32_t F0 = 2000.0f;           // 2 kHz boundary
const float32_t F1 = 14000.0f;          // 14 kHz boundary
const float32_t BW = (F1-F0);           // Total Bandwidth (F1 - F0)
const float32_t T  = (float32_t)I2S_BUFFER_SIZE / FS;
const float32_t chirp_rate = BW / T;
const float32_t chirp_vol = 0.1f;

// DYNAMIC DERIVATION OF SYMBOL SPACE:
// 1. Calculate how many physical FFT bins fit into the chosen acoustic bandwidth
const uint32_t BINS_IN_BANDWIDTH = (uint32_t)((BW * (float32_t)I2S_BUFFER_SIZE) / FS);

// 2. Derive the maximum safe data symbols by finding the next lowest power of 2.
// This prevents over-the-air signals from bleeding outside your 2kHz-14kHz window.
const uint32_t MAX_SYMBOLS = (BINS_IN_BANDWIDTH >= 256) ? 256 :
                             (BINS_IN_BANDWIDTH >= 128) ? 128 :
                             (BINS_IN_BANDWIDTH >= 64)  ? 64  : 32;

/**
 * Generates an acoustic chirp symbol with a cyclic frequency shift.
 * @param symbol_val: The data value to send (must be between 0 and BUFFER_SIZE - 1)
 */
void generate_modulated_chirp(float32_t *tx_audio_buffer, uint32_t symbol_val) {

    static float32_t global_tx_phase = 0.0f;
    
    // ADJUSTMENT: Map our symbol index (0...MAX_SYMBOLS - 1) onto the sample shift timeline.
    // Instead of shifting sample-by-sample, we shift by chunks so that each symbol 
    // lands cleanly on a discrete, readable FFT bin center at the receiver.
    float32_t sample_shift = ((float32_t)symbol_val / (float32_t)MAX_SYMBOLS) * (float32_t)I2S_BUFFER_SIZE;

    for (int n = 0; n < I2S_BUFFER_SIZE; n++) {
        // 1. Determine our position in the unshifted timeline (0 to BUFFER_SIZE - 1)
        float32_t unshifted_sample_idx = (float32_t)n;

        // 2. Apply the cyclic symbol shift and wrap it within the symbol block
        float32_t shifted_sample_idx = unshifted_sample_idx + sample_shift;
        if (shifted_sample_idx >= (float32_t)I2S_BUFFER_SIZE) {
            shifted_sample_idx -= (float32_t)I2S_BUFFER_SIZE;
        }

        // 3. Calculate target instantaneous frequency at this specific shifted index
        // f_inst = F0 + (chirp_rate * t)
        float32_t t_shifted = shifted_sample_idx / FS;
        float32_t f_inst = F0 + (chirp_rate * t_shifted);

        // 4. Convert instantaneous frequency to a phase increment for this single sample
        // phase_delta = 2 * pi * f / fs
        float32_t phase_delta = (2.0f * M_PI * f_inst) / FS;

        // 5. Accumulate into our global transmitter phase
        global_tx_phase += phase_delta;

        // 6. Keep the accumulator bounded between -pi and +pi to maintain float32 precision
        if (global_tx_phase > M_PI) {
            global_tx_phase -= (2.0f * M_PI);
        } 
        else if (global_tx_phase < -M_PI) {
            global_tx_phase += (2.0f * M_PI);
        }

        // 7. Generate the clean, glitch-free sine sample
        float32_t float_sample = sinf(global_tx_phase);

        tx_audio_buffer[n] = float_sample * chirp_vol;
    }
}








#define MIC_LEVEL_PRINT_PERIOD_MS 100
#define BAR_WIDTH 100
#define TONE_AMPLITUDE (1<<30)
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

// 2. Output buffer must be the same size as input for CMSIS-DSP's packed format
float32_t fft_output_buffer[FFT_SIZE];

// 3. Magnitude buffer needs 513 elements to capture DC through Nyquist
float32_t fft_magnitude_buffer[(FFT_SIZE / 2) + 1]; 


// 1. Initialize the instance GLOBALLY once at startup, NOT inside the function
static arm_rfft_fast_instance_f32 fft_instance;

void init_audio_system(void) {
    arm_status status = arm_rfft_fast_init_1024_f32(&fft_instance);

    if (status != ARM_MATH_SUCCESS) {
        // Initialization error handling (e.g., unsupported FFT size)
        printf("Error initializing FFT: %d\n    ", status);
    }
}

float32_t* __not_in_flash_func(fft_mic_input_buffer)(float32_t* input_buffer) {
    // 2. Execute the Forward FFT
    arm_rfft_fast_f32(&fft_instance, input_buffer, fft_output_buffer, 0);

    // 3. Handle DC and Nyquist separately due to CMSIS-DSP packed layout
    // Use absolute value since they have no imaginary parts
    fft_magnitude_buffer[0] = fabsf(fft_output_buffer[0]);   // DC Bin (0 Hz)
    fft_magnitude_buffer[512] = fabsf(fft_output_buffer[1]); // Nyquist Bin (Fs/2)

    // 4. Calculate magnitudes for bins 1 to 511
    // We pass 'fft_output_buffer + 2' to skip DC/Nyquist
    // We pass 'fft_magnitude_buffer + 1' to align the output
    // The number of complex pairs remaining is 511
    arm_cmplx_mag_f32(fft_output_buffer + 2, fft_magnitude_buffer + 1, 511);

    return &fft_magnitude_buffer[0];
}

void visualize_fft(float32_t *magnitude_buf) {
    // 1. Send ANSI escape codes: Clear screen and reset cursor to top-left
    // This stops the terminal from scrolling and keeps the graph stationary
    printf("\033[2J\033[H");
    
    printf("=== RP2350 FFT SPECTRUM ANALYZER (44.1 kHz / 1024-pt) ===\n\n");

    // We look at the first 128 bins (~0 Hz to 5.5 kHz) where audio is active.
    // We group them by 4 to print 32 neat lines on the screen.
    for (int i = 4; i < 128 + (4 * 8); i += 4) { // Start at bin 4 to ignore low-end DC noise
        
        // Average 4 adjacent bins together to make the display stable
        float32_t avg_mag = (magnitude_buf[i] + magnitude_buf[i+1] + 
                             magnitude_buf[i+2] + magnitude_buf[i+3]) / 4.0f;
        
        // Calculate the actual center frequency for this display line
        int center_freq = (int)((i + 1.5f) * FS / (float32_t)FFT_SIZE);

        // Convert the raw magnitude into a character width.
        // The INMP441 is sensitive; you may need to tweak this '150.0f' multiplier
        // up or down depending on how loud you are speaking!
        int bar_length = (int)(avg_mag * 150.0f); 
        
        // Cap the bar length to fit comfortably in a standard 80-character terminal
        if (bar_length > 80) bar_length = 80;
        if (bar_length < 0)  bar_length = 0;

        // Print the frequency label cleanly padded to 5 characters
        printf("%5d Hz | ", center_freq);
        
        // Draw the amplitude bar
        for (int b = 0; b < bar_length; b++) {
            printf("#");
        }
        
        printf("\n");
    }
}



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
void __not_in_flash_func(i2s_callback_rx_ready)(const float *buffer) {
    update_mic_level_stats(buffer);
    fft_mic_input_buffer((float32_t *)buffer);
}

// Interrupt Hook: Invoked automatically when the MAX98357A requests audio bytes
void __not_in_flash_func(i2s_callback_tx_demanded)(float *buffer) {
    const size_t size = I2S_BUFFER_SIZE;
    switch (current_mode) {

        case MODE_LOOPBACK:
            for (size_t i = 0; i < size; ++i) {
                buffer[i] = 0.0f;
            }
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
            generate_modulated_chirp(buffer, 0);
            break;
    }
}

void core1_main() {
    static float float_buf[I2S_BUFFER_SIZE];
    static int32_t *tx_buffer = NULL;
    static int32_t *rx_buffer = NULL;

    init_audio_system(); // Initialize the FFT instance once at startup

    while(1) {
        int32_t *t_tx_buffer = get_tx_buffer();

        if (tx_buffer != t_tx_buffer) {
            i2s_callback_tx_demanded(float_buf);
            // Blazing fast vector conversion using optimized CMSIS assembly loops
            arm_float_to_q31((float32_t *)float_buf, (q31_t *)t_tx_buffer, I2S_BUFFER_SIZE);
            tx_buffer = t_tx_buffer;
        }

        int32_t *t_rx_buffer = get_rx_buffer();

        if (rx_buffer != t_rx_buffer) {
            // Blazing fast vector conversion using optimized CMSIS assembly loops
            arm_q31_to_float((q31_t *)t_rx_buffer, (float32_t *)float_buf, I2S_BUFFER_SIZE);
            i2s_callback_rx_ready(float_buf);
            rx_buffer = t_rx_buffer;
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
            visualize_fft(fft_magnitude_buffer);
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
