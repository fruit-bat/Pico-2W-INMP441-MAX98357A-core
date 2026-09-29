#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "i2s_core.h"

#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, etc.)
#include "arm_const_structs.h"

#include "cyclic_weighted_centroid.h"

#define MAX_SYMBOLS 64u

const float32_t FS = I2S_SAMPLE_RATE;
const float32_t BPB = FS / (float32_t)I2S_BUFFER_SIZE; // Bandwidth per FFT bin
const float32_t BW = BPB * MAX_SYMBOLS; // Total bandwidth for the chirp signal
const float32_t F0 = 15000.0f;
const float32_t F1 = F0 + BW;
const float32_t T  = (float32_t)I2S_BUFFER_SIZE / FS;
const float32_t chirp_rate = BW / T;
const float32_t chirp_vol = 0.005f;


inline float32_t chirp_phase(float32_t t) {
    return 2.0f * PI * (F0 * t + 0.5f * chirp_rate * t * t);
}

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

// So we can phase align with the input signal
static volatile uint32_t rx_sample_delay = 967;
// A couple of mic buffers so we can phase shift
static float rx_float_buf[2][I2S_BUFFER_SIZE];
static uint32_t rx_float_buf_idx = 0;

// Currently must be 1024
#define FFT_SIZE I2S_BUFFER_SIZE

float32_t fft_output_buffer[FFT_SIZE * 2]; // Complex output: real + imag interleaved
float32_t fft_magnitude_buffer[FFT_SIZE]; 

static float32_t complex_dechirp_vector[FFT_SIZE * 2];

// Space for the centroid calculation tables
static float32_t cos_table[MAX_SYMBOLS];
static float32_t sin_table[MAX_SYMBOLS];
// Centroid configuration
static CyclicWeightedCentroid_t cwc;
// Centroid result vector
static CyclicWeightedCentroidVector_t cwc_vector;
static CyclicWeightedCentroidResult_t cwc_result;





void generate_complex_dechirp_vector() {

    const float32_t sample_rate = FS;
    const float32_t f_min = F0;
    const float32_t T = (float32_t)FFT_SIZE / sample_rate;
    
    for (uint32_t n = 0; n < FFT_SIZE; n++) {
        float32_t t = (float32_t)n / sample_rate;
        // The standard Up-Chirp phase equation
        float32_t phase = chirp_phase(t);
        
        // Complex Conjugate: [Cos(phase), -Sin(phase)]
        complex_dechirp_vector[2 * n]     = cosf(phase);  
        complex_dechirp_vector[2 * n + 1] = -sinf(phase); 
    }
}

void generate_modulated_chirp(float32_t *tx_audio_buffer, uint32_t symbol_val) {
    uint32_t shift = (float32_t)symbol_val * FS / BW; // Shift in samples for the symbol value
    for (int n = 0; n < I2S_BUFFER_SIZE; n++) {
        tx_audio_buffer[n] = complex_dechirp_vector[((n + shift) % I2S_BUFFER_SIZE) * 2] * chirp_vol; // Apply the complex de-chirp and scale by volume
    }
}

// The 1024-point complex instance is globally defined by CMSIS-DSP
const arm_cfft_instance_f32 *cfft_instance = &arm_cfft_sR_f32_len1024;

void init_audio_system(void) {
    generate_complex_dechirp_vector();

    // Initialize the centroid configuration
    cyclic_weighted_centroid_init(
        &cwc,
        FFT_SIZE,
        4.0f, // Power exponent for weighting
        MAX_SYMBOLS,
        cos_table,
        sin_table
    );
}

float32_t* __not_in_flash_func(fft_mic_input_buffer)() {

    static float32_t complex_input_buffer[FFT_SIZE * 2]; // Interleaved complex input: [Real, Imag, Real, Imag...]    

    // 1. Stage real mic data into the complex array layout
    for (uint32_t i = 0; i < FFT_SIZE; i++) {

        uint32_t rxb_idx = (rx_float_buf_idx + (rx_sample_delay > i ? 1 : 0)) % 2;
        uint32_t rxs_idx = (FFT_SIZE - rx_sample_delay + i) % FFT_SIZE;

        float32_t rx_sample = rx_float_buf[rxb_idx][rxs_idx];

        complex_input_buffer[2 * i]     = rx_sample; // Real part
        complex_input_buffer[2 * i + 1] = 0.0f;      // Imaginary part
    }

    // 2. Complex Element-wise Multiplication (De-chirp)
    // Multiplies complex_input by complex_dechirp and saves to fft_output_buffer
    arm_cmplx_mult_cmplx_f32(complex_input_buffer, complex_dechirp_vector, fft_output_buffer, FFT_SIZE);

    // 3. Execute the Complex FFT
    // Note: arm_cfft_f32 processes data IN-PLACE. 
    // The last parameter '0' means Forward FFT (1 would mean Inverse)
    // The bit reversal flag is hardcoded to 1 in modern CMSIS-DSP
    arm_cfft_f32(cfft_instance, fft_output_buffer, 0, 1);

    // 4. Calculate magnitudes for all 1024 bins
    // No more complex packing layouts or splitting DC/Nyquist!
    arm_cmplx_mag_squared_f32(fft_output_buffer, fft_magnitude_buffer, FFT_SIZE);

    return &fft_magnitude_buffer[0];
}

static int visualize_fft_start = 0;
const static int visualize_fft_page_size = 50;

void visualize_fft(float32_t *magnitude_buf) {
    // 1. Send ANSI escape codes: Clear screen and reset cursor to top-left
    // This stops the terminal from scrolling and keeps the graph stationary
    printf("\033[2J\033[H");
    
    printf("=== RP2350 FFT SPECTRUM ANALYZER (%2.1f kHz / 1024-pt) ===\n\n", FS/1000.0f);

    for (int i = visualize_fft_start * visualize_fft_page_size;
         i < (visualize_fft_page_size * (1 + visualize_fft_start)); 
         i += 1) { 
        
        // Average 4 adjacent bins together to make the display stable
        float32_t avg_mag = magnitude_buf[i];
        
        // Calculate the actual center frequency for this display line
        int center_freq = (int)((i) * FS / (float32_t)FFT_SIZE);

        // Convert the raw magnitude into a character width.
        // The INMP441 is sensitive; you may need to tweak this '150.0f' multiplier
        // up or down depending on how loud you are speaking!
        int bar_length = (int)(avg_mag * 1500.0f); 
        
        // Cap the bar length to fit comfortably in a standard 80-character terminal
        if (bar_length > 120) bar_length = 120;
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
void __not_in_flash_func(i2s_callback_rx_ready)() {
    //update_mic_level_stats(buffer);
    float32_t* fft_result = fft_mic_input_buffer();
    
    // Calculate the centroid of the FFT magnitude spectrum
    cyclic_weighted_centroid_init_vector(&cwc_vector);
    cyclic_weighted_centroid_accumulate_forwards(
        &cwc, 
        fft_result, 
        cwc.power_exponent, 
        &cwc_vector
    );
    cyclic_weighted_centroid_accumulate_backwards(
        &cwc, 
        fft_result, 
        cwc.power_exponent, 
        &cwc_vector
    );
    cyclic_weighted_centroid_finalize(&cwc, &cwc_vector, &cwc_result);
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
                    rx_float_buf_idx = 1 - rx_float_buf_idx; // Toggle between 0 and 1
                    memcpy(
                        (float32_t *)&rx_float_buf[rx_float_buf_idx][0], 
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
                    rx_float_buf_idx = 1 - rx_float_buf_idx; // Toggle between 0 and 1
                    // fast vector conversion using optimized CMSIS assembly loops
                    arm_q31_to_float(
                        (q31_t *)t_rx_buffer, 
                        (float32_t *)&rx_float_buf[rx_float_buf_idx][0], 
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
                case 'z':
                    rx_sample_delay = rx_sample_delay > 0 ? rx_sample_delay - 1 : FFT_SIZE;
                    break;
                case 'x':
                    rx_sample_delay = rx_sample_delay < (FFT_SIZE - 1) ? rx_sample_delay + 1 : 0;
                    break;
                case 'c':
                    visualize_fft_start = visualize_fft_start > 0 ? visualize_fft_start - 1 : 0;
                    break;
                case 'v':
                    visualize_fft_start = visualize_fft_start  + 1;
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
            //print_bar(percent);
            printf("Symbol %3lu of %lu RX delay %4ld RX symbol centroid: %3.4f Rx symbol strength: %ef     \n", 
                symbol_index, 
                MAX_SYMBOLS,
                rx_sample_delay,
                cwc_result.bin,
                cwc_result.strength
            );
            uint32_t rsym = cwc_result.ubin;
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
