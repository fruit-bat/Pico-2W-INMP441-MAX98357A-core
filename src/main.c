#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "i2s_core.h"

#define MIC_LEVEL_PRINT_PERIOD_MS 200
#define BAR_WIDTH 40
#define TONE_AMPLITUDE 18000
#define TONE_STARTUP_STEPS 32

enum test_mode {
    MODE_LOOPBACK = 0,
    MODE_MIC_LEVEL = 1,
    MODE_TONE_440 = 2,
};

static volatile enum test_mode current_mode = MODE_LOOPBACK;
static volatile bool loopback_enabled = true;
static int32_t shared_dsp_buffer[I2S_BUFFER_SIZE];
static volatile uint32_t mic_peak_value = 0;
static volatile uint32_t mic_rms_value = 0;
static uint32_t tone_phase_accum = 0;
static uint32_t tone_step_index = 0;

static void print_bar(uint32_t percent) {
    int count = (percent * BAR_WIDTH) / 100;
    printf("[MIC] %3lu%% |", (unsigned long)percent);
    for (int i = 0; i < BAR_WIDTH; ++i) {
        putchar(i < count ? '#' : ' ');
    }
    printf("|\n");
}

static void update_mic_level_stats(const int32_t *buffer, size_t size) {
    uint64_t sum_sq = 0;
    uint32_t peak = 0;

    for (size_t i = 0; i < size; ++i) {
        int32_t sample = buffer[i];
        uint32_t mag = (sample < 0) ? (uint32_t)(-(sample + 1)) + 1U : (uint32_t)sample;
        if (mag > peak) {
            peak = mag;
        }
        sum_sq += (uint64_t)((uint32_t)sample * (uint32_t)sample);
    }

    mic_peak_value = peak;
    if (size > 0) {
        uint32_t rms = (uint32_t)sqrt((double)sum_sq / (double)size);
        mic_rms_value = rms;
    } else {
        mic_rms_value = 0;
    }
}

static void set_mode(enum test_mode mode) {
    current_mode = mode;
    loopback_enabled = (mode == MODE_LOOPBACK);
    printf("[MODE] Selected mode: %s\n",
           mode == MODE_LOOPBACK ? "loopback" :
           mode == MODE_MIC_LEVEL ? "mic-level" : "tone-440");
}

static void fill_tone_buffer(int32_t *buffer, size_t size) {
    const float phase_step = (2.0f * (float)M_PI * 440.0f) / (float)I2S_SAMPLE_RATE;
    float gain = 1.0f;

    if (tone_step_index < TONE_STARTUP_STEPS) {
        gain = (float)tone_step_index / (float)TONE_STARTUP_STEPS;
        tone_step_index++;
    }

    for (size_t i = 0; i < size; ++i) {
        float angle = (float)tone_phase_accum * phase_step;
        float sine = sinf(angle);
        int32_t sample = (int32_t)(sine * (float)TONE_AMPLITUDE * gain);
        buffer[i] = sample;
        tone_phase_accum = (tone_phase_accum + 1U) % 65536U;
    }
}

// Interrupt Hook: Invoked automatically when the INMP441 fills a memory chunk
void i2s_callback_rx_ready(const int32_t *buffer, size_t size) {
    if (current_mode == MODE_LOOPBACK) {
        for (size_t i = 0; i < size; ++i) {
            shared_dsp_buffer[i] = buffer[i];
        }
    } else if (current_mode == MODE_MIC_LEVEL) {
        update_mic_level_stats(buffer, size);
    }
}

// Interrupt Hook: Invoked automatically when the MAX98357A requests audio bytes
void i2s_callback_tx_demanded(int32_t *buffer, size_t size) {
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

int main() {
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

    set_mode(MODE_LOOPBACK);
    printf("[CORE] Waiting for input. Press 1, 2 or 3 to switch mode.\n");
    i2s_core_start();

    uint32_t last_print_ms = 0;
    uint32_t last_status_ms = 0;
    while (true) {
        int ch = getchar_timeout_us(1000);
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

        if (current_mode == MODE_MIC_LEVEL && now_ms - last_print_ms >= MIC_LEVEL_PRINT_PERIOD_MS) {
            uint32_t percent = (mic_rms_value * 100U) / 32768U;
            if (percent > 100U) {
                percent = 100U;
            }
            print_bar(percent);
            last_print_ms = now_ms;
        }

        if (now_ms - last_status_ms >= 1000U) {
            const char *mode_name = (current_mode == MODE_LOOPBACK) ? "loopback" :
                                    (current_mode == MODE_MIC_LEVEL) ? "mic-level" : "tone-440";
            printf("[STATUS] running=%s peak=%lu rms=%lu\n",
                   mode_name,
                   (unsigned long)mic_peak_value,
                   (unsigned long)mic_rms_value);
            last_status_ms = now_ms;
        }

        tight_loop_contents();
    }

    return 0;
}
