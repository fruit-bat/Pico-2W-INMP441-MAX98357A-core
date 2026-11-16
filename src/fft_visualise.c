#include <stdio.h>
#include "fft_visualise.h"

void fft_visualise_init(
    FftVisualise_t *vis,
    uint32_t fft_size,
    float32_t sample_rate,
    uint32_t page_size,
    float32_t scale,
    uint32_t max_bar_width
) {
    vis->fft_size = fft_size;
    vis->sample_rate = sample_rate;
    vis->page_size = page_size;
    vis->page = 0;
    vis->scale = scale;
    vis->max_bar_width = max_bar_width;
}

uint32_t fft_visualise_page_count(const FftVisualise_t *vis) {
    return (vis->fft_size + vis->page_size - 1) / vis->page_size;
}

void fft_visualise_page_next(FftVisualise_t *vis) {
    if (vis->page + 1 < fft_visualise_page_count(vis)) {
        vis->page++;
    }
}

void fft_visualise_page_prev(FftVisualise_t *vis) {
    if (vis->page > 0) {
        vis->page--;
    }
}

void fft_visualise_draw(const FftVisualise_t *vis, const float32_t *magnitude_buffer) {
    // Clear screen and reset cursor to top-left.
    // This stops the terminal from scrolling and keeps the graph stationary.
    printf("\033[2J\033[H");

    printf("=== RP2350 FFT SPECTRUM ANALYZER (%2.1f kHz / %lu-pt) page %lu of %lu ===\n\n",
        vis->sample_rate / 1000.0f,
        (unsigned long)vis->fft_size,
        (unsigned long)(vis->page + 1),
        (unsigned long)fft_visualise_page_count(vis));

    const uint32_t first = vis->page * vis->page_size;
    uint32_t last = first + vis->page_size;
    if (last > vis->fft_size) last = vis->fft_size;

    for (uint32_t i = first; i < last; i++) {
        // Center frequency of this bin
        const int center_freq = (int)((float32_t)i * vis->sample_rate / (float32_t)vis->fft_size);

        // Convert the raw magnitude into a character width, clipped to fit the terminal.
        float32_t length = magnitude_buffer[i] * vis->scale;
        if (length > (float32_t)vis->max_bar_width) length = (float32_t)vis->max_bar_width;
        if (length < 0.0f) length = 0.0f;
        const uint32_t bar_length = (uint32_t)length;

        printf("%5d Hz | ", center_freq);
        for (uint32_t b = 0; b < bar_length; b++) {
            putchar('#');
        }
        putchar('\n');
    }

    // Pad a short final page so the display is always page_size rows tall
    // and any text printed after it stays in the same place.
    for (uint32_t i = last; i < first + vis->page_size; i++) {
        putchar('\n');
    }
}
