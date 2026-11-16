#ifndef FFT_VISUALISE_H
#define FFT_VISUALISE_H

/**
 * @file fft_visualise.h
 *
 * Paged text-mode bar graph of an FFT magnitude spectrum, drawn to stdout
 * using ANSI escape codes so the display stays stationary in the terminal.
 *
 * One line is drawn per FFT bin. Because a full spectrum does not fit on a
 * screen, the bins are split into pages of page_size lines.
 */

#include <stdint.h>
#include "arm_math.h"

/**
 * Configuration and paging state for the visualiser.
 */
typedef struct {
    uint32_t fft_size;       // Number of bins in the magnitude spectrum
    float32_t sample_rate;   // Hz, used to label each bin with its frequency
    uint32_t page_size;      // Number of bins drawn per page
    uint32_t page;           // Index of the page currently displayed
    float32_t scale;         // Multiplier converting magnitude to bar length in characters
    uint32_t max_bar_width;  // Bars are clipped to this many characters
} FftVisualise_t;

/**
 * Initializes the visualiser, starting on the first page.
 *
 * @param vis           Target visualiser state.
 * @param fft_size      Number of bins in the magnitude spectrum.
 * @param sample_rate   Sample rate in Hz.
 * @param page_size     Number of bins drawn per page (must be > 0).
 * @param scale         Magnitude to bar-length multiplier; tune for microphone gain.
 * @param max_bar_width Maximum bar length in characters.
 */
void fft_visualise_init(
    FftVisualise_t *vis,
    uint32_t fft_size,
    float32_t sample_rate,
    uint32_t page_size,
    float32_t scale,
    uint32_t max_bar_width
);

/**
 * Returns the number of pages needed to show every bin.
 */
uint32_t fft_visualise_page_count(const FftVisualise_t *vis);

/**
 * Moves to the next page, stopping at the last page.
 */
void fft_visualise_page_next(FftVisualise_t *vis);

/**
 * Moves to the previous page, stopping at the first page.
 */
void fft_visualise_page_prev(FftVisualise_t *vis);

/**
 * Clears the terminal and draws the current page of the spectrum.
 *
 * @param vis              Visualiser state.
 * @param magnitude_buffer Magnitude spectrum (length = fft_size).
 */
void fft_visualise_draw(const FftVisualise_t *vis, const float32_t *magnitude_buffer);

#endif // FFT_VISUALISE_H
