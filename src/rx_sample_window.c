#include <string.h>
#include "rx_sample_window.h"
#include "pico/stdlib.h"

void rx_sample_window_init(
    RxSampleWindow_t *w,
    float32_t *storage,
    uint32_t frame_size,
    uint32_t delay
) {
    w->storage = storage;
    w->frame_size = frame_size;
    w->newest = 0;
    w->delay = delay % frame_size;

    memset(storage, 0, RX_SAMPLE_WINDOW_STORAGE_LEN(frame_size) * sizeof(*storage));
}

float32_t* __not_in_flash_func(rx_sample_window_next_block)(RxSampleWindow_t *w) {
    w->newest = 1u - w->newest;
    return &w->storage[w->newest * w->frame_size];
}

void rx_sample_window_set_delay(RxSampleWindow_t *w, uint32_t delay) {
    w->delay = delay % w->frame_size;
}

uint32_t __not_in_flash_func(rx_sample_window_get_delay)(const RxSampleWindow_t *w) {
    return w->delay;
}

void __not_in_flash_func(rx_sample_window_read)(const RxSampleWindow_t *w, float32_t *frame_out) {
    const uint32_t n = w->frame_size;
    const uint32_t d = w->delay; // Read once; may be changed from another core
    const float32_t *newer = &w->storage[w->newest * n];
    const float32_t *older = &w->storage[(1u - w->newest) * n];

    // Tail of the older block, then the head of the newer block
    memcpy(frame_out, &older[n - d], d * sizeof(*frame_out));
    memcpy(&frame_out[d], newer, (n - d) * sizeof(*frame_out));
}
