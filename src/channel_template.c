#include <string.h>
#include "channel_template.h"
#include "pico/stdlib.h"

void channel_template_init(ChannelTemplate_t *t, uint32_t bins, float32_t *storage) {
    t->bins = bins;
    t->shape = storage;
    t->work = storage + bins;
    channel_template_reset(t);
}

void channel_template_reset(ChannelTemplate_t *t) {
    memset(t->shape, 0, t->bins * sizeof(*t->shape));
    t->frames = 0;
}

void __not_in_flash_func(channel_template_learn)(
    ChannelTemplate_t *t,
    const float32_t *folded_power,
    uint32_t known_bin
) {
    const uint32_t n = t->bins;
    float32_t total = 0.0f;
    for (uint32_t k = 0; k < n; k++) total += folded_power[k];
    if (total <= 0.0f) return;

    // Normalised, so loud and quiet frames count equally
    const float32_t scale = 1.0f / total;
    for (uint32_t o = 0; o < n; o++) {
        t->shape[o] += scale * folded_power[(known_bin + o) % n];
    }
    t->frames++;
}

uint32_t __not_in_flash_func(channel_template_decide)(
    const ChannelTemplate_t *t,
    const float32_t *folded_power,
    uint32_t first_bin,
    uint32_t step,
    uint32_t count,
    float32_t *margin
) {
    const uint32_t n = t->bins;
    float32_t *amp = t->work;
    for (uint32_t k = 0; k < n; k++) {
        float32_t a;
        arm_sqrt_f32(folded_power[k] > 0.0f ? folded_power[k] : 0.0f, &a);
        amp[k] = a;
    }

    float32_t best = -1.0f, second = -1.0f;
    uint32_t best_i = 0;
    for (uint32_t i = 0; i < count; i++) {
        const uint32_t bin = (first_bin + i * step) % n;
        float32_t score = 0.0f;
        if (t->frames == 0) {
            score = amp[bin]; // Nothing learned: the strongest bin
        } else {
            // Correlate with the shape: sum over offsets o of amp[bin + o] * shape[o]
            const uint32_t head = n - bin; // offsets before amp wraps round
            for (uint32_t o = 0; o < head; o++) score += amp[bin + o] * t->shape[o];
            for (uint32_t o = head; o < n; o++) score += amp[bin + o - n] * t->shape[o];
        }
        if (score > best) {
            second = best;
            best = score;
            best_i = i;
        } else if (score > second) {
            second = score;
        }
    }

    if (margin != NULL) {
        *margin = second > 0.0f ? best / second : (best > 0.0f ? 1e9f : 1.0f);
    }
    return best_i;
}
