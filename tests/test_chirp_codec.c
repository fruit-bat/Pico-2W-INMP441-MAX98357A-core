/**
 * @file test_chirp_codec.c
 *
 * Host tests for the chirp encoder and decoder, using the same parameters as
 * the firmware (see main.c). Received frames come from the analytic channel
 * model in chirp_channel.c.
 */

#include <stdio.h>
#include <math.h>
#include "chirp_test_rig.h"
#include "test_util.h"

int test_failures = 0;
int test_checks = 0;

#define N RIG_N
#define M RIG_M
#define VOLUME RIG_VOLUME

static float frame[N];

// ---------------------------------------------------------------------------

// The encoder's table-driven output matches the analytic chirp
static void test_encoder_matches_model(void) {
    const ChirpChannel_t ideal = { .offset = 0.0, .clock_scale = 1.0, .noise_rms = 0.0 };
    double worst = 0.0;
    for (uint32_t s = 0; s < M; s++) {
        static float enc_out[N];
        chirp_encoder_symbol_to_samples(&rig_encoder, s, enc_out);
        chirp_channel_rx_frame(&rig_tx_model, &ideal, &s, 1, 0, frame);
        for (uint32_t i = 0; i < N; i++) {
            const double d = fabs((double)enc_out[i] - (double)frame[i]);
            if (d > worst) worst = d;
        }
    }
    printf("  worst |encoder - model| = %.3g (%.3g of full scale)\n", worst, worst / VOLUME);
    CHECK(worst < 1e-3 * VOLUME);
}

// Every symbol decodes to itself when aligned
static void test_round_trip_all_symbols(void) {
    double worst_err = 0.0, min_mag = 1.0;
    for (uint32_t s = 0; s < M; s++) {
        static float enc_out[N];
        ChirpSymbolRx_t rx;
        chirp_encoder_symbol_to_samples(&rig_encoder, s, enc_out);
        rig_decode(enc_out, &rx);
        CHECK_EQ_U(rx.symbol_index, s);
        const double e = fabs(rig_timing_error_samples(&rx));
        if (e > worst_err) worst_err = e;
        if (rx.magnitude < min_mag) min_mag = rx.magnitude;
    }
    printf("  worst |timing error| = %.4f samples, min magnitude = %.4f\n", worst_err, min_mag);
    CHECK(worst_err < 0.2);
}

// fft_magnitude_buffer keeps the plain de-chirped power; folded_power_buffer
// holds the coherent fold. Symbol M/2 wraps half way through the frame, so its
// two pieces land in bins M/2 and N - M/2 with a quarter of the full tone's
// power each, and folding them coherently recovers the full tone: exactly the
// coherent sum (sqrt(a) + sqrt(b))^2 of the two raw bins, which is within a
// few percent of the full tone (the band-pass edges trim the chirp's spectral
// tails slightly, ~0.1 dB).
static void test_spectrum_buffers(void) {
    const uint32_t s = M / 2;
    const uint32_t stream[3] = { s, s, s };
    const ChirpChannel_t ch = { .offset = 0.0, .clock_scale = 1.0, .noise_rms = 0.0 };
    ChirpSymbolRx_t rx;
    chirp_channel_rx_frame(&rig_tx_model, &ch, stream, 3, 1, frame);
    rig_decode(frame, &rx);

    const double full = pow(N * VOLUME / 2.0, 2.0); // Band-pass keeps half the real amplitude
    const double raw_a = rig_decoder.fft_magnitude_buffer[s];
    const double raw_b = rig_decoder.fft_magnitude_buffer[N - M + s];
    const double folded = rig_decoder.folded_power_buffer[s];
    printf("  raw %.4f, %.4f of full tone; folded %.4f\n", raw_a / full, raw_b / full, folded / full);
    CHECK_NEAR(raw_a / full, 0.25, 0.02);
    CHECK_NEAR(raw_b / full, 0.25, 0.02);
    const double coherent_sum = pow(sqrt(raw_a) + sqrt(raw_b), 2.0);
    CHECK_NEAR(folded / coherent_sum, 1.0, 0.002);
    CHECK_NEAR(folded / full, 1.0, 0.05);
}

// A repeated symbol read with the window offset by d samples decodes as the
// same symbol, with a timing error of d samples (|d| < half a step). The
// decoder refines the peak by a mean shift on the folded spectrum sampled
// every half bin, which keeps this close to linear for every symbol (on whole
// bins alone small offsets were under-reported by up to 5x).
static void test_timing_offset_repeated_symbol(void) {
    printf("  offset  mean      ratio min  ratio max  worst |error|  (over all symbols)\n");
    double worst_all = 0.0;
    for (double d = -7.5; d <= 7.5; d += 0.5) {
        if (d == 0.0) continue;
        double mean = 0.0, ratio_min = 1e9, ratio_max = -1e9, worst = 0.0;
        for (uint32_t s = 0; s < M; s++) {
            const uint32_t stream[3] = { s, s, s };
            const ChirpChannel_t ch = { .offset = d, .clock_scale = 1.0, .noise_rms = 0.0 };
            ChirpSymbolRx_t rx;
            chirp_channel_rx_frame(&rig_tx_model, &ch, stream, 3, 1, frame);
            rig_decode(frame, &rx);
            CHECK_EQ_U(rx.symbol_index, s);
            const double measured = rig_timing_error_samples(&rx);
            const double ratio = measured / d;
            mean += measured / M;
            if (ratio < ratio_min) ratio_min = ratio;
            if (ratio > ratio_max) ratio_max = ratio;
            if (fabs(measured - d) > worst) worst = fabs(measured - d);
        }
        printf("  %+5.1f   %+7.3f   %6.3f     %6.3f     %6.3f\n", d, mean, ratio_min, ratio_max, worst);
        if (worst > worst_all) worst_all = worst;
    }
    printf("  worst |measured - offset| over all = %.3f samples\n", worst_all);
    CHECK(worst_all < 0.25);
}

// Multipath: echoes spread the de-chirped peak over several bins. The timing
// estimate stays monotonic with slope ~1 and consistent across symbols; it is
// offset by roughly the echoes' energy-weighted delay, which is where the
// tracking loop will settle.
static void test_timing_with_echoes(void) {
    static const ChirpEcho_t echoes[] = { { 3.0, 0.6 }, { 7.0, 0.4 }, { 15.0, 0.3 }, { 31.0, 0.2 } };
    printf("  offset  mean      sd over symbols\n");
    double prev = -1e9, first_mean = 0.0, last_mean = 0.0;
    const double d_lo = -4.0, d_hi = 4.0;
    for (double d = d_lo; d <= d_hi; d += 1.0) {
        double sum = 0.0, sum_sq = 0.0;
        for (uint32_t s = 0; s < M; s += 3) {
            const uint32_t stream[3] = { s, s, s };
            const ChirpChannel_t ch = { .offset = d, .clock_scale = 1.0, .echoes = echoes,
                                        .echo_count = sizeof echoes / sizeof echoes[0] };
            ChirpSymbolRx_t rx;
            chirp_channel_rx_frame(&rig_tx_model, &ch, stream, 3, 1, frame);
            rig_decode(frame, &rx);
            CHECK_EQ_U(rx.symbol_index, s);
            const double e = rig_timing_error_samples(&rx);
            sum += e;
            sum_sq += e * e;
        }
        const double n = (M + 2) / 3;
        const double mean = sum / n, sd = sqrt(fmax(0.0, sum_sq / n - mean * mean));
        printf("  %+5.1f   %+7.3f   %6.3f\n", d, mean, sd);
        CHECK(mean > prev);   // Monotonic
        CHECK(sd < 0.2);      // Consistent across symbols
        prev = mean;
        if (d == d_lo) first_mean = mean;
        last_mean = mean;
    }
    const double slope = (last_mean - first_mean) / (d_hi - d_lo);
    printf("  slope %.3f\n", slope);
    CHECK_NEAR(slope, 1.0, 0.15);
}

// Random symbols (so the window straddles two different symbols): decisions
// stay correct for offsets up to nearly half a step
static void test_timing_offset_random_symbols(void) {
    enum { COUNT = 200 };
    static uint32_t stream[COUNT];
    chirp_channel_seed(1);
    for (uint32_t i = 0; i < COUNT; i++) stream[i] = chirp_channel_rand_u32() % M;

    printf("  offset  errors  mean timing  rms timing\n");
    for (double d = -7.0; d <= 7.0; d += 1.0) {
        const ChirpChannel_t ch = { .offset = d, .clock_scale = 1.0, .noise_rms = 0.0 };
        uint32_t errors = 0;
        double sum = 0.0, sum_sq = 0.0;
        for (uint32_t k = 1; k < COUNT - 1; k++) {
            ChirpSymbolRx_t rx;
            chirp_channel_rx_frame(&rig_tx_model, &ch, stream, COUNT, k, frame);
            rig_decode(frame, &rx);
            if (rx.symbol_index != stream[k]) errors++;
            const double e = rig_timing_error_samples(&rx);
            sum += e;
            sum_sq += e * e;
        }
        const double n = COUNT - 2;
        printf("  %+5.1f   %4u    %+7.3f      %6.3f\n", d, errors, sum / n, sqrt(sum_sq / n - (sum / n) * (sum / n)));
        CHECK_EQ_U(errors, 0);
    }
}

// Symbol error rate against SNR (signal power / total white noise power over
// the full band). The chirp's processing gain lets it decode well below 0 dB.
//
// Baseline: error free down to -12 dB. History: -6 dB with the wrapped
// pieces folded by adding powers; -9 dB with the band-pass and coherent fold;
// -12 dB once the symbol was decided from the strongest bin rather than by
// rounding the refined angle.
static void test_noise(void) {
    enum { COUNT = 500 };
    static uint32_t stream[COUNT];
    chirp_channel_seed(2);
    for (uint32_t i = 0; i < COUNT; i++) stream[i] = chirp_channel_rand_u32() % M;

    const double signal_rms = VOLUME / sqrt(2.0);
    printf("  SNR dB  errors/%u  rms timing (correct symbols)\n", COUNT - 2);
    for (int snr_db = 0; snr_db >= -24; snr_db -= 3) {
        const ChirpChannel_t ch = {
            .offset = 0.0,
            .clock_scale = 1.0,
            .noise_rms = signal_rms / pow(10.0, snr_db / 20.0),
        };
        uint32_t errors = 0, good = 0;
        double sum_sq = 0.0;
        for (uint32_t k = 1; k < COUNT - 1; k++) {
            ChirpSymbolRx_t rx;
            chirp_channel_rx_frame(&rig_tx_model, &ch, stream, COUNT, k, frame);
            rig_decode(frame, &rx);
            if (rx.symbol_index != stream[k]) {
                errors++;
            } else {
                const double e = rig_timing_error_samples(&rx);
                sum_sq += e * e;
                good++;
            }
        }
        printf("  %+4d    %5u      %6.3f\n", snr_db, errors, good ? sqrt(sum_sq / good) : 0.0);
        if (snr_db >= -12) CHECK_EQ_U(errors, 0);
    }
}

int main(void) {
    rig_setup();
    RUN_TEST(test_encoder_matches_model);
    RUN_TEST(test_round_trip_all_symbols);
    RUN_TEST(test_spectrum_buffers);
    RUN_TEST(test_timing_offset_repeated_symbol);
    RUN_TEST(test_timing_with_echoes);
    RUN_TEST(test_timing_offset_random_symbols);
    RUN_TEST(test_noise);
    printf("\n%d checks, %d failures\n", test_checks, test_failures);
    return test_failures ? 1 : 0;
}
