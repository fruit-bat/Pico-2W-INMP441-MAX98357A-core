#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "hardware/clocks.h"
#include "tusb.h"
#include "i2s_core.h"

#include "arm_math.h" // For CMSIS DSP functions (e.g., sinf, cosf, etc.)
#include "arm_const_structs.h"

#include "biquad_filter.h"
#include "chirp_decoder.h"
#include "chirp_encoder.h"
#include "chirp_symbol_rx.h"
#include "frame_snapshot.h"
#include "rx_sample_window.h"
#include "spsc_ring.h"
#include "channel_template.h"
#include "packet_rx.h"
#include "packet_tx.h"
#include "protocol.h"
#include "symbol_sequence.h"
#include "symbol_timing_tracker.h"

// The host talks to the board over USB serial with the protocol in
// protocol.md (src/protocol.c); tools/chirp_host.py is a client.
#define FIRMWARE_VERSION "0.2.0"

// The chirp sweeps CHIRP_BINS FFT bins (64 bins = 2.76 kHz from F0). Symbols
// are spaced bins_per_symbol bins apart (setting link.spacing: 1, 2 or 4 while
// running; both boards must match): 1 gives 64 symbols (6 bits each),
// 2 gives 32 (5 bits), 4 gives 16 (4 bits). Wider spacing costs rate but
// tolerates a channel that smears each symbol over neighbouring bins (the
// desk channel spreads about +/-1 bin), and widens the tracker's pull-in to
// +/- half a symbol step. Between the two boards 1 bin per symbol lost long
// packets to neighbouring-symbol slips (echoes from the previous symbol); 2
// did not; 4 has the most margin (board 2's speaker is ~10 dB weaker in band).
#define CHIRP_BINS 64u
#define DEFAULT_BINS_PER_SYMBOL 4u // 1, 2 or 4
static volatile uint32_t bins_per_symbol_request = DEFAULT_BINS_PER_SYMBOL; // Set by core 0 (link.spacing)
static volatile uint32_t bins_per_symbol = DEFAULT_BINS_PER_SYMBOL;         // In use; core 1 applies requests
static inline uint32_t num_symbols(void) { return CHIRP_BINS / bins_per_symbol; }
static inline uint32_t bits_per_symbol(void) { return (uint32_t)__builtin_ctz(num_symbols()); }

// Currently must be 1024
#define FFT_SIZE I2S_BUFFER_SIZE

static const float32_t FS = I2S_SAMPLE_RATE;
static const float32_t BPB = FS / (float32_t)FFT_SIZE; // Bandwidth per FFT bin
static const float32_t BW = BPB * CHIRP_BINS; // Total bandwidth for the chirp signal
static const float32_t F0 = 15000.0f;
static const float32_t T  = (float32_t)FFT_SIZE / FS;
static const float32_t chirp_rate = BW / T;
// Chirp transmit level, dB relative to full scale (setting tx.volume, -55..-1;
// core 1 applies changes between blocks). -28 dB is the x8 level used so far.
#define TX_VOLUME_DEFAULT_DB (-28)
static volatile int32_t tx_volume_db = TX_VOLUME_DEFAULT_DB;

static float32_t volume_from_db(int32_t db) {
    return powf(10.0f, (float32_t)db / 20.0f);
}

#define LOG_BAND_BINS (2u * CHIRP_BINS) // FFT bins logged per frame: -CHIRP_BINS .. CHIRP_BINS-1
#define TONE_STARTUP_STEPS 32
#define RX_RECORD_QUEUE_CAPACITY 64u // Must be a power of two
#define TX_PACKET_QUEUE_CAPACITY 2u  // Packets waiting to be sent; must be a power of two
#define RX_PACKET_QUEUE_CAPACITY 4u  // Received packets waiting for core 0; must be a power of two
#define TX_EVENT_QUEUE_CAPACITY 8u   // Packet starts/ends waiting for core 0; must be a power of two

// Symbol timing tracking (see symbol_timing_tracker.h)
#define TRACK_ALPHA          0.25f // Proportional gain
#define TRACK_BETA           0.0f  // Drift (rate) gain; 0 while TX and RX share a clock (nothing to learn)
#define TRACK_MAX_RATE       0.5f  // Max drift, samples per frame
#define TRACK_RATE_WINDOW    4.0f  // Only learn drift when |error| <= this many samples
#define TRACK_MIN_MAGNITUDE  0.8f  // Ignore frames whose centroid magnitude is below this (no clear tone)

// Microphone high-pass (see biquad_filter.h): removes the mic's DC offset and
// low-frequency noise, which would otherwise leak into the chirp band at the
// frame edges. Second-order Butterworth, well below the 15 kHz band edge.
#define RX_HIGHPASS_HZ       10000.0f
#define RX_HIGHPASS_Q        0.70710678f

enum test_mode {
    MODE_LOOPBACK = 0,
    MODE_MIC_LEVEL = 1,
    MODE_TONE_440 = 2,
    MODE_TONE_CHIRP = 3,
    MODE_SEQUENCE = 4,  // A known pseudo-random symbol per frame (see symbol_sequence.h), for link tests
    MODE_PACKET = 5,    // Silent until a packet is queued ('s' key), then sends it (see packet_format.h)
};

enum tracking_mode {
    TRACK_OFF = 0,      // Delay only changes with the z/x keys
    TRACK_DECISION = 1, // Track using each frame's error from the nearest symbol (no knowledge of TX);
                        // aligns to the symbol grid, but symbol labels may be offset by a constant
    TRACK_PILOT = 2,    // Track using the error from the known transmitted symbol (testing only);
                        // locks from any delay with correct symbol labels
};

// ---------------------------------------------------------------------------
// Shared between cores.
// Core 0 (UI) writes these single-word controls; core 1 (DSP) reads them.
// ---------------------------------------------------------------------------
static volatile enum test_mode current_mode = MODE_PACKET; // Silent until a packet is queued
static volatile uint32_t tx_symbol = 0;        // Symbol transmitted in chirp modes
static volatile uint32_t tone_step_index = 0;  // Reset by core 0 to restart the tone ramp
static volatile enum tracking_mode tracking_mode = TRACK_OFF;
static volatile bool packet_align = true;      // Move the window when the packet receiver locks (rx.align)
// The packet receiver also runs while this board is transmitting, so a single
// board hears its own packets (useful for testing); rx.during_tx=off for half duplex
static volatile bool packet_rx_during_tx = true;
static volatile bool packet_rx_reset_request = false; // Core 0 asks core 1 to abandon reception ("reset rx")

// Core 1 writes these counters; core 0 reads them.
static volatile uint32_t rx_blocks_missed = 0;  // RX blocks not processed in time
static volatile uint32_t tx_blocks_missed = 0;  // TX blocks not refilled in time
static volatile uint32_t rx_records_dropped = 0; // Records lost because the queue was full

//
// One record per received frame, passed from core 1 to core 0.
// This is the receiver's output: core 0 turns it into protocol events and
// stream lines for the host.
//
typedef struct {
    uint32_t frame;          // RX block count when this frame was decoded
    uint32_t mode;           // enum test_mode in effect for this frame
    uint32_t bins_per_symbol; // Symbol spacing in effect for this frame
    int32_t tx_volume_db;     // Transmit level in effect for this frame, dBFS
    uint32_t delay;          // Alignment delay in samples used for this frame
    uint32_t tx_symbol;      // Symbol this board was transmitting at the time (in sequence mode, the latest sent)
    float32_t peak;          // Peak absolute sample value in the frame after the RX high-pass, [0, 1]
    ChirpSymbolRx_t symbol;  // Decoded symbol
    uint32_t tracking;       // enum tracking_mode in effect for this frame
    float32_t timing_error;  // Timing error in samples (positive: delay should increase)
    float32_t timing_rate;   // Tracker's drift estimate, samples per frame
    float32_t band_power[LOG_BAND_BINS]; // De-chirped FFT power |X|^2 of bins -CHIRP_BINS .. CHIRP_BINS-1
                                         // (bin = index mod FFT_SIZE)
    float32_t folded_power[CHIRP_BINS]; // Coherently folded power the decoder decides from, bins 0 .. CHIRP_BINS-1
    PacketRxStatus_t packet;             // Packet receiver after this frame (see packet_rx.h)
    int32_t align_moved;                 // Samples the window was moved on this frame (when it locked)
    float32_t packet_shape[CHIRP_BINS];  // Channel shape it last learned (from a packet's B)
} RxFrameRecord_t;

// A packet queued by core 0 for core 1 to send
typedef struct {
    uint32_t id;     // The protocol's id for it (tx.start / tx.done)
    uint32_t length;
    uint8_t payload[PACKET_MAX_PAYLOAD];
} TxPacketRequest_t;

// A packet started or ended (core 1 to core 0)
typedef struct {
    uint32_t id;
    uint32_t frame;  // RX block count at the time
    bool start;      // Else done
    bool abandoned;  // Done, but cut short (spacing change)
} TxEvent_t;

static uint8_t rx_record_storage[SPSC_RING_STORAGE_LEN(sizeof(RxFrameRecord_t), RX_RECORD_QUEUE_CAPACITY)];
static SpscRing_t rx_record_queue;

// A packet received by core 1 (good or bad), for core 0. Pushed just before
// the record whose packet.event is PACKET_RX_EVENT_PACKET; that record's
// packet fields hold the details (CRC, margins).
typedef struct {
    uint32_t length;
    uint8_t payload[PACKET_MAX_PAYLOAD];
} RxPacket_t;

static uint8_t rx_packet_storage[SPSC_RING_STORAGE_LEN(sizeof(RxPacket_t), RX_PACKET_QUEUE_CAPACITY)];
static SpscRing_t rx_packet_queue;
static volatile uint32_t rx_packets_dropped = 0; // Received packets lost because the queue was full

static uint8_t tx_packet_storage[SPSC_RING_STORAGE_LEN(sizeof(TxPacketRequest_t), TX_PACKET_QUEUE_CAPACITY)];
static SpscRing_t tx_packet_queue;

static uint8_t tx_event_storage[SPSC_RING_STORAGE_LEN(sizeof(TxEvent_t), TX_EVENT_QUEUE_CAPACITY)];
static SpscRing_t tx_event_queue;

// Consistent copy of a frame's FFT power spectrum (the spectrum stream), tagged with its frame
static float32_t spectrum_snapshot_buffer[FFT_SIZE];
static FrameSnapshot_t spectrum_snapshot;

// Double-buffered mic input so we can phase align with the input signal.
// Core 1 writes samples; core 0 may adjust the delay.
static float32_t rx_window_storage[RX_SAMPLE_WINDOW_STORAGE_LEN(FFT_SIZE)];
static RxSampleWindow_t rx_window;

// ---------------------------------------------------------------------------
// Core 1 only.
// ---------------------------------------------------------------------------

// The aligned frame passed to the decoder
static float32_t rx_frame[FFT_SIZE];

// Statically allocated storage for the chirp decoder (no malloc)
static float32_t complex_dechirp_vector[CHIRP_DECODER_COMPLEX_LEN(FFT_SIZE)];
static float32_t complex_dechirp_half_vector[CHIRP_DECODER_COMPLEX_LEN(FFT_SIZE)];
static float32_t fft_half_buffer[CHIRP_DECODER_COMPLEX_LEN(FFT_SIZE)];
static float32_t fft_output_buffer[CHIRP_DECODER_COMPLEX_LEN(FFT_SIZE)];
static float32_t fft_magnitude_buffer[FFT_SIZE];
static float32_t folded_power_buffer[CHIRP_BINS];
static float32_t fine_power_buffer[CHIRP_DECODER_FINE_LEN(CHIRP_BINS)];
static float32_t cos_table[CHIRP_BINS];
static float32_t sin_table[CHIRP_BINS];

static ChirpDecoderState_t chirp_decoder;
static ChirpEncoder_t chirp_encoder;
static ChirpSymbolRxConfig_t symbol_rx_config;
static SymbolTimingTracker_t timing_tracker;
static SymbolSequence_t tx_sequence;   // MODE_SEQUENCE: the symbols being sent
static uint32_t tx_sequence_symbol;    // MODE_SEQUENCE: the symbol most recently sent
static PacketTx_t packet_tx;           // MODE_PACKET: the packet being sent
static uint32_t packet_tx_symbol;      // MODE_PACKET: the symbol most recently sent
static uint32_t packet_tx_id;          // MODE_PACKET: the protocol's id for it
static float32_t packet_rx_storage[PACKET_RX_STORAGE_LEN(CHIRP_BINS)];
static float32_t channel_template_storage[CHANNEL_TEMPLATE_STORAGE_LEN(CHIRP_BINS)];
static ChannelTemplate_t channel_template;
static PacketRx_t packet_rx;
static BiquadFilter_t rx_highpass; // Applied to the microphone stream, block by block

static float tone_phase = 0.0f;

static void init_audio_system(void) {
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
        4, // Power exponent for weighting
        CHIRP_BINS,
        3, // Bins either side of the peak used to refine its position; the symbol is
           // decided from the refined centre, so this sets how much of a multipath-spread
           // symbol is gathered into the decision and the timing
        complex_dechirp_vector,
        complex_dechirp_half_vector,
        fft_half_buffer,
        fft_output_buffer,
        fft_magnitude_buffer,
        folded_power_buffer,
        fine_power_buffer,
        cos_table,
        sin_table
    );

    // The transmitter shares the decoder's chirp table
    chirp_encoder_init(
        &chirp_encoder,
        chirp_decoder.complex_dechirp_vector,
        FFT_SIZE,
        num_symbols(),
        volume_from_db(tx_volume_db)
    );

    chirp_symbol_rx_config_init(&symbol_rx_config, num_symbols());

    symbol_timing_tracker_init(
        &timing_tracker,
        FFT_SIZE,
        TRACK_ALPHA,
        TRACK_BETA,
        TRACK_MAX_RATE,
        TRACK_RATE_WINDOW,
        rx_sample_window_get_delay(&rx_window)
    );

    biquad_filter_init_highpass(&rx_highpass, FS, RX_HIGHPASS_HZ, RX_HIGHPASS_Q);

    packet_tx_init(&packet_tx, bits_per_symbol());
    channel_template_init(&channel_template, CHIRP_BINS, channel_template_storage);
    packet_rx_init(&packet_rx, CHIRP_BINS, bits_per_symbol(), packet_rx_storage, &channel_template);
}

static float32_t __not_in_flash_func(peak_abs)(const float32_t *buffer, uint32_t size) {
    float32_t peak = 0.0f;
    for (uint32_t i = 0; i < size; ++i) {
        const float32_t mag = fabsf(buffer[i]);
        if (mag > peak) {
            peak = mag;
        }
    }
    return peak;
}

static void __not_in_flash_func(fill_tone_buffer)(float *buffer) {
    const float phase_step = (2.0f * (float)M_PI * 440.0f) / (float)I2S_SAMPLE_RATE;
    const float two_pi = 2.0f * (float)M_PI;
    float gain = 0.1f;

    const uint32_t step = tone_step_index;
    if (step < TONE_STARTUP_STEPS) {
        gain *= (float)step / (float)TONE_STARTUP_STEPS;
        tone_step_index = step + 1;
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

// Wraps an angle into [-pi, pi)
static float32_t __not_in_flash_func(wrap_pi)(float32_t a) {
    while (a >= PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

//
// Core 1: timing error in samples for a decoded frame (positive: increase the delay).
//
// A symbol is the base chirp cyclically shifted by FFT_SIZE / num_symbols()
// samples per symbol step, so an angle error converts directly into samples.
// A late window (delay too large) reduces the effective shift and so the
// decoded angle, giving a negative error.
//
static float32_t __not_in_flash_func(timing_error_samples)(const RxFrameRecord_t *record, enum tracking_mode mode) {
    const float32_t samples_per_radian =
        (float32_t)(FFT_SIZE / num_symbols()) * symbol_rx_config.symbols_per_radian;

    float32_t error_angle = record->symbol.error_angle; // Relative to the decided symbol (the strongest bin)
    if (mode == TRACK_PILOT) {
        // Relative to the symbol actually sent: +/- half a frame
        error_angle = wrap_pi(record->symbol.angle - (float32_t)record->tx_symbol * symbol_rx_config.radians_per_symbol);
    }
    return error_angle * samples_per_radian;
}

// Core 1: decode the newest block in the RX window and publish the result.
static void __not_in_flash_func(process_rx_block)(uint32_t frame) {
    static enum tracking_mode last_tracking = TRACK_OFF;

    rx_sample_window_read(&rx_window, rx_frame);

    // Static: records are large (band_power) and core 1's stack is small
    static RxFrameRecord_t record;
    record = (RxFrameRecord_t){
        .frame = frame,
        .mode = current_mode,
        .bins_per_symbol = bins_per_symbol,
        .tx_volume_db = tx_volume_db,
        .delay = rx_sample_window_get_delay(&rx_window),
        .tx_symbol = current_mode == MODE_SEQUENCE ? tx_sequence_symbol
                   : current_mode == MODE_PACKET ? packet_tx_symbol : tx_symbol,
        .peak = peak_abs(rx_frame, FFT_SIZE),
    };

    chirp_decoder_real_chirp_to_symbol(
        &chirp_decoder,
        rx_frame,
        &symbol_rx_config,
        &record.symbol
    );

    // Symbol timing tracking; the new delay applies from the next frame
    const enum tracking_mode tracking = tracking_mode;
    if (tracking != last_tracking) {
        symbol_timing_tracker_reset(&timing_tracker, record.delay);
        last_tracking = tracking;
    }
    record.tracking = tracking;
    record.timing_error = timing_error_samples(&record, tracking);
    if (tracking != TRACK_OFF) {
        const bool valid = record.symbol.magnitude >= TRACK_MIN_MAGNITUDE;
        rx_sample_window_set_delay(
            &rx_window,
            symbol_timing_tracker_update(&timing_tracker, record.delay, record.timing_error, valid));
    }
    record.timing_rate = timing_tracker.rate;

    // Keep the band around DC (where de-chirped symbols land) with the record
    for (uint32_t i = 0; i < LOG_BAND_BINS; i++) {
        record.band_power[i] = chirp_decoder.fft_magnitude_buffer[(FFT_SIZE + i - CHIRP_BINS) % FFT_SIZE];
    }
    memcpy(record.folded_power, chirp_decoder.folded_power_buffer, sizeof(record.folded_power));

    // Packet receiver (optionally not while this board is sending a packet)
    if (packet_rx_reset_request) {
        packet_rx_reset(&packet_rx);
        packet_rx_reset_request = false;
    }
    if (packet_rx_during_tx || !packet_tx_busy(&packet_tx)) {
        const float32_t centre_bins = record.symbol.angle * (float32_t)CHIRP_BINS / (2.0f * (float32_t)M_PI);
        record.packet = *packet_rx_frame(&packet_rx, chirp_decoder.folded_power_buffer, centre_bins);
        // Preamble A found: move the window onto the earliest path, with its
        // energy on a bin centre (from the next frame)
        const int32_t move = (int32_t)lroundf(
            ((float32_t)record.packet.align_bins + record.packet.align_fraction) * (float32_t)(FFT_SIZE / CHIRP_BINS)); // Samples earlier
        if (packet_align && record.packet.event == PACKET_RX_EVENT_LOCKED && move != 0) {
            record.align_moved = move;
            const uint32_t delay = (uint32_t)(((int32_t)record.delay + move) % (int32_t)FFT_SIZE + (int32_t)FFT_SIZE) % FFT_SIZE;
            rx_sample_window_set_delay(&rx_window, delay);
            packet_rx_window_moved(&packet_rx, record.packet.align_bins);
            symbol_timing_tracker_reset(&timing_tracker, delay);
        }
        // A complete packet: to core 0 ahead of this frame's record
        if (record.packet.event == PACKET_RX_EVENT_PACKET) {
            static RxPacket_t rx_packet; // Static: core 1's stack is small
            rx_packet.length = record.packet.length;
            memcpy(rx_packet.payload, packet_rx_payload(&packet_rx), rx_packet.length);
            if (!spsc_ring_push(&rx_packet_queue, &rx_packet)) rx_packets_dropped++;
        }
    } else {
        packet_rx_reset(&packet_rx);
        record.packet = packet_rx.status;
    }
    memcpy(record.packet_shape, channel_template.shape, sizeof(record.packet_shape));

    // The spectrum first, so it is ready when core 0 sees this frame's record
    frame_snapshot_offer(&spectrum_snapshot, chirp_decoder.fft_magnitude_buffer, frame);

    if (!spsc_ring_push(&rx_record_queue, &record)) {
        rx_records_dropped++;
    }
}

// Core 1: produce the next block of audio to transmit.
static void __not_in_flash_func(fill_tx_block)(float32_t *buffer) {
    static enum test_mode last_mode = MODE_PACKET;
    const enum test_mode mode = current_mode;
    if (mode == MODE_SEQUENCE && last_mode != MODE_SEQUENCE) {
        symbol_sequence_init(&tx_sequence, bits_per_symbol()); // Restart from the beginning
    }
    last_mode = mode;

    switch (mode) {
        case MODE_PACKET: {
            // Start the next queued packet when idle; silence between packets
            static TxPacketRequest_t request;
            if (!packet_tx_busy(&packet_tx) && spsc_ring_pop(&tx_packet_queue, &request)) {
                packet_tx_start(&packet_tx, request.payload, request.length);
                packet_tx_id = request.id;
                const TxEvent_t e = { .id = request.id, .frame = i2s_core_get_rx_block_count(), .start = true };
                spsc_ring_push(&tx_event_queue, &e); // 8 deep and 2 a packet: can't fill
            }
            if (packet_tx_next_symbol(&packet_tx, &packet_tx_symbol)) {
                chirp_encoder_symbol_to_samples(&chirp_encoder, packet_tx_symbol, buffer);
                if (!packet_tx_busy(&packet_tx)) { // That was the last symbol
                    const TxEvent_t e = { .id = packet_tx_id, .frame = i2s_core_get_rx_block_count() };
                    spsc_ring_push(&tx_event_queue, &e);
                }
            } else {
                memset(buffer, 0, I2S_BUFFER_SIZE * sizeof(*buffer));
            }
            break;
        }

        case MODE_SEQUENCE:
            tx_sequence_symbol = symbol_sequence_next(&tx_sequence);
            chirp_encoder_symbol_to_samples(&chirp_encoder, tx_sequence_symbol, buffer);
            break;

        case MODE_LOOPBACK:
        case MODE_TONE_CHIRP:
            chirp_encoder_symbol_to_samples(&chirp_encoder, tx_symbol, buffer);
            break;

        case MODE_MIC_LEVEL:
            memset(buffer, 0, I2S_BUFFER_SIZE * sizeof(*buffer));
            break;

        case MODE_TONE_440:
            fill_tone_buffer(buffer);
            break;
    }
}

// Core 1: changes the symbol spacing (between blocks, so the transmitter and
// receiver change together). A packet being sent or received is abandoned.
static void apply_spacing(uint32_t bps) {
    if (packet_tx_busy(&packet_tx)) {
        const TxEvent_t e = { .id = packet_tx_id, .frame = i2s_core_get_rx_block_count(), .abandoned = true };
        spsc_ring_push(&tx_event_queue, &e);
    }
    bins_per_symbol = bps;
    const uint32_t n = num_symbols();
    chirp_encoder.number_of_symbols = n;
    chirp_symbol_rx_config_init(&symbol_rx_config, n);
    packet_tx_init(&packet_tx, bits_per_symbol());
    packet_rx_init(&packet_rx, CHIRP_BINS, bits_per_symbol(), packet_rx_storage, &channel_template);
    symbol_sequence_init(&tx_sequence, bits_per_symbol());
}

// Core 1: real-time audio loop. Services each DMA block as it completes.
static void __not_in_flash_func(core1_main)(void) {
    static float32_t tx_block[I2S_BUFFER_SIZE];

    init_audio_system();

    uint32_t last_tx_count = i2s_core_get_tx_block_count();
    uint32_t last_rx_count = i2s_core_get_rx_block_count();

    while (1) {
        if (bins_per_symbol_request != bins_per_symbol) apply_spacing(bins_per_symbol_request);
        {
            static int32_t applied_volume_db = TX_VOLUME_DEFAULT_DB;
            const int32_t db = tx_volume_db;
            if (db != applied_volume_db) {
                applied_volume_db = db;
                chirp_encoder.volume = volume_from_db(db);
            }
        }

        const uint32_t tx_count = i2s_core_get_tx_block_count();
        if (tx_count != last_tx_count) {
            tx_blocks_missed += tx_count - last_tx_count - 1;
            last_tx_count = tx_count;

            int32_t *tx_buffer = get_tx_buffer();
            fill_tx_block(tx_block);

            if (current_mode == MODE_LOOPBACK) {
                // Send silence and feed the generated audio straight to the receiver
                memset(tx_buffer, 0, I2S_BUFFER_SIZE * sizeof(*tx_buffer));
                memcpy(rx_sample_window_next_block(&rx_window), tx_block, I2S_BUFFER_SIZE * sizeof(*tx_block));
                process_rx_block(tx_count);
            } else {
                // fast vector conversion using optimized CMSIS assembly loops
                arm_float_to_q31(tx_block, (q31_t *)tx_buffer, I2S_BUFFER_SIZE);
            }
        }

        const uint32_t rx_count = i2s_core_get_rx_block_count();
        if (rx_count != last_rx_count) {
            rx_blocks_missed += rx_count - last_rx_count - 1;
            last_rx_count = rx_count;

            if (current_mode != MODE_LOOPBACK) {
                // fast vector conversion using optimized CMSIS assembly loops
                float32_t *block = rx_sample_window_next_block(&rx_window);
                arm_q31_to_float((q31_t *)get_rx_buffer(), block, I2S_BUFFER_SIZE);

                // High-pass the continuous stream, before it is cut into frames
                biquad_filter_process(&rx_highpass, block, I2S_BUFFER_SIZE);
                process_rx_block(rx_count);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Core 0: the host protocol (protocol.md). Core 0 owns the USB serial port:
// it turns host lines into settings and packets for core 1, and core 1's
// records, received packets and transmissions into events and streams.
// Nothing else may print: any other output would break the protocol's lines.
// ---------------------------------------------------------------------------
static Protocol_t protocol;

// Core 0's picture of the receiver, for status and the last lock/training
static struct {
    uint32_t frame;                 // Latest record's frame
    uint32_t good, bad, header_failed, lost;
    PacketRxState_t state;
    PacketRxStatus_t last_lock, last_train;
    int32_t last_moved;
} rx_view;

static char board_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

static void board_apply(void *ctx, const ProtocolSettings_t *s, uint32_t changed) {
    (void)ctx;
    const int32_t *v = s->value;
    if (changed & (1u << SETTING_LINK_SPACING)) bins_per_symbol_request = (uint32_t)v[SETTING_LINK_SPACING];
    if (changed & (1u << SETTING_TX_VOLUME)) tx_volume_db = v[SETTING_TX_VOLUME];
    if (changed & (1u << SETTING_RX_ALIGN)) packet_align = v[SETTING_RX_ALIGN] != 0;
    if (changed & (1u << SETTING_RX_TRACKING)) tracking_mode = (enum tracking_mode)v[SETTING_RX_TRACKING];
    if (changed & (1u << SETTING_RX_DURING_TX)) packet_rx_during_tx = v[SETTING_RX_DURING_TX] != 0;
    if (changed & (1u << SETTING_TEST_SYMBOL)) tx_symbol = (uint32_t)v[SETTING_TEST_SYMBOL];
    if (changed & (1u << SETTING_TEST_MODE)) {
        current_mode = (enum test_mode)v[SETTING_TEST_MODE]; // Same order as the protocol's choices
        tone_step_index = 0;
    }
}

static void board_get_status(void *ctx, ProtocolStatus_t *st) {
    (void)ctx;
    st->sample_rate = FS;
    st->fft_size = FFT_SIZE;
    st->bins = CHIRP_BINS;
    st->f0 = F0;
    st->bandwidth = BW;
    st->highpass_hz = RX_HIGHPASS_HZ;
    st->track_alpha = TRACK_ALPHA;
    st->track_beta = TRACK_BETA;
    st->track_max_rate = TRACK_MAX_RATE;
    st->track_rate_window = TRACK_RATE_WINDOW;
    st->track_min_magnitude = TRACK_MIN_MAGNITUDE;
    st->frames = rx_view.frame;
    st->rx_missed = rx_blocks_missed;
    st->tx_missed = tx_blocks_missed;
    st->records_dropped = rx_records_dropped;
    st->good = rx_view.good;
    st->bad = rx_view.bad;
    st->header_failed = rx_view.header_failed;
    st->lost = rx_view.lost;
    st->rx_dropped = rx_packets_dropped;
    st->tx_queued = atomic_load(&tx_packet_queue.head) - atomic_load(&tx_packet_queue.tail);
    st->rx_state = (uint32_t)rx_view.state;
    st->rx_delay = rx_sample_window_get_delay(&rx_window);
    st->lock_align = rx_view.last_lock.align_bins;
    st->lock_fraction = rx_view.last_lock.align_fraction;
    st->lock_moved = rx_view.last_moved;
    st->train_frame = rx_view.last_train.b_start_frame;
    st->train_coherence = rx_view.last_train.coherence;
    st->train_strongest = rx_view.last_train.strongest_offset;
    st->train_earliest = rx_view.last_train.earliest_offset;
}

static int board_send(void *ctx, const uint8_t *payload, uint32_t length, uint32_t id) {
    (void)ctx;
    static TxPacketRequest_t request;
    request.id = id;
    request.length = length;
    memcpy(request.payload, payload, length);
    return spsc_ring_push(&tx_packet_queue, &request) ? 0 : PROTO_ERR_QUEUE_FULL;
}

static uint32_t board_nudge(void *ctx, int32_t samples) {
    (void)ctx;
    const int32_t n = (int32_t)FFT_SIZE;
    const uint32_t delay = (uint32_t)((((int32_t)rx_sample_window_get_delay(&rx_window) + samples) % n + n) % n);
    rx_sample_window_set_delay(&rx_window, delay);
    return delay;
}

static void board_reset_counters(void *ctx) {
    (void)ctx;
    rx_blocks_missed = tx_blocks_missed = rx_records_dropped = rx_packets_dropped = 0;
    rx_view.good = rx_view.bad = rx_view.header_failed = rx_view.lost = 0;
}

static void board_reset_rx(void *ctx) {
    (void)ctx;
    packet_rx_reset_request = true;
}

static const ProtocolBoard_t board = {
    .version = FIRMWARE_VERSION,
    .board_id = board_id,
    .apply = board_apply,
    .get_status = board_get_status,
    .send = board_send,
    .nudge = board_nudge,
    .reset_counters = board_reset_counters,
    .reset_rx = board_reset_rx,
    // save/load: no flash storage yet ("save" answers err 452)
};

// Core 0: one record from core 1 -> events and stream lines
static void handle_record(const RxFrameRecord_t *r) {
    const PacketRxStatus_t *st = &r->packet;
    rx_view.frame = r->frame;
    rx_view.state = st->state;

    ProtocolEvent_t e = { .frame = r->frame };
    bool have_event = true;
    switch (st->event) {
        case PACKET_RX_EVENT_LOCKED:
            rx_view.last_lock = *st;
            rx_view.last_moved = r->align_moved;
            e.type = PROTO_EV_RX_LOCKED;
            e.u.locked.align = st->align_bins;
            e.u.locked.fraction = st->align_fraction;
            e.u.locked.moved = r->align_moved;
            break;
        case PACKET_RX_EVENT_TRAINED:
            rx_view.last_train = *st;
            e.type = PROTO_EV_RX_TRAINED;
            e.u.trained.b_start = st->b_start_frame;
            e.u.trained.coherence = st->coherence;
            e.u.trained.strongest = st->strongest_offset;
            e.u.trained.earliest = st->earliest_offset;
            break;
        case PACKET_RX_EVENT_TRAIN_FAILED:
            e.type = PROTO_EV_RX_TRAIN_FAILED;
            break;
        case PACKET_RX_EVENT_TONE:
            e.type = PROTO_EV_RX_TONE;
            break;
        case PACKET_RX_EVENT_HEADER_FAILED:
            rx_view.header_failed++;
            e.type = PROTO_EV_RX_HEADER_FAILED;
            e.u.header_failed.length = st->length;
            break;
        case PACKET_RX_EVENT_LOST:
            rx_view.lost++;
            e.type = PROTO_EV_RX_LOST;
            e.u.lost.symbols = st->data_count;
            break;
        case PACKET_RX_EVENT_PACKET: {
            // Its payload was queued just before this record
            static RxPacket_t rx_packet;
            if (!spsc_ring_pop(&rx_packet_queue, &rx_packet)) rx_packet.length = 0;
            if (st->crc_ok) rx_view.good++; else rx_view.bad++;
            e.type = PROTO_EV_RX_PACKET;
            e.u.packet.crc_ok = st->crc_ok;
            e.u.packet.length = rx_packet.length;
            e.u.packet.min_margin = st->min_margin;
            e.u.packet.weakest = st->weakest_symbol;
            e.u.packet.weak = st->weak_symbols;
            e.u.packet.payload = rx_packet.payload;
            break;
        }
        default:
            have_event = false;
            break;
    }
    if (have_event) protocol_event(&protocol, &e);

    // The spectrum, if core 1 copied this frame's (it copies one frame
    // after each release, so the stream gets as many as it can)
    const float32_t *spectrum = frame_snapshot_acquire(&spectrum_snapshot);
    if (spectrum && frame_snapshot_tag(&spectrum_snapshot) != r->frame) {
        if ((int32_t)(frame_snapshot_tag(&spectrum_snapshot) - r->frame) < 0) frame_snapshot_release(&spectrum_snapshot); // Stale
        spectrum = NULL; // (or a later frame's: keep it for that record)
    }
    const ProtocolFrame_t f = {
        .frame = r->frame,
        .tx_symbol = r->tx_symbol,
        .delay = r->delay,
        .rx_symbol = r->symbol.symbol_index,
        .angle = r->symbol.angle,
        .error_angle = r->symbol.error_angle,
        .magnitude = r->symbol.magnitude,
        .strength = r->symbol.strength,
        .peak = r->peak,
        .timing_error = r->timing_error,
        .timing_rate = r->timing_rate,
        .state = (uint32_t)st->state,
        .corr = st->correlation,
        .data_symbol = st->data_symbol,
        .data_margin = st->data_margin,
        .data_quality = st->data_quality,
        .folded = r->folded_power,
        .band = r->band_power,
        .shape = r->packet_shape,
        .shape_changed = st->event == PACKET_RX_EVENT_TRAINED,
        .spectrum = spectrum,
        .spectrum_len = spectrum ? FFT_SIZE : 0,
    };
    protocol_frame(&protocol, &f);
    if (spectrum) frame_snapshot_release(&spectrum_snapshot);
}

// Core 0: bytes to the host, only as many as USB will take now (never blocks)
static void write_output(void) {
    static char buf[CFG_TUD_CDC_TX_BUFSIZE];
    if (!stdio_usb_connected()) return; // Nobody listening: output waits (and events are dropped when full)
    const uint32_t room = tud_cdc_write_available();
    if (room == 0) return;
    const size_t n = protocol_output(&protocol, buf, room < sizeof buf ? room : sizeof buf);
    if (n > 0) stdio_put_string(buf, (int)n, false, false);
}

// Core 0: bytes from the host. A command waiting for room in the output
// keeps its remaining bytes here until the engine takes them
static void read_input(void) {
    static char buf[64];
    static size_t len = 0, pos = 0;
    if (pos == len) {
        pos = len = 0;
        int c;
        while (len < sizeof buf && (c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) buf[len++] = (char)c;
    }
    if (pos < len) pos += protocol_input(&protocol, buf + pos, len - pos);
}

int main() {

    // Set clock to 200 MHz (200,000 kHz)
    set_sys_clock_khz(200000, true);

    stdio_init_all();
    pico_get_unique_board_id_string(board_id, sizeof board_id);

    i2s_core_init();

    // Everything shared between the cores is initialized before core 1 starts
    spsc_ring_init(&rx_record_queue, rx_record_storage, sizeof(RxFrameRecord_t), RX_RECORD_QUEUE_CAPACITY);
    spsc_ring_init(&rx_packet_queue, rx_packet_storage, sizeof(RxPacket_t), RX_PACKET_QUEUE_CAPACITY);
    spsc_ring_init(&tx_packet_queue, tx_packet_storage, sizeof(TxPacketRequest_t), TX_PACKET_QUEUE_CAPACITY);
    spsc_ring_init(&tx_event_queue, tx_event_storage, sizeof(TxEvent_t), TX_EVENT_QUEUE_CAPACITY);
    frame_snapshot_init(&spectrum_snapshot, spectrum_snapshot_buffer, FFT_SIZE);
    rx_sample_window_init(
        &rx_window,
        rx_window_storage,
        FFT_SIZE,
        // 961 is about best. but using 960 for now to see some errors.
        960 // Initial delay in samples to phase align with the input signal
    );

    // Settings go to the shared controls before core 1 reads them; "* hello"
    // waits in the output queue until the host connects
    protocol_init(&protocol, &board);
    protocol_start(&protocol);

    i2s_core_start();

    // Start core 1
    multicore_launch_core1(core1_main);

    static RxFrameRecord_t record; // Static: records are large (band_power) and the stack is small
    static TxEvent_t tx_event;

    while (true) {
        read_input();

        // Packet starts/ends, then the receiver's frames, in order
        while (spsc_ring_pop(&tx_event_queue, &tx_event)) {
            ProtocolEvent_t e = { .type = tx_event.start ? PROTO_EV_TX_START : PROTO_EV_TX_DONE, .frame = tx_event.frame };
            e.u.tx.id = tx_event.id;
            e.u.tx.abandoned = tx_event.abandoned;
            protocol_event(&protocol, &e);
        }
        while (spsc_ring_pop(&rx_record_queue, &record)) {
            handle_record(&record);
        }

        protocol_poll(&protocol);
        write_output();
        tight_loop_contents();
    }

    return 0;
}
