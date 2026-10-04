#ifndef CHIRP_SYMBOL_RX_H
#define CHIRP_SYMBOL_RX_H

#include <stddef.h>
#include "arm_math.h"

// TODO I need to rethink my receiver a little.
// TODO I think I have linked the FFT bin and symbol concept a little to much.
// TODO I will try to be a bit more 'general' in this file... but need to revisit the code in main.c


struct {
    uint32_t number_of_symbols;
    float32_t symbols_per_radian;
    float32_t radians_per_symbol;
    float32_t radians_per_hz; // Placeholder for now, but will be used to convert from FFT bin index to radians
} ChirpSymbolRxConfig_t;

inline void chirp_symbol_rx_config_init(
    ChirpSymbolRxConfig_t *config,
    uint32_t number_of_symbols,
    float32_t radians_per_hz) 
{
    config->number_of_symbols = number_of_symbols;
    config->symbols_per_radian = config->number_of_symbols / (2.0f * PI);
    config->radians_per_symbol = (2.0f * PI) / config->number_of_symbols;
    config->radians_per_hz = radians_per_hz;
}


/**
 * Used to track a single received chirp symbol and its associated properties. 
 */
struct {
    float32_t angle;      // Centroid angle of the received symbol
    float32_t magnitude;  // Centroid magnitude of the received symbol
    float32_t strength;   // Centroid strength of the received symbol
    uint32_t symbol_index; // Index of the received symbol in the chirp modulation scheme
    float32_t error_angle; // Error angle of the received symbol
} ChirpSymbolRx_t;

/**
 * Used to track a window of received chirp symbols and their associated properties. 
 */
struct {
    ChirpSymbolRx_t *symbols; // Pointer to an array of received symbols
    size_t size;              // Number of symbols in the window
    size_t head;              // Index of the most recent symbol
} ChirpSymbolRxWindow_t;

// TODO Just an idea for now, but I think I will need to track the error slope and mean error over time to help with symbol tracking and prediction.
typedef struct {
    float32_t mean_error;
    float32_t error_slope;
    float32_t total_weight;
} ChirpSymbolRxTrack_t;

inline void chirp_symbol_rx_set(
    ChirpSymbolRxConfig_t *config,
    ChirpSymbolRx_t *symbol_rx,
    float32_t angle,
    float32_t magnitude,
    float32_t strength) 
{
    symbol_rx->angle = angle;
    symbol_rx->magnitude = magnitude;
    symbol_rx->strength = strength;

    const float32_t fsym = angle * config->symbols_per_radian;
    const int32_t n = (int32_t)config->number_of_symbols;
    const float32_t m = lroundf(fsym);
    const int32_t usym = (int32_t)m % n;
    symbol_rx->symbol_index = (uint32_t)(usym < 0 ? usym + n : usym);
    symbol_rx->error_angle = angle - (m * config->radians_per_symbol);
}

#endif // CHIRP_SYMBOL_RX_H 
