#ifndef CHIRP_SYMBOL_RX_H
#define CHIRP_SYMBOL_RX_H

#include <stddef.h>
#include "arm_math.h"

// TODO I need to rethink my receiver a little.
// TODO I think I have linked the FFT bin and symbol concept a little to much.
// TODO I will try to be a bit more 'general' in this file... but need to revisit the code in main.c


//
// Configuration structure for the chirp symbol receiver
//
typedef struct {
    uint32_t number_of_symbols;
    float32_t symbols_per_radian;
    float32_t radians_per_symbol;
} ChirpSymbolRxConfig_t;

//
// Initializes the chirp symbol receiver configuration with the specified number of symbols and radians per Hz.
// This function calculates the symbols per radian and radians per symbol based on the provided parameters.
//
void chirp_symbol_rx_config_init(
    ChirpSymbolRxConfig_t *config,
    uint32_t number_of_symbols);

//
// Used to track a single received chirp symbol and its associated properties. 
//
typedef struct {
    float32_t angle;      // Centroid angle of the received symbol
    float32_t magnitude;  // Centroid magnitude of the received symbol
    float32_t strength;   // Centroid strength of the received symbol
    uint32_t symbol_index; // Index of the received symbol in the chirp modulation scheme
    float32_t error_angle; // Error angle of the received symbol
} ChirpSymbolRx_t;

//
// Sets the properties of a received chirp symbol based on the provided angle, magnitude, and strength.
// This function calculates the symbol index and error angle based on the configuration and the provided angle.
//
void chirp_symbol_rx_set(
    ChirpSymbolRxConfig_t *config,
    ChirpSymbolRx_t *symbol_rx,
    float32_t angle,
    float32_t magnitude,
    float32_t strength);

//
// Used to track a window of received chirp symbols and their associated properties. 
//
typedef struct {
    ChirpSymbolRx_t *symbols; // Pointer to an array of received symbols
    size_t size;              // Number of symbols in the window
    size_t head;              // Index of the most recent symbol
} ChirpSymbolRxWindow_t;

//
// TODO Just an idea for now, but I think I will need to track the error slope and mean error over time to help with symbol tracking and prediction.
//
typedef struct {
    float32_t mean_error;
    float32_t error_slope;
    float32_t total_weight;
} ChirpSymbolRxTrack_t;


#endif // CHIRP_SYMBOL_RX_H 
