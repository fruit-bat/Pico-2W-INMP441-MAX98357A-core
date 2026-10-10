#include "chirp_symbol_rx.h"

void chirp_symbol_rx_config_init(
    ChirpSymbolRxConfig_t *config,
    uint32_t number_of_symbols) 
{
    config->number_of_symbols = number_of_symbols;
    config->symbols_per_radian = config->number_of_symbols / (2.0f * PI);
    config->radians_per_symbol = (2.0f * PI) / config->number_of_symbols;
}

void chirp_symbol_rx_set(
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

