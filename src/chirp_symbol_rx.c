#include "chirp_symbol_rx.h"
#include "pico/stdlib.h"

void chirp_symbol_rx_config_init(
    ChirpSymbolRxConfig_t *config,
    uint32_t number_of_symbols) 
{
    config->number_of_symbols = number_of_symbols;
    config->symbols_per_radian = config->number_of_symbols / (2.0f * PI);
    config->radians_per_symbol = (2.0f * PI) / config->number_of_symbols;
}

void __not_in_flash_func(chirp_symbol_rx_set)(
    const ChirpSymbolRxConfig_t *config,
    ChirpSymbolRx_t *symbol_rx,
    uint32_t symbol_index,
    float32_t angle,
    float32_t magnitude,
    float32_t strength) 
{
    symbol_rx->angle = angle;
    symbol_rx->magnitude = magnitude;
    symbol_rx->strength = strength;
    symbol_rx->symbol_index = symbol_index % config->number_of_symbols;

    float32_t error = angle - (float32_t)symbol_rx->symbol_index * config->radians_per_symbol;
    while (error >= PI) error -= 2.0f * PI;
    while (error < -PI) error += 2.0f * PI;
    symbol_rx->error_angle = error;
}

