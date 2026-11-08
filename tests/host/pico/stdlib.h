#ifndef HOST_PICO_STDLIB_H
#define HOST_PICO_STDLIB_H

/**
 * @file stdlib.h
 *
 * Host build stand-in for the Pico SDK's pico/stdlib.h, providing just enough
 * for the firmware's DSP sources to compile and run on a PC.
 */

#include <stdint.h>
#include <stdbool.h>

// On the Pico this places a function in RAM; on the host it does nothing
#define __not_in_flash_func(func_name) func_name

#endif // HOST_PICO_STDLIB_H
