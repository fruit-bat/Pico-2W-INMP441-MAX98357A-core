cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(audio_core_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(Pico-2W-INMP441-MAX98357A-core-test
    src/main.c
    src/i2s_core.c
    src/audio_i2s_setup.c
)

target_include_directories(Pico-2W-INMP441-MAX98357A-core-test PRIVATE src)

# Add the PIO file generation step for the actual target
pico_generate_pio_header(Pico-2W-INMP441-MAX98357A-core-test ${CMAKE_CURRENT_LIST_DIR}/src/audio_i2s.pio)

# Link hardware libraries required for RP2350 DMA and low-level control
target_link_libraries(Pico-2W-INMP441-MAX98357A-core-test 
    pico_stdlib 
    hardware_pio 
    hardware_dma 
    hardware_irq
    hardware_clocks
)

pico_enable_stdio_usb(Pico-2W-INMP441-MAX98357A-core-test 1)
pico_enable_stdio_uart(Pico-2W-INMP441-MAX98357A-core-test 0)
pico_add_extra_outputs(Pico-2W-INMP441-MAX98357A-core-test)