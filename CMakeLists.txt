cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(audio_core_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# https://github.com/jptrainor/cmsis-sandbox/blob/main/src/CMakeLists.txt
# Pull in CMSIS-DSP
set(CMSISCORE "$ENV{PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core")
set(DISABLEFLOAT16 ON)
include(FetchContent)
FetchContent_Declare(cmsisdsp
   GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP.git
   GIT_TAG "v1.15.0"
)
FetchContent_MakeAvailable(cmsisdsp)

add_executable(Pico-2W-INMP441-MAX98357A-core-test
    src/main.c
    src/i2s_core.c
    src/audio_i2s_setup.c
)

add_compile_options(
    -Ofast                     # Maximize math execution speed
    -mfpu=fpv5-sp-d16          # Force single-precision hardware FPU
    -mfloat-abi=hard           # Enable hardware floating-point ABI
    -ffunction-sections -fdata-sections # Tell the compiler to split functions and data into separate sections
)

add_link_options(
    -mfloat-abi=hard 
    -mfpu=fpv5-sp-d16
    "-Wl,--no-warn-mismatch,--gc-sections"
)

target_include_directories(Pico-2W-INMP441-MAX98357A-core-test PRIVATE 
    src
    ${cmsisdsp_SOURCE_DIR}/Include
    ${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core/Include
)

# Add the PIO file generation step for the actual target
pico_generate_pio_header(Pico-2W-INMP441-MAX98357A-core-test ${CMAKE_CURRENT_LIST_DIR}/src/audio_i2s.pio)

# Link hardware libraries required for RP2350 DMA and low-level control
target_link_libraries(Pico-2W-INMP441-MAX98357A-core-test 
    pico_stdlib
    pico_multicore
    hardware_pio 
    hardware_dma 
    hardware_irq
    hardware_clocks
    cmsis_core
    CMSISDSP
)

pico_enable_stdio_usb(Pico-2W-INMP441-MAX98357A-core-test 1)
pico_enable_stdio_uart(Pico-2W-INMP441-MAX98357A-core-test 0)
pico_add_extra_outputs(Pico-2W-INMP441-MAX98357A-core-test)

# Dependency to ensure that libCMSISDSP.a is built
add_dependencies(Pico-2W-INMP441-MAX98357A-core-test CMSISDSP)
