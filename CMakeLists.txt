cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(audio_core_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Pull down CMSIS-DSP to get access to source directories
set(CMSISCORE "$ENV{PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core")
set(DISABLEFLOAT16 ON)
include(FetchContent)

FetchContent_Declare(cmsisdsp
   GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP.git
   GIT_TAG "v1.15.0"
   EXCLUDE_FROM_ALL  # <--- Stops CMake from auto-building targets we don't explicitly ask for
)

# This is the modern standard; because EXCLUDE_FROM_ALL is set above, 
# it downloads the package and exposes paths safely without running heavy build chains.
FetchContent_MakeAvailable(cmsisdsp)

# Global build options applied to your sources
add_compile_options(
    -Ofast                            
    -ffunction-sections 
    -fdata-sections 
)

add_link_options(
    "-Wl,--no-warn-mismatch,--gc-sections"
)

# Define executable and ONLY compile the essential FFT source blocks 
# This avoids compiling thousands of unwanted matrix, filtering, and fixed-point functions
add_executable(Pico-2W-INMP441-MAX98357A-core-test
    src/main.c
    src/i2s_core.c
    src/audio_i2s_setup.c
    src/cyclic_weighted_centroid.c
    src/chirp_symbol_rx.c
    src/chirp_decoder.c
    src/fft_visualise.c
    src/rx_sample_window.c
    src/chirp_encoder.c
    src/spsc_ring.c
    src/frame_snapshot.c
    src/symbol_timing_tracker.c
    
    # Target precisely the Float32 Transform source files
    ${cmsisdsp_SOURCE_DIR}/Source/TransformFunctions/TransformFunctions.c
    ${cmsisdsp_SOURCE_DIR}/Source/CommonTables/CommonTables.c
    ${cmsisdsp_SOURCE_DIR}/Source/ComplexMathFunctions/ComplexMathFunctions.c
    ${cmsisdsp_SOURCE_DIR}/Source/SupportFunctions/SupportFunctions.c 
    # Target precisely the essential CMSIS-DSP source files
)

# Enforce compiler flags onto the CMSIS source files we just added
set_source_files_properties(
    ${cmsisdsp_SOURCE_DIR}/Source/TransformFunctions/TransformFunctions.c
    ${cmsisdsp_SOURCE_DIR}/Source/CommonTables/CommonTables.c
    ${cmsisdsp_SOURCE_DIR}/Source/ComplexMathFunctions/ComplexMathFunctions.c
    ${cmsisdsp_SOURCE_DIR}/Source/SupportFunctions/SupportFunctions.c 
    PROPERTIES COMPILE_FLAGS "-Ofast -ffunction-sections -fdata-sections"
)

target_include_directories(Pico-2W-INMP441-MAX98357A-core-test PRIVATE 
    src
    ${cmsisdsp_SOURCE_DIR}/Include
    ${cmsisdsp_SOURCE_DIR}/PrivateInclude
    ${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core/Include
)

# Add the PIO file generation step for the actual target
pico_generate_pio_header(Pico-2W-INMP441-MAX98357A-core-test ${CMAKE_CURRENT_LIST_DIR}/src/audio_i2s.pio)

# Standard hardware links (Notice CMSISDSP library target is completely removed here)
target_link_libraries(Pico-2W-INMP441-MAX98357A-core-test 
    pico_stdlib
    pico_multicore
    hardware_pio 
    hardware_dma 
    hardware_irq
    hardware_clocks
    cmsis_core
)

# Run the whole program from RAM.
#
# The boot loader copies all code and read-only data from flash to RAM at
# startup, so nothing on the real-time path (including CMSIS-DSP and the SDK
# float functions, which __not_in_flash_func cannot reach) waits on flash.
# The cost is RAM: roughly the size of the 'text' column reported by
# arm-none-eabi-size (about 62 KB at the time of writing).
#
# The __not_in_flash_func markers in the sources are kept on purpose: they
# have no effect in this mode, but take over again if this line is removed.
pico_set_binary_type(Pico-2W-INMP441-MAX98357A-core-test copy_to_ram)

# ---------------------------------------------------------------------------
# Alternative if RAM runs short: run from flash, but move just the hot
# library code into RAM.
#
# 1. Delete the pico_set_binary_type(... copy_to_ram) line above.
#
# 2. Create a directory (e.g. linker/) holding project copies of these two
#    SDK files from
#      ${PICO_SDK_PATH}/src/rp2_common/pico_standard_link/script_include/
#    The SDK's default linker script places everything listed in
#    EXCLUDE_FILE(...) in RAM instead of flash; we just extend the lists.
#
#    linker/default_text_excludes.incl (code):
#      *(EXCLUDE_FILE(*libgcc.a: *libc.a: *lib*_a-mem*.o *libm.a: *TransformFunctions.c.o *ComplexMathFunctions.c.o *SupportFunctions.c.o *float_math.c.o *float_sci_m33_vfp.S.o) .text*)
#
#    linker/default_rodata_excludes.incl (constant tables, e.g. FFT twiddles):
#      *(EXCLUDE_FILE(*libgcc.a: *libc.a: *lib*_a-mem*.o *libm.a: *CommonTables.c.o) .rodata*)
#
#    The first four entries of each list are the SDK defaults; keep them.
#    The extra entries are:
#      TransformFunctions.c.o    CMSIS-DSP FFT (arm_cfft_f32, butterflies, bit reversal)
#      ComplexMathFunctions.c.o  CMSIS-DSP complex multiply / magnitude
#      SupportFunctions.c.o      CMSIS-DSP q31 <-> float conversion
#      float_math.c.o            SDK powf etc.
#      float_sci_m33_vfp.S.o     SDK sinf, cosf, atan2f etc.
#      CommonTables.c.o          CMSIS-DSP FFT twiddle and bit-reversal tables
#    Because of --gc-sections only functions/tables actually used are kept.
#    To find the object file for any symbol, search the .elf.map file in the
#    build directory for ".text.<symbol>" or ".rodata.<symbol>".
#
# 3. Tell the SDK to use them in preference to its defaults:
#
#    pico_add_linker_script_override_path(Pico-2W-INMP441-MAX98357A-core-test
#        ${CMAKE_CURRENT_LIST_DIR}/linker
#        FILES default_text_excludes.incl default_rodata_excludes.incl)
#
# 4. Check placement with:
#      arm-none-eabi-nm build/Pico-2W-INMP441-MAX98357A-core-test.elf | grep arm_cfft_f32
#    Addresses starting 0x2... are RAM, 0x1... are flash.
#
# (Requires Pico SDK >= 2.2 for the modular linker scripts; tested with 2.3.0.)
# ---------------------------------------------------------------------------

pico_enable_stdio_usb(Pico-2W-INMP441-MAX98357A-core-test 1)
pico_enable_stdio_uart(Pico-2W-INMP441-MAX98357A-core-test 0)
pico_add_extra_outputs(Pico-2W-INMP441-MAX98357A-core-test)
