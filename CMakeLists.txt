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

pico_enable_stdio_usb(Pico-2W-INMP441-MAX98357A-core-test 1)
pico_enable_stdio_uart(Pico-2W-INMP441-MAX98357A-core-test 0)
pico_add_extra_outputs(Pico-2W-INMP441-MAX98357A-core-test)
