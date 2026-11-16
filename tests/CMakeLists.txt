# Host (PC) tests for the DSP code in ../src. Built separately from the
# firmware, with the native compiler:
#
#   cmake -S tests -B tests/build
#   cmake --build tests/build
#   ctest --test-dir tests/build --output-on-failure
#
# CMSIS-DSP is taken from the firmware build's checkout if present, otherwise
# downloaded. Override with -DCMSISDSP_DIR=/path/to/CMSIS-DSP.

cmake_minimum_required(VERSION 3.13)
project(chirp_host_tests C)
set(CMAKE_C_STANDARD 11)

set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../src)

if(NOT CMSISDSP_DIR AND EXISTS ${CMAKE_CURRENT_LIST_DIR}/../build/_deps/cmsisdsp-src/Include/arm_math.h)
    set(CMSISDSP_DIR ${CMAKE_CURRENT_LIST_DIR}/../build/_deps/cmsisdsp-src)
endif()
if(NOT CMSISDSP_DIR)
    include(FetchContent)
    FetchContent_Declare(cmsisdsp
        GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP.git
        GIT_TAG "v1.15.0"
    )
    FetchContent_Populate(cmsisdsp)
    set(CMSISDSP_DIR ${cmsisdsp_SOURCE_DIR})
endif()
message(STATUS "CMSIS-DSP: ${CMSISDSP_DIR}")

# CMSIS-DSP's portable C path for non-ARM hosts
add_compile_definitions(__GNUC_PYTHON__ DISABLEFLOAT16)
add_compile_options(-O2 -ffunction-sections -fdata-sections)
add_link_options(-Wl,--gc-sections)

add_library(cmsisdsp_host STATIC
    ${CMSISDSP_DIR}/Source/TransformFunctions/TransformFunctions.c
    ${CMSISDSP_DIR}/Source/CommonTables/CommonTables.c
    ${CMSISDSP_DIR}/Source/ComplexMathFunctions/ComplexMathFunctions.c
    ${CMSISDSP_DIR}/Source/SupportFunctions/SupportFunctions.c
)
target_include_directories(cmsisdsp_host PUBLIC
    ${CMSISDSP_DIR}/Include
    ${CMSISDSP_DIR}/PrivateInclude
)
target_compile_options(cmsisdsp_host PRIVATE -w)

# Firmware DSP sources under test
add_library(chirp_dsp STATIC
    ${SRC_DIR}/chirp_decoder.c
    ${SRC_DIR}/chirp_encoder.c
    ${SRC_DIR}/chirp_symbol_rx.c
    ${SRC_DIR}/cyclic_weighted_centroid.c
    ${SRC_DIR}/rx_sample_window.c
    ${SRC_DIR}/symbol_timing_tracker.c
)
target_include_directories(chirp_dsp PUBLIC
    ${SRC_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/host   # pico/stdlib.h stand-in
)
target_link_libraries(chirp_dsp PUBLIC cmsisdsp_host m)

# Shared test support: channel model and firmware-configured codec
add_library(test_support STATIC
    chirp_channel.c
    chirp_test_rig.c
)
target_include_directories(test_support PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(test_support PRIVATE -Wall -Wextra)
target_link_libraries(test_support PUBLIC chirp_dsp)

enable_testing()

add_executable(test_chirp_codec test_chirp_codec.c)
target_compile_options(test_chirp_codec PRIVATE -Wall -Wextra)
target_link_libraries(test_chirp_codec PRIVATE test_support)
add_test(NAME chirp_codec COMMAND test_chirp_codec)

add_executable(test_timing test_timing.c)
target_compile_options(test_timing PRIVATE -Wall -Wextra)
target_link_libraries(test_timing PRIVATE test_support)
add_test(NAME timing COMMAND test_timing)
