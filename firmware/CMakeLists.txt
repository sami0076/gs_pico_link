# Builds every Pico role for gs_pico_link from this one file.
#
# The role is chosen by WHICH TARGET YOU BUILD, never by editing a #define, so
# there is no way to flash a Pico with the wrong job by forgetting to change a
# line back:
#
#   gs_rx        receiver: listens, prints one text line per packet
#   gs_tx        transmitter, simulated GGA (the normal bench transmitter)
#   gs_tx_diag   transmitter, diagnostic counter instead of a position
#   gs_tx_gps    transmitter, forwards GGA from a real GPS on UART0
#
# gs_tx_diag is a fourth target rather than a CMake option (the spec allowed
# either). A target means all four roles exist in one build tree at once, so
# switching roles is a reflash, not a reconfigure.
#
# Build (inside WSL - see README):
#   cmake -S firmware -B ~/build/gs -G Ninja && ninja -C ~/build/gs
cmake_minimum_required(VERSION 3.18)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pico 2 W / RP2350. Must be set before the SDK is pulled in, because that is
# where the SDK picks the cross-compiler for this chip.
set(PICO_BOARD pico2_w CACHE STRING "Target board")

# Must come before project(): this swaps in the ARM toolchain.
include(pico_sdk_import.cmake)

project(gs_pico_link C CXX ASM)

pico_sdk_init()

# ----------------------------------------------------------------------------
# RadioLib
# ----------------------------------------------------------------------------
# Wired up the way RadioLib's own examples/NonArduino/Pico does it: build the
# library from the submodule, and compile its Pico HAL straight into each
# executable. The HAL is not part of the library target upstream, so it has to
# be listed as a source.
set(RADIOLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/RadioLib")
set(RADIOLIB_PICO_HAL "${RADIOLIB_DIR}/src/hal/RPiPico/PicoHal.cpp")

if(NOT EXISTS "${RADIOLIB_PICO_HAL}")
    message(FATAL_ERROR
        "RadioLib is missing. It is a git submodule:\n"
        "  git submodule update --init --recursive")
endif()

add_subdirectory("${RADIOLIB_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/RadioLib")

# ----------------------------------------------------------------------------
# One function that defines a role, so the four targets can't drift apart
# ----------------------------------------------------------------------------
# Every target gets identical treatment except its sources and its -D flags.
function(gs_add_role target)
    cmake_parse_arguments(ARG "" "" "SOURCES;DEFINES;LIBS" ${ARGN})

    add_executable(${target} ${ARG_SOURCES} "${RADIOLIB_PICO_HAL}")

    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

    target_compile_definitions(${target} PRIVATE
        RADIOLIB_BUILD_RPI_PICO
        ${ARG_DEFINES})

    # Strict warnings for OUR code. Deliberately per-target and not a global
    # add_compile_options: RadioLib builds as its own target and we don't want
    # to turn its warnings into our noise, or be tempted to patch the library.
    target_compile_options(${target} PRIVATE -Wall -Wextra)

    target_link_libraries(${target} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_gpio
        hardware_timer
        hardware_pwm
        pico_multicore
        RadioLib
        ${ARG_LIBS})

    # The laptop talks to these over USB. UART stdio stays off so the GPS pins
    # are never fighting a console for the same wires.
    pico_enable_stdio_usb(${target} 1)
    pico_enable_stdio_uart(${target} 0)

    # .uf2 for drag-and-drop flashing, plus .elf/.map for debugging.
    pico_add_extra_outputs(${target})
endfunction()

# ----------------------------------------------------------------------------
# The four roles
# ----------------------------------------------------------------------------
gs_add_role(gs_rx
    SOURCES src/rx_main.cpp)

gs_add_role(gs_tx
    SOURCES src/tx_main.cpp
    DEFINES GS_TX_USE_REAL_GPS=0 GS_TX_MODE_DIAG=0)

gs_add_role(gs_tx_diag
    SOURCES src/tx_main.cpp
    DEFINES GS_TX_USE_REAL_GPS=0 GS_TX_MODE_DIAG=1)

gs_add_role(gs_tx_gps
    SOURCES src/tx_main.cpp
    DEFINES GS_TX_USE_REAL_GPS=1 GS_TX_MODE_DIAG=0
    LIBS hardware_uart)
