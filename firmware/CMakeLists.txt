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
# Build stamp for the startup banner (spec 5.3 step 3)
# ----------------------------------------------------------------------------
# So a Pico you found in a drawer can tell you which build is on it.
#
# Two wrinkles worth knowing:
#
#  - safe.directory: we build in WSL but the repo lives on a Windows drive under
#    /mnt/c, so the checkout is owned by a different uid than the one running
#    git. Git calls that "dubious ownership" and refuses. Passing safe.directory
#    for this one invocation fixes it without touching anyone's global config.
#  - Captured at CONFIGURE time, not build time. Commit and then only run ninja
#    and this string is from the last cmake run. Re-run cmake to refresh it.
get_filename_component(GS_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)

execute_process(
    COMMAND git -c "safe.directory=${GS_REPO_ROOT}" -C "${GS_REPO_ROOT}"
            describe --always --dirty --tags
    OUTPUT_VARIABLE GS_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_VARIABLE GS_VERSION_ERR
    RESULT_VARIABLE GS_VERSION_RESULT)

if(NOT GS_VERSION_RESULT EQUAL 0 OR NOT GS_VERSION)
    set(GS_VERSION "unknown")
    # Loud on purpose: a banner reading "unknown" should not be a silent mystery.
    message(WARNING
        "Could not read a version from git, banner will say \"unknown\".\n"
        "  git said: ${GS_VERSION_ERR}")
endif()

message(STATUS "gs_pico_link version stamp: ${GS_VERSION}")

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
        GS_TARGET_NAME=\"${target}\"
        GS_VERSION=\"${GS_VERSION}\"
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

# Motor actuation test. No radio: the test procedure has the radio hardware
# disconnected, and this is the build you flash for that.
#
# It still links RadioLib, because gs_add_role() gives every target the same
# treatment and config.hpp includes <RadioLib.h>. That costs some flash and
# nothing else, which is a fair price for not having two ways to define a
# target.
gs_add_role(gs_motor
    SOURCES src/motor_main.cpp
            src/stepper.cpp
            src/motion_profile.cpp
            src/motor_cmd.cpp
            src/motor_reply.cpp
            src/line_reader.cpp)
