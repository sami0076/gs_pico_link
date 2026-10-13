# Tests that run on the laptop, not on the Pico.
#
# This is a separate CMake project from firmware/CMakeLists.txt on purpose. It
# uses the normal host compiler and never touches the Pico SDK, so it builds in
# seconds and needs no hardware. Only files that are free of Pico headers can
# be tested here - that is why the maths lives in its own source files (spec
# section 4).
#
# Build and run:
#   cmake -S firmware/host_tests -B ~/build/gs-host -G Ninja
#   ctest --test-dir ~/build/gs-host --output-on-failure
cmake_minimum_required(VERSION 3.18)

project(gs_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -Werror here but not in the firmware build: this code is all ours, so there
# is no third-party library whose warnings we would be forced to fix.
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# The code under test, and this directory for check.hpp.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src ${CMAKE_CURRENT_SOURCE_DIR})

enable_testing()

# One executable per test file, so a crash in one does not hide the others.
# Add new tests by listing the extra .cpp files they need.
function(gs_add_host_test name)
    add_executable(${name} ${name}.cpp ${ARGN})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

gs_add_host_test(test_motion_profile ../src/motion_profile.cpp)
