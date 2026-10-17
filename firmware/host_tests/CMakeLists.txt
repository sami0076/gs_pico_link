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
gs_add_host_test(test_line_reader ../src/line_reader.cpp)
gs_add_host_test(test_motor_cmd ../src/motor_cmd.cpp)
gs_add_host_test(test_motor_reply ../src/motor_reply.cpp ../src/motor_cmd.cpp
                                  ../src/motion_profile.cpp)

# ----------------------------------------------------------------------------
# The contract with the laptop
# ----------------------------------------------------------------------------
# gen_motor_vectors runs the firmware's own code and writes down what it
# produced. The file is checked into the repo because the C++ builds in WSL
# and the Python runs on Windows, and a file that only exists in one of those
# is no use to the other.
#
# Two tests, in order: write a fresh copy, then compare it with the one that
# is checked in. A protocol change made on the C++ side and not regenerated
# fails here, which is the point - otherwise the pytest would keep passing
# against a file describing firmware that no longer exists.
set(GS_VECTORS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../laptop/tests/data/motor_vectors.txt)

add_executable(gen_motor_vectors gen_motor_vectors.cpp
               ../src/motor_reply.cpp ../src/motor_cmd.cpp
               ../src/motion_profile.cpp)

add_test(NAME write_motor_vectors
         COMMAND gen_motor_vectors
                 ${CMAKE_CURRENT_BINARY_DIR}/motor_vectors.txt)

add_test(NAME motor_vectors_are_current
         COMMAND ${CMAKE_COMMAND} -E compare_files
                 ${CMAKE_CURRENT_BINARY_DIR}/motor_vectors.txt
                 ${GS_VECTORS})

# compare_files only means anything once the fresh copy exists.
set_tests_properties(write_motor_vectors PROPERTIES FIXTURES_SETUP vectors)
set_tests_properties(motor_vectors_are_current
                     PROPERTIES FIXTURES_REQUIRED vectors)
