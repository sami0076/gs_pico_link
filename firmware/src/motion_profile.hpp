// Turning an angle into a number of steps, and back.
//
// PURE C++. No Pico headers, so this compiles on the laptop and is covered by
// firmware/host_tests (spec section 4).
//
// This is the piece of maths that decides whether the antenna ends up where
// the operator asked, so it is kept small enough to check by hand.
#pragma once

#include <stdint.h>

#include "motor_config.hpp"

// How many steps to move this axis by the given angle.
//
// Rounds to the nearest whole step, away from zero on a tie, so that +X and -X
// always give the same number of steps in opposite directions.
int32_t mdeg_to_steps(const AxisSpec &axis, int32_t mdeg);

// The angle those steps actually represent.
//
// This is NOT the inverse of mdeg_to_steps. A step is a fixed size and most
// angles do not land on one, so asking for 45.000 deg on azimuth really moves
// 45.006 deg. That difference is the honest answer and belongs in the test
// data, which is why it is reported rather than hidden.
int32_t steps_to_mdeg(const AxisSpec &axis, int32_t steps);
