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

// ============================================================================
// Working out whether a move is allowed
// ============================================================================

// Why a move was refused, or Ok if it was not.
enum class PlanResult {
    Ok,
    BadSpeed,    // speed outside SPEED_MIN_MDPS..SPEED_MAX_MDPS
    OutOfRange,  // the move would finish outside the axis's soft limits
    NoMovement,  // the angle is so small it rounds to zero steps
};

// What a move will do. Filled in even when the move is refused, so an error
// message can say where the move would have ended up.
struct Move {
    int32_t steps;       // signed - how many steps, and which way
    int32_t from_mdeg;   // where the axis is now
    int32_t to_mdeg;     // where it will really stop, after rounding to steps
    int32_t speed_mdps;  // as commanded
};

// Check a command and work out the move it describes.
//
// pos_steps is where the axis is now, counted in steps rather than degrees.
// Steps are what the machine actually does, so counting them is exact; storing
// an angle instead would re-round on every move and drift.
//
// delta_mdeg is relative and signed, because that is how the test procedure is
// written ("+45, then -45") and what the operator types.
//
// Nothing here moves a motor. It only decides whether the move is allowed.
PlanResult plan_move(const AxisSpec &axis, int32_t pos_steps,
                     int32_t delta_mdeg, int32_t speed_mdps, Move *out);
