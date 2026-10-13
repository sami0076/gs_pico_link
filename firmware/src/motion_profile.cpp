#include "motion_profile.hpp"

// Divide a by b and round to the nearest whole number, going away from zero
// when the answer lands exactly halfway.
//
// C++ integer division truncates toward zero, so 2285.7 would become 2285 and
// the antenna would stop a third of a step short on every single move. Adding
// half the divisor before dividing fixes that.
//
// b is always positive here. a can be negative, and the sign is handled by
// doing the work on the magnitude and putting the sign back afterwards -
// otherwise rounding would be lopsided and +45 and -45 would disagree.
static int64_t div_round(int64_t a, int64_t b)
{
    if (a >= 0) {
        return (a + b / 2) / b;
    }
    return -((-a + b / 2) / b);
}

int32_t mdeg_to_steps(const AxisSpec &axis, int32_t mdeg)
{
    // steps = mdeg/1000 * steps_num/steps_den, rearranged to divide only once.
    //
    // int64 because the top can get large: a full 360 deg turn of azimuth is
    // 360000 * 3200 = 1.15 billion, which is close enough to the int32 ceiling
    // to be worth not thinking about.
    const int64_t top    = static_cast<int64_t>(mdeg) * axis.steps_num;
    const int64_t bottom = static_cast<int64_t>(axis.steps_den) * 1000;
    return static_cast<int32_t>(div_round(top, bottom));
}

int32_t steps_to_mdeg(const AxisSpec &axis, int32_t steps)
{
    const int64_t top    = static_cast<int64_t>(steps) * axis.steps_den * 1000;
    const int64_t bottom = axis.steps_num;
    return static_cast<int32_t>(div_round(top, bottom));
}

PlanResult plan_move(const AxisSpec &axis, int32_t pos_steps,
                     int32_t delta_mdeg, int32_t speed_mdps, Move *out)
{
    const int32_t delta_steps  = mdeg_to_steps(axis, delta_mdeg);
    const int32_t target_steps = pos_steps + delta_steps;

    // Fill this in before any check can fail, so an error message can report
    // where the move would have ended up.
    out->steps      = delta_steps;
    out->from_mdeg  = steps_to_mdeg(axis, pos_steps);
    out->to_mdeg    = steps_to_mdeg(axis, target_steps);
    out->speed_mdps = speed_mdps;

    if (speed_mdps < SPEED_MIN_MDPS || speed_mdps > SPEED_MAX_MDPS) {
        return PlanResult::BadSpeed;
    }

    // A tiny angle can round to no steps at all. Say so rather than reporting
    // a successful move that does nothing - otherwise the operator writes a
    // number in the data table for a move that never happened.
    if (delta_steps == 0) {
        return PlanResult::NoMovement;
    }

    // Compare in steps, not degrees.
    //
    // The limits are written in degrees, but the machine only moves in whole
    // steps, and converting the result back to degrees rounds a second time.
    // Checking "360 degrees" that way fails: 18286 steps reads back as
    // 360.006 deg, which looks past a 360.000 limit even though it is the
    // correct endpoint. Converting the limit into steps once removes the
    // second rounding and lets a move land exactly on the boundary.
    const int32_t min_steps = mdeg_to_steps(axis, axis.min_mdeg);
    const int32_t max_steps = mdeg_to_steps(axis, axis.max_mdeg);

    if (target_steps < min_steps || target_steps > max_steps) {
        return PlanResult::OutOfRange;
    }

    return PlanResult::Ok;
}
