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
