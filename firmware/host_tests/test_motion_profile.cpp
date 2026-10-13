// Tests for the angle <-> step conversion.
//
// Expected step counts were worked out by hand from the gear ratios, not by
// running the code and writing down whatever it printed. A test that just
// records current behaviour cannot catch a wrong constant.
#include "check.hpp"
#include "motion_profile.hpp"

// Azimuth is 3200/63 = 50.7937 steps per degree.
static void test_azimuth_angles()
{
    CHECK_EQ(mdeg_to_steps(AZ_SPEC, 0), 0);

    // 1 deg  = 50.79  -> 51
    CHECK_EQ(mdeg_to_steps(AZ_SPEC, 1000), 51);

    // 45 deg = 2285.71 -> 2286. This is the test procedure's first command.
    CHECK_EQ(mdeg_to_steps(AZ_SPEC, 45000), 2286);

    // 180 deg = 9142.86 -> 9143. Used by the speed sweep.
    CHECK_EQ(mdeg_to_steps(AZ_SPEC, 180000), 9143);

    // 360 deg = 18285.71 -> 18286. The largest move this axis allows, and the
    // one that would overflow a 16-bit step count if anyone tried to use one.
    CHECK_EQ(mdeg_to_steps(AZ_SPEC, 360000), 18286);
}

// Elevation is 2000/63 = 31.746 steps per degree.
static void test_elevation_angles()
{
    CHECK_EQ(mdeg_to_steps(EL_SPEC, 0), 0);

    // 1 deg  = 31.75   -> 32
    CHECK_EQ(mdeg_to_steps(EL_SPEC, 1000), 32);

    // 45 deg = 1428.57 -> 1429
    CHECK_EQ(mdeg_to_steps(EL_SPEC, 45000), 1429);

    // 90 deg = 2857.14 -> 2857. Outside the shipped soft limit, but the
    // conversion itself must still be right for when the limit is widened.
    CHECK_EQ(mdeg_to_steps(EL_SPEC, 90000), 2857);
}

// Turning one way must take exactly as many steps as turning back.
// If rounding were lopsided, a +45/-45 pair would not return to the start,
// and the error would accumulate over the ten random angles in the procedure.
static void test_negative_is_symmetric()
{
    CHECK_EQ(mdeg_to_steps(AZ_SPEC, -45000), -2286);
    CHECK_EQ(mdeg_to_steps(AZ_SPEC, -180000), -9143);
    CHECK_EQ(mdeg_to_steps(EL_SPEC, -45000), -1429);

    // The general rule, across a spread of angles.
    for (int32_t mdeg = 0; mdeg <= 360000; mdeg += 1234) {
        CHECK_EQ(mdeg_to_steps(AZ_SPEC, -mdeg), -mdeg_to_steps(AZ_SPEC, mdeg));
        CHECK_EQ(mdeg_to_steps(EL_SPEC, -mdeg), -mdeg_to_steps(EL_SPEC, mdeg));
    }
}

// Going back the other way: what angle do those steps actually represent?
static void test_steps_back_to_angle()
{
    CHECK_EQ(steps_to_mdeg(AZ_SPEC, 0), 0);

    // 2286 steps is 45.006 deg, not 45.000. This is the quantisation the test
    // data table has to report - the antenna cannot stop at exactly 45.
    CHECK_EQ(steps_to_mdeg(AZ_SPEC, 2286), 45006);

    // 9143 steps is 180.003 deg.
    CHECK_EQ(steps_to_mdeg(AZ_SPEC, 9143), 180003);

    // Elevation: 1429 steps is 45.014 deg.
    CHECK_EQ(steps_to_mdeg(EL_SPEC, 1429), 45014);

    CHECK_EQ(steps_to_mdeg(AZ_SPEC, -2286), -45006);
}

// A round trip should land within half a step of where it started. It will not
// land exactly, and that is expected - see steps_to_mdeg's comment.
static void test_round_trip_is_within_one_step()
{
    // Half a step, in millidegrees, rounded up: az 10, el 16.
    const int32_t az_tol = 10;
    const int32_t el_tol = 16;

    for (int32_t mdeg = -360000; mdeg <= 360000; mdeg += 777) {
        const int32_t az_back =
            steps_to_mdeg(AZ_SPEC, mdeg_to_steps(AZ_SPEC, mdeg));
        CHECK(az_back - mdeg <= az_tol && mdeg - az_back <= az_tol);

        const int32_t el_back =
            steps_to_mdeg(EL_SPEC, mdeg_to_steps(EL_SPEC, mdeg));
        CHECK(el_back - mdeg <= el_tol && mdeg - el_back <= el_tol);
    }
}

// The constants themselves, checked against the gear maths one more time:
//   azimuth    800 pulses/rev * (10 * 32/14) / 360
//   elevation  800 pulses/rev * (10 * 20/14) / 360
static void test_ratios_match_the_gears()
{
    CHECK_EQ(AZ_SPEC.steps_num, 3200);
    CHECK_EQ(AZ_SPEC.steps_den, 63);
    CHECK_EQ(EL_SPEC.steps_num, 2000);
    CHECK_EQ(EL_SPEC.steps_den, 63);

    // 1000 degrees is a convenient way to check the ratio to 3 decimals
    // without floating point: 50793.7 -> 50794, 31746.0 -> 31746.
    CHECK_EQ(mdeg_to_steps(AZ_SPEC, 1000000), 50794);
    CHECK_EQ(mdeg_to_steps(EL_SPEC, 1000000), 31746);
}

int main()
{
    test_azimuth_angles();
    test_elevation_angles();
    test_negative_is_symmetric();
    test_steps_back_to_angle();
    test_round_trip_is_within_one_step();
    test_ratios_match_the_gears();
    return test_summary();
}
