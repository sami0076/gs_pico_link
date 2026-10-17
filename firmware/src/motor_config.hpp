// The numbers that describe the two motor axes: how many steps make a degree,
// how far each axis is allowed to travel, and how fast the driver can be
// pulsed.
//
// This header deliberately includes NOTHING but <stdint.h>. No Pico headers.
// That is what lets the same file compile on the laptop, so the step and limit
// maths can be unit-tested at a desk instead of with a motor turning
// (spec section 4).
//
// Two unit conventions are used throughout, both integers:
//
//   mdeg  millidegrees        45 degrees  -> 45000
//   mdps  millidegrees/second  5 deg/sec  ->  5000
//
// Integers, not floats, because these values cross the USB link as text and
// get compared for exact equality in tests. Degrees are for humans; the CLI
// converts.
//
// IMPORTANT: every angle here is at the OUTPUT - the antenna itself, what you
// measure with a protractor - not at the motor shaft. The gearbox is already
// folded into the steps-per-degree numbers below.
#pragma once

#include <stdint.h>

// ============================================================================
// Per-axis description
// ============================================================================
struct AxisSpec {
    // Steps per output degree, kept as an exact fraction steps_num/steps_den
    // rather than a decimal. 50.7937... does not divide evenly, and rounding
    // it here would make the error grow over a long move.
    int32_t steps_num;
    int32_t steps_den;

    // How far this axis may travel, in millidegrees. There are no limit
    // switches on this machine, so these two numbers are the only thing
    // stopping a move from driving the gimbal into a hard stop.
    int32_t min_mdeg;
    int32_t max_mdeg;

    // Set true if a positive command turns the axis the wrong way.
    // Which way the antenna actually moves depends on the driver's SW5 switch
    // and on the gear mesh, so it cannot be known until it is tried.
    bool invert_dir;

    // The speed a move begins and ends at, rather than starting from a dead
    // stop. See the note below on why this is 5 deg/sec.
    int32_t v_start_mdps;

    // How quickly the axis is allowed to change speed, in millideg/sec/sec.
    int32_t accel_mdps2;
};

// Why moves start at 5 deg/sec instead of from zero.
//
// A stepper does not need to ease away from rest at these speeds - 5 deg/sec
// is about 5 rpm at the azimuth motor, behind a 22.86:1 gearbox. Starting
// there instead of at zero buys two things:
//
//   1. The whole written test procedure runs at 5 deg/sec. With the ramp
//      starting at exactly that speed, those moves have no ramp at all - they
//      are pure constant speed, and take exactly angle/speed seconds. That
//      keeps the test data clean and easy to check by hand.
//   2. Starting from a true zero makes the first step interval enormous
//      (tens of milliseconds), which looks like the program has hung.
//
// Ramping only comes into play during the speed sweep, which is the one part
// of the procedure that already expects it.
constexpr int32_t V_START_MDPS = 5000;  // 5 deg/sec

// ----------------------------------------------------------------------------
// Azimuth (the board calls it Servo 1)
// ----------------------------------------------------------------------------
// Where 6400/63 comes from:
//
//   driver pulses per motor rev           1600        (CL57TE DIP switches)
//   gearbox                              10 : 1       (planetary)
//   final gear pair                      32 : 14      (= 16/7)
//   -> gear ratio                        160/7 : 1    (22.857 : 1)
//
//   steps per output rev    = 1600 * 160/7 = 256000/7
//   steps per output degree = 256000/7/360 = 6400/63 = 101.5873...
//
// THE 1600 IS A SWITCH SETTING, AND IT IS THE ONE NUMBER HERE THAT NOTHING
// IN SOFTWARE CAN CHECK. It is DIP switches 1-4 on each driver, which the
// CL57T manual's table reads as:
//
//   1600 pulses/rev (microstep 8)   SW1 on   SW2 off  SW3 on   SW4 on
//    800 pulses/rev (microstep 4)   SW1 off  SW2 on   SW3 on   SW4 on
//
// The boxes on this machine are set to 1600, checked by eye at the bench on
// 17 Oct 2026, and these numbers follow the hardware rather than the other
// way round. If a driver is ever swapped or reset, check this first: get it
// wrong and every angle is out by a clean factor of two, the Pico reports
// the angle it meant to turn, and nothing anywhere will contradict it.
//
// Count the gear teeth yourself too. If a gear was swapped, every angle in
// the test data will be wrong by the same factor.
// Acceleration is deliberately low to begin with. Nobody has measured the
// supply voltage or the driver's current setting yet, and both limit how hard
// the motor can actually push. 45 deg/sec/sec still reaches the top speed of
// 90 deg/sec in two seconds. Raise it once the machine has run.
constexpr AxisSpec AZ_SPEC = {
    6400,   // steps_num
    63,     // steps_den     -> 101.5873 steps per degree
    0,      // min_mdeg      0 degrees
    360000, // max_mdeg      360 degrees, one full turn
    false,  // invert_dir    unverified - see the bring-up notes
    V_START_MDPS,
    45000,  // accel_mdps2   45 deg/sec/sec (conservative; target 180)
};

// ----------------------------------------------------------------------------
// Elevation (the board calls it Servo 2)
// ----------------------------------------------------------------------------
//   final gear pair  20 : 14 (= 10/7)  ->  gear ratio 100/7 : 1 (14.286 : 1)
//   steps per output degree = 1600 * 100/7 / 360 = 4000/63 = 63.492...
//
// The travel limit here is deliberately tighter than the real mechanism.
// The documented range is -10 to +90 degrees, but nobody has measured where
// the hard stops actually are, and elevation is the axis that carries weight.
// Starting narrow means a wrong direction sign or a bad zero runs out of
// commanded travel long before it reaches anything solid.
//
// Widen this to -10000 / 90000 once the stops have been measured.
// Elevation accelerates at half the azimuth rate. It lifts a gravity-loaded
// mass and has less gear reduction to help it, so it has the harder job.
constexpr AxisSpec EL_SPEC = {
    4000,   // steps_num
    63,     // steps_den     -> 63.492 steps per degree
    0,      // min_mdeg      0 degrees   (conservative; real limit is -10)
    45000,  // max_mdeg      45 degrees  (conservative; real limit is +90)
    false,  // invert_dir    unverified - expect this one to need flipping
    V_START_MDPS,
    30000,  // accel_mdps2   30 deg/sec/sec (conservative; target 90)
};

// ============================================================================
// What level each pin sits at
// ============================================================================
// Both signals are active low: the driver's + side is tied to +5V and the
// Pico pulls the - side down. So "on" means driving the pin LOW.
constexpr bool PUL_IDLE  = true;   // HIGH between steps
constexpr bool PUL_PULSE = false;  // LOW makes one step

// Which level the DIR pin needs for a move in the positive direction.
//
// Two things combine here, and getting either backwards sends the antenna
// the wrong way:
//
//   active low      positive means driving DIR LOW, not HIGH
//   invert_dir      set per axis when the gearing reverses the rotation
//
// Written as one expression so there is a single place to be right about it,
// and so the host tests can check all four combinations.
constexpr bool dir_level_for(const AxisSpec &axis, bool positive)
{
    return (positive != axis.invert_dir) ? false : true;
}

// ============================================================================
// Pulse timing
// ============================================================================
// The CL57TE manual's minimums are a 1 us pulse and 5 us of DIR setup. The
// values below are what last year's ground station actually ran, which is far
// more generous. There is no reason to shave these down: at the fastest speed
// this project uses, one step is still about 219 us away from the next.
constexpr uint32_t PULSE_LOW_US    = 5;    // how long PUL is held LOW
constexpr uint32_t DIR_SETUP_US    = 100;  // settle time after changing DIR
constexpr uint32_t REVERSE_PAUSE_MS = 1;   // extra pause before reversing

// ============================================================================
// Which pins the PIN command may touch
// ============================================================================
// The wiring itself lives in config.hpp, next to the rest of the pin map.
// These numbers are repeated here because this header has to stay free of
// Pico headers to be host-testable, and the decision about what PIN is
// allowed to do belongs with the logic rather than with the wiring.
// static_asserts in config.hpp stop the two copies drifting apart.
//
// Two kinds of pin are allowed, for two different reasons.
//
// GP12-GP15 are spare. Nothing is attached, so setting one high or low
// proves the laptop can reach the Pico and the meter can read it, and
// nothing can come of getting it wrong.
//
// GP4-GP7 are the motor drivers' step and direction lines. Being able to
// hold one steady is the only way to check that wiring with a multimeter -
// a step pulse is five microseconds long and no meter will ever see it.
// They were shut out of PIN at first for good reason, and the reason has
// an answer rather than needing a refusal: driving a PUL line low makes
// the driver take a step that the firmware does not count, so afterwards
// its idea of where the axis is points one step away from the truth. So
// poking a PUL line gives up that axis's zero, exactly as an emergency
// stop does, and the next move is refused until the operator re-zeros.
//
// Driving a DIR line moves nothing - the direction is set again at the
// start of every move - so that one costs nothing and keeps its zero.
constexpr uint8_t SPARE_GPIO_MIN = 12;
constexpr uint8_t SPARE_GPIO_MAX = 15;

constexpr uint8_t MOTOR_GPIO_AZ_PUL = 4;
constexpr uint8_t MOTOR_GPIO_AZ_DIR = 5;
constexpr uint8_t MOTOR_GPIO_EL_PUL = 6;
constexpr uint8_t MOTOR_GPIO_EL_DIR = 7;

constexpr bool gpio_is_spare(uint8_t gpio)
{
    return gpio >= SPARE_GPIO_MIN && gpio <= SPARE_GPIO_MAX;
}

constexpr bool gpio_is_motor_step(uint8_t gpio)
{
    return gpio == MOTOR_GPIO_AZ_PUL || gpio == MOTOR_GPIO_EL_PUL;
}

constexpr bool gpio_is_motor_direction(uint8_t gpio)
{
    return gpio == MOTOR_GPIO_AZ_DIR || gpio == MOTOR_GPIO_EL_DIR;
}

constexpr bool gpio_may_be_poked(uint8_t gpio)
{
    return gpio_is_spare(gpio) || gpio_is_motor_step(gpio) ||
           gpio_is_motor_direction(gpio);
}

// Which axis a motor pin belongs to, for taking its zero away.
constexpr bool gpio_is_azimuth(uint8_t gpio)
{
    return gpio == MOTOR_GPIO_AZ_PUL || gpio == MOTOR_GPIO_AZ_DIR;
}

// ============================================================================
// Speed range
// ============================================================================
// The floor exists so a typo like "0" cannot start a move that never ends.
// The ceiling is last year's limit. It is not a stall limit - 90 deg/sec is
// only about 343 rpm at the azimuth motor, which a closed-loop NEMA 23 does
// not notice - it is just the fastest anyone has reason to command.
constexpr int32_t SPEED_MIN_MDPS = 100;    //  0.1 deg/sec
constexpr int32_t SPEED_MAX_MDPS = 90000;  // 90   deg/sec

// Sanity rails on the gap between two steps. Nothing should ever reach these:
// the fastest this project runs is 90 deg/sec on azimuth, which is one step
// every 219 us, and the slowest is 0.1 deg/sec on elevation, one step every
// 315 ms. If a computed interval hits a rail, the maths is wrong, and failing
// toward "too slow" is the safe direction.
constexpr uint32_t MIN_INTERVAL_US = 50;      // 20 kHz, far above anything used
constexpr uint32_t MAX_INTERVAL_US = 500000;  // half a second between steps
