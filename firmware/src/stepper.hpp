// Driving one stepper axis: two pins, a direction, and a count of where we
// think the axis is.
//
// This file DOES use Pico headers, so it is not host-testable. That is why it
// is kept thin - the arithmetic lives in motion_profile and motor_config,
// which are tested on the laptop. What is left here is pin wiggling.
//
// About the position count: it goes up or down by one for every pulse this
// code sends out. It is what the firmware BELIEVES, not what the gimbal has
// done. The driver's ALM line is not wired, so if a drive faults or the
// mechanism jams, the pulses keep going out and the count keeps rising while
// the antenna stands still. There is no way for the firmware to notice. The
// operator watching the red LED on the driver is the only check.
#pragma once

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "motion_profile.hpp"
#include "motor_config.hpp"

class Stepper {
public:
    Stepper(const AxisSpec &spec, uint pin_pul, uint pin_dir)
        : spec_(spec), pin_pul_(pin_pul), pin_dir_(pin_dir)
    {
    }

    // Claim the two pins as outputs and park them at their idle levels.
    //
    // Push-pull outputs, always. Never open-drain, never inputs: the +5V rail
    // sits on the other side of the optocoupler, and Pico pins are not 5V
    // tolerant, so letting a pin float lets that rail push current into it.
    void init();

    // Read the two pins back and check they are at the levels init() set.
    // Catches a pin that was never configured, or one held down by something
    // external. It cannot tell whether anything is plugged in.
    bool pins_ok() const;

    // Point the axis the right way and wait for the driver to notice.
    //
    // Reversing gets an extra pause: the drive wants the direction settled
    // well before the next pulse edge, and a reversal is where that is
    // tightest. Only call this while stopped.
    void set_direction(bool positive);

    // Send one pulse and count it.
    void step();

    // --- running a move ----------------------------------------------------

    // Begin a move that plan_move() has already approved. This sets the
    // direction, which takes a moment, so it is not called while moving.
    void start(const Move &m, uint16_t id);

    // Called as often as possible. Sends a pulse if one is due, otherwise
    // returns straight away. Returns true while the move is still running.
    bool tick(absolute_time_t now);

    // Slow down and stop. The axis keeps stepping for as long as it takes to
    // come down from its current speed, so nothing is jerked to a halt.
    void request_stop();

    // Stop on the next loop, no deceleration. This also clears the zero,
    // because after an abrupt halt the driver's real position is no longer
    // something we can claim to know.
    void estop();

    bool moving() const { return moving_; }
    bool aborted() const { return aborted_; }

    // Non-zero between starting a move and reporting it finished, so the
    // main loop knows there is still a DONE owing for this axis.
    uint16_t move_id() const { return move_id_; }
    void clear_move_id() { move_id_ = 0; }
    const Move &plan() const { return plan_; }

    // Signed, so it reads the same way as the commanded step count.
    int32_t steps_done() const;

    // First pulse to last pulse. The serial round trip is deliberately not
    // included - this is how long the axis was actually turning.
    uint32_t elapsed_us() const;

    // --- where we think we are --------------------------------------------
    int32_t pos_steps() const { return pos_steps_; }
    int32_t pos_mdeg() const { return steps_to_mdeg(spec_, pos_steps_); }

    // Declare the current position. This is what ZERO does.
    void set_pos_steps(int32_t steps)
    {
        pos_steps_ = steps;
        zeroed_    = true;
    }

    // Until an axis has been zeroed, the soft limits are measured from an
    // unknown origin, which is no protection at all. Moves are refused.
    bool zeroed() const { return zeroed_; }
    void clear_zero() { zeroed_ = false; }

    const AxisSpec &spec() const { return spec_; }
    uint pin_pul() const { return pin_pul_; }
    uint pin_dir() const { return pin_dir_; }

private:
    const AxisSpec &spec_;
    uint pin_pul_;
    uint pin_dir_;

    int32_t pos_steps_ = 0;
    bool zeroed_       = false;

    // Which way the DIR pin is currently pointing, so a reversal can be
    // spotted. Starts matching what init() writes.
    bool dir_positive_ = true;

    // --- the move in progress ---------------------------------------------
    Move plan_          = {};
    uint32_t index_     = 0;  // how many pulses sent so far
    bool moving_        = false;
    bool aborted_       = false;
    uint16_t move_id_   = 0;

    absolute_time_t next_step_time_ = {};
    absolute_time_t first_pulse_    = {};
    absolute_time_t last_pulse_     = {};
};
