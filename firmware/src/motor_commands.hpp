// Deciding what to do about a command, and what to say back.
//
// PURE C++. No Pico headers, so this is covered by firmware/host_tests and
// by the contract vectors the laptop is held to.
//
// This used to live inside motor_main.cpp, where it could not be tested at
// all: the one file that decides whether a move is allowed, which refusal a
// bad command earns, and what order the replies go out in was also the file
// full of GPIO and USB calls. The laptop's simulator mirrors these rules,
// so a difference between the two is a dry run that rehearses the wrong
// machine - and there was nothing to catch it.
//
// The split is: this file decides, motor_main.cpp does. decide() is handed
// a flat description of the machine and gives back the lines to send and
// one thing to do. It touches no hardware and keeps no state of its own.
//
// ORDER MATTERS, AND IT IS THE CALLER'S JOB. Send the lines first, then
// carry out the action. A move's ACK and PLAN have to be on the wire before
// the first pulse, because nothing may be printed between the first pulse
// and the last.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "motion_profile.hpp"
#include "motor_cmd.hpp"
#include "motor_reply.hpp"

// What the firmware knows about itself at the moment a command arrives.
//
// Flat data rather than a pointer to the Steppers, so a test can describe a
// machine that is halfway through a move with an unzeroed elevation axis
// without needing either.
struct MachineState {
    // True while either axis is turning. One axis moving blocks commands
    // for both, which is the firmware's rule and not an accident.
    bool busy = false;

    bool az_zeroed = false;
    bool el_zeroed = false;
    int32_t az_pos_steps = 0;
    int32_t el_pos_steps = 0;

    uint32_t uptime_ms = 0;
    const char *target = "";
    const char *version = "";
};

// The one thing the caller should do, once the replies are sent.
enum class Act {
    Nothing,
    StartMove,
    RequestStop,  // both axes, a ramped stop
    Estop,        // both axes, at once
    SetZero,
    SetPin,
};

struct Decision {
    Act act = Act::Nothing;

    // StartMove
    AxisId axis = AxisId::None;
    Move move{};
    uint16_t id = 0;

    // SetZero: which axes to place, and where.
    bool zero_az = false;
    bool zero_el = false;
    int32_t az_steps = 0;
    int32_t el_steps = 0;

    // SetPin
    uint8_t gpio = 0;
    uint8_t level = 0;
    // Driving a step line makes the driver take a step nobody counted, so
    // that axis no longer knows where it is.
    bool clear_az_zero = false;
    bool clear_el_zero = false;

    // Up to two complete lines, in the order they go out. Length 0 means
    // there is nothing to send - which is what a stop during a move looks
    // like, since it is answered by the move's own DONE.
    char line1[MAX_REPLY_LEN] = {};
    size_t len1 = 0;
    char line2[MAX_REPLY_LEN] = {};
    size_t len2 = 0;
};

// Work out what to do about one command. Never touches hardware.
Decision decide(const Command &c, const MachineState &s);
