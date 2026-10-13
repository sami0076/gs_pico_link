// Turning a line of text from the laptop into a command.
//
// PURE C++. No Pico headers, so this is covered by firmware/host_tests.
//
// The commands, one per line:
//
//   MOVE,<id>,<AZ|EL>,<speed_mdps>,<delta_mdeg>   turn an axis
//   STOP,<id>                                     slow to a stop
//   ESTOP,<id>                                    stop at once
//   ZERO,<id>,<AZ|EL|ALL>[,<mdeg>]                say where the axis is now
//   POS,<id>                                      report both positions
//   PIN,<id>,<gpio>,<0|1>                         set a pin high or low
//   LIMITS,<id>                                   report the soft limits
//   PING,<id>                                     are you there?
//
// Every command carries an id that the Pico echoes back in its replies. A
// move takes up to a minute, during which other commands can be sent, so the
// id is how the laptop tells which reply belongs to which command. It also
// matters because the operator sends the same command repeatedly - ten random
// angles, then +180 over and over - and each row of the test data has to be
// tied to the move that produced it.
//
// Angles and speeds arrive as whole millidegrees, never as decimals. The
// laptop turns "45.5" into 45500 before sending, which keeps this parser to
// integers only: no floating point, no locale surprises, nothing that rounds
// differently on two machines.
#pragma once

#include <stdint.h>

enum class Verb {
    None,
    Move,
    Stop,
    Estop,
    Zero,
    Pos,
    Pin,
    Limits,
    Ping,
};

enum class AxisId {
    None,
    Az,
    El,
    All,  // ZERO only
};

// Why a line could not be understood. These are faults in the text itself.
// Whether a command can actually be carried out - speed in range, inside the
// soft limits, axis not already moving - is decided later, not here.
enum class ParseError {
    None,
    Syntax,       // wrong number of fields, or a field that is not a number
    UnknownVerb,  // first field is not a command we know
    BadAxis,      // not AZ, EL, or ALL where ALL is allowed
    BadGpio,      // pin number or level out of range
};

struct Command {
    Verb verb       = Verb::None;
    ParseError err  = ParseError::None;
    uint16_t id     = 0;

    AxisId axis     = AxisId::None;  // MOVE, ZERO
    int32_t mdps    = 0;             // MOVE: commanded speed
    int32_t mdeg    = 0;             // MOVE: how far. ZERO: where we are now
    bool has_mdeg   = false;         // ZERO: was the optional angle given?

    uint8_t gpio    = 0;             // PIN
    uint8_t level   = 0;             // PIN
};

// Read one line. On failure the returned command has err set and verb left as
// None, except for an unknown verb where there is nothing else to report.
//
// Command words and axis names may be upper or lower case, so that someone
// poking at the Pico from a plain serial monitor during bring-up does not
// have to hold shift. Numbers are strict: digits with an optional leading
// sign, nothing else - no spaces, no decimal point, no exponent.
Command parse_command(const char *line);
