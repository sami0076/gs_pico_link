// Building the lines the Pico sends back to the laptop.
//
// PURE C++. No Pico headers, so this is covered by firmware/host_tests.
//
// The replies, one per line, each ending in \n:
//
//   ACK,<id>,<verb>
//   PLAN,<id>,<axis>,<steps>,<from>,<to>,<speed>,<TRAP|TRI>,<est_us>
//   DONE,<id>,<axis>,<OK|ABORT>,<steps_done>,<from>,<to>,<elapsed_us>
//   POS,<id>,<az>,<el>,<az_steps>,<el_steps>,<ZEROED|UNZEROED>
//   PIN,<id>,<gpio>,<level>
//   LIM,<id>,<az_min>,<az_max>,<el_min>,<el_max>,<spd_min>,<spd_max>
//   PONG,<id>,<target>,<version>,<uptime_ms>
//   MSTAT,<uptime_s>,<az>,<el>,<IDLE|MOVING>
//   ERR,<id>,<code>,<detail>
//
// All angles and speeds are whole millidegrees, matching the commands going
// the other way. The laptop turns them back into degrees for the operator.
//
// Every function here writes a complete line into the caller's buffer and
// returns its length. Nothing is printed directly. That is deliberate: the
// line has to leave the Pico in a single write, or a status message landing
// mid-line would corrupt it and the laptop would see neither (spec line 168).
//
// On a buffer too small, these return 0 and write nothing usable. A caller
// must treat 0 as "send nothing" - half a line is worse than no line, because
// the laptop would parse the fragment as a different record.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "motion_profile.hpp"
#include "motor_cmd.hpp"

// Comfortably larger than the longest line above, which is PLAN at about 60
// characters.
constexpr size_t MAX_REPLY_LEN = 120;

// Why a command was refused. The parse ones mirror ParseError; the rest are
// decided once the command is understood.
enum class ErrCode {
    Syntax,     // the line did not make sense
    BadVerb,    // no such command
    BadAxis,    // not AZ or EL
    BadGpio,    // pin number or level out of range, or a pin we must not touch
    BadSpeed,   // outside the allowed speed range
    Limit,      // the move would finish outside the soft limits
    NoZero,     // a move was asked for before the axis was zeroed
    Busy,       // that axis is already moving
    ZeroSteps,  // the angle rounds to no steps at all
    TooLong,    // the command line overran the input buffer
};

// Names used on the wire. Exposed so the tests and the laptop agree on them.
const char *verb_name(Verb v);
const char *axis_name(AxisId a);
const char *err_name(ErrCode e);

// Copy text into a reply safely.
//
// Anything the laptop sent back could contain a comma or a newline, which
// would split the line into something that parses as a different record, or
// a byte that is not printable at all. Each such character becomes '?', and
// the result is cut to 16 characters. Returns the length written.
size_t sanitise_token(char *dst, size_t dstn, const char *src);

size_t format_ack(char *buf, size_t n, uint16_t id, Verb verb);

size_t format_plan(char *buf, size_t n, uint16_t id, AxisId axis,
                   const Move &m);

size_t format_done(char *buf, size_t n, uint16_t id, AxisId axis, bool ok,
                   int32_t steps_done, int32_t from_mdeg, int32_t to_mdeg,
                   uint32_t elapsed_us);

size_t format_pos(char *buf, size_t n, uint16_t id, int32_t az_mdeg,
                  int32_t el_mdeg, int32_t az_steps, int32_t el_steps,
                  bool zeroed);

size_t format_pin(char *buf, size_t n, uint16_t id, uint8_t gpio,
                  uint8_t level);

size_t format_lim(char *buf, size_t n, uint16_t id);

size_t format_pong(char *buf, size_t n, uint16_t id, const char *target,
                   const char *version, uint32_t uptime_ms);

size_t format_mstat(char *buf, size_t n, uint32_t uptime_s, int32_t az_mdeg,
                    int32_t el_mdeg, bool moving);

// detail may be null. If given it is sanitised, because it usually quotes
// something the laptop sent.
size_t format_err(char *buf, size_t n, uint16_t id, ErrCode code,
                  const char *detail);

// The limit error carries numbers rather than text, so the laptop can show
// exactly how far out of range the move was.
size_t format_err_limit(char *buf, size_t n, uint16_t id, AxisId axis,
                        int32_t target_mdeg, int32_t min_mdeg,
                        int32_t max_mdeg);
