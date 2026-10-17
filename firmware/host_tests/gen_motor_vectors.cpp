// Writes down exactly what the firmware does, so the laptop can be held to
// it.
//
// The motor protocol is implemented twice: once in C++ on the Pico, and once
// in Python on the laptop - `gs/protocol.py` reads and writes the same lines,
// and `gs/motor_sim.py` reproduces the whole of it so the rest can be built
// with no hardware. Two implementations of one format drift. They drift
// quietly, and the first sign of it is a bench day where the numbers are
// wrong in a way nobody can explain.
//
// So this program runs the real firmware code and writes what came out. The
// pytest in `laptop/tests/test_contract_vectors.py` reads that file and holds
// the Python to it, line by line and number by number. Neither side is
// checked against itself.
//
// The file is checked into the repo at laptop/tests/data/motor_vectors.txt,
// because the C++ is built in WSL and the Python runs on Windows, and a
// generated file that only exists inside one of those is no use to the
// other. ctest regenerates it and compares, so a stale file fails the C++
// build rather than quietly passing the Python one.
//
// Regenerating it is the first half of changing the protocol:
//
//     cmake -S firmware/host_tests -B ~/build/gs-host -G Ninja
//     ninja -C ~/build/gs-host
//     ~/build/gs-host/gen_motor_vectors OUT
//
// where OUT is laptop/tests/data/motor_vectors.txt. (A trailing backslash
// cannot be used to wrap a line here: inside a // comment it continues the
// comment onto the next line, which -Wcomment rightly complains about.)
//
// ONE GAP, AND IT IS WORTH KNOWING. The choice of which error a bad command
// earns - `nozero` before `limit`, `busy` while moving - lives in
// `motor_main.cpp`, which includes Pico headers and cannot be linked here.
// The mapping below mirrors it, the same way the Python simulator does, so a
// change made in motor_main.cpp alone would be caught by neither. Moving
// that decision into a file free of Pico headers would close it.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "motion_profile.hpp"
#include "motor_cmd.hpp"
#include "motor_commands.hpp"
#include "motor_config.hpp"
#include "motor_reply.hpp"

namespace {

// Bumped when the shape of the file changes, so an old file cannot be read
// as though it were a new one.
constexpr int FORMAT_VERSION = 1;

FILE *out = nullptr;
long vectors_written = 0;

// Fields are separated by tabs because every value in this protocol is full
// of commas. `line=` always comes last, so its value can hold anything that
// is not a tab.
void put_line(const char *buf)
{
    fputs("\tline=", out);
    for (const char *p = buf; *p != '\0'; p++) {
        // The trailing newline is the wire framing, not part of the record.
        if (*p != '\n') {
            fputc(*p, out);
        }
    }
    fputc('\n', out);
    vectors_written++;
}

void end_record()
{
    fputc('\n', out);
    vectors_written++;
}

// Some inputs are deliberately horrible - a detail string with a comma or a
// newline in it is exactly what sanitise_token exists for - and writing one
// raw would break the file it is being recorded in. Those fields are written
// backslash-escaped, and the pytest puts them back before using them.
void put_escaped(const char *text)
{
    for (const char *p = text; *p != '\0'; p++) {
        switch (*p) {
        case '\n': fputs("\\n", out); break;
        case '\r': fputs("\\r", out); break;
        case '\t': fputs("\\t", out); break;
        case '\\': fputs("\\\\", out); break;
        default:   fputc(*p, out); break;
        }
    }
}

const char *plan_result_name(PlanResult r)
{
    switch (r) {
    case PlanResult::Ok:         return "ok";
    case PlanResult::BadSpeed:   return "speed";
    case PlanResult::OutOfRange: return "limit";
    case PlanResult::NoMovement: return "zerosteps";
    }
    return "?";
}

// The names the firmware would put on the wire for a line it could not read.
// Mirrored from handle() in motor_main.cpp - see the note at the top.
const char *parse_error_name(ParseError e)
{
    switch (e) {
    case ParseError::None:        return "none";
    case ParseError::Syntax:      return "syntax";
    case ParseError::UnknownVerb: return "verb";
    case ParseError::BadAxis:     return "axis";
    case ParseError::BadGpio:     return "gpio";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Steps and angles
// ---------------------------------------------------------------------------

void conv_vectors(const char *name, const AxisSpec &spec)
{
    static const int32_t angles[] = {
        0,      1,      499,     500,     501,    999,    1000,
        1500,   45000,  90000,   135000,  180000, 359999, 360000,
        -1,     -500,   -1000,   -45000,  -90000, -360000, 12345,
        -12345, 1000000, -1000000,
    };
    for (size_t i = 0; i < sizeof(angles) / sizeof(angles[0]); i++) {
        fprintf(out, "conv\tdir=to_steps\taxis=%s\tmdeg=%ld\tsteps=%ld\n",
                name, static_cast<long>(angles[i]),
                static_cast<long>(mdeg_to_steps(spec, angles[i])));
        vectors_written++;
    }

    static const int32_t steps[] = {
        0,    1,     2,     -1,   -2,    317,   508,   1016,
        2286, -2286, 2857,  4571, 9143,  18286, -18286, 12345,
    };
    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        fprintf(out, "conv\tdir=to_mdeg\taxis=%s\tsteps=%ld\tmdeg=%ld\n",
                name, static_cast<long>(steps[i]),
                static_cast<long>(steps_to_mdeg(spec, steps[i])));
        vectors_written++;
    }
}

// ---------------------------------------------------------------------------
// Planning a move
// ---------------------------------------------------------------------------

void plan_vector(const char *name, const AxisSpec &spec, AxisId axis,
                 int32_t pos_steps, int32_t delta_mdeg, int32_t speed_mdps)
{
    Move m{};
    const PlanResult r =
        plan_move(spec, pos_steps, delta_mdeg, speed_mdps, &m);

    fprintf(out,
            "plan\taxis=%s\tpos_steps=%ld\tdelta_mdeg=%ld\tspeed_mdps=%ld"
            "\tresult=%s\tsteps=%ld\tfrom_mdeg=%ld\tto_mdeg=%ld"
            "\tn_steps=%lu\tn_ramp=%lu\ttriangular=%d\test_us=%lu",
            name, static_cast<long>(pos_steps),
            static_cast<long>(delta_mdeg), static_cast<long>(speed_mdps),
            plan_result_name(r), static_cast<long>(m.steps),
            static_cast<long>(m.from_mdeg), static_cast<long>(m.to_mdeg),
            static_cast<unsigned long>(m.n_steps),
            static_cast<unsigned long>(m.n_ramp), m.triangular ? 1 : 0,
            static_cast<unsigned long>(m.est_us));

    char buf[MAX_REPLY_LEN];
    if (r == PlanResult::Ok) {
        format_plan(buf, sizeof(buf), 7, axis, m);
        put_line(buf);
    } else {
        end_record();
    }
}

void interval_vector(const char *name, const AxisSpec &spec,
                     int32_t delta_mdeg, int32_t speed_mdps, uint32_t i)
{
    Move m{};
    if (plan_move(spec, 0, delta_mdeg, speed_mdps, &m) != PlanResult::Ok) {
        return;
    }
    fprintf(out,
            "interval\taxis=%s\tdelta_mdeg=%ld\tspeed_mdps=%ld\ti=%lu"
            "\tinterval_us=%lu\n",
            name, static_cast<long>(delta_mdeg),
            static_cast<long>(speed_mdps), static_cast<unsigned long>(i),
            static_cast<unsigned long>(interval_us(m, i)));
    vectors_written++;
}

void plan_vectors()
{
    struct Case {
        const char *name;
        const AxisSpec *spec;
        AxisId axis;
        int32_t pos_steps;
        int32_t delta_mdeg;
        int32_t speed_mdps;
    };

    static const Case cases[] = {
        // The test procedure's own moves.
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 45000, 5000},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, -45000, 5000},
        {"AZ", &AZ_SPEC, AxisId::Az, 4571, 45000, 5000},
        {"AZ", &AZ_SPEC, AxisId::Az, 2286, 180000, 5000},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 180000, 90000},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 360000, 90000},
        // Short and fast: too brief to reach the speed asked for.
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 2000, 90000},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 500, 45000},
        // The slowest and fastest the firmware allows.
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 10000, 100},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 10000, 90000},
        // Landing exactly on a soft limit, which must be allowed.
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 360000, 5000},
        // ...and a step past it, which must not be.
        {"AZ", &AZ_SPEC, AxisId::Az, 18286, 1000, 5000},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, -1000, 5000},
        // Refusals.
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 1, 5000},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 10, 5000},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 45000, 50},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 45000, 95000},
        {"AZ", &AZ_SPEC, AxisId::Az, 0, 400000, 5000},
        // Elevation: fewer steps per degree and a tighter travel.
        {"EL", &EL_SPEC, AxisId::El, 0, 45000, 5000},
        {"EL", &EL_SPEC, AxisId::El, 0, 10000, 5000},
        {"EL", &EL_SPEC, AxisId::El, 0, -10000, 5000},
        {"EL", &EL_SPEC, AxisId::El, 1428, 20000, 30000},
        {"EL", &EL_SPEC, AxisId::El, 0, 50000, 5000},
        {"EL", &EL_SPEC, AxisId::El, 0, 1, 5000},
        {"EL", &EL_SPEC, AxisId::El, 0, 45000, 90000},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        const Case &c = cases[i];
        plan_vector(c.name, *c.spec, c.axis, c.pos_steps, c.delta_mdeg,
                    c.speed_mdps);
    }

    // The gap between pulses, at the places where the ramp changes shape:
    // the first step, inside the ramp, the middle, and the last step.
    struct Probe {
        const char *name;
        const AxisSpec *spec;
        int32_t delta_mdeg;
        int32_t speed_mdps;
    };
    static const Probe probes[] = {
        {"AZ", &AZ_SPEC, 45000, 5000},
        {"AZ", &AZ_SPEC, 180000, 90000},
        {"AZ", &AZ_SPEC, 2000, 90000},
        {"AZ", &AZ_SPEC, 10000, 100},
        {"EL", &EL_SPEC, 45000, 45000},
        {"EL", &EL_SPEC, 20000, 5000},
    };
    for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); i++) {
        const Probe &pr = probes[i];
        Move m{};
        if (plan_move(*pr.spec, 0, pr.delta_mdeg, pr.speed_mdps, &m) !=
            PlanResult::Ok) {
            continue;
        }
        const uint32_t points[] = {
            0,
            1,
            m.n_ramp > 1 ? m.n_ramp - 1 : 0,
            m.n_ramp,
            m.n_steps / 2,
            m.n_steps > 1 ? m.n_steps - 2 : 0,
            m.n_steps - 1,
        };
        for (size_t k = 0; k < sizeof(points) / sizeof(points[0]); k++) {
            interval_vector(pr.name, *pr.spec, pr.delta_mdeg, pr.speed_mdps,
                            points[k]);
        }
    }
}

// ---------------------------------------------------------------------------
// The lines the Pico sends
// ---------------------------------------------------------------------------

void reply_vectors()
{
    char b[MAX_REPLY_LEN];

    // ACK
    struct AckCase { uint16_t id; Verb verb; };
    static const AckCase acks[] = {
        {7, Verb::Move},   {1, Verb::Stop},   {2, Verb::Estop},
        {3, Verb::Zero},   {65535, Verb::Ping},
    };
    for (size_t i = 0; i < sizeof(acks) / sizeof(acks[0]); i++) {
        format_ack(b, sizeof(b), acks[i].id, acks[i].verb);
        fprintf(out, "reply\tkind=ack\tid=%u\tverb=%s",
                static_cast<unsigned>(acks[i].id), verb_name(acks[i].verb));
        put_line(b);
    }

    // DONE, both ways it can end.
    struct DoneCase {
        uint16_t id; AxisId axis; bool ok; int32_t steps;
        int32_t from_mdeg; int32_t to_mdeg; uint32_t elapsed_us;
    };
    static const DoneCase dones[] = {
        {7, AxisId::Az, true, 2286, 89992, 134997, 9001234},
        {8, AxisId::El, false, -400, 45000, 32400, 2500000},
        {9, AxisId::Az, true, -2286, 45006, 0, 8996045},
        {0, AxisId::El, true, 0, 0, 0, 0},
    };
    for (size_t i = 0; i < sizeof(dones) / sizeof(dones[0]); i++) {
        const DoneCase &d = dones[i];
        format_done(b, sizeof(b), d.id, d.axis, d.ok, d.steps, d.from_mdeg,
                    d.to_mdeg, d.elapsed_us);
        fprintf(out,
                "reply\tkind=done\tid=%u\taxis=%s\tok=%d\tsteps_done=%ld"
                "\tfrom_mdeg=%ld\tto_mdeg=%ld\telapsed_us=%lu",
                static_cast<unsigned>(d.id), axis_name(d.axis), d.ok ? 1 : 0,
                static_cast<long>(d.steps), static_cast<long>(d.from_mdeg),
                static_cast<long>(d.to_mdeg),
                static_cast<unsigned long>(d.elapsed_us));
        put_line(b);
    }

    // POS
    struct PosCase {
        uint16_t id; int32_t az_mdeg; int32_t el_mdeg;
        int32_t az_steps; int32_t el_steps; bool zeroed;
    };
    static const PosCase positions[] = {
        {3, 89992, 0, 4571, 0, true},
        {4, 0, 0, 0, 0, false},
        {5, -45006, -10000, -2286, -317, true},
        {6, 360000, 45000, 18286, 1428, true},
    };
    for (size_t i = 0; i < sizeof(positions) / sizeof(positions[0]); i++) {
        const PosCase &q = positions[i];
        format_pos(b, sizeof(b), q.id, q.az_mdeg, q.el_mdeg, q.az_steps,
                   q.el_steps, q.zeroed);
        fprintf(out,
                "reply\tkind=pos\tid=%u\taz_mdeg=%ld\tel_mdeg=%ld"
                "\taz_steps=%ld\tel_steps=%ld\tzeroed=%d",
                static_cast<unsigned>(q.id), static_cast<long>(q.az_mdeg),
                static_cast<long>(q.el_mdeg), static_cast<long>(q.az_steps),
                static_cast<long>(q.el_steps), q.zeroed ? 1 : 0);
        put_line(b);
    }

    // PIN
    for (uint8_t gpio = 12; gpio <= 15; gpio++) {
        for (uint8_t level = 0; level <= 1; level++) {
            format_pin(b, sizeof(b), 20, gpio, level);
            fprintf(out, "reply\tkind=pin\tid=20\tgpio=%u\tlevel=%u",
                    static_cast<unsigned>(gpio),
                    static_cast<unsigned>(level));
            put_line(b);
        }
    }

    // LIM - read straight from the build's own configuration.
    format_lim(b, sizeof(b), 4);
    fprintf(out,
            "reply\tkind=lim\tid=4\taz_min_mdeg=%ld\taz_max_mdeg=%ld"
            "\tel_min_mdeg=%ld\tel_max_mdeg=%ld\tspeed_min_mdps=%ld"
            "\tspeed_max_mdps=%ld",
            static_cast<long>(AZ_SPEC.min_mdeg),
            static_cast<long>(AZ_SPEC.max_mdeg),
            static_cast<long>(EL_SPEC.min_mdeg),
            static_cast<long>(EL_SPEC.max_mdeg),
            static_cast<long>(SPEED_MIN_MDPS),
            static_cast<long>(SPEED_MAX_MDPS));
    put_line(b);

    // PONG, including what happens to a version string with a comma in it.
    struct PongCase {
        uint16_t id; const char *target; const char *version; uint32_t up;
    };
    static const PongCase pongs[] = {
        {5, "gs_motor", "eb4ad64", 12345},
        {6, "gs_motor", "eb4ad64-dirty", 0},
        {7, "gs_motor", "a,b", 1},
        {8, "gs_motor", "0123456789abcdefghij", 4294967295u},
    };
    for (size_t i = 0; i < sizeof(pongs) / sizeof(pongs[0]); i++) {
        const PongCase &g = pongs[i];
        format_pong(b, sizeof(b), g.id, g.target, g.version, g.up);
        fprintf(out, "reply\tkind=pong\tid=%u\ttarget=%s\tuptime_ms=%lu"
                     "\tversion_in=",
                static_cast<unsigned>(g.id), g.target,
                static_cast<unsigned long>(g.up));
        put_escaped(g.version);
        put_line(b);
    }

    // MSTAT
    struct StatCase { uint32_t up_s; int32_t az; int32_t el; bool moving; };
    static const StatCase stats[] = {
        {42, 89992, 0, false},
        {43, 91000, 0, true},
        {0, -1000, -500, false},
    };
    for (size_t i = 0; i < sizeof(stats) / sizeof(stats[0]); i++) {
        const StatCase &s = stats[i];
        format_mstat(b, sizeof(b), s.up_s, s.az, s.el, s.moving);
        fprintf(out,
                "reply\tkind=mstat\tuptime_s=%lu\taz_mdeg=%ld\tel_mdeg=%ld"
                "\tmoving=%d",
                static_cast<unsigned long>(s.up_s), static_cast<long>(s.az),
                static_cast<long>(s.el), s.moving ? 1 : 0);
        put_line(b);
    }

    // ERR, every code, with and without a detail.
    static const ErrCode codes[] = {
        ErrCode::Syntax, ErrCode::BadVerb,  ErrCode::BadAxis,
        ErrCode::BadGpio, ErrCode::BadSpeed, ErrCode::Limit,
        ErrCode::NoZero, ErrCode::Busy,     ErrCode::ZeroSteps,
        ErrCode::TooLong,
    };
    for (size_t i = 0; i < sizeof(codes) / sizeof(codes[0]); i++) {
        format_err(b, sizeof(b), 9, codes[i], "AZ");
        fprintf(out, "reply\tkind=err\tid=9\tcode=%s\tdetail_in=AZ",
                err_name(codes[i]));
        put_line(b);

        format_err(b, sizeof(b), 0, codes[i], "");
        fprintf(out, "reply\tkind=err\tid=0\tcode=%s\tdetail_in=",
                err_name(codes[i]));
        put_line(b);
    }

    // A detail that would break the line if it were not cleaned up first.
    static const char *nasty[] = {
        "a,b",
        "line\nbreak",
        "tab\there",
        "use GP12-GP15",
        "0123456789abcdefghijklmno",
    };
    for (size_t i = 0; i < sizeof(nasty) / sizeof(nasty[0]); i++) {
        format_err(b, sizeof(b), 11, ErrCode::BadGpio, nasty[i]);
        fputs("reply\tkind=err\tid=11\tcode=gpio\tdetail_in=", out);
        put_escaped(nasty[i]);
        put_line(b);
    }

    // The limit refusal, which carries numbers instead of text.
    struct LimitCase {
        uint16_t id; AxisId axis; int32_t target; int32_t lo; int32_t hi;
    };
    static const LimitCase limits[] = {
        {12, AxisId::Az, 444997, 0, 360000},
        {13, AxisId::El, 60008, 0, 45000},
        {14, AxisId::El, -10000, 0, 45000},
    };
    for (size_t i = 0; i < sizeof(limits) / sizeof(limits[0]); i++) {
        const LimitCase &l = limits[i];
        format_err_limit(b, sizeof(b), l.id, l.axis, l.target, l.lo, l.hi);
        fprintf(out,
                "reply\tkind=errlimit\tid=%u\taxis=%s\ttarget_mdeg=%ld"
                "\tmin_mdeg=%ld\tmax_mdeg=%ld",
                static_cast<unsigned>(l.id), axis_name(l.axis),
                static_cast<long>(l.target), static_cast<long>(l.lo),
                static_cast<long>(l.hi));
        put_line(b);
    }
}

// ---------------------------------------------------------------------------
// The lines the laptop sends
// ---------------------------------------------------------------------------

void cmd_vector(const char *line)
{
    const Command c = parse_command(line);
    fprintf(out,
            "cmd\terr=%s\tverb=%s\tid=%u\taxis=%s\tmdps=%ld\tmdeg=%ld"
            "\thas_mdeg=%d\tgpio=%u\tlevel=%u\tline=%s\n",
            parse_error_name(c.err), verb_name(c.verb),
            static_cast<unsigned>(c.id), axis_name(c.axis),
            static_cast<long>(c.mdps), static_cast<long>(c.mdeg),
            c.has_mdeg ? 1 : 0, static_cast<unsigned>(c.gpio),
            static_cast<unsigned>(c.level), line);
    vectors_written++;
}

void cmd_vectors()
{
    static const char *lines[] = {
        // Everything protocol.py can build. The pytest checks that the
        // builders still produce these exact strings, which is what makes
        // this a contract and not just a parser test.
        "MOVE,7,AZ,5000,45000",
        "MOVE,1,AZ,5000,-45000",
        "MOVE,2,EL,90000,10000",
        "MOVE,65535,AZ,100,1",
        "MOVE,3,AZ,0,0",
        "STOP,1",
        "ESTOP,2",
        "ZERO,3,ALL",
        "ZERO,4,AZ",
        "ZERO,5,EL",
        "ZERO,6,ALL,90000",
        "ZERO,7,AZ,-10000",
        "POS,8",
        "PIN,9,13,1",
        "PIN,10,15,0",
        "LIMITS,11",
        "PING,12",

        // Case does not matter, so that a serial monitor is usable by hand.
        "move,13,az,5000,45000",
        "Move,14,Az,5000,45000",
        "ping,15",
        "zero,16,all",

        // Lines that must be refused, and with which complaint.
        "",
        ",",
        ",,,",
        "WOBBLE,1",
        "MOVE",
        "MOVE,1",
        "MOVE,1,AZ",
        "MOVE,1,AZ,5000",
        "MOVE,1,AZ,5000,45000,99",
        "MOVE,1,ZZ,5000,45000",
        "MOVE,1,ALL,5000,45000",
        "MOVE,1,AZ,5000,45.5",
        "MOVE,1,AZ,5e3,45000",
        "MOVE,1,AZ, 5000,45000",
        "MOVE,1,AZ,5000, 45000",
        "MOVE,+1,AZ,5000,45000",
        "MOVE,-1,AZ,5000,45000",
        "MOVE,1,AZ,-5000,45000",
        "MOVE,1,AZ,5000,+45000",
        "MOVE,65536,AZ,5000,45000",
        "MOVE,1,AZ,5000,1000001",
        "MOVE,1,AZ,9999999999,45000",

        // Padded with zeros: a long string whose value is small. The
        // firmware adds up digits and only gives up once the total gets
        // large, so these are accepted - counting characters instead
        // would refuse them, which is a difference worth pinning down.
        "MOVE,00000000000000000001,AZ,5000,45000",
        "MOVE,1,AZ,00000000000000005000,45000",
        "PIN,1,00000000000000000013,1",
        "PING,0000000001",
        // ...and the same thing long enough to actually matter.
        "MOVE,1,AZ,5000,99999999999999999999",
        "PING,11111111111111111111111111",
        "MOVE,,AZ,5000,45000",
        "MOVE,x,AZ,5000,45000",
        "PIN,1,30,1",
        "PIN,1,13,2",
        "PIN,1,13",
        "PIN,1,-1,1",
        "ZERO,1",
        "ZERO,1,ALL,90000,5",
        "ZERO,1,ZZ",
        "PING",
        "PING,1,2",
        "POS,1,2",
        " PING,1",
        "PING,1 ",
    };

    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        cmd_vector(lines[i]);
    }
}

// ---------------------------------------------------------------------------
// What the firmware decides to do about a command
// ---------------------------------------------------------------------------
// Until 7.9b these rules lived in motor_main.cpp and could not be reached
// from a host test, so nothing checked that the laptop's simulator refused
// the same things the Pico refuses. They live in motor_commands.cpp now, so
// they can be written down here with everything else.

const char *act_name(Act a)
{
    switch (a) {
    case Act::Nothing:     return "nothing";
    case Act::StartMove:   return "startmove";
    case Act::RequestStop: return "requeststop";
    case Act::Estop:       return "estop";
    case Act::SetZero:     return "setzero";
    case Act::SetPin:      return "setpin";
    }
    return "?";
}

// A line that may hold anything, written escaped and without the trailing
// newline that frames it on the wire.
void put_field(const char *name, const char *text)
{
    fprintf(out, "\t%s=", name);
    char trimmed[MAX_REPLY_LEN];
    size_t n = 0;
    for (const char *p = text; *p != '\0' && n + 1 < sizeof(trimmed); p++) {
        if (*p != '\n') {
            trimmed[n++] = *p;
        }
    }
    trimmed[n] = '\0';
    put_escaped(trimmed);
}

void decide_vector(const MachineState &s, const char *line)
{
    const Decision d = decide(parse_command(line), s);

    fprintf(out,
            "decide\tbusy=%d\taz_zeroed=%d\tel_zeroed=%d\taz_pos_steps=%ld"
            "\tel_pos_steps=%ld\tuptime_ms=%lu\tact=%s\taxis=%s"
            "\tzero_az=%d\tzero_el=%d\taz_steps=%ld\tel_steps=%ld"
            "\tgpio=%u\tlevel=%u\tclear_az_zero=%d\tclear_el_zero=%d",
            s.busy ? 1 : 0, s.az_zeroed ? 1 : 0, s.el_zeroed ? 1 : 0,
            static_cast<long>(s.az_pos_steps),
            static_cast<long>(s.el_pos_steps),
            static_cast<unsigned long>(s.uptime_ms), act_name(d.act),
            axis_name(d.axis), d.zero_az ? 1 : 0, d.zero_el ? 1 : 0,
            static_cast<long>(d.az_steps), static_cast<long>(d.el_steps),
            static_cast<unsigned>(d.gpio), static_cast<unsigned>(d.level),
            d.clear_az_zero ? 1 : 0, d.clear_el_zero ? 1 : 0);

    put_field("line1", d.len1 > 0 ? d.line1 : "");
    put_field("line2", d.len2 > 0 ? d.line2 : "");
    put_field("cmd", line);
    fputc('\n', out);
    vectors_written++;
}

void decide_vectors()
{
    MachineState ready;
    ready.busy         = false;
    ready.az_zeroed    = true;
    ready.el_zeroed    = true;
    ready.az_pos_steps = 0;
    ready.el_pos_steps = 0;
    ready.uptime_ms    = 12345;
    ready.target       = "gs_motor";
    ready.version      = "abc1234";

    MachineState cold = ready;  // nothing zeroed yet, as after a reboot
    cold.az_zeroed    = false;
    cold.el_zeroed    = false;

    MachineState half = ready;  // one axis placed, the other not
    half.el_zeroed    = false;

    MachineState moving = ready;  // an axis is turning
    moving.busy         = true;

    MachineState placed = ready;  // partway round, to catch the limits
    placed.az_pos_steps = 18000;
    placed.el_pos_steps = 1400;

    struct Case { const MachineState *state; const char *line; };
    static const char *commands[] = {
        "MOVE,7,AZ,5000,45000",
        "MOVE,8,EL,5000,10000",
        "MOVE,9,AZ,5000,-45000",
        "MOVE,10,AZ,5000,400000",
        "MOVE,11,AZ,50,1000",
        "MOVE,12,AZ,5000,1",
        "MOVE,13,ZZ,5000,1000",
        "STOP,14",
        "ESTOP,15",
        "ZERO,16,ALL",
        "ZERO,17,AZ,90000",
        "ZERO,18,EL,-10000",
        "POS,19",
        "LIMITS,20",
        "PING,21",
        "PIN,22,13,1",
        "PIN,23,4,0",
        "PIN,24,5,1",
        "PIN,25,6,0",
        "PIN,26,7,1",
        "PIN,27,25,1",
        "PIN,28,99,1",
        "WOBBLE,29",
        "MOVE,30,AZ",
    };

    const MachineState *states[] = {&ready, &cold, &half, &moving, &placed};
    for (size_t si = 0; si < sizeof(states) / sizeof(states[0]); si++) {
        for (size_t ci = 0;
             ci < sizeof(commands) / sizeof(commands[0]); ci++) {
            decide_vector(*states[si], commands[ci]);
        }
    }
}

}  // namespace

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : nullptr;

    if (path != nullptr) {
        out = fopen(path, "wb");  // binary: LF endings on every platform
        if (out == nullptr) {
            fprintf(stderr, "could not write %s\n", path);
            return 2;
        }
    } else {
        out = stdout;
    }

    fputs("# gs_pico_link motor protocol contract vectors\n", out);
    fputs("# Generated by firmware/host_tests/gen_motor_vectors.cpp from\n",
          out);
    fputs("# the firmware's own code. Do not edit by hand: regenerate it,\n",
          out);
    fputs("# and ctest will say if what is checked in has gone stale.\n", out);
    fprintf(out, "version\t%d\n", FORMAT_VERSION);

    conv_vectors("AZ", AZ_SPEC);
    conv_vectors("EL", EL_SPEC);
    plan_vectors();
    reply_vectors();
    cmd_vectors();
    decide_vectors();

    fprintf(out, "count\t%ld\n", vectors_written);

    if (out != stdout) {
        fclose(out);
    }
    return 0;
}
