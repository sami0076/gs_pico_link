// Tests for the rules: which command is allowed, what is refused, and what
// goes back on the wire.
//
// This logic used to live inside motor_main.cpp, where none of it could be
// reached without a Pico. Everything here is the part that decides, with
// the GPIO and USB left behind in motor_main.

#include <string.h>

#include "check.hpp"
#include "motor_commands.hpp"
#include "motor_config.hpp"

namespace {

// A machine with both axes zeroed at the origin and nothing moving.
MachineState ready()
{
    MachineState s;
    s.busy         = false;
    s.az_zeroed    = true;
    s.el_zeroed    = true;
    s.az_pos_steps = 0;
    s.el_pos_steps = 0;
    s.uptime_ms    = 12345;
    s.target       = "gs_motor";
    s.version      = "abc1234";
    return s;
}

Decision decide_line(const char *line, const MachineState &s)
{
    return decide(parse_command(line), s);
}

// Compare a reply with the text it should be, printing both when it is not.
void expect_line(const char *got, size_t len, const char *want,
                 const char *what)
{
    g_checks_run++;
    const bool ok = (len == strlen(want)) && (strcmp(got, want) == 0);
    if (!ok) {
        g_checks_failed++;
        printf("FAIL  %s\n        got  \"%s\"\n        want \"%s\"\n", what,
               got, want);
    }
}

void expect_silence(const Decision &d, const char *what)
{
    g_checks_run++;
    if (d.len1 != 0 || d.len2 != 0) {
        g_checks_failed++;
        printf("FAIL  %s: expected nothing, got \"%s\"\n", what, d.line1);
    }
}

// ---------------------------------------------------------------------------
// Moving
// ---------------------------------------------------------------------------

void test_a_move_is_acknowledged_then_planned()
{
    const Decision d = decide_line("MOVE,7,AZ,5000,45000", ready());

    CHECK(d.act == Act::StartMove);
    CHECK(d.axis == AxisId::Az);
    CHECK_EQ(d.id, 7);
    CHECK_EQ(d.move.steps, 2286);
    expect_line(d.line1, d.len1, "ACK,7,MOVE\n", "move ack");
    expect_line(d.line2, d.len2, "PLAN,7,AZ,2286,0,45006,5000,TRAP,8999982\n",
                "move plan");
}

void test_a_move_before_zeroing_is_refused()
{
    MachineState s = ready();
    s.az_zeroed    = false;

    const Decision d = decide_line("MOVE,4,AZ,5000,10000", s);
    CHECK(d.act == Act::Nothing);
    expect_line(d.line1, d.len1, "ERR,4,nozero,AZ\n", "move with no zero");
}

void test_each_axis_keeps_its_own_zero()
{
    // Elevation not being zeroed must not stop azimuth moving.
    MachineState s = ready();
    s.el_zeroed    = false;

    CHECK(decide_line("MOVE,1,AZ,5000,10000", s).act == Act::StartMove);
    CHECK(decide_line("MOVE,2,EL,5000,10000", s).act == Act::Nothing);
}

void test_a_move_past_the_soft_limit_reports_how_far_past()
{
    const Decision d = decide_line("MOVE,5,EL,5000,60000", ready());
    CHECK(d.act == Act::Nothing);
    expect_line(d.line1, d.len1, "ERR,5,limit,EL,60008,0,45000\n",
                "move past the limit");
}

void test_a_speed_outside_the_range_is_refused()
{
    expect_line(decide_line("MOVE,6,AZ,50,10000", ready()).line1,
                decide_line("MOVE,6,AZ,50,10000", ready()).len1,
                "ERR,6,speed,AZ\n", "too slow");
    expect_line(decide_line("MOVE,7,AZ,95000,10000", ready()).line1,
                decide_line("MOVE,7,AZ,95000,10000", ready()).len1,
                "ERR,7,speed,AZ\n", "too fast");
}

void test_an_angle_too_small_to_make_a_step_is_refused()
{
    const Decision d = decide_line("MOVE,8,AZ,5000,1", ready());
    expect_line(d.line1, d.len1, "ERR,8,zerosteps,AZ\n", "no steps");
}

// ---------------------------------------------------------------------------
// While an axis is turning
// ---------------------------------------------------------------------------

void test_one_axis_moving_blocks_commands_for_both()
{
    MachineState s = ready();
    s.busy         = true;

    const Decision d = decide_line("MOVE,9,EL,5000,1000", s);
    CHECK(d.act == Act::Nothing);
    expect_line(d.line1, d.len1, "ERR,9,busy,\n", "busy");

    expect_line(decide_line("POS,10", s).line1, decide_line("POS,10", s).len1,
                "ERR,10,busy,\n", "pos while busy");
}

void test_a_stop_during_a_move_says_nothing_at_the_time()
{
    // It is answered by the move's own DONE: nothing may be printed
    // between the first pulse and the last.
    MachineState s = ready();
    s.busy         = true;

    const Decision stop = decide_line("STOP,11", s);
    CHECK(stop.act == Act::RequestStop);
    expect_silence(stop, "stop while moving");

    const Decision estop = decide_line("ESTOP,12", s);
    CHECK(estop.act == Act::Estop);
    expect_silence(estop, "estop while moving");
}

void test_a_stop_with_nothing_moving_is_still_answered()
{
    const Decision d = decide_line("STOP,13", ready());
    CHECK(d.act == Act::Nothing);
    expect_line(d.line1, d.len1, "ACK,13,STOP\n", "stop when idle");
}

// ---------------------------------------------------------------------------
// Zeroing
// ---------------------------------------------------------------------------

void test_zeroing_everything()
{
    const Decision d = decide_line("ZERO,14,ALL", ready());
    CHECK(d.act == Act::SetZero);
    CHECK(d.zero_az);
    CHECK(d.zero_el);
    CHECK_EQ(d.az_steps, 0);
    CHECK_EQ(d.el_steps, 0);
    expect_line(d.line1, d.len1, "ACK,14,ZERO\n", "zero all");
}

void test_zeroing_one_axis_at_an_angle()
{
    const Decision d = decide_line("ZERO,15,AZ,90000", ready());
    CHECK(d.zero_az);
    CHECK(!d.zero_el);
    CHECK_EQ(d.az_steps, 4571);  // 90 degrees of azimuth
}

// ---------------------------------------------------------------------------
// Poking a pin with a meter
// ---------------------------------------------------------------------------

void test_a_spare_pin_can_be_set_and_costs_nothing()
{
    const Decision d = decide_line("PIN,16,13,1", ready());
    CHECK(d.act == Act::SetPin);
    CHECK_EQ(d.gpio, 13);
    CHECK_EQ(d.level, 1);
    CHECK(!d.clear_az_zero);
    CHECK(!d.clear_el_zero);
    expect_line(d.line1, d.len1, "PIN,16,13,1\n", "spare pin");
}

void test_holding_a_step_line_gives_up_that_axis_zero()
{
    // Driving PUL low makes the driver take a step this firmware did not
    // count, so afterwards its position is one step from the truth.
    const Decision az = decide_line("PIN,17,4,0", ready());
    CHECK(az.act == Act::SetPin);
    CHECK(az.clear_az_zero);
    CHECK(!az.clear_el_zero);

    const Decision el = decide_line("PIN,18,6,0", ready());
    CHECK(el.clear_el_zero);
    CHECK(!el.clear_az_zero);
}

void test_holding_a_direction_line_keeps_the_zero()
{
    // Nothing moves, and the direction is set again at the start of every
    // move, so this one costs nothing.
    const Decision d = decide_line("PIN,19,5,1", ready());
    CHECK(d.act == Act::SetPin);
    CHECK(!d.clear_az_zero);
    CHECK(!d.clear_el_zero);
}

void test_any_other_pin_is_refused_with_advice()
{
    const Decision d = decide_line("PIN,20,25,1", ready());
    CHECK(d.act == Act::Nothing);
    // The hint has to fit in the sixteen characters sanitise_token keeps,
    // or the operator is told to use "GP4-GP7 or GP12-" and left guessing.
    expect_line(d.line1, d.len1, "ERR,20,gpio,GP4-7 or GP12-15\n",
                "pin out of bounds");
}

// ---------------------------------------------------------------------------
// Asking it things
// ---------------------------------------------------------------------------

void test_position_says_unzeroed_unless_both_axes_are_placed()
{
    MachineState s = ready();
    s.az_pos_steps = 4571;
    expect_line(decide_line("POS,21", s).line1, decide_line("POS,21", s).len1,
                "POS,21,89992,0,4571,0,ZEROED\n", "pos zeroed");

    s.el_zeroed = false;
    expect_line(decide_line("POS,22", s).line1, decide_line("POS,22", s).len1,
                "POS,22,89992,0,4571,0,UNZEROED\n", "pos unzeroed");
}

void test_ping_and_limits()
{
    const Decision pong = decide_line("PING,23", ready());
    expect_line(pong.line1, pong.len1, "PONG,23,gs_motor,abc1234,12345\n",
                "pong");

    const Decision lim = decide_line("LIMITS,24", ready());
    expect_line(lim.line1, lim.len1, "LIM,24,0,360000,0,45000,100,90000\n",
                "limits");
}

// ---------------------------------------------------------------------------
// Lines that could not be read
// ---------------------------------------------------------------------------

void test_a_refusal_quotes_the_id_when_the_id_was_readable()
{
    // The laptop has nothing else to tie a refusal to the command that
    // earned it. Only a line whose id could not be read reports 0.
    const Decision axis = decide_line("MOVE,42,ZZ,5000,1000", ready());
    expect_line(axis.line1, axis.len1, "ERR,42,axis,\n", "bad axis keeps id");

    const Decision gpio = decide_line("PIN,43,99,1", ready());
    expect_line(gpio.line1, gpio.len1, "ERR,43,gpio,\n", "bad gpio keeps id");

    const Decision tail = decide_line("MOVE,44,AZ,5000,45.5", ready());
    expect_line(tail.line1, tail.len1, "ERR,44,syntax,\n",
                "bad number keeps id");

    const Decision zero = decide_line("ZERO,45,BOTH", ready());
    expect_line(zero.line1, zero.len1, "ERR,45,axis,\n", "bad zero axis");
}

void test_a_line_with_no_readable_id_reports_zero()
{
    const Decision verb = decide_line("WOBBLE,1", ready());
    expect_line(verb.line1, verb.len1, "ERR,0,verb,\n", "unknown verb");

    const Decision count = decide_line("MOVE,1,AZ", ready());
    expect_line(count.line1, count.len1, "ERR,0,syntax,\n", "wrong fields");

    const Decision id = decide_line("PING,x", ready());
    expect_line(id.line1, id.len1, "ERR,0,syntax,\n", "unreadable id");
}

}  // namespace

int main()
{
    test_a_move_is_acknowledged_then_planned();
    test_a_move_before_zeroing_is_refused();
    test_each_axis_keeps_its_own_zero();
    test_a_move_past_the_soft_limit_reports_how_far_past();
    test_a_speed_outside_the_range_is_refused();
    test_an_angle_too_small_to_make_a_step_is_refused();

    test_one_axis_moving_blocks_commands_for_both();
    test_a_stop_during_a_move_says_nothing_at_the_time();
    test_a_stop_with_nothing_moving_is_still_answered();

    test_zeroing_everything();
    test_zeroing_one_axis_at_an_angle();

    test_a_spare_pin_can_be_set_and_costs_nothing();
    test_holding_a_step_line_gives_up_that_axis_zero();
    test_holding_a_direction_line_keeps_the_zero();
    test_any_other_pin_is_refused_with_advice();

    test_position_says_unzeroed_unless_both_axes_are_placed();
    test_ping_and_limits();

    test_a_refusal_quotes_the_id_when_the_id_was_readable();
    test_a_line_with_no_readable_id_reports_zero();

    return test_summary();
}
