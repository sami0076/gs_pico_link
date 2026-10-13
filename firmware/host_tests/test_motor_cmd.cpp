// Tests for the command parser.
//
// Half of these are malformed input. A parser that only ever sees correct
// lines is untested - the ones that matter arrive when something upstream is
// confused, and the Pico has to say so rather than act on a guess.
#include "check.hpp"
#include "motor_cmd.hpp"

// --- the commands, spelled correctly ----------------------------------------

static void test_move()
{
    const Command c = parse_command("MOVE,7,AZ,5000,45000");
    CHECK(c.err == ParseError::None);
    CHECK(c.verb == Verb::Move);
    CHECK_EQ(c.id, 7);
    CHECK(c.axis == AxisId::Az);
    CHECK_EQ(c.mdps, 5000);
    CHECK_EQ(c.mdeg, 45000);
}

static void test_move_negative_angle()
{
    const Command c = parse_command("MOVE,8,EL,5000,-45000");
    CHECK(c.err == ParseError::None);
    CHECK(c.axis == AxisId::El);
    CHECK_EQ(c.mdeg, -45000);

    // An explicit + is accepted too.
    const Command p = parse_command("MOVE,9,AZ,5000,+45000");
    CHECK(p.err == ParseError::None);
    CHECK_EQ(p.mdeg, 45000);
}

static void test_the_simple_commands()
{
    const Command s = parse_command("STOP,1");
    CHECK(s.err == ParseError::None);
    CHECK(s.verb == Verb::Stop);
    CHECK_EQ(s.id, 1);

    CHECK(parse_command("ESTOP,2").verb == Verb::Estop);
    CHECK(parse_command("POS,3").verb == Verb::Pos);
    CHECK(parse_command("LIMITS,4").verb == Verb::Limits);
    CHECK(parse_command("PING,5").verb == Verb::Ping);
}

// ZERO takes an optional angle: "I am at 90 degrees right now".
static void test_zero_with_and_without_an_angle()
{
    const Command bare = parse_command("ZERO,10,AZ");
    CHECK(bare.err == ParseError::None);
    CHECK(bare.verb == Verb::Zero);
    CHECK(bare.axis == AxisId::Az);
    CHECK(!bare.has_mdeg);

    const Command at90 = parse_command("ZERO,11,AZ,90000");
    CHECK(at90.err == ParseError::None);
    CHECK(at90.has_mdeg);
    CHECK_EQ(at90.mdeg, 90000);

    // ALL is only meaningful for ZERO.
    const Command all = parse_command("ZERO,12,ALL");
    CHECK(all.err == ParseError::None);
    CHECK(all.axis == AxisId::All);
    CHECK(parse_command("MOVE,13,ALL,5000,1000").err == ParseError::BadAxis);
}

static void test_pin()
{
    const Command c = parse_command("PIN,20,14,1");
    CHECK(c.err == ParseError::None);
    CHECK(c.verb == Verb::Pin);
    CHECK_EQ(c.gpio, 14);
    CHECK_EQ(c.level, 1);

    CHECK_EQ(parse_command("PIN,21,14,0").level, 0);
}

// Typing into a plain serial monitor during bring-up should not need shift.
static void test_case_does_not_matter()
{
    CHECK(parse_command("pos,1").verb == Verb::Pos);
    CHECK(parse_command("Move,1,az,5000,1000").verb == Verb::Move);
    CHECK(parse_command("move,1,El,5000,1000").axis == AxisId::El);
    CHECK(parse_command("zero,1,all").axis == AxisId::All);
}

// --- input that should be refused -------------------------------------------

static void test_unknown_commands()
{
    CHECK(parse_command("WIGGLE,1").err == ParseError::UnknownVerb);
    CHECK(parse_command("").err == ParseError::UnknownVerb);
    CHECK(parse_command("MOV,1,AZ,5000,1000").err == ParseError::UnknownVerb);

    // A near miss must not be accepted as the real thing.
    CHECK(parse_command("MOVES,1,AZ,5000,1000").err == ParseError::UnknownVerb);
    CHECK(parse_command("PIN2,1,14,1").err == ParseError::UnknownVerb);
}

static void test_wrong_number_of_fields()
{
    CHECK(parse_command("MOVE,1,AZ,5000").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,1000,99").err == ParseError::Syntax);
    CHECK(parse_command("MOVE").err == ParseError::Syntax);
    CHECK(parse_command("POS").err == ParseError::Syntax);
    CHECK(parse_command("POS,1,2").err == ParseError::Syntax);
    CHECK(parse_command("ZERO,1").err == ParseError::Syntax);
    CHECK(parse_command("ZERO,1,AZ,0,0").err == ParseError::Syntax);
    CHECK(parse_command("PIN,1,14").err == ParseError::Syntax);
}

static void test_bad_axis()
{
    CHECK(parse_command("MOVE,1,XX,5000,1000").err == ParseError::BadAxis);
    CHECK(parse_command("MOVE,1,,5000,1000").err == ParseError::BadAxis);
    CHECK(parse_command("MOVE,1,AZIMUTH,5000,1000").err == ParseError::BadAxis);
    CHECK(parse_command("MOVE,1,A,5000,1000").err == ParseError::BadAxis);
    CHECK(parse_command("ZERO,1,BOTH").err == ParseError::BadAxis);
}

// Numbers are strict. Anything that is not a plain integer is a mistake, and
// guessing at it would be worse than refusing.
static void test_numbers_that_are_not_numbers()
{
    CHECK(parse_command("MOVE,1,AZ,5000,45.5").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,4e4").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,--45").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,+-45").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,-").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,+").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000, 45").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,45 ").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,0x10").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,abc,1000").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,,AZ,5000,1000").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,x,AZ,5000,1000").err == ParseError::Syntax);
}

// A very long run of digits must be refused, not wrapped into something that
// looks reasonable.
static void test_absurd_numbers()
{
    CHECK(parse_command("MOVE,1,AZ,5000,99999999999999999999")
              .err == ParseError::Syntax);
    CHECK(parse_command("MOVE,99999,AZ,5000,1000").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,2000000").err == ParseError::Syntax);
    CHECK(parse_command("MOVE,1,AZ,5000,-2000000").err == ParseError::Syntax);

    // 65535 is the largest id, and must still be accepted.
    CHECK(parse_command("POS,65535").err == ParseError::None);
    CHECK_EQ(parse_command("POS,65535").id, 65535);
    CHECK(parse_command("POS,65536").err == ParseError::Syntax);
}

static void test_bad_pin()
{
    CHECK(parse_command("PIN,1,30,1").err == ParseError::BadGpio);
    CHECK(parse_command("PIN,1,-1,1").err == ParseError::BadGpio);
    CHECK(parse_command("PIN,1,14,2").err == ParseError::BadGpio);
    CHECK(parse_command("PIN,1,14,-1").err == ParseError::BadGpio);

    // Both ends of the valid range.
    CHECK(parse_command("PIN,1,0,0").err == ParseError::None);
    CHECK(parse_command("PIN,1,29,1").err == ParseError::None);
}

// A refused line must leave the verb unset, so a caller that forgets to check
// err does nothing rather than something wrong.
static void test_a_refused_command_does_nothing()
{
    const char *bad[] = {
        "", "WIGGLE,1", "MOVE,1,XX,5000,1000", "MOVE,1,AZ,5000,45.5",
        "POS", ",,,,", "PIN,1,99,1",
    };
    for (const char *s : bad) {
        const Command c = parse_command(s);
        CHECK(c.err != ParseError::None);
        CHECK(c.verb == Verb::None);
    }
}

// Whatever arrives, the parser must simply return - no crash, no hang.
static void test_odd_input_is_survivable()
{
    CHECK(parse_command(",").err != ParseError::None);
    CHECK(parse_command(",,,,,,,,").err != ParseError::None);
    CHECK(parse_command("\x01\x02\xff").err != ParseError::None);
    CHECK(parse_command("MOVE,1,AZ,5000,1000,,,,,,").err != ParseError::None);

    char long_line[200];
    for (int i = 0; i < 199; i++) {
        long_line[i] = 'A';
    }
    long_line[199] = '\0';
    CHECK(parse_command(long_line).err == ParseError::UnknownVerb);
}

int main()
{
    test_move();
    test_move_negative_angle();
    test_the_simple_commands();
    test_zero_with_and_without_an_angle();
    test_pin();
    test_case_does_not_matter();

    test_unknown_commands();
    test_wrong_number_of_fields();
    test_bad_axis();
    test_numbers_that_are_not_numbers();
    test_absurd_numbers();
    test_bad_pin();
    test_a_refused_command_does_nothing();
    test_odd_input_is_survivable();
    return test_summary();
}
