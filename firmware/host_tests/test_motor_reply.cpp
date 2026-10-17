// Tests for the reply formatter.
//
// These check exact bytes. The laptop parses these lines by splitting on
// commas, so a stray character or a missing field is a protocol break, not a
// cosmetic problem.
#include <string.h>

#include "check.hpp"
#include "motor_reply.hpp"

// Compare a formatted line against the exact text expected, and check the
// returned length agrees with it.
static void expect(const char *got, size_t len, const char *want,
                   const char *what)
{
    g_checks_run++;
    if (strcmp(got, want) != 0 || len != strlen(want)) {
        g_checks_failed++;
        printf("FAIL  %s\n        got  \"%s\" (len %zu)\n"
               "        want \"%s\" (len %zu)\n",
               what, got, len, want, strlen(want));
    }
}

// --- the happy cases, byte for byte -----------------------------------------

static void test_ack()
{
    char b[MAX_REPLY_LEN];
    expect(b, format_ack(b, sizeof(b), 7, Verb::Move), "ACK,7,MOVE\n", "ack move");
    expect(b, format_ack(b, sizeof(b), 1, Verb::Stop), "ACK,1,STOP\n", "ack stop");
    expect(b, format_ack(b, sizeof(b), 65535, Verb::Ping), "ACK,65535,PING\n",
           "ack ping, largest id");
}

static void test_plan()
{
    char b[MAX_REPLY_LEN];

    // A real 45 degree azimuth move from 90 degrees at 5 deg/sec.
    Move m{};
    const int32_t at_90 = mdeg_to_steps(AZ_SPEC, 90000);
    CHECK(plan_move(AZ_SPEC, at_90, 45000, 5000, &m) == PlanResult::Ok);

    const size_t len = format_plan(b, sizeof(b), 7, AxisId::Az, m);
    CHECK(len > 0);

    // 4571 steps, and no ramp at all at 5 deg/sec.
    //
    // Note the start angle is 90001, not 90000. "I am at 90 degrees" is
    // stored as 9143 steps, and 9143 steps is 90.001 degrees - the zero point
    // is quantised just like everything else. The end is 134997, which is
    // 90001 plus the 44996 that 4571 steps really covers.
    //
    // 4571 steps x 1968 us is 8995728 us, just under the 9 seconds that
    // 45 degrees at 5 deg/sec would ideally take.
    expect(b, len, "PLAN,7,AZ,4571,90001,134997,5000,TRAP,8995728\n",
           "plan 45 deg at 5 deg/sec");
}

static void test_plan_reports_a_triangular_move()
{
    char b[MAX_REPLY_LEN];
    Move m{};
    CHECK(plan_move(AZ_SPEC, 0, 1000, 90000, &m) == PlanResult::Ok);
    CHECK(m.triangular);

    const size_t len = format_plan(b, sizeof(b), 2, AxisId::Az, m);
    CHECK(len > 0);
    CHECK(strstr(b, ",TRI,") != nullptr);
}

static void test_done()
{
    char b[MAX_REPLY_LEN];

    expect(b, format_done(b, sizeof(b), 7, AxisId::Az, true, 4571, 90001,
                          134997, 9001234),
           "DONE,7,AZ,OK,4571,90001,134997,9001234\n", "done ok");

    // An aborted move reports how far it actually got.
    expect(b, format_done(b, sizeof(b), 8, AxisId::El, false, -400, 45000,
                          32400, 2500000),
           "DONE,8,EL,ABORT,-400,45000,32400,2500000\n", "done abort");
}

static void test_pos()
{
    char b[MAX_REPLY_LEN];

    expect(b, format_pos(b, sizeof(b), 3, 90001, 0, 9143, 0, true),
           "POS,3,90001,0,9143,0,ZEROED\n", "pos zeroed");

    expect(b, format_pos(b, sizeof(b), 4, 0, 0, 0, 0, false),
           "POS,4,0,0,0,0,UNZEROED\n", "pos unzeroed");

    // Negative positions must survive the round trip.
    expect(b, format_pos(b, sizeof(b), 5, -44996, -10000, -4571, -635, true),
           "POS,5,-44996,-10000,-4571,-635,ZEROED\n", "pos negative");
}

static void test_pin()
{
    char b[MAX_REPLY_LEN];
    expect(b, format_pin(b, sizeof(b), 20, 14, 1), "PIN,20,14,1\n", "pin high");
    expect(b, format_pin(b, sizeof(b), 21, 14, 0), "PIN,21,14,0\n", "pin low");
}

// The limits come from the config, so the laptop never guesses what this
// build was flashed with. Elevation ships clamped to 0..45.
static void test_lim_reports_the_shipped_limits()
{
    char b[MAX_REPLY_LEN];
    expect(b, format_lim(b, sizeof(b), 4),
           "LIM,4,0,360000,0,45000,100,90000\n", "limits");
}

static void test_pong()
{
    char b[MAX_REPLY_LEN];
    expect(b, format_pong(b, sizeof(b), 5, "gs_motor", "eb4ad64", 12345),
           "PONG,5,gs_motor,eb4ad64,12345\n", "pong");
}

static void test_mstat()
{
    char b[MAX_REPLY_LEN];

    // No id on this one - it is not a reply to anything.
    expect(b, format_mstat(b, sizeof(b), 42, 90001, 0, false),
           "MSTAT,42,90001,0,IDLE\n", "mstat idle");
    expect(b, format_mstat(b, sizeof(b), 43, 91000, 0, true),
           "MSTAT,43,91000,0,MOVING\n", "mstat moving");
}

static void test_err()
{
    char b[MAX_REPLY_LEN];

    expect(b, format_err(b, sizeof(b), 9, ErrCode::NoZero, "AZ"),
           "ERR,9,nozero,AZ\n", "err nozero");
    expect(b, format_err(b, sizeof(b), 10, ErrCode::Busy, "EL"),
           "ERR,10,busy,EL\n", "err busy");
    expect(b, format_err(b, sizeof(b), 0, ErrCode::Syntax, nullptr),
           "ERR,0,syntax,\n", "err with no detail");

    // Every code must render as something, never blank.
    const ErrCode all[] = {
        ErrCode::Syntax,  ErrCode::BadVerb, ErrCode::BadAxis,
        ErrCode::BadGpio, ErrCode::BadSpeed, ErrCode::Limit,
        ErrCode::NoZero,  ErrCode::Busy,    ErrCode::ZeroSteps,
        ErrCode::TooLong,
    };
    for (ErrCode e : all) {
        CHECK(strlen(err_name(e)) > 0);
        CHECK(format_err(b, sizeof(b), 1, e, "x") > 0);
    }
}

// The limit error carries numbers, so the laptop can show how far out the
// move was. This is the example from the bring-up procedure: at 40 degrees
// elevation, asking for another 10 would finish at 50, past the 45 limit.
static void test_err_limit()
{
    char b[MAX_REPLY_LEN];
    expect(b, format_err_limit(b, sizeof(b), 11, AxisId::El, 50000, 0, 45000),
           "ERR,11,limit,EL,50000,0,45000\n", "err limit");
    expect(b, format_err_limit(b, sizeof(b), 12, AxisId::Az, -1000, 0, 360000),
           "ERR,12,limit,AZ,-1000,0,360000\n", "err limit below zero");
}

// --- the awkward cases ------------------------------------------------------

// Text echoed back could contain anything the laptop sent. A comma would add
// a field and a newline would end the record early, and the laptop would then
// read the remainder as a different reply.
static void test_dangerous_characters_cannot_escape()
{
    char b[MAX_REPLY_LEN];

    CHECK(format_err(b, sizeof(b), 1, ErrCode::Syntax, "a,b") > 0);
    expect(b, strlen(b), "ERR,1,syntax,a?b\n", "comma becomes ?");

    CHECK(format_err(b, sizeof(b), 2, ErrCode::Syntax, "a\nb") > 0);
    expect(b, strlen(b), "ERR,2,syntax,a?b\n", "newline becomes ?");

    CHECK(format_err(b, sizeof(b), 3, ErrCode::Syntax, "a\r\nb") > 0);
    expect(b, strlen(b), "ERR,3,syntax,a??b\n", "crlf becomes ??");

    // Split literal: in "a\xffb" the b would be read as part of the hex
    // escape and overflow it.
    CHECK(format_err(b, sizeof(b), 4, ErrCode::Syntax, "a\xff" "b") > 0);
    expect(b, strlen(b), "ERR,4,syntax,a?b\n", "high byte becomes ?");

    CHECK(format_err(b, sizeof(b), 5, ErrCode::Syntax, "a\x01\x02 b") > 0);
    expect(b, strlen(b), "ERR,5,syntax,a?? b\n", "control bytes become ?");

    // Whatever goes in, exactly one newline comes out, at the very end.
    const char *nasty[] = {"a,b", "a\nb", "\n\n\n", "x\r\ny", "\xff\xfe"};
    for (const char *s : nasty) {
        CHECK(format_err(b, sizeof(b), 1, ErrCode::Syntax, s) > 0);
        CHECK_EQ(strchr(b, '\n') - b, static_cast<long>(strlen(b)) - 1);
    }
}

static void test_long_text_is_cut_short()
{
    char b[MAX_REPLY_LEN];
    CHECK(format_err(b, sizeof(b), 1, ErrCode::Syntax,
                     "0123456789abcdefghijklmnop") > 0);
    expect(b, strlen(b), "ERR,1,syntax,0123456789abcdef\n", "detail cut to 16");

    char token[64];
    CHECK_EQ(sanitise_token(token, sizeof(token), "0123456789abcdefghij"), 16);
}

// Every line must end with exactly one newline and nothing after it.
static void test_every_line_ends_with_one_newline()
{
    char b[MAX_REPLY_LEN];
    Move m{};
    plan_move(AZ_SPEC, 0, 45000, 5000, &m);

    size_t lens[9];
    lens[0] = format_ack(b, sizeof(b), 1, Verb::Move);
    CHECK(b[lens[0] - 1] == '\n');
    lens[1] = format_plan(b, sizeof(b), 1, AxisId::Az, m);
    CHECK(b[lens[1] - 1] == '\n');
    lens[2] = format_done(b, sizeof(b), 1, AxisId::Az, true, 1, 0, 0, 0);
    CHECK(b[lens[2] - 1] == '\n');
    lens[3] = format_pos(b, sizeof(b), 1, 0, 0, 0, 0, true);
    CHECK(b[lens[3] - 1] == '\n');
    lens[4] = format_pin(b, sizeof(b), 1, 1, 1);
    CHECK(b[lens[4] - 1] == '\n');
    lens[5] = format_lim(b, sizeof(b), 1);
    CHECK(b[lens[5] - 1] == '\n');
    lens[6] = format_pong(b, sizeof(b), 1, "a", "b", 1);
    CHECK(b[lens[6] - 1] == '\n');
    lens[7] = format_mstat(b, sizeof(b), 1, 0, 0, false);
    CHECK(b[lens[7] - 1] == '\n');
    lens[8] = format_err(b, sizeof(b), 1, ErrCode::Busy, "AZ");
    CHECK(b[lens[8] - 1] == '\n');

    for (size_t len : lens) {
        CHECK(len > 0 && len < MAX_REPLY_LEN);
    }
}

// A buffer too small must produce nothing, not half a line. Half a line would
// be read by the laptop as some other record entirely.
static void test_a_short_buffer_writes_nothing()
{
    // "ACK,65535,LIMITS\n" needs 18 bytes; give it 8.
    char small[8];
    CHECK_EQ(format_ack(small, sizeof(small), 65535, Verb::Limits), 0);
    CHECK_EQ(strlen(small), 0);

    // Same for the long ones.
    Move m{};
    plan_move(AZ_SPEC, 0, 360000, 90000, &m);
    char tiny[12];
    CHECK_EQ(format_plan(tiny, sizeof(tiny), 65535, AxisId::Az, m), 0);
    CHECK_EQ(format_lim(tiny, sizeof(tiny), 65535), 0);
    CHECK_EQ(format_err_limit(tiny, sizeof(tiny), 1, AxisId::El, 50000, 0,
                              45000),
             0);

    // A zero-length buffer must not be written to at all.
    CHECK_EQ(format_ack(nullptr, 0, 1, Verb::Pos), 0);

    // Exactly big enough still works: "ACK,1,POS\n" is 10 bytes plus a NUL.
    char exact[11];
    CHECK_EQ(format_ack(exact, sizeof(exact), 1, Verb::Pos), 10);
    CHECK(strcmp(exact, "ACK,1,POS\n") == 0);

    // One byte short of that produces nothing.
    char one_short[10];
    CHECK_EQ(format_ack(one_short, sizeof(one_short), 1, Verb::Pos), 0);
}

int main()
{
    test_ack();
    test_plan();
    test_plan_reports_a_triangular_move();
    test_done();
    test_pos();
    test_pin();
    test_lim_reports_the_shipped_limits();
    test_pong();
    test_mstat();
    test_err();
    test_err_limit();

    test_dangerous_characters_cannot_escape();
    test_long_text_is_cut_short();
    test_every_line_ends_with_one_newline();
    test_a_short_buffer_writes_nothing();
    return test_summary();
}
