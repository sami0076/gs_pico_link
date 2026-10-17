// Motor test target entry point.
//
// Reads commands from the laptop over USB, moves the two axes, and reports
// what happened. This is the build flashed for the motor actuation test.
//
// The whole program is one loop:
//
//     forever:
//         read any bytes that arrived, act on a complete line
//         let each axis send a pulse if one is due
//         send a heartbeat if it is time and nothing is moving
//
// No interrupts, no second core. The loop runs far more often than steps are
// needed - the fastest step rate used here is one every 219 us - so an axis
// is never waiting on it. Keeping it to a single flat loop means the whole
// program can be read top to bottom.
//
// ONE RULE MATTERS MORE THAN THE REST: nothing is printed between the first
// and last pulse of a move. Writing to USB blocks when the host stops
// reading, and a block in the middle of a move would stall the step loop
// with the motor part way round. So the heartbeat is suppressed while
// moving, DONE is sent after the last pulse, and a command that arrives
// mid-move is refused with a single short line.
//
// Deliberately absent, and staying absent:
//   no radio          the test procedure says to disconnect it
//   no ENA, no ALM    not wired on this board; the Pico cannot disable a
//                     drive and cannot tell when one faults
//   no limit switches none fitted; the soft limits are all there is
//   no encoder        position here is what we believe, not what happened
#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "config.hpp"
#include "line_reader.hpp"
#include "motor_cmd.hpp"
#include "motor_commands.hpp"
#include "motor_reply.hpp"
#include "stepper.hpp"

constexpr uint32_t MSTAT_PERIOD_MS = 5000;

static Stepper g_az(AZ_SPEC, PIN_AZ_PUL, PIN_AZ_DIR);
static Stepper g_el(EL_SPEC, PIN_EL_PUL, PIN_EL_DIR);
static LineReader g_reader;

// One buffer, reused. Every reply is built into it whole and then sent with a
// single fputs, so two messages can never interleave (spec line 168).
static char g_out[MAX_REPLY_LEN];

static void send(size_t n)
{
    if (n > 0) {
        fputs(g_out, stdout);
    }
}

// ----------------------------------------------------------------------------
// Startup
// ----------------------------------------------------------------------------

static void wait_for_usb_forever()
{
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(200);
}

static void print_mdeg(int32_t mdeg)
{
    const long whole = mdeg / 1000;
    long frac        = mdeg % 1000;
    if (frac < 0) {
        frac = -frac;
    }
    if (mdeg < 0 && whole == 0) {
        printf("-0.%03ld", frac);
    } else {
        printf("%ld.%03ld", whole, frac);
    }
}

static void print_steps_per_deg(const AxisSpec &axis)
{
    const int32_t milli = mdeg_to_steps(axis, 1000000);
    printf("%ld.%03ld", static_cast<long>(milli / 1000),
           static_cast<long>(milli % 1000));
}

// Spec 5.3 step 6: never die quietly, and never carry on regardless.
static void fatal_forever(const char *reason)
{
    for (;;) {
        printf("[fatal] %s\n", reason);
        sleep_ms(2000);
    }
}

// ----------------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------------

static Stepper &axis_ref(AxisId a)
{
    return (a == AxisId::El) ? g_el : g_az;
}

// What the machine looks like right now, for decide() to judge.
//
// Taken fresh for every command. The old code worked this out once per
// batch of received bytes, before reading them, so two commands arriving
// in one USB packet were both judged against the state from before either
// had run - a second MOVE would restart the axis and the first move's DONE
// was never sent. Reading it here means a command is always judged against
// the machine as it is when that command is handled.
static MachineState snapshot()
{
    MachineState s;
    s.busy         = g_az.moving() || g_el.moving();
    s.az_zeroed    = g_az.zeroed();
    s.el_zeroed    = g_el.zeroed();
    s.az_pos_steps = g_az.pos_steps();
    s.el_pos_steps = g_el.pos_steps();
    s.uptime_ms    = to_ms_since_boot(get_absolute_time());
    s.target       = GS_TARGET_NAME;
    s.version      = GS_VERSION;
    return s;
}

static void set_test_pin(const Decision &d)
{
    // A spare pin has never been set up, so it needs claiming first.
    //
    // A motor pin has: Stepper::init() already made it an output sitting
    // at its idle level, and gpio_init() would make it an input again for
    // a moment on the way past. The +5V rail sits on the other side of the
    // optocoupler and Pico pins are not 5V tolerant, so letting one float
    // is exactly what config.hpp warns against. Write the level straight
    // to it instead.
    if (gpio_is_spare(d.gpio)) {
        gpio_init(d.gpio);
        gpio_set_dir(d.gpio, GPIO_OUT);
    }
    gpio_put(d.gpio, d.level != 0);

    if (d.clear_az_zero) {
        g_az.clear_zero();
    }
    if (d.clear_el_zero) {
        g_el.clear_zero();
    }
}

// Carry out what decide() asked for. Nothing here chooses anything.
static void apply(const Decision &d)
{
    switch (d.act) {
    case Act::Nothing:
        break;
    case Act::StartMove:
        axis_ref(d.axis).start(d.move, d.id);
        break;
    case Act::RequestStop:
        g_az.request_stop();
        g_el.request_stop();
        break;
    case Act::Estop:
        g_az.estop();
        g_el.estop();
        break;
    case Act::SetZero:
        if (d.zero_az) {
            g_az.set_pos_steps(d.az_steps);
        }
        if (d.zero_el) {
            g_el.set_pos_steps(d.el_steps);
        }
        break;
    case Act::SetPin:
        set_test_pin(d);
        break;
    }
}

static void handle(const Command &c)
{
    const Decision d = decide(c, snapshot());

    // Lines first, then the action. A move's ACK and PLAN have to be on
    // the wire before its first pulse, because nothing may be printed
    // between the first pulse and the last.
    if (d.len1 > 0) {
        fputs(d.line1, stdout);
    }
    if (d.len2 > 0) {
        fputs(d.line2, stdout);
    }
    apply(d);
}

// Read whatever has arrived without waiting for more.
static void poll_input()
{
    for (;;) {
        const int ch = getchar_timeout_us(0);
        if (ch < 0) {
            return;
        }

        const LineReader::Result r = g_reader.feed(static_cast<char>(ch));
        if (r == LineReader::Result::Line) {
            handle(parse_command(g_reader.line()));
        } else if (r == LineReader::Result::TooLong) {
            send(format_err(g_out, sizeof(g_out), 0, ErrCode::TooLong, ""));
        }
    }
}

// Called once each time an axis finishes, to report what it did.
static void report_done(Stepper &s, AxisId axis)
{
    send(format_done(g_out, sizeof(g_out), s.move_id(), axis, !s.aborted(),
                     s.steps_done(), s.plan().from_mdeg, s.pos_mdeg(),
                     s.elapsed_us()));
}

int main()
{
    stdio_init_all();
    wait_for_usb_forever();

    printf("=== gs_pico_link %s %s ===\n", GS_TARGET_NAME, GS_VERSION);

    g_az.init();
    g_el.init();

    if (!g_az.pins_ok()) {
        fatal_forever("azimuth pins did not read back idle HIGH");
    }
    if (!g_el.pins_ok()) {
        fatal_forever("elevation pins did not read back idle HIGH");
    }

    printf("[motor] pins AZ PUL=GP%u DIR=GP%u  EL PUL=GP%u DIR=GP%u  idle HIGH OK\n",
           PIN_AZ_PUL, PIN_AZ_DIR, PIN_EL_PUL, PIN_EL_DIR);
    printf("[motor] active low: PUL rests HIGH and is pulled LOW to step\n");

    printf("[motor] ready az=");
    print_steps_per_deg(AZ_SPEC);
    printf(" st/deg el=");
    print_steps_per_deg(EL_SPEC);
    printf(" st/deg\n");

    printf("[motor] limits az ");
    print_mdeg(AZ_SPEC.min_mdeg);
    printf("..");
    print_mdeg(AZ_SPEC.max_mdeg);
    printf("  el ");
    print_mdeg(EL_SPEC.min_mdeg);
    printf("..");
    print_mdeg(EL_SPEC.max_mdeg);
    printf("\n");

    printf("[motor] NOT ZEROED - send ZERO before any MOVE\n");

    g_reader.reset();

    absolute_time_t next_stat = delayed_by_ms(get_absolute_time(),
                                              MSTAT_PERIOD_MS);
    bool was_moving = false;

    for (;;) {
        const absolute_time_t now = get_absolute_time();

        const bool az_moving = g_az.tick(now);
        const bool el_moving = g_el.tick(now);
        const bool moving    = az_moving || el_moving;

        poll_input();

        // An axis that has just finished reports once. Checked after tick()
        // so the last pulse is already out.
        if (was_moving && !moving) {
            if (!g_az.moving() && g_az.move_id() != 0) {
                report_done(g_az, AxisId::Az);
                g_az.clear_move_id();
            }
            if (!g_el.moving() && g_el.move_id() != 0) {
                report_done(g_el, AxisId::El);
                g_el.clear_move_id();
            }
        }
        was_moving = moving;

        // The heartbeat stays quiet for two reasons.
        //
        // Not while moving - see the rule at the top of this file.
        //
        // And not in the middle of someone typing. A status line landing
        // between two keystrokes does not corrupt the command, which is read
        // on a different path, but it scrambles the screen and makes a hand
        // typed command hard to check before pressing enter.
        //
        // In both cases the deadline is pushed along rather than left in the
        // past, so the heartbeat resumes a full period after things go quiet
        // instead of firing the instant a move ends or a line is finished.
        if (moving || g_reader.partial()) {
            next_stat = delayed_by_ms(now, MSTAT_PERIOD_MS);
        } else if (absolute_time_diff_us(now, next_stat) <= 0) {
            next_stat = delayed_by_ms(now, MSTAT_PERIOD_MS);
            send(format_mstat(g_out, sizeof(g_out),
                              to_ms_since_boot(now) / 1000, g_az.pos_mdeg(),
                              g_el.pos_mdeg(), false));
        }
    }
}
