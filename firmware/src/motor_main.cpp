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

static Stepper *axis_for(AxisId a)
{
    if (a == AxisId::Az) {
        return &g_az;
    }
    if (a == AxisId::El) {
        return &g_el;
    }
    return nullptr;
}

static void do_move(const Command &c)
{
    Stepper *s = axis_for(c.axis);
    if (s == nullptr) {
        send(format_err(g_out, sizeof(g_out), c.id, ErrCode::BadAxis, ""));
        return;
    }

    // Without a zero, the soft limits are measured from an unknown origin,
    // which is no protection at all. This is the main thing standing between
    // a mistyped command and the gimbal's hard stops.
    if (!s->zeroed()) {
        send(format_err(g_out, sizeof(g_out), c.id, ErrCode::NoZero,
                        axis_name(c.axis)));
        return;
    }

    Move m{};
    const PlanResult r =
        plan_move(s->spec(), s->pos_steps(), c.mdeg, c.mdps, &m);

    switch (r) {
    case PlanResult::BadSpeed:
        send(format_err(g_out, sizeof(g_out), c.id, ErrCode::BadSpeed,
                        axis_name(c.axis)));
        return;
    case PlanResult::OutOfRange:
        // Refused whole, never shortened. A clamped move would put a
        // different angle on the gimbal than the one written down.
        send(format_err_limit(g_out, sizeof(g_out), c.id, c.axis, m.to_mdeg,
                              s->spec().min_mdeg, s->spec().max_mdeg));
        return;
    case PlanResult::NoMovement:
        send(format_err(g_out, sizeof(g_out), c.id, ErrCode::ZeroSteps,
                        axis_name(c.axis)));
        return;
    case PlanResult::Ok:
        break;
    }

    send(format_ack(g_out, sizeof(g_out), c.id, Verb::Move));

    // PLAN carries the quantised end angle, which is not simply start plus
    // commanded: 45 degrees of azimuth is 2286 steps, or 45.006 degrees.
    // That number is the test data's theoretical angle.
    send(format_plan(g_out, sizeof(g_out), c.id, c.axis, m));

    s->start(m, c.id);
}

static void do_zero(const Command &c)
{
    const int32_t mdeg = c.has_mdeg ? c.mdeg : 0;

    if (c.axis == AxisId::All) {
        g_az.set_pos_steps(mdeg_to_steps(AZ_SPEC, mdeg));
        g_el.set_pos_steps(mdeg_to_steps(EL_SPEC, mdeg));
    } else {
        Stepper *s = axis_for(c.axis);
        if (s == nullptr) {
            send(format_err(g_out, sizeof(g_out), c.id, ErrCode::BadAxis, ""));
            return;
        }
        s->set_pos_steps(mdeg_to_steps(s->spec(), mdeg));
    }
    send(format_ack(g_out, sizeof(g_out), c.id, Verb::Zero));
}

static void do_pin(const Command &c)
{
    if (!pin_is_safe_for_test(c.gpio)) {
        send(format_err(g_out, sizeof(g_out), c.id, ErrCode::BadGpio,
                        "use GP12-GP15"));
        return;
    }
    gpio_init(c.gpio);
    gpio_set_dir(c.gpio, GPIO_OUT);
    gpio_put(c.gpio, c.level != 0);
    send(format_pin(g_out, sizeof(g_out), c.id, c.gpio, c.level));
}

static void do_pos(const Command &c)
{
    send(format_pos(g_out, sizeof(g_out), c.id, g_az.pos_mdeg(),
                    g_el.pos_mdeg(), g_az.pos_steps(), g_el.pos_steps(),
                    g_az.zeroed() && g_el.zeroed()));
}

static void handle(const Command &c, bool busy)
{
    if (c.err != ParseError::None) {
        ErrCode e = ErrCode::Syntax;
        if (c.err == ParseError::UnknownVerb) {
            e = ErrCode::BadVerb;
        } else if (c.err == ParseError::BadAxis) {
            e = ErrCode::BadAxis;
        } else if (c.err == ParseError::BadGpio) {
            e = ErrCode::BadGpio;
        }
        send(format_err(g_out, sizeof(g_out), c.id, e, ""));
        return;
    }

    // While an axis is turning, only the two stop commands are acted on.
    // They are answered by the DONE that follows rather than by an ACK, so
    // that nothing is printed mid-move.
    if (busy) {
        if (c.verb == Verb::Stop) {
            g_az.request_stop();
            g_el.request_stop();
            return;
        }
        if (c.verb == Verb::Estop) {
            g_az.estop();
            g_el.estop();
            return;
        }
        send(format_err(g_out, sizeof(g_out), c.id, ErrCode::Busy, ""));
        return;
    }

    switch (c.verb) {
    case Verb::Move:
        do_move(c);
        break;
    case Verb::Zero:
        do_zero(c);
        break;
    case Verb::Pin:
        do_pin(c);
        break;
    case Verb::Pos:
        do_pos(c);
        break;
    case Verb::Limits:
        send(format_lim(g_out, sizeof(g_out), c.id));
        break;
    case Verb::Ping:
        send(format_pong(g_out, sizeof(g_out), c.id, GS_TARGET_NAME, GS_VERSION,
                         to_ms_since_boot(get_absolute_time())));
        break;
    case Verb::Stop:
    case Verb::Estop:
        // Nothing is moving, so there is nothing to stop. Still answered, so
        // the laptop is never left waiting.
        send(format_ack(g_out, sizeof(g_out), c.id, c.verb));
        break;
    case Verb::None:
        break;
    }
}

// Read whatever has arrived without waiting for more.
static void poll_input(bool busy)
{
    for (;;) {
        const int ch = getchar_timeout_us(0);
        if (ch < 0) {
            return;
        }

        const LineReader::Result r = g_reader.feed(static_cast<char>(ch));
        if (r == LineReader::Result::Line) {
            handle(parse_command(g_reader.line()), busy);
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

        poll_input(moving);

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
            // Don't let a heartbeat land immediately after the DONE line.
            next_stat = delayed_by_ms(now, MSTAT_PERIOD_MS);
        }
        was_moving = moving;

        // Suppressed while moving - see the rule at the top of this file.
        if (!moving && absolute_time_diff_us(now, next_stat) <= 0) {
            next_stat = delayed_by_ms(now, MSTAT_PERIOD_MS);
            send(format_mstat(g_out, sizeof(g_out),
                              to_ms_since_boot(now) / 1000, g_az.pos_mdeg(),
                              g_el.pos_mdeg(), false));
        }
    }
}
