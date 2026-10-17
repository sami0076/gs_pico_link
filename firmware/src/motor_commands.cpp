#include "motor_commands.hpp"

#include "motor_config.hpp"

namespace {

// Each of these fills in one of the Decision's two line slots. Kept as
// little helpers so the rules below read as rules rather than as buffer
// handling.
void say(Decision &d, size_t written)
{
    d.len1 = written;
}

void say_also(Decision &d, size_t written)
{
    d.len2 = written;
}

const AxisSpec &spec_for(AxisId axis)
{
    return (axis == AxisId::El) ? EL_SPEC : AZ_SPEC;
}

bool axis_zeroed(AxisId axis, const MachineState &s)
{
    return (axis == AxisId::El) ? s.el_zeroed : s.az_zeroed;
}

int32_t axis_pos(AxisId axis, const MachineState &s)
{
    return (axis == AxisId::El) ? s.el_pos_steps : s.az_pos_steps;
}

Decision refuse(const Command &c, ErrCode code, const char *detail)
{
    Decision d;
    say(d, format_err(d.line1, sizeof(d.line1), c.id, code, detail));
    return d;
}

// ---------------------------------------------------------------------------

Decision do_move(const Command &c, const MachineState &s)
{
    if (c.axis != AxisId::Az && c.axis != AxisId::El) {
        return refuse(c, ErrCode::BadAxis, "");
    }

    // Without a zero the soft limits are measured from an unknown origin,
    // which is no protection at all. This is the main thing standing
    // between a mistyped command and the gimbal's hard stops.
    if (!axis_zeroed(c.axis, s)) {
        return refuse(c, ErrCode::NoZero, axis_name(c.axis));
    }

    const AxisSpec &spec = spec_for(c.axis);
    Move m{};
    const PlanResult r =
        plan_move(spec, axis_pos(c.axis, s), c.mdeg, c.mdps, &m);

    switch (r) {
    case PlanResult::BadSpeed:
        return refuse(c, ErrCode::BadSpeed, axis_name(c.axis));

    case PlanResult::OutOfRange: {
        // Refused whole, never shortened. A clamped move would put a
        // different angle on the gimbal than the one written down.
        Decision d;
        say(d, format_err_limit(d.line1, sizeof(d.line1), c.id, c.axis,
                                m.to_mdeg, spec.min_mdeg, spec.max_mdeg));
        return d;
    }

    case PlanResult::NoMovement:
        return refuse(c, ErrCode::ZeroSteps, axis_name(c.axis));

    case PlanResult::Ok:
        break;
    }

    Decision d;
    d.act  = Act::StartMove;
    d.axis = c.axis;
    d.move = m;
    d.id   = c.id;

    say(d, format_ack(d.line1, sizeof(d.line1), c.id, Verb::Move));
    // PLAN carries the quantised end angle, which is not simply start plus
    // commanded: 45 degrees of azimuth is 2286 steps, or 45.006 degrees.
    // That number is the test data's theoretical angle.
    say_also(d, format_plan(d.line2, sizeof(d.line2), c.id, c.axis, m));
    return d;
}

Decision do_zero(const Command &c, const MachineState &s)
{
    (void)s;
    const int32_t mdeg = c.has_mdeg ? c.mdeg : 0;

    Decision d;
    d.act = Act::SetZero;

    if (c.axis == AxisId::All) {
        d.zero_az  = true;
        d.zero_el  = true;
        d.az_steps = mdeg_to_steps(AZ_SPEC, mdeg);
        d.el_steps = mdeg_to_steps(EL_SPEC, mdeg);
    } else if (c.axis == AxisId::Az) {
        d.zero_az  = true;
        d.az_steps = mdeg_to_steps(AZ_SPEC, mdeg);
    } else if (c.axis == AxisId::El) {
        d.zero_el  = true;
        d.el_steps = mdeg_to_steps(EL_SPEC, mdeg);
    } else {
        return refuse(c, ErrCode::BadAxis, "");
    }

    say(d, format_ack(d.line1, sizeof(d.line1), c.id, Verb::Zero));
    return d;
}

Decision do_pin(const Command &c, const MachineState &s)
{
    (void)s;
    if (!gpio_may_be_poked(c.gpio)) {
        // Sixteen characters is all sanitise_token keeps, so the
        // hint has to fit in that or arrive half-eaten.
        return refuse(c, ErrCode::BadGpio, "GP4-7 or GP12-15");
    }

    Decision d;
    d.act   = Act::SetPin;
    d.gpio  = c.gpio;
    d.level = c.level;

    // Driving a step line makes the driver take a step this firmware did
    // not count, so afterwards the position it reports is one step away
    // from the truth. Giving up the zero is the honest response, the same
    // as after an emergency stop. A direction line moves nothing, and the
    // direction is set again at the start of every move, so that one
    // costs nothing.
    if (gpio_is_motor_step(c.gpio)) {
        d.clear_az_zero = gpio_is_azimuth(c.gpio);
        d.clear_el_zero = !gpio_is_azimuth(c.gpio);
    }

    say(d, format_pin(d.line1, sizeof(d.line1), c.id, c.gpio, c.level));
    return d;
}

Decision do_pos(const Command &c, const MachineState &s)
{
    Decision d;
    say(d, format_pos(d.line1, sizeof(d.line1), c.id,
                      steps_to_mdeg(AZ_SPEC, s.az_pos_steps),
                      steps_to_mdeg(EL_SPEC, s.el_pos_steps), s.az_pos_steps,
                      s.el_pos_steps, s.az_zeroed && s.el_zeroed));
    return d;
}

}  // namespace

Decision decide(const Command &c, const MachineState &s)
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
        return refuse(c, e, "");
    }

    // While an axis is turning, only the two stop commands are acted on.
    // They are answered by the DONE that follows rather than by an ACK, so
    // that nothing is printed between the first pulse and the last.
    if (s.busy) {
        Decision d;
        if (c.verb == Verb::Stop) {
            d.act = Act::RequestStop;
            return d;
        }
        if (c.verb == Verb::Estop) {
            d.act = Act::Estop;
            return d;
        }
        return refuse(c, ErrCode::Busy, "");
    }

    switch (c.verb) {
    case Verb::Move:
        return do_move(c, s);
    case Verb::Zero:
        return do_zero(c, s);
    case Verb::Pin:
        return do_pin(c, s);
    case Verb::Pos:
        return do_pos(c, s);

    case Verb::Limits: {
        Decision d;
        say(d, format_lim(d.line1, sizeof(d.line1), c.id));
        return d;
    }

    case Verb::Ping: {
        Decision d;
        say(d, format_pong(d.line1, sizeof(d.line1), c.id, s.target,
                           s.version, s.uptime_ms));
        return d;
    }

    case Verb::Stop:
    case Verb::Estop: {
        // Nothing is moving, so there is nothing to stop. Still answered,
        // so the laptop is never left waiting.
        Decision d;
        say(d, format_ack(d.line1, sizeof(d.line1), c.id, c.verb));
        return d;
    }

    case Verb::None:
        break;
    }

    return Decision{};
}
