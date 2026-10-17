"""Tests for the prompt.

These run the real thing: `main()` with `--dry-run`, which wires the
prompt to the simulator with no socket and no pyserial, and reads back
everything it printed. So each test is the same session an operator would
have, just with the clock wound forward and the typing done in advance.

What is checked here is what the operator sees - the words, the numbers
and whether a refusal is explained or merely reported. Whether the move
itself was right is `test_session.py` and `test_motor_sim.py`'s job.
"""

from __future__ import annotations

import io

import pytest

from gs import motor
from gs import protocol as p
from gs import session as sn

# The simulator's clock, wound forward. Moves in these tests are short, so
# this keeps the whole file well under a second.
FAST = ["--fast", "50"]


def run(*commands, extra=None):
    """One dry run with these commands typed into it. Returns what it said."""
    out = io.StringIO()
    argv = ["--dry-run", *FAST, *(extra or [])]
    for text in commands:
        argv += ["-c", text]
    code = motor.main(argv, out=out)
    return code, out.getvalue()


def ready(*commands):
    """...with the axes zeroed first, since nothing moves before that."""
    return run("zero all", *commands)


# ============================================================================
# Starting up
# ============================================================================


def test_a_dry_run_says_so_loudly():
    code, text = run("ping")
    assert code == 0
    assert "SIMULATION - nothing is plugged in" in text
    # And the Pico's own banner comes through underneath it.
    assert "gs_pico_link gs_motor" in text


def test_the_banner_is_shown_once_not_twice():
    _, text = run("ping")
    assert text.count("NOT ZEROED - send ZERO before any MOVE") == 1


def test_ping_points_out_that_it_is_not_a_real_pico():
    _, text = run("ping")
    assert "this is the simulator, not a Pico" in text


def test_fast_without_dry_run_is_refused():
    out = io.StringIO()
    code = motor.main(["--fast", "10"], out=out)
    assert code == 2
    assert "only means anything with --dry-run" in out.getvalue()


# ============================================================================
# Asking it things
# ============================================================================


def test_limits_are_printed_in_degrees_not_millidegrees():
    _, text = run("limits")
    assert "az 0.000..360.000 deg" in text
    assert "speed 0.100..90.000 deg/s" in text


def test_position_says_when_it_has_no_zero_yet():
    _, text = run("pos")
    assert "NOT ZEROED - send zero first" in text


def test_position_shows_steps_as_well_as_degrees():
    _, text = ready("move az 90 10", "pos")
    assert "az 10.001 deg (508 steps)" in text


def test_help_lists_every_command_it_accepts():
    _, text = run("help")
    for word in motor.Console.COMMANDS:
        if word in ("?", "exit"):  # aliases, not listed separately
            continue
        assert word in text, f"help does not mention {word}"


# ============================================================================
# Moving
# ============================================================================


def test_a_move_prints_the_plan_and_then_the_result():
    _, text = ready("move az 5 45")

    assert "+2286 steps" in text
    assert "0.000 -> 45.006 deg" in text  # the quantised end angle
    assert "at 5.000 deg/s" in text
    assert "finished" in text
    assert "now at 45.006 deg" in text


def test_a_move_reports_predicted_and_measured_time_side_by_side():
    # The two numbers next to each other are what tells you the step loop
    # is keeping real time rather than drifting.
    _, text = ready("move az 5 45")
    assert "in 9.00 s  (predicted 9.00 s)" in text


def test_a_negative_angle_turns_back():
    _, text = ready("move az 90 10", "move az 90 -10", "pos")
    assert "az 0.000 deg (0 steps)" in text


def test_a_short_fast_move_is_marked_as_ramped():
    # Too short to reach the speed asked for, which is worth seeing in the
    # data rather than wondering about later.
    _, text = ready("move az 90 2")
    assert "ramped" in text


def test_moving_before_zeroing_is_explained_not_just_refused():
    _, text = run("move az 5 45")
    assert "refused" in text
    assert "send ZERO first" in text


def test_a_move_past_the_limit_says_how_far_past():
    _, text = ready("move el 5 60")
    assert "EL would reach 60.008 deg, allowed 0.000..45.000" in text


def test_an_angle_too_small_to_move_is_refused():
    _, text = ready("move az 5 0.001")
    assert "rounds to no steps" in text


# ============================================================================
# Zeroing
# ============================================================================


def test_zero_with_no_arguments_means_everything_at_nothing():
    _, text = run("zero", "pos")
    assert "ALL is now 0.000 deg. Nothing moved." in text
    assert "zeroed" in text


def test_zero_can_be_told_the_angle_on_the_protractor():
    _, text = run("zero az 90", "pos")
    assert "az 89.992 deg (4571 steps)" in text  # 90 deg is 4571 steps


def test_a_bare_number_means_all_axes():
    # "zero 90" is the obvious thing to type and should not be read as an
    # axis called 90.
    _, text = run("zero 90", "pos")
    # Each axis lands on its own nearest whole step, so the two read back
    # as slightly different angles from the same command.
    assert "az 89.992 deg" in text
    assert "el 89.996 deg" in text


# ============================================================================
# Stopping
# ============================================================================


def test_estop_warns_that_the_zero_is_gone():
    _, text = ready("estop")
    assert "Re-zero before moving again" in text


def test_a_stop_with_nothing_moving_is_harmless():
    code, text = ready("stop")
    assert code == 0
    assert "stop sent" in text


# ============================================================================
# Typing it wrong
# ============================================================================


@pytest.mark.parametrize(
    "typed, expected",
    [
        ("wobble", "no such command: wobble"),
        ("move az 5", "move <az|el> <deg/s> <degrees>"),
        ("move up 5 45", "axis must be az or el"),
        ("move az fast 45", "speed must be a number"),
        ("pin 13", "pin <gpio> <0|1>"),
        ("zero az 1 2 3", "at most an axis and an angle"),
    ],
)
def test_bad_input_gets_a_hint_rather_than_a_traceback(typed, expected):
    code, text = run(typed)
    assert code == 0  # the prompt survives it
    assert expected in text


def test_blank_lines_and_comments_are_ignored():
    code, text = run("", "   ", "# a note to self")
    assert code == 0
    assert "no such command" not in text


def test_a_pin_the_firmware_guards_is_refused_with_advice():
    _, text = run("pin 4 1")
    assert "use GP12-GP15" in text


# ============================================================================
# The heartbeat
# ============================================================================


def test_the_heartbeat_is_hidden_by_default():
    # One line every five seconds would scroll a long session away.
    _, text = run("verbose off")
    assert "heartbeat hidden" in text
    assert "heartbeat  az" not in text


def test_the_heartbeat_can_be_turned_on():
    _, text = run("verbose on")
    assert "heartbeat shown" in text


# ============================================================================
# Reading from somewhere other than a keyboard
# ============================================================================


def test_it_reads_a_piped_script_and_echoes_it():
    out = io.StringIO()
    script = io.StringIO("zero all\nmove az 90 10\npos\n")
    code = motor.main(["--dry-run", *FAST], stdin=script, out=out)
    text = out.getvalue()

    assert code == 0
    # Echoed, so the transcript reads as a session rather than as answers
    # to invisible questions.
    assert "motor> move az 90 10" in text
    assert "508 steps" in text


def test_quit_stops_reading():
    out = io.StringIO()
    script = io.StringIO("quit\nping\n")
    motor.main(["--dry-run", *FAST], stdin=script, out=out)
    assert "up " not in out.getvalue()  # the ping never ran


# ============================================================================
# Ctrl-C
# ============================================================================


class StubSession:
    """A session that can be made to misbehave on command."""

    def __init__(self, interrupts=1):
        self.interrupts = interrupts
        self.stops = 0
        self.plan = p.Plan(
            id=1,
            axis="AZ",
            steps=2286,
            from_mdeg=0,
            to_mdeg=45006,
            speed_mdps=5000,
            triangular=False,
            est_us=9_000_000,
        )

    def begin_move(self, axis, speed, delta):
        return self.plan

    def await_done(self, plan, timeout_s=None):
        if self.interrupts > 0:
            self.interrupts -= 1
            raise KeyboardInterrupt
        return p.Done(
            id=plan.id,
            axis="AZ",
            ok=False,
            steps_done=900,
            from_mdeg=0,
            to_mdeg=17718,
            elapsed_us=3_500_000,
        )

    def stop(self):
        self.stops += 1


def test_ctrl_c_during_a_move_stops_the_motor_and_keeps_waiting():
    # The move is still running on the Pico whatever this end does, so
    # giving up on it would leave the next command reading its answer.
    out = io.StringIO()
    stub = StubSession(interrupts=1)
    console = motor.Console(stub, out=out)

    outcome = console.run_move("AZ", 5.0, 45.0)
    text = out.getvalue()

    assert stub.stops == 1
    assert "stopping" in text
    assert outcome is not None
    assert outcome.done.ok is False
    assert "STOPPED SHORT" in text


def test_ctrl_c_twice_gives_up_but_says_the_position_is_now_a_guess():
    out = io.StringIO()
    stub = StubSession(interrupts=2)
    console = motor.Console(stub, out=out)

    outcome = console.run_move("AZ", 5.0, 45.0)
    text = out.getvalue()

    assert outcome is None
    assert "may still be turning" in text
    assert "estop" in text


def test_a_move_that_times_out_says_what_to_do_about_it():
    out = io.StringIO()

    class Silent(StubSession):
        def await_done(self, plan, timeout_s=None):
            raise sn.Timeout("the Pico did not answer in time")

    console = motor.Console(Silent(), out=out)
    assert console.run_move("AZ", 5.0, 45.0) is None
    assert "may still be running" in out.getvalue()
