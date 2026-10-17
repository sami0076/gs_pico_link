r"""The two scripted sequences: `sweep` and `random`.

Both are things the written test procedure asks for and nobody wants to
type forty times at a prompt. They are ordinary callers of
`Console.run_move`, so a scripted move and a typed one go through exactly
the same code, log the same way, and stop the same way on Ctrl-C.

**sweep** turns the same angle at a series of speeds, alternating
direction so it does not walk into a soft limit. It is the run that shows
whether the acceleration ramp is eating the fast rows: at the conservative
45 deg/sec/sec the firmware ships with, a 180 degree azimuth move
commanded at 90 deg/sec spends almost all of its time ramping and averages
nearer 47, which looks like a fault and is not one.

**random** sends the axis to unpredictable angles inside its limits, which
is the part of the procedure that catches an error that only shows up at
particular positions - a tight spot in the gearing, or a step lost on one
direction change.

What they refuse to do
----------------------

**Guess at the limits.** Both ask the Pico what it was flashed with rather
than reading `motor_config.hpp`. A laptop keeping its own copy of the soft
limits is how the two ends come to disagree about where the gimbal may
point.

**Carry on after a refusal.** If any move is turned down, the run stops
there. A refusal means the run's idea of where the antenna is no longer
matches the Pico's, and the moves after it would be measuring something
nobody planned.
"""

from __future__ import annotations

import random as random_module
from dataclasses import dataclass, field

from gs import protocol as p

# The speeds a sweep uses unless told otherwise, in degrees per second.
# Spread wide on purpose: the slow end is where the test procedure lives
# and the fast end is where the ramp starts to dominate.
DEFAULT_SWEEP_SPEEDS = (1.0, 2.0, 5.0, 10.0, 20.0, 45.0, 90.0)

DEFAULT_SWEEP_ANGLE_DEG = 45.0
DEFAULT_RANDOM_COUNT = 10
DEFAULT_RANDOM_SPEED_DPS = 5.0

# Kept clear of each soft limit when choosing a random target. A target
# landing exactly on the limit can round outward by half a step and be
# refused, which would end a run for no useful reason.
LIMIT_MARGIN_MDEG = 50  # 0.05 degrees


@dataclass
class RunResult:
    """What a scripted run managed to do."""

    moves: list = field(default_factory=list)
    stopped_early: bool = False
    reason: str = ""

    @property
    def count(self) -> int:
        return len(self.moves)


def _axis_limits(lim: p.Lim, axis: str) -> tuple[int, int]:
    if axis == "AZ":
        return lim.az_min_mdeg, lim.az_max_mdeg
    return lim.el_min_mdeg, lim.el_max_mdeg


def _axis_position(pos: p.Pos, axis: str) -> int:
    return pos.az_mdeg if axis == "AZ" else pos.el_mdeg


def _take_reading(console, row, result: RunResult) -> bool:
    """Ask for the protractor reading. False if the operator wants out.

    Asked between moves rather than afterwards because the antenna has to
    be still to be read, and the next move would spoil it.
    """
    if row is None:
        return True

    answer = console.ask(
        f"         row {row.row} - protractor reading in degrees "
        f"(enter to skip, q to stop): "
    )
    if answer.lower() in ("q", "quit", "stop"):
        result.stopped_early = True
        result.reason = "stopped by the operator"
        return False
    if answer:
        try:
            entry = console.log.set_true(float(answer), row.row)
        except ValueError:
            console.say(
                f"         {answer!r} is not a number - row left blank"
            )
        else:
            console.report_reading(entry, indent="         ")
    return True


def _after_move(console, outcome, result: RunResult, pause: bool, note: str):
    """Shared tail of both runs. False means stop the run."""
    if outcome is None:
        # run_move already explained it: refused, timed out, or the
        # operator pressed Ctrl-C twice.
        result.stopped_early = True
        result.reason = "a move did not finish"
        return False

    result.moves.append(outcome)

    row = None
    if console.log is not None:
        row = console.log.rows[-1] if console.log.rows else None
        if row is not None and note:
            console.log.annotate(note, row.row)

    if pause and console.log is not None:
        return _take_reading(console, row, result)
    return True


# ============================================================================
# sweep
# ============================================================================


def sweep(
    console,
    axis: str = "AZ",
    angle_deg: float = DEFAULT_SWEEP_ANGLE_DEG,
    speeds=DEFAULT_SWEEP_SPEEDS,
    pause: bool = False,
) -> RunResult:
    """The same angle at each speed in turn, alternating direction."""
    result = RunResult()

    lim = console.session.limits()
    low, high = _axis_limits(lim, axis)

    usable = [s for s in speeds
              if lim.speed_min_mdps <= p.deg_to_mdeg(s) <= lim.speed_max_mdps]
    skipped = [s for s in speeds if s not in usable]
    if skipped:
        console.say(
            f"  skipping {', '.join(str(s) for s in skipped)} deg/s - "
            f"outside what this Pico allows"
        )
    if not usable:
        result.stopped_early = True
        result.reason = "no usable speeds"
        console.say("  nothing to do")
        return result

    console.say(
        f"  sweep: {axis} {angle_deg} deg at "
        f"{', '.join(str(s) for s in usable)} deg/s"
    )

    step_mdeg = p.deg_to_mdeg(angle_deg)
    for speed in usable:
        here = _axis_position(console.session.position(), axis)

        # Go whichever way there is room for, so a sweep started near a
        # limit works its way back rather than stopping on the first move.
        if here + step_mdeg <= high:
            delta = angle_deg
        elif here - step_mdeg >= low:
            delta = -angle_deg
        else:
            result.stopped_early = True
            result.reason = "no room to move either way"
            console.say(
                f"  no room for {angle_deg} deg either way from "
                f"{p.format_mdeg(here)}. Move it clear and start again."
            )
            return result

        outcome = console.run_move(axis, speed, delta, row_hint=not pause)
        if not _after_move(console, outcome, result, pause, f"sweep {speed}"):
            return result

    console.say(f"  sweep done: {result.count} moves")
    return result


# ============================================================================
# random
# ============================================================================


def random_moves(
    console,
    axis: str = "AZ",
    count: int = DEFAULT_RANDOM_COUNT,
    speed_dps: float = DEFAULT_RANDOM_SPEED_DPS,
    seed: int | None = None,
    pause: bool = False,
) -> RunResult:
    """Unpredictable angles inside the axis's limits."""
    result = RunResult()

    lim = console.session.limits()
    low, high = _axis_limits(lim, axis)
    low += LIMIT_MARGIN_MDEG
    high -= LIMIT_MARGIN_MDEG

    if high <= low:
        result.stopped_early = True
        result.reason = "the limits leave no room"
        console.say("  that axis has no room to move")
        return result

    # Seeded on request, so a run that found something odd can be run
    # again exactly. The seed is said out loud either way.
    if seed is None:
        seed = random_module.randrange(1_000_000)
    picker = random_module.Random(seed)

    console.say(
        f"  random: {count} moves on {axis} at {speed_dps} deg/s "
        f"within {p.format_mdeg(low)}..{p.format_mdeg(high)} deg, seed {seed}"
    )

    for _ in range(count):
        here = _axis_position(console.session.position(), axis)

        target = _pick_target(picker, here, low, high)
        if target is None:
            result.stopped_early = True
            result.reason = "could not find anywhere to move to"
            console.say("  could not find a target far enough away")
            return result

        delta_deg = p.mdeg_to_deg(target - here)
        outcome = console.run_move(
            axis, speed_dps, delta_deg, row_hint=not pause
        )
        if not _after_move(console, outcome, result, pause, f"random {seed}"):
            return result

    console.say(f"  random done: {result.count} moves, seed {seed}")
    return result


def _pick_target(picker, here: int, low: int, high: int) -> int | None:
    """An angle inside the limits that is actually somewhere else.

    A target a few thousandths of a degree away rounds to no steps at all
    and the Pico refuses it, which would end the run on nothing. Tried a
    handful of times rather than looped forever, because on a very narrow
    axis there may be nowhere to go.
    """
    smallest = 1000  # one degree is comfortably more than one step
    for _ in range(20):
        target = picker.randint(low, high)
        if abs(target - here) >= smallest:
            return target
    return None
