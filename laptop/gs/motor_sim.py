r"""A pretend Pico: the motor firmware's behaviour, in Python.

This is the other end of `protocol.py`. That module builds the lines we send
and reads the lines that come back; this one answers them the way the real
Pico does, so the rest of the laptop software can be written and tested with
nothing plugged in.

It is a mirror of three firmware files, and has to be changed whenever they
are:

    motor_config.hpp    steps per degree, limits, speeds
    motion_profile.cpp  the step maths and the speed ramp
    motor_main.cpp      the command loop, and what gets answered when

Nothing here does input or output. Feed it bytes and a time, and it hands
back the lines the Pico would have printed. The TCP server that turns it
into something pyserial can open lives in `fake_motor_pico.py`, and the
tests drive this module directly with a clock they control, so a one-minute
move takes no time at all to test.

Lines come back **without** their newline. The real wire format ends every
line with "\n"; adding it is the sender's job, in one place.

Three things it copies that are easy to miss
--------------------------------------------

**The clock is yours.** Every method takes `now_us`, a microsecond count you
supply. Hand it real time and moves take as long as they really would; hand
it jumps and they finish at once. The simulator never looks at the wall
clock itself.

**Timing is ideal.** A real Pico's loop has jitter, so its pulses land a few
microseconds late. Here every pulse lands exactly on its deadline, so the
fake's `elapsed_us` is a cleaner number than the real one's.

**It refuses what the firmware refuses, including the awkward parts.** A
simulator that is kinder than the machine is worse than none: the dry run
passes and the bench does not. Two commands arriving in one packet are
judged one at a time, a move on an unzeroed axis is turned down, and
poking a step line costs that axis its zero - all because the firmware
does exactly those things. The contract vectors in
`tests/test_contract_vectors.py` hold this file to that, decision by
decision, so the two cannot quietly part company.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import Enum

# ============================================================================
# The numbers, copied from motor_config.hpp
# ============================================================================
# Copied, not computed: the firmware is the original and this is the copy.
# The contract test is what stops the two drifting apart.
#
# The 1600 pulses per motor turn behind these numbers is a DIP switch
# setting on each driver (1 on, 2 off, 3 on, 4 on), checked by eye at the
# bench. Nothing in software can verify it: set the switches to 800 and
# every angle silently comes out double.
# Every angle is at the output - the antenna itself - with the gearbox
# already folded into steps_num/steps_den.


@dataclass(frozen=True)
class AxisSpec:
    """One axis, exactly as the firmware describes it."""

    name: str
    steps_num: int
    steps_den: int
    min_mdeg: int
    max_mdeg: int
    invert_dir: bool
    v_start_mdps: int
    accel_mdps2: int


V_START_MDPS = 5000  # 5 deg/sec, the speed a move starts and ends at

AZ_SPEC = AxisSpec(
    name="AZ",
    steps_num=6400,  # 1600 driver pulses/rev x 160/7 gearing, over 360 deg
    steps_den=63,  # -> 101.5873 steps per output degree
    min_mdeg=0,
    max_mdeg=360000,
    invert_dir=False,
    v_start_mdps=V_START_MDPS,
    accel_mdps2=45000,
)

EL_SPEC = AxisSpec(
    name="EL",
    steps_num=4000,  # 1600 pulses/rev x 100/7 gearing
    steps_den=63,  # -> 63.492 steps per output degree
    min_mdeg=0,  # conservative; the real stop is at -10 deg
    max_mdeg=45000,  # conservative; the real stop is at +90 deg
    invert_dir=False,
    v_start_mdps=V_START_MDPS,
    accel_mdps2=30000,
)

SPEED_MIN_MDPS = 100  # 0.1 deg/sec
SPEED_MAX_MDPS = 90000  # 90 deg/sec

MIN_INTERVAL_US = 50
MAX_INTERVAL_US = 500000

# From config.hpp. The pause before the first pulse of a move, which the
# firmware spends waiting for the driver to notice the direction pin.
DIR_SETUP_US = 100
REVERSE_PAUSE_US = 1000  # REVERSE_PAUSE_MS, only when the direction flips

MAX_LINE_LEN = 95  # LineReader's buffer
MSTAT_PERIOD_US = 5_000_000

# Which pins the PIN command may touch, from motor_config.hpp.
#
# GP12-GP15 are spare: nothing is attached, so setting one high or low
# proves the laptop can reach the Pico and a meter can read it.
#
# GP4-GP7 are the drivers' step and direction lines. Holding one steady is
# the only way to check that wiring with a multimeter, because a step pulse
# is five microseconds long and no meter will ever catch it. Driving a step
# line makes the driver take a step the firmware did not count, so that
# axis gives up its zero afterwards - the same as an emergency stop. A
# direction line moves nothing and keeps its zero.
SPARE_GPIO_MIN = 12
SPARE_GPIO_MAX = 15

MOTOR_GPIO_AZ_PUL = 4
MOTOR_GPIO_AZ_DIR = 5
MOTOR_GPIO_EL_PUL = 6
MOTOR_GPIO_EL_DIR = 7


def gpio_is_spare(gpio: int) -> bool:
    return SPARE_GPIO_MIN <= gpio <= SPARE_GPIO_MAX


def gpio_is_motor_step(gpio: int) -> bool:
    return gpio in (MOTOR_GPIO_AZ_PUL, MOTOR_GPIO_EL_PUL)


def gpio_is_motor_direction(gpio: int) -> bool:
    return gpio in (MOTOR_GPIO_AZ_DIR, MOTOR_GPIO_EL_DIR)


def gpio_may_be_poked(gpio: int) -> bool:
    return (
        gpio_is_spare(gpio)
        or gpio_is_motor_step(gpio)
        or gpio_is_motor_direction(gpio)
    )


def gpio_is_azimuth(gpio: int) -> bool:
    return gpio in (MOTOR_GPIO_AZ_PUL, MOTOR_GPIO_AZ_DIR)

TARGET_NAME = "gs_motor"  # GS_TARGET_NAME, the CMake target that is flashed


# ============================================================================
# 32-bit float arithmetic
# ============================================================================
# The firmware's speed maths runs in C++ `float`, which carries about seven
# digits. Python's floats carry fifteen. Every arithmetic step below is
# rounded back to 32 bits, in the order the C++ does it, so the two cannot
# disagree.
#
# Honestly: no case has been found where it matters. 6000 planned moves were
# compared with the rounding on and off - every speed and angle the firmware
# allows, on both axes, at today's accelerations and at the higher ones 7.10
# raises them to - and est_us, the ramp length and the step intervals came
# out identical every time. interval_us truncates to whole microseconds, and
# nothing in range lands close enough to a boundary for seven digits versus
# fifteen to show.
#
# It stays because it is free (a plan is worked out once per move), because
# it mirrors what the firmware actually does rather than what happens to
# agree with it today, and because the cost if some future gearing did land
# on a boundary is a silent one-microsecond disagreement between the two
# ends. It is insurance, not load-bearing - do not point at it as the reason
# the numbers match.


def f32(x: float) -> float:
    """Round a Python float to what a C++ `float` would hold."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def _sqrtf(x: float) -> float:
    return f32(math.sqrt(x))


def _per_step(axis: AxisSpec, milli_per_sec: int) -> float:
    """Millidegrees per second into steps per second, as per_step() does."""
    steps_per_deg = f32(f32(axis.steps_num) / f32(axis.steps_den))
    return f32(f32(f32(milli_per_sec) * steps_per_deg) / f32(1000.0))


def _div_round(a: int, b: int) -> int:
    """Divide and round to nearest, ties away from zero, as div_round() does.

    Python's // floors, so -3 // 2 is -2 where C++ gives -1. Doing the work
    on the magnitude and putting the sign back keeps +X and -X symmetric,
    which is the whole point of the C++ version.
    """
    if a >= 0:
        return (a + b // 2) // b
    return -((-a + b // 2) // b)


# ============================================================================
# Steps and angles
# ============================================================================


def mdeg_to_steps(axis: AxisSpec, mdeg: int) -> int:
    """How many steps that angle is, rounded to the nearest whole step."""
    return _div_round(mdeg * axis.steps_num, axis.steps_den * 1000)


def steps_to_mdeg(axis: AxisSpec, steps: int) -> int:
    """The angle those steps really represent.

    Not the inverse of mdeg_to_steps: a step is a fixed size and most angles
    do not land on one, so 45.000 deg of azimuth comes back as 45.006.
    """
    return _div_round(steps * axis.steps_den * 1000, axis.steps_num)


class PlanResult(Enum):
    OK = "ok"
    BAD_SPEED = "speed"
    OUT_OF_RANGE = "limit"
    NO_MOVEMENT = "zerosteps"


@dataclass
class Move:
    """What a move will do.

    Filled in even when the move is refused, so an error can say where it
    would have ended up.
    """

    steps: int = 0
    from_mdeg: int = 0
    to_mdeg: int = 0
    speed_mdps: int = 0

    n_steps: int = 0
    n_ramp: int = 0
    v_start_sps: float = 0.0
    v_cruise_sps: float = 0.0
    accel_sps2: float = 0.0
    triangular: bool = False
    est_us: int = 0


def interval_us(move: Move, i: int) -> int:
    """How long to wait before step number i, counting from 0."""
    if move.n_steps == 0:
        v = move.v_start_sps
    elif i < move.n_ramp:
        # Speeding up: v^2 = v0^2 + 2*a*distance, distance counted in steps.
        v = _sqrtf(
            f32(
                f32(move.v_start_sps * move.v_start_sps)
                + f32(f32(f32(2.0) * move.accel_sps2) * f32(i))
            )
        )
    elif i + move.n_ramp < move.n_steps:
        v = move.v_cruise_sps
    else:
        # Slowing down: the mirror of the ramp up, so the move is symmetric.
        left = (move.n_steps - 1 - i) if i < move.n_steps else 0
        v = _sqrtf(
            f32(
                f32(move.v_start_sps * move.v_start_sps)
                + f32(f32(f32(2.0) * move.accel_sps2) * f32(left))
            )
        )

    if v < f32(0.001):
        return MAX_INTERVAL_US

    us = f32(f32(1000000.0) / v)

    if us <= f32(MIN_INTERVAL_US):
        return MIN_INTERVAL_US
    if us >= f32(MAX_INTERVAL_US):
        return MAX_INTERVAL_US
    return int(us)  # C++ casts to uint32_t, which truncates


def _plan_speed(axis: AxisSpec, m: Move) -> None:
    """Work out the speed-up / hold / slow-down plan, as plan_speed() does."""
    m.n_steps = abs(m.steps)
    m.accel_sps2 = _per_step(axis, axis.accel_mdps2)

    v_cmd = _per_step(axis, m.speed_mdps)
    v0 = _per_step(axis, axis.v_start_mdps)

    # Asking for less than the normal starting speed just runs slower the
    # whole way; starting faster than asked would be surprising.
    if v_cmd < v0:
        v0 = v_cmd

    m.v_start_sps = v0
    m.v_cruise_sps = v_cmd

    n_ramp = 0
    if v_cmd > v0 and m.accel_sps2 > 0.0:
        d = f32(
            f32(f32(v_cmd * v_cmd) - f32(v0 * v0))
            / f32(f32(2.0) * m.accel_sps2)
        )
        n_ramp = int(math.ceil(d))

    # Speeding up and slowing down need the same distance. Without room for
    # both, split the move down the middle: the triangular case.
    m.triangular = 2 * n_ramp > m.n_steps
    if m.triangular:
        n_ramp = m.n_steps // 2
        m.v_cruise_sps = _sqrtf(
            f32(f32(v0 * v0) + f32(f32(f32(2.0) * m.accel_sps2) * f32(n_ramp)))
        )
    m.n_ramp = n_ramp

    # Added up rather than worked out in closed form, so this cannot
    # disagree with what the step loop will actually do.
    total = 0
    for i in range(m.n_steps):
        total += interval_us(m, i)
    m.est_us = 0xFFFFFFFF if total > 0xFFFFFFFF else total


def plan_move(
    axis: AxisSpec, pos_steps: int, delta_mdeg: int, speed_mdps: int
) -> tuple[PlanResult, Move]:
    """Decide whether a move is allowed, and work out its speed plan."""
    delta_steps = mdeg_to_steps(axis, delta_mdeg)
    target_steps = pos_steps + delta_steps

    m = Move(
        steps=delta_steps,
        from_mdeg=steps_to_mdeg(axis, pos_steps),
        to_mdeg=steps_to_mdeg(axis, target_steps),
        speed_mdps=speed_mdps,
    )

    if speed_mdps < SPEED_MIN_MDPS or speed_mdps > SPEED_MAX_MDPS:
        return PlanResult.BAD_SPEED, m

    # A tiny angle can round to no steps at all. Saying so beats reporting a
    # move that never happened.
    if delta_steps == 0:
        return PlanResult.NO_MOVEMENT, m

    # Compared in steps, not degrees: converting the endpoint back to degrees
    # rounds a second time, and a move landing exactly on 360.000 would read
    # as 360.006 and be refused.
    min_steps = mdeg_to_steps(axis, axis.min_mdeg)
    max_steps = mdeg_to_steps(axis, axis.max_mdeg)
    if target_steps < min_steps or target_steps > max_steps:
        return PlanResult.OUT_OF_RANGE, m

    _plan_speed(axis, m)
    return PlanResult.OK, m


# ============================================================================
# One axis
# ============================================================================


class AxisSim:
    """The Stepper class, without the pins.

    Position is counted in steps because steps are what the machine does.
    It is what the firmware *believes*, not where the gimbal is: the real
    one has no encoder feeding back and neither does this.
    """

    def __init__(self, spec: AxisSpec) -> None:
        self.spec = spec
        self.pos_steps = 0
        self.zeroed = False

        self.moving = False
        self.aborted = False
        self.move_id = 0
        self.plan = Move()

        self._index = 0  # pulses sent so far
        self._dir_positive = True  # matches what init() leaves the pin at
        self._next_step_us = 0
        self._first_pulse_us = 0
        self._last_pulse_us = 0
        self._finish_us = 0  # when the last pulse of this move is due

    # --- what the laptop can ask about ------------------------------------

    @property
    def pos_mdeg(self) -> int:
        return steps_to_mdeg(self.spec, self.pos_steps)

    @property
    def steps_done(self) -> int:
        """Signed, so it reads the same way as the commanded step count."""
        return -self._index if self.plan.steps < 0 else self._index

    @property
    def elapsed_us(self) -> int:
        """First pulse to last pulse, as the firmware measures it.

        Always one interval less than the PLAN's est_us, because est_us adds
        up every gap including the one before the first pulse. The real
        firmware has the same gap, so this is a faithful copy of a real
        quirk rather than an error here.
        """
        return max(0, self._last_pulse_us - self._first_pulse_us)

    def set_pos_steps(self, steps: int) -> None:
        """Declare where the axis is. This is what ZERO does."""
        self.pos_steps = steps
        self.zeroed = True

    # --- running a move ----------------------------------------------------

    def start(self, m: Move, move_id: int, now_us: int) -> None:
        self.plan = m
        self._index = 0
        self.aborted = False
        self.move_id = move_id
        self.moving = m.n_steps > 0

        # set_direction() blocks the firmware for the driver's setup time,
        # and a bit longer when the direction flips, so the first pulse is
        # that much later than the command.
        positive = m.steps >= 0
        delay = DIR_SETUP_US
        if positive != self._dir_positive:
            delay += REVERSE_PAUSE_US
        self._dir_positive = positive

        t0 = now_us + delay
        self._next_step_us = t0
        self._first_pulse_us = t0
        self._last_pulse_us = t0
        self._recompute_finish()

    def _recompute_finish(self) -> None:
        """When the last pulse of the move is due.

        The server uses this to sleep until something actually happens
        instead of waking thousands of times a second to do nothing. Worked
        out by walking the remaining intervals, the same ones the step loop
        will use, so the two cannot disagree.
        """
        if not self.moving:
            self._finish_us = self._next_step_us
            return
        t = self._next_step_us
        for i in range(self._index + 1, self.plan.n_steps):
            t += interval_us(self.plan, i)
        self._finish_us = t

    def finish_us(self) -> int | None:
        """When this axis will next need attention, or None if it is idle."""
        return self._finish_us if self.moving else None

    def tick(self, now_us: int) -> bool:
        """Send every pulse that has come due. True while still moving.

        The firmware sends at most one pulse per pass of its loop, but that
        loop runs thousands of times faster than steps are needed, so the
        honest simulation of it is a pulse landing exactly on each deadline.
        """
        if not self.moving:
            return False

        while self.moving and self._next_step_us <= now_us:
            if self._index == 0:
                self._first_pulse_us = self._next_step_us
            self.pos_steps += 1 if self._dir_positive else -1
            self._last_pulse_us = self._next_step_us
            self._index += 1

            if self._index >= self.plan.n_steps:
                self.moving = False
                return False

            self._next_step_us += interval_us(self.plan, self._index)

        return True

    def request_stop(self) -> None:
        """Slow down and stop, as request_stop() does.

        Done by shortening the move rather than with a separate braking
        mode: interval_us already slows over the last n_ramp steps, so
        ending the move that far ahead produces a proper ramp down from
        whatever speed we are doing now.
        """
        if not self.moving:
            return

        ramp = self.plan.n_ramp
        braking = self._index if self._index < ramp else ramp
        shortened = self._index + braking + 1
        if shortened < self.plan.n_steps:
            self.plan.n_steps = shortened
        self.aborted = True
        self._recompute_finish()

    def estop(self) -> None:
        """Stop at once, and give up the zero.

        Pulses stop mid-move with no ramp down. A closed-loop drive settles
        somewhere near, but not exactly where our count says, and there is
        no feedback to find out - so refusing further moves until the
        operator re-zeros is the honest response.
        """
        if not self.moving:
            return
        self.moving = False
        self.aborted = True
        self.zeroed = False


# ============================================================================
# Reading command lines
# ============================================================================
# A mirror of LineReader and parse_command. Both are deliberately strict:
# the point of a simulator is to refuse exactly what the Pico refuses.


class LineResult(Enum):
    NONE = "none"
    LINE = "line"
    TOO_LONG = "toolong"


class LineReader:
    """Collects bytes into one complete command line, as LineReader does."""

    def __init__(self) -> None:
        self._buf: list[str] = []
        self._overflow = False
        self.line = ""

    def reset(self) -> None:
        self._buf.clear()
        self._overflow = False

    @property
    def partial(self) -> bool:
        """Some of a line has arrived but its newline has not.

        The heartbeat is held back while this is true, so a status line
        cannot land in the middle of something being typed.
        """
        return bool(self._buf)

    def feed(self, c: str) -> LineResult:
        # Either newline ends a line, so Windows, Unix and old Mac senders
        # all work. With \r\n the \r finishes the line and the \n then lands
        # on an empty buffer, which is ignored.
        if c in ("\n", "\r"):
            if self._overflow:
                self.reset()
                return LineResult.TOO_LONG
            if not self._buf:
                return LineResult.NONE
            self.line = "".join(self._buf)
            self._buf.clear()
            return LineResult.LINE

        # Already past the limit: swallow the rest in silence. The complaint
        # happens once, when the newline finally arrives.
        if self._overflow:
            return LineResult.NONE

        if len(self._buf) >= MAX_LINE_LEN:
            self._overflow = True
            return LineResult.NONE

        self._buf.append(c)
        return LineResult.NONE


class ParseError(Enum):
    NONE = "none"
    SYNTAX = "syntax"
    UNKNOWN_VERB = "verb"
    BAD_AXIS = "axis"
    BAD_GPIO = "gpio"


@dataclass
class Command:
    verb: str = ""
    err: ParseError = ParseError.NONE
    id: int = 0
    axis: str = ""
    mdps: int = 0
    mdeg: int = 0
    has_mdeg: bool = False
    gpio: int = 0
    level: int = 0


# verb -> how many comma-separated fields it must have. ZERO also accepts 4.
_VERB_FIELDS = {
    "MOVE": 5,
    "STOP": 2,
    "ESTOP": 2,
    "ZERO": 3,
    "POS": 2,
    "PIN": 4,
    "LIMITS": 2,
    "PING": 2,
}

MAX_FIELDS = 6  # one more than any command needs, so extras are noticed


def _parse_int(text: str, lo: int, hi: int) -> int | None:
    """A whole number, as strictly as the firmware reads one.

    An optional sign, then at least one digit, then the end. No spaces, no
    decimal point, no underscores - "45.5" from a confused sender should be
    refused, not quietly become 45.
    """
    if not text:
        return None
    body = text[1:] if text[0] in ("+", "-") else text
    if not body or not body.isascii() or not body.isdigit():
        return None

    # The firmware adds up the digits as it goes and gives up once the
    # running total passes four billion, before it could wrap round into a
    # plausible small number. Mirrored digit by digit rather than by
    # counting characters: leading zeros make a long string whose value is
    # still small, and the firmware accepts those - "0000000000000001" is
    # one, not a syntax error. The contract test pins this down; counting
    # characters is what it caught.
    value = 0
    for digit in body:
        value = value * 10 + (ord(digit) - ord("0"))
        if value > 4000000000:
            return None

    if text[0] == "-":
        value = -value
    if value < lo or value > hi:
        return None
    return value


def parse_command(line: str) -> Command:
    """One line of text into a command, as parse_command() does."""
    fields = line.split(",")
    if len(fields) > MAX_FIELDS:
        return Command(err=ParseError.SYNTAX)

    # Which command it is comes first, so an unknown word is reported as
    # such rather than as a field-count problem.
    verb = fields[0].upper()
    if verb not in _VERB_FIELDS:
        return Command(err=ParseError.UNKNOWN_VERB)

    zero_with_angle = verb == "ZERO" and len(fields) == 4
    if len(fields) != _VERB_FIELDS[verb] and not zero_with_angle:
        return Command(err=ParseError.SYNTAX)

    cmd_id = _parse_int(fields[1], 0, 65535)
    if cmd_id is None:
        return Command(err=ParseError.SYNTAX)

    # From here on the id is known, so every refusal quotes it back. It is
    # the only thing tying the complaint to the command that earned it.

    c = Command(verb=verb, id=cmd_id)

    if verb == "MOVE":
        axis = fields[2].upper()
        if axis not in ("AZ", "EL"):
            return Command(err=ParseError.BAD_AXIS, id=cmd_id)
        mdps = _parse_int(fields[3], 0, 1000000)
        mdeg = _parse_int(fields[4], -1000000, 1000000)
        if mdps is None or mdeg is None:
            return Command(err=ParseError.SYNTAX, id=cmd_id)
        c.axis, c.mdps, c.mdeg = axis, mdps, mdeg

    elif verb == "ZERO":
        axis = fields[2].upper()
        if axis not in ("AZ", "EL", "ALL"):
            return Command(err=ParseError.BAD_AXIS, id=cmd_id)
        c.axis = axis
        if zero_with_angle:
            mdeg = _parse_int(fields[3], -1000000, 1000000)
            if mdeg is None:
                return Command(err=ParseError.SYNTAX, id=cmd_id)
            c.mdeg, c.has_mdeg = mdeg, True

    elif verb == "PIN":
        gpio = _parse_int(fields[2], 0, 29)
        level = _parse_int(fields[3], 0, 1)
        if gpio is None or level is None:
            return Command(err=ParseError.BAD_GPIO, id=cmd_id)
        c.gpio, c.level = gpio, level

    return c


# ============================================================================
# Building reply lines
# ============================================================================
# A mirror of motor_reply.cpp. The firmware writes each line whole into one
# buffer and sends it with a single write, so two messages can never
# interleave; here a line is just a string, which has the same effect.

MAX_TOKEN = 16


def sanitise_token(text: str) -> str:
    """Make text safe to put in a field, as sanitise_token() does.

    A comma would split the line into an extra field and a newline would end
    the record early, so the laptop would read the rest as a different
    reply. Those and anything unprintable become '?', and the result is cut
    to 16 characters.
    """
    out = []
    for ch in text[:MAX_TOKEN]:
        code = ord(ch)
        out.append("?" if code < 0x20 or code > 0x7E or ch == "," else ch)
    return "".join(out)


class MotorSim:
    """The whole firmware: two axes, a command loop and a heartbeat.

    `version` is what PONG and the banner report. It defaults to "fake" on
    purpose - every line this produces is byte-for-byte what the Pico would
    send, so the one safe way to tell a simulated run from a real one is for
    the simulator to say so when asked.
    """

    def __init__(
        self,
        version: str = "fake",
        target: str = TARGET_NAME,
        boot_us: int = 0,
        heartbeat_us: int = MSTAT_PERIOD_US,
    ) -> None:
        self.az = AxisSim(AZ_SPEC)
        self.el = AxisSim(EL_SPEC)
        self.version = version
        self.target = target
        self.pins: dict[int, int] = {}

        self._boot_us = boot_us
        self._heartbeat_us = heartbeat_us
        self._next_stat_us = boot_us + heartbeat_us
        self._reader = LineReader()
        self._was_moving = False

    # --- what the outside world calls --------------------------------------

    @property
    def moving(self) -> bool:
        return self.az.moving or self.el.moving

    def uptime_ms(self, now_us: int) -> int:
        return max(0, now_us - self._boot_us) // 1000

    def banner(self) -> list[str]:
        """What the firmware prints once USB is connected.

        Not records - these come back from `protocol.parse_reply` as Info
        lines, the same as they would from a real Pico.
        """
        return [
            f"=== gs_pico_link {self.target} {self.version} ===",
            "[motor] pins AZ PUL=GP4 DIR=GP5  EL PUL=GP6 DIR=GP7  "
            "idle HIGH OK",
            "[motor] active low: PUL rests HIGH and is pulled LOW to step",
            f"[motor] ready az={_steps_per_deg(AZ_SPEC)} st/deg "
            f"el={_steps_per_deg(EL_SPEC)} st/deg",
            f"[motor] limits az {_mdeg_text(AZ_SPEC.min_mdeg)}.."
            f"{_mdeg_text(AZ_SPEC.max_mdeg)}  el "
            f"{_mdeg_text(EL_SPEC.min_mdeg)}..{_mdeg_text(EL_SPEC.max_mdeg)}",
            "[motor] NOT ZEROED - send ZERO before any MOVE",
            # Not a firmware line. A simulated run must never be mistaken for
            # a real one in somebody's notes.
            "[motor] SIMULATOR - nothing is plugged in and nothing will turn",
        ]

    def next_due_us(self, now_us: int) -> int:
        """When this wants calling again, so a caller can sleep until then."""
        due = []
        for axis in (self.az, self.el):
            end = axis.finish_us()
            if end is not None:
                due.append(end)
        if not self.moving and not self._reader.partial:
            due.append(self._next_stat_us)
        return min(due) if due else now_us + self._heartbeat_us

    def poll(self, now_us: int) -> list[str]:
        """Let time pass. Returns whatever the Pico would have printed."""
        out: list[str] = []
        self._advance(now_us, out)
        self._heartbeat(now_us, out)
        return out

    def feed(self, data: bytes | str, now_us: int) -> list[str]:
        """Hand over received bytes. Returns the lines that go back.

        The firmware works out whether it is busy once, before reading the
        bytes that have arrived, and every complete line in that batch is
        judged against that one answer. So two commands in a single packet
        are both handled as though neither had run - a MOVE and then another
        MOVE would restart the axis, and the first move's DONE would never
        be sent.

        That is copied deliberately. The real one does it, so a simulator
        that behaved better would let a bug through to the bench.
        """
        if isinstance(data, bytes):
            # latin-1 keeps one byte as one character, which is what the
            # firmware's getchar sees. Rubbish bytes should reach the parser
            # and be refused there, not blow up here.
            text = data.decode("latin-1")
        else:
            text = data

        out: list[str] = []
        self._advance(now_us, out)

        # While a line sits half-typed, the firmware pushes the heartbeat
        # deadline along on every pass of its loop - thousands of times a
        # second. This simulator only runs when it is called, so the push
        # that would have happened at this instant happens here. Without it
        # the heartbeat's timing would depend on how often the caller polls.
        if self._reader.partial:
            self._next_stat_us = now_us + self._heartbeat_us

        for ch in text:
            result = self._reader.feed(ch)
            if result is LineResult.LINE:
                command = parse_command(self._reader.line)
                # Read fresh for every line. The firmware used to work it
                # out once per batch of received bytes, before reading
                # them, so two commands in one packet were both judged
                # against the state from before either had run - a second
                # MOVE restarted the axis and the first move's DONE was
                # never sent. Fixed in the firmware in 7.9b; mirrored here.
                self._handle(command, self.moving, now_us, out)
            elif result is LineResult.TOO_LONG:
                # Id 0: the line was thrown away, so there is no id to quote.
                out.append(_err(0, "toolong", ""))

        self._heartbeat(now_us, out)
        return out

    # --- the loop ----------------------------------------------------------

    def _advance(self, now_us: int, out: list[str]) -> None:
        self.az.tick(now_us)
        self.el.tick(now_us)
        moving = self.moving

        # An axis that has just finished reports once. Checked only when
        # everything has stopped, which is what the firmware does - so two
        # axes moving together both report after the slower one ends.
        if self._was_moving and not moving:
            for axis, name in ((self.az, "AZ"), (self.el, "EL")):
                if not axis.moving and axis.move_id != 0:
                    out.append(
                        _done(
                            axis.move_id,
                            name,
                            not axis.aborted,
                            axis.steps_done,
                            axis.plan.from_mdeg,
                            axis.pos_mdeg,
                            axis.elapsed_us,
                        )
                    )
                    axis.move_id = 0
        self._was_moving = moving

    def _heartbeat(self, now_us: int, out: list[str]) -> None:
        # Quiet while moving, because writing to USB blocks when the host
        # stops reading and a block mid-move would stall the step loop. Also
        # quiet mid-way through someone typing, which only scrambles the
        # screen but makes a typed command hard to check before sending.
        #
        # Either way the deadline is pushed along rather than left in the
        # past, so the heartbeat resumes a full period after things go quiet.
        if self.moving or self._reader.partial:
            self._next_stat_us = now_us + self._heartbeat_us
        elif now_us >= self._next_stat_us:
            self._next_stat_us = now_us + self._heartbeat_us
            out.append(
                _mstat(
                    self.uptime_ms(now_us) // 1000,
                    self.az.pos_mdeg,
                    self.el.pos_mdeg,
                    False,
                )
            )

    def _axis_for(self, name: str) -> AxisSim | None:
        if name == "AZ":
            return self.az
        if name == "EL":
            return self.el
        return None

    def _handle(
        self, c: Command, busy: bool, now_us: int, out: list[str]
    ) -> None:
        if c.err is not ParseError.NONE:
            out.append(_err(c.id, c.err.value, ""))
            return

        # While an axis is turning, only the two stop commands are acted on.
        # They are answered by the DONE that follows rather than by an ACK,
        # so that nothing is printed mid-move.
        if busy:
            if c.verb == "STOP":
                self.az.request_stop()
                self.el.request_stop()
                return
            if c.verb == "ESTOP":
                self.az.estop()
                self.el.estop()
                return
            out.append(_err(c.id, "busy", ""))
            return

        if c.verb == "MOVE":
            self._do_move(c, now_us, out)
        elif c.verb == "ZERO":
            self._do_zero(c, out)
        elif c.verb == "PIN":
            self._do_pin(c, out)
        elif c.verb == "POS":
            out.append(
                _pos(
                    c.id,
                    self.az.pos_mdeg,
                    self.el.pos_mdeg,
                    self.az.pos_steps,
                    self.el.pos_steps,
                    self.az.zeroed and self.el.zeroed,
                )
            )
        elif c.verb == "LIMITS":
            out.append(_lim(c.id))
        elif c.verb == "PING":
            out.append(
                _pong(c.id, self.target, self.version, self.uptime_ms(now_us))
            )
        elif c.verb in ("STOP", "ESTOP"):
            # Nothing is moving, so there is nothing to stop. Still answered,
            # so the laptop is never left waiting.
            out.append(_ack(c.id, c.verb))

    def _do_move(self, c: Command, now_us: int, out: list[str]) -> None:
        axis = self._axis_for(c.axis)
        if axis is None:
            out.append(_err(c.id, "axis", ""))
            return

        # Without a zero the soft limits are measured from an unknown origin,
        # which is no protection at all. This is the main thing standing
        # between a mistyped command and the gimbal's hard stops.
        if not axis.zeroed:
            out.append(_err(c.id, "nozero", c.axis))
            return

        result, m = plan_move(axis.spec, axis.pos_steps, c.mdeg, c.mdps)

        if result is PlanResult.BAD_SPEED:
            out.append(_err(c.id, "speed", c.axis))
            return
        if result is PlanResult.OUT_OF_RANGE:
            # Refused whole, never shortened: a clamped move would put a
            # different angle on the gimbal than the one written down.
            out.append(
                _err_limit(
                    c.id,
                    c.axis,
                    m.to_mdeg,
                    axis.spec.min_mdeg,
                    axis.spec.max_mdeg,
                )
            )
            return
        if result is PlanResult.NO_MOVEMENT:
            out.append(_err(c.id, "zerosteps", c.axis))
            return

        out.append(_ack(c.id, "MOVE"))
        out.append(_plan(c.id, c.axis, m))
        axis.start(m, c.id, now_us)

        # The firmware only reports DONE on the pass where moving goes from
        # true to false, and its loop always runs at least once while a move
        # is in progress. Here the next call might come after the move has
        # already finished, so without this the change would never be seen
        # and the DONE would be lost.
        self._was_moving = True

    def _do_zero(self, c: Command, out: list[str]) -> None:
        mdeg = c.mdeg if c.has_mdeg else 0
        if c.axis == "ALL":
            self.az.set_pos_steps(mdeg_to_steps(AZ_SPEC, mdeg))
            self.el.set_pos_steps(mdeg_to_steps(EL_SPEC, mdeg))
        else:
            axis = self._axis_for(c.axis)
            if axis is None:
                out.append(_err(c.id, "axis", ""))
                return
            axis.set_pos_steps(mdeg_to_steps(axis.spec, mdeg))
        out.append(_ack(c.id, "ZERO"))

    def _do_pin(self, c: Command, out: list[str]) -> None:
        if not gpio_may_be_poked(c.gpio):
            # Sixteen characters is all the firmware keeps of a detail, so
            # the hint has to fit in that or arrive half-eaten.
            out.append(_err(c.id, "gpio", "GP4-7 or GP12-15"))
            return

        self.pins[c.gpio] = c.level

        # Driving a step line makes the driver take a step nobody counted,
        # so that axis no longer knows where it is.
        if gpio_is_motor_step(c.gpio):
            if gpio_is_azimuth(c.gpio):
                self.az.zeroed = False
            else:
                self.el.zeroed = False

        out.append(_pin(c.id, c.gpio, c.level))


# ============================================================================
# The line formats themselves
# ============================================================================
# One function per reply, matching motor_reply.cpp field for field.


def _ack(cmd_id: int, verb: str) -> str:
    return f"ACK,{cmd_id},{verb}"


def _plan(cmd_id: int, axis: str, m: Move) -> str:
    # The shape is reported so an odd row in the test data explains itself:
    # TRI means the move was too short to reach the commanded speed.
    shape = "TRI" if m.triangular else "TRAP"
    return (
        f"PLAN,{cmd_id},{axis},{m.steps},{m.from_mdeg},{m.to_mdeg},"
        f"{m.speed_mdps},{shape},{m.est_us}"
    )


def _done(
    cmd_id: int,
    axis: str,
    ok: bool,
    steps_done: int,
    from_mdeg: int,
    to_mdeg: int,
    elapsed_us: int,
) -> str:
    return (
        f"DONE,{cmd_id},{axis},{'OK' if ok else 'ABORT'},{steps_done},"
        f"{from_mdeg},{to_mdeg},{elapsed_us}"
    )


def _pos(
    cmd_id: int,
    az_mdeg: int,
    el_mdeg: int,
    az_steps: int,
    el_steps: int,
    zeroed: bool,
) -> str:
    return (
        f"POS,{cmd_id},{az_mdeg},{el_mdeg},{az_steps},{el_steps},"
        f"{'ZEROED' if zeroed else 'UNZEROED'}"
    )


def _pin(cmd_id: int, gpio: int, level: int) -> str:
    return f"PIN,{cmd_id},{gpio},{level}"


def _lim(cmd_id: int) -> str:
    return (
        f"LIM,{cmd_id},{AZ_SPEC.min_mdeg},{AZ_SPEC.max_mdeg},"
        f"{EL_SPEC.min_mdeg},{EL_SPEC.max_mdeg},"
        f"{SPEED_MIN_MDPS},{SPEED_MAX_MDPS}"
    )


def _pong(cmd_id: int, target: str, version: str, uptime_ms: int) -> str:
    return (
        f"PONG,{cmd_id},{sanitise_token(target)},"
        f"{sanitise_token(version)},{uptime_ms}"
    )


def _mstat(uptime_s: int, az_mdeg: int, el_mdeg: int, moving: bool) -> str:
    # No id: this one is not a reply to anything, it just arrives.
    return (
        f"MSTAT,{uptime_s},{az_mdeg},{el_mdeg},"
        f"{'MOVING' if moving else 'IDLE'}"
    )


def _err(cmd_id: int, code: str, detail: str) -> str:
    return f"ERR,{cmd_id},{code},{sanitise_token(detail)}"


def _err_limit(
    cmd_id: int, axis: str, target_mdeg: int, min_mdeg: int, max_mdeg: int
) -> str:
    # The limit error carries numbers rather than text, so the laptop can
    # show exactly how far out of range the move was.
    return f"ERR,{cmd_id},limit,{axis},{target_mdeg},{min_mdeg},{max_mdeg}"


# --- the two numbers the banner prints --------------------------------------


def _steps_per_deg(axis: AxisSpec) -> str:
    """Steps per degree to three places, as print_steps_per_deg() does."""
    milli = mdeg_to_steps(axis, 1000000)
    return f"{milli // 1000}.{milli % 1000:03d}"


def _mdeg_text(mdeg: int) -> str:
    """Millidegrees as a signed fixed-point string, as print_mdeg() does.

    Built from the magnitude with the sign put back, which is how the C++
    gets "-0.500" right - there the whole part is 0 and has no sign of its
    own, so it is handled as a special case.
    """
    sign = "-" if mdeg < 0 else ""
    whole, frac = divmod(abs(mdeg), 1000)
    return f"{sign}{whole}.{frac:03d}"
