r"""The laptop's half of the motor protocol.

This is the mirror of two firmware files: `motor_cmd.cpp`, which reads the
lines we send, and `motor_reply.cpp`, which writes the lines we read. The
wire format lives in both places, so this module and those two have to be
changed together. The contract test in a later chunk is what stops them
drifting apart.

The two directions, one line each, "\n" terminated:

    laptop -> Pico                      Pico -> laptop
    MOVE,<id>,<AZ|EL>,<mdps>,<mdeg>     ACK,<id>,<verb>
    STOP,<id>                           PLAN,<id>,<axis>,<steps>,...
    ESTOP,<id>                          DONE,<id>,<axis>,<OK|ABORT>,...
    ZERO,<id>,<AZ|EL|ALL>[,<mdeg>]      POS,<id>,<az>,<el>,...
    POS,<id>                            PIN,<id>,<gpio>,<level>
    PIN,<id>,<gpio>,<0|1>               LIM,<id>,<az_min>,...
    LIMITS,<id>                         PONG,<id>,<target>,...
    PING,<id>                           MSTAT,<uptime_s>,<az>,<el>,...
                                        ERR,<id>,<code>,<detail>

Two rules run through all of it.

**Whole millidegrees on the wire, never decimals.** The Pico's parser takes
integers only: no floating point, no locale surprises, nothing that rounds
differently on two machines. Turning the operator's "45.5" into 45500 is
this module's job, and turning it back for display is too.

**Reading a reply never raises.** A line that does not make sense becomes an
Info record carrying the original text. The Pico prints banners, `[motor]`
notes and `[fatal]` lines on the same wire as the records, and a half-read
line at startup is normal - none of that should take down a program that is
in the middle of driving a motor. Building a command is the opposite: it
raises CommandError rather than send a line the Pico will refuse, because
there the mistake is ours and we would rather hear about it here.
"""

from __future__ import annotations

from dataclasses import dataclass

# ============================================================================
# Limits, copied from the firmware
# ============================================================================
# These mirror the guards in motor_cmd.cpp. They are only a nonsense filter -
# whether a move is actually allowed (speed in range, inside the soft limits,
# axis not already moving) is the Pico's decision, made in plan_move with the
# config it was flashed with. We deliberately do not second-guess that here:
# the laptop keeping its own copy of the soft limits is how the two ends come
# to disagree about where the gimbal may point. Ask with LIMITS instead.
MAX_LINE_LEN = 95  # LineReader's buffer; longer lines come back as ERR toolong

ID_MIN = 1  # 0 is reserved - see IdSource
ID_MAX = 65535

MDPS_MAX = 1000000  # 1000 deg/sec
MDEG_MAX = 1000000  # 1000 degrees
GPIO_MAX = 29

MOVE_AXES = ("AZ", "EL")
ZERO_AXES = ("AZ", "EL", "ALL")

# Every axis word the firmware can put in a reply, including the two that
# only turn up in errors.
AXIS_NAMES = ("AZ", "EL", "ALL", "NONE")

# err_name() in motor_reply.cpp, with what each one means to the operator.
# Kept here so the CLI has one place to look up an explanation.
ERR_MEANING = {
    "syntax": "the Pico could not read that line",
    "verb": "no such command",
    "axis": "axis must be AZ or EL",
    "gpio": "pin number or level out of range, or a pin we must not touch",
    "speed": "speed outside the range the Pico allows",
    "limit": "the move would finish outside the soft limits",
    "nozero": "that axis has not been zeroed yet - send ZERO first",
    "busy": "that axis is already moving",
    "zerosteps": "the angle rounds to no steps at all",
    "toolong": "the command line was too long and was thrown away",
}


class CommandError(ValueError):
    """A command could not be built, with a message meant for the operator."""


# ============================================================================
# Degrees and millidegrees
# ============================================================================


def deg_to_mdeg(deg: float) -> int:
    """Degrees from a human into whole millidegrees for the wire.

    Rounds half away from zero, so 0.0005 becomes 1 and -0.0005 becomes -1.
    Python's built-in round() would send both to 0, and a rule that depends
    on whether the neighbouring digit is even is not one anybody wants to
    reason about while reading a row of test data.
    """
    whole = int(abs(float(deg)) * 1000.0 + 0.5)
    return -whole if deg < 0 else whole


def mdeg_to_deg(mdeg: int) -> float:
    """Millidegrees back to degrees, for display and for the CSV."""
    return mdeg / 1000.0


def format_mdeg(mdeg: int) -> str:
    """Millidegrees as a fixed three-decimal string: 45006 -> "45.006".

    Built from the integer rather than from mdeg/1000.0, so the text is
    exactly the number the Pico used with no float formatting in between.
    Rows of test data get compared against these strings by eye.
    """
    sign = "-" if mdeg < 0 else ""
    whole, frac = divmod(abs(mdeg), 1000)
    return f"{sign}{whole}.{frac:03d}"


# ============================================================================
# Building commands
# ============================================================================


class IdSource:
    """Hands out the id that ties a reply back to the command that caused it.

    A move takes up to a minute and other commands can be sent while it runs,
    so replies do not arrive in the order the commands went out. The id is
    the only thing connecting them, and it is also what ties each row of the
    test data to the move that produced it.

    Ids run 1..65535 and then wrap. Zero is skipped because the firmware uses
    it for the errors it raises on its own - a line too long to have been
    read, which therefore has no id to quote back.
    """

    def __init__(self, start: int = ID_MIN) -> None:
        self._next = start

    def next(self) -> int:
        value = self._next
        self._next = ID_MIN if value >= ID_MAX else value + 1
        return value


def _check_id(cmd_id: int) -> int:
    if not isinstance(cmd_id, int) or isinstance(cmd_id, bool):
        raise CommandError(f"id must be a whole number, got {cmd_id!r}")
    if not ID_MIN <= cmd_id <= ID_MAX:
        raise CommandError(f"id must be {ID_MIN}..{ID_MAX}, got {cmd_id}")
    return cmd_id


def _check_axis(axis: str, allowed: tuple[str, ...]) -> str:
    # Accepted in any case for the operator's sake, but always sent upper
    # case so the line on the wire looks the same however it was typed.
    name = str(axis).strip().upper()
    if name not in allowed:
        raise CommandError(f"axis must be {' or '.join(allowed)}, got {axis!r}")
    return name


def _check_mdeg(mdeg: int, deg: float, what: str) -> int:
    if not -MDEG_MAX <= mdeg <= MDEG_MAX:
        raise CommandError(
            f"{what} must be within +/-{MDEG_MAX // 1000} degrees, got {deg}"
        )
    return mdeg


def move(cmd_id: int, axis: str, speed_dps: float, delta_deg: float) -> str:
    """Turn an axis by delta_deg degrees at speed_dps degrees per second.

    The angle is a change from where the axis is now, not a position to go
    to, which is why it may be negative. The Pico works out the end angle,
    checks it against the soft limits, and reports both in its PLAN line.
    """
    _check_id(cmd_id)
    name = _check_axis(axis, MOVE_AXES)

    mdps = deg_to_mdeg(speed_dps)
    if not 0 <= mdps <= MDPS_MAX:
        raise CommandError(
            f"speed must be 0..{MDPS_MAX // 1000} deg/sec, got {speed_dps}"
        )
    mdeg = _check_mdeg(deg_to_mdeg(delta_deg), delta_deg, "angle")

    return f"MOVE,{cmd_id},{name},{mdps},{mdeg}"


def stop(cmd_id: int) -> str:
    """Bring both axes to a controlled stop."""
    return f"STOP,{_check_id(cmd_id)}"


def estop(cmd_id: int) -> str:
    """Stop both axes at once, without slowing down.

    A stepper told to stop dead can lose steps, so the position the Pico
    reports afterwards may not be where the gimbal really is. Re-zero after
    one of these.
    """
    return f"ESTOP,{_check_id(cmd_id)}"


def zero(cmd_id: int, axis: str = "ALL", deg: float | None = None) -> str:
    """Tell the Pico what angle an axis is sitting at right now.

    Nothing moves. There are no limit switches on this machine, so this is
    the only thing that gives the soft limits an origin, and a MOVE sent
    before it is refused with ERR nozero.
    """
    _check_id(cmd_id)
    name = _check_axis(axis, ZERO_AXES)

    if deg is None:
        return f"ZERO,{cmd_id},{name}"
    mdeg = _check_mdeg(deg_to_mdeg(deg), deg, "angle")
    return f"ZERO,{cmd_id},{name},{mdeg}"


def pos(cmd_id: int) -> str:
    """Ask where both axes are."""
    return f"POS,{_check_id(cmd_id)}"


def pin(cmd_id: int, gpio: int, level: int | bool) -> str:
    """Drive one GPIO high or low.

    This is for checking wiring with a multimeter during bring-up. The Pico
    refuses any pin that is not one of the spare test pins, so a typo here
    cannot toggle a motor's step line.
    """
    _check_id(cmd_id)
    if not isinstance(gpio, int) or isinstance(gpio, bool):
        raise CommandError(f"gpio must be a whole number, got {gpio!r}")
    if not 0 <= gpio <= GPIO_MAX:
        raise CommandError(f"gpio must be 0..{GPIO_MAX}, got {gpio}")
    return f"PIN,{cmd_id},{gpio},{1 if level else 0}"


def limits(cmd_id: int) -> str:
    """Ask what soft limits and speed range this build was flashed with."""
    return f"LIMITS,{_check_id(cmd_id)}"


def ping(cmd_id: int) -> str:
    """Ask the Pico to identify itself and say how long it has been up."""
    return f"PING,{_check_id(cmd_id)}"


def encode(line: str) -> bytes:
    """A command line as the bytes to write to the port.

    The newline is added here rather than by each builder, so the builders
    return something printable that can go straight into a log, and the one
    piece of framing lives in one place.

    The length check cannot fire on anything the builders produce - the
    longest is about 30 characters against a 95 character buffer - but a
    hand-written line could reach it, and the Pico throws an overlong line
    away, so this is the kinder place to find out.
    """
    if "\n" in line or "\r" in line:
        raise CommandError("a command must be a single line")
    if len(line) > MAX_LINE_LEN:
        raise CommandError(
            f"command is {len(line)} characters; the Pico accepts "
            f"{MAX_LINE_LEN}"
        )
    return (line + "\n").encode("ascii", errors="strict")


# ============================================================================
# Reading replies
# ============================================================================


class Reply:
    """Base class, so a reader has one type to check against."""


@dataclass(frozen=True)
class Ack(Reply):
    """The command was understood and is being acted on."""

    id: int
    verb: str


@dataclass(frozen=True)
class Plan(Reply):
    """What the Pico worked out a MOVE would do, sent before it starts.

    `to_mdeg` is the quantised end angle, which is not simply start plus
    commanded: 45 degrees of azimuth is 2286 steps, or 45.006 degrees. That
    number is the theoretical angle a row of test data is checked against.
    """

    id: int
    axis: str
    steps: int
    from_mdeg: int
    to_mdeg: int
    speed_mdps: int
    triangular: bool  # True = too short a move to ever reach full speed
    est_us: int

    @property
    def from_deg(self) -> float:
        return mdeg_to_deg(self.from_mdeg)

    @property
    def to_deg(self) -> float:
        return mdeg_to_deg(self.to_mdeg)

    @property
    def speed_dps(self) -> float:
        return mdeg_to_deg(self.speed_mdps)

    @property
    def est_s(self) -> float:
        return self.est_us / 1e6


@dataclass(frozen=True)
class Done(Reply):
    """A move finished. `ok` is False when a stop cut it short."""

    id: int
    axis: str
    ok: bool
    steps_done: int
    from_mdeg: int
    to_mdeg: int
    elapsed_us: int

    @property
    def from_deg(self) -> float:
        return mdeg_to_deg(self.from_mdeg)

    @property
    def to_deg(self) -> float:
        return mdeg_to_deg(self.to_mdeg)

    @property
    def elapsed_s(self) -> float:
        return self.elapsed_us / 1e6


@dataclass(frozen=True)
class Pos(Reply):
    """Where both axes are.

    Steps come with the angles because steps are what the firmware actually
    counts; the millidegrees are derived from them.
    """

    id: int
    az_mdeg: int
    el_mdeg: int
    az_steps: int
    el_steps: int
    zeroed: bool

    @property
    def az_deg(self) -> float:
        return mdeg_to_deg(self.az_mdeg)

    @property
    def el_deg(self) -> float:
        return mdeg_to_deg(self.el_mdeg)


@dataclass(frozen=True)
class Pin(Reply):
    """A GPIO was set."""

    id: int
    gpio: int
    level: int


@dataclass(frozen=True)
class Lim(Reply):
    """The soft limits and speed range this Pico was flashed with."""

    id: int
    az_min_mdeg: int
    az_max_mdeg: int
    el_min_mdeg: int
    el_max_mdeg: int
    speed_min_mdps: int
    speed_max_mdps: int


@dataclass(frozen=True)
class Pong(Reply):
    """The answer to PING.

    Uptime is here so we can spot a Pico that rebooted: if it goes backwards
    the position was lost, and moving before re-zeroing would be unsafe.
    """

    id: int
    target: str
    version: str
    uptime_ms: int


@dataclass(frozen=True)
class Mstat(Reply):
    """The heartbeat. No id - this one is not a reply to anything.

    It only arrives while nothing is moving and nothing is half-typed, so a
    gap in the heartbeat during a move is normal rather than a sign of
    trouble.
    """

    uptime_s: int
    az_mdeg: int
    el_mdeg: int
    moving: bool

    @property
    def az_deg(self) -> float:
        return mdeg_to_deg(self.az_mdeg)

    @property
    def el_deg(self) -> float:
        return mdeg_to_deg(self.el_mdeg)


@dataclass(frozen=True)
class Err(Reply):
    """A command was refused.

    The limit error carries numbers instead of a detail string, so we can
    show how far out of range the move was. It is kept as the same class
    rather than a separate one so a caller asking "did my command fail?" has
    a single type to test for; the extra fields are None on every other code.
    """

    id: int
    code: str
    detail: str = ""
    axis: str | None = None
    target_mdeg: int | None = None
    min_mdeg: int | None = None
    max_mdeg: int | None = None

    @property
    def is_limit(self) -> bool:
        return self.target_mdeg is not None

    def message(self) -> str:
        """One line fit to print to the operator."""
        meaning = ERR_MEANING.get(self.code, f"error '{self.code}'")
        if self.is_limit:
            return (
                f"{meaning}: {self.axis} would reach "
                f"{format_mdeg(self.target_mdeg)} deg, allowed "
                f"{format_mdeg(self.min_mdeg)}..{format_mdeg(self.max_mdeg)}"
            )
        if self.detail:
            return f"{meaning} ({self.detail})"
        return meaning


@dataclass(frozen=True)
class Info(Reply):
    """Anything that is not a record: banners, `[motor]` notes, `[fatal]`.

    `malformed` marks a line that began like a record we know but did not
    parse - a truncated DONE, a field that is not a number. It still reaches
    the operator as text, because dropping it would hide exactly the kind of
    fault worth noticing, but it is flagged so the reader can say so instead
    of printing it as ordinary chatter.
    """

    text: str
    malformed: bool = False


# Every record name parse_reply knows. A line starting with one of these but
# not parsing is flagged as malformed; anything else is the Pico talking to a
# human and is passed through untouched.
RECORD_NAMES = frozenset(
    {"ACK", "PLAN", "DONE", "POS", "PIN", "LIM", "PONG", "MSTAT", "ERR"}
)


def _int(text: str) -> int:
    """Strict whole number: an optional sign, then digits, and nothing else.

    Python's int() is looser than the firmware's parser - it accepts
    surrounding whitespace and digit underscores, so "1_0" would come back as
    10. Being exactly as strict as the wire format keeps a corrupted line
    from parsing as a plausible wrong number.
    """
    body = text[1:] if text[:1] in ("+", "-") else text
    if not body or not body.isascii() or not body.isdigit():
        raise ValueError(f"not a whole number: {text!r}")
    return int(text)


def _axis(text: str) -> str:
    if text not in AXIS_NAMES:
        raise ValueError(f"not an axis: {text!r}")
    return text


def _word(text: str, true_word: str, false_word: str) -> bool:
    """One of two fixed words, as a bool. Anything else is a bad line."""
    if text == true_word:
        return True
    if text == false_word:
        return False
    raise ValueError(f"expected {true_word} or {false_word}, got {text!r}")


def _parse_err(f: list[str]) -> Err:
    # Two shapes share the ERR name: the usual four fields, and the seven of
    # a limit refusal, which reports numbers rather than text.
    if len(f) == 7 and f[2] == "limit":
        return Err(
            id=_int(f[1]),
            code="limit",
            axis=_axis(f[3]),
            target_mdeg=_int(f[4]),
            min_mdeg=_int(f[5]),
            max_mdeg=_int(f[6]),
        )
    if len(f) != 4:
        raise ValueError("wrong field count for ERR")
    return Err(id=_int(f[1]), code=f[2], detail=f[3])


def parse_reply(line: str) -> Reply:
    r"""One line from the Pico as a record. Never raises.

    The line may arrive with or without its newline, and with a stray "\r"
    if something along the way added one.
    """
    text = line.rstrip("\r\n")
    if not text.strip():
        return Info("")

    f = text.split(",")
    name = f[0]

    try:
        if name == "ACK" and len(f) == 3:
            return Ack(id=_int(f[1]), verb=f[2])

        if name == "PLAN" and len(f) == 9:
            return Plan(
                id=_int(f[1]),
                axis=_axis(f[2]),
                steps=_int(f[3]),
                from_mdeg=_int(f[4]),
                to_mdeg=_int(f[5]),
                speed_mdps=_int(f[6]),
                triangular=_word(f[7], "TRI", "TRAP"),
                est_us=_int(f[8]),
            )

        if name == "DONE" and len(f) == 8:
            return Done(
                id=_int(f[1]),
                axis=_axis(f[2]),
                ok=_word(f[3], "OK", "ABORT"),
                steps_done=_int(f[4]),
                from_mdeg=_int(f[5]),
                to_mdeg=_int(f[6]),
                elapsed_us=_int(f[7]),
            )

        if name == "POS" and len(f) == 7:
            return Pos(
                id=_int(f[1]),
                az_mdeg=_int(f[2]),
                el_mdeg=_int(f[3]),
                az_steps=_int(f[4]),
                el_steps=_int(f[5]),
                zeroed=_word(f[6], "ZEROED", "UNZEROED"),
            )

        if name == "PIN" and len(f) == 4:
            return Pin(id=_int(f[1]), gpio=_int(f[2]), level=_int(f[3]))

        if name == "LIM" and len(f) == 8:
            return Lim(
                id=_int(f[1]),
                az_min_mdeg=_int(f[2]),
                az_max_mdeg=_int(f[3]),
                el_min_mdeg=_int(f[4]),
                el_max_mdeg=_int(f[5]),
                speed_min_mdps=_int(f[6]),
                speed_max_mdps=_int(f[7]),
            )

        if name == "PONG" and len(f) == 5:
            return Pong(
                id=_int(f[1]), target=f[2], version=f[3], uptime_ms=_int(f[4])
            )

        if name == "MSTAT" and len(f) == 5:
            return Mstat(
                uptime_s=_int(f[1]),
                az_mdeg=_int(f[2]),
                el_mdeg=_int(f[3]),
                moving=_word(f[4], "MOVING", "IDLE"),
            )

        if name == "ERR":
            return _parse_err(f)

    except ValueError:
        return Info(text, malformed=True)

    # A record name we know, with the wrong number of fields.
    if name in RECORD_NAMES:
        return Info(text, malformed=True)
    return Info(text)
