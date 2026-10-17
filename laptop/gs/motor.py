r"""The motor test prompt: `py -m gs.motor`.

This is the thing an operator actually uses. Everything below it is
already written and tested - `session.py` holds the conversation,
`protocol.py` knows the wire format - so what is left here is turning
typed words into those calls and printing the answers in a form worth
reading at a bench.

    py -m gs.motor --dry-run        no hardware at all, runs the simulator
    py -m gs.motor                  finds the Pico by itself
    py -m gs.motor --port COM5      when there is more than one

Type `help` at the prompt for the commands.

Why a prompt and not a pile of flags
------------------------------------

The test procedure is a sequence: zero it, move it, read the protractor,
write the number down, move it again. A prompt keeps the Pico's position
between steps, which a command that exits after each move cannot do - the
Pico would keep its position, but nothing would be tying one reading to
the next.

Two rules it will not bend
--------------------------

**One command at a time.** Enforced in `session.py`, where the reason is
spelled out. Nothing here can send a second command while one is still
being answered.

**Ctrl-C stops the motor, it does not abandon it.** Pressing it during a
move sends a stop and then goes on waiting for the move to report what it
did. Walking away from a running move would leave the Pico mid-turn with
the laptop no longer listening - and the next command would read the old
move's answer.
"""

from __future__ import annotations

import argparse
import sys

from gs import motor_log
from gs import motor_runs
from gs import protocol as p
from gs import session as sn

PROMPT = "motor> "

# A protractor reading further than this from the predicted angle is
# probably not a measurement of the same thing. The usual cause is typing
# how far it moved instead of where it ended up - the two are the same
# number on the first move of a run and nowhere near it after that.
READING_WARN_DEG = 5.0

HELP_TEXT = """\
  move <az|el> <deg/s> <degrees>   turn an axis, + or -
  zero [az|el|all] [degrees]       say where it is now
  pos                              where it thinks it is
  stop                             slow to a stop
  estop                            stop at once (loses the zero)
  pin <gpio> <0|1>                 set a spare pin, for a meter
  ping                             is it there, and how long up
  limits                           what it was built with
  sweep [angle=45] [speeds=1,5,90] [pause]   one angle at each speed
  random [count=10] [speed=5] [seed=N] [pause]   angles within the limits
  true [row] <degrees>             what the protractor really reads
  note [row] <text>                write a note against a row
  rows [n]                         show the last n rows of the table
  verbose [on|off]                 show the heartbeat
  help                             this
  quit                             leave

  Angles are relative: 'move az 5 -45' turns back 45 degrees.
  Ctrl-C during a move stops the motor, then waits for its report."""


class UsageError(Exception):
    """The operator typed a command we know, with arguments we do not."""


def _axis(word: str, allow_all: bool = False) -> str:
    name = word.strip().upper()
    allowed = p.ZERO_AXES if allow_all else p.MOVE_AXES
    if name not in allowed:
        names = " or ".join(a.lower() for a in allowed)
        raise UsageError(f"axis must be {names}")
    return name


def _looks_like_a_number(word: str) -> bool:
    try:
        float(word)
    except ValueError:
        return False
    return True


def _options(args: list[str], allowed: set[str], flags=("pause",)):
    """Read `name=value` arguments, plus bare words like `pause`.

    The scripted runs have too many knobs to remember in order, and
    getting the order wrong silently would mean running the wrong test.
    """
    values: dict[str, str] = {}
    marks: set[str] = set()

    for token in args:
        if "=" in token:
            key, _, value = token.partition("=")
            key = key.strip().lower()
            if key not in allowed:
                known = ", ".join(sorted(allowed))
                raise UsageError(f"no setting called {key!r}. Try: {known}")
            values[key] = value.strip()
        elif token.lower() in flags:
            marks.add(token.lower())
        else:
            raise UsageError(
                f"do not understand {token!r} - settings look like name=value"
            )
    return values, marks


def _number(word: str, what: str) -> float:
    try:
        return float(word)
    except ValueError:
        raise UsageError(f"{what} must be a number, got {word!r}") from None


class Console:
    """The prompt itself, with the session it drives.

    Printing goes through `self.say` rather than straight to stdout so the
    tests can read back everything it said.
    """

    def __init__(
        self,
        session: sn.Session,
        out=None,
        verbose: bool = False,
        log: motor_log.MoveLog | None = None,
    ):
        self.session = session
        self.verbose = verbose
        self.log = log
        # Where `pause` reads the protractor reading from. interact()
        # replaces it with whatever it was given.
        self.stdin = sys.stdin
        self._out = out if out is not None else sys.stdout
        self.quit = False

    # --- output ------------------------------------------------------------

    def say(self, text: str = "") -> None:
        print(text, file=self._out)

    def ask(self, question: str) -> str:
        """Put a question to the operator and wait for the answer.

        Used by the scripted runs when they pause between moves. An
        empty answer, or no input at all, means skip it - a reading
        nobody took is better left blank than guessed at.
        """
        print(question, end="", file=self._out, flush=True)
        try:
            line = self.stdin.readline()
        except (KeyboardInterrupt, ValueError):
            return ""
        if not line:
            return ""
        answer = line.strip()
        if not getattr(self.stdin, "isatty", lambda: False)():
            # No terminal to echo it, so echo it ourselves and keep the
            # transcript readable.
            self.say(answer)
        return answer

    def show_chatter(self, record: p.Reply) -> None:
        """Lines the Pico sends unprompted.

        The heartbeat is hidden unless asked for - one line every five
        seconds would scroll everything else away during a long session.
        Anything else it says is shown, because the only things it says
        unprompted are the banner and bad news.
        """
        if isinstance(record, p.Mstat):
            if self.verbose:
                self.say(
                    f"       heartbeat  az {p.format_mdeg(record.az_mdeg)}  "
                    f"el {p.format_mdeg(record.el_mdeg)}  "
                    f"up {record.uptime_s} s"
                )
        elif isinstance(record, p.Info) and record.text:
            mark = "  ??" if record.malformed else "    "
            self.say(f"{mark}   {record.text}")

    # --- the commands -------------------------------------------------------

    def do_help(self, args: list[str]) -> None:
        self.say(HELP_TEXT)

    def do_quit(self, args: list[str]) -> None:
        self.quit = True

    def do_ping(self, args: list[str]) -> None:
        pong = self.session.ping()
        up_s = pong.uptime_ms / 1000.0
        self.say(f"  {pong.target} {pong.version}, up {up_s:.1f} s")
        if pong.version == "fake":
            self.say("  (this is the simulator, not a Pico)")

    def do_limits(self, args: list[str]) -> None:
        lim = self.session.limits()
        self.say(
            f"  az {p.format_mdeg(lim.az_min_mdeg)}"
            f"..{p.format_mdeg(lim.az_max_mdeg)} deg"
            f"   el {p.format_mdeg(lim.el_min_mdeg)}"
            f"..{p.format_mdeg(lim.el_max_mdeg)} deg"
        )
        self.say(
            f"  speed {p.format_mdeg(lim.speed_min_mdps)}"
            f"..{p.format_mdeg(lim.speed_max_mdps)} deg/s"
        )

    def do_pos(self, args: list[str]) -> None:
        pos = self.session.position()
        state = "zeroed" if pos.zeroed else "NOT ZEROED - send zero first"
        self.say(
            f"  az {p.format_mdeg(pos.az_mdeg)} deg ({pos.az_steps} steps)"
            f"   el {p.format_mdeg(pos.el_mdeg)} deg ({pos.el_steps} steps)"
            f"   {state}"
        )

    def do_zero(self, args: list[str]) -> None:
        # "zero 90" means all axes at 90, not an axis called 90.
        if args and _looks_like_a_number(args[0]):
            args = ["ALL"] + args

        axis = _axis(args[0], allow_all=True) if args else "ALL"
        deg = _number(args[1], "angle") if len(args) > 1 else None
        if len(args) > 2:
            raise UsageError("zero takes at most an axis and an angle")

        self.session.zero(axis, deg)
        where = "0.000" if deg is None else f"{deg}"
        self.say(f"  {axis} is now {where} deg. Nothing moved.")

    def do_pin(self, args: list[str]) -> None:
        if len(args) != 2:
            raise UsageError("pin <gpio> <0|1>")
        gpio = int(_number(args[0], "gpio"))
        level = int(_number(args[1], "level"))
        reply = self.session.set_pin(gpio, level)
        self.say(f"  GP{reply.gpio} is now {'high' if reply.level else 'low'}")

    def do_stop(self, args: list[str]) -> None:
        self.session.stop()
        self.say("  stop sent. Nothing was moving, if nothing answers.")

    def do_estop(self, args: list[str]) -> None:
        self.session.estop()
        self.say("  emergency stop sent. Re-zero before moving again.")

    def do_verbose(self, args: list[str]) -> None:
        if args:
            self.verbose = args[0].lower() in ("on", "yes", "true", "1")
        self.say(f"  heartbeat {'shown' if self.verbose else 'hidden'}")

    def do_move(self, args: list[str]) -> None:
        if len(args) != 3:
            raise UsageError("move <az|el> <deg/s> <degrees>")
        axis = _axis(args[0])
        speed = _number(args[1], "speed")
        angle = _number(args[2], "angle")

        self.run_move(axis, speed, angle)

    # --- the move itself ----------------------------------------------------

    def run_move(
        self,
        axis: str,
        speed_dps: float,
        delta_deg: float,
        row_hint: bool = True,
    ):
        """One move, start to finish, with the waiting handled.

        Returns the outcome so the scripted runs can log it, or None if
        the move was refused - a refusal is reported here and is not worth
        raising again further up.
        """
        try:
            plan = self.session.begin_move(axis, speed_dps, delta_deg)
        except sn.Refused as refused:
            self.say(f"  refused: {refused}")
            return None

        self.say(
            f"  plan   {plan.steps:+d} steps  "
            f"{p.format_mdeg(plan.from_mdeg)} -> "
            f"{p.format_mdeg(plan.to_mdeg)} deg"
            f"  at {p.format_mdeg(plan.speed_mdps)} deg/s"
            f"  {'ramped' if plan.triangular else 'flat out'}"
            f"  about {plan.est_s:.2f} s"
        )

        done = self._wait_out(plan)
        if done is None:
            return None

        outcome = sn.MoveOutcome(plan, done)
        self.report_move(outcome)

        if self.log is not None:
            row = self.log.add(outcome, commanded_deg=delta_deg)
            # Left out when a scripted run is about to ask for the
            # reading itself, which would otherwise print two prompts
            # for the same number.
            if row_hint:
                self.say(f"         row {row.row} - type 'true <degrees>' "
                         f"once you have read the protractor")
        return outcome

    def _wait_out(self, plan: p.Plan):
        """Wait for the move, turning Ctrl-C into a stop rather than a bolt."""
        stopping = False
        while True:
            try:
                return self.session.await_done(plan)
            except KeyboardInterrupt:
                if not stopping:
                    stopping = True
                    self.say(
                        "  ^C - stopping. Waiting for it to come to rest."
                    )
                    self.session.stop()
                    continue
                # Twice means they want out now. The move is still running
                # on the Pico and its DONE is still coming, so say so
                # rather than pretending the axis is parked.
                self.say(
                    "  ^C again - no longer waiting. The axis may still be "
                    "turning, and its position here is now a guess. Send "
                    "estop, then pos, then zero."
                )
                return None
            except sn.Timeout as exc:
                self.say(f"  {exc}")
                self.say(
                    "  The move may still be running. Try pos, and estop "
                    "if it is not where it should be."
                )
                return None

    def report_move(self, outcome: sn.MoveOutcome) -> None:
        done = outcome.done
        verdict = "finished" if done.ok else "STOPPED SHORT"
        self.say(
            f"  done   {verdict}  {done.steps_done:+d} steps in "
            f"{done.elapsed_s:.2f} s  (predicted {outcome.plan.est_s:.2f} s)"
        )
        self.say(f"         now at {p.format_mdeg(done.to_mdeg)} deg")

    # --- the scripted runs --------------------------------------------------

    def do_sweep(self, args: list[str]) -> None:
        """The same angle at a series of speeds."""
        values, marks = _options(args, {"axis", "angle", "speeds"})

        axis = _axis(values.get("axis", "az"))
        angle = _number(
            str(values.get("angle", motor_runs.DEFAULT_SWEEP_ANGLE_DEG)),
            "angle",
        )
        if "speeds" in values:
            speeds = [
                _number(word, "speed")
                for word in values["speeds"].split(",")
                if word
            ]
        else:
            speeds = list(motor_runs.DEFAULT_SWEEP_SPEEDS)

        motor_runs.sweep(
            self,
            axis=axis,
            angle_deg=angle,
            speeds=speeds,
            pause="pause" in marks,
        )

    def do_random(self, args: list[str]) -> None:
        """Unpredictable angles inside the soft limits."""
        values, marks = _options(args, {"axis", "count", "speed", "seed"})

        axis = _axis(values.get("axis", "az"))
        count = int(
            _number(str(values.get("count", motor_runs.DEFAULT_RANDOM_COUNT)),
                    "count")
        )
        speed = _number(
            str(values.get("speed", motor_runs.DEFAULT_RANDOM_SPEED_DPS)),
            "speed",
        )
        seed = None
        if "seed" in values:
            seed = int(_number(values["seed"], "seed"))

        if count < 1:
            raise UsageError("count must be at least 1")

        motor_runs.random_moves(
            self,
            axis=axis,
            count=count,
            speed_dps=speed,
            seed=seed,
            pause="pause" in marks,
        )

    # --- the results table --------------------------------------------------

    def _need_log(self) -> motor_log.MoveLog:
        if self.log is None:
            raise UsageError(
                "no results file is open. Start again with --log"
            )
        return self.log

    def do_true(self, args: list[str]) -> None:
        """Record what the protractor actually read."""
        log = self._need_log()
        if not args or len(args) > 2:
            raise UsageError("true [row] <degrees>")

        if len(args) == 2:
            row = int(_number(args[0], "row"))
            deg = _number(args[1], "angle")
        else:
            row, deg = None, _number(args[0], "angle")

        try:
            entry = log.set_true(deg, row)
        except motor_log.LogError as exc:
            raise UsageError(str(exc)) from None

        self.report_reading(entry)

    def report_reading(self, entry, indent: str = "  ") -> None:
        """Say what was recorded, and query it if it looks unlikely.

        Recorded either way - a surprising measurement is still a
        measurement, and this is a question rather than a correction.
        """
        self.say(
            f"{indent}row {entry.row}: measured "
            f"{p.format_mdeg(entry.true_mdeg)}, predicted "
            f"{p.format_mdeg(entry.predicted_mdeg)}, out by "
            f"{p.format_mdeg(entry.error_mdeg)} deg"
        )
        if abs(entry.error_mdeg) > p.deg_to_mdeg(READING_WARN_DEG):
            self.say(
                f"{indent}that is a long way out. The reading wanted is "
                f"where the antenna is now, not how far it just moved. "
                f"Retype it with 'true {entry.row} <degrees>' if that was it."
            )

    def do_note(self, args: list[str]) -> None:
        log = self._need_log()
        if not args:
            raise UsageError("note [row] <text>")

        row = None
        if len(args) > 1 and _looks_like_a_number(args[0]):
            row = int(_number(args[0], "row"))
            args = args[1:]

        try:
            entry = log.annotate(" ".join(args), row)
        except motor_log.LogError as exc:
            raise UsageError(str(exc)) from None
        self.say(f"  noted against row {entry.row}")

    def do_rows(self, args: list[str]) -> None:
        log = self._need_log()
        count = int(_number(args[0], "count")) if args else 10
        if not log.rows:
            self.say("  nothing logged yet")
            return
        for entry in log.rows[-count:]:
            self.say(entry.summary())
        missing = log.missing_readings()
        if missing:
            numbers = ", ".join(str(r.row) for r in missing)
            self.say(f"  still waiting on a reading for row {numbers}")

    # --- the loop -----------------------------------------------------------

    COMMANDS = {
        "move": do_move,
        "zero": do_zero,
        "pos": do_pos,
        "stop": do_stop,
        "estop": do_estop,
        "pin": do_pin,
        "ping": do_ping,
        "limits": do_limits,
        "sweep": do_sweep,
        "random": do_random,
        "true": do_true,
        "note": do_note,
        "rows": do_rows,
        "verbose": do_verbose,
        "help": do_help,
        "?": do_help,
        "quit": do_quit,
        "exit": do_quit,
    }

    def run_line(self, text: str) -> None:
        """One typed line. Never raises for anything the operator can cause."""
        line = text.strip()
        if not line or line.startswith("#"):
            return

        word, _, rest = line.partition(" ")
        handler = self.COMMANDS.get(word.lower())
        if handler is None:
            self.say(f"  no such command: {word}. Type help.")
            return

        try:
            handler(self, rest.split())
        except UsageError as exc:
            self.say(f"  {exc}")
        except sn.Refused as exc:
            self.say(f"  refused: {exc}")
        except sn.Rebooted as exc:
            self.say(f"  {exc}")
        except sn.Timeout as exc:
            self.say(f"  {exc}")
        except sn.SessionError as exc:
            self.say(f"  {exc}")
        except KeyboardInterrupt:
            self.say("  ^C")

    def interact(self, stdin=None, prompt: str = PROMPT) -> None:
        """Read lines until they run out or someone types quit."""
        stream = stdin if stdin is not None else sys.stdin
        self.stdin = stream
        interactive = stream.isatty() if hasattr(stream, "isatty") else False

        while not self.quit:
            if interactive:
                print(prompt, end="", file=self._out, flush=True)
            try:
                line = stream.readline()
            except KeyboardInterrupt:
                self.say("")
                continue
            if not line:  # end of input, or Ctrl-D
                self.say("")
                return
            if not interactive:
                # Echo what came in, so a piped session reads as a
                # transcript rather than as answers to invisible questions.
                self.say(f"{prompt}{line.rstrip()}")
            self.run_line(line)


# ============================================================================
# Starting up
# ============================================================================


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="py -m gs.motor",
        description="Drive the ground station's two motors by hand.",
    )
    parser.add_argument(
        "--port",
        help="serial port, or a URL such as socket://localhost:7777. "
        "Left out, the Pico is found by itself",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="talk to the built-in simulator instead of a Pico. "
        "Nothing is plugged in and nothing turns",
    )
    parser.add_argument(
        "--fast",
        type=float,
        default=1.0,
        metavar="N",
        help="with --dry-run, run the simulator's clock N times faster",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="list the serial ports and exit",
    )
    parser.add_argument(
        "--log",
        nargs="?",
        const="",
        metavar="FILE",
        help="write the results table to this CSV as the test runs. "
        "Give --log with no name for a dated one in the current folder",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="show the heartbeat as it arrives",
    )
    parser.add_argument(
        "-c",
        "--command",
        action="append",
        default=[],
        metavar="TEXT",
        help="run this command and exit. May be given more than once",
    )
    return parser


def open_session(args) -> sn.Session:
    """The one place that decides what we are talking to."""
    if args.dry_run:
        # No socket and no pyserial: the simulator is handed straight to
        # the session. This is why --dry-run works on a machine that has
        # never had pyserial installed.
        from gs.fake_motor_pico import LoopbackLink
        from gs.motor_sim import MotorSim

        return sn.Session(LoopbackLink(sim=MotorSim(), scale=args.fast))

    return sn.Session.open(args.port)


def main(argv: list[str] | None = None, stdin=None, out=None) -> int:
    args = build_parser().parse_args(argv)
    stream = sys.stdout if out is None else out

    if args.list:
        from gs import ports

        return ports.main([])

    if args.fast != 1.0 and not args.dry_run:
        print("--fast only means anything with --dry-run", file=stream)
        return 2

    try:
        session = open_session(args)
    except Exception as exc:  # PortError, or pyserial missing
        print(exc, file=stream)
        return 2

    console = Console(session, out=stream, verbose=args.verbose)
    session.on_chatter = console.show_chatter

    try:
        if args.dry_run:
            print(
                "SIMULATION - nothing is plugged in, nothing will turn.",
                file=stream,
            )

        # Collect the banner before the first prompt, so the operator sees
        # what they are connected to rather than a bare cursor. Printing
        # it is the chatter hook's job, already attached above - doing it
        # again here would show the whole banner twice.
        session.drain_until_quiet()

        # One PING before anything else. It proves the link works rather
        # than letting the first real command be the thing that discovers
        # it does not, and it names what answered - which goes in every
        # row of the results table, so a simulated run cannot be mistaken
        # for a real one later.
        who = ""
        try:
            pong = session.ping()
            who = f"{pong.target} {pong.version}"
            print(f"connected: {who}", file=stream)
        except sn.SessionError as exc:
            print(f"warning: no answer to PING ({exc})", file=stream)

        if args.log is not None:
            path = args.log or motor_log.default_name()
            console.log = motor_log.MoveLog(path=path, pico=who)
            console.log.write()  # so the file exists from the start
            print(f"logging to {path}", file=stream)

        if stdin is not None:
            console.stdin = stdin

        if args.command:
            for text in args.command:
                console.say(f"{PROMPT}{text}")
                console.run_line(text)
        else:
            console.interact(stdin)

        _warn_about_blanks(console, stream)
    finally:
        session.close()
    return 0


def _warn_about_blanks(console: Console, stream) -> None:
    """Say so before anyone packs up.

    A missing protractor reading is not a gap that can be filled in
    later from anywhere - the antenna has moved on.
    """
    if console.log is None:
        return
    missing = console.log.missing_readings()
    if missing:
        numbers = ", ".join(str(r.row) for r in missing)
        print("", file=stream)
        print(
            f"{len(missing)} move(s) have no protractor reading: "
            f"row {numbers}",
            file=stream,
        )
    print(f"results in {console.log.path}", file=stream)


if __name__ == "__main__":
    sys.exit(main())
