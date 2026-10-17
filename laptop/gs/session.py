r"""Holding a conversation with the Pico.

`protocol.py` turns commands into text and text into records. `ports.py`
opens the port. Neither of them talks: this is the piece in the middle that
actually does, and everything above it - the prompt, the scripted runs, the
CSV - is written against this.

Four jobs, and each one exists because of something that bites otherwise.

**Gluing lines back together.** USB hands over whatever has arrived, split
wherever it likes. Half a line now and the rest in a moment is normal, and
so is two lines in one lump. The Pico does the same job on its side with
LineReader; this is the mirror of it.

**One command at a time.** The firmware works out whether it is busy once
per batch of bytes, before reading them, so two commands that arrive
together are both judged as though neither had run - two MOVEs restart the
axis and the first one's DONE never comes. Nothing here ever has two
commands outstanding. That is not caution, it is the only safe way to speak
to this firmware, and it is why the matching below can be as simple as it
is.

**Matching replies to commands.** A move takes up to a minute and the
replies to it arrive long after the command went out, with heartbeats in
between. Every record carries the id of the command that caused it - except
the ones that do not, which is the next paragraph.

**Noticing a Pico that restarted.** PING and the heartbeat both report how
long the Pico has been up. If that goes backwards it rebooted, which means
it has forgotten where the antenna is. Moving after that without a fresh
zero would drive the gimbal from an imaginary starting point, so every move
is refused until someone re-zeros.

About errors with no id
-----------------------

Some refusals come back as `ERR,0,...`: a line too long to have been read,
and - firmware behaviour, not a typo - any line the parser could not make
sense of, even when its id field was perfectly readable. There is no way to
tell from the line itself which command it belongs to.

Because only one command is ever outstanding, an id-0 error can only be
about that one, so it is treated as its answer. Pipelining would make that
untrue, which is one more reason this layer does not pipeline.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

from gs import protocol as p

# How long to wait for the answer to an ordinary command. Generous: the
# Pico answers in microseconds, so anything approaching this means the line
# or the port is in trouble, not that the Pico is thinking.
DEFAULT_TIMEOUT_S = 2.0

# A move is given its own predicted time plus this much slack, because the
# prediction is what the Pico itself worked out and the only things left to
# cover are the serial round trip and a slow laptop.
MOVE_SLACK_FACTOR = 1.5
MOVE_SLACK_S = 5.0

# Chatter worth keeping for the operator to look at. Older lines fall off
# the end rather than growing without limit during a long session.
CHATTER_KEPT = 200


class SessionError(Exception):
    """Anything that went wrong talking to the Pico."""


class Timeout(SessionError):
    """The Pico did not answer in time."""


class Refused(SessionError):
    """The Pico understood and said no.

    Carries the ERR record, so a caller can tell a soft limit refusal from
    a typo and print the operator-ready message.
    """

    def __init__(self, err: p.Err) -> None:
        super().__init__(err.message())
        self.err = err


class Rebooted(SessionError):
    """The Pico restarted and no longer knows where the antenna is."""


@dataclass(frozen=True)
class MoveOutcome:
    """A finished move: what was predicted, and what happened.

    Both halves are kept because the test data compares them. `plan` is
    what the Pico worked out before it started; `done` is what it reports
    afterwards.
    """

    plan: p.Plan
    done: p.Done

    @property
    def ok(self) -> bool:
        return self.done.ok

    @property
    def overshoot_us(self) -> int:
        """Predicted time minus measured time.

        Always a little positive: est_us counts the gap before the first
        pulse, and elapsed_us is measured from the first pulse to the last.
        A value far from one step interval means something is wrong.
        """
        return self.plan.est_us - self.done.elapsed_us


class Session:
    """One conversation with one Pico.

    Not thread safe, and deliberately so: two threads sharing it would put
    two commands on the wire at once, which is the thing this layer exists
    to prevent.
    """

    def __init__(
        self,
        conn,
        *,
        ids: p.IdSource | None = None,
        on_chatter=None,
        clock=time.monotonic,
        default_timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> None:
        self.conn = conn
        self.ids = ids if ids is not None else p.IdSource()
        self.clock = clock
        self.default_timeout_s = default_timeout_s

        self.on_chatter = on_chatter
        self.chatter: deque[p.Reply] = deque(maxlen=CHATTER_KEPT)

        self._buf = b""
        # Records read while waiting for a different answer.
        self._spare: list[p.Reply] = []

        self.rebooted = False
        self._last_uptime_s: int | None = None

    # --- opening and closing ----------------------------------------------

    @classmethod
    def open(cls, port: str | None = None, *, timeout: float = 0.2, **kwargs):
        """Open a port and wrap it. `port` may be a device, a URL, or None.

        The read timeout is short on purpose. It is how often this layer
        gets to check its own deadline, not how long it is prepared to
        wait, so a long move still works - it just checks in frequently
        while nothing is arriving.
        """
        from gs import ports  # imported here so dry runs need no pyserial

        return cls(ports.open_port(port, timeout=timeout), **kwargs)

    def close(self) -> None:
        self.conn.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- reading ------------------------------------------------------------

    def _read_some(self) -> bytes:
        """Whatever has arrived, or empty if nothing has."""
        waiting = getattr(self.conn, "in_waiting", 0) or 0
        try:
            return self.conn.read(max(1, waiting))
        except Exception as exc:  # the port vanished under us
            raise SessionError(f"lost the connection: {exc}") from exc

    def _next_record(self, deadline: float) -> p.Reply | None:
        """One record, or None once the deadline has passed."""
        while True:
            # Either line ending finishes a line, matching the Pico's own
            # reader, and a \r\n pair leaves an empty line which is dropped.
            # Whichever comes first wins: splitting on "\n" alone would
            # swallow a "\r" in the middle and join two records into one.
            cut = min(
                (
                    i
                    for i in (self._buf.find(b"\n"), self._buf.find(b"\r"))
                    if i >= 0
                ),
                default=-1,
            )
            if cut >= 0:
                head, self._buf = self._buf[:cut], self._buf[cut + 1 :]
                text = head.decode("ascii", errors="replace").strip()
                if text:
                    return self._note(p.parse_reply(text))
                continue  # blank line, or the second half of a \r\n pair

            if self.clock() >= deadline:
                return None

            data = self._read_some()
            if data:
                self._buf += data

    def _note(self, record: p.Reply) -> p.Reply:
        """Watch every record go past, for the things worth tracking."""
        # Uptime is compared in whole seconds. The heartbeat reports
        # seconds and PONG reports milliseconds, so comparing the two
        # directly would make an ordinary heartbeat look like a reboot.
        uptime_s = None
        if isinstance(record, p.Pong):
            uptime_s = record.uptime_ms // 1000
        elif isinstance(record, p.Mstat):
            uptime_s = record.uptime_s

        if uptime_s is not None:
            went_back = (
                self._last_uptime_s is not None
                and uptime_s < self._last_uptime_s
            )
            if went_back:
                self.rebooted = True
            self._last_uptime_s = uptime_s

        if isinstance(record, (p.Mstat, p.Info)):
            self.chatter.append(record)
            if self.on_chatter is not None:
                self.on_chatter(record)

        return record

    def drain(self, seconds: float = 0.0) -> list[p.Reply]:
        """Read anything waiting and hand it back.

        Used to collect the startup banner, and to clear the line before a
        command so an old heartbeat is not mistaken for an answer.
        """
        deadline = self.clock() + seconds
        seen = list(self._spare)
        self._spare.clear()
        while True:
            record = self._next_record(deadline)
            if record is None:
                return seen
            seen.append(record)

    def drain_until_quiet(
        self, quiet_s: float = 0.05, max_s: float = 1.0
    ) -> list[p.Reply]:
        """Read until the Pico stops talking, or until `max_s` is up.

        Better than waiting a fixed time for the banner: a Pico that
        answers at once is not kept waiting, and one that is still
        printing is not cut off mid-way. Used at startup, where how long
        the banner takes to arrive depends on the port rather than on us.
        """
        seen = []
        hard_stop = self.clock() + max_s
        while True:
            until = min(self.clock() + quiet_s, hard_stop)
            record = self._next_record(until)
            if record is None:
                return seen
            seen.append(record)
            # A heartbeat means the Pico has nothing else to say: it only
            # sends one when nothing is moving and no line is half-read,
            # so the banner must be finished. Waiting for silence as well
            # would just add the quiet period to every startup.
            if isinstance(record, p.Mstat):
                return seen
            if self.clock() >= hard_stop:
                return seen

    # --- sending ------------------------------------------------------------

    def _send(self, line: str) -> None:
        self.conn.write(p.encode(line))
        flush = getattr(self.conn, "flush", None)
        if flush is not None:
            flush()

    def _await(self, cmd_id: int, kinds: tuple, deadline: float, what: str):
        """Wait for this command's answer, keeping anything else for later."""
        # Something read earlier while waiting for a different answer.
        for i, record in enumerate(self._spare):
            if _matches(record, cmd_id, kinds):
                return self._spare.pop(i)
            if _is_error_for(record, cmd_id):
                raise Refused(self._spare.pop(i))

        while True:
            record = self._next_record(deadline)
            if record is None:
                raise Timeout(
                    f"the Pico did not answer {what} (id {cmd_id}) in time"
                )
            if _matches(record, cmd_id, kinds):
                return record
            if _is_error_for(record, cmd_id):
                raise Refused(record)
            if not isinstance(record, (p.Mstat, p.Info)):
                # A reply to something else: an abandoned command's DONE,
                # most likely. Kept rather than dropped, so whoever is
                # waiting for it can still find it.
                self._spare.append(record)
                del self._spare[:-CHATTER_KEPT]

    def request(
        self,
        line: str,
        cmd_id: int,
        kinds: tuple,
        *,
        timeout_s: float | None = None,
        what: str = "that command",
    ):
        """Send one command and wait for its answer."""
        if self.rebooted:
            raise Rebooted(
                "the Pico restarted and has forgotten where the antenna is. "
                "Re-zero before moving."
            )
        wait = self.default_timeout_s if timeout_s is None else timeout_s
        self._send(line)
        return self._await(cmd_id, kinds, self.clock() + wait, what)

    # --- the commands -------------------------------------------------------

    def ping(self) -> p.Pong:
        cmd_id = self.ids.next()
        return self.request(
            p.ping(cmd_id), cmd_id, (p.Pong,), what="PING"
        )

    def limits(self) -> p.Lim:
        cmd_id = self.ids.next()
        return self.request(
            p.limits(cmd_id), cmd_id, (p.Lim,), what="LIMITS"
        )

    def position(self) -> p.Pos:
        cmd_id = self.ids.next()
        return self.request(p.pos(cmd_id), cmd_id, (p.Pos,), what="POS")

    def zero(self, axis: str = "ALL", deg: float | None = None) -> p.Ack:
        """Tell the Pico what angle an axis is sitting at. Nothing moves.

        This also clears a noticed reboot, because re-zeroing is exactly
        the thing that makes moving safe again.
        """
        cmd_id = self.ids.next()
        line = p.zero(cmd_id, axis, deg)
        # Sent directly rather than through request(), which would refuse
        # while the reboot flag is up - and this is the cure for that.
        self._send(line)
        ack = self._await(
            cmd_id,
            (p.Ack,),
            self.clock() + self.default_timeout_s,
            "ZERO",
        )
        self.rebooted = False
        return ack

    def set_pin(self, gpio: int, level: int) -> p.Pin:
        cmd_id = self.ids.next()
        return self.request(
            p.pin(cmd_id, gpio, level), cmd_id, (p.Pin,), what="PIN"
        )

    # --- moving -------------------------------------------------------------

    def begin_move(
        self, axis: str, speed_dps: float, delta_deg: float
    ) -> p.Plan:
        """Start a move and return the plan. Does not wait for it to finish."""
        cmd_id = self.ids.next()
        line = p.move(cmd_id, axis, speed_dps, delta_deg)
        self.request(line, cmd_id, (p.Ack,), what="MOVE")
        return self._await(
            cmd_id,
            (p.Plan,),
            self.clock() + self.default_timeout_s,
            "the plan for MOVE",
        )

    def await_done(
        self, plan: p.Plan, *, timeout_s: float | None = None
    ) -> p.Done:
        """Wait for the move described by `plan` to report that it finished.

        Safe to call again after it raises Timeout, or after a Ctrl-C: the
        move keeps running on the Pico whatever this end does, so the DONE
        is still coming and giving up on it would leave the next command
        reading somebody else's answer.
        """
        if timeout_s is None:
            timeout_s = plan.est_s * MOVE_SLACK_FACTOR + MOVE_SLACK_S
        return self._await(
            plan.id, (p.Done,), self.clock() + timeout_s, "the end of the move"
        )

    def move(
        self, axis: str, speed_dps: float, delta_deg: float, *, timeout_s=None
    ) -> MoveOutcome:
        """Start a move and wait for it to finish."""
        plan = self.begin_move(axis, speed_dps, delta_deg)
        return MoveOutcome(plan, self.await_done(plan, timeout_s=timeout_s))

    def stop(self) -> None:
        """Ask for a controlled stop.

        No reply is waited for. While an axis is turning the Pico answers a
        stop with the move's own DONE and nothing else, because it will not
        print anything between two pulses - so the thing to wait for
        afterwards is the DONE, with await_done.
        """
        self._send(p.stop(self.ids.next()))

    def estop(self) -> None:
        """Stop at once. The Pico gives up its zero, so re-zero afterwards."""
        self._send(p.estop(self.ids.next()))


def _matches(record: p.Reply, cmd_id: int, kinds: tuple) -> bool:
    return isinstance(record, kinds) and getattr(record, "id", None) == cmd_id


def _is_error_for(record: p.Reply, cmd_id: int) -> bool:
    """Is this refusal ours?

    Id 0 counts, because the firmware uses it for lines it could not read
    at all - and with only one command outstanding there is nothing else it
    could be about.
    """
    return isinstance(record, p.Err) and record.id in (cmd_id, 0)
