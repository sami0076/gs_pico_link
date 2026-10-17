r"""The pretend Pico, served over a socket so the real code can talk to it.

`motor_sim.py` is the behaviour; this is the plug. There are two of them.

`FakePico` listens on a TCP port and speaks the motor protocol down it, so
anything that opens a serial port can open this instead. `LoopbackLink`,
further down, skips the socket and hands the simulator straight to a
Session, which is what `--dry-run` uses: no second terminal, and no
pyserial needed at all.

The socket one exists so that the real pyserial path gets exercised rather
than quietly bypassed:

    py -m gs.fake_motor_pico          # in one terminal
    ... --port socket://localhost:7777

That URL works because `ports.open_port()` uses pyserial's
`serial_for_url`, which treats `socket://` as just another port. Nothing in
the laptop code needs to know whether it is talking to a simulator or to a
Pico - which is the point, because a test that only exercises a special
"pretend" path proves nothing about the real one.

Running fast
------------

By default a move takes exactly as long as it really would: a half turn of
azimuth at 5 deg/sec is 36 seconds of sitting and waiting. That is the
honest default, because waiting is part of what the software has to cope
with. `--fast 20` runs the simulator's clock twenty times over, so the same
move takes under two seconds. The heartbeat speeds up with it - the whole
machine is simply living faster.

What it does not pretend to be
------------------------------

**It says so when asked.** PONG reports the version as "fake" and the
banner carries a SIMULATOR line. Every record is byte-for-byte what the
real firmware sends, so that one line is the only thing standing between a
simulated run and a page of test data nobody can trust.

**One client at a time**, like a real serial port, which only one program
may hold open.

**Writes block.** A real Pico stops dead when the laptop stops reading its
USB output, which is the reason the firmware goes quiet during a move.
Sending here blocks the same way rather than quietly buffering, so that
behaviour can be provoked on purpose.
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
import threading
import time

from gs.motor_sim import MotorSim

DEFAULT_PORT = 7777

# Longest the loop will sleep with nothing to do. Short enough that Ctrl-C
# and stop() feel immediate, long enough not to spin the processor.
MAX_SLEEP_S = 0.05


class FakePico:
    """A simulator on a socket.

    Use it as a context manager, or call `serve_forever()` directly. The
    tests run it in a thread and point pyserial at `url`.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        scale: float = 1.0,
        sim: MotorSim | None = None,
        log=None,
    ) -> None:
        if scale <= 0:
            raise ValueError("scale must be greater than zero")

        self.sim = sim if sim is not None else MotorSim()
        self.scale = scale
        self._log = log

        self._listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Without this, a restart within a minute or so of the last one is
        # refused on Linux while the old socket finishes closing down.
        self._listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen.bind((host, port))
        self._listen.listen(1)

        # Asked back rather than remembered, so passing port 0 - which lets
        # the machine choose a free one - gives the real number. The tests
        # rely on that to avoid colliding with a server someone left running.
        self.host, self.port = self._listen.getsockname()[:2]

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._t0 = time.monotonic()

    # --- addresses ---------------------------------------------------------

    @property
    def url(self) -> str:
        """What to pass as --port."""
        return f"socket://{self.host}:{self.port}"

    def now_us(self) -> int:
        """The simulator's clock: real time, sped up by the scale."""
        return int((time.monotonic() - self._t0) * 1_000_000 * self.scale)

    # --- running -----------------------------------------------------------

    def start(self) -> FakePico:
        """Serve in a background thread."""
        self._thread = threading.Thread(target=self.serve_forever, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._listen.close()

    def __enter__(self) -> FakePico:
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def serve_forever(self) -> None:
        """Accept one client at a time until stopped."""
        while not self._stop.is_set():
            ready, _, _ = select.select([self._listen], [], [], MAX_SLEEP_S)
            if not ready:
                continue
            try:
                conn, addr = self._listen.accept()
            except OSError:  # the listening socket was closed under us
                return
            self._say(f"client connected from {addr[0]}:{addr[1]}")
            try:
                self._serve_client(conn)
            except (ConnectionError, OSError) as exc:
                self._say(f"client lost: {exc}")
            finally:
                conn.close()
                self._say("client gone - the simulator keeps its position")

    def _serve_client(self, conn: socket.socket) -> None:
        # The banner goes out on every connection. A real Pico only prints
        # it once, when the port is first opened after boot, but a client
        # that reconnects and sees nothing at all looks broken. Uptime keeps
        # counting either way, so nothing here can be mistaken for a reboot.
        self._send(conn, self.sim.banner())

        while not self._stop.is_set():
            now = self.now_us()

            # Sleep until the simulator has something to say - the end of a
            # move, or the next heartbeat - rather than waking constantly.
            due = self.sim.next_due_us(now)
            wait = (due - now) / 1_000_000.0 / self.scale
            wait = max(0.0, min(MAX_SLEEP_S, wait))

            ready, _, _ = select.select([conn], [], [], wait)

            if ready:
                data = conn.recv(4096)
                if not data:
                    return  # the client closed the port
                self._send(conn, self.sim.feed(data, self.now_us()))

            self._send(conn, self.sim.poll(self.now_us()))

    def _send(self, conn: socket.socket, lines: list[str]) -> None:
        if not lines:
            return
        for line in lines:
            self._say(f"  -> {line}")
        payload = "".join(line + "\n" for line in lines)
        conn.sendall(payload.encode("ascii", errors="replace"))

    def _say(self, message: str) -> None:
        if self._log is not None:
            self._log(message)


class LoopbackLink:
    """The simulator wired straight to a Session, with no socket at all.

    This is what `--dry-run` uses. The socket server above exists so that
    the real pyserial path gets exercised; this exists so the whole CLI
    runs on a laptop with no pyserial installed and no second terminal
    open. It answers to the few things a Session asks of a port - read,
    write, close - and nothing else.

    `scale` winds the clock forward the same way the server's `--fast`
    does, so the tests can run a nine second move in nine milliseconds.
    """

    def __init__(
        self,
        sim: MotorSim | None = None,
        scale: float = 1.0,
        timeout: float = 0.05,
    ) -> None:
        if scale <= 0:
            raise ValueError("scale must be greater than zero")

        self.sim = sim if sim is not None else MotorSim()
        self.scale = scale
        self.timeout = timeout

        self._t0 = time.monotonic()
        self._out = bytearray()
        self._closed = False

        # Opening a port is what makes the Pico print its banner.
        self._queue(self.sim.banner())

    # --- the clock ---------------------------------------------------------

    def now_us(self) -> int:
        return int((time.monotonic() - self._t0) * 1_000_000 * self.scale)

    # --- what a Session needs ---------------------------------------------

    @property
    def in_waiting(self) -> int:
        self._pump()
        return len(self._out)

    def write(self, data: bytes) -> int:
        self._check_open()
        self._queue(self.sim.feed(bytes(data), self.now_us()))
        return len(data)

    def read(self, size: int = 1) -> bytes:
        """Up to `size` bytes, waiting up to `timeout` for some to appear.

        Returning empty on a quiet line is what a real port does, and it
        is how the caller gets a chance to check its own deadline.
        """
        self._check_open()
        deadline = time.monotonic() + self.timeout
        while True:
            self._pump()
            if self._out:
                taken = bytes(self._out[:size])
                del self._out[:size]
                return taken
            if time.monotonic() >= deadline:
                return b""
            # Nothing to do but let time pass. Short enough to stay
            # responsive, long enough not to spin the processor flat out.
            time.sleep(0.001)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self._closed = True

    def _check_open(self) -> None:
        if self._closed:
            raise OSError("the link is closed")

    def _pump(self) -> None:
        if not self._closed:
            self._queue(self.sim.poll(self.now_us()))

    def _queue(self, lines: list[str]) -> None:
        for line in lines:
            self._out += (line + "\n").encode("ascii", errors="replace")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="py -m gs.fake_motor_pico",
        description="A pretend motor Pico on a TCP port, for testing with "
        "no hardware.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"TCP port to listen on (default {DEFAULT_PORT}; "
        "0 picks a free one)",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="address to listen on (default 127.0.0.1, this machine only)",
    )
    parser.add_argument(
        "--fast",
        type=float,
        default=1.0,
        metavar="N",
        help="run the simulator's clock N times faster (default 1, real time)",
    )
    parser.add_argument(
        "--version-string",
        default="fake",
        metavar="TEXT",
        help="what PONG and the banner report (default 'fake')",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="do not print the lines being sent",
    )
    args = parser.parse_args(argv)

    if args.fast <= 0:
        print("--fast must be greater than zero")
        return 2

    sim = MotorSim(version=args.version_string)
    log = None if args.quiet else (lambda m: print(m, flush=True))

    try:
        pico = FakePico(
            host=args.host, port=args.port, scale=args.fast, sim=sim, log=log
        )
    except OSError as exc:
        print(f"Could not listen on {args.host}:{args.port}: {exc}")
        print("Another fake Pico may already be running.")
        return 2

    print(f"Fake Pico listening on {pico.url}")
    print(f"Connect with:  --port {pico.url}")
    if args.fast != 1.0:
        print(f"Clock running {args.fast}x - moves finish that much sooner.")
    print("Nothing is plugged in and nothing will turn. Ctrl-C to stop.")

    try:
        pico.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        pico.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
