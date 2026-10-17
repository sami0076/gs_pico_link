"""Tests for the fake Pico served over a socket.

`test_motor_sim.py` checks the behaviour by calling it directly. These tests
go the long way round instead: they open the simulator with
`ports.open_port()`, the same function the real CLI will use on the real
Pico, and talk to it through pyserial. That is the part worth proving -
a simulator the code can only reach through a special "pretend" path would
tell us nothing about the path that matters.

They need pyserial, and they are skipped without it.

Everything runs with the simulator's clock sped up, so a move that would
take nine seconds on the bench finishes here in a fraction of one. Reads
are given a generous deadline because these tests share a machine with
whatever else is running on it, not because anything is expected to be slow.
"""

from __future__ import annotations

import socket
import time

import pytest

from gs import protocol as p
from gs.fake_motor_pico import FakePico
from gs.motor_sim import MotorSim

pytest.importorskip("serial", reason="pyserial is not installed")

from gs import ports  # noqa: E402  (imported after the skip on purpose)

# How long any single read may wait before the test gives up. Generous: a
# missed line should fail the test, not hang the suite.
DEADLINE_S = 10.0

# The simulator's clock, wound forward this many times. A 45 degree move at
# 5 deg/sec takes nine seconds of its time and under a tenth of ours.
FAST = 200.0


@pytest.fixture
def pico():
    """A fake Pico on a free port, stopped again afterwards.

    Port 0 lets the machine pick, so these tests cannot collide with a fake
    Pico someone left running on 7777.
    """
    server = FakePico(port=0, scale=FAST, sim=MotorSim())
    server.start()
    try:
        yield server
    finally:
        server.stop()


@pytest.fixture
def link(pico):
    """An open connection to it, through the same code the CLI will use."""
    conn = ports.open_port(pico.url, timeout=0.2)
    try:
        yield conn
    finally:
        conn.close()


def read_records(conn, want, deadline_s=DEADLINE_S):
    """Read until `want` says we have enough, or give up.

    Returns every record read, parsed the way the CLI parses them. The
    banner and any heartbeat come through here too, which is realistic:
    a reader has to cope with lines it did not ask for.
    """
    # Bytes are glued back into lines here rather than leaning on
    # readline(), which hands back whatever it has when its timeout runs
    # out - half a line included. Under load that splits a record in two
    # and both halves parse as nonsense. The real reader does this job in
    # session.py; this is the same thing, kept local so these tests stay
    # about the server rather than about the session layer.
    seen = []
    buffer = ""
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        raw = conn.read(conn.in_waiting or 1)
        if not raw:
            continue
        buffer += raw.decode("ascii", errors="replace")
        while "\n" in buffer:
            line, _, buffer = buffer.partition("\n")
            if line.strip():
                seen.append(p.parse_reply(line))
        if want(seen):
            return seen
    raise AssertionError(
        "gave up waiting. Records so far:\n  "
        + "\n  ".join(repr(r) for r in seen)
    )


def send(conn, line):
    conn.write(p.encode(line))


def read_until(conn, kind, deadline_s=DEADLINE_S):
    """Read records until one of `kind` turns up. Returns all of them."""
    return read_records(
        conn, lambda seen: any(isinstance(r, kind) for r in seen), deadline_s
    )


def expect(conn, kind):
    """Read until a record of `kind` arrives, and hand back that one."""
    return first(read_until(conn, kind), kind)


def first(records, kind):
    for r in records:
        if isinstance(r, kind):
            return r
    raise AssertionError(f"no {kind.__name__} among {records!r}")


# ============================================================================
# Connecting
# ============================================================================


def test_the_url_is_one_pyserial_understands(pico):
    assert pico.url.startswith("socket://127.0.0.1:")
    assert pico.port != 0  # the machine chose a real one


def test_the_banner_is_sent_as_soon_as_a_client_connects(pico):
    """Read with a plain socket, deliberately not through pyserial.

    Opening a serial port throws away whatever is already waiting -
    pyserial does it on open, to clear stale bytes left in a real port -
    so a banner that arrives in the moment between connecting and
    finishing the open is discarded. That is a property of opening a
    port and not something the server can do anything about, so the
    server's side of it is checked here with nothing in the way.

    It is also why the prompt says what it is connected to from a PING
    rather than from the banner.
    """
    with socket.create_connection((pico.host, pico.port), timeout=5) as raw:
        raw.settimeout(5)
        received = b""
        while b"SIMULATOR" not in received:
            chunk = raw.recv(4096)
            if not chunk:
                break
            received += chunk

    text = received.decode("ascii", errors="replace")
    assert "gs_pico_link gs_motor" in text
    assert "NOT ZEROED" in text
    # Every record is byte-for-byte what the real firmware sends, so this
    # line is the only thing marking a simulated run as simulated.
    assert "SIMULATOR" in text


def test_it_answers_a_ping(link):
    send(link, p.ping(1))
    pong = expect(link, p.Pong)
    assert pong.id == 1
    assert pong.version == "fake"


def test_it_reports_the_limits_it_was_built_with(link):
    send(link, p.limits(2))
    lim = expect(link, p.Lim)
    assert (lim.az_min_mdeg, lim.az_max_mdeg) == (0, 360000)
    assert (lim.el_min_mdeg, lim.el_max_mdeg) == (0, 45000)
    assert (lim.speed_min_mdps, lim.speed_max_mdps) == (100, 90000)


# ============================================================================
# A move, start to finish, over the wire
# ============================================================================


def test_a_whole_move_arrives_in_order(link):
    send(link, p.zero(1, "ALL"))
    send(link, p.move(2, "AZ", 5.0, 45.0))

    seen = read_until(link, p.Done)

    plan = first(seen, p.Plan)
    done = first(seen, p.Done)

    # The order matters: ACK, then PLAN, then the move, then DONE.
    kinds = [type(r).__name__ for r in seen if not isinstance(r, p.Info)]
    assert kinds.index("Ack") < kinds.index("Plan") < kinds.index("Done")

    assert plan.steps == 2286
    assert plan.to_mdeg == 45006  # the quantised angle, not 45.000
    assert done.id == 2
    assert done.ok is True
    assert done.steps_done == 2286


def test_the_position_sticks_after_the_move(link):
    send(link, p.zero(1, "ALL"))
    send(link, p.move(2, "AZ", 90.0, 10.0))
    read_until(link, p.Done)

    send(link, p.pos(3))
    pos = expect(link, p.Pos)
    assert pos.az_steps == 508  # 10 degrees of azimuth
    assert pos.zeroed is True


def test_a_refused_move_comes_back_as_an_error(link):
    send(link, p.zero(1, "ALL"))
    send(link, p.move(2, "EL", 5.0, 60.0))  # past elevation's soft limit

    err = expect(link, p.Err)
    assert err.is_limit
    assert err.axis == "EL"
    assert "outside the soft limits" in err.message()


# ============================================================================
# Behaving like a port
# ============================================================================


def test_closing_and_reopening_does_not_lose_the_position(pico):
    conn = ports.open_port(pico.url, timeout=0.2)
    send(conn, p.zero(1, "ALL"))
    send(conn, p.move(2, "AZ", 90.0, 10.0))
    read_until(conn, p.Done)
    conn.close()

    # A real Pico keeps running when the laptop closes the port: it has not
    # rebooted, and it still knows where it is. Anything else here would
    # train the CLI to expect a reset that will not happen on the bench.
    again = ports.open_port(pico.url, timeout=0.2)
    try:
        send(again, p.pos(3))
        pos = expect(again, p.Pos)
        assert pos.az_steps == 508
        assert pos.zeroed is True
    finally:
        again.close()


def test_the_heartbeat_keeps_arriving_when_nothing_is_happening(link):
    # Five seconds of simulator time, which at this speed is almost no time
    # at all. Two of them, so we see it repeat rather than fire once.
    seen = read_records(
        link,
        lambda s: len([r for r in s if isinstance(r, p.Mstat)]) >= 2,
    )
    beats = [r for r in seen if isinstance(r, p.Mstat)]
    assert beats[1].uptime_s > beats[0].uptime_s
    assert all(b.moving is False for b in beats)


def test_a_bad_line_is_refused_without_dropping_the_connection(link):
    link.write(b"WOBBLE,1\n")
    err = expect(link, p.Err)
    assert err.code == "verb"

    # Still usable afterwards, which is the actual point of the test.
    send(link, p.ping(2))
    pong = expect(link, p.Pong)
    assert pong.id == 2


def test_the_server_refuses_a_clock_that_does_not_move():
    with pytest.raises(ValueError):
        FakePico(port=0, scale=0)
