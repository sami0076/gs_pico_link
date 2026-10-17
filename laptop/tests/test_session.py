"""Tests for the layer that actually talks to the Pico.

Most of these run against the simulator wired straight to the session,
with its clock wound forward a thousand times, so a nine second move takes
nine milliseconds and the test suite stays quick. One test at the bottom
goes the long way round over a real socket, to show the same code works on
the path the bench will use.

Where a test needs the Pico to say something strange - a reply that never
comes, an uptime that goes backwards - it uses `ScriptedLink`, which hands
back whatever lines the test decided on. The simulator cannot be made to
misbehave, which is the whole point of it, so faults have to be staged.
"""

from __future__ import annotations

import pytest

from gs import protocol as p
from gs import session as sn
from gs.fake_motor_pico import LoopbackLink
from gs.motor_sim import MotorSim

# The simulator's clock, this many times over. A 45 degree move at 5
# deg/sec is nine seconds of its time and nine milliseconds of ours.
FAST = 1000.0

# Slower, for the tests that have to do something while a move is still
# running. At this rate the same move takes about a third of a second,
# which is ages next to the microseconds a stop command needs.
SLOW = 30.0


class ScriptedLink:
    """A port that says exactly what the test tells it to.

    Records everything written, so a test can also check what went out and
    in what order.
    """

    def __init__(self) -> None:
        self.written: list[bytes] = []
        self._out = bytearray()
        self.closed = False

    # --- what the test drives it with --------------------------------------

    def feed(self, text: str) -> None:
        self._out += text.encode("ascii")

    def lines_written(self) -> list[str]:
        return b"".join(self.written).decode("ascii").splitlines()

    # --- what a Session asks of it -----------------------------------------

    @property
    def in_waiting(self) -> int:
        return len(self._out)

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def read(self, size: int = 1) -> bytes:
        taken = bytes(self._out[:size])
        del self._out[:size]
        return taken

    def close(self) -> None:
        self.closed = True


@pytest.fixture
def link():
    """The simulator, running fast, with no socket in the way."""
    return LoopbackLink(sim=MotorSim(), scale=FAST)


@pytest.fixture
def live(link):
    """A session on it, with the banner already read and thrown away."""
    s = sn.Session(link)
    s.drain(0.05)
    return s


@pytest.fixture
def ready(live):
    """...and both axes zeroed, so moves are allowed."""
    live.zero("ALL")
    return live


@pytest.fixture
def mid_move():
    """A zeroed session whose moves last long enough to interrupt."""
    s = sn.Session(LoopbackLink(sim=MotorSim(), scale=SLOW))
    s.drain(0.05)
    s.zero("ALL")
    return s


# ============================================================================
# The simple commands
# ============================================================================


def test_the_banner_arrives_as_chatter(live):
    text = " ".join(getattr(r, "text", "") for r in live.chatter)
    assert "gs_pico_link gs_motor" in text
    assert "SIMULATOR" in text


def test_ping_comes_back_with_the_id_that_was_sent(live):
    pong = live.ping()
    assert isinstance(pong, p.Pong)
    assert pong.version == "fake"
    # A second one gets a different id, so the two can never be confused.
    assert live.ping().id != pong.id


def test_it_reports_the_limits_the_pico_was_built_with(live):
    lim = live.limits()
    assert (lim.az_min_mdeg, lim.az_max_mdeg) == (0, 360000)
    assert (lim.el_min_mdeg, lim.el_max_mdeg) == (0, 45000)


def test_zero_then_position(live):
    assert live.position().zeroed is False
    live.zero("ALL")
    pos = live.position()
    assert pos.zeroed is True
    assert (pos.az_steps, pos.el_steps) == (0, 0)


def test_zero_can_say_what_angle_we_are_at(live):
    live.zero("AZ", 90.0)
    assert live.position().az_steps == 9143  # 90 degrees of azimuth


def test_setting_a_test_pin(live):
    assert live.set_pin(13, 1).level == 1


# ============================================================================
# Moving
# ============================================================================


def test_a_move_reports_both_what_was_planned_and_what_happened(ready):
    outcome = ready.move("AZ", 5.0, 45.0)

    assert isinstance(outcome, sn.MoveOutcome)
    assert outcome.ok is True
    assert outcome.plan.id == outcome.done.id  # the same move, tied together
    assert outcome.plan.steps == 4571
    assert outcome.done.steps_done == 4571
    assert outcome.done.to_mdeg == 44996  # quantised, not 45.000


def test_the_predicted_time_is_a_hair_over_the_measured_one(ready):
    # est counts the gap before the first pulse; elapsed is first pulse to
    # last. The difference should be one step interval - about 1968 us for
    # a 5 deg/sec azimuth move - and nothing like a second.
    outcome = ready.move("AZ", 5.0, 45.0)
    assert 0 < outcome.overshoot_us < 10_000


def test_the_position_follows_the_move(ready):
    ready.move("AZ", 90.0, 10.0)
    assert ready.position().az_steps == 1016


def test_a_refused_move_raises_with_the_reason_attached(ready):
    with pytest.raises(sn.Refused) as caught:
        ready.move("EL", 5.0, 60.0)  # past elevation's soft limit

    err = caught.value.err
    assert err.is_limit
    assert err.axis == "EL"
    assert "outside the soft limits" in str(caught.value)


def test_moving_before_zeroing_is_refused(live):
    with pytest.raises(sn.Refused) as caught:
        live.move("AZ", 5.0, 10.0)
    assert caught.value.err.code == "nozero"


def test_a_refusal_now_quotes_the_id_that_earned_it(live):
    # Since 7.9b the firmware keeps the id whenever it could read one, so
    # a mistyped axis can be matched to the command that caused it.
    with pytest.raises(sn.Refused) as caught:
        live.request("MOVE,5,ZZ,5000,1000", 5, (p.Ack,), what="a bad move")
    assert caught.value.err.code == "axis"
    assert caught.value.err.id == 5


def test_an_error_with_no_id_is_still_matched_to_the_command(live):
    # Some refusals genuinely have no id to quote: a line too long to have
    # been read at all is reported as ERR,0. Only one command is ever
    # outstanding, so there is nothing else it could be about, and taking
    # it as the answer is what stops the session hanging for a reply that
    # will never carry the right number.
    with pytest.raises(sn.Refused) as caught:
        live.request("WOBBLE,6", 6, (p.Ack,), what="a command it never heard of")
    assert caught.value.err.code == "verb"
    assert caught.value.err.id == 0


# ============================================================================
# Stopping
# ============================================================================


def test_a_stop_mid_move_ends_it_early_and_says_so(mid_move):
    plan = mid_move.begin_move("AZ", 5.0, 45.0)
    mid_move.stop()

    # A stop during a move is answered by the move's own DONE, because the
    # Pico prints nothing between two pulses.
    done = mid_move.await_done(plan)
    assert done.id == plan.id
    assert done.ok is False
    assert 0 < abs(done.steps_done) < plan.steps


def test_an_emergency_stop_means_the_next_move_is_refused(mid_move):
    plan = mid_move.begin_move("AZ", 5.0, 45.0)
    mid_move.estop()
    mid_move.await_done(plan)

    # The Pico gave up its zero, because after an abrupt halt it can no
    # longer claim to know where the gimbal is.
    with pytest.raises(sn.Refused) as caught:
        mid_move.move("AZ", 5.0, 1.0)
    assert caught.value.err.code == "nozero"

    # Re-zeroing is the cure, and then it moves again.
    mid_move.zero("ALL")
    assert mid_move.move("AZ", 5.0, 1.0).ok


def test_waiting_for_a_move_can_be_resumed_after_giving_up(mid_move):
    # A Ctrl-C at the prompt leaves the move running on the Pico, so the
    # DONE is still coming. Abandoning it would leave the next command
    # reading somebody else's answer.
    plan = mid_move.begin_move("AZ", 5.0, 45.0)

    with pytest.raises(sn.Timeout):
        mid_move.await_done(plan, timeout_s=0.001)

    done = mid_move.await_done(plan)  # picked up where it left off
    assert done.ok is True


# ============================================================================
# One command at a time
# ============================================================================


def test_nothing_is_sent_until_the_last_answer_is_in():
    # The firmware judges every line in one packet against the busy flag
    # read before any of them ran, so two commands sent together restart
    # the axis and the first DONE never arrives. This layer exists partly
    # to make that impossible.
    link = ScriptedLink()
    s = sn.Session(link, default_timeout_s=0.05)

    link.feed("PONG,1,gs_motor,fake,1000\n")
    s.ping()
    assert link.lines_written() == ["PING,1"]

    link.feed("PONG,2,gs_motor,fake,2000\n")
    s.ping()
    assert link.lines_written() == ["PING,1", "PING,2"]


def test_a_silent_pico_times_out_rather_than_hanging():
    link = ScriptedLink()  # answers nothing, ever
    s = sn.Session(link, default_timeout_s=0.05)

    with pytest.raises(sn.Timeout) as caught:
        s.ping()
    assert "did not answer" in str(caught.value)


# ============================================================================
# Lines arriving in awkward pieces
# ============================================================================


def test_a_reply_split_across_reads_is_put_back_together():
    link = ScriptedLink()
    s = sn.Session(link, default_timeout_s=0.5)

    # Written in pieces, the way USB really delivers it.
    link.feed("PON")
    link.feed("G,1,gs_mo")
    link.feed("tor,fake,1234\n")

    assert s.ping().uptime_ms == 1234


def test_two_replies_in_one_lump_are_read_as_two():
    link = ScriptedLink()
    s = sn.Session(link, default_timeout_s=0.5)

    link.feed("MSTAT,7,0,0,IDLE\nPONG,1,gs_motor,fake,7000\n")
    pong = s.ping()

    assert pong.uptime_ms == 7000
    # The heartbeat that came with it was kept, not thrown away.
    assert any(isinstance(r, p.Mstat) for r in s.chatter)


@pytest.mark.parametrize("ending", ["\n", "\r\n", "\r"])
def test_every_line_ending_is_understood(ending):
    link = ScriptedLink()
    s = sn.Session(link, default_timeout_s=0.5)
    link.feed(f"PONG,1,gs_motor,fake,10{ending}")
    assert s.ping().uptime_ms == 10


def test_a_carriage_return_in_the_middle_does_not_join_two_records():
    # Splitting on "\n" alone would read this as one unparseable line and
    # the PONG would be lost.
    link = ScriptedLink()
    s = sn.Session(link, default_timeout_s=0.5)
    link.feed("MSTAT,3,0,0,IDLE\rPONG,1,gs_motor,fake,3000\n")
    assert s.ping().uptime_ms == 3000


# ============================================================================
# Noticing a Pico that restarted
# ============================================================================


def test_an_uptime_that_goes_backwards_means_it_rebooted():
    link = ScriptedLink()
    s = sn.Session(link, default_timeout_s=0.5)

    link.feed("PONG,1,gs_motor,fake,30000\n")
    s.ping()
    assert s.rebooted is False

    link.feed("PONG,2,gs_motor,fake,400\n")
    s.ping()
    assert s.rebooted is True


def test_a_reboot_blocks_every_command_until_it_is_re_zeroed():
    # It has forgotten where the antenna is. Moving from an imaginary
    # starting point is exactly what the soft limits cannot save you from.
    link = ScriptedLink()
    s = sn.Session(link, default_timeout_s=0.5)

    link.feed("PONG,1,gs_motor,fake,30000\n")
    s.ping()
    link.feed("PONG,2,gs_motor,fake,400\n")
    s.ping()

    with pytest.raises(sn.Rebooted):
        s.position()

    link.feed("ACK,4,ZERO\n")
    s.zero("ALL")
    assert s.rebooted is False

    link.feed("POS,5,0,0,0,0,ZEROED\n")
    assert s.position().zeroed is True


def test_the_heartbeat_does_not_look_like_a_reboot():
    # PONG counts milliseconds and the heartbeat counts whole seconds.
    # Comparing one against the other directly would make every ordinary
    # heartbeat after a PONG look like the Pico had restarted.
    link = ScriptedLink()
    s = sn.Session(link, default_timeout_s=0.5)

    link.feed("PONG,1,gs_motor,fake,5400\n")  # 5.4 seconds up
    s.ping()
    link.feed("MSTAT,5,0,0,IDLE\n")  # the same moment, in seconds
    link.feed("PONG,2,gs_motor,fake,6000\n")
    s.ping()

    assert s.rebooted is False


# ============================================================================
# The same code, over a real socket
# ============================================================================


def test_it_all_works_through_pyserial_too():
    # Everything above runs the simulator in this process. This one goes
    # out through a socket and back in through pyserial, which is the path
    # the bench will use - proving the layer is not quietly relying on
    # being wired straight to the simulator.
    pytest.importorskip("serial", reason="pyserial is not installed")
    from gs.fake_motor_pico import FakePico

    server = FakePico(port=0, scale=200.0, sim=MotorSim())
    server.start()
    try:
        with sn.Session.open(server.url, timeout=0.2) as s:
            s.drain(0.3)
            assert s.ping().version == "fake"
            s.zero("ALL")
            outcome = s.move("AZ", 90.0, 10.0)
            assert outcome.ok
            assert s.position().az_steps == 1016
    finally:
        server.stop()
