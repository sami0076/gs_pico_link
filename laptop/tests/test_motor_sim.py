"""Tests for the pretend Pico.

Two kinds of check live here.

**Does it match the firmware?** Where a line or a number appears below as a
literal, it is copied from the matching C++ host test
(`firmware/host_tests/test_motion_profile.cpp`, `test_motor_reply.cpp`)
rather than from running this code and writing down what came out. A
simulator checked against itself proves nothing. The automatic version of
this is the contract test in a later chunk; this is the hand-checked one.

**Does it fit the rest of the laptop code?** Several tests send commands
built by `protocol.py` and read the answers back with `protocol.parse_reply`,
so the two halves are proved to fit rather than assumed to.

The clock is a number we pass in, so a thirty-six second move takes no time
at all to test.
"""

from __future__ import annotations

import subprocess
import sys

import pytest

from gs import motor_sim as sim
from gs import protocol as p

# One second, in the microseconds the simulator counts in.
SEC = 1_000_000


def lines(out):
    """Replies as plain strings, for comparing against literals."""
    return list(out)


def records(out):
    """Replies run through the real parser, which is how the CLI sees them."""
    return [p.parse_reply(line) for line in out]


def zeroed_sim(**kwargs):
    """A simulator with both axes zeroed at 0, ready to move."""
    s = sim.MotorSim(**kwargs)
    s.feed(p.encode(p.zero(1, "ALL")), 0)
    return s


# ============================================================================
# Steps and angles - the literals come from test_motion_profile.cpp
# ============================================================================


@pytest.mark.parametrize(
    "mdeg, steps",
    [
        (45000, 2286),  # 45 deg = 2285.71, the procedure's first command
        (-45000, -2286),  # symmetric, which is why rounding goes away from 0
        (0, 0),
        (360000, 18286),
    ],
)
def test_azimuth_degrees_to_steps(mdeg, steps):
    assert sim.mdeg_to_steps(sim.AZ_SPEC, mdeg) == steps


@pytest.mark.parametrize(
    "steps, mdeg",
    [
        (2286, 45006),  # 2286 steps is 45.006 deg, not 45.000
        (-2286, -45006),
        (4571, 89992),  # "I am at 90 degrees" is really 89.992
    ],
)
def test_azimuth_steps_to_degrees(steps, mdeg):
    assert sim.steps_to_mdeg(sim.AZ_SPEC, steps) == mdeg


def test_steps_and_degrees_do_not_round_trip():
    # Deliberate: a step is a fixed size and most angles do not land on one.
    # The difference is the honest answer and belongs in the test data.
    there = sim.mdeg_to_steps(sim.AZ_SPEC, 45000)
    assert sim.steps_to_mdeg(sim.AZ_SPEC, there) == 45006


# ============================================================================
# The PLAN line, against the exact text the C++ test asserts
# ============================================================================


def test_plan_line_matches_the_firmware_character_for_character():
    # From test_motor_reply.cpp:
    #     "PLAN,7,AZ,2286,89992,134997,5000,TRAP,8999982\n"
    # Every field has to agree, est_us included - that last number is the
    # one that would drift if the speed maths here were done in Python's
    # wider floats instead of being held to the firmware's 32-bit ones.
    s = sim.MotorSim()
    s.feed(p.encode(p.zero(1, "AZ", 89.992)), 0)

    out = s.feed(p.encode(p.move(7, "AZ", 5.0, 45.0)), 0)

    assert out == [
        "ACK,7,MOVE",
        "PLAN,7,AZ,2286,89992,134997,5000,TRAP,8999982",
    ]


def test_a_move_at_the_starting_speed_has_no_ramp():
    # 5 deg/sec is where moves begin, so a move at that speed is flat out
    # from the first step: no ramp, and 3937 us between every pulse.
    _, m = sim.plan_move(sim.AZ_SPEC, 0, 45000, 5000)
    assert m.n_ramp == 0
    assert m.triangular is False
    assert sim.interval_us(m, 0) == pytest.approx(3937, abs=2)
    assert sim.interval_us(m, m.n_steps - 1) == pytest.approx(3937, abs=2)
    assert m.est_us == pytest.approx(9_000_000, abs=20_000)


def test_a_fast_move_speeds_up_and_slows_down_symmetrically():
    _, m = sim.plan_move(sim.AZ_SPEC, 0, 180000, 90000)
    assert m.n_ramp > 0
    # First and last steps run at the starting speed; the middle is fastest.
    assert sim.interval_us(m, 0) == pytest.approx(3937, abs=2)
    assert sim.interval_us(m, m.n_steps - 1) == pytest.approx(3937, abs=2)
    assert sim.interval_us(m, m.n_steps // 2) == pytest.approx(218, abs=2)
    # The ramp up and the ramp down are mirror images.
    for i in range(m.n_ramp):
        assert sim.interval_us(m, i) == sim.interval_us(m, m.n_steps - 1 - i)


def test_a_short_fast_move_never_reaches_the_speed_asked_for():
    # Too short to finish speeding up before it has to slow down, so the
    # trapezium collapses to a triangle. Reported as TRI so an odd row in
    # the test data explains itself.
    _, m = sim.plan_move(sim.AZ_SPEC, 0, 2000, 90000)
    assert m.triangular is True
    assert m.n_ramp <= m.n_steps // 2


def test_no_interval_ever_reaches_a_sanity_rail():
    # Hitting a rail would mean the maths went wrong, so nothing the
    # firmware allows should come close to one.
    for spec in (sim.AZ_SPEC, sim.EL_SPEC):
        for speed in (sim.SPEED_MIN_MDPS, 5000, 45000, sim.SPEED_MAX_MDPS):
            result, m = sim.plan_move(spec, 0, spec.max_mdeg, speed)
            assert result is sim.PlanResult.OK
            for i in (0, m.n_steps // 2, m.n_steps - 1):
                gap = sim.interval_us(m, i)
                assert sim.MIN_INTERVAL_US < gap < sim.MAX_INTERVAL_US


# ============================================================================
# Refusals
# ============================================================================


def test_a_move_before_zeroing_is_refused():
    # The soft limits are measured from the zero, so without one they are no
    # protection at all. This is the main thing between a mistyped command
    # and the gimbal's hard stops.
    s = sim.MotorSim()
    out = s.feed(p.encode(p.move(4, "AZ", 5.0, 10.0)), 0)
    assert out == ["ERR,4,nozero,AZ"]

    err = records(out)[0]
    assert isinstance(err, p.Err)
    assert "ZERO first" in err.message()


def test_a_move_past_the_soft_limit_is_refused_whole():
    # Never shortened to fit: a clamped move would put a different angle on
    # the gimbal than the one written down.
    s = zeroed_sim()
    out = s.feed(p.encode(p.move(5, "EL", 5.0, 50.0)), 0)

    err = records(out)[0]
    assert isinstance(err, p.Err)
    assert err.is_limit
    assert err.axis == "EL"
    assert (err.min_mdeg, err.max_mdeg) == (0, 45000)
    # Nothing moved.
    assert s.el.pos_steps == 0


def test_an_angle_too_small_to_make_a_step_is_refused():
    # Otherwise the operator writes a number in the data table for a move
    # that never happened.
    s = zeroed_sim()
    out = s.feed(p.encode(p.move(6, "AZ", 5.0, 0.001)), 0)
    assert out == ["ERR,6,zerosteps,AZ"]


@pytest.mark.parametrize("speed_dps", [0.05, 95.0])
def test_a_speed_outside_the_allowed_range_is_refused(speed_dps):
    s = zeroed_sim()
    out = s.feed(p.encode(p.move(7, "AZ", speed_dps, 10.0)), 0)
    assert out == ["ERR,7,speed,AZ"]


def test_a_spare_pin_can_be_set_and_costs_nothing():
    s = zeroed_sim()
    assert s.feed(p.encode(p.pin(8, 14, 1)), 0) == ["PIN,8,14,1"]
    assert s.pins == {14: 1}
    assert s.az.zeroed and s.el.zeroed


def test_holding_a_step_line_gives_up_that_axis_zero():
    # Being able to hold a motor pin steady is the only way to check that
    # wiring with a meter, since a step pulse is five microseconds long.
    # But driving PUL low makes the driver take a step nobody counted, so
    # the position afterwards is one step away from the truth.
    s = zeroed_sim()
    assert s.feed(p.encode(p.pin(9, 4, 0)), 0) == ["PIN,9,4,0"]
    assert s.az.zeroed is False
    assert s.el.zeroed is True

    # ...and the next move on that axis is refused until it is re-zeroed.
    assert s.feed(p.encode(p.move(10, "AZ", 5.0, 1.0)), 0) == [
        "ERR,10,nozero,AZ"
    ]
    assert s.feed(p.encode(p.move(11, "EL", 5.0, 1.0)), 0)[0] == "ACK,11,MOVE"


def test_holding_a_direction_line_keeps_the_zero():
    # Nothing moves, and the direction is set again at the start of every
    # move, so this one costs nothing.
    s = zeroed_sim()
    assert s.feed(p.encode(p.pin(12, 5, 1)), 0) == ["PIN,12,5,1"]
    assert s.az.zeroed is True


def test_a_pin_belonging_to_something_else_is_refused():
    # The radio and the GPS are not ours to poke.
    s = sim.MotorSim()
    assert s.feed(p.encode(p.pin(13, 25, 1)), 0) == [
        "ERR,13,gpio,GP4-7 or GP12-15"
    ]
    assert s.pins == {}


def test_a_line_too_long_is_thrown_away_and_reported_once():
    s = sim.MotorSim()
    out = s.feed(b"PING," + b"9" * 200 + b"\n", 0)
    # Id 0, because the line was discarded and there is no id to quote back.
    assert out == ["ERR,0,toolong,"]
    # And the next command still works, rather than the parser staying stuck.
    assert s.feed(p.encode(p.ping(2)), 0)[0].startswith("PONG,2,")


@pytest.mark.parametrize(
    "line, code",
    [
        ("WOBBLE,1", "verb"),
        ("MOVE,1,AZ,5000", "syntax"),
        ("MOVE,1,ZZ,5000,1000", "axis"),
        ("MOVE,1,AZ,5000,45.5", "syntax"),  # decimals are refused, not rounded
        ("PIN,1,99,1", "gpio"),
    ],
)
def test_lines_the_pico_cannot_read(line, code):
    s = sim.MotorSim()
    out = s.feed((line + "\n").encode(), 0)
    assert len(out) == 1
    err = p.parse_reply(out[0])
    assert isinstance(err, p.Err)
    assert err.code == code


def test_a_refusal_quotes_the_id_that_earned_it():
    # Until 7.9b the firmware reported every parse fault on a blank
    # command, so a mistyped axis came back as ERR,0 and nothing on the
    # laptop could tell which command had been turned down.
    s = sim.MotorSim()
    assert s.feed(b"MOVE,42,ZZ,5000,1000\n", 0) == ["ERR,42,axis,"]
    assert s.feed(b"PIN,43,99,1\n", 0) == ["ERR,43,gpio,"]


def test_a_line_with_no_readable_id_still_reports_zero():
    # There is nothing honest to put there: an unknown verb, or an id that
    # is not a number, leaves no id to quote back.
    s = sim.MotorSim()
    assert s.feed(b"WOBBLE,1\n", 0) == ["ERR,0,verb,"]
    assert s.feed(b"PING,x\n", 0) == ["ERR,0,syntax,"]


# ============================================================================
# Running a move
# ============================================================================


def test_a_move_finishes_at_the_time_the_plan_predicted():
    s = zeroed_sim()
    out = s.feed(p.encode(p.move(2, "AZ", 5.0, 45.0)), 0)
    plan = records(out)[1]
    assert isinstance(plan, p.Plan)

    # Still going one second in, and a command sent now is refused.
    assert s.feed(p.encode(p.pos(3)), 1 * SEC) == ["ERR,3,busy,"]

    # Nothing is reported mid-move, not even a heartbeat.
    assert s.poll(5 * SEC) == []

    done = records(s.poll(10 * SEC))[0]
    assert isinstance(done, p.Done)
    assert done.id == 2
    assert done.ok is True
    assert done.steps_done == plan.steps
    assert done.to_mdeg == plan.to_mdeg


def test_the_move_takes_one_interval_less_than_the_plan_said():
    # est_us adds up every gap, including the one before the first pulse;
    # elapsed_us is measured first pulse to last. So DONE always comes in
    # one interval under PLAN. The firmware does the same, and the bench
    # notes compare these two numbers, so the gap is worth knowing about.
    s = zeroed_sim()
    out = s.feed(p.encode(p.move(2, "AZ", 5.0, 45.0)), 0)
    plan = records(out)[1]

    done = records(s.poll(60 * SEC))[0]
    first_gap = 3937
    assert plan.est_us - done.elapsed_us == pytest.approx(first_gap, abs=2)


def test_position_is_reported_in_steps_as_well_as_degrees():
    s = zeroed_sim()
    s.feed(p.encode(p.move(2, "AZ", 90.0, 45.0)), 0)
    s.poll(60 * SEC)

    pos = records(s.feed(p.encode(p.pos(3)), 60 * SEC))[0]
    assert isinstance(pos, p.Pos)
    assert pos.az_steps == 2286
    assert pos.az_mdeg == 45006  # the quantised angle, not 45000
    assert pos.zeroed is True


def test_a_move_the_other_way_comes_back_to_where_it_started():
    s = zeroed_sim()
    s.feed(p.encode(p.move(2, "AZ", 90.0, 45.0)), 0)
    s.poll(60 * SEC)
    s.feed(p.encode(p.move(3, "AZ", 90.0, -45.0)), 60 * SEC)
    s.poll(120 * SEC)

    assert s.az.pos_steps == 0


def test_stop_ramps_down_and_reports_an_abort():
    s = zeroed_sim()
    s.feed(p.encode(p.move(2, "AZ", 5.0, 45.0)), 0)

    # A stop mid-move is answered by the DONE that follows, not by an ACK,
    # because nothing may be printed between the first and last pulse.
    assert s.feed(p.encode(p.stop(3)), 2 * SEC) == []

    done = records(s.poll(60 * SEC))[0]
    assert isinstance(done, p.Done)
    assert done.id == 2  # the move's id, not the stop's
    assert done.ok is False
    assert 0 < abs(done.steps_done) < 2286
    # The axis is still trusted: a controlled stop keeps the count honest.
    assert s.az.zeroed is True


def test_an_emergency_stop_gives_up_the_zero():
    # Pulses stop with no ramp down, so a closed-loop drive settles near but
    # not exactly where the count says - and there is no feedback to find
    # out. Refusing to move again until re-zeroed is the honest response.
    s = zeroed_sim()
    s.feed(p.encode(p.move(2, "AZ", 5.0, 45.0)), 0)
    s.feed(p.encode(p.estop(3)), 2 * SEC)

    done = records(s.poll(2 * SEC))[0]
    assert isinstance(done, p.Done)
    assert done.ok is False
    assert s.az.zeroed is False

    # And the next move is refused until a new zero arrives.
    assert s.feed(p.encode(p.move(4, "AZ", 5.0, 1.0)), 3 * SEC) == [
        "ERR,4,nozero,AZ"
    ]


def test_stop_with_nothing_moving_is_still_answered():
    # Otherwise the laptop waits for a reply that never comes.
    s = zeroed_sim()
    assert s.feed(p.encode(p.stop(5)), 0) == ["ACK,5,STOP"]
    assert s.feed(p.encode(p.estop(6)), 0) == ["ACK,6,ESTOP"]


def test_both_axes_report_only_once_everything_has_stopped():
    # The firmware checks for finished moves when the last axis stops, so a
    # short move running alongside a long one waits for the long one before
    # its DONE appears.
    s = zeroed_sim()
    s.feed(p.encode(p.move(2, "AZ", 5.0, 45.0)), 0)  # ~9 s
    # The second MOVE has to go in its own batch, or the stale busy flag
    # below would apply.
    s.poll(1 * SEC)
    out = s.feed(p.encode(p.move(3, "EL", 5.0, 1.0)), 1 * SEC)
    assert out == ["ERR,3,busy,"]  # one axis moving blocks the other


def test_two_commands_in_one_packet_are_judged_one_at_a_time():
    # Until 7.9b the firmware worked out whether it was busy once per batch
    # of received bytes, before reading them, so both commands in one
    # packet were judged as though neither had run: a second MOVE restarted
    # the axis and the first move's DONE was never sent. The state is now
    # read fresh for each line.
    s = zeroed_sim()
    out = s.feed(
        p.encode(p.move(2, "AZ", 5.0, 45.0))
        + p.encode(p.move(3, "AZ", 5.0, 45.0)),
        0,
    )

    assert [r.id for r in records(out) if isinstance(r, p.Ack)] == [2]
    refused = [r for r in records(out) if isinstance(r, p.Err)]
    assert [(r.id, r.code) for r in refused] == [(3, "busy")]

    # And the first move still reports what it did.
    done = [r for r in records(s.poll(60 * SEC)) if isinstance(r, p.Done)]
    assert [d.id for d in done] == [2]


# ============================================================================
# The heartbeat
# ============================================================================


def test_the_heartbeat_arrives_every_five_seconds():
    s = sim.MotorSim()
    assert s.poll(4 * SEC) == []

    beat = records(s.poll(5 * SEC))[0]
    assert isinstance(beat, p.Mstat)
    assert beat.uptime_s == 5
    assert beat.moving is False

    assert s.poll(9 * SEC) == []
    assert records(s.poll(10 * SEC))[0].uptime_s == 10


def test_the_heartbeat_stays_quiet_during_a_move():
    # Writing to USB blocks when the host stops reading, and a block between
    # two pulses would stall the step loop with the motor part way round.
    s = zeroed_sim()
    s.feed(p.encode(p.move(2, "AZ", 5.0, 45.0)), 0)

    for t in range(1, 9):
        assert s.poll(t * SEC) == [], f"something was printed at {t} s"


def test_the_heartbeat_stays_quiet_mid_way_through_a_typed_line():
    # It would not corrupt the command, which is read on another path, but
    # it scrambles the screen and makes a typed line hard to check before
    # pressing enter.
    s = sim.MotorSim()
    s.feed(b"PI", 1 * SEC)  # half a command, no newline yet
    assert s.poll(20 * SEC) == []

    # Finishing the line lets the heartbeat come back, a full period after
    # the last moment the line was incomplete - not a full period after the
    # last time anyone happened to look.
    s.feed(b"NG,3\n", 21 * SEC)
    assert s.poll(25 * SEC) == []
    assert records(s.poll(26 * SEC))[0].uptime_s == 26


# ============================================================================
# Bytes on the wire
# ============================================================================


def test_a_command_split_across_packets_is_still_read():
    # USB hands over whatever has arrived, split wherever it likes.
    s = sim.MotorSim()
    assert s.feed(b"PI", 0) == []
    assert s.feed(b"NG,", 0) == []
    assert s.feed(b"7", 0) == []
    out = s.feed(b"\n", 0)
    assert out[0].startswith("PONG,7,")


@pytest.mark.parametrize("ending", [b"\n", b"\r\n", b"\r"])
def test_every_line_ending_works(ending):
    s = sim.MotorSim()
    out = s.feed(b"PING,1" + ending, 0)
    assert len(out) == 1
    assert out[0].startswith("PONG,1,")


def test_blank_lines_are_ignored():
    s = sim.MotorSim()
    assert s.feed(b"\n\n\r\n", 0) == []


def test_rubbish_bytes_do_not_bring_it_down():
    # A noisy cable should produce a refusal, not a crash.
    s = sim.MotorSim()
    out = s.feed(bytes([0x00, 0xFF, 0x80]) + b"\n", 0)
    assert len(out) == 1
    assert p.parse_reply(out[0]).code == "verb"


# ============================================================================
# Asking it about itself
# ============================================================================


def test_the_limits_it_reports_are_the_ones_it_enforces():
    # From test_motor_reply.cpp: "LIM,4,0,360000,0,45000,100,90000\n"
    s = sim.MotorSim()
    assert s.feed(p.encode(p.limits(4)), 0) == [
        "LIM,4,0,360000,0,45000,100,90000"
    ]


def test_ping_says_it_is_a_fake():
    # Every record is byte-for-byte what the real firmware sends, so this is
    # the only thing standing between a simulated run and a page of test
    # data nobody can trust.
    s = sim.MotorSim()
    pong = records(s.feed(p.encode(p.ping(5)), 3 * SEC))[0]
    assert isinstance(pong, p.Pong)
    assert pong.target == "gs_motor"
    assert pong.version == "fake"
    assert pong.uptime_ms == 3000


def test_uptime_only_goes_forwards():
    # The laptop watches this to spot a Pico that rebooted and lost its
    # position. A simulator whose clock jumped backwards would look like a
    # reboot that never happened.
    s = sim.MotorSim()
    first = records(s.feed(p.encode(p.ping(1)), 1 * SEC))[0]
    second = records(s.feed(p.encode(p.ping(2)), 9 * SEC))[0]
    assert second.uptime_ms > first.uptime_ms


def test_the_banner_reads_as_information_not_records():
    # The Pico prints these to a human on the same wire as the records, and
    # the parser has to pass them through without choking.
    s = sim.MotorSim()
    banner = records(s.banner())
    assert all(isinstance(r, p.Info) for r in banner)
    assert not any(r.malformed for r in banner)
    assert "SIMULATOR" in " ".join(r.text for r in banner)


def test_the_banner_quotes_the_same_numbers_the_firmware_prints():
    text = "\n".join(sim.MotorSim().banner())
    assert "az=50.794 st/deg el=31.746 st/deg" in text
    assert "limits az 0.000..360.000  el 0.000..45.000" in text
    assert "NOT ZEROED" in text


# ============================================================================
# Everything it can say is something the laptop can read
# ============================================================================


def test_no_reply_is_ever_malformed():
    # Drive it through a whole session and check that every single line
    # comes back as a proper record. A field in the wrong place here would
    # show up in the CLI as unreadable chatter.
    s = sim.MotorSim()
    script = [
        p.zero(1, "ALL"),
        p.limits(2),
        p.ping(3),
        p.pos(4),
        p.pin(5, 13, 1),
        p.move(6, "AZ", 90.0, 10.0),
        p.move(7, "EL", 5.0, 500.0),  # refused: past the soft limit
        p.stop(8),
    ]
    seen = []
    now = 0
    for line in script:
        seen += s.feed(p.encode(line), now)
        now += 30 * SEC
        seen += s.poll(now)

    assert seen, "the session produced no replies at all"
    for line in seen:
        record = p.parse_reply(line)
        assert not isinstance(record, p.Info), f"not a record: {line!r}"


def test_the_fake_pico_runs_without_pyserial():
    # Plain standard library on purpose. Only the side that opens a port
    # needs pyserial, so the simulator can be started on a machine that has
    # never had it installed - including one with no serial support at all.
    code = "import gs.fake_motor_pico, sys; print('serial' in sys.modules)"
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert out.stdout.strip() == "False"
