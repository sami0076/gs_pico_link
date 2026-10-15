"""Tests for the laptop's half of the motor protocol.

Nothing here needs a Pico, or pyserial. The point of keeping the protocol in
one module of plain functions is that both directions can be checked at a
desk.

Where a test uses a literal line, it is copied from the matching C++ host
test (`firmware/host_tests/test_motor_cmd.cpp`, `test_motor_reply.cpp`)
rather than written fresh, so the two sides are compared against the same
text. The contract test in a later chunk automates that; this is the
hand-checked version of it.
"""

from __future__ import annotations

import pytest

from gs import protocol as p


# ============================================================================
# Degrees and millidegrees
# ============================================================================


@pytest.mark.parametrize(
    "deg, mdeg",
    [
        (45, 45000),
        (45.5, 45500),
        (-45.5, -45500),
        (0, 0),
        (-0.0, 0),
        (0.001, 1),
        (360, 360000),
        (0.145, 145),  # 0.145 * 1000 is 144.99999999999997 as a float
    ],
)
def test_degrees_to_millidegrees(deg, mdeg):
    assert p.deg_to_mdeg(deg) == mdeg


# Half away from zero, not Python's round-half-to-even. Both directions, so
# a sign change cannot quietly move the boundary.
def test_a_half_millidegree_rounds_away_from_zero():
    assert p.deg_to_mdeg(0.0005) == 1
    assert p.deg_to_mdeg(-0.0005) == -1
    assert p.deg_to_mdeg(0.0015) == 2  # round() would give 2 here, but
    assert p.deg_to_mdeg(0.0025) == 3  # ... 2 here, where we want 3


@pytest.mark.parametrize(
    "mdeg, text",
    [
        (45006, "45.006"),
        (0, "0.000"),
        (-45006, "-45.006"),
        (-500, "-0.500"),  # the sign survives when the whole part is zero
        (360000, "360.000"),
    ],
)
def test_format_mdeg(mdeg, text):
    assert p.format_mdeg(mdeg) == text


# ============================================================================
# Ids
# ============================================================================


def test_ids_count_up():
    ids = p.IdSource()
    assert [ids.next() for _ in range(3)] == [1, 2, 3]


# Zero is the firmware's own id for errors it raises unprompted, so ours
# must never collide with it.
def test_ids_wrap_past_the_top_without_using_zero():
    ids = p.IdSource(start=p.ID_MAX - 1)
    assert [ids.next() for _ in range(3)] == [p.ID_MAX - 1, p.ID_MAX, 1]


# ============================================================================
# Building commands
# ============================================================================


def test_move_lines():
    # These three are the vectors test_motor_cmd.cpp parses.
    assert p.move(7, "AZ", 5.0, 45.0) == "MOVE,7,AZ,5000,45000"
    assert p.move(8, "EL", 5.0, -45.0) == "MOVE,8,EL,5000,-45000"
    assert p.move(9, "AZ", 5.0, 0.5) == "MOVE,9,AZ,5000,500"


def test_the_simple_commands():
    assert p.stop(1) == "STOP,1"
    assert p.estop(2) == "ESTOP,2"
    assert p.pos(3) == "POS,3"
    assert p.limits(4) == "LIMITS,4"
    assert p.ping(5) == "PING,5"


def test_zero_with_and_without_an_angle():
    assert p.zero(10, "AZ") == "ZERO,10,AZ"
    assert p.zero(11, "AZ", 90.0) == "ZERO,11,AZ,90000"
    assert p.zero(12, "ALL") == "ZERO,12,ALL"
    assert p.zero(13, "EL", -0.5) == "ZERO,13,EL,-500"


# Zeroing at 0 is the common case and must still send the angle only when
# one was asked for - ZERO,1,AZ,0 and ZERO,1,AZ mean the same thing to the
# Pico, but only the second is what "no angle given" should look like.
def test_zero_at_zero_is_explicit_when_asked_for():
    assert p.zero(1, "AZ", 0) == "ZERO,1,AZ,0"
    assert p.zero(1, "AZ") == "ZERO,1,AZ"


def test_pin_lines():
    assert p.pin(20, 14, 1) == "PIN,20,14,1"
    assert p.pin(21, 14, 0) == "PIN,21,14,0"
    assert p.pin(22, 14, True) == "PIN,22,14,1"


def test_axis_may_be_typed_in_any_case():
    assert p.move(1, "az", 5.0, 1.0) == "MOVE,1,AZ,5000,1000"
    assert p.move(1, "El", 5.0, 1.0) == "MOVE,1,EL,5000,1000"
    assert p.zero(1, " all ") == "ZERO,1,ALL"


@pytest.mark.parametrize(
    "call",
    [
        lambda: p.move(1, "ALL", 5.0, 1.0),  # ALL is a ZERO-only axis
        lambda: p.move(1, "XX", 5.0, 1.0),
        lambda: p.move(1, "AZIMUTH", 5.0, 1.0),
        lambda: p.zero(1, "BOTH"),
    ],
)
def test_a_bad_axis_is_refused_before_it_is_sent(call):
    with pytest.raises(p.CommandError):
        call()


@pytest.mark.parametrize("bad_id", [0, -1, 65536, 1.5, "7", True])
def test_a_bad_id_is_refused(bad_id):
    with pytest.raises(p.CommandError):
        p.pos(bad_id)


def test_nonsense_speeds_and_angles_are_refused():
    with pytest.raises(p.CommandError):
        p.move(1, "AZ", -5.0, 1.0)
    with pytest.raises(p.CommandError):
        p.move(1, "AZ", 2000.0, 1.0)
    with pytest.raises(p.CommandError):
        p.move(1, "AZ", 5.0, 2000.0)
    with pytest.raises(p.CommandError):
        p.zero(1, "AZ", -2000.0)


# A speed the Pico will refuse is still sent. Whether 0.05 deg/sec is allowed
# is plan_move's call, made against the config that build was flashed with,
# and the whole point of ERR speed is to hear that answer from the Pico
# rather than guess it here.
def test_a_speed_the_pico_may_refuse_is_still_sent():
    assert p.move(1, "AZ", 0.05, 1.0) == "MOVE,1,AZ,50,1000"


def test_bad_gpio_is_refused():
    with pytest.raises(p.CommandError):
        p.pin(1, 30, 1)
    with pytest.raises(p.CommandError):
        p.pin(1, -1, 1)


# ============================================================================
# Putting a command on the wire
# ============================================================================


def test_encode_adds_the_newline():
    assert p.encode("POS,1") == b"POS,1\n"


def test_encode_refuses_a_line_that_is_already_more_than_one():
    with pytest.raises(p.CommandError):
        p.encode("POS,1\nPOS,2")


def test_encode_refuses_an_overlong_line():
    with pytest.raises(p.CommandError) as exc:
        p.encode("X" * (p.MAX_LINE_LEN + 1))
    assert str(p.MAX_LINE_LEN) in str(exc.value)


# Every command this module builds is far inside the Pico's buffer, including
# the longest one that can be built.
def test_the_longest_possible_command_still_fits():
    longest = p.move(p.ID_MAX, "AZ", 1000.0, -1000.0)
    assert len(longest) < p.MAX_LINE_LEN
    assert p.encode(longest).endswith(b"\n")


# ============================================================================
# Reading replies
# ============================================================================


def test_ack():
    assert p.parse_reply("ACK,7,MOVE\n") == p.Ack(id=7, verb="MOVE")
    assert p.parse_reply("ACK,65535,PING") == p.Ack(id=65535, verb="PING")


def test_plan():
    r = p.parse_reply("PLAN,7,AZ,2286,89992,134997,5000,TRAP,8999982\n")
    assert r == p.Plan(
        id=7,
        axis="AZ",
        steps=2286,
        from_mdeg=89992,
        to_mdeg=134997,
        speed_mdps=5000,
        triangular=False,
        est_us=8999982,
    )
    assert r.to_deg == pytest.approx(134.997)
    assert r.speed_dps == pytest.approx(5.0)
    assert r.est_s == pytest.approx(8.999982)


def test_plan_triangular():
    r = p.parse_reply("PLAN,1,EL,10,0,315,90000,TRI,12000")
    assert r.triangular is True


def test_done_ok_and_abort():
    assert p.parse_reply("DONE,7,AZ,OK,2286,89992,134997,9001234\n") == p.Done(
        id=7,
        axis="AZ",
        ok=True,
        steps_done=2286,
        from_mdeg=89992,
        to_mdeg=134997,
        elapsed_us=9001234,
    )
    aborted = p.parse_reply("DONE,8,EL,ABORT,-400,45000,32400,2500000\n")
    assert aborted.ok is False
    assert aborted.steps_done == -400
    assert aborted.elapsed_s == pytest.approx(2.5)


def test_pos():
    r = p.parse_reply("POS,3,89992,0,4571,0,ZEROED\n")
    assert r == p.Pos(
        id=3, az_mdeg=89992, el_mdeg=0, az_steps=4571, el_steps=0, zeroed=True
    )
    assert r.az_deg == pytest.approx(89.992)

    assert p.parse_reply("POS,4,0,0,0,0,UNZEROED\n").zeroed is False

    negative = p.parse_reply("POS,5,-45006,-10000,-2286,-317,ZEROED\n")
    assert negative.az_mdeg == -45006
    assert negative.el_steps == -317


def test_pin():
    assert p.parse_reply("PIN,20,14,1\n") == p.Pin(id=20, gpio=14, level=1)
    assert p.parse_reply("PIN,21,14,0\n").level == 0


def test_lim():
    assert p.parse_reply("LIM,4,0,360000,0,45000,100,90000\n") == p.Lim(
        id=4,
        az_min_mdeg=0,
        az_max_mdeg=360000,
        el_min_mdeg=0,
        el_max_mdeg=45000,
        speed_min_mdps=100,
        speed_max_mdps=90000,
    )


def test_pong():
    assert p.parse_reply("PONG,5,gs_motor,eb4ad64,12345\n") == p.Pong(
        id=5, target="gs_motor", version="eb4ad64", uptime_ms=12345
    )


def test_mstat():
    assert p.parse_reply("MSTAT,42,89992,0,IDLE\n") == p.Mstat(
        uptime_s=42, az_mdeg=89992, el_mdeg=0, moving=False
    )
    assert p.parse_reply("MSTAT,43,91000,0,MOVING\n").moving is True


def test_err():
    assert p.parse_reply("ERR,9,nozero,AZ\n") == p.Err(
        id=9, code="nozero", detail="AZ"
    )
    assert p.parse_reply("ERR,10,busy,EL\n").code == "busy"


# The detail is empty whenever there was nothing useful to quote back, which
# leaves a trailing comma. That is a well-formed line, not a truncated one.
def test_err_with_no_detail():
    r = p.parse_reply("ERR,0,syntax,\n")
    assert r == p.Err(id=0, code="syntax", detail="")
    assert r.is_limit is False


def test_err_limit_carries_numbers_instead_of_a_detail():
    r = p.parse_reply("ERR,11,limit,EL,50000,0,45000\n")
    assert r == p.Err(
        id=11,
        code="limit",
        axis="EL",
        target_mdeg=50000,
        min_mdeg=0,
        max_mdeg=45000,
    )
    assert r.is_limit is True
    assert "50.000" in r.message()
    assert "45.000" in r.message()


def test_err_limit_below_zero():
    r = p.parse_reply("ERR,12,limit,AZ,-1000,0,360000\n")
    assert r.target_mdeg == -1000
    assert "-1.000" in r.message()


# Every code the firmware can send has something to say to the operator, so
# message() never falls back to the bare code.
@pytest.mark.parametrize("code", sorted(p.ERR_MEANING))
def test_every_error_code_has_a_message(code):
    assert p.Err(id=1, code=code).message() == p.ERR_MEANING[code]


def test_an_unknown_error_code_still_reads_sensibly():
    assert "wobble" in p.Err(id=1, code="wobble").message()


# ============================================================================
# Lines that are not records
# ============================================================================


@pytest.mark.parametrize(
    "line",
    [
        "=== gs_pico_link gs_motor eb4ad64 ===",
        "[motor] ready az=50.793 st/deg el=31.746 st/deg",
        "[motor] NOT ZEROED - send ZERO before any MOVE",
        "[fatal] azimuth pins did not read back idle HIGH",
        "RX,-42.0,9.75,12,hello",  # the radio build's record, not ours
    ],
)
def test_the_picos_own_chatter_comes_through_as_info(line):
    r = p.parse_reply(line)
    assert r == p.Info(line)
    assert r.malformed is False


def test_a_blank_line_is_harmless():
    assert p.parse_reply("\n") == p.Info("")
    assert p.parse_reply("") == p.Info("")


# A line that starts like a record but does not hold up is still shown, so
# nothing is silently dropped, but it is flagged. Dropping these would hide
# exactly the kind of fault worth noticing - a corrupted DONE means a move
# whose outcome is unknown.
@pytest.mark.parametrize(
    "line",
    [
        "DONE,7,AZ,OK,2286,89992,134997",  # one field short
        "DONE,7,AZ,MAYBE,2286,89992,134997,9001234",  # not OK or ABORT
        "ACK,notanumber,MOVE",
        "ACK,1_0,MOVE",  # int() would read this as 10
        "ACK, 7,MOVE",  # the firmware never pads; a space means corruption
        "POS,3,89992,0,4571,0,MAYBE",
        "PLAN,7,XX,2286,89992,134997,5000,TRAP,8999982",
        "MSTAT,42,89992,0",
        "ERR,9",
        "ERR,9,limit,EL,50000,0",  # a limit error cut short
        "LIM,4,0,360000,0,45000,100",
    ],
)
def test_a_broken_record_is_flagged_but_not_lost(line):
    r = p.parse_reply(line)
    assert isinstance(r, p.Info)
    assert r.malformed is True
    assert r.text == line


def test_parsing_never_raises_on_anything():
    # Whatever arrives - a half-flashed Pico, line noise, a partial line at
    # startup - reading it must not take down a program driving a motor.
    for line in ["", ",", ",,,,,,,,", "\x00\xff", "ACK", "ACK,", "A" * 500]:
        assert isinstance(p.parse_reply(line), p.Reply)


def test_a_trailing_carriage_return_is_stripped():
    assert p.parse_reply("ACK,7,MOVE\r\n") == p.Ack(id=7, verb="MOVE")
    assert p.parse_reply("[motor] hi\r\n") == p.Info("[motor] hi")


# ============================================================================
# Both directions together
# ============================================================================


# The id is the only thing tying a reply to the command that caused it, so
# the round trip through a line of text has to preserve it exactly.
def test_an_id_survives_the_round_trip():
    ids = p.IdSource(start=65530)
    for _ in range(6):
        sent = ids.next()
        line = p.move(sent, "AZ", 5.0, 45.0)
        assert p.parse_reply(f"ACK,{sent},MOVE").id == sent
        assert line.startswith(f"MOVE,{sent},")
