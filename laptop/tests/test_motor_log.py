"""Tests for the results table.

Two halves: the table itself, built from records made up here, and then
the `true` command at the prompt driving it for real through a dry run.

The thing being protected is the deliverable. A row that cannot be tied
to its move, a reading that lands on the wrong line, or a file left half
written when the laptop is unplugged all lose data that cannot be taken
again afterwards - the antenna has moved on.
"""

from __future__ import annotations

import csv
import io

import pytest

from gs import motor
from gs import motor_log
from gs import protocol as p
from gs import session as sn

FAST = ["--fast", "50"]


def outcome(
    cmd_id=1,
    axis="AZ",
    steps=2286,
    from_mdeg=0,
    to_mdeg=45006,
    speed_mdps=5000,
    est_us=9_000_000,
    elapsed_us=8_996_045,
    ok=True,
    triangular=False,
):
    """A finished move, made up, with the real record types."""
    plan = p.Plan(
        id=cmd_id,
        axis=axis,
        steps=steps,
        from_mdeg=from_mdeg,
        to_mdeg=to_mdeg,
        speed_mdps=speed_mdps,
        triangular=triangular,
        est_us=est_us,
    )
    done = p.Done(
        id=cmd_id,
        axis=axis,
        ok=ok,
        steps_done=steps if ok else steps // 2,
        from_mdeg=from_mdeg,
        to_mdeg=to_mdeg,
        elapsed_us=elapsed_us,
    )
    return sn.MoveOutcome(plan, done)


def read_back(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


@pytest.fixture
def log(tmp_path):
    return motor_log.MoveLog(
        path=str(tmp_path / "results.csv"), pico="gs_motor abc"
    )


# ============================================================================
# Writing rows
# ============================================================================


def test_a_move_becomes_a_row_straight_away(log):
    # Written as it happens, not at the end: a session that ends with the
    # laptop being unplugged should still have everything up to then.
    log.add(outcome(), commanded_deg=45.0)

    rows = read_back(log.path)
    assert len(rows) == 1
    assert rows[0]["axis"] == "AZ"
    assert rows[0]["commanded_deg"] == "45.000"
    assert rows[0]["predicted_deg"] == "45.006"
    assert rows[0]["result"] == "OK"


def test_the_columns_are_the_ones_the_spreadsheet_expects(log):
    log.add(outcome())
    with open(log.path, newline="", encoding="utf-8") as handle:
        header = next(csv.reader(handle))
    assert header == motor_log.COLUMNS


def test_rows_are_numbered_from_one_and_keep_counting(log):
    for i in range(3):
        log.add(outcome(cmd_id=i + 1))
    assert [r["row"] for r in read_back(log.path)] == ["1", "2", "3"]


def test_which_pico_answered_is_recorded_on_every_row(log):
    # So a simulated run cannot quietly become a page of real test data.
    log.add(outcome())
    assert read_back(log.path)[0]["pico"] == "gs_motor abc"


def test_predicted_and_measured_times_both_go_in(log):
    # Side by side is the point: the pair is what says the step loop kept
    # real time rather than drifting.
    log.add(outcome(est_us=9_000_000, elapsed_us=8_996_045))
    row = read_back(log.path)[0]
    assert row["predicted_s"] == "9.000"
    assert row["measured_s"] == "8.996"


def test_a_stopped_move_is_kept_and_marked(log):
    # Dropping it would lose the reason a reading is missing.
    log.add(outcome(ok=False))
    assert read_back(log.path)[0]["result"] == "ABORT"


def test_a_short_move_records_that_it_never_reached_speed(log):
    log.add(outcome(triangular=True))
    assert read_back(log.path)[0]["shape"] == "TRI"


# ============================================================================
# The protractor reading
# ============================================================================


def test_the_reading_lands_on_the_last_move_by_default(log):
    log.add(outcome(cmd_id=1))
    log.add(outcome(cmd_id=2, from_mdeg=45006, to_mdeg=90011))

    log.set_true(90.2)

    rows = read_back(log.path)
    assert rows[0]["true_deg"] == ""  # the earlier one is untouched
    assert rows[1]["true_deg"] == "90.200"


def test_a_reading_can_be_put_on_an_earlier_row(log):
    # Readings do not always arrive in order - somebody looks up, reads
    # the wrong scale, and comes back to it.
    log.add(outcome(cmd_id=1))
    log.add(outcome(cmd_id=2))

    log.set_true(44.9, row=1)

    assert read_back(log.path)[0]["true_deg"] == "44.900"
    assert read_back(log.path)[1]["true_deg"] == ""


def test_the_error_is_measured_against_what_was_predicted(log):
    # Not against what was asked for. Asking for 45 degrees of azimuth
    # really commands 45.006, and measuring against 45.000 would fold a
    # known rounding into what is meant to measure the gearing.
    log.add(outcome(to_mdeg=45006), commanded_deg=45.0)
    log.set_true(45.1)

    row = read_back(log.path)[0]
    assert row["predicted_deg"] == "45.006"
    assert row["error_deg"] == "0.094"  # 45.100 - 45.006, not 45.100 - 45.000


def test_a_reading_the_wrong_side_of_the_prediction_reads_negative(log):
    log.add(outcome(to_mdeg=45006))
    log.set_true(44.9)
    assert read_back(log.path)[0]["error_deg"] == "-0.106"


def test_a_reading_with_nothing_to_attach_it_to_is_refused(log):
    with pytest.raises(motor_log.LogError, match="nothing has been moved"):
        log.set_true(45.0)


def test_a_reading_for_a_row_that_does_not_exist_is_refused(log):
    log.add(outcome())
    with pytest.raises(motor_log.LogError, match="no row 7"):
        log.set_true(45.0, row=7)


def test_a_reading_can_be_corrected(log):
    log.add(outcome())
    log.set_true(45.1)
    log.set_true(45.2)  # misread it the first time
    assert read_back(log.path)[0]["true_deg"] == "45.200"


# ============================================================================
# Notes and what is still missing
# ============================================================================


def test_a_note_can_be_put_against_a_row(log):
    log.add(outcome())
    log.annotate("belt slipped")
    assert read_back(log.path)[0]["note"] == "belt slipped"


def test_notes_build_up_rather_than_replacing_each_other(log):
    log.add(outcome())
    log.annotate("belt slipped")
    log.annotate("retightened")
    assert read_back(log.path)[0]["note"] == "belt slipped retightened"


def test_it_knows_which_rows_are_still_waiting_for_a_reading(log):
    log.add(outcome(cmd_id=1))
    log.add(outcome(cmd_id=2))
    log.set_true(45.0, row=1)

    assert [r.row for r in log.missing_readings()] == [2]


def test_a_stopped_move_is_not_counted_as_a_missing_reading(log):
    # Nobody is going to protractor a move that was cut short.
    log.add(outcome(ok=False))
    assert log.missing_readings() == []


# ============================================================================
# The file itself
# ============================================================================


def test_no_half_written_file_is_left_behind(log, tmp_path):
    log.add(outcome())
    log.set_true(45.0)
    leftovers = [f.name for f in tmp_path.iterdir() if f.suffix == ".part"]
    assert leftovers == []


def test_the_default_name_carries_the_date():
    from datetime import datetime

    name = motor_log.default_name(datetime(2026, 10, 17, 9, 5, 1))
    assert name == "motor_test_20261017_090501.csv"


# ============================================================================
# Through the prompt
# ============================================================================


def run(*commands, log_path=None):
    out = io.StringIO()
    argv = ["--dry-run", *FAST]
    if log_path is not None:
        argv += ["--log", str(log_path)]
    for text in commands:
        argv += ["-c", text]
    code = motor.main(argv, out=out)
    return code, out.getvalue()


def test_the_prompt_logs_a_move_and_takes_the_reading(tmp_path):
    path = tmp_path / "bench.csv"
    code, text = run(
        "zero all", "move az 5 45", "true 45.1", log_path=path
    )

    assert code == 0
    assert "row 1 - type 'true <degrees>'" in text
    assert "out by 0.094 deg" in text

    rows = read_back(path)
    assert rows[0]["true_deg"] == "45.100"
    assert rows[0]["pico"] == "gs_motor fake"  # not mistakable for real data


def test_the_file_exists_from_the_start(tmp_path):
    # So an operator can see it was created, rather than wondering until
    # the first move whether --log was spelled right.
    path = tmp_path / "bench.csv"
    run("pos", log_path=path)
    assert path.exists()
    assert read_back(path) == []


def test_it_says_which_rows_have_no_reading_when_the_session_ends(tmp_path):
    path = tmp_path / "bench.csv"
    _, text = run("zero all", "move az 90 10", log_path=path)
    assert "1 move(s) have no protractor reading: row 1" in text
    assert str(path) in text


def test_true_without_a_log_explains_what_to_do():
    _, text = run("zero all", "move az 90 10", "true 10.0")
    assert "no results file is open" in text
    assert "--log" in text


def test_rows_prints_the_table_so_far(tmp_path):
    path = tmp_path / "bench.csv"
    _, text = run(
        "zero all", "move az 90 10", "true 10.1", "move az 90 10", "rows",
        log_path=path,
    )
    assert "true   10.100" in text
    assert "still waiting on a reading for row 2" in text


def test_a_note_can_be_typed_at_the_prompt(tmp_path):
    path = tmp_path / "bench.csv"
    run("zero all", "move az 90 10", "note belt slipped", log_path=path)
    assert read_back(path)[0]["note"] == "belt slipped"


def test_a_note_can_name_its_row(tmp_path):
    path = tmp_path / "bench.csv"
    run(
        "zero all",
        "move az 90 10",
        "move az 90 10",
        "note 1 first one only",
        log_path=path,
    )
    rows = read_back(path)
    assert rows[0]["note"] == "first one only"
    assert rows[1]["note"] == ""


# ============================================================================
# Querying a reading that looks wrong
# ============================================================================


def test_a_reading_far_from_the_prediction_is_queried(tmp_path):
    # The protractor gives where the antenna is, not how far it just
    # moved. Those are the same number on the first move of a run and
    # nowhere near it afterwards, which makes it an easy mistake to make
    # and a silent one to live with.
    path = tmp_path / "bench.csv"
    _, text = run(
        "zero all",
        "move az 20 20",
        "true 20.1",
        "move az 20 20",
        "true 19.9",  # meant 40-ish: typed how far it moved
        log_path=path,
    )

    assert "that is a long way out" in text
    assert "not how far it just moved" in text
    # Still recorded: a surprising measurement is still a measurement.
    assert read_back(path)[1]["true_deg"] == "19.900"


def test_a_reading_close_to_the_prediction_is_not_queried(tmp_path):
    path = tmp_path / "bench.csv"
    _, text = run("zero all", "move az 20 20", "true 20.1", log_path=path)
    assert "a long way out" not in text
