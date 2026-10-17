"""Tests for the two scripted runs.

They go through `main()` with `--dry-run`, the same as the prompt tests,
because a scripted move and a typed one are meant to be the same thing
and testing them through different doors would stop proving that.

The results file is read back rather than the printed output wherever
the question is about data, because the file is the deliverable.
"""

from __future__ import annotations

import csv
import io

import pytest

from gs import motor

FAST = ["--fast", "300"]


def run(*commands, log_path=None, stdin=None):
    out = io.StringIO()
    argv = ["--dry-run", *FAST]
    if log_path is not None:
        argv += ["--log", str(log_path)]
    for text in commands:
        argv += ["-c", text]
    code = motor.main(argv, out=out, stdin=stdin)
    return code, out.getvalue()


def read_back(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


# ============================================================================
# sweep
# ============================================================================


def test_sweep_does_one_move_at_each_speed(tmp_path):
    path = tmp_path / "sweep.csv"
    run("zero all", "sweep angle=10 speeds=1,5,20", log_path=path)

    rows = read_back(path)
    assert [r["speed_dps"] for r in rows] == ["1.000", "5.000", "20.000"]
    assert all(r["result"] == "OK" for r in rows)


def test_sweep_notes_which_run_each_row_came_from(tmp_path):
    # Otherwise a sweep row and a hand-typed row look identical in the
    # table, and the speed column alone does not say which was which.
    path = tmp_path / "sweep.csv"
    run("zero all", "sweep angle=10 speeds=5", log_path=path)
    assert read_back(path)[0]["note"] == "sweep 5.0"


def test_sweep_turns_back_when_it_runs_out_of_room(tmp_path):
    # Elevation only has 45 degrees. A sweep of four 20 degree moves has
    # to come back on itself rather than stopping at the top.
    path = tmp_path / "sweep.csv"
    run("zero all", "sweep axis=el angle=20 speeds=1,2,5,10", log_path=path)

    rows = read_back(path)
    assert len(rows) == 4
    # Every endpoint stayed inside the soft limits.
    for row in rows:
        assert 0.0 <= float(row["predicted_deg"]) <= 45.0
    # And it did move both ways to manage it.
    directions = {float(r["commanded_deg"]) > 0 for r in rows}
    assert directions == {True, False}


def test_sweep_skips_speeds_the_pico_will_not_take(tmp_path):
    # Rather than firing them off and collecting refusals.
    path = tmp_path / "sweep.csv"
    _, text = run("zero all", "sweep angle=10 speeds=5,500", log_path=path)

    assert "skipping 500.0 deg/s" in text
    assert len(read_back(path)) == 1


def test_sweep_stops_when_there_is_no_room_either_way():
    # 40 degree moves on an axis with 45 degrees of travel, starting in
    # the middle. Said plainly rather than collecting refusals.
    _, text = run("zero el 20", "sweep axis=el angle=40 speeds=5")
    assert "no room for 40.0 deg either way" in text


def test_sweep_uses_a_sensible_set_of_speeds_by_default(tmp_path):
    path = tmp_path / "sweep.csv"
    run("zero all", "sweep angle=5", log_path=path)
    speeds = [float(r["speed_dps"]) for r in read_back(path)]
    assert speeds == list(motor.motor_runs.DEFAULT_SWEEP_SPEEDS)


# ============================================================================
# random
# ============================================================================


def test_random_does_the_number_of_moves_asked_for(tmp_path):
    path = tmp_path / "random.csv"
    run("zero all", "random count=4 speed=20 seed=1", log_path=path)
    assert len(read_back(path)) == 4


def test_random_stays_inside_the_soft_limits(tmp_path):
    # The whole point of asking the Pico for its limits rather than
    # keeping our own copy.
    path = tmp_path / "random.csv"
    run("zero all", "random axis=el count=6 speed=45 seed=3", log_path=path)

    for row in read_back(path):
        assert 0.0 <= float(row["predicted_deg"]) <= 45.0


def test_the_same_seed_gives_the_same_run(tmp_path):
    # So a run that found something odd can be done again exactly.
    first = tmp_path / "a.csv"
    second = tmp_path / "b.csv"
    run("zero all", "random count=4 speed=45 seed=99", log_path=first)
    run("zero all", "random count=4 speed=45 seed=99", log_path=second)

    def angles(path):
        return [r["predicted_deg"] for r in read_back(path)]

    assert angles(first) == angles(second)


def test_the_seed_is_said_out_loud_even_when_it_was_not_given():
    # A run nobody can repeat is worth less, so the seed goes in the
    # transcript whether it was chosen here or asked for.
    _, text = run("zero all", "random count=1 speed=45")
    assert "seed " in text


def test_random_never_asks_for_a_move_too_small_to_happen(tmp_path):
    # An angle that rounds to no steps is refused by the Pico, which
    # would end the run on nothing.
    path = tmp_path / "random.csv"
    run("zero all", "random count=8 speed=90 seed=5", log_path=path)

    rows = read_back(path)
    assert len(rows) == 8
    assert all(int(r["steps"]) != 0 for r in rows)


# ============================================================================
# Pausing for the protractor
# ============================================================================


def test_a_paused_run_asks_for_each_reading_and_files_it(tmp_path):
    path = tmp_path / "paused.csv"
    typed = io.StringIO("10.1\n20.4\n")

    _, text = run(
        "zero all",
        "sweep angle=10 speeds=5,20 pause",
        log_path=path,
        stdin=typed,
    )

    rows = read_back(path)
    assert [r["true_deg"] for r in rows] == ["10.100", "20.400"]
    # Measured against the predicted angle, not the one asked for.
    assert rows[0]["error_deg"] == "0.099"
    assert "protractor reading" in text


def test_an_empty_answer_leaves_the_row_blank(tmp_path):
    # Better than guessing at a number nobody read.
    path = tmp_path / "paused.csv"
    typed = io.StringIO("\n10.2\n")

    run("zero all", "sweep angle=10 speeds=5,20 pause", log_path=path,
        stdin=typed)

    rows = read_back(path)
    assert rows[0]["true_deg"] == ""
    assert rows[1]["true_deg"] == "10.200"


def test_an_answer_that_is_not_a_number_is_not_filed(tmp_path):
    path = tmp_path / "paused.csv"
    typed = io.StringIO("about ten\n")

    _, text = run("zero all", "sweep angle=10 speeds=5 pause", log_path=path,
                  stdin=typed)

    assert "is not a number - row left blank" in text
    assert read_back(path)[0]["true_deg"] == ""


def test_q_at_the_prompt_stops_the_run(tmp_path):
    path = tmp_path / "paused.csv"
    typed = io.StringIO("10.1\nq\n")

    run("zero all", "sweep angle=10 speeds=5,20,45 pause", log_path=path,
        stdin=typed)

    # Two moves happened; the third never started.
    assert len(read_back(path)) == 2


def test_a_paused_run_does_not_ask_twice_for_the_same_number(tmp_path):
    path = tmp_path / "paused.csv"
    typed = io.StringIO("10.1\n")
    _, text = run("zero all", "sweep angle=10 speeds=5 pause", log_path=path,
                  stdin=typed)
    assert "type 'true <degrees>'" not in text


# ============================================================================
# Typing it wrong
# ============================================================================


@pytest.mark.parametrize(
    "typed, expected",
    [
        ("sweep wobble", "settings look like name=value"),
        ("sweep nonsense=3", "no setting called 'nonsense'"),
        ("random count=0", "count must be at least 1"),
        ("sweep axis=up", "axis must be az or el"),
        ("random speed=quick", "speed must be a number"),
    ],
)
def test_bad_settings_get_a_hint(typed, expected):
    code, text = run(typed)
    assert code == 0
    assert expected in text


def test_a_scripted_run_needs_a_zero_like_anything_else():
    _, text = run("sweep angle=10 speeds=5")
    assert "send ZERO first" in text
