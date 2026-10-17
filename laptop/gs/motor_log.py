r"""The results table, written as the test happens.

The deliverable from the motor test is a table: what was commanded, what
the Pico predicted, what it reported, and what the protractor actually
read. Three of those four the laptop already knows the moment a move
finishes. The fourth is a human squinting at a scale, which arrives some
seconds later and has to find its way onto the right row.

So a row is written as soon as the move ends, with the protractor column
left empty, and `set_true()` fills it in afterwards. At the prompt that is
the `true` command.

Why the whole file is rewritten every time
------------------------------------------

Appending would be the obvious thing, but a reading that arrives after the
row was written would then have nowhere to go. A test run is a few hundred
rows at most, so the file is rebuilt from the rows held in memory each
time anything changes. It goes to a temporary file first and is then moved
into place, so an interrupted write cannot leave half a table behind - at
a bench the laptop is as likely to be unplugged as anything else.

The error column
----------------

`error_deg` is the protractor reading minus the angle the Pico predicted,
not minus the angle that was asked for. Those differ by the quantisation:
asking for 45 degrees of azimuth really commands 45.006, and measuring the
mechanism against a number it was never given would fold a known rounding
into what is meant to be a measurement of the gearing.
"""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass, field
from datetime import datetime

from gs import protocol as p

# The columns, in the order they appear in the file. Changing this order
# changes every file written afterwards, so leave it alone unless the
# spreadsheet on the other end is changing too.
COLUMNS = [
    "row",
    "when",
    "pico",
    "id",
    "axis",
    "speed_dps",
    "commanded_deg",
    "from_deg",
    "predicted_deg",
    "true_deg",
    "error_deg",
    "steps",
    "steps_done",
    "shape",
    "predicted_s",
    "measured_s",
    "result",
    "note",
]


def default_name(when: datetime | None = None) -> str:
    """A filename nobody has to think about: motor_test_<date>_<time>.csv."""
    stamp = (when or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return f"motor_test_{stamp}.csv"


@dataclass
class Row:
    """One move, and the reading that belongs to it."""

    row: int
    when: str
    pico: str
    id: int
    axis: str
    speed_mdps: int
    commanded_mdeg: int | None
    from_mdeg: int
    predicted_mdeg: int
    steps: int
    steps_done: int
    shape: str
    predicted_us: int
    measured_us: int
    result: str
    true_mdeg: int | None = None
    note: str = ""

    @property
    def error_mdeg(self) -> int | None:
        """How far the antenna really ended up from where it was predicted."""
        if self.true_mdeg is None:
            return None
        return self.true_mdeg - self.predicted_mdeg

    def as_record(self) -> dict:
        """The row as the strings that go in the file.

        Angles are formatted from the whole millidegrees rather than from
        a float, so the text is exactly the number the Pico used.
        """
        return {
            "row": self.row,
            "when": self.when,
            "pico": self.pico,
            "id": self.id,
            "axis": self.axis,
            "speed_dps": p.format_mdeg(self.speed_mdps),
            "commanded_deg": (
                "" if self.commanded_mdeg is None
                else p.format_mdeg(self.commanded_mdeg)
            ),
            "from_deg": p.format_mdeg(self.from_mdeg),
            "predicted_deg": p.format_mdeg(self.predicted_mdeg),
            "true_deg": (
                "" if self.true_mdeg is None else p.format_mdeg(self.true_mdeg)
            ),
            "error_deg": (
                "" if self.error_mdeg is None
                else p.format_mdeg(self.error_mdeg)
            ),
            "steps": self.steps,
            "steps_done": self.steps_done,
            "shape": self.shape,
            "predicted_s": f"{self.predicted_us / 1e6:.3f}",
            "measured_s": f"{self.measured_us / 1e6:.3f}",
            "result": self.result,
            "note": self.note,
        }

    def summary(self) -> str:
        """One line, for showing at the prompt."""
        true = (
            "   -   " if self.true_mdeg is None
            else f"{p.format_mdeg(self.true_mdeg):>8}"
        )
        error = (
            "" if self.error_mdeg is None
            else f"  err {p.format_mdeg(self.error_mdeg)}"
        )
        return (
            f"  {self.row:>3}  {self.axis}  "
            f"{p.format_mdeg(self.from_mdeg):>9} ->"
            f"{p.format_mdeg(self.predicted_mdeg):>9} deg"
            f"  true {true}{error}  {self.result}"
        )


class LogError(Exception):
    """Something was asked of the log that it cannot do."""


@dataclass
class MoveLog:
    """Every move of a session, and the file they are written to."""

    path: str
    pico: str = ""
    rows: list[Row] = field(default_factory=list)

    def add(self, outcome, commanded_deg: float | None = None,
            note: str = "") -> Row:
        """Record a finished move. Writes the file before returning."""
        plan, done = outcome.plan, outcome.done
        row = Row(
            row=len(self.rows) + 1,
            when=datetime.now().isoformat(timespec="seconds"),
            pico=self.pico,
            id=done.id,
            axis=done.axis,
            speed_mdps=plan.speed_mdps,
            commanded_mdeg=(
                None if commanded_deg is None else p.deg_to_mdeg(commanded_deg)
            ),
            from_mdeg=plan.from_mdeg,
            predicted_mdeg=done.to_mdeg,
            steps=plan.steps,
            steps_done=done.steps_done,
            shape="TRI" if plan.triangular else "TRAP",
            predicted_us=plan.est_us,
            measured_us=done.elapsed_us,
            # A stopped move did not go where it was predicted to, so the
            # row is kept but marked - leaving it out would lose the
            # reason a reading is missing.
            result="OK" if done.ok else "ABORT",
            note=note,
        )
        self.rows.append(row)
        self.write()
        return row

    def set_true(self, deg: float, row: int | None = None) -> Row:
        """Fill in what the protractor said. Defaults to the last move."""
        if not self.rows:
            raise LogError("nothing has been moved yet, so there is no row")

        if row is None:
            target = self.rows[-1]
        else:
            matching = [r for r in self.rows if r.row == row]
            if not matching:
                raise LogError(
                    f"there is no row {row}. Rows go 1..{len(self.rows)}"
                )
            target = matching[0]

        target.true_mdeg = p.deg_to_mdeg(deg)
        self.write()
        return target

    def annotate(self, note: str, row: int | None = None) -> Row:
        """Put a note against a row. Defaults to the last one."""
        if not self.rows:
            raise LogError("nothing has been moved yet, so there is no row")
        target = self.rows[-1] if row is None else self._find(row)
        target.note = (target.note + " " + note).strip()
        self.write()
        return target

    def _find(self, row: int) -> Row:
        for r in self.rows:
            if r.row == row:
                return r
        raise LogError(f"there is no row {row}. Rows go 1..{len(self.rows)}")

    def missing_readings(self) -> list[Row]:
        """Successful moves with no protractor reading yet.

        Worth saying out loud before anyone packs up: a blank here is a
        measurement nobody can go back and take afterwards.
        """
        return [
            r for r in self.rows
            if r.true_mdeg is None and r.result == "OK"
        ]

    def write(self) -> None:
        """Rewrite the whole file, through a temporary, in one go."""
        temporary = self.path + ".part"
        with open(temporary, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row.as_record())
        os.replace(temporary, self.path)
