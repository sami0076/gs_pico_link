# Bench bring-up: the motor test

This is the sheet to work through on the day the gimbal is wired up and
powered for the first time. Everything in this repo has been built and
tested with no hardware attached, so nothing below is *known* to work —
that is what the day is for.

Work top to bottom. The order is a safety order: each step only powers up
what the step before it proved.

> **The one rule.** Until the very last section, nothing on this page needs
> the gimbal to be able to move far. If an axis starts turning and you did
> not ask it to, **cut the motor power supply** — not the USB, not the
> program. The Pico cannot switch the drivers off; the big supply is the
> only off switch there is.

---

## Part 1 — What all the files are

You do not need to read any code to run this test. This is here so that
when something goes wrong you know which thing to look at.

### The things you run

| File | In plain words |
|---|---|
| `laptop/gs/motor.py` | **The program you actually use.** It gives you a `motor>` prompt where you type `move`, `zero` and so on. Everything else on this list is something it uses. |
| `laptop/gs/fake_motor_pico.py` | **A pretend Pico.** Lets you practise the whole procedure on a laptop with nothing plugged in. |
| `laptop/gs/ports.py` | **Finds the Pico.** Works out which COM port it is on, and gives a readable complaint when it cannot. |

### The things they use

| File | In plain words |
|---|---|
| `laptop/gs/session.py` | **Holds the conversation.** Sends one command at a time, waits for the reply, matches each reply to the command that asked for it, and notices if the Pico restarts. |
| `laptop/gs/protocol.py` | **The dictionary.** Turns `move az 5 45` into the exact line of text the Pico understands, and turns the Pico's replies back into something printable. |
| `laptop/gs/motor_sim.py` | **The pretend Pico's brain.** A copy of the firmware's behaviour, written in Python, so a practice run behaves like the real thing. |
| `laptop/gs/motor_log.py` | **The results file.** Writes a row of the spreadsheet every time a move finishes. |
| `laptop/gs/motor_runs.py` | **The scripted runs** — `sweep` and `random`, so you are not typing forty commands by hand. |

### What runs on the Pico

| File | In plain words |
|---|---|
| `firmware/src/motor_main.cpp` | **The program on the Pico.** Reads commands, pulses the motors, reports back. |
| `firmware/src/motor_commands.cpp` | **The rules.** Decides whether a command is allowed and what to answer. Kept separate so it can be tested on a laptop. |
| `firmware/src/motion_profile.cpp` | **The maths.** Turns an angle into a number of steps, and works out the gap between pulses so the motor speeds up and slows down smoothly. |
| `firmware/src/stepper.cpp` | **The pin wiggling.** Sends the actual pulses. |
| `firmware/src/motor_config.hpp` | **The numbers.** Steps per degree, how far each axis may travel, how fast it may go. The file to edit after the bench. |
| `firmware/src/config.hpp` | **The wiring.** Which Pico pin goes to which driver wire. |
| `gs_motor.uf2` (repo root) | **The file you copy onto the Pico** to flash it. |

### The documents

| File | In plain words |
|---|---|
| `docs/HARDWARE_BRINGUP.md` | This page. |
| `docs/datasheets/CL57T_V4.0_driver_manual.pdf` | The driver box's own manual. The maker's site blocks downloads, so keep this copy. |
| `GS_PICO_LINK_SPEC.md` | The specification for the whole ground station. |
| `laptop/tests/data/motor_vectors.txt` | A record of exactly what the firmware produces, used to prove the laptop and the Pico agree. Generated, not written by hand. |

---

## Part 2 — Every command

### Commands you type in a terminal

Run these from the `laptop` folder unless it says otherwise.

| Command | What it does |
|---|---|
| `py -m pip install -e laptop` | Installs the tools. Run once per machine, from the folder *above* `laptop`. |
| `py -m gs.motor` | **Starts the prompt**, finding the Pico by itself. |
| `py -m gs.motor --port COM5` | Same, but you say which port. Needed if more than one Pico is plugged in. |
| `py -m gs.motor --dry-run` | Same, but talks to the pretend Pico. Nothing is plugged in and nothing turns. |
| `py -m gs.motor --dry-run --fast 50` | A practice run with the clock sped up 50 times, so a nine-second move takes a fifth of a second. Only works with `--dry-run`. |
| `py -m gs.motor --log bench.csv` | **Writes the results file as you go.** Use this for the real test. |
| `py -m gs.motor --log` | Same, but names the file after today's date and time. |
| `py -m gs.motor --list` | Lists the serial ports it can see, then stops. |
| `py -m gs.motor --verbose` | Also shows the heartbeat line the Pico sends every five seconds. |
| `py -m gs.motor -c "pos"` | Runs one command and exits. Can be repeated. Useful in scripts. |
| `py -m gs.ports` | Lists serial ports and says which one looks like a Pico. |
| `py -m gs.fake_motor_pico` | Runs the pretend Pico as a server, so another program can connect to it. |
| `py -m pytest -q` | **Runs all the laptop tests.** Should say `727 passed`. |

In WSL, for the code that runs on the Pico:

| Command | What it does |
|---|---|
| `cmake -S firmware -B ~/build/gs -G Ninja` | Sets up the Pico build. Once per machine. |
| `ninja -C ~/build/gs` | **Builds the Pico program.** Produces `gs_motor.uf2`. |
| `cmake -S firmware/host_tests -B ~/build/gs-host -G Ninja` | Sets up the laptop-side tests of the Pico code. |
| `ninja -C ~/build/gs-host` | Builds those tests. |
| `ctest --test-dir ~/build/gs-host --output-on-failure` | **Runs them.** Should say `100% tests passed`. |
| `~/build/gs-host/gen_motor_vectors laptop/tests/data/motor_vectors.txt` | Rewrites the record of what the firmware does. Only needed after changing the protocol. |

### Commands you type at the `motor>` prompt

This is the full list. Type `help` to see a short version of it.

**Moving**

| Command | What it does |
|---|---|
| `move az 5 45` | Turn **azimuth** by **45 degrees** at **5 degrees per second**. |
| `move el 5 -10` | Turn **elevation back** by 10 degrees. A minus sign means the other way. |
| | The angle is *how far to turn*, not *where to go*. `move az 5 45` twice ends up 90 degrees round. |
| `stop` | Slow down and stop. The axis ramps down rather than halting dead, so it does not lose its place. |
| `estop` | **Stop now.** No slowing down. The Pico gives up its zero afterwards, because a motor stopped dead may have slipped. Re-zero before moving again. |
| Ctrl-C during a move | Same as `stop`, then it waits for the axis to come to rest. Press it twice to stop waiting — but then the position on screen is a guess. |

**Telling it where it is**

| Command | What it does |
|---|---|
| `zero` | "Both axes are at 0 degrees right now." **Nothing moves.** |
| `zero 90` | "Both axes are at 90 degrees right now." |
| `zero az` | Azimuth only, at 0. |
| `zero az 90` | Azimuth only, at 90. |
| `pos` | Where does it think it is? Shows degrees and steps for both axes, and whether it has been zeroed. |

A `move` is **refused** until the axis has been zeroed. That is deliberate:
there are no limit switches on this machine, so the travel limits are
measured from the zero, and without one they protect nothing.

**Checking and asking**

| Command | What it does |
|---|---|
| `ping` | Is it there? Says which program it is running and how long it has been powered. |
| `limits` | How far each axis may travel and how fast, as built into the Pico. |
| `pin 13 1` | Set pin GP13 high. For checking wiring with a multimeter. |
| `pin 4 0` | Set the azimuth **step** wire low. **This costs that axis its zero** — see Part 4. |
| `verbose on` | Show the heartbeat line. `verbose off` hides it again. |
| `help` | The list of commands. |
| `quit` | Leave. (`exit` works too.) |

**The scripted runs**

| Command | What it does |
|---|---|
| `sweep` | The same angle at each of several speeds, turning back and forth so it does not run out of room. Defaults to 45 degrees at 1, 2, 5, 10, 20, 45 and 90 degrees per second. |
| `sweep angle=20 speeds=1,5,20` | The same, your numbers. |
| `sweep axis=el angle=10` | On elevation instead. |
| `sweep ... pause` | **Stop after each move and ask you for the protractor reading.** This is the one to use for real. |
| `random` | Ten moves to unpredictable angles inside the limits, at 5 degrees per second. |
| `random count=5 speed=20 seed=7` | Your numbers. The same `seed` gives the same run again. |
| `random ... pause` | Stops for a reading after each move, as above. |

At a `pause` prompt: type the angle, or press enter to skip it, or type `q`
to end the run.

**Recording what you measured**

| Command | What it does |
|---|---|
| `true 45.1` | "The protractor says 45.1 degrees." Fills in the **last** move's row. |
| `true 3 45.1` | Fills in row 3 instead. |
| `note belt slipped` | Writes a note against the last row. |
| `note 3 belt slipped` | Against row 3. |
| `rows` | Shows the table so far, and which rows still have no reading. |

> **Type the angle the antenna is sitting at now**, not how far it just
> moved. Those are the same number on the first move and nowhere near it
> after that. If a reading is more than 5 degrees from what was predicted,
> the program says so and offers to let you retype it.

---

## Part 3 — Before any power: check the driver boxes

Three switches on each CL57T/CL57TE box. The code assumes all three. If any
is wrong, every number you write down today is wrong too, and nothing in
the software can tell.

**1. The row of eight small switches — pulses per turn.**
The code is built for **1600 pulses per motor turn**, which the CL57T
manual's table calls microstep 8:

```
switch 1: ON     switch 2: off    switch 3: ON     switch 4: ON
```

That is how the boxes on this machine are set, checked by eye at the bench
on 17 Oct 2026, and `motor_config.hpp` follows the hardware rather than the
other way round.

> **This is the one number no software can check.** The Pico counts the
> pulses it sends out and believes them. If a driver is swapped or reset to
> 800 (switch 1 off, switch 2 on), every angle comes out at exactly double
> what you asked for, the Pico reports the angle it meant to turn, and
> nothing anywhere will contradict it. Check these four switches before
> trusting any number on this page.

Count the gear teeth too, if you have not already: azimuth should be a 10:1
planetary gearbox and a 32:14 gear pair, elevation 20:14. If a gear has
been swapped, every angle is wrong by the same factor.

**2. Switch 6 in that same row — open or closed loop.**
**Off** means closed loop: the motor's encoder is being used, and the
driver corrects itself. **On** means open loop and the encoder is ignored.
You want **off**.

**3. The single switch on top — 5 V or 24 V signals.**
The board feeds 5 V signals. **From the factory this switch is set to
24 V**, and the manual is blunt: with 5 V signals and the switch at 24 V,
*the motor will not work*. Set it to 5 V.

> Do not guess this one in the other direction: if the signals really were
> 24 V and you set the switch to 5 V, the manual says you will **damage the
> input**. Look at the wiring, then set the switch.

Also note switch 5 sets which way the motor turns. Leave it alone for now —
step 6 below is where you find out whether it needs flipping.

---

## Part 4 — The bench procedure

### Step 1 · Everything off, eyes only

Motor supply **off**. USB unplugged. Check Part 3's three switches on both
boxes. Check the four signal wires against this table (from `config.hpp`):

```
driver        Pico 2 W    physical pin   wire
Servo 1 PUL-  GP4         6              red       (azimuth step)
Servo 1 DIR-  GP5         7              red       (azimuth direction)
Servo 2 PUL-  GP6         9              blue      (elevation step)
Servo 2 DIR-  GP7         10             blue      (elevation direction)
all PUL+/DIR+ +5V, top two pins of each screw terminal, green
```

Two more holes that matter every time you pick up a meter, from the board's
own pinout sheet:

```
GND           physical pin 38    <- the black probe goes here, always
VSYS 5V       physical pin 39    <- where PUL+ and DIR+ are fed from
```

**Use pin 38 and nothing else as your ground reference.** Measuring a
driver terminal against some other ground gives numbers that look like
readings and are not: a floating input can read negative, which no Pico
output can ever do.

And VSYS reaches the board through a diode, so on USB power it is about
**4.7 V**, not a clean 5.00. Every expected voltage below allows for that.

### Step 2 · Flash the Pico

Motor supply still **off**.

Hold BOOTSEL, plug the Pico into the laptop. A drive appears. Copy
`gs_motor.uf2` onto it. The drive disappears and the Pico restarts.

### Step 3 · Say hello

```
py -m gs.motor --log bench.csv
```

**Expect:**

```
connected: gs_motor <some git hash>
logging to bench.csv
```

and, usually, the Pico's own startup lines:

```
=== gs_pico_link gs_motor <hash> ===
[motor] pins AZ PUL=GP4 DIR=GP5  EL PUL=GP6 DIR=GP7  idle HIGH OK
[motor] active low: PUL rests HIGH and is pulled LOW to step
[motor] ready az=101.587 st/deg el=63.492 st/deg
[motor] limits az 0.000..360.000  el 0.000..45.000
[motor] NOT ZEROED - send ZERO before any MOVE
```

**If the startup lines do not appear but `connected:` does** — that is
fine and expected sometimes. Opening a serial port throws away whatever
was already waiting in it, so a banner that arrived a moment too early is
lost. `ping` works regardless.

**If it says `gs_motor fake`** you are talking to the simulator, not the
Pico. Check you did not leave `--dry-run` on.

| What you see | What it means |
|---|---|
| `No serial ports at all` | Charge-only USB cable, or the `.uf2` never copied |
| `Port busy` | Another program has it — close the VS Code serial monitor |
| `azimuth pins did not read back idle HIGH` | A pin is being held down by something external; check the wiring before going further |

### Step 4 · Check the wires with a multimeter — still no motor power

This is why the `pin` command exists. A step pulse is five millionths of a
second long and no multimeter will see one, but the **direction** wire
holds steady, so it can be measured.

Black probe on a Pico ground pin. Red probe on physical pin 7 (GP5,
azimuth direction).

```
motor> pin 5 1
  GP5 is now high
motor> pin 5 0
  GP5 is now low
```

**Expect** about 3.3 V, then about 0 V. If it never changes, that wire is
broken or in the wrong hole.

Repeat on pin 10 (GP7) for elevation with `pin 7 1` / `pin 7 0`.

You can hold a **step** line too — `pin 4 0` for azimuth — but note:

> Driving a step line low makes the driver take a step the Pico did not
> count, so that axis gives up its zero and `move` is refused until you
> `zero` again. This is on purpose. Set it back high (`pin 4 1`) when you
> have finished measuring.

### Step 5 · Power the drivers

Motor supply on. **Hands clear.** Nothing should move. The green light on
each box should come on and stay on; the red light should stay off.

If a red light blinks, count the blinks and see Part 5 before going on.

### Step 6 · The first move — one degree

This is the one that tells you whether the axes turn the way the software
thinks they do.

```
motor> zero
  ALL is now 0.000 deg. Nothing moved.
motor> move az 5 1
```

**Expect** a plan line, a short wait, and a done line:

```
  plan   +102 steps  0.000 -> 1.004 deg  at 5.000 deg/s  flat out  about 0.20 s
  done   finished  +102 steps in 0.20 s  (predicted 0.20 s)
         now at 1.004 deg
```

**Watch which way it actually turned.** Decide now what you are calling
positive — write it down. If it turned the wrong way, that axis needs
`invert_dir` set to `true` in `motor_config.hpp` (a rebuild and a reflash),
or switch 5 on the driver flipped. Note it and carry on; it is a 7.10 job.

Then the same on elevation — **elevation carries the weight, so be ready**:

```
motor> move el 5 1
```

### Step 7 · A real move, and the first reading

```
motor> move az 5 45
motor> true <whatever the protractor says>
```

The predicted angle will read **44.996**, not 45.000. That is not an error:
a step is a fixed size and 45 degrees is 4571.4 of them, so the Pico rounds
to 4571 and tells you honestly where that lands. At 1600 pulses per turn a
step is about 0.0098 degrees, so the gap is never more than half of that.

Compare the two times on the `done` line. They should agree to within a few
hundredths of a second. If "measured" is much longer than "predicted", the
motor is struggling — stop and look at the supply voltage and the driver's
current setting before pushing the speed up.

### Step 8 · Both directions

```
motor> move az 5 -45
motor> pos
```

**Expect** to be back at `0.000 deg (0 steps)`. If the protractor disagrees
with the screen, that difference is backlash plus lost steps, and it is
worth a `note`.

### Step 9 · The speed sweep

```
motor> sweep angle=45 speeds=1,2,5,10,20,45,90 pause
```

It will move, then ask for a reading, seven times.

> Acceleration is still at the cautious 45 deg/s² it shipped with. At that
> rate a 45 degree move commanded at 90 deg/s spends almost all of its time
> speeding up and slowing down, so the average works out far below 90. That
> is the ramp, not a fault. Raising acceleration is a 7.10 job — if you want
> the fast rows to mean anything, do that first and run the sweep again.

### Step 10 · Random angles

```
motor> random count=10 speed=5 pause
```

Ten unpredictable angles inside the limits. This is the part that catches a
tight spot in the gearing or a step lost on a direction change. Write the
seed down — it is printed — so the same run can be repeated.

### Step 11 · Elevation, carefully

Elevation is clamped to **0–45 degrees** in software, which is tighter than
the real mechanism (−10 to +90). That is deliberate: until the hard stops
have been measured, a wrong direction sign runs out of commanded travel
long before it reaches anything solid.

```
motor> move el 5 10
motor> move el 5 -10
```

Then measure where the hard stops really are, by hand, with the power off.
Write the numbers down for 7.10.

### Step 12 · Finish

```
motor> rows
motor> quit
```

`rows` shows anything still missing a reading. On the way out it says so
again and tells you where the file is. A blank reading cannot be filled in
later — the antenna has moved on.

---

## Part 5 · When the red light blinks

The driver complains by blinking. Count the blinks in each three-second
group. **The Pico cannot see any of this** — the alarm wire is not
connected, so the software will keep reporting a position it believes while
the motor does something else.

| Blinks | Meaning | What to do |
|---|---|---|
| 1 | Over-current | **Power off now.** Check for a short in the wiring or motor |
| 2 | Over-voltage | **Power off now.** Check the supply |
| 3 | Chip fault | Restart the power; if it persists the box is faulty |
| 4 | Cannot hold the shaft | Motor not plugged in, or a bad motor cable |
| 5 | Memory fault | Restart the power |
| 6 | Self-setup failed | Restart the power |
| **7** | **Lost position** | **The one that matters.** The motor is not where it was told to be — too much load, too fast, or an encoder wiring fault. Everything measured after this is suspect |
| on solid | Board dead | Stop |

---

## Part 6 · Other things that go wrong

| What you see | Most likely cause |
|---|---|
| Nothing moves, no error | The 5 V / 24 V switch on top of the driver is still at 24 V |
| Motor hums but does not turn | A motor phase wire is in the wrong terminal |
| Moves the wrong way | `invert_dir`, or driver switch 5 — note it for 7.10 |
| Angles all wrong by the same factor | Pulses-per-turn switches are not at 800, or a gear was swapped |
| `that axis has not been zeroed yet` | Send `zero`. Also happens after `estop` and after poking a step pin |
| `that axis is already moving` | Wait for the move to finish |
| `the move would finish outside the soft limits` | It says the angle it would have reached and the allowed range |
| Measured time much longer than predicted | Motor struggling: supply voltage, current setting, or acceleration too high |
| Results file will not save | Close the spreadsheet if you have it open |

---

## Part 7 · What the results file holds

One row per move, written the moment the move ends.

| Column | What it is |
|---|---|
| `row` | Row number, for `true 3 45.1` |
| `when` | Time of day |
| `pico` | Which Pico answered — says `fake` for a practice run |
| `id` | The command number this row came from |
| `axis` | `AZ` or `EL` |
| `speed_dps` | Speed commanded, degrees per second |
| `commanded_deg` | How far you asked it to turn |
| `from_deg` | Where it started |
| `predicted_deg` | Where the Pico says it ended up |
| `true_deg` | **What the protractor said** — blank until you type `true` |
| `error_deg` | `true_deg` minus `predicted_deg` |
| `steps`, `steps_done` | Pulses planned and pulses sent |
| `shape` | `TRAP` normal, `TRI` too short to reach the speed asked for |
| `predicted_s`, `measured_s` | Expected and actual duration |
| `result` | `OK`, or `ABORT` if a stop cut it short |
| `note` | Whatever you typed with `note` |

The error is measured against **predicted**, not against what you asked
for, because asking for 45 degrees really commands 45.006 and measuring
against a number the machine was never given would fold a known rounding
into what is meant to measure the gearing.

---

## Part 8 · After the bench — the numbers to change

These are the 7.10 jobs. All but the last are in `motor_config.hpp`.

| Setting | Now | After |
|---|---|---|
| `invert_dir` (per axis) | `false`, unverified | Whatever step 6 showed |
| Azimuth acceleration | 45 deg/s² (cautious) | Up towards 180 once the machine has run |
| Elevation acceleration | 30 deg/s² | Up towards 90 |
| Elevation travel | 0 to 45 deg | The real stops, measured in step 11 |
| Supply voltage and driver current | Not recorded | Write both down |

---

## What this page does not cover

The **radio** bring-up — `gs_rx`, `gs_tx`, antennas and range — is a
separate procedure, listed in `GS_PICO_LINK_SPEC.md` section 11, and is not
written yet. The motor test is run with the radio disconnected.
