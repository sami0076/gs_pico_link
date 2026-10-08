# gs_pico_link: implementation spec for Claude Code

You are implementing the software for a rocket-tracking ground station receiver, in this repo, **without access to the hardware**. Read this whole file before writing code. Then plan, build in the phases in section 9, and finish with the report in section 10.

The owner is a student on Virginia Tech's IREC rocketry team. He is learning how the Pico works, so code must be **readable and commented for a learner**: plain C/C++, no clever templates, short functions, comments that explain *why*.

---

## 1. What the system does

```
Rocket GPS --NMEA/UART--> Rocket Pico + RFM95W --LoRa 915 MHz--> Ground Pico + RFM95W --USB serial--> Laptop (Python)
                                                                                                         |
                                                                       (later) AZ/EL commands --> motor drivers --> antenna gimbal
```

- **Ground Pico (receiver, `gs_rx`)** is deliberately dumb: every LoRa packet becomes one text line on USB serial. No parsing on the Pico.
- **Laptop (`gs` Python package)** reads those lines, validates and parses NMEA GGA, computes azimuth / elevation / range from the ground station to the rocket, tracks link health, logs CSV.
- **Test transmitter (`gs_tx`, `gs_tx_gps`)** is a second Pico that stands in for the rocket: it sends simulated GGA sentences, or forwards GGA from a real GPS module.
- **Motors are out of scope.** Only define the command line format (section 6.4) so the next phase can plug in.

## 2. Reference material

Put these files in `reference/` (the owner has them) and use them as the starting point. They compile and the Python was tested, but they need the refactor and fixes in this spec:

- `reference/gs_pico_link.cpp`: single-file firmware, RX/TX switched by `#define NODE_ROLE`.
- `reference/gs_reader.py`: single-file laptop reader.
- `reference/sample_rx.txt`: replay sample.

If `reference/` is missing, ask the owner for it before starting. Do not try to reconstruct it from memory.

## 3. Locked decisions (do not change without asking the owner)

### 3.1 Hardware

- Board: **Raspberry Pi Pico 2 W** (RP2350, ARM cores). CMake `PICO_BOARD pico2_w`, platform `rp2350-arm-s`.
- Radio: **Adafruit RFM95W breakout, 915 MHz (Semtech SX1276)**, one per Pico.
- Optional GPS on the transmitter: any 3.3 V NMEA module at 9600 baud.

### 3.2 Pinout (GPIO numbers; physical pins in parentheses)

| RFM95W | Pico | Notes |
| --- | --- | --- |
| VIN | 3V3 OUT (36) | |
| GND | GND (38) | |
| SCK | GP18 (24) | SPI0 SCK |
| MOSI | GP19 (25) | SPI0 TX |
| MISO | GP16 (21) | SPI0 RX |
| CS | GP17 (22) | plain GPIO, driven by RadioLib |
| RST | GP20 (26) | SX1276 reset is **active low** |
| G0 / DIO0 | GP21 (27) | RxDone / TxDone interrupt |
| G1 / DIO1 | GP22 (29) | |
| EN, G2–G5 | not connected | |

GPS (transmitter only, `gs_tx_gps`): GPS TX → GP1 (pin 2, UART0 RX), GPS RX ← GP0 (pin 1, UART0 TX), 9600 baud.

This pinout comes from the team's Test 1 sheet. **Verify every GPIO-to-function mapping** (GP16 = SPI0 RX, GP17 = SPI0 CSn, GP18 = SPI0 SCK, GP19 = SPI0 TX, GP0/GP1 = UART0 TX/RX) against the RP2350 datasheet's GPIO function table or the Pico SDK headers, and the physical pin numbers against the Pico 2 W pinout. Report the result.

### 3.3 Radio settings (must be identical on both ends)

These match last year's flight link, so the receiver can still hear last year's rocket hardware.

| Setting | Value | RadioLib argument |
| --- | --- | --- |
| Frequency | 915.0 MHz | `freq` |
| Bandwidth | 125 kHz | `bw` |
| Spreading factor | 7 | `sf` |
| Coding rate | 4/5 | `cr = 5` (RadioLib takes the denominator) |
| Sync word | 0x12 | `syncWord` |
| Preamble | 8 symbols | `preambleLength` |
| CRC | on | RadioLib default for SX127x LoRa; verify, and set it explicitly |
| Header | explicit | default |
| TX power | 10 dBm bench default, configurable; last year flew 20 dBm | `power` |

Put all of these in **one** header (`firmware/src/config.hpp`) used by every target.

### 3.4 Payload format (what goes over the air)

- **NMEA mode (default):** the payload is exactly one NMEA sentence as ASCII, `$` through the two checksum hex digits, no CR/LF. Example: `$GPGGA,000012.00,3713.7904,N,08024.8124,W,1,09,0.9,1234.0,M,-33.0,M,,*69`. Any talker ID is valid (`GP`, `GN`, `GL`, ...).
- **Diagnostic mode (`gs_tx` built with `GS_TX_MODE_DIAG`):** payload `SEQ,<n>,transmitting`, n = 0, 1, 2 ... incrementing per packet, reset on transmitter boot. This is the team's planned Test 1 diagnostic and lets the laptop count lost packets exactly.
- Future binary payloads must not break the receiver: see `RXH` in section 6.1.

## 4. Repo layout to create

```
firmware/
  CMakeLists.txt            # builds three Pico targets: gs_rx, gs_tx, gs_tx_gps
  pico_sdk_import.cmake
  src/
    config.hpp              # pins + radio settings (section 3)
    radio.hpp / radio.cpp   # SPI self-test, radio bring-up (RadioLib)
    rx_main.cpp             # receiver
    tx_main.cpp             # transmitter (simulated, diag, or real GPS by compile definition)
    nmea_build.hpp / .cpp   # PURE C++: checksum, coordinate formatting, simulated GGA. No Pico headers.
    rx_format.hpp / .cpp    # PURE C++: formats RX/RXH/ERR/STAT lines. No Pico headers.
    gps_uart.hpp / .cpp     # IRQ-driven UART ring buffer + GGA line assembler (gs_tx_gps only)
  lib/RadioLib/             # git submodule pinned to a release tag
  host_tests/
    CMakeLists.txt          # builds with the HOST compiler (g++/clang), no Pico SDK
    test_nmea_build.cpp
    test_rx_format.cpp
    gen_contract_vectors.cpp  # writes lines the Python tests consume (section 8.3)
laptop/
  pyproject.toml            # package "gs", Python >= 3.10, runtime dep: pyserial
  gs/
    __init__.py
    nmea.py                 # checksum, GGA parse, ddmm.mmmm -> degrees
    geo.py                  # WGS-84 geodetic -> ECEF -> ENU -> az/el/range
    protocol.py             # parse Pico lines (RX, RXH, ERR, STAT, other) into typed records
    link.py                 # link statistics (section 6.3)
    reader.py               # CLI: python -m gs.reader
    fake_pico.py            # CLI: python -m gs.fake_pico (section 8.4)
  tests/                    # pytest
  samples/sample_rx.txt
docs/
  HARDWARE_BRINGUP.md       # checklist of everything that can only be verified on hardware
.github/workflows/ci.yml
README.md
```

If the repo already contains a project generated by the Raspberry Pi Pico VS Code extension, keep the block at the top of its `CMakeLists.txt` marked "DO NOT EDIT" so the extension's buttons keep working, and fold this layout around it. Ask the owner if the existing layout conflicts.

## 5. Firmware requirements

### 5.1 Build

- Three executables from one `firmware/CMakeLists.txt`:
  - `gs_rx`: receiver.
  - `gs_tx`: transmitter, simulated GGA. `GS_TX_MODE_DIAG=1` variant may be a fourth target `gs_tx_diag` or a CMake option; pick one and document it.
  - `gs_tx_gps`: transmitter forwarding GGA from a real GPS.
- Roles are chosen by **build target**, never by editing a `#define`. This removes the risk of flashing the wrong role.
- USB stdio on, UART stdio off (`pico_enable_stdio_usb(<t> 1)`, `pico_enable_stdio_uart(<t> 0)`).
- RadioLib built from the submodule exactly as its `examples/NonArduino/Pico` does: `add_subdirectory(lib/RadioLib)`, add `lib/RadioLib/src/hal/RPiPico/PicoHal.cpp` to each target, define `RADIOLIB_BUILD_RPI_PICO`, link `RadioLib hardware_spi hardware_gpio hardware_timer pico_multicore hardware_pwm` (+ `hardware_uart` for `gs_tx_gps`).
- Pin RadioLib to a **release tag** (the reference was checked against 7.8.1; use that or a newer stable tag) and record it in the README.
- Pico SDK: the team's VS Code setup records SDK 2.3.1, toolchain 15_2_Rel1, picotool 2.3.1. Use the same in CI if that tag exists; otherwise the newest 2.x release, and document what you used.
- Compile project sources with `-Wall -Wextra`. **Zero warnings from project sources** (RadioLib's own warnings are acceptable; don't patch the library).

### 5.2 Known issues in the reference to keep fixed

1. RadioLib's `PicoHal::init()` calls `stdio_init_all()` a second time when `radio.begin()` runs. The reference subclasses `PicoHal` as `GsHal` with `init()` overridden to call only `spiBegin()`. Keep that, after confirming `spiBegin()` is still public in the pinned RadioLib version.
2. **NMEA coordinate rounding bug:** formatting minutes with `%07.4f` can print `60.0000` (e.g. latitude 37.9999999° prints as `3760.0000`). Format from an integer count of 1/10000 minutes so a carry rolls into the degrees: 37.9999999 must become `3800.0000`. Test it (section 8.1).
3. **Transmitter cadence:** the reference does `transmit()` (about 175 ms) then `sleep_ms(1000)`, so the real period is about 1.175 s. Schedule on absolute time so packets go out at exactly 1 Hz (configurable).
4. **GPS bytes lost during transmit:** `transmit()` blocks for about 175 ms, and the RP2350 UART RX FIFO is 32 bytes (about 33 ms at 9600 baud). Use a UART RX interrupt feeding a ring buffer (≥ 1 KB) so no bytes are dropped. Alternatively, use RadioLib's non-blocking `startTransmit()` + TX-done flag. Either way, document the choice.

### 5.3 Startup (all targets)

1. `stdio_init_all()`.
2. Wait for a USB host to open the port: forever on `gs_rx` (it lives on the laptop), up to 3 s on the TX targets (they run on a power bank).
3. Print a banner: `=== gs_pico_link <target> <git describe or version> ===`.
4. **Raw SPI self-test**, before RadioLib: reset the SX1276 (RST low ≥ 100 µs, release, wait 10 ms), read register 0x42 (`RegVersion`; read = address with MSB 0). Expect `0x12`. Print one of:
   - `[spi] RegVersion=0x12 OK`
   - `[spi] RegVersion=0x00 FAIL: check MISO, VIN/GND, CS`
   - `[spi] RegVersion=0xFF FAIL: no power or RST held low`
   - `[spi] RegVersion=0x?? FAIL: check SPI wiring`
5. `radio.begin(...)` with the locked settings, then explicitly set CRC on. Print `[radio] ready 915.0MHz BW125 SF7 CR4/5 sync=0x12 pwr=<dBm>` or `[radio] begin failed code=<n>`.
6. On any failure, print `[fatal] <reason>` every 2 s forever. Never die silently.

Verify the SX1276 register address, expected value and reset polarity against the Semtech SX1276 datasheet, and the RadioLib `SX1276::begin` signature against the pinned source. Report both.

### 5.4 Receiver (`gs_rx`)

- Interrupt-driven: `setPacketReceivedAction(flag_isr)`, `startReceive()`. The ISR only sets a `volatile bool`. The main loop reads the packet, prints, and calls `startReceive()` again.
- On a good packet: `getPacketLength()`, `readData()`, `getRSSI()`, `getSNR()`. Then format through `rx_format` (section 6.1):
  - payload is all printable ASCII (0x20–0x7E), after stripping a trailing CR/LF → `RX` line
  - otherwise → `RXH` line (hex)
- CRC error → `ERR,crc,<rssi>,<snr>`; other read error → `ERR,read,<code>`.
- Heartbeat every 5 s: `STAT,<uptime_s>,<good>,<crc_errors>,<other_errors>`.
- Keep USB output atomic per line: build the full line in a buffer, then a single `printf`/`fputs`.
- No dynamic allocation in the loop.

### 5.5 Transmitter (`gs_tx`, `gs_tx_gps`)

- `gs_tx` simulated GGA: launch site 37.2296, -80.4139, 634 m MSL; 180 s cycle: climb 50 m/s for 60 s to 3,000 m AGL, then descend 25 m/s to the ground; drift +0.00002°/s lat, +0.00003°/s lon from the cycle start; fix quality 1, 9 sats, HDOP 0.9, geoid separation -33.0 M; UTC = seconds since boot. Keep these in named constants so they're easy to edit.
- `gs_tx` diag mode: `SEQ,<n>,transmitting`.
- `gs_tx_gps`: assemble complete lines from the ring buffer, forward only sentences whose type is `GGA` (any talker) and whose checksum is valid. Forward GGA even with fix quality 0 (the laptop decides what to do with it). Drop sentences longer than 82 characters, the NMEA 0183 maximum.
- Every transmit prints `TX,<count>,<payload>` or `TXERR,<code>`.

## 6. Serial line protocol (Pico → laptop)

One record per line, `\n` terminated, ASCII. This is a **contract** between C++ and Python and has a contract test (section 8.3).

### 6.1 Records

| Record | Format | Notes |
| --- | --- | --- |
| Text packet | `RX,<rssi>,<snr>,<len>,<payload>` | rssi `%.1f` dBm, snr `%.2f` dB, len = payload bytes after CR/LF strip. Payload may contain commas: split on the first 4 commas only |
| Binary packet | `RXH,<rssi>,<snr>,<len>,<hex>` | lowercase hex, 2 chars per byte |
| CRC error | `ERR,crc,<rssi>,<snr>` | |
| Other error | `ERR,read,<code>` | RadioLib error code |
| Heartbeat | `STAT,<uptime_s>,<good>,<crc_errors>,<other_errors>` | |
| Transmit | `TX,<count>,<payload>` / `TXERR,<code>` | TX targets only |
| Anything else | free text | banners, `[spi]`, `[radio]`, `[fatal]`. The reader shows them as info |

### 6.2 Reading GGA (laptop)

- Validate the checksum: XOR of all characters strictly between `$` and `*` equals the two hex digits after `*` (case-insensitive).
- Fields: time, lat `ddmm.mmmm`, N/S, lon `dddmm.mmmm`, E/W, fix quality, sats, HDOP, altitude (m MSL), `M`, geoid separation, `M`, DGPS age, DGPS station.
- S and W are negative. Fix quality 0, or empty lat/lon → a "no fix" record, not an error.
- Altitude is MSL. The ground station altitude given to the reader must also be MSL.

### 6.3 Link statistics (laptop, `link.py`)

Matching the team's Test 1 needs:

- Counts: valid packets, CRC errors, other errors.
- Diag mode (`SEQ`): expected = last − first + 1, missing = gaps in sequence, loss % = missing / (valid + missing) × 100. A sequence that goes **backwards** = transmitter reboot: start a new window, don't count it as loss.
- RSSI/SNR mean and minimum per window.
- Time since last valid packet; **LINK LOST** after 3.0 s with no valid packet (configurable), **LINK RESTORED** on the next valid packet, with the gap length. Longest silence per window.
- CRC errors are not counted again as missing packets.

### 6.4 Pointing output (defined now, used later)

The reader computes, for each valid fix: azimuth (degrees clockwise from **true** north, 0–360), elevation (degrees above the local horizon), slant range (m).

- Method: geodetic (WGS-84) → ECEF → local ENU at the ground station → `az = atan2(E, N)`, `el = atan2(U, hypot(E, N))`. Not haversine plus flat-earth.
- If the horizontal distance is < 1 m, azimuth is undefined: report `az = None` and hold the last valid azimuth for any command output.
- Command line format for the future motor controller (print with `--emit-commands`; don't send it anywhere yet): `AZEL,<az_deg %.2f>,<el_deg %.2f>,<range_m %.1f>,<fix_utc>`.

## 7. Laptop CLI (`python -m gs.reader`)

- `--list`: list serial ports and mark Raspberry Pi devices (USB VID 0x2E8A).
- `--port <name|url>`: `COM5`, `/dev/ttyACM0`, or any pyserial URL such as `socket://localhost:7777` (use `serial.serial_for_url`). Default: auto-detect by VID, fail with a clear message if 0 or more than 1 are found.
- `--replay <file>` with `--replay-delay <s>` (default 0.2; 0 = as fast as possible).
- `--gs-lat --gs-lon --gs-alt` (alt in m MSL; the defaults are fine for development, but print a warning when they're used).
- `--log <csv>`: append; header once. Columns: `pc_time_iso, utc, lat, lon, alt_m, fix_quality, sats, hdop, rssi_dbm, snr_db, az_deg, el_deg, range_m, seq`.
- `--emit-commands`: also print `AZEL` lines.
- Clean Ctrl+C: flush the log, print a final link-statistics summary.
- If the port is busy, say: "Port busy: close the VS Code Serial Monitor or any other program using it." This is the most common first-run problem on Windows.
- Must run on Windows 10/11 with `py -m gs.reader`. Don't use `select()` on serial ports, POSIX-only paths, or ANSI escape codes unless the terminal supports them.

## 8. Verification without hardware (required)

All of this must pass locally and in CI. Put the exact commands in the README.

### 8.1 C++ host tests (`firmware/host_tests`, host compiler, no Pico SDK)

- `nmea_checksum`: `$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47` → 0x47.
- Coordinate formatting: 37.2296 → `3713.7760,N`; -80.4139 → `08024.8340,W`; 37.9999999 → `3800.0000,N` (the rounding bug); 0.0 → `0000.0000,N`; -0.5 → `0030.0000,S`.
- Simulated GGA at t = 0, 4, 12 s must equal these exactly:
  - t=0: `$GPGGA,000000.00,3713.7760,N,08024.8340,W,1,09,0.9,634.0,M,-33.0,M,,*53`
  - t=4: `$GPGGA,000004.00,3713.7808,N,08024.8268,W,1,09,0.9,834.0,M,-33.0,M,,*53`
  - t=12: `$GPGGA,000012.00,3713.7904,N,08024.8124,W,1,09,0.9,1234.0,M,-33.0,M,,*69`
- `rx_format`: an ASCII payload produces an `RX` line; a payload with byte 0x00 or 0xFF produces `RXH` with correct hex; trailing `\r\n` is stripped; a 255-byte payload doesn't overflow.

### 8.2 Python tests (pytest)

- Checksum: the 4807.038 vector → valid; one digit changed → invalid; lowercase hex accepted.
- Parsing the 4807.038 vector → lat 48.1173, lon 11.516667 (within 1e-6), alt 545.4, sats 8, quality 1.
- `$GNGGA` accepted. Fix quality 0 → no-fix record. Empty fields → no-fix, no exception. Garbage → rejected, no exception.
- Look angles. Ground station 37.2290, -80.4150, 634.0 m. Tolerance: 0.01° on angles, 0.5 m on range. These values were cross-checked against `pymap3d.geodetic2aer`:

  | Target lat, lon, alt (m) | az (°) | el (°) | range (m) |
  | --- | --- | --- | --- |
  | 37.2310, -80.4120, 3634.0 | 50.1799 | 83.4055 | 3019.97 |
  | 37.2380, -80.4150, 634.0 | 0.0000 | -0.0045 | 998.94 |
  | 37.2200, -80.4300, 700.0 | 233.1234 | 2.2632 | 1665.78 |
  | 37.2290, -80.4150, 1634.0 (straight up) | undefined → `None` | 90.0000 | 1000.00 |

  Optionally also compare against `pymap3d` as a dev-only dependency on random points.
- Protocol: every record type in 6.1 parses; an `RX` payload containing commas is kept whole; malformed lines become "info", never crash.
- Link stats: synthetic `SEQ` streams with gaps, a reboot (sequence reset), CRC errors and a 4 s silence produce the expected loss %, LINK LOST/RESTORED events and longest silence.
- Replay end-to-end: `samples/sample_rx.txt` through the reader produces the expected fixes and a CSV with the right header and row count.

### 8.3 C++ ↔ Python contract test

`gen_contract_vectors` (host C++) writes simulated GGA sentences for t = 0…179 and `rx_format` output for a set of payloads to a file. A pytest reads that file and checks that every line parses, every checksum validates, and the parsed lat/lon/alt match the simulator's inputs (lat/lon within 1e-6°, alt within 0.05 m). CI runs the C++ step first. This is the test that catches the two sides drifting apart.

### 8.4 Fake Pico (`python -m gs.fake_pico`)

A TCP server (default port 7777) that speaks the exact protocol in 6.1: banner, `[spi] ... OK`, `[radio] ready ...`, then `RX` lines with simulated GGA (same trajectory as `gs_tx`) at 1 Hz, `STAT` every 5 s. Options: `--loss <0..1>` (random drops), `--crc <0..1>` (`ERR,crc` instead of `RX`), `--diag` (`SEQ` payloads), `--outage <start_s>:<dur_s>` (silence, to trigger LINK LOST). The reader connects with `--port socket://localhost:7777`. Add a pytest that starts it on an ephemeral port and runs the reader against it for a few seconds.

### 8.5 Firmware cross-build

CI (ubuntu-latest) installs the ARM toolchain, checks out the Pico SDK at the chosen tag with submodules, and builds every Pico target for `pico2_w`. The job fails on any warning from project sources. Upload the `.uf2` files as build artifacts so the owner can download and flash them.

### 8.6 What you must NOT claim

You can't verify radio behaviour, timing on the chip, USB enumeration on Windows, RF range, or anything physical. Don't say any of it "works". Put every such item in `docs/HARDWARE_BRINGUP.md` (section 11) with the exact expected output, so the owner can tick them off.

## 9. Phases (commit after each, with passing checks)

1. **Skeleton:** layout, RadioLib submodule at a tag, CMake with three targets building the reference logic, README with build commands. Check: all targets build.
2. **Pure logic extracted:** `nmea_build`, `rx_format` + host tests (8.1), including the rounding fix. Check: host tests pass.
3. **Firmware refactor:** `config.hpp`, `radio.cpp`, `rx_main.cpp`, `tx_main.cpp`, fixed cadence, `gps_uart` with IRQ ring buffer, `RXH`, startup sequence (5.3). Check: zero project warnings.
4. **Python package:** `nmea`, `geo`, `protocol`, `link`, `reader` + pytest (8.2). Check: pytest passes.
5. **Contract test and fake Pico** (8.3, 8.4). Check: both pass.
6. **CI** (8.5) + `docs/HARDWARE_BRINGUP.md`. Check: the workflow file is valid; run its steps locally if you can't run Actions.

## 10. Report back when done

- A table of every check in section 8: passed / failed / not run, with the command used.
- Versions used: Pico SDK, toolchain, RadioLib tag, Python.
- Results of the verifications requested in 3.2, 5.2 and 5.3 (pinout, `spiBegin()` visibility, SX1276 register and reset polarity, RadioLib signatures), with the file or document each came from.
- Any deviation from this spec, and why.
- Open questions for the owner.

## 11. Contents required in `docs/HARDWARE_BRINGUP.md`

A checklist the owner works through once the wiring is done. Each item gives the exact expected serial output and what a failure means:

1. Flash `gs_rx` with BOOTSEL (drive name `RP2350`). The COM port appears as "USB Serial Device".
2. Banner + `[spi] RegVersion=0x12 OK` + `[radio] ready ...` + `STAT` every 5 s with zero packets.
3. Self-test failure table (0x00 / 0xFF / other → which wire).
4. Flash `gs_tx` on the second Pico. `TX,` lines at 1 Hz on its own monitor.
5. At 5–10 yards, the receiver shows `RX` lines at about 1 Hz. Power off the TX → lines stop and the reader prints LINK LOST after 3 s. Power on → LINK RESTORED.
6. `gs_tx_diag`: loss % near 0 at close range. Record RSSI/SNR (Test 1 sheet).
7. `gs_tx_gps` outdoors: fix quality goes from 0 to ≥ 1, and coordinates match a phone GPS to within about 10 m.
8. Reader against the real port: `py -m gs.reader --port COMx --gs-lat ... --log test.csv`. Az/el look sensible for a known walk-test direction.
9. Known gotchas: charge-only USB cables; only one program can open the COM port; never transmit without an antenna; antennas 5–10 yards apart, not touching.

## 12. Out of scope (don't build)

Motor control, the Tauri/React desktop app, WiFi/MQTT, the 433 MHz RFM69 backup link, IMU/magnetometer, FreeRTOS. Plain bare-metal Pico SDK loops are enough for this phase.
