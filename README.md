# gs_pico_link

Ground-station software for tracking a rocket by radio. A Pico on the rocket sends
its GPS position over LoRa; a Pico on the ground receives it and passes it to a
laptop, which works out where to point an antenna.

```
Rocket GPS --NMEA/UART--> Rocket Pico + RFM95W --LoRa 915 MHz--> Ground Pico + RFM95W --USB--> Laptop
                                                                                                 |
                                                             (later) AZ/EL commands --> antenna gimbal
```

The split in one line: **the ground Pico is deliberately dumb** and turns every
radio packet into one line of text. The laptop does all the parsing and math.
That keeps the embedded code simple and the tricky parts testable on a PC.

## Repo layout

| Path | What's in it | Status |
| --- | --- | --- |
| `firmware/CMakeLists.txt` | Builds the four Pico roles | done |
| `firmware/src/` | C++ for the Picos (receiver and transmitter) | `config.hpp` done; `main`s are placeholders |
| `firmware/host_tests/` | Tests for the pure-logic C++, run on the laptop | not started |
| `firmware/lib/RadioLib/` | The LoRa radio library (git submodule) | pinned at 7.8.1 |
| `laptop/gs/` | The `gs` Python package: parsing, math, CLI | not started |
| `laptop/tests/` | pytest suite | not started |
| `docs/HARDWARE_BRINGUP.md` | Bench checklist for once the wiring is done | not started |
| `reference/` | The original single-file versions this project refactors | starting point |
| `GS_PICO_LINK_SPEC.md` | The full specification | done |

## Hardware

- **Raspberry Pi Pico 2 W** (RP2350) x2
- **Adafruit RFM95W** 915 MHz LoRa breakout (Semtech SX1276) x1 per Pico
- Optional 3.3 V NMEA GPS module at 9600 baud on the transmitter

Wiring table lives in `GS_PICO_LINK_SPEC.md` section 3.2, mirrored in
`firmware/src/config.hpp`.

## Development environment

This was set up and verified on a Windows 11 ARM64 laptop (Snapdragon X Elite).
That machine needs an unusual split, because the official Pico VS Code extension
does not support Windows on ARM:

- **C++ builds happen in WSL2 Ubuntu**, where native `aarch64` compilers exist.
  This also matches the Linux CI job, so local and CI results agree.
- **The Python reader runs on Windows**, because only Windows sees the Pico's
  COM port.

### Verified versions

| Tool | Version | Where |
| --- | --- | --- |
| Pico SDK | 2.3.1 | WSL, `~/pico/pico-sdk` |
| ARM toolchain | 15.2.Rel1 (gcc 15.2.1) | WSL, `/opt/arm/...` |
| picotool | 2.3.1 | built by the SDK on demand |
| CMake / Ninja | 3.28.3 / 1.11.1 | WSL |
| Host compiler | g++ 13.3.0 | WSL |
| Python (reader) | 3.13.15 ARM64 + pyserial 3.5 | Windows |
| Python (CI/tests) | 3.12.3 | WSL |

The SDK and toolchain versions match what the team's VS Code setup records.

### Setting up a fresh machine

In WSL Ubuntu:

```bash
sudo apt install -y git cmake ninja-build build-essential python3 python3-pip \
                    libusb-1.0-0-dev pkg-config

# ARM cross-compiler (makes code for the Pico's chip)
cd /tmp
curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-aarch64-arm-none-eabi.tar.xz
sudo mkdir -p /opt/arm
sudo tar -xJf arm-gnu-toolchain-15.2.rel1-aarch64-arm-none-eabi.tar.xz -C /opt/arm

# Pico SDK (the submodules carry USB support, so don't skip them)
git clone --branch 2.3.1 --depth 1 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
git -C ~/pico/pico-sdk submodule update --init --depth 1

cat >> ~/.bashrc << 'SETUP'
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
export PATH="/opt/arm/arm-gnu-toolchain-15.2.rel1-aarch64-arm-none-eabi/bin:$PATH"
SETUP
source ~/.bashrc
```

On Windows, for the reader:

```powershell
winget install --id Python.Python.3.13 --architecture arm64 --custom "PrependPath=1"
py -m pip install pyserial pytest
```

If `py` isn't found afterwards, the PATH change hasn't reached your terminal yet.
Windows Terminal tabs inherit a stale PATH, so quit it completely and reopen.

## Build and run

Build output goes to a directory **inside WSL**, not here, because this folder is
synced by OneDrive and a build tree is thousands of object files.

```bash
# in WSL Ubuntu, from anywhere
cmake -S /mnt/c/Users/samis/OneDrive/Desktop/Sami_GS_Software_Testing/firmware \
      -B ~/build/gs -G Ninja
ninja -C ~/build/gs
```

That produces four `.uf2` files in `~/build/gs`, one per role:

| Target | Role |
| --- | --- |
| `gs_rx.uf2` | Receiver: listens and prints one text line per packet |
| `gs_tx.uf2` | Transmitter, simulated GGA (the normal bench transmitter) |
| `gs_tx_diag.uf2` | Transmitter, diagnostic counter instead of a position |
| `gs_tx_gps.uf2` | Transmitter, forwards GGA from a real GPS on UART0 |

**The role is the target, not a `#define`.** The spec allowed the diagnostic
transmitter to be either a CMake option or its own target; it is a target, so all
four roles exist in one build tree and switching roles is a reflash rather than a
reconfigure. There is no edit-this-line-back step that could flash a Pico with
the wrong job.

Build a single role with `ninja -C ~/build/gs gs_rx`. Build for a different board
with `-DPICO_BOARD=...`; it defaults to `pico2_w`.

### Smoke test: read a Pico over USB

This checks one thing only - that a Pico boots, enumerates as a USB serial port
and the laptop can read its output. **No radio is needed**; the placeholder
`main`s never touch SPI. One Pico and a data-capable USB cable is the whole rig.

1. **Flash.** Hold BOOTSEL while plugging the Pico in. It appears as a drive
   called `RPI-RP2`. Copy `gs_rx.uf2` onto it. The drive disappears and the Pico
   reboots into the program.

2. **Find the port.** On Windows:

   ```powershell
   python -m serial.tools.list_ports -v
   ```

   The Pico shows up as a USB Serial Device on some `COMx`. If no port appears at
   all, see the troubleshooting notes below.

3. **Read it.**

   ```powershell
   python -m serial.tools.miniterm COM5 115200
   ```

   Substitute your port. The baud rate is ignored for USB CDC - it is a virtual
   serial port, so any number works. Quit with `Ctrl-]`.

Expected output, appearing immediately and then once a second forever:

```
=== gs_pico_link gs_rx 9f3a1c2 ===
[rx] placeholder build: no radio touched, USB serial only
[rx] would use 915.0 MHz  BW 125.0 kHz  SF7  CR 4/5  sync 0x12
[rx] SPI0  SCK GP18  MOSI GP19  MISO GP16  CS GP17  RST GP20  DIO0 GP21
[rx] heartbeat every 1000 ms
[rx] alive n=1 up=1234 ms
[rx] alive n=2 up=2234 ms
```

The `alive` counter is the point of the test: a banner alone could be a stale
buffer, but a counter that keeps climbing proves the link is live.

The hash in the banner is whatever `git describe` saw at your last `cmake` run,
with `-dirty` appended if the tree had uncommitted changes. It is captured when
you configure, not when you build, so re-run `cmake` if you want it refreshed.

`gs_rx` waits forever for you to open the port, so you can flash it, walk away,
and still catch the banner whenever you connect. The three `gs_tx*` builds wait
only 3 s and then start regardless, because a transmitter normally runs off a
power bank with nothing watching - flash one of those and you will likely miss
its banner but still see the heartbeat.

#### If it doesn't work

| What you see | What it means |
| --- | --- |
| No `COMx` at all | Not a data cable (charge-only cables are common), or the `.uf2` never copied - re-check that `RPI-RP2` disappeared after the copy |
| Port appears, no text | Expected on a `gs_tx*` build if you connected after its 3 s window. The heartbeat should still arrive within a second |
| `could not open port` / access denied | Another program holds it. Close other terminals, VS Code serial monitors, Arduino IDE |
| `python` not recognised | Covered under Development environment above: the PATH change needs a full restart of Windows Terminal, or call the interpreter by full path |
| Garbage characters | Wrong port - you have opened some other device |

### Warnings

Project sources build with `-Wall -Wextra` and are held to **zero warnings**.
RadioLib is compiled as its own target with its own flags, so the warnings it
emits from `Module.cpp` (84 at the time of writing, all `-Wstringop-overflow=`)
are the library's, not ours. The spec says not to patch the library, so those are
left alone. If you want to confirm the split:

```bash
ninja -C ~/build/gs 2>&1 | grep ': warning' | grep firmware/src/   # should print nothing
```

One more piece of expected noise: the first `cmake` run in a fresh build
directory prints a `CMake Warning` from the SDK's `Findpicotool.cmake` saying
no installed picotool was found and it is building one from source. That is the
SDK talking about its own tooling, not a problem with this project.

## A note on what is and isn't verified

Everything in this repo is built and tested without access to the radios. Host
tests, Python tests and the C++/Python contract test all run on a PC. Radio
behaviour, on-chip timing, USB enumeration and RF range **cannot** be checked
that way, so none of it is claimed to work. Those checks live in
`docs/HARDWARE_BRINGUP.md` as a checklist with the exact expected output.
