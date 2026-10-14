"""Finding and opening the Pico's serial port.

Three jobs: list what is plugged in, work out which one is the Pico, and
open it with a useful message when that fails.

`import serial` happens inside the functions, never at the top of the file.
That is on purpose: it lets everything else here - the port picking, the
error messages, the tests - run on a machine with no pyserial installed, and
lets the CLI's dry-run mode work with no serial support at all.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

# Every Raspberry Pi USB device reports this vendor id, the Pico included.
# It is how we tell a Pico from the four other things claiming a COM port.
PICO_VID = 0x2E8A

# Required word for word by the spec. It is the most common first-run problem
# on Windows, where only one program may hold a COM port at a time.
PORT_BUSY_MESSAGE = (
    "Port busy: close the VS Code Serial Monitor or any other program using it."
)


class PortError(Exception):
    """Something went wrong finding or opening a port, with a message meant
    to be printed straight to the operator."""


@dataclass(frozen=True)
class PortInfo:
    """One serial port, as far as we care about it."""

    device: str  # "COM3", "/dev/ttyACM0"
    description: str
    is_pico: bool

    def label(self) -> str:
        tag = "  <-- Pico" if self.is_pico else ""
        return f"{self.device:15} {self.description}{tag}"


def pick_pico(ports: list[PortInfo]) -> str:
    """Choose the one Pico among the ports given.

    Separate from the listing so it can be tested without any hardware, and
    kept strict on purpose. Guessing between two Picos could mean sending a
    move command to the rocket's transmitter instead of the motor controller.
    """
    picos = [p for p in ports if p.is_pico]

    if not picos:
        if ports:
            others = ", ".join(p.device for p in ports)
            raise PortError(
                "No Pico found. Serial ports present: "
                f"{others}.\nIs it plugged in, and is the cable a data cable "
                "rather than charge-only?"
            )
        raise PortError(
            "No serial ports at all. Is the Pico plugged in, and is the cable "
            "a data cable rather than charge-only?"
        )

    if len(picos) > 1:
        found = ", ".join(p.device for p in picos)
        raise PortError(
            f"More than one Pico found: {found}.\n"
            "Say which one with --port, for example --port " + picos[0].device
        )

    return picos[0].device


def list_ports() -> list[PortInfo]:
    """Every serial port the machine can see, Picos marked."""
    try:
        from serial.tools import list_ports as pyserial_list_ports
    except ImportError as exc:  # pragma: no cover - needs pyserial missing
        raise PortError(
            "pyserial is not installed. Run:  py -m pip install -e laptop"
        ) from exc

    found = []
    for p in sorted(pyserial_list_ports.comports(), key=lambda p: p.device):
        found.append(
            PortInfo(
                device=p.device,
                description=p.description or "",
                is_pico=(p.vid == PICO_VID),
            )
        )
    return found


def find_pico_port() -> str:
    """The Pico's port, or a PortError saying why not."""
    return pick_pico(list_ports())


def open_port(port: str | None = None, timeout: float = 1.0):
    """Open a port and hand back the serial object.

    `port` may be a device name, any pyserial URL such as
    socket://localhost:7777, or None to go looking for a Pico.

    serial_for_url is used rather than serial.Serial so that the URL forms
    work. The fake Pico in a later chunk is a TCP server, and being able to
    point the same code at it is what lets the whole interface be tested
    without hardware.
    """
    try:
        import serial
    except ImportError as exc:  # pragma: no cover - needs pyserial missing
        raise PortError(
            "pyserial is not installed. Run:  py -m pip install -e laptop"
        ) from exc

    target = port or find_pico_port()

    try:
        return serial.serial_for_url(target, baudrate=115200, timeout=timeout)
    except serial.SerialException as exc:
        text = str(exc).lower()
        # Windows says "access is denied"; Linux says "resource busy". Both
        # mean the same thing to the operator.
        if "access is denied" in text or "busy" in text or "permission" in text:
            raise PortError(PORT_BUSY_MESSAGE) from exc
        raise PortError(f"Could not open {target}: {exc}") from exc


def main(argv: list[str] | None = None) -> int:
    """`py -m gs.ports` lists the ports, the same way the CLI's --list will."""
    del argv  # no options yet; the real CLI arrives in chunk 7.8

    try:
        ports = list_ports()
    except PortError as exc:
        print(exc)
        return 2

    if not ports:
        print("No serial ports found.")
        return 1

    for p in ports:
        print(p.label())

    try:
        print(f"\nWould use: {pick_pico(ports)}")
    except PortError as exc:
        print(f"\n{exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
