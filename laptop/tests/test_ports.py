"""Tests for finding the Pico's port.

None of these need a Pico, or even pyserial. pick_pico() works on a plain
list, which is why it is a separate function from the listing.
"""

from __future__ import annotations

import subprocess
import sys

import pytest

from gs.ports import PICO_VID, PORT_BUSY_MESSAGE, PortError, PortInfo, pick_pico


def port(device: str, description: str = "USB Serial Device", *, pico=False):
    return PortInfo(device=device, description=description, is_pico=pico)


def test_one_pico_is_chosen():
    ports = [
        port("COM1", "Communications Port"),
        port("COM3", "USB Serial Device", pico=True),
    ]
    assert pick_pico(ports) == "COM3"


def test_a_pico_among_several_other_ports():
    ports = [
        port("COM1", "Communications Port"),
        port("COM4", "Bluetooth link"),
        port("COM7", "USB Serial Device", pico=True),
        port("COM9", "Prolific USB-to-Serial"),
    ]
    assert pick_pico(ports) == "COM7"


# Refusing to guess matters here. Picking the wrong one of two Picos could
# mean sending a move command to the rocket's transmitter.
def test_two_picos_is_refused_and_says_how_to_choose():
    ports = [
        port("COM3", "USB Serial Device", pico=True),
        port("COM5", "USB Serial Device", pico=True),
    ]
    with pytest.raises(PortError) as exc:
        pick_pico(ports)

    message = str(exc.value)
    assert "COM3" in message
    assert "COM5" in message
    assert "--port" in message  # tells the operator what to do next


def test_no_pico_but_other_ports_lists_them():
    ports = [port("COM1", "Communications Port")]
    with pytest.raises(PortError) as exc:
        pick_pico(ports)

    message = str(exc.value)
    assert "COM1" in message
    assert "charge-only" in message  # the usual cause


def test_no_ports_at_all():
    with pytest.raises(PortError) as exc:
        pick_pico([])
    assert "charge-only" in str(exc.value)


def test_the_busy_message_is_exactly_as_specified():
    # Spec section 7 requires this wording. Worth pinning: it is the most
    # common first-run problem on Windows.
    assert PORT_BUSY_MESSAGE == (
        "Port busy: close the VS Code Serial Monitor or any other "
        "program using it."
    )


def test_the_vendor_id_is_the_raspberry_pi_one():
    assert PICO_VID == 0x2E8A


def test_the_label_marks_a_pico():
    assert "<-- Pico" in port("COM3", pico=True).label()
    assert "<-- Pico" not in port("COM1").label()


# pyserial is imported inside the functions, never at module scope, so that
# the CLI's dry-run mode works on a machine without it. Checked in a fresh
# interpreter because this one has already imported plenty.
def test_importing_does_not_pull_in_pyserial():
    code = "import gs.ports, sys; print('serial' in sys.modules)"
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert out.stdout.strip() == "False"
