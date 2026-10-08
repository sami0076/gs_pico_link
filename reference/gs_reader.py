#!/usr/bin/env python3
"""
gs_reader.py - laptop side of the ground station.

Reads lines from the receiver Pico over USB serial, checks and parses the NMEA
GGA sentence inside each packet, and prints where the rocket is plus the
azimuth / elevation the antenna would need to point at it.

Lines the Pico sends (see gs_pico_link.cpp):
    RX,<rssi>,<snr>,<len>,<payload>      a received packet
    ERR,crc,<rssi>,<snr>                 a packet arrived but was corrupted
    STAT,<uptime_s>,<good>,<crc_errors>  heartbeat every 5 s
    anything else                        startup / debug messages

Usage:
    python gs_reader.py --list                       # show serial ports
    python gs_reader.py                              # auto-find the Pico
    python gs_reader.py --port COM5 --log flight.csv
    python gs_reader.py --replay sample_rx.txt       # test without hardware

Set your ground-station position with --gs-lat/--gs-lon/--gs-alt (alt in m MSL,
same reference as GGA altitude).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from typing import Iterator, Optional

PICO_USB_VID = 0x2E8A  # Raspberry Pi's USB vendor ID


# ----------------------------------------------------------------------------
# NMEA parsing
# ----------------------------------------------------------------------------
@dataclass
class GgaFix:
    utc: str          # hhmmss.ss
    lat: float        # decimal degrees, + = North
    lon: float        # decimal degrees, + = East
    alt_m: float      # metres above mean sea level
    fix_quality: int  # 0 = no fix, 1 = GPS, 2 = DGPS, ...
    sats: int
    hdop: float


def nmea_checksum_ok(sentence: str) -> bool:
    """XOR of every char between '$' and '*' must equal the 2 hex digits after '*'."""
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, _, given = sentence[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(given[:2], 16)
    except ValueError:
        return False


def nmea_to_degrees(value: str, hemisphere: str) -> float:
    """'3713.7760','N' -> 37.229600.  Format is (d)ddmm.mmmm: degrees then minutes."""
    raw = float(value)
    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    result = degrees + minutes / 60.0
    return -result if hemisphere in ("S", "W") else result


def parse_gga(sentence: str) -> Optional[GgaFix]:
    """Return a GgaFix, or None if this isn't a valid GGA sentence with a fix."""
    if not nmea_checksum_ok(sentence):
        return None
    fields = sentence[1:].split("*")[0].split(",")
    # fields[0] is the talker+type, e.g. GPGGA or GNGGA
    if len(fields) < 10 or not fields[0].endswith("GGA"):
        return None
    try:
        quality = int(fields[6] or 0)
        if quality == 0 or not fields[2] or not fields[4]:
            return None  # GPS has no fix yet
        return GgaFix(
            utc=fields[1],
            lat=nmea_to_degrees(fields[2], fields[3]),
            lon=nmea_to_degrees(fields[4], fields[5]),
            alt_m=float(fields[9] or 0.0),
            fix_quality=quality,
            sats=int(fields[7] or 0),
            hdop=float(fields[8] or 0.0),
        )
    except ValueError:
        return None


# ----------------------------------------------------------------------------
# Pointing math: geodetic -> ECEF -> local East/North/Up -> az/el/range
# ----------------------------------------------------------------------------
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1 - WGS84_E2) + alt_m) * math.sin(lat)
    return x, y, z


def look_angles(gs_lat, gs_lon, gs_alt, tgt_lat, tgt_lon, tgt_alt):
    """Azimuth (deg from true north, clockwise), elevation (deg above horizon), slant range (m)."""
    gx, gy, gz = geodetic_to_ecef(gs_lat, gs_lon, gs_alt)
    tx, ty, tz = geodetic_to_ecef(tgt_lat, tgt_lon, tgt_alt)
    dx, dy, dz = tx - gx, ty - gy, tz - gz
    lat, lon = math.radians(gs_lat), math.radians(gs_lon)
    east = -math.sin(lon) * dx + math.cos(lon) * dy
    north = (-math.sin(lat) * math.cos(lon) * dx
             - math.sin(lat) * math.sin(lon) * dy
             + math.cos(lat) * dz)
    up = (math.cos(lat) * math.cos(lon) * dx
          + math.cos(lat) * math.sin(lon) * dy
          + math.sin(lat) * dz)
    rng = math.sqrt(east ** 2 + north ** 2 + up ** 2)
    az = math.degrees(math.atan2(east, north)) % 360.0
    el = math.degrees(math.atan2(up, math.hypot(east, north)))
    return az, el, rng


# ----------------------------------------------------------------------------
# Line sources: real serial port, or a replay file for testing
# ----------------------------------------------------------------------------
def find_pico_port() -> Optional[str]:
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == PICO_USB_VID:
            return p.device
    return None


def serial_lines(port: str) -> Iterator[str]:
    import serial  # pip install pyserial
    # Baud rate is ignored for USB CDC; any value works.
    with serial.Serial(port, 115200, timeout=1) as ser:
        print(f"[reader] opened {port}  (Ctrl+C to stop)")
        while True:
            raw = ser.readline()
            if raw:
                yield raw.decode("utf-8", errors="replace").strip()


def replay_lines(path: str, delay_s: float) -> Iterator[str]:
    with open(path, encoding="utf-8") as f:
        for line in f:
            yield line.strip()
            time.sleep(delay_s)


# ----------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="Read LoRa telemetry from the ground-station Pico")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0 (default: auto-detect)")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--replay", help="read lines from a text file instead of the Pico")
    ap.add_argument("--replay-delay", type=float, default=0.2)
    ap.add_argument("--log", help="append parsed fixes to this CSV file")
    ap.add_argument("--gs-lat", type=float, default=37.2290, help="ground station latitude (deg)")
    ap.add_argument("--gs-lon", type=float, default=-80.4150, help="ground station longitude (deg)")
    ap.add_argument("--gs-alt", type=float, default=634.0, help="ground station altitude (m MSL)")
    args = ap.parse_args()

    if args.list:
        from serial.tools import list_ports
        for p in list_ports.comports():
            tag = "  <-- Pico" if p.vid == PICO_USB_VID else ""
            print(f"{p.device:15} {p.description}{tag}")
        return 0

    if args.replay:
        lines = replay_lines(args.replay, args.replay_delay)
    else:
        port = args.port or find_pico_port()
        if not port:
            print("No Pico found. Plug it in, or pass --port (see --list).")
            return 1
        lines = serial_lines(port)

    writer = None
    if args.log:
        log_file = open(args.log, "a", newline="", encoding="utf-8")
        writer = csv.writer(log_file)
        if log_file.tell() == 0:
            writer.writerow(["pc_time", "utc", "lat", "lon", "alt_m", "sats", "hdop",
                             "rssi", "snr", "az_deg", "el_deg", "range_m"])

    print(f"[reader] ground station at {args.gs_lat:.6f}, {args.gs_lon:.6f}, {args.gs_alt:.1f} m MSL")
    last_packet = time.time()
    try:
        for line in lines:
            if line.startswith("RX,"):
                last_packet = time.time()
                # maxsplit=4 so commas inside the NMEA payload stay together
                _, rssi, snr, length, payload = line.split(",", 4)
                fix = parse_gga(payload)
                if fix is None:
                    ok = "bad checksum" if payload.startswith("$") and not nmea_checksum_ok(payload) else "not a GGA fix"
                    print(f"RX  rssi {rssi:>6} snr {snr:>6}  ({ok}) {payload}")
                    continue
                az, el, rng = look_angles(args.gs_lat, args.gs_lon, args.gs_alt,
                                          fix.lat, fix.lon, fix.alt_m)
                print(f"RX  rssi {rssi:>6} snr {snr:>6} | {fix.utc} {fix.lat:10.6f} {fix.lon:11.6f} "
                      f"{fix.alt_m:8.1f} m  sats {fix.sats:2} | AZ {az:6.1f}  EL {el:5.1f}  R {rng:8.0f} m")
                if writer:
                    writer.writerow([f"{time.time():.3f}", fix.utc, f"{fix.lat:.7f}", f"{fix.lon:.7f}",
                                     fix.alt_m, fix.sats, fix.hdop, rssi, snr,
                                     f"{az:.2f}", f"{el:.2f}", f"{rng:.1f}"])
                    log_file.flush()
            elif line.startswith("ERR,"):
                print(f"!!  {line}")
            elif line.startswith("STAT,"):
                _, up, good, crc = line.split(",")
                quiet = time.time() - last_packet
                print(f"--  pico up {up}s  good {good}  crc_err {crc}  last packet {quiet:.0f}s ago")
            elif line:
                print(f"    {line}")
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
