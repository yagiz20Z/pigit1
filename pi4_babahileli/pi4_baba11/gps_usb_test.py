#!/usr/bin/env python3
"""NEO-M8N doğrudan USB/NMEA bağlantı testi.

Kullanım:
  python3 gps_usb_test.py /dev/ttyACM1 115200
  python3 gps_usb_test.py /dev/serial/by-id/usb-u-blox_... 9600

Port verilmezse GPS/GNSS/M8N/u-blox isimli by-id yolları ve ardından ttyACM/ttyUSB
portları denenir. Baud verilmezse yaygın GPS hızları otomatik taranır.
"""

from __future__ import annotations

import glob
import os
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:
    print("pyserial eksik: python3 -m pip install --user pyserial", file=sys.stderr)
    raise SystemExit(2)

BAUDS = [115200, 9600, 38400, 57600, 19200, 4800]


@dataclass
class GpsState:
    fix: int = 1
    sats: int = 0
    lat: float | None = None
    lon: float | None = None
    alt_m: float = 0.0
    speed_m_s: float = 0.0
    course_deg: float = 0.0


def candidates() -> list[str]:
    by_id = glob.glob("/dev/serial/by-id/*")
    preferred = [
        p
        for p in by_id
        if any(k in os.path.basename(p).lower() for k in ("u-blox", "ublox", "m8n", "gnss", "gps"))
    ]
    generic = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    result: list[str] = []
    for path in preferred + generic:
        if path not in result:
            result.append(path)
    return result


def checksum_ok(line: str) -> bool:
    line = line.strip()
    if not line.startswith("$"):
        return False
    if "*" not in line:
        return True
    body, received = line[1:].split("*", 1)
    if len(received) < 2:
        return False
    try:
        expected = int(received[:2], 16)
    except ValueError:
        return False
    value = 0
    for char in body.encode("ascii", errors="ignore"):
        value ^= char
    return value == expected


def coord(raw: str, hemi: str) -> float | None:
    if not raw or not hemi:
        return None
    try:
        value = float(raw)
    except ValueError:
        return None
    degrees = int(value // 100)
    minutes = value - degrees * 100
    result = degrees + minutes / 60.0
    if hemi in ("S", "W"):
        result = -result
    elif hemi not in ("N", "E"):
        return None
    return result


def parse(line: str, state: GpsState) -> bool:
    if not checksum_ok(line):
        return False
    body = line.strip()[1:].split("*", 1)[0]
    fields = body.split(",")
    if not fields:
        return False
    sentence = fields[0]

    try:
        if sentence.endswith("GSA") and len(fields) >= 3:
            state.fix = min(3, int(fields[2] or "1"))

        elif sentence.endswith("GGA") and len(fields) >= 10:
            quality = int(fields[6] or "0")
            state.sats = int(fields[7] or "0")
            state.alt_m = float(fields[9] or "0")
            if quality == 0:
                state.fix = 1
            else:
                state.lat = coord(fields[2], fields[3])
                state.lon = coord(fields[4], fields[5])
                if state.fix < 2:
                    state.fix = 2
            return True

        elif sentence.endswith("RMC") and len(fields) >= 9:
            state.speed_m_s = float(fields[7] or "0") * 0.514444
            state.course_deg = float(fields[8] or "0") % 360.0
            if fields[2] == "A":
                state.lat = coord(fields[3], fields[4])
                state.lon = coord(fields[5], fields[6])
                if state.fix < 2:
                    state.fix = 2
            else:
                state.fix = 1
            return True

        elif sentence.endswith("VTG") and len(fields) >= 8:
            state.course_deg = float(fields[1] or "0") % 360.0
            state.speed_m_s = float(fields[7] or "0") / 3.6
            return True

        return sentence.startswith(("GN", "GP", "GL", "GA", "GB", "BD"))
    except (ValueError, IndexError):
        return False


def try_port(port: str, baud: int, seconds: float = 5.0) -> bool:
    print(f"\nDeneniyor: {port} @ {baud}")
    state = GpsState()
    valid = 0
    deadline = time.monotonic() + seconds

    with serial.Serial(port, baud, timeout=0.8) as ser:
        ser.reset_input_buffer()
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not parse(line, state):
                continue
            valid += 1
            print(f"NMEA: {line[:140]}")
            if state.lat is not None and state.lon is not None:
                print(
                    "ISLENMIS: "
                    f"fix={state.fix} uydu={state.sats} "
                    f"lat={state.lat:.7f} lon={state.lon:.7f} "
                    f"alt={state.alt_m:.1f}m hiz={state.speed_m_s:.3f}m/s "
                    f"yon={state.course_deg:.2f}deg"
                )
            if valid >= 5:
                return True
    return valid > 0


def main() -> int:
    ports = [sys.argv[1]] if len(sys.argv) >= 2 else candidates()
    if not ports:
        print("Seri port bulunamadı. Önce: ls -l /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB*", file=sys.stderr)
        return 1

    bauds = [int(sys.argv[2])] if len(sys.argv) >= 3 else BAUDS

    for port in ports:
        for baud in bauds:
            try:
                if try_port(port, baud):
                    print(f"\nGPS bulundu: {port} @ {baud} baud")
                    print("Ana program için:")
                    print(f"  export IDA_GPS_PORT='{port}'")
                    print(f"  export IDA_GPS_BAUD='{baud}'")
                    print("  cargo run --release --bin pi4_baba")
                    return 0
            except (serial.SerialException, PermissionError, OSError) as exc:
                print(f"Açılamadı/okunamadı: {exc}")
                break

    print("Geçerli NMEA bulunamadı. Portu, kabloyu ve baudrate'i kontrol edin.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
