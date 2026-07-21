#!/usr/bin/env python3
"""NAV ve DR telemetri satırlarını seri porttan okuyup iki ayrı iz halinde gösterir."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Optional


@dataclass
class DrFrame:
    lat: float
    lon: float
    yaw: float
    relative_yaw: float
    gps_error_m: float
    active: bool


def checksum(payload: str) -> str:
    return f"{sum(payload.encode('ascii')) & 0xFF:02X}"


def checked_payload(line: str) -> Optional[str]:
    clean = line.strip()
    if "*" not in clean:
        return None
    payload, received = clean.rsplit("*", 1)
    return payload if checksum(payload).lower() == received.lower() else None


def parse_nav(payload: str) -> Optional[tuple[float, float]]:
    if not payload.startswith("NAV:"):
        return None
    values = payload[4:].split(",")
    if len(values) != 9:
        return None
    lat, lon = float(values[0]), float(values[1])
    if lat == 0.0 and lon == 0.0:
        return None
    return lat, lon


def parse_dr(payload: str) -> Optional[DrFrame]:
    if not payload.startswith("DR:"):
        return None
    values = payload[3:].split(",")
    if len(values) != 10:
        return None
    return DrFrame(
        lat=float(values[0]),
        lon=float(values[1]),
        yaw=float(values[2]),
        relative_yaw=float(values[3]),
        gps_error_m=float(values[8]),
        active=values[9] == "1",
    )


def main() -> None:
    try:
        import matplotlib.pyplot as plt
        import serial
    except ImportError as exc:
        raise SystemExit(
            "Eksik paket. Kurulum: python3 -m pip install --user pyserial matplotlib"
        ) from exc

    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Örnek: /dev/ttyUSB0")
    parser.add_argument("baud", nargs="?", type=int, default=57600)
    args = parser.parse_args()

    gps_lat: list[float] = []
    gps_lon: list[float] = []
    dr_lat: list[float] = []
    dr_lon: list[float] = []
    last_dr: Optional[DrFrame] = None

    plt.ion()
    fig, ax = plt.subplots()
    gps_line, = ax.plot([], [], "-", label="GPS izi")
    dr_line, = ax.plot([], [], "--", label="PWM + IMU tahmini")
    ax.set_xlabel("Boylam")
    ax.set_ylabel("Enlem")
    ax.legend()

    with serial.Serial(args.port, args.baud, timeout=1) as port:
        while plt.fignum_exists(fig.number):
            raw = port.readline().decode("ascii", errors="ignore")
            payload = checked_payload(raw)
            if payload is None:
                plt.pause(0.01)
                continue

            nav = parse_nav(payload)
            if nav is not None:
                gps_lat.append(nav[0])
                gps_lon.append(nav[1])

            dr = parse_dr(payload)
            if dr is not None and dr.active:
                last_dr = dr
                dr_lat.append(dr.lat)
                dr_lon.append(dr.lon)

            gps_line.set_data(gps_lon, gps_lat)
            dr_line.set_data(dr_lon, dr_lat)
            ax.relim()
            ax.autoscale_view()
            ax.set_aspect("equal", adjustable="datalim")

            if last_dr is not None:
                ax.set_title(
                    f"Yaw {last_dr.yaw:.1f}° | Merkeze göre "
                    f"{last_dr.relative_yaw:+.1f}° | GPS farkı {last_dr.gps_error_m:.1f} m"
                )
            fig.canvas.draw_idle()
            plt.pause(0.01)


if __name__ == "__main__":
    main()
