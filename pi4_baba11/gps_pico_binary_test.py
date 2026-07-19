#!/usr/bin/env python3
import glob
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial eksik: python3 -m pip install --user pyserial", file=sys.stderr)
    raise SystemExit(2)

HEADER = b"\xAA\xCC"
FRAME_LEN = 33


def candidates():
    preferred = glob.glob('/dev/serial/by-id/*IDA_GPS_001*')
    return preferred + [p for p in glob.glob('/dev/ttyACM*') if p not in preferred]


def read_frame(port: str):
    print(f"Deneniyor: {port}")
    with serial.Serial(port, 115200, timeout=0.5) as ser:
        deadline = time.monotonic() + 8
        state = 0
        while time.monotonic() < deadline:
            b = ser.read(1)
            if not b:
                continue
            v = b[0]
            if state == 0:
                state = 1 if v == 0xAA else 0
            else:
                if v == 0xCC:
                    rest = ser.read(31)
                    if len(rest) != 31:
                        state = 0
                        continue
                    frame = HEADER + rest
                    calc = 0
                    for x in frame[2:32]:
                        calc ^= x
                    if calc != frame[32]:
                        print(f"  CRC hata hesap={calc:02X} gelen={frame[32]:02X}")
                        state = 0
                        continue
                    fix, sats = frame[2], frame[3]
                    lon, lat, alt, speed, heading = struct.unpack_from('<iiiii', frame, 4)
                    ts = struct.unpack_from('<Q', frame, 24)[0]
                    if fix == 0 and sats == 255:
                        print(f"  PICO OK | GPS baud={alt} | ham UART bayt={speed} | çözümlenen={heading} | pico_ms={ts}")
                    else:
                        print(f"  GPS | fix={fix} uydu={sats} lat={lat/1e7:.7f} lon={lon/1e7:.7f} alt_mm={alt} hiz={speed} yon={heading} pico_ms={ts}")
                    return True
                state = 1 if v == 0xAA else 0
    print("  8 saniyede AA CC paketi gelmedi")
    return False


ports = [sys.argv[1]] if len(sys.argv) > 1 else candidates()
if not ports:
    print("Seri cihaz bulunamadı. ls -l /dev/ttyACM* /dev/serial/by-id/ çalıştırın.")
    raise SystemExit(1)

for p in ports:
    try:
        if read_frame(p):
            raise SystemExit(0)
    except (serial.SerialException, PermissionError) as e:
        print(f"  Açılamadı: {e}")

raise SystemExit(1)
