# IMU ve SD Kart Kaydı

Bu paket, mevcut BNO085 IMU haberleşmesini kullanır ve araç verilerini Raspberry Pi
üzerindeki depolamaya otomatik kaydeder.

## Varsayılan konum

```text
$HOME/ida_logs/oturum_<unix_ms>/
```

Raspberry Pi işletim sistemi SD karttan çalışıyorsa bu dizin doğrudan SD kart
üzerindedir.

## Harici SD kart

Harici kartın bağlama noktasını programı çalıştırmadan önce verin:

```bash
export IDA_LOG_DIR=/media/alinux/SD_KART/ida_logs
cargo run --release --bin pi4_baba
```

## Oluşan dosyalar

- `imu.csv`: roll, pitch, yaw, gyro ve ivme; her yeni IMU paketinde.
- `gps.csv`: fix, uydu, konum, yükseklik, hız ve yönelim; her yeni GPS paketinde.
- `tum_veriler.csv`: 10 Hz birleşik GPS + IMU + setpoint + motor + mod + dead-reckoning kaydı.
- `oturum_bilgisi.txt`: oturum başlangıç zamanı ve kayıt bilgisi.

CSV dosyalarının ilk satırı başlıktır. Kayıtlar her saniye flush edilir, her beş
saniyede depolamaya senkronlanır. Normal Ctrl+C kapanışında son tamponlar da
`sync_all` ile yazılır.

## IMU portu

Varsayılan:

```text
/dev/ttyACM0 @ 115200
```

Kalıcı cihaz yolu önerilir:

```bash
export IDA_IMU_PORT=/dev/serial/by-id/usb-Embassy_USB-serial_logger-if00
```
