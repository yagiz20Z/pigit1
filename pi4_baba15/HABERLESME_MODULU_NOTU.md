# Haberleşme Modülü Ayrımı

Bu sürüm, `pi4_baba61` kodu temel alınarak hazırlanmıştır.

## Değişiklik

- GPS, IMU, YKİ telemetrisi ve STM32 motor UART görevleri `src/haberlesme.rs` içine taşındı.
- `main.rs` artık port ve UART ayrıntılarını içermez.
- `beyin.rs` değiştirilmedi; 4 nokta takibi, PID, manuel/otonom mod ve motor kararı aynı yerde kaldı.
- Mapping ve aktif tahmin için ayrı modül oluşturulmadı. Daha sonra eklenecek mantık `beyin.rs` içinde kalacaktır.
- Sensör sürücüleri (`sensorler/m8n.rs`, `sensorler/bno085.rs`) korunmuştur.

## Çalıştırma

```bash
cargo run --release --bin pi4_baba
```

Portları değiştirmek için:

```bash
IDA_TEL_PORT=/dev/ttyUSB0 \
IDA_MOTOR_PORT=/dev/ttyUSB1 \
IDA_GPS_PORT=/dev/ttyACM0 \
IDA_IMU_PORT=/dev/ttyACM1 \
cargo run --release --bin pi4_baba
```
