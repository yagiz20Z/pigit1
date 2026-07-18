# GPS entegrasyon notu

Kullanıcının mevcut Pi 4 kod yapısı korunmuştur.

Kodda değiştirilen tek mevcut dosya:

- `src/sensorler/m8n.rs`

Değişiklik:

```rust
// Önceki, BNO085 ile çakışan başlık
0xAA 0xBB

// GPS Pico firmware başlığı
0xAA 0xCC
```

Paket gövdesi, checksum, `GpsVeri`, `main.rs`, motor, BNO085, telemetri ve otonomi kodları değiştirilmemiştir.
