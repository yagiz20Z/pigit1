# NEO-M8N doğrudan USB/NMEA entegrasyonu

Bu sürümde GPS için Pico veya STM kullanılmaz. NEO-M8N'in USB çıkışı doğrudan
Raspberry Pi'ye bağlanır ve Linux seri portundan gelen NMEA cümleleri Rust içinde
çözülür.

İşlenen cümleler:

- `GNGGA` / `GPGGA`: enlem, boylam, uydu sayısı, fix ve yükseklik
- `GNRMC` / `GPRMC`: enlem, boylam, hız ve rota yönü
- `GNGSA` / `GPGSA`: 1=no fix, 2=2D, 3=3D fix tipi
- `GNVTG` / `GPVTG`: yer hızı ve rota yönü

Çıktı mevcut `GpsVeri` yapısına çevrilir:

- `enlem`, `boylam`: derece × 10.000.000
- `yukseklik_mm`: milimetre
- `hiz`: mm/s
- `yonelim`: derece × 100.000
- `zaman_ms`: Raspberry Pi UNIX zamanı

## 1. GPS portunu bul

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Doğrudan u-blox USB cihazı çoğunlukla `/dev/ttyACM...` veya
`/dev/serial/by-id/...u-blox...` şeklinde görünür. USB-TTL dönüştürücü ile bağlıysa
`/dev/ttyUSB...` olarak görünebilir.

## 2. GPS'i tek başına test et

```bash
cd pi4_baba6211
python3 -m pip install --user pyserial
python3 gps_usb_test.py /dev/ttyACM1 115200
```

Baud bilinmiyorsa yalnızca portu ver:

```bash
python3 gps_usb_test.py /dev/ttyACM1
```

Script 115200, 9600, 38400, 57600, 19200 ve 4800 hızlarını dener. Doğru bağlantıda
`NMEA:` ve `ISLENMIS:` satırları görülür.

## 3. Yalnız Rust GPS görevini test et

```bash
cargo run --release --bin gps_usb_only -- /dev/ttyACM1 115200
```

Baud bilinmiyorsa ikinci argüman ilk denenecek hızdır; görev diğer yaygın hızları
otomatik tarar.

## 4. Ana programı çalıştır

```bash
export IDA_GPS_PORT=/dev/serial/by-id/GPS_CIHAZININ_TAM_ADI
export IDA_GPS_BAUD=115200
cargo run --release --bin pi4_baba
```

GPS 9600 kullanıyorsa:

```bash
export IDA_GPS_BAUD=9600
```

`IDA_GPS_BAUD` yalnızca ilk denenen hızdır. Rust GPS görevi geçerli NMEA bulamazsa
diğer yaygın hızları otomatik tarar.

## Beklenen ana program çıktısı

```text
GPS USB/NMEA portu açılmaya çalışılıyor: /dev/... @ 115200 baud
GPS doğrudan USB/NMEA bağlantısı kuruldu: /dev/... @ 115200 baud
[GPS USB OK] ... fix=3 uydu=10 enlem=... boylam=... alt_mm=... hiz_mm_s=...
```

- `fix=1`: uydu kilidi yok
- `fix=2`: 2B konum
- `fix=3`: 3B konum; mevcut otonomi kodunun istediği durum
- `uydu < 6`: mevcut `beyin.rs` güvenliği otonom sürüşe izin vermez

## Değişen dosyalar

- `src/sensorler/m8n.rs`: USB NMEA okuyucu ve parser
- `src/haberlesme.rs`: doğrudan USB port bulma ve `IDA_GPS_BAUD`
- `gps_usb_test.py`: ham NMEA ve işlenmiş veri testi
- `src/bin/gps_usb_only.rs`: yalnızca Rust GPS katmanını çalıştıran test binary
- `gps_pico_binary_test.py`: eski Pico ikili paket testi, yalnızca yedek amaçlı
