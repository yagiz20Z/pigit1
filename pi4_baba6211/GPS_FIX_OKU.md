# GPS HABERLEŞME DÜZELTMESİ

Bu paket iki temel sorunu giderir:

1. Pico firmware'i yalnızca 115200 varsaymıyor. Açılışta 4800, 9600, 19200, 38400, 57600 ve 115200 hızlarında M8N'e UBX-CFG-PRT gönderip GPS'i 115200'e taşıyor.
2. Pi4_baba GPS'i `/dev/ttyACM0` veya `/dev/ttyACM1` sırasına göre seçmiyor. Önce `IDA_GPS_PORT`, sonra `/dev/ida-gps`, sonra USB seri kimliği `IDA_GPS_001` aranıyor.

## Kablolama

- M8N TX -> Pico GP5 (fiziksel pin 7)
- M8N RX -> Pico GP4 (fiziksel pin 6)
- M8N GND -> Pico GND

## Sıra

Önce `gps_m8n_auto_baud_fix` firmware'ini Pico 2 W'ye yükleyin. Sonra Pico'yu Pi'ye takıp şu testi çalıştırın:

```bash
python3 gps_usb_test.py
```

Beklenen teşhislerden biri:

- `ham_uart_bayt=0`: GPS TX kablosu, GND veya besleme yok.
- `ham_uart_bayt` artıyor ama `cozumlenen_ornek=0`: UART verisi var, ancak GPS protokolü/elektriksel veri bozuk.
- `[GPS OK] fix=0`: haberleşme tamam, uydu kilidi bekleniyor.
- `[GPS OK] fix=3`: tam çalışıyor.

Pi ana kodunu çalıştırırken sabit yol otomatik bulunacaktır. Elle vermek gerekirse:

```bash
export IDA_GPS_PORT=/dev/serial/by-id/usb-StarsOfHydro_M8N_GPS_USB_Binary_IDA_GPS_001-if00
cargo run --release --bin pi4_baba
```
