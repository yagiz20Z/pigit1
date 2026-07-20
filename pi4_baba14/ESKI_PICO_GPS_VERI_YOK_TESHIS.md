# GPS veri gelmiyor teşhisi

Bu sürüm, M8N'den geçerli cümle alınmasa bile Pico USB üzerinden 3 saniyede bir teşhis paketi yollar.

Pi terminalinde görülebilecek durumlar:

- `[GPS PICO OK / UART DURUM] ... ham_uart_bayt=0`: Pico ve USB çalışıyor; M8N TX hattından hiç veri yok. Kablo, GND, besleme, yanlış pin veya GPS'in kapalı olması kontrol edilir.
- `ham_uart_bayt` artıyor, `cozumlenen_ornek=0`: UART'ta elektriksel veri var ancak 9600 baud/protokol uyuşmuyor veya veri bozuk. Önce GPS baud hızı kontrol edilir.
- `[GPS OK] fix=0 ...`: M8N cümlesi çözümleniyor ancak henüz uydu konum kilidi yok. Açık alanda beklenir.
- `[GPS OK] fix=3 ...`: GPS tamamen çalışıyor.
- `[GPS USB SESSIZ]`: Pi yanlış seri portu açmış veya Pico'da eski firmware çalışıyor.

## Pico fiziksel pinleri

`GP4` ve `GP5`, kart üzerindeki fiziksel 4 ve 5 numaralı pinler değildir:

- M8N TX -> Pico **GP5**, fiziksel pin **7**
- M8N RX -> Pico **GP4**, fiziksel pin **6**
- M8N GND -> Pico GND, örneğin fiziksel pin **8**

Sadece veri okumak için M8N TX -> GP5 ve ortak GND yeterlidir. M8N RX bağlantısı yapılandırma komutları için kullanılır.

## Sabit USB yolu

Pi'de:

```bash
ls -l /dev/serial/by-id/
```

`StarsOfHydro` ve `IDA_GPS_001` içeren yolu kullanın:

```bash
IDA_GPS_PORT=/dev/serial/by-id/usb-StarsOfHydro_M8N_GPS_USB_Binary_IDA_GPS_001-if00 \
  cargo run --release
```

Gerçek ad sistemde biraz farklıysa `ls` çıktısındaki tam yolu kopyalayın.
