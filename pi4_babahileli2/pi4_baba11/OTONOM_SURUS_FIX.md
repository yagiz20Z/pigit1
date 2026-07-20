# Otonom sürüş düzeltmesi

Bu sürüm arayüzdeki oyun kolunun otonom görevi istemeden MANUEL moda geri çevirmesiyle uyumludur.
Arayüz ve Pi projesi birlikte güncellenmelidir.

## Düzeltilen ana hata

Eski arayüz, oyun kolu daha önce aktif olmuşsa otonoma geçişten hemen sonra `CMD:MAN:0,0`
gönderiyordu. `pi4_baba11` içinde her `CMD:MAN` paketi, değerleri sıfır olsa bile aracı MANUEL
moda geçirir. Bu nedenle rota ve START alınsa bile görev birkaç milisaniye sonra iptal oluyordu.

## Pi tarafındaki ek düzeltmeler

- `STATE:ROTA` ile rota doğrulaması ve `ACK:MAP`, `ACK:MOD`, `ACK:START`, `ACK:STOP` cevapları eklendi.
- NAV telemetrisi aktif waypoint, toplam waypoint, hedef mesafesi, GPS/IMU/origin/RF sağlık alanları ve otonom durum kodu içerir.
- Su üstü navigasyonu için geçerli GPS eşiği 2B fix + en az 4 uydu yapıldı.
- IMU tazelik süresi 500 ms'den 2 saniyeye çıkarıldı.
- Origin oluşturma 10 örnekten 5 örneğe indirildi.
- 4 m kare rota için waypoint toleransı 2.5 m'den 1.0 m'ye indirildi.

## Çalıştırma

```bash
cd pi4_baba11
cargo run --release
```

Kalıcı port yolları önerilir:

```bash
export IDA_TEL_PORT=/dev/serial/by-id/TELEMETRI_CIHAZI
export IDA_MOTOR_PORT=/dev/serial/by-id/STM_USB_TTL_CIHAZI
export IDA_GPS_PORT=/dev/serial/by-id/GPS_CIHAZI
export IDA_IMU_PORT=/dev/serial/by-id/IMU_CIHAZI
cargo run --release
```

## Arayüzde beklenen sıra

1. Telemetriye bağlanın.
2. Sağlık satırında `GPS: OK`, `IMU: OK`, `Origin: HAZIR`, `RF: BAĞLI` görülmesini bekleyin.
3. `GPS'TEN 4 m KARE` düğmesine basın.
4. `START` veya `ROTA + OTONOM + START` düğmesine basın. İki düğme de artık tam görev sırasını çalıştırır.
5. Otonom durum önce `HEDEFE DÖNÜŞ YAPILIYOR`, sonra `HEDEFE İLERLİYOR` olmalıdır.

Pervaneli testi ilk olarak tekne sabitlenmişken ve acil durdurma erişilebilirken düşük riskli ortamda yapın.


## V2 ek düzeltmesi

Bu pakette ayrıca `OTONOM_SURUS_FIX_V2.md` bulunur. V2; ilk komutta PING zorunluluğunu kaldırır, START/ROTA paket sırasını dayanıklı hâle getirir ve otonomdayken gelen `CMD:MAN:0,0` paketlerini Pi tarafında da engeller.
