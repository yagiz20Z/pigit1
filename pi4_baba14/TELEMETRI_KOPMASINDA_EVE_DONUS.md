# Telemetri kopmasında güvenli dönüş

Bu sürümde yalnızca telemetri kopması için güvenli dönüş davranışı eklendi.

## Davranış

- Telemetri UART'ı kapanır, okuma/yazma hatası verir veya heartbeat zaman aşımına uğrarsa `AracMod::EveDonus` başlatılır.
- Araç güvenli dönüş koordinatına mevcut GPS + IMU navigasyonuyla gider.
- Telemetri görevi aynı anda portu her 1 saniyede yeniden açmayı dener.
- Telemetri gönderim kuyruğu dolsa bile navigasyon döngüsü bloklanmaz.
- Güvenli dönüş noktasına 2.5 m kala motorlar sıfırlanır ve araç `GorevBekliyor` moduna geçer.
- Geçerli GPS veya dönüş koordinatı yoksa araç rastgele hareket etmez; motorları durdurur.

## Güvenli dönüş koordinatı

Öncelik YKİ'den gönderilen koordinattadır:

`CMD:HOME:<enlem>,<boylam>*<CHECKSUM>\n`

Örnek payload:

`CMD:HOME:41.0256480,28.9741100`

Bu komut gönderilmezse Pi, ilk 10 güvenilir GPS örneğinden çıkardığı başlangıç origin'ini otomatik olarak dönüş noktası kabul eder.

## RF kopmasını algılamak için heartbeat

USB/UART cihazı fiziksel olarak çıkarsa kopma doğrudan algılanır. RF hava bağlantısı kesildiğinde USB seri cihazı açık kalabileceği için YKİ şu paketi saniyede bir göndermelidir:

`CMD:PING*<CHECKSUM>\n`

İlk `CMD:PING` alındıktan sonra watchdog aktif olur. 4 saniye geçerli YKİ paketi alınamazsa telemetri kopmuş kabul edilir. Eski YKİ yazılımıyla geriye uyumluluk için ilk PING gelmeden heartbeat zaman aşımı uygulanmaz.

## Not

Telemetri yeniden bağlansa bile başlamış güvenli dönüş hedefe kadar devam eder. YKİ `CMD:STOP`, `CMD:MOD`, `CMD:MAN`, yeni rota veya diğer geçerli komutlarla modu değiştirebilir.
