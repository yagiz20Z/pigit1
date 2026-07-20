# pi4_baba62 — v12 güvenlik düzenlemeleri

Bu sürümde aşağıdaki değişiklikler yapıldı:

1. Motor komutu `watch::channel` ile yalnızca son değer olarak tutulur.
2. STM bağlantısı açılırken ve program kapanırken sıfır motor paketi gönderilir.
3. GPS freshness sınırı 2 saniye, IMU freshness sınırı 500 ms'dir.
4. Otonom/Eve Dönüş sensör verisi bayatlarsa motor istekleri sıfır olur.
5. Telemetri için `CMD:PING` zorunludur; 4 saniyelik timeout vardır.
6. Telemetri daha önce hiç bağlanmadıysa başlangıçta Eve Dönüş tetiklenmez.
7. Rota, HOME ve manuel komutları aralık/finite/fazladan alan açısından doğrulanır.
8. Manuel yatay giriş hem normalize `-1..1` hem eski `-90..90` derece formatını kabul eder.
9. GPS hızı telemetriye `mm/s / 1000 = m/s` olarak gönderilir.
10. GPS/IMU okuma işlemlerine header ve paket timeout'ları eklendi.
11. Lidar modülündeki sabit geçersiz port ve `expect()` kaldırıldı; bağlantı yeniden deneme yapısına geçirildi. Lidar henüz ana haberleşmede başlatılmaz.
12. Son doğrulanan varsayılan port sırası GPS=`/dev/ttyACM1`, IMU=`/dev/ttyACM0` yapıldı.

## STM tarafında yapılması gereken

STM firmware'i bu ZIP içinde olmadığı için STM watchdog'u burada eklenemedi. STM, 300–500 ms boyunca geçerli Pi motor paketi alamazsa dört motor PWM'ini sıfırlamalıdır.
