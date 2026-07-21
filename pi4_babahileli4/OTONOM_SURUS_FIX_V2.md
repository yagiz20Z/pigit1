# Otonom sürüş düzeltmesi V2

Bu sürüm, otonom moda geçildiği hâlde motor isteklerinin sıfır kalmasına yol açan üç kilidi Pi tarafında giderir.

## Giderilen kilitler

1. **İlk komut için `CMD:PING` zorunluluğu kaldırıldı.**
   - İlk geçerli `CMD:ROTA`, `CMD:MOD`, `CMD:START` veya `CMD:PING` paketi RF oturumunu kurar.
   - Arayüz hiç PING kullanmıyorsa 4 saniyede sahte bağlantı kopması oluşmaz.
   - Arayüz PING kullanıyorsa 4 saniyelik watchdog korunur.

2. **Otonomdayken gelen `CMD:MAN:0,0` artık görevi iptal etmez.**
   - Manuel paketler yalnız araç açıkça `CMD:MOD:0` ile Manuel moda alınmışsa kabul edilir.
   - Oyun kolu döngüsünden kalan sıfır paketleri Otonom/Eve Dönüş/Görev Bekliyor modlarında yok sayılır.

3. **START/ROTA sırası dayanıklı hâle getirildi.**
   - `START` rotadan önce gelirse istek kaybolmaz; rota geldiği anda otonom başlar.
   - Aynı rota ACK gecikmesi nedeniyle tekrar gelirse aktif waypoint indeksi sıfırlanmaz.

## Motor varsayılan eşlemesi

- M1: sol yatay
- M2: sağ yatay
- M3 ve M4: ileri iticiler

Varsayılan `MotorEsleme`: `sol=1, ileri1=3, sağ=2, ileri2=4`.
YKİ `CMD:MAP:sol,ileri1,sag,ileri2` gönderirse bu eşleme çalışma anında değişebilir.

## Beklenen komut sırası

Aşağıdaki sıra önerilir fakat artık paket sırası gecikmelerine karşı dayanıklıdır:

```text
CMD:MAP:1,3,2,4
CMD:ROTA:<lat,lon;lat,lon;lat,lon;lat,lon>
CMD:MOD:1
CMD:START
```

Her pakette mevcut protokoldeki `*CHECKSUM` ve satır sonu bulunmalıdır.

## Konsolda yeni teşhis satırı

Otonom moddayken her 2 saniyede şu satır basılır:

```text
OTONOM DURUM | rota=... GPS[taze=...,fix=...,uydu=...,hazır=...] IMU[...] origin=... hedef_mesafe=... PWM=[...]
```

Motorlar sıfır kalırsa bu satır doğrudan hangi koşulun eksik olduğunu gösterir. Hareket için rota, güncel GPS (`fix >= 2`, `uydu >= 4`), güncel/geçerli IMU ve hazırlanmış origin birlikte gereklidir.
