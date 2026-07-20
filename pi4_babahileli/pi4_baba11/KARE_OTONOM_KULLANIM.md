# OTONOM tuşuna basınca zaman tabanlı kare görevi

Bu sürüm gerçek GPS waypoint otonomisini **bilerek devre dışı bırakır** ve test amaçlı açık çevrim bir kare hareketi uygular.

## Başlatma

Aşağıdaki komutlardan herhangi biri kareyi başlatır:

- `CMD:MOD:1`
- `CMD:START`

Arayüz ikisini peş peşe gönderirse görev ikinci komutla baştan başlamaz; aktif kaldığı yerden devam eder.

## Uygulanan hareket

1. M3 ve M4: 10 saniye, PWM 400 ile ileri
2. M1: 2.5 saniye, PWM 400 ile sola dönüş
3. Bu akış 4 kenar tamamlanıncaya kadar sürer
4. Dördüncü ileri hareketten sonra bütün motorlar 0 yapılır

Toplam nominal süre: `4 × 10 + 3 × 2.5 = 47.5 saniye`.

GPS, IMU, origin veya rota hazır olmasa bile kare görevi çalışır.

## Güvenlik davranışı

Şunlardan biri gelirse kare görevi aynı kontrol turunda iptal edilir ve motor isteği sıfıra düşer:

- `CMD:STOP`
- Manuel moda geçiş: `CMD:MOD:0`
- Telemetri seri/RF bağlantısının kopması

İlk testi pervaneler sökülüyken veya araç güvenli biçimde sabitlenmişken yapın.

## Süre ve güç ayarı

`src/beyin.rs` dosyasının üst kısmındaki değerler:

```rust
const KARE_KENAR_SURESI: Duration = Duration::from_secs(10);
const KARE_DONUS_SURESI: Duration = Duration::from_millis(2500);
const KARE_ILERI_PWM: u16 = 400;
const KARE_SOL_DONUS_PWM: u16 = 400;
```

Tekne 90 dereceden az dönüyorsa `KARE_DONUS_SURESI` değerini artırın. Fazla dönüyorsa azaltın.

Örnek:

```rust
const KARE_DONUS_SURESI: Duration = Duration::from_millis(3200);
```

## Tekne sağa dönüyorsa

Fiziksel motor yönünüz ters olabilir. `src/beyin.rs` içinde kare dönüş bölümündeki:

```rust
motor_esleme.sol
```

değerini:

```rust
motor_esleme.sag
```

olarak değiştirin.

## Beklenen terminal çıktısı

```text
OTONOM TUŞU: GPS/IMU/ROTA bağımsız kare görevi başladı. Kenar 1/4 ileri.
KARE DURUM | kenar=1/4 aşama=Ileri kalan=... PWM=[0,0,400,400]
KARE: 1. kenar tamamlandı; 2500 ms sola dönüş başladı.
KARE DURUM | kenar=1/4 aşama=SolaDon kalan=... PWM=[400,0,0,0]
...
KARE GÖREVİ TAMAMLANDI: 4. kenar bitti, bütün motorlar durduruldu.
```

Motor eşlemesi varsayılan olarak M1=sol dönüş, M2=sağ dönüş, M3 ve M4=ileri kabul edilmiştir.
