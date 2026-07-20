# OTONOM tuşuna basınca GPS'siz, IMU dönüşlü kare görevi

Bu sürüm kare görevi sırasında **GPS, origin ve waypoint rotasını kullanmaz**.

- Düz kenarlar süreyle sürülür.
- Her köşede sola dönüş motoru çalışır.
- IMU yaw değişimi 90 dereceye ulaşınca dönüş bitirilir.
- Dönüş için sabit süre kullanılmaz.

## Başlatma

Aşağıdaki komutlardan herhangi biri kareyi başlatır:

- `CMD:MOD:1`
- `CMD:START`

Arayüz iki komutu peş peşe gönderirse aktif görev sıfırlanmaz.

IMU henüz veri vermiyorsa komut kaybolmaz. Program:

```text
OTONOM alındı fakat IMU hazır değil; görev IMU gelince otomatik başlayacak.
```

mesajını verir ve ilk taze IMU paketi geldiğinde GPS beklemeden görevi başlatır.

## Hareket akışı

1. M3 ve M4, 10 saniye PWM 400 ile ileri.
2. M1 sola dönüş motoru çalışır.
3. Program IMU yaw farklarını toplar.
4. Toplam dönüş 90 dereceye ulaştığında motor durur ve sonraki kenar başlar.
5. Dört ileri kenar ve üç adet 90 derece dönüş sonunda motorlar sıfırlanır.

GPS bağlı olmasa, fix vermese veya uydu sayısı sıfır olsa bile kare görevi çalışır. IMU zorunludur.

## IMU dönüş hesabı

Yaw 359 dereceden 1 dereceye geçse bile fark `+2°` olarak hesaplanır. İlk gerçek yaw değişiminin işareti otomatik algılanır; bu nedenle IMU'nun sola dönüşte yaw değerini artırması veya azaltması fark etmez.

Dönüşte hedefe son 20 derece kaldığında PWM 400'den 280'e düşürülerek aşma azaltılır.

## Güvenlik

Şunlardan biri olursa görev iptal edilir ve motorlar sıfırlanır:

- Acil durdurma / `CMD:STOP`
- Manuel moda geçiş / `CMD:MOD:0`
- Telemetri bağlantısının kopması
- IMU verisinin 2 saniyeden uzun süre gelmemesi
- Bir dönüşün 10 saniye içinde 90 dereceye ulaşmaması

## Ayarlar

`src/beyin.rs` dosyasının üstündeki değerler:

```rust
const KARE_KENAR_SURESI: Duration = Duration::from_secs(10);
const KARE_ILERI_PWM: u16 = 400;
const KARE_SOL_DONUS_PWM: u16 = 400;
const KARE_SOL_DONUS_YAVAS_PWM: u16 = 280;
const KARE_DONUS_ACISI_DEG: f32 = 90.0;
const KARE_DONUS_TOLERANSI_DEG: f32 = 3.0;
const KARE_DONUS_YAVAS_BOLGE_DEG: f32 = 20.0;
const KARE_DONUS_MAX_SURE: Duration = Duration::from_secs(10);
```

Kenarları uzatmak için yalnız `KARE_KENAR_SURESI` değerini artırın.

## Tekne sağa dönüyorsa

Fiziksel motor eşlemesi ters olabilir. Kare dönüş bölümündeki:

```rust
motor_esleme.sol
```

ifadesini:

```rust
motor_esleme.sag
```

olarak değiştirin. IMU yön işaretini ayrıca değiştirmeniz gerekmez; yazılım yaw değişim yönünü otomatik algılar.

## Beklenen terminal çıktısı

```text
OTONOM TUŞU: GPS/ROTA kullanılmadan kare görevi başladı. Dönüşler IMU yaw ile 90 derece kontrol edilecek.
KARE DURUM | kenar=1/4 aşama=Ileri ... GPS_KULLANILMIYOR PWM=[0,0,400,400]
KARE: 1. kenar tamamlandı; IMU yaw=42.30°. 90° sola dönüş başladı.
KARE IMU yönü algılandı: işaret=+1, hedef_yaw=132.30°
KARE DURUM | kenar=1/4 aşama=SolaDon ... dönüş=54.20/90.00° ... PWM=[400,0,0,0]
KARE: IMU dönüşü tamamlandı; başlangıç=42.30°, bitiş=130.10°, ölçülen=87.80°. 2. kenar ileri başlıyor.
```

Varsayılan motor eşlemesi M1=sol dönüş, M2=sağ dönüş, M3 ve M4=ileri kabul edilir.
