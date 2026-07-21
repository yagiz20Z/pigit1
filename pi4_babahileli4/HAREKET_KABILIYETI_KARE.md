# OTONOM tuşuyla hareket kabiliyeti kare görevi

Bu sürümde arayüzden **OTONOM** tuşuna basılmasıyla gönderilen `CMD:MOD:1` komutu kare hareketini doğrudan başlatır. Ayrı bir START komutu zorunlu değildir.

## Hareket sırası

1. M3 ve M4, PWM 400 ile 6 saniye ileri çalışır.
2. M3 ve M4 durur.
3. M1 çalışır ve BNO085 yaw başlangıç açısı kaydedilir.
4. Başlangıç açısına göre 90 derece yön değişimi tamamlanınca M1 durur.
5. Aynı sıra üç dönüş boyunca tekrarlanır.
6. Toplam dört ileri kenar ve üç adet 90 derece sol dönüş yapılır.
7. Dördüncü ileri hareket tamamlanınca bütün motorlar sıfırlanır.

## Motor eşlemesi

- M1: sol dönüş motoru
- M2: sağ dönüş/düzeltme motoru
- M3: arka ileri motor 1
- M4: arka ileri motor 2

Arayüz farklı motor sırası kullanıyorsa görevden önce `CMD:MAP:1,3,2,4` eşlemesinin doğru olduğundan emin olun.

## Güvenlik

- IMU hazır değilse OTONOM isteği bekletilir; ilk geçerli IMU verisinde görev başlar.
- IMU verisi kesilirse motorlar durur.
- Bir dönüş 12 saniyede tamamlanamazsa görev iptal edilir.
- Manuel veya acil durdurma seçilirse kare görevi anında kesilir.

## Ayarlanabilir değerler

`src/beyin.rs` içindeki değerler:

```rust
const KARE_KENAR_SURESI: Duration = Duration::from_secs(6);
const KARE_ILERI_PWM: u16 = 400;
const KARE_SOL_DONUS_PWM: u16 = 400;
const KARE_DONUS_ACISI_DEG: f32 = 90.0;
const KARE_KENAR_SAYISI: u8 = 4;
```
