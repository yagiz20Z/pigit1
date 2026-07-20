# GPS yanında PWM + IMU tahmini rota

## Çalışma mantığı

1. İlk güvenilir GPS fix ve taze IMU yaw geldiğinde tahmini rota başlatılır.
2. O anki GPS koordinatı tahmini rotanın başlangıç noktası olur.
3. O anki IMU yaw değeri `0° göreli açı` kabul edilir.
4. İki ileri motorun PWM ortalaması yaklaşık ileri hıza çevrilir.
5. Sağ ve sol yatay motorların PWM farkı yaklaşık yanal hıza çevrilir.
6. Gövde hızları IMU yaw kullanılarak kuzey/doğu hareketine dönüştürülür.
7. Tahmini konum GPS'e düzeltilmeden bağımsız ilerler.
8. Gerçek GPS ve tahmini konum arasındaki fark metre olarak hesaplanır.

Bu yöntem gerçek odometri değildir. Akıntı, rüzgâr, batarya gerilimi, motor verimi,
yük ve gövde sürüklemesi nedeniyle zamanla drift oluşur. Zaten ikinci izin amacı
bu farkı görünür kılmak ve motor-hız kalibrasyonu yapabilmektir.

## Kalibrasyon

Başlangıç değerleri:

```bash
export IDA_DR_MAX_FORWARD_MPS=2.0
export IDA_DR_MAX_LATERAL_MPS=0.45
export IDA_DR_PWM_DEADZONE=0.03
```

- `IDA_DR_MAX_FORWARD_MPS`: iki ileri motor 1000 iken yaklaşık hız.
- `IDA_DR_MAX_LATERAL_MPS`: sağ/sol yatay motor 1000 iken yaklaşık yanal hız.
- `IDA_DR_PWM_DEADZONE`: hareket üretmeyen normalize PWM oranı.

Kalibrasyon için düz hatta sabit PWM ile 10-20 saniye gidin. GPS mesafesini süreye
bölerek gerçek hızı bulun ve `IDA_DR_MAX_FORWARD_MPS` değerini güncelleyin.

## Telemetri

Yeni üçüncü telemetri satırı:

```text
DR:lat,lon,yaw,goreli_yaw,referans_yaw,ileri_mps,yatay_mps,mesafe_m,gps_fark_m,aktif*CS
```

Örnek:

```text
DR:41.0000102,29.0000043,87.20,12.20,75.00,0.800,0.000,8.45,1.32,1*AB
```

## Yeniden merkezleme

Arayüz şu payload'ı normal telemetri checksum sistemiyle yollar:

```text
CMD:DR:RESET*61\n
```

Checksum hesaplanmadan önceki payload `CMD:DR:RESET` değeridir.

Komut geldiğinde ilk taze/geçerli GPS+IMU çifti yeni başlangıç olur. GPS izi silinmek
zorunda değildir; arayüz isterse yalnız tahmini izi temizleyebilir.

## Test görüntüleyicisi

```bash
python3 -m pip install --user pyserial matplotlib
python3 dr_dual_track_viewer.py /dev/ttyUSB0 57600
```

Düz çizgi GPS, kesikli çizgi PWM+IMU tahminidir.
