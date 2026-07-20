# Avalonia çift iz entegrasyonu

## Önemli entegrasyon notu

Bu pakette kullanıcının gerçek Avalonia arayüz projesi bulunmadığı için harita
kütüphanesinin somut `Polyline` nesnelerine doğrudan bağlanan satırlar otomatik
yamalanamadı. Aşağıdaki sınıflar telemetriyi hazırlar ve iki iz listesini üretir;
`UpdateGpsPolyline` ile `UpdateEstimatedPolyline` metotları mevcut harita
bileşenindeki gerçek katmanlara bağlanmalıdır.

# Arayüz: GPS ve PWM/IMU tahmini izi birlikte gösterme

Pi artık her telemetri çevriminde üç satır gönderir:

- `NAV:` gerçek GPS izi
- `MOT:` motor komutları
- `DR:` PWM + IMU tabanlı bağımsız tahmini iz

## DR paketi

```text
DR:enlem,boylam,mutlak_yaw,goreli_yaw,referans_yaw,ileri_hiz,yatay_hiz,toplam_mesafe,gps_fark,aktif*CS
```

Örnek:

```text
DR:41.0123456,29.0123456,82.40,17.20,65.20,0.800,-0.040,12.50,1.80,1*CS
```

- `mutlak_yaw`: IMU'nun dünya yönü; haritadaki tekne ikonunu döndürmek için.
- `goreli_yaw`: tahmini iz başlatıldığı andaki yön `0°` kabul edilerek dönüş miktarı.
- `referans_yaw`: merkez kabul edilen ilk açı.
- `gps_fark`: gerçek GPS noktası ile PWM/IMU tahmini arasındaki mesafe.
- `aktif=0`: henüz geçerli GPS+IMU ile merkez kurulmadı.

`DeadReckoningProtocol.cs` dosyasını arayüz projesine ekle. Telemetri satırlarını
`DualTrackState.ProcessTelemetryLine(...)` metoduna ver. Haritada iki farklı polyline tut:

1. `GpsTrail`: gerçek GPS izi.
2. `EstimatedTrail`: PWM/IMU tahmini izi.

Tahmini izi yeniden merkezlemek için checksum'lu şu komutu gönder:

```text
CMD:DR:RESET
```

C# tarafında doğru checksum'lu tam satırı şu metod üretir:

```csharp
IdaTelemetryProtocol.BuildDeadReckoningResetCommand()
```

## Harita görünümü önerisi

- GPS izi: düz çizgi.
- Tahmini iz: kesikli çizgi.
- GPS işaretçisi: normal konum simgesi.
- Tahmini işaretçi: farklı simge ve `mutlak_yaw` kadar döndürülmüş ok.
- Açı göstergesi: `goreli_yaw`; merkez çizgisi `0°`.
- Sağlık bilgisi: `gps_fark` büyüdükçe tahmin drift ediyor demektir.

`MainWindow_entegrasyon_ornegi.cs.txt`, mevcut Avalonia olay akışına eklenecek
çağrıları gösterir. Harita kütüphanesinin adı bu pakette bulunmadığı için yalnız
`UpdateGpsPolyline` ve `UpdateEstimatedPolyline` adaptörleri mevcut harita kontrolüne
bağlanmalıdır.

## Merkez yönlü açı göstergesi

`RelativeHeadingGauge.cs` doğrudan Avalonia projesine eklenebilir. Gösterge içinde
mavi çizgi DR sıfırlandığı andaki yönü, kırmızı çizgi ise bu merkeze göre güncel
dönüşü gösterir.

```xml
xmlns:tel="clr-namespace:Arayuz.Telemetri"

<tel:RelativeHeadingGauge
    x:Name="HeadingGauge"
    Width="150"
    Height="150" />
```

DR paketi işlendiğinde:

```csharp
HeadingGauge.RelativeYaw = dr.RelativeYawDeg;
HeadingGauge.AbsoluteYaw = dr.AbsoluteYawDeg;
```
