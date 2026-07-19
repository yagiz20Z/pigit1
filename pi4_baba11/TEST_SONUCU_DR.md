# PWM + IMU çift iz doğrulaması

Bu çalışma ortamında `cargo`, `rustc` ve `dotnet` kurulu olmadığı için gerçek Rust ve
Avalonia derlemesi çalıştırılamadı.

Yapılan kontroller:

- `gps_usb_test.py` ve `dr_dual_track_viewer.py` Python sözdizimi doğrulandı.
- `NAV` ve yeni `DR` alan sayıları/checksum üretimi örnek paketlerle doğrulandı.
- `CMD:DR:RESET` tam satır checksum değerinin `61` olduğu doğrulandı.
- Rust ve C# kaynaklarında parantez/köşeli parantez/süslü parantez bütünlüğü kontrol edildi.
- `GidenTelemetri -> DR satırı -> seri yazım` bağlantı noktaları statik olarak kontrol edildi.
- ZIP oluşturulduktan sonra arşiv bütünlük testi yapılmalıdır.

Araç üstündeki ilk testte motor-hız katsayıları mutlaka kalibre edilmelidir. PWM'den
hesaplanan konum fiziksel geri besleme olmadığı için zamanla gerçek GPS izinden sapar.
