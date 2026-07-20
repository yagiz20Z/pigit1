# Kontrol sonucu

- Rust kaynakları `rustfmt` ile biçimlendirildi.
- Tokio 1.43.1 ve seri-port API uyumluluk stub'ı ile çevrimdışı tip kontrolü yapıldı.
- 5 birim testi geçti:
  - Gaz sıfırken yatay motorların çalışmaması
  - İki ileri motorun aynı komutu alması
  - Normalize ve derece manuel komutlarının çözümlenmesi
  - Geçersiz rotaların reddedilmesi
  - PING komutunun çözümlenmesi
- Gerçek `tokio-serial 5.5.0` bağımlılığı bu çalışma ortamında crates.io DNS erişimi olmadığı için indirilemedi. Proje API olarak mevcut `tokio-serial` kullanımını korur.
- Donanım üzerinde motor/pervane bağlı ilk test düşük güçte ve güvenli sehpa üzerinde yapılmalıdır.
