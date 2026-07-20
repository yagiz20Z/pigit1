# BNO085 ile 6 saniyelik kare görevi

Bu sürümde kare görevi GPS ve rota kullanmaz.

Akış:

1. OTONOM veya START komutu alınır.
2. BNO085 verisi taze ve geçerliyse iki ileri motor 6 saniye çalışır.
3. Sol dönüş motoru çalıştırılır.
4. BNO085 yaw değişiminin gerçek işareti otomatik belirlenir.
5. Başlangıç yaw değerine göre tam 90 derece hedef yaw oluşturulur.
6. Hedefe 20 derece kala dönüş PWM'i düşürülür, 5 derece kala ince PWM kullanılır.
7. Tekne hedefi aşarsa sağ dönüş motoru ile ters yönde ince düzeltme yapılır.
8. Yaw değeri hedefin +/-1 derece aralığında 350 ms boyunca kalırsa dönüş tamamlanır.
9. Dört adet 6 saniyelik kenar ve üç adet 90 derecelik dönüş sonunda motorlar durur.

Önemli:

- BNO085 paketi `src/sensorler/bno085.rs` içinde 0xAA 0xBB başlığı, 45 bayt gövde ve XOR kontrolü ile okunur.
- IMU varsayılan olarak `/dev/ttyACM0` ve 115200 baud kullanır.
- Kare hareketinde GPS fix, uydu sayısı, koordinat veya rota kontrol edilmez.
- Su akıntısı, rüzgar, motor farkı ve gövde ataleti nedeniyle GPS kullanılmadan geometrik olarak tamamen kapanan bir kare garanti edilemez. Yazılım 6 saniyelik eşit kenarlar ve BNO085'e göre 90 +/-1 derece dönüş hedefler.
- İlk testte tekne sabitlenmeli veya pervaneler güvenli durumda olmalıdır.
