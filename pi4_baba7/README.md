# Telemetri YKI den IDA ya
KESINLIKLE STRING GONDERILECEKTIR.
## CMD
her paketin basinda olmak zorundadir.
### START
Kullanim: CMD:START*<CHECKSUM>

sistemi baslatir. sistem ilk baslangicinda otomatik olarak manuel modda yani kumandayla kontrol edilebilir sekilde calismaya ayarlanmistir. Eger degistirilmesi istenirse arac bu komuttan sonra mod_id = 2 yani GorevBekleniyor a atilabilir.

### STOP
Kullanim: CMD:STOP*<CHECKSUM>

sistemi durdurur. Uzaktan acil durdurma komutu bu komut ile verilir. Motorlar 0 a cekilir. Sonrasinda komut alabilir haldedir. Sartname farkliysa degistirilebilir.

### MOD
Kullanim: CMD:MOD:<mod_id>*<CHECKSUM>

mod_id yerine kesinlikle u8 formatina (0-255) arasi degerler verilmelidir ki zaten alacagi degerler cok kucuktur.

mod_id = 1 Otonom sistemi baslatir. => CMD:MOD:1*<CHECKSUM>

mod_id = 2 Araci bekleme moduna alir. Arac o an sadece dinlemede kalir.=> CMD:MOD:2*<CHECKSUM>

mod_id = 3 Aracin farkli bir acil durum cikisidir. STOP kullanilmasi tavsiye edilir. =>             CMD:MOD:3*<CHECKSUM>

kalan tum degerler araci manuel moda sokar. Burada kumandadan aldigi degerlerle arac hareket eder. digr verileri toplasa da umursamaz. => 
    CMD:MOD:0*<CHECKSUM>
    CMD:MOD:5*<CHECKSUM>
    CMD:MOD:10*<CHECKSUM>
    CMD:MOD:50*<CHECKSUM> 
    ...

### PING — zorunlu bağlantı heartbeat'i
Kullanım: `CMD:PING*3C`

YKİ bağlantı açık olduğu sürece en geç **1 saniyede bir** PING göndermelidir.
Pi 4 saniye boyunca PING alamazsa RF bağlantısını kopmuş kabul eder. Fiziksel
USB/seri portun açık olması tek başına bağlantı sayılmaz. PING kurulmadan START,
MOD, MAN, HOME, MAP ve ROTA komutları işlenmez. Güvenli yöndeki acil STOP,
geçerli checksum ile gelirse PING kurulmadan da kabul edilir.

### MAN
Kullanım: `CMD:MAN:<ileri>,<yatay>*<CHECKSUM>`

Araç `CMD:MOD:0*<CHECKSUM>` ile manuel moda alınmalıdır.

- `ileri`: `0.0..1.0`
- `yatay`: önerilen format `-1.0..1.0` (`-1` tam sol, `0` düz, `1` tam sağ)
- Eski arayüz uyumluluğu için `-90..90` derece de kabul edilir ve normalize edilir.

Örnekler:

    CMD:MAN:0.75,0.5*AD     => %75 ileri, yarım sağ
    CMD:MAN:0.75,-1.0*<CS>  => %75 ileri, tam sol
    CMD:MAN:0.75,45.0*E1    => eski derece formatıyla yarım sağ
    CMD:MAN:0.0,0.0*<CS>    => bütün motor istekleri sıfır

Geçerli son manuel komut, yeni MAN/STOP/mod değişimi veya telemetri kopması
gelene kadar korunur. Telemetri heartbeat'i koparsa manuel motor komutu sıfırlanır.

## ROTA
Kullanim: CMD:ROTA:<enlem1>,<boylam1>;<enlem2>,<boylam2>*<CHECKSUM>

Enlem ve boylam verileri formati => 41.025632 (tekil enlem ornegi)
En fazla 100 geçerli enlem/boylam noktası gönderilebilir. Enlem -90..90, boylam -180..180 aralığında olmalıdır; NaN, sonsuz, 0/0 ve fazladan alan içeren paketler reddedilir. arac SIRAYLA bu noktalara gidecek sekilde tasarlanmistir.

Veri gonderim ornekleri;
    CMD:ROTA:41.025648,28.974110;41.026098,28.975045*<CHECKSUM>
    CMD:ROTA:41.025643,28.974132;41.026021,28.975015;37.792254,29.057634...*<CHECKSUM>

# TELEMETRI IDA dan YKI ye
IDA yki ye surekli 2 paket gonderecek. Bu paketler;
    NAV:41.025612,28.974134,1.50,2.00,2.1,-1.5,45.5,45.0,1*5A
    MOT:0,400,0,400,0,400,0,400*3B
    NAV:41.025615,28.974137,1.52,2.00,2.0,-1.4,45.2,45.0,1*6C
    MOT:0,400,0,400,0,400,0,400*3B
    ...
seklinde olacak. NMEA yani string formatinda olacak. parse edilmesi gerekecek.

NAV paketi icin veri siralamasi;
    NAV:<enlem>,<boylam>,<gercek_hiz>,<hedef_hiz>,<roll>,<pitch>,<yaw>,<hedef_yaw>,<mod_id>*<CHECKSUM>\n
MOT paketi icin veri siralamasi;
    MOT:<gercek_IskeleOn>,<gercek_IskeleArka>,<gercek_SancakOn>,<gercek_SancakArka>,<istek_IskeleOn>,<istek_IskeleArka>,<istek_SancakOn>,<istek_SancakArka>*<CHECKSUM>\n
    

# ACIL
HER ACİL DURUMDA MOTORLAR 0 LANMALI
WATCHDOG BELKI ?
KAMERA VE LIDAR VERILERI DE BELKI YKI YE AKTARILABILIR. SU AN ELZEM DEGIL.

# Testten sonra degistirilebilecekler
=> Pid degerleri (beyin.rs icinde) ile oynanabilir.
=> IMU kuzeye kalibre degilse kalibrasyon kat sayisi belirlenebilir. (sensorler/bno085.rs icinde degerlere constlar tanimlayarak direkt +-90 gibi bir deger.)
=> imu gps organizasyonu icin belki bir complementary filter eklenebilir.
=> imu icin cok oynak degerlerle karsilasilirsa ve arac cok zigzag atarsa (pid hesabi ve motor iletiminde beyin.rs icinde acilara gore hata paylari birakilmistir - ihmal aci, base_hiz gibi) KALMAN FILTRESI eklenebilir. (3x3), roll,pitch, yaw icin.
=> gps icin halihazirda low level bir filre var ilk konumunu bulmasi icin. ama yine de gerekli goruldugu taktirde low level bir filtre ile cok daha stabil gps verileri elde edilebilir.
=> gps verileri ortamdan dolayi asiri oynak cikarsa enlem,boylam;hiz;yonelim sapma degerleri degiskenleri gps_driver uzerinden pi4 e aktarilip beyin.rs te isleme sokularak belirli bir esik degeriyle cok cok daha stabil sonuclar elde edilebilir. (su an baglanilan uydu sayisi > 6 ve fix_tipi >= 3 kullaniliyor. kilitlendigi kesin ama yine de cok edge caselerle karsilasilirsa yapilsin.)


## v11 Manuel PWM sabitleme

Varsayılan motor eşlemesi `Sol=M2, İleri=M4+M3, Sağ=M1` olarak ayarlandı. YKİ üzerinden `CMD:MAP` ile değiştirilebilir. Manuel `CMD:MAN` komutu zaman aşımına uğramaz; son PWM, yeni bir manuel komut, sıfır komutu, STOP veya mod değişimi gelene kadar korunur.


## v12 güvenlik düzeltmeleri

- Motor komut kanalı `mpsc` kuyruğundan `watch` kanalına çevrildi. Yalnızca en güncel komut tutulur; bağlantı sonrası eski komutlar oynatılmaz.
- STM portu her açıldığında önce `0,0,0,0` gönderilir. Program kapanırken de sıfır komutu gönderilir.
- GPS verisi 2 saniyeden, IMU verisi 500 ms'den eskiyse Otonom/Eve Dönüş motor istekleri sıfırlanır.
- IMU değerleri finite değilse ve GPS fix/uydu/koordinat koşulları geçersizse otonom hareket durur.
- Telemetri bağlantısı yalnızca düzenli `CMD:PING` ile geçerlidir.
- Program ilk açıldığında telemetri hiç bağlanmamışsa yanlışlıkla Eve Dönüş başlatılmaz.
- GPS hızı YKİ'ye mm/s yerine m/s gönderilir.
- GPS ve IMU paket okumalarına timeout eklendi; yarım paketlerde port yeniden açılır.
- Varsayılan son eşleşme GPS=`/dev/ttyACM1`, IMU=`/dev/ttyACM0` olarak ayarlandı. Kalıcı kullanımda `/dev/serial/by-id/...` kullanın.

Ortam değişkeni örneği:

```bash
export IDA_GPS_PORT=/dev/serial/by-id/usb-u-blox_GNSS_receiver-if00
export IDA_GPS_BAUD=115200
export IDA_IMU_PORT=/dev/serial/by-id/usb-Embassy_USB-serial_logger-if00
export IDA_MOTOR_PORT=/dev/ttyUSB1
export IDA_TEL_PORT=/dev/ttyUSB0
cargo run --release --bin pi4_baba
```

> **STM güvenlik watchdog'u hâlâ STM firmware'inde uygulanmalıdır.** Pi tarafı 50 ms'de bir güncel motor paketi yollar. STM 300–500 ms yeni/geçerli paket alamazsa dört PWM kanalını kendi başına sıfırlamalıdır. Pi'nin güç kaybetmesi veya prosesin aniden çökmesi yalnızca STM tarafındaki watchdog ile güvenli hale gelir.


## GPS doğrudan USB/NMEA sürümü

Bu pakette GPS, Pico/STM ikili paketi yerine NEO-M8N USB seri çıkışındaki NMEA
cümlelerinden doğrudan okunur. Ayrıntılı kurulum ve test için
`GPS_DIREKT_USB_NMEA.md` dosyasına bakın.

Örnek:

```bash
python3 gps_usb_test.py /dev/ttyACM1 115200
export IDA_GPS_PORT=/dev/ttyACM1
export IDA_GPS_BAUD=115200
cargo run --release --bin pi4_baba
```
