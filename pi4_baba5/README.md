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

### MAN
Kullanim: CMD:MAN:<gaz>,<aci>*<CHECKSUM>

aracin kumandadan verileri okuma protokoludur. Aracin CMD:MOD:0*<CHECKSUM> ile manuel moda alindigindan emin olunmalidir.

gaz ve aci degerleri f32 formatta gonderilmelidir.

    CMD:MAN:0.75,15.5*<CHECKSUM> => motorlara yuzde 75 yuk, aci degeri 15.5
    CMD:MAN:0.22,30.0*<CHECKSUM> =>
    motorlara yuzde 22 yuk, 30 derece aci
    CMD:MAN:0.20,90.0*<CHECKSUM> => motorlara yuzde 20 yuk, 90 derece aci
    ...

## ROTA
Kullanim: CMD:ROTA:<enlem1>,<boylam1>;<enlem2>,<boylam2>*<CHECKSUM>

Enlem ve boylam verileri formati => 41.025632 (tekil enlem ornegi)
istenilen kadar enlem ve boylam verileri gonderilerek araca belirli noktalar olusturulur. arac SIRAYLA bu noktalara gidecek sekilde tasarlanmistir.

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
