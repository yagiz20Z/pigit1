# Raspberry Pi açılışta otomatik başlatma ve çökünce yeniden çalıştırma

Bu paketteki `install_autostart.sh`, karar verme modülünü `systemd` servisi olarak
kurar. Sonuç olarak:

- Raspberry Pi açıldığında `pi4_baba` masaüstü veya terminal açılmadan başlar.
- Program hata vererek, panikleyerek veya normal biçimde kapanırsa 2 saniye sonra
  yeniden açılır.
- Yeniden başlatma sınırı kapalıdır; servis tekrar tekrar çökerse dahi denemeye
  devam eder.
- Sistem kapatılırken servise `SIGINT` gönderilir. Böylece mevcut Rust kodundaki
  motorları sıfırlama ve SD kayıtlarını kapatma bölümü çalışır.
- Çıktılar `journalctl` üzerinden saklanır ve izlenir.

## Tek komutla kurulum

Proje klasörüne girin:

```bash
cd ~/Downloads/pi4_baba11
chmod +x install_autostart.sh
./install_autostart.sh
```

Betik gerektiğinde `sudo` yetkisini kendisi ister. Kullanıcı otomatik
belirlenemezse:

```bash
sudo IDA_USER=alinux ./install_autostart.sh
```

Kurulum sırasında Pi üzerinde `cargo build --release --locked` çalıştırılır ve
program şuraya kurulur:

```text
/usr/local/bin/ida-karar-verme
```

## Seri port ayarları

Ayar dosyası:

```bash
sudo nano /etc/ida/ida.env
```

Varsayılanlar:

```text
IDA_TEL_PORT=/dev/ttyUSB0
IDA_MOTOR_PORT=/dev/ttyUSB1
IDA_GPS_PORT=/dev/ttyACM1
IDA_GPS_BAUD=115200
IDA_IMU_PORT=/dev/ttyACM0
```

USB numaraları açılışlar arasında değişebildiği için mümkün olduğunda
`/dev/serial/by-id/...` kullanın. Mevcut kalıcı yolları görmek için:

```bash
ls -l /dev/serial/by-id/
```

Ayar değiştirdikten sonra:

```bash
sudo systemctl restart ida-karar-verme
```

## Kontrol komutları

Servis durumu:

```bash
systemctl status ida-karar-verme --no-pager -l
```

Canlı çıktı:

```bash
journalctl -u ida-karar-verme -f
```

Bu açılıştaki bütün kayıtlar:

```bash
journalctl -u ida-karar-verme -b --no-pager
```

Yeniden başlatma:

```bash
sudo systemctl restart ida-karar-verme
```

Geçici durdurma:

```bash
sudo systemctl stop ida-karar-verme
```

Tekrar çalıştırma:

```bash
sudo systemctl start ida-karar-verme
```

Açılışta başlamayı kapatma:

```bash
sudo systemctl disable --now ida-karar-verme
```

Yeniden açma:

```bash
sudo systemctl enable --now ida-karar-verme
```

## Kendini yeniden açma testi

Önce PID'yi görün:

```bash
systemctl show -p MainPID --value ida-karar-verme
```

Prosesi öldürün:

```bash
sudo systemctl kill -s SIGKILL ida-karar-verme
```

Yaklaşık 2 saniye sonra servis tekrar `active (running)` olmalıdır:

```bash
systemctl status ida-karar-verme --no-pager -l
```

## Önemli güvenlik notu

Pi programının yeniden başlaması motor güvenliği için tek başına yeterli değildir.
Pi güç kaybederse veya işletim sistemi kilitlenirse `systemd` çalışamaz. STM tarafında
300–500 ms boyunca geçerli motor paketi gelmezse dört PWM kanalını nötr/sıfır yapan
bağımsız bir watchdog bulunmalıdır.
