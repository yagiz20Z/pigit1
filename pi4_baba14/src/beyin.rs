use crate::dead_reckoning::DeadReckoning;
use crate::veri_tipleri::{
    AracMod, GelenTelemetri, GidenTelemetri, GpsVeri, ImuVeri, MotorEsleme, MotorVeri,
};
use std::{
    f64::consts::PI,
    time::{Duration, Instant},
};
use tokio::{
    sync::{mpsc, watch},
    time::interval,
};

pub struct PidKontrolcu {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    integral: f64,
    onceki_hata: f64,
    integral_siniri: f64,
}

impl PidKontrolcu {
    pub fn new(kp: f64, ki: f64, kd: f64, integral_siniri: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            onceki_hata: 0.0,
            integral_siniri,
        }
    }

    pub fn guncelle(&mut self, hata: f64, dt: f64) -> f64 {
        self.integral += hata * dt;
        self.integral = self
            .integral
            .clamp(-self.integral_siniri, self.integral_siniri);
        let turev = if dt > 0.0 {
            ((hata - self.onceki_hata) / dt).clamp(-100.0, 100.0)
        } else {
            0.0
        };
        self.onceki_hata = hata;
        (self.kp * hata) + (self.ki * self.integral) + (self.kd * turev)
    }
}

const IHMALACI: f64 = 3.0;
// 4 metrelik kare görevde 2.5 m tolerans köşeleri erken atlatıyordu.
const HEDEF_TOLERANS: f64 = 1.0;
const MANUEL_OLU_BOLGE: f32 = 0.02;
// Su üstü araç için 2B fix yeterlidir; irtifa navigasyonda kullanılmıyor.
const MIN_GPS_FIX: u8 = 2;
const MIN_GPS_UYDU: u8 = 4;
const ORIGIN_ORNEK_SAYISI: u8 = 5;
const GPS_VERI_ZAMAN_ASIMI: Duration = Duration::from_secs(3);
// BNO/Pico yayın hızı 1 Hz'e düştüğünde 500 ms watchdog otonomiyi sürekli kesiyordu.
const IMU_VERI_ZAMAN_ASIMI: Duration = Duration::from_secs(2);

// Varsayılan fiziksel motor sırası:
// Varsayılan: M1 = sağa yatay, M2 = sola yatay, M3 = ileri-2, M4 = ileri-1
// Bu sıra artık YKİ'den CMD:MAP:sol,ileri1,sag,ileri2 komutuyla değiştirilebilir.
//
// Manuel paket: CMD:MAN:ileri,yatay
// ileri: 0.0..1.0
// yatay: -1.0..1.0  (eksi=sol, artı=sağ)
fn motor_kanalina_yaz(motor: &mut MotorVeri, kanal: u8, deger: u16) {
    match kanal {
        1 => motor.iskeleon = deger,
        2 => motor.iskelearka = deger,
        3 => motor.sancakon = deger,
        4 => motor.sancakarka = deger,
        _ => {}
    }
}

fn motor_kanalindan_oku(motor: &MotorVeri, kanal: u8) -> u16 {
    match kanal {
        1 => motor.iskeleon,
        2 => motor.iskelearka,
        3 => motor.sancakon,
        4 => motor.sancakarka,
        _ => 0,
    }
}

fn gps_gecerli(gps: &GpsVeri) -> bool {
    gps.algi_boyut >= MIN_GPS_FIX
        && gps.uydu_sayi >= MIN_GPS_UYDU
        && (-900_000_000..=900_000_000).contains(&gps.enlem)
        && (-1_800_000_000..=1_800_000_000).contains(&gps.boylam)
        && gps.enlem != 0
        && gps.boylam != 0
        && gps.hiz >= 0
}

fn imu_gecerli(imu: &ImuVeri) -> bool {
    [
        imu.roll, imu.pitch, imu.yaw, imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az,
    ]
    .into_iter()
    .all(f32::is_finite)
}

fn veri_taze(son_gelis: Option<Instant>, zaman_asimi: Duration) -> bool {
    son_gelis
        .map(|an| an.elapsed() <= zaman_asimi)
        .unwrap_or(false)
}

fn koordinat_gecerli(lat: f64, lon: f64) -> bool {
    lat.is_finite()
        && lon.is_finite()
        && (-90.0..=90.0).contains(&lat)
        && (-180.0..=180.0).contains(&lon)
        && !(lat == 0.0 && lon == 0.0)
}

fn manuel_motor_karistir(ileri_girdisi: f32, yatay_girdisi: f32, esleme: MotorEsleme) -> MotorVeri {
    let ileri = ileri_girdisi.clamp(0.0, 1.0);
    let yatay = yatay_girdisi.clamp(-1.0, 1.0);
    let mut motor = MotorVeri::default();

    if !esleme.gecerli() {
        return motor;
    }

    // Kullanıcı güvenlik şartı: gaz/ileri sıfırken yatay/açı ne olursa olsun
    // hiçbir motor çalışmaz.
    if ileri <= MANUEL_OLU_BOLGE {
        return motor;
    }

    let ileri_komutu = if ileri > MANUEL_OLU_BOLGE {
        (ileri * 1000.0).round() as u16
    } else {
        0
    };

    let yatay_komutu = if yatay.abs() > MANUEL_OLU_BOLGE {
        (yatay.abs() * 1000.0).round() as u16
    } else {
        0
    };

    // İki ileri motor kesinlikle aynı değişkenden beslenir.
    motor_kanalina_yaz(&mut motor, esleme.ileri1, ileri_komutu);
    motor_kanalina_yaz(&mut motor, esleme.ileri2, ileri_komutu);

    if yatay < -MANUEL_OLU_BOLGE {
        motor_kanalina_yaz(&mut motor, esleme.sol, yatay_komutu);
    } else if yatay > MANUEL_OLU_BOLGE {
        motor_kanalina_yaz(&mut motor, esleme.sag, yatay_komutu);
    }

    motor
}

pub struct NavData {
    origin_enlem: f64,
    origin_boylam: f64,
    cos_enlem: f64,
    is_origin_set: bool,
    gps_ornek_sayaci: u8,
    ornek_enlem_toplam: i64,
    ornek_boylam_toplam: i64,
    current_x: f64,
    current_y: f64,
    current_yaw: f64,
    hedef_noktalar: Vec<(f64, f64)>,
    current_hn_index: usize,
}

impl NavData {
    pub fn new() -> Self {
        Self {
            origin_enlem: 0.0,
            origin_boylam: 0.0,
            cos_enlem: 1.0,
            is_origin_set: false,
            gps_ornek_sayaci: 0,
            ornek_enlem_toplam: 0,
            ornek_boylam_toplam: 0,
            current_x: 0.0,
            current_y: 0.0,
            current_yaw: 0.0,
            hedef_noktalar: Vec::new(),
            current_hn_index: 0,
        }
    }

    pub fn guvenli_origin_belirle(&mut self, gps: &GpsVeri) -> bool {
        if !gps_gecerli(gps) {
            self.gps_ornek_sayaci = 0;
            self.ornek_enlem_toplam = 0;
            self.ornek_boylam_toplam = 0;
            return false;
        }
        self.ornek_enlem_toplam += gps.enlem as i64;
        self.ornek_boylam_toplam += gps.boylam as i64;
        self.gps_ornek_sayaci += 1;
        if self.gps_ornek_sayaci >= ORIGIN_ORNEK_SAYISI {
            let bolen = ORIGIN_ORNEK_SAYISI as i64;
            let ortalama_enlem = (self.ornek_enlem_toplam / bolen) as i32;
            let ortalama_boylam = (self.ornek_boylam_toplam / bolen) as i32;
            self.origin_enlem = ortalama_enlem as f64 / 10_000_000.0;
            self.origin_boylam = ortalama_boylam as f64 / 10_000_000.0;
            self.cos_enlem = (self.origin_enlem * std::f64::consts::PI / 180.0).cos();
            self.is_origin_set = true;
            self.ornek_enlem_toplam = 0;
            self.ornek_boylam_toplam = 0;
            return true;
        }
        false
    }

    pub fn guncelle_konum(&mut self, lat_i32: i32, lon_i32: i32, yaw: f64) {
        let lat = lat_i32 as f64 / 10_000_000.0;
        let lon = lon_i32 as f64 / 10_000_000.0;
        self.current_y = (lat - self.origin_enlem) * 111_320.0;
        self.current_x = (lon - self.origin_boylam) * 111_320.0 * self.cos_enlem;
        self.current_yaw = yaw;
    }

    pub fn set_rota(&mut self, noktalar: Vec<(f64, f64)>) {
        self.hedef_noktalar = noktalar;
        self.current_hn_index = 0;
    }

    pub fn guncel_hedef(&self) -> Option<(f64, f64)> {
        if self.current_hn_index < self.hedef_noktalar.len() {
            Some(self.hedef_noktalar[self.current_hn_index])
        } else {
            None
        }
    }
    pub fn calc_mesafe(&self, hedef_x: f64, hedef_y: f64) -> f64 {
        let dx = hedef_x - self.current_x;
        let dy = hedef_y - self.current_y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn calc_hedefeaci(&self, hedef_x: f64, hedef_y: f64) -> f64 {
        let dx = hedef_x - self.current_x;
        let dy = hedef_y - self.current_y;
        dx.atan2(dy) * 180.0 / PI
    }

    pub fn bakisyonu_hata(&self, hedefeaci: f64) -> f64 {
        let mut hata = hedefeaci - self.current_yaw;
        while hata > 180.0 {
            hata -= 360.0
        }
        while hata < -180.0 {
            hata += 360.0
        }
        hata
    }
}

pub async fn nav_task(
    mut imu_rx: watch::Receiver<ImuVeri>,
    mut gps_rx: watch::Receiver<GpsVeri>,
    motor_tx: watch::Sender<MotorVeri>,
    mut rx_yki: mpsc::Receiver<GelenTelemetri>,
    yki_tx: mpsc::Sender<GidenTelemetri>,
    sd_kayit_tx: watch::Sender<GidenTelemetri>,
) {
    let mut tick = interval(Duration::from_millis(50));
    let mut last_time = Instant::now();
    let mut nav = NavData::new();
    let mut dead_reckoning = DeadReckoning::new();
    let mut dr_reset_istendi = false;
    let mut dr_bekleme_bildirildi = false;
    let mut pid = PidKontrolcu::new(4.0, 0.1, 0.5, 150.0);
    let base_hiz: f32 = 400.0;
    let mut kaba_donus_modu = false;
    let mut guncel_mod = AracMod::Manuel;
    let mut telemetri_sayaci = 0;
    let mut son_manuel_ileri: f32 = 0.0;
    let mut son_manuel_yatay: f32 = 0.0;
    let mut manuel_komut_alindi = false;
    let mut motor_esleme = MotorEsleme::default();
    // Öncelik YKİ'den gelen CMD:HOME koordinatıdır. Gönderilmezse ilk güvenilir
    // GPS origin'i otomatik olarak kalkış/geri dönüş noktası kabul edilir.
    let mut eve_donus_noktasi: Option<(f64, f64)> = None;
    let mut telemetri_bagli = false;
    let mut telemetri_daha_once_baglandi = false;
    let mut eve_donus_tamamlandi = false;
    let mut gps = GpsVeri::default();
    let mut imu = ImuVeri::default();
    let mut son_gps_gelisi: Option<Instant> = None;
    let mut son_imu_gelisi: Option<Instant> = None;
    let mut sensor_guvenlik_bildirildi = false;
    println!(
        "Motor eşlemesi başlangıç: sol=M{}, ileri=M{}+M{}, sağ=M{}",
        motor_esleme.sol, motor_esleme.ileri1, motor_esleme.ileri2, motor_esleme.sag,
    );
    let dr_ayar = dead_reckoning.ayar();
    println!(
        "PWM dead reckoning: ileri_max={:.2} m/s, yatay_max={:.2} m/s, ölü_bölge={:.3}",
        dr_ayar.max_ileri_hiz_m_s, dr_ayar.max_yatay_hiz_m_s, dr_ayar.pwm_olu_bolge,
    );
    loop {
        tick.tick().await;
        let simdi = Instant::now();
        let dt = simdi.duration_since(last_time).as_secs_f64();
        last_time = simdi;

        // watch kanallarında yeni paket görülünce Pi tarafındaki geliş anını kaydet.
        // GPS zaman_ms kaynağından bağımsız olarak freshness hesabında Pi tarafındaki geliş anı kullanılır.
        let mut gps_yeni = false;
        if gps_rx.has_changed().unwrap_or(false) {
            gps = *gps_rx.borrow_and_update();
            son_gps_gelisi = Some(Instant::now());
            gps_yeni = true;
        }
        if imu_rx.has_changed().unwrap_or(false) {
            imu = *imu_rx.borrow_and_update();
            son_imu_gelisi = Some(Instant::now());
        }

        let gps_taze = veri_taze(son_gps_gelisi, GPS_VERI_ZAMAN_ASIMI);
        let imu_taze = veri_taze(son_imu_gelisi, IMU_VERI_ZAMAN_ASIMI);
        let gps_hazir = gps_taze && gps_gecerli(&gps);
        let imu_hazir = imu_taze && imu_gecerli(&imu);
        let otonom_sensorler_hazir = gps_hazir && imu_hazir;

        while let Ok(komut) = rx_yki.try_recv() {
            match komut {
                GelenTelemetri::AcilDurdur => {
                    guncel_mod = AracMod::AcilDurum;
                    son_manuel_ileri = 0.0;
                    son_manuel_yatay = 0.0;
                    manuel_komut_alindi = false;
                    println!("Acil durdur!");
                }
                GelenTelemetri::GoreviBaslat => {
                    if guncel_mod != AracMod::AcilDurum {
                        if !nav.hedef_noktalar.is_empty() {
                            son_manuel_ileri = 0.0;
                            son_manuel_yatay = 0.0;
                            manuel_komut_alindi = false;
                            guncel_mod = AracMod::Otonom;
                            println!("Otonom Modu");
                        } else {
                            println!("Rotasız otonom moduna geçilmez.");
                        }
                    }
                }
                GelenTelemetri::ModDegistir(istenen_mod) => {
                    guncel_mod = if istenen_mod == AracMod::Otonom && nav.hedef_noktalar.is_empty() {
                        println!("OTONOM reddedildi: önce geçerli rota gönderilmelidir.");
                        AracMod::GorevBekliyor
                    } else {
                        istenen_mod
                    };
                    son_manuel_ileri = 0.0;
                    son_manuel_yatay = 0.0;
                    manuel_komut_alindi = false;
                    println!("Mod değişimi: {:?}", guncel_mod);
                }
                GelenTelemetri::ManuelKontrol(ileri, yatay) => {
                    guncel_mod = AracMod::Manuel;
                    son_manuel_ileri = ileri.clamp(0.0, 1.0);
                    son_manuel_yatay = yatay.clamp(-1.0, 1.0);
                    manuel_komut_alindi = true;
                    println!(
                        "Manuel PWM sabitlendi: ileri={:.3}, yatay={:.3}; DUR/STOP gelene kadar korunacak.",
                        son_manuel_ileri, son_manuel_yatay,
                    );
                }
                GelenTelemetri::MotorEslemeDegistir(yeni_esleme) => {
                    if yeni_esleme.gecerli() {
                        motor_esleme = yeni_esleme;
                        son_manuel_ileri = 0.0;
                        son_manuel_yatay = 0.0;
                        manuel_komut_alindi = false;
                        println!(
                            "Motor eşlemesi güncellendi: sol=M{}, ileri=M{}+M{}, sağ=M{}",
                            motor_esleme.sol,
                            motor_esleme.ileri1,
                            motor_esleme.ileri2,
                            motor_esleme.sag,
                        );
                    }
                }
                GelenTelemetri::EvKonumuBelirle(lat, lon) => {
                    if koordinat_gecerli(lat, lon) {
                        eve_donus_noktasi = Some((lat, lon));
                        eve_donus_tamamlandi = false;
                        println!(
                            "Güvenli dönüş konumu YKİ'den alındı: {:.7}, {:.7}",
                            lat, lon
                        );
                    }
                }
                GelenTelemetri::TelemetriBaglandi => {
                    if !telemetri_bagli {
                        println!("Telemetri yeniden bağlandı.");
                    }
                    telemetri_bagli = true;
                    telemetri_daha_once_baglandi = true;
                    // Güvenli dönüş başladıysa bağlantı geri gelse bile hedefe kadar
                    // devam eder. YKİ yeni bir mod/STOP komutuyla bunu değiştirebilir.
                }
                GelenTelemetri::TelemetriKoptu => {
                    telemetri_bagli = false;
                    son_manuel_ileri = 0.0;
                    son_manuel_yatay = 0.0;
                    manuel_komut_alindi = false;

                    // Program ilk açıldığında henüz hiç RF bağlantısı kurulmamış olabilir.
                    // Bu durum gerçek bir "bağlandıktan sonra kopma" değildir ve otomatik
                    // eve dönüş başlatmamalıdır.
                    if !telemetri_daha_once_baglandi {
                        eprintln!("Telemetri henüz hiç kurulmadı; motorlar sıfırda bekliyor.");
                    } else {
                        eprintln!("Telemetri koptu: güvenli dönüş hazırlanıyor.");
                        eve_donus_tamamlandi = false;

                        if guncel_mod != AracMod::AcilDurum {
                            if eve_donus_noktasi.is_some() && otonom_sensorler_hazir {
                                guncel_mod = AracMod::EveDonus;
                                pid.integral = 0.0;
                                kaba_donus_modu = false;
                                println!(
                                    "EVE DÖNÜŞ MODU: motor kontrolü güvenli dönüş konumuna yönlendirildi."
                                );
                            } else {
                                // Hedef veya güncel sensör verisi yoksa rastgele hareket edilmez.
                                guncel_mod = AracMod::GorevBekliyor;
                                eprintln!(
                                    "Eve dönüş başlatılamadı: güvenli HOME veya güncel GPS/IMU yok. Motorlar duruyor."
                                );
                            }
                        }
                    }
                }
                GelenTelemetri::TelemetriHeartbeat => {
                    // Bağlantı watchdog'u telemetri katmanında tutulur.
                }
                GelenTelemetri::DeadReckoningSifirla => {
                    dr_reset_istendi = true;
                    dr_bekleme_bildirildi = false;
                    println!(
                        "DR sıfırlama istendi: güncel GPS konumu ve IMU yaw yeni merkez olacak."
                    );
                }
                GelenTelemetri::RotaBelirle(noktalar) => {
                    nav.set_rota(noktalar);
                    son_manuel_ileri = 0.0;
                    son_manuel_yatay = 0.0;
                    manuel_komut_alindi = false;
                    guncel_mod = AracMod::GorevBekliyor;
                    println!("Yeni rota: ({} nokta)", nav.hedef_noktalar.len());
                }
            }
        }
        let mut motor_istek = MotorVeri::default();
        let mut anlik_hedef_aci = imu.yaw as f64;
        let mut anlik_hedef_mesafe = 0.0_f64;
        if guncel_mod == AracMod::Manuel && manuel_komut_alindi {
            // Son manuel komut kilitlenir. Yeni CMD:MAN, DUR/0 komutu, STOP veya
            // mod değişimi gelene kadar aynı PWM her 50 ms motor katmanına gönderilir.
            motor_istek = manuel_motor_karistir(son_manuel_ileri, son_manuel_yatay, motor_esleme);
        }
        if !nav.is_origin_set {
            if gps_yeni
                && gps_hazir
                && nav.guvenli_origin_belirle(&gps)
                && eve_donus_noktasi.is_none()
            {
                eve_donus_noktasi = Some((nav.origin_enlem, nav.origin_boylam));
                eve_donus_tamamlandi = false;
                println!(
                    "Otomatik güvenli dönüş konumu (ilk GPS origin): {:.7}, {:.7}",
                    nav.origin_enlem, nav.origin_boylam,
                );
            }
        } else {
            // Kopma anında sensör geçersizse motorlar durur. Veriler tekrar güncel
            // olduğunda aynı kopma için güvenli dönüş otomatik başlatılır.
            if telemetri_daha_once_baglandi
                && !telemetri_bagli
                && !eve_donus_tamamlandi
                && guncel_mod != AracMod::AcilDurum
                && guncel_mod != AracMod::EveDonus
                && eve_donus_noktasi.is_some()
                && otonom_sensorler_hazir
            {
                guncel_mod = AracMod::EveDonus;
                son_manuel_ileri = 0.0;
                son_manuel_yatay = 0.0;
                manuel_komut_alindi = false;
                pid.integral = 0.0;
                kaba_donus_modu = false;
                println!("GPS/IMU hazır: telemetri kopuk güvenli dönüşü başlatıldı.");
            }

            if otonom_sensorler_hazir {
                sensor_guvenlik_bildirildi = false;
                nav.guncelle_konum(gps.enlem, gps.boylam, imu.yaw as f64);
                anlik_hedef_aci = imu.yaw as f64;

                if matches!(guncel_mod, AracMod::Otonom | AracMod::EveDonus) {
                    let hedef = if guncel_mod == AracMod::EveDonus {
                        eve_donus_noktasi
                    } else {
                        nav.guncel_hedef()
                    };

                    if let Some((hedef_lat, hedef_lon)) = hedef {
                        let hedefy_metre = (hedef_lat - nav.origin_enlem) * 111_320.0;
                        let hedefx_metre =
                            (hedef_lon - nav.origin_boylam) * 111_320.0 * nav.cos_enlem;
                        let mesafe = nav.calc_mesafe(hedefx_metre, hedefy_metre);
                        anlik_hedef_mesafe = mesafe;
                        let hedefe_aci = nav.calc_hedefeaci(hedefx_metre, hedefy_metre);
                        let hata = nav.bakisyonu_hata(hedefe_aci);
                        anlik_hedef_aci = hedefe_aci;
                        if mesafe < HEDEF_TOLERANS {
                            if guncel_mod == AracMod::EveDonus {
                                println!("Güvenli dönüş konumuna ulaşıldı. Motorlar durduruldu.");
                                eve_donus_tamamlandi = true;
                                guncel_mod = AracMod::GorevBekliyor;
                            } else {
                                println!("{}. hedef noktasına ulaşıldı!", nav.current_hn_index + 1);
                                nav.current_hn_index += 1;
                            }
                            pid.integral = 0.0;
                            pid.onceki_hata = hata;
                            kaba_donus_modu = false;
                        } else {
                            let onceki_kaba = kaba_donus_modu;
                            if hata.abs() > 30.0 {
                                kaba_donus_modu = true;
                            } else if hata.abs() < 15.0 {
                                kaba_donus_modu = false;
                            }
                            if onceki_kaba && !kaba_donus_modu {
                                pid.integral = 0.0;
                                pid.onceki_hata = hata;
                            }
                            let donus_gucu = if kaba_donus_modu {
                                0.0
                            } else {
                                pid.guncelle(hata, dt)
                            };
                            if kaba_donus_modu {
                                if hata < 0.0 {
                                    motor_kanalina_yaz(&mut motor_istek, motor_esleme.sag, 400);
                                } else {
                                    motor_kanalina_yaz(&mut motor_istek, motor_esleme.sol, 400);
                                }
                            } else {
                                let duzeltme = if hata.abs() < IHMALACI {
                                    0.0
                                } else {
                                    donus_gucu
                                };

                                // YKİ'den seçilmiş iki ileri motor daima aynı komutu alır.
                                let ileri_komutu = base_hiz.clamp(0.0, 1000.0) as u16;
                                motor_kanalina_yaz(
                                    &mut motor_istek,
                                    motor_esleme.ileri1,
                                    ileri_komutu,
                                );
                                motor_kanalina_yaz(
                                    &mut motor_istek,
                                    motor_esleme.ileri2,
                                    ileri_komutu,
                                );

                                // Yön düzeltmesi seçilmiş yatay motorlarla yapılır.
                                if duzeltme > 0.0 {
                                    motor_kanalina_yaz(
                                        &mut motor_istek,
                                        motor_esleme.sol,
                                        duzeltme.clamp(0.0, 1000.0) as u16,
                                    );
                                } else if duzeltme < 0.0 {
                                    motor_kanalina_yaz(
                                        &mut motor_istek,
                                        motor_esleme.sag,
                                        (-duzeltme).clamp(0.0, 1000.0) as u16,
                                    );
                                }
                            }
                        }
                    } else {
                        guncel_mod = AracMod::GorevBekliyor;
                        if !telemetri_bagli {
                            eprintln!(
                                "Telemetri yok ve güvenli dönüş hedefi bulunamadı. Motorlar duruyor."
                            );
                        }
                    }
                }
            } else if matches!(guncel_mod, AracMod::Otonom | AracMod::EveDonus) {
                // motor_istek varsayılan sıfır olarak kalır. Sensör geri gelince mod
                // korunarak hesap otomatik devam eder.
                if !sensor_guvenlik_bildirildi {
                    eprintln!(
                        "SENSÖR WATCHDOG: otonom motorlar durduruldu. GPS[taze={}, fix={}, uydu={}] IMU[taze={}, finite={}].",
                        gps_taze,
                        gps.algi_boyut,
                        gps.uydu_sayi,
                        imu_taze,
                        imu_gecerli(&imu),
                    );
                    sensor_guvenlik_bildirildi = true;
                }
            }
        }

        // Tahmini iz GPS'ten bağımsız ilerler. İlk geçerli GPS+IMU geldiğinde veya
        // YKİ CMD:DR:RESET gönderdiğinde o anki konum/yaw yeni merkez kabul edilir.
        if (!dead_reckoning.aktif_mi() || dr_reset_istendi) && gps_hazir && imu_hazir {
            if dead_reckoning.sifirla(&gps, imu.yaw) {
                dr_reset_istendi = false;
                dr_bekleme_bildirildi = false;
                println!(
                    "DR merkezi ayarlandı: GPS={:.7},{:.7} referans_yaw={:.2}°",
                    gps.enlem as f64 / 10_000_000.0,
                    gps.boylam as f64 / 10_000_000.0,
                    imu.yaw,
                );
            }
        } else if dr_reset_istendi && !dr_bekleme_bildirildi {
            eprintln!("DR sıfırlama bekliyor: geçerli/taze GPS ve IMU birlikte gerekli.");
            dr_bekleme_bildirildi = true;
        }

        let dr_cikti = dead_reckoning.guncelle(
            dt,
            &motor_istek,
            motor_esleme,
            imu.yaw,
            imu_hazir,
            gps_hazir.then_some(&gps),
        );

        let _ = motor_tx.send(motor_istek.clone());

        telemetri_sayaci += 1;
        if telemetri_sayaci % 2 == 0 {
            let toplam_waypoint = nav.hedef_noktalar.len();
            let aktif_waypoint = if toplam_waypoint == 0 {
                0
            } else if nav.current_hn_index < toplam_waypoint {
                nav.current_hn_index + 1
            } else {
                toplam_waypoint
            };

            let otonom_durum = if guncel_mod == AracMod::AcilDurum {
                8
            } else if guncel_mod == AracMod::Otonom {
                if toplam_waypoint == 0 {
                    1
                } else if !gps_hazir {
                    2
                } else if !imu_hazir {
                    3
                } else if !nav.is_origin_set {
                    4
                } else if kaba_donus_modu {
                    5
                } else {
                    6
                }
            } else if guncel_mod == AracMod::GorevBekliyor
                && toplam_waypoint > 0
                && nav.current_hn_index >= toplam_waypoint
            {
                7
            } else {
                0
            };

            let ileri_pwm = motor_kanalindan_oku(&motor_istek, motor_esleme.ileri1)
                .max(motor_kanalindan_oku(&motor_istek, motor_esleme.ileri2));
            let telemetri_paketi = GidenTelemetri {
                arac_enlem: gps.enlem as f64 / 10_000_000.0,
                arac_boylam: gps.boylam as f64 / 10_000_000.0,
                yer_hiz: gps.hiz as f32 / 1000.0,
                setpoint_hiz: match guncel_mod {
                    AracMod::Otonom | AracMod::EveDonus => ileri_pwm as f32 / 500.0,
                    AracMod::Manuel => son_manuel_ileri.clamp(0.0, 1.0) * 2.0,
                    _ => 0.0,
                },
                imu_veri: (imu.roll, imu.pitch, imu.yaw),
                setpoint_yaw: anlik_hedef_aci as f32,
                arac_mod: guncel_mod,
                aktif_waypoint,
                toplam_waypoint,
                hedef_mesafe_m: anlik_hedef_mesafe as f32,
                gps_hazir,
                imu_hazir,
                origin_hazir: nav.is_origin_set,
                telemetri_bagli,
                otonom_durum,
                motorlar_veri: (
                    motor_istek.iskeleon,
                    motor_istek.iskelearka,
                    motor_istek.sancakon,
                    motor_istek.sancakarka,
                ),
                motorlar_istek: (
                    motor_istek.iskeleon,
                    motor_istek.iskelearka,
                    motor_istek.sancakon,
                    motor_istek.sancakarka,
                ),
                dead_reckoning: dr_cikti,
            };
            // SD kayıt katmanına yalnızca en güncel tam durum verilir. Kayıt görevi
            // kendi 10 Hz zamanlamasıyla bu değeri CSV'ye yazar.
            let _ = sd_kayit_tx.send(telemetri_paketi.clone());

            // Telemetri kopukken bu kanalın dolması navigasyon döngüsünü kesinlikle
            // durdurmamalı. Paket sığmıyorsa eski telemetri atılır, dönüş sürer.
            let _ = yki_tx.try_send(telemetri_paketi);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gaz_sifirken_yatay_motor_calistirmaz() {
        let motor = manuel_motor_karistir(0.0, 1.0, MotorEsleme::default());
        assert_eq!(motor.iskeleon, 0);
        assert_eq!(motor.iskelearka, 0);
        assert_eq!(motor.sancakon, 0);
        assert_eq!(motor.sancakarka, 0);
    }

    #[test]
    fn iki_ileri_motor_ayni_komutu_alir() {
        let esleme = MotorEsleme::default();
        let motor = manuel_motor_karistir(0.5, 0.0, esleme);
        let kanal = |m: &MotorVeri, no: u8| match no {
            1 => m.iskeleon,
            2 => m.iskelearka,
            3 => m.sancakon,
            4 => m.sancakarka,
            _ => 0,
        };
        assert_eq!(kanal(&motor, esleme.ileri1), 500);
        assert_eq!(kanal(&motor, esleme.ileri2), 500);
    }
}
