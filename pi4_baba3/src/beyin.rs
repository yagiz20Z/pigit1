use tokio::{sync::{mpsc, watch}, time::interval};
use crate::veri_tipleri::{AracMod, GelenTelemetri, GidenTelemetri, GpsVeri, ImuVeri, MotorVeri};
use std::{f64::consts::PI, time::{Duration, Instant}};

pub struct PidKontrolcu
{
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    integral: f64,
    onceki_hata: f64,
    integral_siniri: f64,
}

impl PidKontrolcu
{
    pub fn new(kp: f64, ki: f64, kd: f64, integral_siniri: f64) -> Self
    {
        Self
        {
            kp,
            ki,
            kd,
            integral: 0.0,
            onceki_hata: 0.0,
            integral_siniri,
        }
    }

    pub fn guncelle(&mut self, hata: f64, dt: f64) -> f64
    {
        self.integral += hata * dt;
        self.integral = self.integral.clamp(-self.integral_siniri, self.integral_siniri);
        let turev = if dt > 0.0 { ((hata - self.onceki_hata) / dt).clamp(-100.0, 100.0) } else { 0.0 };
        self.onceki_hata = hata;
        (self.kp * hata) + (self.ki * self.integral) + (self.kd * turev)
    }
}

const IHMALACI: f64 = 3.0;
const HEDEF_TOLERANS: f64 = 2.5;

pub struct NavData
{
    origin_enlem: f64,
    origin_boylam: f64,
    cos_enlem: f64,
    is_origin_set: bool,
    gps_ornek_sayaci: u8,
    ornek_enlem_toplam: i64,
    ornek_boylam_toplam: i64,
    son_ornek_enlem: i32,
    son_ornek_boylam: i32,
    current_x: f64,
    current_y: f64,
    current_yaw: f64,
    hedef_noktalar: Vec<(f64, f64)>,
    current_hn_index: usize,
}

impl NavData
{
    pub fn new() -> Self
    {
        Self
        {
            origin_enlem: 0.0,
            origin_boylam: 0.0,
            cos_enlem: 1.0,
            is_origin_set: false,
            gps_ornek_sayaci: 0,
            ornek_enlem_toplam: 0,
            ornek_boylam_toplam: 0,
            son_ornek_enlem: 0,
            son_ornek_boylam: 0,
            current_x: 0.0,
            current_y: 0.0,
            current_yaw: 0.0,
            hedef_noktalar: Vec::new(),
            current_hn_index: 0,
        }
    }

    pub fn guvenli_origin_belirle(&mut self, gps: &GpsVeri) -> bool
    {
        let yeterli_fix = gps.algi_boyut >= 3; 
        let yeterli_uydu = gps.uydu_sayi >= 6;
        let gecerli_koordinat = gps.enlem != 0 && gps.boylam != 0;
        if !yeterli_fix || !yeterli_uydu || !gecerli_koordinat
        {
            self.gps_ornek_sayaci = 0;
            self.ornek_enlem_toplam = 0;
            self.ornek_boylam_toplam = 0;
            return false;
        }
        if gps.enlem == self.son_ornek_enlem && gps.boylam == self.son_ornek_boylam
        {
            return false;
        }
        self.son_ornek_enlem = gps.enlem;
        self.son_ornek_boylam = gps.boylam;
        self.ornek_enlem_toplam += gps.enlem as i64;
        self.ornek_boylam_toplam += gps.boylam as i64;
        self.gps_ornek_sayaci += 1;
        if self.gps_ornek_sayaci >= 10
        {
            let ortalama_enlem = (self.ornek_enlem_toplam / 10) as i32;
            let ortalama_boylam = (self.ornek_boylam_toplam / 10) as i32;
            self.origin_enlem = ortalama_enlem as f64 / 10_000_000.0;
            self.origin_boylam = ortalama_boylam as f64 / 10_000_000.0;
            self.cos_enlem = (self.origin_enlem * std::f64::consts::PI / 180.0).cos();
            self.is_origin_set = true;
            self.son_ornek_boylam = 0;
            self.son_ornek_enlem = 0;
            self.ornek_enlem_toplam = 0;
            self.ornek_boylam_toplam= 0;
            return true;
        }
        false
    }

    pub fn guncelle_konum(&mut self, lat_i32: i32, lon_i32: i32, yaw: f64)
    {
        let lat = lat_i32 as f64 / 10_000_000.0;
        let lon = lon_i32 as f64 / 10_000_000.0;
        self.current_y = (lat - self.origin_enlem) * 111_320.0;
        self.current_x = (lon - self.origin_boylam) * 111_320.0 * self.cos_enlem;
        self.current_yaw = yaw;
    }

    pub fn set_rota(&mut self, noktalar: Vec<(f64, f64)>)
    {
        self.hedef_noktalar = noktalar;
        self.current_hn_index = 0;
    }

    pub fn guncel_hedef(&self) -> Option<(f64, f64)>
    {
        if self.current_hn_index < self.hedef_noktalar.len()
        {
            Some(self.hedef_noktalar[self.current_hn_index])
        }
        else
        {
            None
        }
    }
    pub fn calc_mesafe(&self, hedef_x: f64, hedef_y: f64) -> f64
    {
        let dx = hedef_x - self.current_x;
        let dy = hedef_y - self.current_y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn calc_hedefeaci(&self, hedef_x: f64, hedef_y: f64) -> f64
    {
        let dx = hedef_x - self.current_x;
        let dy = hedef_y - self.current_y;
        dx.atan2(dy) * 180.0 / PI
    }

    pub fn bakisyonu_hata(&self, hedefeaci: f64) -> f64
    {
        let mut hata = hedefeaci - self.current_yaw;
        while hata > 180.0 { hata -= 360.0 }
        while hata < -180.0 { hata += 360.0 }
        hata
    }
}

pub async fn nav_task(
    imu_rx: watch::Receiver<ImuVeri>,
    gps_rx: watch::Receiver<GpsVeri>,
    motor_tx: mpsc::Sender<MotorVeri>,
    mut rx_yki: mpsc::Receiver<GelenTelemetri>,
    yki_tx: mpsc::Sender<GidenTelemetri>,
)
{
    let mut tick = interval(Duration::from_millis(50));
    let mut last_time = Instant::now();
    let mut nav = NavData::new();
    let mut pid = PidKontrolcu::new(4.0, 0.1, 0.5, 150.0);
    let base_hiz = 400.0;
    let mut kaba_donus_modu = false;
    let mut guncel_mod = AracMod::Manuel;
    let mut telemetri_sayaci = 0;
    let mut son_manuel_gaz: f32 = 0.0;
    let mut son_manuel_aci: f32 = 0.0;
    loop
    {
        tick.tick().await;
        let simdi = Instant::now();
        let dt = simdi.duration_since(last_time).as_secs_f64();
        last_time = simdi;

        while let Ok(komut) = rx_yki.try_recv()
        {
            match komut
            {
                GelenTelemetri::AcilDurdur =>
                {
                    guncel_mod = AracMod::AcilDurum;
                    println!("Acil durdur!");
                }
                GelenTelemetri::GoreviBaslat =>
                {
                    if guncel_mod != AracMod::AcilDurum
                    {
                        if !nav.hedef_noktalar.is_empty()
                        {
                            guncel_mod = AracMod::Otonom;
                            println!("Otonom Modu");
                        }
                        else
                        {
                            println!("Rotasız otonom moduna geçilmez.");
                        }
                    }
                }
                GelenTelemetri::ModDegistir(istenen_mod) =>
                {
                    guncel_mod = istenen_mod;
                    println!("Mod değişimi: {:?}", guncel_mod);
                }
                GelenTelemetri::ManuelKontrol(gaz, aci) =>
                {
                    guncel_mod = AracMod::Manuel;
                    son_manuel_gaz = gaz;
                    son_manuel_aci = aci;
                }
                GelenTelemetri::RotaBelirle(noktalar) =>
                {
                    nav.set_rota(noktalar);
                    guncel_mod = AracMod::GorevBekliyor;
                    println!("Yeni rota: ({} nokta)", nav.hedef_noktalar.len());
                }
            }
        }
        let gps = gps_rx.borrow().clone();
        let imu = imu_rx.borrow().clone();
        let mut motor_istek = MotorVeri::default();
        let mut anlik_hedef_aci = imu.yaw as f64;
        if guncel_mod == AracMod::Manuel
        {
            let motor_gaz = (son_manuel_gaz * 1000.0).clamp(0.0, 1000.0) as u16;
            let motor_aci: f32 = (son_manuel_aci * 1000.0).clamp(-1000.0, 1000.0);
            motor_istek.iskelearka = motor_gaz;
            motor_istek.sancakarka = motor_gaz;
            if motor_aci > 0.0
            {
                motor_istek.sancakon = 0;
                motor_istek.iskeleon = motor_aci as u16;
            }
            else if motor_aci < 0.0
            {
                motor_istek.iskeleon = 0;
                motor_istek.sancakon = (-motor_aci) as u16;
            }
            else
            {
                motor_istek.iskeleon = 0;
                motor_istek.sancakon = 0;
            }
        }
        if !nav.is_origin_set
        {
            nav.guvenli_origin_belirle(&gps);
        }
        else
        {
            nav.guncelle_konum(gps.enlem, gps.boylam, imu.yaw as f64);
            anlik_hedef_aci = imu.yaw as f64;
            if guncel_mod == AracMod::Otonom
            {
                if let Some((hedef_lat, hedef_lon)) = nav.guncel_hedef()
                {
                    let hedefy_metre = (hedef_lat - nav.origin_enlem) * 111_320.0;
                    let hedefx_metre = (hedef_lon - nav.origin_boylam) * 111_320.0 * nav.cos_enlem;
                    let mesafe = nav.calc_mesafe(hedefx_metre, hedefy_metre);
                    let hedefe_aci = nav.calc_hedefeaci(hedefx_metre, hedefy_metre);
                    let hata = nav.bakisyonu_hata(hedefe_aci);
                    anlik_hedef_aci = hedefe_aci;
                    if mesafe < HEDEF_TOLERANS
                    {
                        println!("{}. hedef noktasına ulaşıldı!", nav.current_hn_index + 1);
                        nav.current_hn_index += 1;
                        pid.integral = 0.0;
                        pid.onceki_hata = hata;
                        kaba_donus_modu = false;
                    }
                    else
                    {
                        let onceki_kaba = kaba_donus_modu;
                        if hata.abs() > 30.0
                        {
                            kaba_donus_modu = true;
                        } else if hata.abs() < 15.0
                        {
                            kaba_donus_modu = false;
                        }
                        if onceki_kaba && !kaba_donus_modu
                        {
                            pid.integral = 0.0;
                            pid.onceki_hata = hata;
                        }
                        let donus_gucu = if kaba_donus_modu { 0.0 } else { pid.guncelle(hata, dt) };
                        if kaba_donus_modu
                        {
                            if hata < 0.0
                            {
                                motor_istek.sancakon = 400;
                            }
                            else
                            {
                                motor_istek.iskeleon = 400;
                            }
                        }
                        else
                        {
                            let duzeltme = if hata.abs() < IHMALACI { 0.0 } else { donus_gucu };
                            motor_istek.iskelearka = (base_hiz - duzeltme).clamp(0.0, 1000.0) as u16;
                            motor_istek.sancakarka = (base_hiz + duzeltme).clamp(0.0, 1000.0) as u16;
                        }
                    }
                }
                else
                {
                    guncel_mod = AracMod::GorevBekliyor;
                }
            }
        }

        let _ = motor_tx.send(motor_istek.clone()).await;

        telemetri_sayaci += 1;
        if telemetri_sayaci % 2 == 0
        {
            let telemetri_paketi = GidenTelemetri
            {
                arac_enlem: gps.enlem as f64 / 10_000_000.0,
                arac_boylam: gps.boylam as f64 / 10_000_000.0,
                yer_hiz: gps.hiz as f32,
                setpoint_hiz: match guncel_mod
                {
                    AracMod::Otonom => 2.0,
                    AracMod::Manuel => son_manuel_gaz.clamp(0.0, 1.0) * 2.0,
                    _ => 0.0,
                },
                imu_veri: (imu.roll, imu.pitch, imu.yaw),
                setpoint_yaw: anlik_hedef_aci as f32,
                arac_mod: guncel_mod,
                motorlar_veri: (motor_istek.iskeleon, motor_istek.iskelearka, motor_istek.sancakon, motor_istek.sancakarka),
                motorlar_istek: (motor_istek.iskeleon, motor_istek.iskelearka, motor_istek.sancakon, motor_istek.sancakarka),
            };
            let _ = yki_tx.send(telemetri_paketi).await;
        }
    }
}
