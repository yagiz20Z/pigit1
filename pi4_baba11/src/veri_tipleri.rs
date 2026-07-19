#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorEsleme {
    /// Sola yatay hareketi sağlayan fiziksel motor kanalı (1..=4)
    pub sol: u8,
    /// Birinci ileri motorun fiziksel kanalı (1..=4)
    pub ileri1: u8,
    /// Sağa yatay hareketi sağlayan fiziksel motor kanalı (1..=4)
    pub sag: u8,
    /// İkinci ileri motorun fiziksel kanalı (1..=4)
    pub ileri2: u8,
}

impl Default for MotorEsleme {
    fn default() -> Self {
        Self {
            sol: 2,
            ileri1: 4,
            sag: 1,
            ileri2: 3,
        }
    }
}

impl MotorEsleme {
    pub fn gecerli(&self) -> bool {
        let kanallar = [self.sol, self.ileri1, self.sag, self.ileri2];
        let mut goruldu = [false; 5];

        for kanal in kanallar {
            if !(1..=4).contains(&kanal) || goruldu[kanal as usize] {
                return false;
            }
            goruldu[kanal as usize] = true;
        }

        true
    }
}

#[derive(Debug, Clone, Default)]
pub struct MotorVeri {
    pub iskeleon: u16,
    pub iskelearka: u16,
    pub sancakon: u16,
    pub sancakarka: u16,
}
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ImuVeri {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub zaman_ms: u64,
}
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsVeri {
    pub algi_boyut: u8,
    pub uydu_sayi: u8,
    pub boylam: i32,
    pub enlem: i32,
    pub yukseklik_mm: i32,
    pub hiz: i32,
    pub yonelim: i32,
    pub zaman_ms: u64,
}
#[allow(dead_code)]
#[derive(Clone, Debug, Default)]
pub struct LidarNokta {
    pub aci: f32,
    pub mesafe_mm: f32,
    pub kalite: u8,
}

#[allow(dead_code)]
#[derive(Debug, Default)]
pub struct LidarVeri {
    pub noktalar: Vec<LidarNokta>,
    pub zaman_ms: u64,
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum AracMod {
    #[default]
    Manuel = 0,

    Otonom = 1,
    GorevBekliyor = 2,
    AcilDurum = 3,
    /// Telemetri bağlantısı kaybolduğunda güvenli dönüş konumuna ilerler.
    EveDonus = 4,
}

impl AracMod {
    pub fn from_u8(value: u8) -> Self {
        match value {
            1 => AracMod::Otonom,
            2 => AracMod::GorevBekliyor,
            3 => AracMod::AcilDurum,
            4 => AracMod::EveDonus,
            _ => AracMod::Manuel,
        }
    }
}

#[derive(Debug, Clone)]
pub enum GelenTelemetri {
    RotaBelirle(Vec<(f64, f64)>),
    ModDegistir(AracMod),
    GoreviBaslat,
    AcilDurdur,
    ManuelKontrol(f32, f32),
    /// Sıra: sol, ileri1, sağ, ileri2. Değerler fiziksel M1..M4 kanallarıdır.
    MotorEslemeDegistir(MotorEsleme),
    /// YKİ/telemetri istasyonunun güvenli dönüş koordinatı.
    EvKonumuBelirle(f64, f64),
    /// Telemetri seri/RF bağlantısının kullanılabilir hale geldiğini bildirir.
    TelemetriBaglandi,
    /// Telemetri seri/RF bağlantısının kaybolduğunu bildirir.
    TelemetriKoptu,
    /// İsteğe bağlı bağlantı watchdog paketi.
    TelemetriHeartbeat,
    /// O anki GPS konumu ve IMU yaw değerini tahmini iz için yeni merkez kabul eder.
    DeadReckoningSifirla,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct DeadReckoningTelemetri {
    pub aktif: bool,
    pub enlem: f64,
    pub boylam: f64,
    /// IMU'dan gelen 0..360 mutlak yaw.
    pub mutlak_yaw_deg: f32,
    /// Dead reckoning sıfırlandığı andaki yöne göre -180..180 dönüş açısı.
    pub goreli_yaw_deg: f32,
    pub referans_yaw_deg: f32,
    pub ileri_hiz_m_s: f32,
    pub yatay_hiz_m_s: f32,
    pub toplam_mesafe_m: f32,
    pub gps_fark_m: f32,
}

#[derive(Debug, Default, Clone)]
pub struct GidenTelemetri {
    pub arac_enlem: f64,
    pub arac_boylam: f64,
    pub yer_hiz: f32,
    pub setpoint_hiz: f32,
    pub imu_veri: (f32, f32, f32),
    pub setpoint_yaw: f32,
    pub arac_mod: AracMod,
    pub motorlar_veri: (u16, u16, u16, u16),
    pub motorlar_istek: (u16, u16, u16, u16),
    pub dead_reckoning: DeadReckoningTelemetri,
}

pub fn calc_checksum(payload: &str) -> String {
    let mut sum: u32 = 0;
    for byte in payload.bytes() {
        sum += byte as u32;
    }
    format!("{:02X}", sum % 256)
}

impl GidenTelemetri {
    pub fn to_rf_strings(&self) -> (String, String, String) {
        let nav_payload = format!(
            "NAV:{:.6},{:.6},{:.2},{:.2},{:.1},{:.1},{:.1},{:.1},{}",
            self.arac_enlem,
            self.arac_boylam,
            self.yer_hiz,
            self.setpoint_hiz,
            self.imu_veri.0,
            self.imu_veri.1,
            self.imu_veri.2,
            self.setpoint_yaw,
            self.arac_mod as u8
        );

        let mot_payload = format!(
            "MOT:{},{},{},{},{},{},{},{}",
            self.motorlar_veri.0,
            self.motorlar_veri.1,
            self.motorlar_veri.2,
            self.motorlar_veri.3,
            self.motorlar_istek.0,
            self.motorlar_istek.1,
            self.motorlar_istek.2,
            self.motorlar_istek.3
        );

        let dr = self.dead_reckoning;
        let dr_payload = format!(
            "DR:{:.7},{:.7},{:.2},{:.2},{:.2},{:.3},{:.3},{:.2},{:.2},{}",
            dr.enlem,
            dr.boylam,
            dr.mutlak_yaw_deg,
            dr.goreli_yaw_deg,
            dr.referans_yaw_deg,
            dr.ileri_hiz_m_s,
            dr.yatay_hiz_m_s,
            dr.toplam_mesafe_m,
            dr.gps_fark_m,
            dr.aktif as u8,
        );

        (
            format!("{}*{}\n", nav_payload, calc_checksum(&nav_payload)),
            format!("{}*{}\n", mot_payload, calc_checksum(&mot_payload)),
            format!("{}*{}\n", dr_payload, calc_checksum(&dr_payload)),
        )
    }
}
