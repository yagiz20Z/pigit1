use crate::veri_tipleri::GelenTelemetri::RotaBelirle;

#[derive(Debug, Clone, Default)]
pub struct MotorVeri
{
    pub iskeleon: u16,
    pub iskelearka: u16,
    pub sancakon: u16,
    pub sancakarka: u16,
}
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ImuVeri
{
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
pub struct GpsVeri
{
    pub algi_boyut: u8,
    pub uydu_sayi: u8,
    pub boylam: i32,
    pub enlem: i32,
    pub yukseklik_mm: i32,
    pub hiz: i32,
    pub yonelim: i32,
    pub zaman_ms: u64,

}
#[derive(Clone,Debug, Default)]
pub struct LidarNokta
{
    pub aci: f32,
    pub mesafe_mm: f32,
    pub kalite: u8
}

#[derive(Debug, Default)]
pub struct LidarVeri
{
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
}

impl AracMod {
    pub fn from_u8(value: u8) -> Self {
        match value {
            1 => AracMod::Otonom,
            2 => AracMod::GorevBekliyor,
            3 => AracMod::AcilDurum,
            _ => AracMod::Manuel,
        }
    }
}

#[derive(Debug, Clone)]
pub enum GelenTelemetri
{
    RotaBelirle(Vec<(f64, f64)>),
    ModDegistir(AracMod),
    GoreviBaslat,
    AcilDurdur,
    ManuelKontrol(f32,f32),
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
}

pub fn calc_checksum(payload: &str) -> String {
    let mut sum: u32 = 0;
    for byte in payload.bytes() {
        sum += byte as u32;
    }
    format!("{:02X}", sum % 256)
}

impl GidenTelemetri {
    pub fn to_rf_strings(&self) -> (String, String) {
        let nav_payload = format!(
            "NAV:{:.6},{:.6},{:.2},{:.2},{:.1},{:.1},{:.1},{:.1},{}",
            self.arac_enlem, self.arac_boylam, self.yer_hiz, self.setpoint_hiz,
            self.imu_veri.0, self.imu_veri.1, self.imu_veri.2, self.setpoint_yaw,
            self.arac_mod as u8
        );
        
        let mot_payload = format!(
            "MOT:{},{},{},{},{},{},{},{}",
            self.motorlar_veri.0, self.motorlar_veri.1, self.motorlar_veri.2, self.motorlar_veri.3,
            self.motorlar_istek.0, self.motorlar_istek.1, self.motorlar_istek.2, self.motorlar_istek.3
        );

        (
            format!("{}*{}\n", nav_payload, calc_checksum(&nav_payload)),
            format!("{}*{}\n", mot_payload, calc_checksum(&mot_payload))
        )
    }
}