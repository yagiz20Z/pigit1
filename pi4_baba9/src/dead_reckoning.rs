use std::env;

use crate::veri_tipleri::{
    DeadReckoningTelemetri, GpsVeri, MotorEsleme, MotorVeri,
};

const METRE_BASINA_ENLEM_DERECESI: f64 = 1.0 / 111_320.0;
const VARSAYILAN_MAX_ILERI_HIZ_M_S: f64 = 2.0;
const VARSAYILAN_MAX_YATAY_HIZ_M_S: f64 = 0.45;
const VARSAYILAN_PWM_OLU_BOLGE: f64 = 0.03;
const MAX_DT_S: f64 = 0.25;

#[derive(Debug, Clone, Copy)]
pub struct DeadReckoningAyar {
    /// İki ileri motor 1000 komutundayken kabul edilen yaklaşık tekne hızı.
    pub max_ileri_hiz_m_s: f64,
    /// Sağ/sol yatay motor 1000 komutundayken kabul edilen yaklaşık yanal hız.
    pub max_yatay_hiz_m_s: f64,
    /// 0..1 normalize PWM ölü bölgesi.
    pub pwm_olu_bolge: f64,
}

impl Default for DeadReckoningAyar {
    fn default() -> Self {
        Self {
            max_ileri_hiz_m_s: VARSAYILAN_MAX_ILERI_HIZ_M_S,
            max_yatay_hiz_m_s: VARSAYILAN_MAX_YATAY_HIZ_M_S,
            pwm_olu_bolge: VARSAYILAN_PWM_OLU_BOLGE,
        }
    }
}

impl DeadReckoningAyar {
    pub fn ortamdan_oku() -> Self {
        let varsayilan = Self::default();
        Self {
            max_ileri_hiz_m_s: pozitif_env_f64(
                "IDA_DR_MAX_FORWARD_MPS",
                varsayilan.max_ileri_hiz_m_s,
            ),
            max_yatay_hiz_m_s: sifir_veya_pozitif_env_f64(
                "IDA_DR_MAX_LATERAL_MPS",
                varsayilan.max_yatay_hiz_m_s,
            ),
            pwm_olu_bolge: aralik_env_f64(
                "IDA_DR_PWM_DEADZONE",
                varsayilan.pwm_olu_bolge,
                0.0,
                0.90,
            ),
        }
    }
}

fn pozitif_env_f64(ad: &str, varsayilan: f64) -> f64 {
    match env::var(ad) {
        Ok(deger) => match deger.parse::<f64>() {
            Ok(v) if v.is_finite() && v > 0.0 => v,
            _ => {
                eprintln!("{ad} geçersiz ({deger:?}); {varsayilan} kullanılacak.");
                varsayilan
            }
        },
        Err(_) => varsayilan,
    }
}

fn sifir_veya_pozitif_env_f64(ad: &str, varsayilan: f64) -> f64 {
    match env::var(ad) {
        Ok(deger) => match deger.parse::<f64>() {
            Ok(v) if v.is_finite() && v >= 0.0 => v,
            _ => {
                eprintln!("{ad} geçersiz ({deger:?}); {varsayilan} kullanılacak.");
                varsayilan
            }
        },
        Err(_) => varsayilan,
    }
}

fn aralik_env_f64(ad: &str, varsayilan: f64, min: f64, max: f64) -> f64 {
    match env::var(ad) {
        Ok(deger) => match deger.parse::<f64>() {
            Ok(v) if v.is_finite() && (min..=max).contains(&v) => v,
            _ => {
                eprintln!("{ad} geçersiz ({deger:?}); {varsayilan} kullanılacak.");
                varsayilan
            }
        },
        Err(_) => varsayilan,
    }
}

#[derive(Debug)]
pub struct DeadReckoning {
    ayar: DeadReckoningAyar,
    aktif: bool,
    baslangic_enlem: f64,
    baslangic_boylam: f64,
    cos_enlem: f64,
    dogu_m: f64,
    kuzey_m: f64,
    toplam_mesafe_m: f64,
    referans_yaw_deg: f64,
    son_cikti: DeadReckoningTelemetri,
}

impl DeadReckoning {
    pub fn new() -> Self {
        Self::with_ayar(DeadReckoningAyar::ortamdan_oku())
    }

    pub fn with_ayar(ayar: DeadReckoningAyar) -> Self {
        Self {
            ayar,
            aktif: false,
            baslangic_enlem: 0.0,
            baslangic_boylam: 0.0,
            cos_enlem: 1.0,
            dogu_m: 0.0,
            kuzey_m: 0.0,
            toplam_mesafe_m: 0.0,
            referans_yaw_deg: 0.0,
            son_cikti: DeadReckoningTelemetri::default(),
        }
    }

    pub fn aktif_mi(&self) -> bool {
        self.aktif
    }

    pub fn ayar(&self) -> DeadReckoningAyar {
        self.ayar
    }

    /// O andaki GPS konumunu ve IMU yaw değerini yeni merkez/referans kabul eder.
    /// Bu çağrı tahmini izi sıfırlar; GPS izi ise ayrı olarak devam eder.
    pub fn sifirla(&mut self, gps: &GpsVeri, yaw_deg: f32) -> bool {
        let enlem = gps.enlem as f64 / 10_000_000.0;
        let boylam = gps.boylam as f64 / 10_000_000.0;
        let yaw = yaw_deg as f64;

        if !koordinat_gecerli(enlem, boylam) || !yaw.is_finite() {
            return false;
        }

        self.aktif = true;
        self.baslangic_enlem = enlem;
        self.baslangic_boylam = boylam;
        self.cos_enlem = enlem.to_radians().cos().abs().max(1.0e-6);
        self.dogu_m = 0.0;
        self.kuzey_m = 0.0;
        self.toplam_mesafe_m = 0.0;
        self.referans_yaw_deg = yaw.rem_euclid(360.0);
        self.son_cikti = DeadReckoningTelemetri {
            aktif: true,
            enlem,
            boylam,
            mutlak_yaw_deg: self.referans_yaw_deg as f32,
            goreli_yaw_deg: 0.0,
            referans_yaw_deg: self.referans_yaw_deg as f32,
            ileri_hiz_m_s: 0.0,
            yatay_hiz_m_s: 0.0,
            toplam_mesafe_m: 0.0,
            gps_fark_m: 0.0,
        };
        true
    }

    /// PWM ve güncel IMU yaw üzerinden bağımsız tahmini konumu ilerletir.
    /// GPS yalnızca ekranda gösterilecek hata mesafesini ölçmek için kullanılır;
    /// tahmini izi GPS'e yapıştırmaz.
    pub fn guncelle(
        &mut self,
        dt_s: f64,
        motor: &MotorVeri,
        esleme: MotorEsleme,
        yaw_deg: f32,
        imu_hazir: bool,
        gps: Option<&GpsVeri>,
    ) -> DeadReckoningTelemetri {
        if !self.aktif {
            return DeadReckoningTelemetri::default();
        }

        let yaw = yaw_deg as f64;
        let dt = dt_s.clamp(0.0, MAX_DT_S);
        let mut ileri_hiz = 0.0;
        let mut yatay_hiz = 0.0;

        if imu_hazir && yaw.is_finite() && dt > 0.0 && esleme.gecerli() {
            let ileri1 = pwm_orani(
                motor_degeri(motor, esleme.ileri1),
                self.ayar.pwm_olu_bolge,
            );
            let ileri2 = pwm_orani(
                motor_degeri(motor, esleme.ileri2),
                self.ayar.pwm_olu_bolge,
            );
            let sol = pwm_orani(motor_degeri(motor, esleme.sol), self.ayar.pwm_olu_bolge);
            let sag = pwm_orani(motor_degeri(motor, esleme.sag), self.ayar.pwm_olu_bolge);

            ileri_hiz = ((ileri1 + ileri2) * 0.5) * self.ayar.max_ileri_hiz_m_s;
            // Pozitif değer aracın gövdesine göre sağ yönü temsil eder.
            yatay_hiz = (sag - sol) * self.ayar.max_yatay_hiz_m_s;

            let heading_rad = yaw.rem_euclid(360.0).to_radians();
            let dogu_hiz = ileri_hiz * heading_rad.sin() + yatay_hiz * heading_rad.cos();
            let kuzey_hiz = ileri_hiz * heading_rad.cos() - yatay_hiz * heading_rad.sin();
            let dogu_adim = dogu_hiz * dt;
            let kuzey_adim = kuzey_hiz * dt;

            self.dogu_m += dogu_adim;
            self.kuzey_m += kuzey_adim;
            self.toplam_mesafe_m += dogu_adim.hypot(kuzey_adim);
        }

        let enlem = self.baslangic_enlem + self.kuzey_m * METRE_BASINA_ENLEM_DERECESI;
        let boylam = self.baslangic_boylam
            + self.dogu_m * METRE_BASINA_ENLEM_DERECESI / self.cos_enlem;
        let mutlak_yaw = if yaw.is_finite() {
            yaw.rem_euclid(360.0)
        } else {
            self.son_cikti.mutlak_yaw_deg as f64
        };
        let goreli_yaw = normalize_signed_deg(mutlak_yaw - self.referans_yaw_deg);
        let gps_fark_m = gps
            .filter(|g| gps_koordinat_gecerli(g))
            .map(|g| {
                let gps_enlem = g.enlem as f64 / 10_000_000.0;
                let gps_boylam = g.boylam as f64 / 10_000_000.0;
                iki_konum_arasi_m(enlem, boylam, gps_enlem, gps_boylam)
            })
            .unwrap_or(self.son_cikti.gps_fark_m as f64);

        self.son_cikti = DeadReckoningTelemetri {
            aktif: true,
            enlem,
            boylam,
            mutlak_yaw_deg: mutlak_yaw as f32,
            goreli_yaw_deg: goreli_yaw as f32,
            referans_yaw_deg: self.referans_yaw_deg as f32,
            ileri_hiz_m_s: ileri_hiz as f32,
            yatay_hiz_m_s: yatay_hiz as f32,
            toplam_mesafe_m: self.toplam_mesafe_m as f32,
            gps_fark_m: gps_fark_m as f32,
        };
        self.son_cikti
    }
}

fn motor_degeri(motor: &MotorVeri, kanal: u8) -> u16 {
    match kanal {
        1 => motor.iskeleon,
        2 => motor.iskelearka,
        3 => motor.sancakon,
        4 => motor.sancakarka,
        _ => 0,
    }
}

fn pwm_orani(pwm: u16, olu_bolge: f64) -> f64 {
    let oran = (pwm.min(1000) as f64) / 1000.0;
    if oran <= olu_bolge {
        0.0
    } else {
        ((oran - olu_bolge) / (1.0 - olu_bolge)).clamp(0.0, 1.0)
    }
}

fn normalize_signed_deg(mut derece: f64) -> f64 {
    while derece > 180.0 {
        derece -= 360.0;
    }
    while derece < -180.0 {
        derece += 360.0;
    }
    derece
}

fn koordinat_gecerli(enlem: f64, boylam: f64) -> bool {
    enlem.is_finite()
        && boylam.is_finite()
        && (-90.0..=90.0).contains(&enlem)
        && (-180.0..=180.0).contains(&boylam)
        && !(enlem == 0.0 && boylam == 0.0)
}

fn gps_koordinat_gecerli(gps: &GpsVeri) -> bool {
    koordinat_gecerli(
        gps.enlem as f64 / 10_000_000.0,
        gps.boylam as f64 / 10_000_000.0,
    )
}

fn iki_konum_arasi_m(enlem1: f64, boylam1: f64, enlem2: f64, boylam2: f64) -> f64 {
    let ortalama_enlem = ((enlem1 + enlem2) * 0.5).to_radians().cos();
    let kuzey = (enlem2 - enlem1) * 111_320.0;
    let dogu = (boylam2 - boylam1) * 111_320.0 * ortalama_enlem;
    dogu.hypot(kuzey)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gps() -> GpsVeri {
        GpsVeri {
            algi_boyut: 3,
            uydu_sayi: 10,
            enlem: 410_000_000,
            boylam: 290_000_000,
            ..GpsVeri::default()
        }
    }

    fn test_ayar() -> DeadReckoningAyar {
        DeadReckoningAyar {
            max_ileri_hiz_m_s: 2.0,
            max_yatay_hiz_m_s: 0.0,
            pwm_olu_bolge: 0.0,
        }
    }

    #[test]
    fn sifir_pwm_konumu_degistirmez() {
        let mut dr = DeadReckoning::with_ayar(test_ayar());
        assert!(dr.sifirla(&gps(), 0.0));
        let c = dr.guncelle(
            1.0,
            &MotorVeri::default(),
            MotorEsleme::default(),
            0.0,
            true,
            Some(&gps()),
        );
        assert!((c.enlem - 41.0).abs() < 1.0e-9);
        assert!((c.boylam - 29.0).abs() < 1.0e-9);
    }

    #[test]
    fn kuzeye_ileri_pwm_konumu_ilerletir() {
        let mut dr = DeadReckoning::with_ayar(test_ayar());
        assert!(dr.sifirla(&gps(), 0.0));
        let esleme = MotorEsleme::default();
        let mut motor = MotorVeri::default();
        super::set_test_motor(&mut motor, esleme.ileri1, 1000);
        super::set_test_motor(&mut motor, esleme.ileri2, 1000);
        let c = dr.guncelle(1.0, &motor, esleme, 0.0, true, None);
        // Güvenlik nedeniyle tek döngü dt'si 250 ms ile sınırlandırılır: yaklaşık 0.5 m.
        assert!((c.toplam_mesafe_m - 0.5).abs() < 0.01);
        assert!(c.enlem > 41.0);
    }

    #[test]
    fn referansa_gore_yaw_eksi_arti_180_arasinda() {
        let mut dr = DeadReckoning::with_ayar(test_ayar());
        assert!(dr.sifirla(&gps(), 350.0));
        let c = dr.guncelle(
            0.05,
            &MotorVeri::default(),
            MotorEsleme::default(),
            10.0,
            true,
            None,
        );
        assert!((c.goreli_yaw_deg - 20.0).abs() < 0.01);
    }

    // Testlerin özel kanal yazıcısı üretim koduna açılmasın.
    fn _dummy() {}
}

#[cfg(test)]
fn set_test_motor(motor: &mut MotorVeri, kanal: u8, deger: u16) {
    match kanal {
        1 => motor.iskeleon = deger,
        2 => motor.iskelearka = deger,
        3 => motor.sancakon = deger,
        4 => motor.sancakarka = deger,
        _ => {}
    }
}
