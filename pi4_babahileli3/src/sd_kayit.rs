use std::{
    env, io,
    path::PathBuf,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use tokio::{
    fs::{self, File},
    io::{AsyncWriteExt, BufWriter},
    sync::watch,
    time::{MissedTickBehavior, interval},
};

use crate::veri_tipleri::{GidenTelemetri, GpsVeri, ImuVeri};

const BIRLESIK_KAYIT_ARALIGI: Duration = Duration::from_millis(100); // 10 Hz
const FLUSH_ARALIGI: Duration = Duration::from_secs(1);
const SD_SYNC_ARALIGI: Duration = Duration::from_secs(5);

fn unix_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

fn kayit_kok_dizini() -> PathBuf {
    if let Ok(dizin) = env::var("IDA_LOG_DIR") {
        let dizin = dizin.trim();
        if !dizin.is_empty() {
            return PathBuf::from(dizin);
        }
    }

    // Raspberry Pi'nin normal kurulumunda HOME dizini SD kart üzerindedir.
    // Harici SD/USB kullanılacaksa IDA_LOG_DIR ile bağlama noktası verilebilir.
    env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."))
        .join("ida_logs")
}

async fn csv_olustur(yol: PathBuf, baslik: &str) -> io::Result<BufWriter<File>> {
    let dosya = File::create(yol).await?;
    let mut yazici = BufWriter::new(dosya);
    yazici.write_all(baslik.as_bytes()).await?;
    yazici.write_all(b"\n").await?;
    yazici.flush().await?;
    Ok(yazici)
}

/// GPS, BNO085 IMU, motor komutları, setpointler, araç modu ve dead-reckoning
/// verilerini Pi'nin SD kartına CSV olarak kaydeder.
///
/// Varsayılan dizin: `$HOME/ida_logs/oturum_<unix_ms>`
/// Harici SD kart için: `IDA_LOG_DIR=/media/.../ida_logs`
pub async fn sd_kayit_task(
    mut imu_rx: watch::Receiver<ImuVeri>,
    mut gps_rx: watch::Receiver<GpsVeri>,
    mut durum_rx: watch::Receiver<GidenTelemetri>,
    mut kapat_rx: watch::Receiver<bool>,
) -> io::Result<()> {
    let kok = kayit_kok_dizini();
    fs::create_dir_all(&kok).await?;

    let oturum_baslangic_ms = unix_ms();
    let oturum = kok.join(format!("oturum_{oturum_baslangic_ms}"));
    fs::create_dir_all(&oturum).await?;

    let imu_yolu = oturum.join("imu.csv");
    let gps_yolu = oturum.join("gps.csv");
    let tum_yolu = oturum.join("tum_veriler.csv");

    let mut imu_dosya = csv_olustur(
        imu_yolu,
        "host_unix_ms,imu_zaman_ms,roll_deg,pitch_deg,yaw_deg,gx,gy,gz,ax,ay,az",
    )
    .await?;

    let mut gps_dosya = csv_olustur(
        gps_yolu,
        "host_unix_ms,gps_zaman_ms,fix_tipi,uydu_sayisi,enlem_deg,boylam_deg,yukseklik_m,yer_hizi_m_s,yonelim_deg",
    )
    .await?;

    let mut tum_dosya = csv_olustur(
        tum_yolu,
        concat!(
            "host_unix_ms,oturum_ms,",
            "gps_zaman_ms,gps_fix_tipi,gps_uydu_sayisi,enlem_deg,boylam_deg,yukseklik_m,yer_hizi_m_s,gps_yonelim_deg,",
            "imu_zaman_ms,roll_deg,pitch_deg,yaw_deg,gx,gy,gz,ax,ay,az,",
            "hiz_setpoint_m_s,yaw_setpoint_deg,arac_mod,",
            "motor_m1,motor_m2,motor_m3,motor_m4,motor_istek_m1,motor_istek_m2,motor_istek_m3,motor_istek_m4,",
            "dr_aktif,dr_enlem_deg,dr_boylam_deg,dr_mutlak_yaw_deg,dr_goreli_yaw_deg,dr_referans_yaw_deg,",
            "dr_ileri_hiz_m_s,dr_yatay_hiz_m_s,dr_toplam_mesafe_m,dr_gps_fark_m"
        ),
    )
    .await?;

    let bilgi = format!(
        "IDA SD kayıt oturumu\nBaşlangıç Unix ms: {oturum_baslangic_ms}\nKayıt hızı: birleşik 10 Hz; IMU/GPS her yeni pakette\n"
    );
    fs::write(oturum.join("oturum_bilgisi.txt"), bilgi).await?;

    println!("SD kayıt başladı: {}", oturum.display());
    println!("  - imu.csv        : her yeni IMU paketi");
    println!("  - gps.csv        : her yeni GPS paketi");
    println!("  - tum_veriler.csv: 10 Hz birleşik kayıt");

    let mut imu = *imu_rx.borrow();
    let mut gps = *gps_rx.borrow();
    let mut durum = durum_rx.borrow().clone();

    let mut birlesik_tick = interval(BIRLESIK_KAYIT_ARALIGI);
    birlesik_tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut flush_tick = interval(FLUSH_ARALIGI);
    flush_tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut sync_tick = interval(SD_SYNC_ARALIGI);
    sync_tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
        tokio::select! {
            sonuc = imu_rx.changed() => {
                if sonuc.is_err() {
                    return Err(io::Error::new(io::ErrorKind::BrokenPipe, "IMU kayıt kanalı kapandı"));
                }

                imu = *imu_rx.borrow_and_update();
                let satir = format!(
                    "{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
                    unix_ms(),
                    imu.zaman_ms,
                    imu.roll,
                    imu.pitch,
                    imu.yaw,
                    imu.gx,
                    imu.gy,
                    imu.gz,
                    imu.ax,
                    imu.ay,
                    imu.az,
                );
                imu_dosya.write_all(satir.as_bytes()).await?;
            }
            sonuc = gps_rx.changed() => {
                if sonuc.is_err() {
                    return Err(io::Error::new(io::ErrorKind::BrokenPipe, "GPS kayıt kanalı kapandı"));
                }

                gps = *gps_rx.borrow_and_update();
                let satir = format!(
                    "{},{},{},{},{:.7},{:.7},{:.3},{:.3},{:.5}\n",
                    unix_ms(),
                    gps.zaman_ms,
                    gps.algi_boyut,
                    gps.uydu_sayi,
                    gps.enlem as f64 / 10_000_000.0,
                    gps.boylam as f64 / 10_000_000.0,
                    gps.yukseklik_mm as f64 / 1000.0,
                    gps.hiz as f64 / 1000.0,
                    gps.yonelim as f64 / 100_000.0,
                );
                gps_dosya.write_all(satir.as_bytes()).await?;
            }
            sonuc = durum_rx.changed() => {
                if sonuc.is_err() {
                    return Err(io::Error::new(io::ErrorKind::BrokenPipe, "Durum kayıt kanalı kapandı"));
                }
                durum = durum_rx.borrow_and_update().clone();
            }
            _ = birlesik_tick.tick() => {
                let simdi_ms = unix_ms();
                let dr = durum.dead_reckoning;
                let satir = format!(
                    concat!(
                        "{},{},{},{},{},{:.7},{:.7},{:.3},{:.3},{:.5},",
                        "{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},",
                        "{:.3},{:.6},{},",
                        "{},{},{},{},{},{},{},{},",
                        "{},{:.7},{:.7},{:.6},{:.6},{:.6},{:.6},{:.6},{:.3},{:.3}\n"
                    ),
                    simdi_ms,
                    simdi_ms.saturating_sub(oturum_baslangic_ms),
                    gps.zaman_ms,
                    gps.algi_boyut,
                    gps.uydu_sayi,
                    gps.enlem as f64 / 10_000_000.0,
                    gps.boylam as f64 / 10_000_000.0,
                    gps.yukseklik_mm as f64 / 1000.0,
                    gps.hiz as f64 / 1000.0,
                    gps.yonelim as f64 / 100_000.0,
                    imu.zaman_ms,
                    imu.roll,
                    imu.pitch,
                    imu.yaw,
                    imu.gx,
                    imu.gy,
                    imu.gz,
                    imu.ax,
                    imu.ay,
                    imu.az,
                    durum.setpoint_hiz,
                    durum.setpoint_yaw,
                    durum.arac_mod as u8,
                    durum.motorlar_veri.0,
                    durum.motorlar_veri.1,
                    durum.motorlar_veri.2,
                    durum.motorlar_veri.3,
                    durum.motorlar_istek.0,
                    durum.motorlar_istek.1,
                    durum.motorlar_istek.2,
                    durum.motorlar_istek.3,
                    dr.aktif as u8,
                    dr.enlem,
                    dr.boylam,
                    dr.mutlak_yaw_deg,
                    dr.goreli_yaw_deg,
                    dr.referans_yaw_deg,
                    dr.ileri_hiz_m_s,
                    dr.yatay_hiz_m_s,
                    dr.toplam_mesafe_m,
                    dr.gps_fark_m,
                );
                tum_dosya.write_all(satir.as_bytes()).await?;
            }
            _ = flush_tick.tick() => {
                imu_dosya.flush().await?;
                gps_dosya.flush().await?;
                tum_dosya.flush().await?;
            }
            _ = sync_tick.tick() => {
                // Ani güç kaybında veri kaybını azaltmak için belirli aralıklarla
                // kullanıcı alanı tamponlarını ve işletim sistemi önbelleğini SD'ye yaz.
                imu_dosya.flush().await?;
                gps_dosya.flush().await?;
                tum_dosya.flush().await?;
                imu_dosya.get_ref().sync_data().await?;
                gps_dosya.get_ref().sync_data().await?;
                tum_dosya.get_ref().sync_data().await?;
            }
            sonuc = kapat_rx.changed() => {
                let kapat = sonuc.is_err() || *kapat_rx.borrow_and_update();
                if kapat {
                    imu_dosya.flush().await?;
                    gps_dosya.flush().await?;
                    tum_dosya.flush().await?;
                    imu_dosya.get_ref().sync_all().await?;
                    gps_dosya.get_ref().sync_all().await?;
                    tum_dosya.get_ref().sync_all().await?;
                    println!("SD kayıt oturumu kapatıldı: {}", oturum.display());
                    return Ok(());
                }
            }
        }
    }
}
