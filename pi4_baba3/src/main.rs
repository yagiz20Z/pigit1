mod beyin;
mod motorlar;
mod sensorler;
mod telemetri;
mod veri_tipleri;

use std::{env, path::Path};

use tokio::sync::{mpsc, watch};
use tokio::time::{sleep, Duration, Instant, MissedTickBehavior};

use crate::motorlar::MotorKontrol;
use crate::veri_tipleri::{
    GelenTelemetri, GidenTelemetri, GpsVeri, ImuVeri, MotorVeri,
};

// USB-Pico kimlikleri:
// BNO085 Pico -> VID:PID 1029:0001
// M8N GPS Pico -> VID:PID 1029:0002
const DEFAULT_IMU_BY_ID: &str = "/dev/serial/by-id/usb-1029_0001-if00";
const DEFAULT_GPS_BY_ID: &str = "/dev/serial/by-id/usb-1029_0002-if00";

// by-id oluşmazsa kullanılacak geçici yollar. Asıl tercih her zaman by-id'dir.
const DEFAULT_IMU_FALLBACK: &str = "/dev/ttyACM0";
const DEFAULT_GPS_FALLBACK: &str = "/dev/ttyACM1";

const DEFAULT_TEL_PORT: &str = "/dev/ttyUSB0";
const DEFAULT_MOTOR_PORT: &str = "/dev/ttyUSB1";

const TEL_BAUD_RATE: u32 = 57_600;
const MOTOR_BAUD_RATE: u32 = 115_200;
const GPS_BAUD_RATE: u32 = 115_200;
const IMU_BAUD_RATE: u32 = 115_200;

fn port_oku(env_adi: &str, by_id: &str, fallback: &str) -> String {
    if let Ok(port) = env::var(env_adi) {
        return port;
    }

    if Path::new(by_id).exists() {
        by_id.to_string()
    } else {
        fallback.to_string()
    }
}

fn sabit_port_oku(env_adi: &str, varsayilan: &str) -> String {
    env::var(env_adi).unwrap_or_else(|_| varsayilan.to_string())
}

fn gps_fix_adi(fix: u8) -> &'static str {
    match fix {
        0 => "FIX YOK",
        1 => "DR",
        2 => "2D",
        3 => "3D",
        4 => "GNSS+DR",
        5 => "TIME",
        _ => "BILINMIYOR",
    }
}

async fn sensor_monitor_task(
    mut imu_rx: watch::Receiver<ImuVeri>,
    mut gps_rx: watch::Receiver<GpsVeri>,
) {
    let mut imu_paket = 0u64;
    let mut gps_paket = 0u64;
    let mut son_imu: Option<Instant> = None;
    let mut son_gps: Option<Instant> = None;

    let mut rapor_zamani = tokio::time::interval(Duration::from_secs(1));
    rapor_zamani.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
        tokio::select! {
            sonuc = imu_rx.changed() => {
                if sonuc.is_err() {
                    eprintln!("[IMU] Veri kanalı kapandı.");
                    return;
                }
                imu_paket = imu_paket.wrapping_add(1);
                son_imu = Some(Instant::now());
            }
            sonuc = gps_rx.changed() => {
                if sonuc.is_err() {
                    eprintln!("[GPS] Veri kanalı kapandı.");
                    return;
                }
                gps_paket = gps_paket.wrapping_add(1);
                son_gps = Some(Instant::now());
            }
            _ = rapor_zamani.tick() => {
                let imu = *imu_rx.borrow();
                let gps = *gps_rx.borrow();

                match son_imu {
                    Some(zaman) => {
                        let yas = zaman.elapsed();
                        let durum = if yas <= Duration::from_secs(2) { "OK" } else { "ESKI" };
                        println!(
                            "[IMU {durum}] paket={} yas={} ms | roll={:.2} pitch={:.2} yaw={:.2} | gyro=({:.2},{:.2},{:.2}) | ivme=({:.2},{:.2},{:.2})",
                            imu_paket,
                            yas.as_millis(),
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
                    }
                    None => {
                        println!("[IMU VERI YOK] Pico portu açılmış olsa bile henüz geçerli BNO085 paketi gelmedi.");
                    }
                }

                match son_gps {
                    Some(zaman) => {
                        let yas = zaman.elapsed();
                        let durum = if yas <= Duration::from_secs(3) { "OK" } else { "ESKI" };
                        let enlem = gps.enlem as f64 / 10_000_000.0;
                        let boylam = gps.boylam as f64 / 10_000_000.0;
                        let hiz_ms = gps.hiz as f64 / 1_000.0;
                        let yon_deg = gps.yonelim as f64 / 100_000.0;

                        println!(
                            "[GPS {durum}] paket={} yas={} ms | fix={}({}) uydu={} | lat={:.7} lon={:.7} | hiz={:.3} m/s yon={:.2} deg yukseklik={} mm",
                            gps_paket,
                            yas.as_millis(),
                            gps_fix_adi(gps.algi_boyut),
                            gps.algi_boyut,
                            gps.uydu_sayi,
                            enlem,
                            boylam,
                            hiz_ms,
                            yon_deg,
                            gps.yukseklik_mm,
                        );
                    }
                    None => {
                        println!("[GPS VERI YOK] Pico portu açılmış olsa bile henüz geçerli M8N paketi gelmedi.");
                    }
                }
            }
        }
    }
}

#[tokio::main]
async fn main() {
    const TEL_CHANNEL_BUF: usize = 100;
    const MOTOR_CHANNEL_BUF: usize = 100;

    let tel_port = sabit_port_oku("IDA_TEL_PORT", DEFAULT_TEL_PORT);
    let motor_port = sabit_port_oku("IDA_MOTOR_PORT", DEFAULT_MOTOR_PORT);
    let imu_port = port_oku("IDA_IMU_PORT", DEFAULT_IMU_BY_ID, DEFAULT_IMU_FALLBACK);
    let gps_port = port_oku("IDA_GPS_PORT", DEFAULT_GPS_BY_ID, DEFAULT_GPS_FALLBACK);

    println!("================ IDA PORT ESLESMESI ================");
    println!("Telemetri : {} @ {} baud", tel_port, TEL_BAUD_RATE);
    println!("STM motor : {} @ {} baud", motor_port, MOTOR_BAUD_RATE);
    println!("BNO085    : {} @ {} baud", imu_port, IMU_BAUD_RATE);
    println!("M8N GPS   : {} @ {} baud", gps_port, GPS_BAUD_RATE);
    println!("=====================================================");

    if !Path::new(DEFAULT_IMU_BY_ID).exists() {
        eprintln!("UYARI: BNO085 by-id yolu bulunamadı: {DEFAULT_IMU_BY_ID}");
    }
    if !Path::new(DEFAULT_GPS_BY_ID).exists() {
        eprintln!("UYARI: GPS by-id yolu bulunamadı: {DEFAULT_GPS_BY_ID}");
        eprintln!("GPS Pico'ya PID=0002 olan yeni firmware yüklenmiş olmalı.");
    }

    let (tel_to_beyin_tx, tel_to_beyin_rx) =
        mpsc::channel::<GelenTelemetri>(TEL_CHANNEL_BUF);
    let (beyin_to_tel_tx, beyin_to_tel_rx) =
        mpsc::channel::<GidenTelemetri>(TEL_CHANNEL_BUF);
    let (imu_tx, imu_rx) = watch::channel(ImuVeri::default());
    let (gps_tx, gps_rx) = watch::channel(GpsVeri::default());
    let (motor_tx, mut motor_rx) = mpsc::channel::<MotorVeri>(MOTOR_CHANNEL_BUF);

    let monitor_imu_rx = imu_rx.clone();
    let monitor_gps_rx = gps_rx.clone();

    let sensor_monitor_handle = tokio::spawn(async move {
        sensor_monitor_task(monitor_imu_rx, monitor_gps_rx).await;
    });

    let gps_handle = tokio::spawn(async move {
        sensorler::m8n::gps_task(gps_port, GPS_BAUD_RATE, gps_tx).await;
    });

    let imu_handle = tokio::spawn(async move {
        sensorler::bno085::imu_task(imu_port, IMU_BAUD_RATE, imu_tx).await;
    });

    let motor_handle = tokio::spawn(async move {
        loop {
            println!("STM UART açılmaya çalışılıyor: {}", motor_port);

            let mut motor_kontrol = match MotorKontrol::new_port(&motor_port, MOTOR_BAUD_RATE) {
                Ok(mk) => {
                    println!("STM UART bağlantısı kuruldu: {}", motor_port);
                    mk
                }
                Err(e) => {
                    eprintln!("STM UART açılamadı: {e}. 1 saniye sonra tekrar denenecek.");
                    sleep(Duration::from_secs(1)).await;
                    continue;
                }
            };

            while let Some(motor_komutu) = motor_rx.recv().await {
                let sonuc = motor_kontrol
                    .set_speeds(
                        motor_komutu.iskeleon,
                        motor_komutu.iskelearka,
                        motor_komutu.sancakon,
                        motor_komutu.sancakarka,
                    )
                    .await;

                if let Err(e) = sonuc {
                    eprintln!("STM UART yazma/okuma hatası: {e}");
                    eprintln!("Motor portu yeniden açılacak.");
                    break;
                }
            }

            if motor_rx.is_closed() {
                println!("Motor komut kanalı kapandı.");
                return;
            }

            sleep(Duration::from_millis(500)).await;
        }
    });

    let telemetri_handle = tokio::spawn(async move {
        if let Err(e) = telemetri::telemetri_task(
            tel_port,
            TEL_BAUD_RATE,
            tel_to_beyin_tx,
            beyin_to_tel_rx,
        )
        .await
        {
            eprintln!("Telemetri görevi sonlandı: {e}");
        }
    });

    let nav_handle = tokio::spawn(async move {
        beyin::nav_task(
            imu_rx,
            gps_rx,
            motor_tx,
            tel_to_beyin_rx,
            beyin_to_tel_tx,
        )
        .await;
    });

    tokio::select! {
        sonuc = nav_handle => eprintln!("nav_task sonlandı: {:?}", sonuc),
        sonuc = motor_handle => eprintln!("Motor görevi sonlandı: {:?}", sonuc),
        sonuc = gps_handle => eprintln!("GPS görevi sonlandı: {:?}", sonuc),
        sonuc = imu_handle => eprintln!("IMU görevi sonlandı: {:?}", sonuc),
        sonuc = sensor_monitor_handle => eprintln!("Sensör izleme görevi sonlandı: {:?}", sonuc),
        sonuc = telemetri_handle => eprintln!("Telemetri görevi sonlandı: {:?}", sonuc),
        sonuc = tokio::signal::ctrl_c() => println!("Ctrl+C alındı: {:?}", sonuc),
    }

    println!("Sistem kapanıyor.");
}
