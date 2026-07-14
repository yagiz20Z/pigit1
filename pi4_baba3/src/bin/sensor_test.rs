use std::{env, path::Path};

use pi4_baba::sensorler::{bno085, m8n};
use pi4_baba::veri_tipleri::{GpsVeri, ImuVeri};
use tokio::sync::watch;
use tokio::time::{Duration, Instant, MissedTickBehavior};

const IMU_BY_ID: &str = "/dev/serial/by-id/usb-1029_0001-if00";
const GPS_BY_ID: &str = "/dev/serial/by-id/usb-1029_0002-if00";
const IMU_FALLBACK: &str = "/dev/ttyACM0";
const GPS_FALLBACK: &str = "/dev/ttyACM1";
const BAUD_RATE: u32 = 115_200;

fn port_sec(env_adi: &str, arg: Option<&String>, by_id: &str, fallback: &str) -> String {
    if let Some(port) = arg {
        return port.clone();
    }

    if let Ok(port) = env::var(env_adi) {
        return port;
    }

    if Path::new(by_id).exists() {
        by_id.to_string()
    } else {
        fallback.to_string()
    }
}

fn fix_adi(fix: u8) -> &'static str {
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

#[tokio::main]
async fn main() {
    let args: Vec<String> = env::args().collect();
    let imu_port = port_sec("IDA_IMU_PORT", args.get(1), IMU_BY_ID, IMU_FALLBACK);
    let gps_port = port_sec("IDA_GPS_PORT", args.get(2), GPS_BY_ID, GPS_FALLBACK);

    println!("Pico sensör testi başladı.");
    println!("BNO085 portu: {imu_port}");
    println!("M8N GPS portu: {gps_port}");
    println!("Not: Ana pi4_baba programı aynı anda çalışmamalı; seri portları tek program açmalıdır.\n");

    let (imu_tx, mut imu_rx) = watch::channel(ImuVeri::default());
    let (gps_tx, mut gps_rx) = watch::channel(GpsVeri::default());

    tokio::spawn(async move {
        bno085::imu_task(imu_port, BAUD_RATE, imu_tx).await;
    });

    tokio::spawn(async move {
        m8n::gps_task(gps_port, BAUD_RATE, gps_tx).await;
    });

    let mut imu_sayac = 0u64;
    let mut gps_sayac = 0u64;
    let mut son_imu: Option<Instant> = None;
    let mut son_gps: Option<Instant> = None;
    let mut rapor = tokio::time::interval(Duration::from_secs(1));
    rapor.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
        tokio::select! {
            sonuc = imu_rx.changed() => {
                if sonuc.is_err() {
                    eprintln!("IMU kanalı kapandı.");
                    break;
                }
                imu_sayac = imu_sayac.wrapping_add(1);
                son_imu = Some(Instant::now());
            }
            sonuc = gps_rx.changed() => {
                if sonuc.is_err() {
                    eprintln!("GPS kanalı kapandı.");
                    break;
                }
                gps_sayac = gps_sayac.wrapping_add(1);
                son_gps = Some(Instant::now());
            }
            _ = rapor.tick() => {
                let imu = *imu_rx.borrow();
                let gps = *gps_rx.borrow();

                if let Some(zaman) = son_imu {
                    let yas = zaman.elapsed();
                    let durum = if yas <= Duration::from_secs(2) { "OK" } else { "ESKI" };
                    println!(
                        "[IMU {durum}] paket={} yas={}ms roll={:.2} pitch={:.2} yaw={:.2} gyro=({:.2},{:.2},{:.2}) accel=({:.2},{:.2},{:.2})",
                        imu_sayac, yas.as_millis(), imu.roll, imu.pitch, imu.yaw,
                        imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az
                    );
                } else {
                    println!("[IMU VERI YOK] USB portu açılmış olsa bile geçerli 47 baytlık BNO085 paketi alınmadı.");
                }

                if let Some(zaman) = son_gps {
                    let yas = zaman.elapsed();
                    let durum = if yas <= Duration::from_secs(3) { "OK" } else { "ESKI" };
                    println!(
                        "[GPS {durum}] paket={} yas={}ms fix={}({}) uydu={} lat={:.7} lon={:.7} hiz={:.3}m/s yon={:.2}deg",
                        gps_sayac,
                        yas.as_millis(),
                        fix_adi(gps.algi_boyut),
                        gps.algi_boyut,
                        gps.uydu_sayi,
                        gps.enlem as f64 / 10_000_000.0,
                        gps.boylam as f64 / 10_000_000.0,
                        gps.hiz as f64 / 1_000.0,
                        gps.yonelim as f64 / 100_000.0,
                    );
                } else {
                    println!("[GPS VERI YOK] USB portu açılmış olsa bile geçerli 33 baytlık M8N paketi alınmadı.");
                }
            }
            sonuc = tokio::signal::ctrl_c() => {
                println!("Ctrl+C alındı: {:?}", sonuc);
                break;
            }
        }
    }
}
