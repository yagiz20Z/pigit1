mod beyin;
mod dead_reckoning;
mod haberlesme;
mod motorlar;
mod sensorler;
mod telemetri;
mod veri_tipleri;

use crate::veri_tipleri::MotorVeri;
use tokio::time::{Duration, sleep};

#[tokio::main]
async fn main() {
    // Haberleşme bütün portları ve donanım görevlerini kendi içinde başlatır.
    // main.rs yalnızca haberleşme ile ana karar verme modülünü birbirine bağlar.
    let (kanallar, gorevler) = haberlesme::baslat();

    let haberlesme::BeyinKanallari {
        imu_rx,
        gps_rx,
        motor_tx,
        yki_komut_rx,
        yki_telemetri_tx,
    } = kanallar;

    // nav_task motor_tx'i alacak; kapanışta sıfır gönderebilmek için bir kopya sakla.
    let kapanis_motor_tx = motor_tx.clone();

    let haberlesme::HaberlesmeGorevleri {
        gps: gps_handle,
        imu: imu_handle,
        motor: motor_handle,
        telemetri: telemetri_handle,
    } = gorevler;

    let nav_handle = tokio::spawn(async move {
        beyin::nav_task(imu_rx, gps_rx, motor_tx, yki_komut_rx, yki_telemetri_tx).await;
    });

    tokio::select! {
        sonuc = nav_handle => {
            eprintln!("nav_task sonlandı: {:?}", sonuc);
        }
        sonuc = motor_handle => {
            eprintln!("Motor haberleşme görevi sonlandı: {:?}", sonuc);
        }
        sonuc = gps_handle => {
            eprintln!("GPS haberleşme görevi sonlandı: {:?}", sonuc);
        }
        sonuc = imu_handle => {
            eprintln!("IMU haberleşme görevi sonlandı: {:?}", sonuc);
        }
        sonuc = telemetri_handle => {
            eprintln!("Telemetri haberleşme görevi sonlandı: {:?}", sonuc);
        }
        sonuc = tokio::signal::ctrl_c() => {
            println!("Ctrl+C alındı: {:?}", sonuc);
        }
    }

    // Runtime kapanmadan motor görevine birkaç kez sıfır komutu gönderecek süre bırak.
    println!("Sistem kapanıyor: motorlar sıfırlanıyor.");
    let _ = kapanis_motor_tx.send(MotorVeri::default());
    sleep(Duration::from_millis(250)).await;
    println!("Sistem kapandı.");
}
