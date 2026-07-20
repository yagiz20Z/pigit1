mod beyin;
mod dead_reckoning;
mod haberlesme;
mod motorlar;
mod sd_kayit;
mod sensorler;
mod telemetri;
mod veri_tipleri;

use crate::veri_tipleri::{GidenTelemetri, MotorVeri};
use tokio::sync::watch;
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

    // IMU ve GPS watch kanalları klonlanabilir. Böylece karar verme döngüsünü
    // bozmadan aynı veriler SD kayıt görevine de aktarılır.
    let imu_kayit_rx = imu_rx.clone();
    let gps_kayit_rx = gps_rx.clone();
    let (durum_kayit_tx, durum_kayit_rx) = watch::channel(GidenTelemetri::default());
    let (kayit_kapat_tx, kayit_kapat_rx) = watch::channel(false);

    // nav_task motor_tx'i alacak; kapanışta sıfır gönderebilmek için bir kopya sakla.
    let kapanis_motor_tx = motor_tx.clone();

    let haberlesme::HaberlesmeGorevleri {
        gps: gps_handle,
        imu: imu_handle,
        motor: motor_handle,
        telemetri: telemetri_handle,
    } = gorevler;

    let mut kayit_handle = tokio::spawn(sd_kayit::sd_kayit_task(
        imu_kayit_rx,
        gps_kayit_rx,
        durum_kayit_rx,
        kayit_kapat_rx,
    ));

    let nav_handle = tokio::spawn(async move {
        beyin::nav_task(
            imu_rx,
            gps_rx,
            motor_tx,
            yki_komut_rx,
            yki_telemetri_tx,
            durum_kayit_tx,
        )
        .await;
    });

    let mut kayit_sonlandi = false;

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
        sonuc = &mut kayit_handle => {
            kayit_sonlandi = true;
            eprintln!("SD kayıt görevi sonlandı: {:?}", sonuc);
        }
        sonuc = tokio::signal::ctrl_c() => {
            println!("Ctrl+C alındı: {:?}", sonuc);
        }
    }

    // Runtime kapanmadan motor görevine sıfır komutu gönder ve CSV tamponlarını
    // fiziksel depolamaya yazdırarak kayıt görevini temiz biçimde kapat.
    println!("Sistem kapanıyor: motorlar sıfırlanıyor ve SD kayıtları tamamlanıyor.");
    let _ = kapanis_motor_tx.send(MotorVeri::default());
    let _ = kayit_kapat_tx.send(true);

    if !kayit_sonlandi {
        match tokio::time::timeout(Duration::from_secs(3), &mut kayit_handle).await {
            Ok(Ok(Ok(()))) => println!("SD kayıtları güvenli biçimde kapatıldı."),
            Ok(Ok(Err(e))) => eprintln!("SD kayıt kapatma hatası: {e}"),
            Ok(Err(e)) => eprintln!("SD kayıt görevi join hatası: {e}"),
            Err(_) => eprintln!("SD kayıt görevi 3 saniyede kapanmadı."),
        }
    }

    sleep(Duration::from_millis(250)).await;
    println!("Sistem kapandı.");
}
