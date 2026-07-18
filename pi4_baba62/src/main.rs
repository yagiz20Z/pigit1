mod sensorler;
mod motorlar;
mod veri_tipleri;
mod beyin;
mod telemetri;
mod haberlesme;

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

    let haberlesme::HaberlesmeGorevleri {
        gps: gps_handle,
        imu: imu_handle,
        motor: motor_handle,
        telemetri: telemetri_handle,
    } = gorevler;

    let nav_handle = tokio::spawn(async move {
        beyin::nav_task(
            imu_rx,
            gps_rx,
            motor_tx,
            yki_komut_rx,
            yki_telemetri_tx,
        )
        .await;
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

    println!("Sistem kapanıyor.");
}
