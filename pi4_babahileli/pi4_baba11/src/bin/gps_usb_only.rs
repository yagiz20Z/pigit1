#[path = "../sensorler/m8n.rs"]
mod m8n;
#[path = "../veri_tipleri.rs"]
mod veri_tipleri;

use std::env;

use tokio::sync::watch;

use veri_tipleri::GpsVeri;

#[tokio::main]
async fn main() {
    let args: Vec<String> = env::args().collect();
    let port = args
        .get(1)
        .cloned()
        .or_else(|| env::var("IDA_GPS_PORT").ok())
        .unwrap_or_else(|| "/dev/ttyACM1".to_string());
    let baud = args
        .get(2)
        .and_then(|deger| deger.parse::<u32>().ok())
        .or_else(|| {
            env::var("IDA_GPS_BAUD")
                .ok()
                .and_then(|deger| deger.parse::<u32>().ok())
        })
        .unwrap_or(115_200);

    println!("GPS USB tekli test: {} @ ilk {} baud", port, baud);
    println!("Durdurmak için Ctrl+C");

    let (tx, mut rx) = watch::channel(GpsVeri::default());
    let gps_handle = tokio::spawn(m8n::gps_task(port, baud, tx));

    loop {
        tokio::select! {
            degisti = rx.changed() => {
                if degisti.is_err() {
                    eprintln!("GPS veri kanalı kapandı.");
                    break;
                }

                let gps = *rx.borrow_and_update();
                println!(
                    "GPS_ISLENMIS fix={} uydu={} lat={:.7} lon={:.7} alt={:.3}m hiz={:.3}m/s yon={:.2}deg zaman_ms={}",
                    gps.algi_boyut,
                    gps.uydu_sayi,
                    gps.enlem as f64 / 10_000_000.0,
                    gps.boylam as f64 / 10_000_000.0,
                    gps.yukseklik_mm as f64 / 1000.0,
                    gps.hiz as f64 / 1000.0,
                    gps.yonelim as f64 / 100_000.0,
                    gps.zaman_ms,
                );
            }
            sonuc = tokio::signal::ctrl_c() => {
                println!("Ctrl+C alındı: {:?}", sonuc);
                break;
            }
        }
    }

    gps_handle.abort();
}
