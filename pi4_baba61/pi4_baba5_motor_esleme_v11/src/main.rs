mod sensorler;
mod motorlar;
mod veri_tipleri;
mod beyin;
mod telemetri;

use std::env;
use tokio::sync::{mpsc, watch};
use tokio::time::{sleep, Duration};

use crate::motorlar::MotorKontrol;
use crate::veri_tipleri::{
    GelenTelemetri, GidenTelemetri, GpsVeri, ImuVeri, MotorVeri,
};

// Raspberry Pi üzerindeki mevcut USB eşleşmen:
// Telemetri : /dev/ttyUSB0
// STM motor : /dev/ttyUSB1
// GPS       : /dev/ttyACM0
// IMU       : /dev/ttyACM1
const DEFAULT_TEL_PORT: &str = "/dev/ttyUSB0";
const DEFAULT_MOTOR_PORT: &str = "/dev/ttyUSB1";
const DEFAULT_GPS_PORT: &str = "/dev/ttyACM0";
const DEFAULT_IMU_PORT: &str = "/dev/ttyACM1";

const TEL_BAUD_RATE: u32 = 57_600;
const MOTOR_BAUD_RATE: u32 = 115_200;
const GPS_BAUD_RATE: u32 = 115_200;
const IMU_BAUD_RATE: u32 = 115_200;

fn port_oku(env_adi: &str, varsayilan: &str) -> String {
    env::var(env_adi).unwrap_or_else(|_| varsayilan.to_string())
}

#[tokio::main]
async fn main() {
    const TEL_CHANNEL_BUF: usize = 100;
    const MOTOR_CHANNEL_BUF: usize = 100;

    let tel_port = port_oku("IDA_TEL_PORT", DEFAULT_TEL_PORT);
    let motor_port = port_oku("IDA_MOTOR_PORT", DEFAULT_MOTOR_PORT);
    let gps_port = port_oku("IDA_GPS_PORT", DEFAULT_GPS_PORT);
    let imu_port = port_oku("IDA_IMU_PORT", DEFAULT_IMU_PORT);

    println!("================ IDA PORT EŞLEŞMESİ ================");
    println!("Telemetri : {} @ {} baud", tel_port, TEL_BAUD_RATE);
    println!("STM motor : {} @ {} baud", motor_port, MOTOR_BAUD_RATE);
    println!("GPS       : {} @ {} baud", gps_port, GPS_BAUD_RATE);
    println!("IMU       : {} @ {} baud", imu_port, IMU_BAUD_RATE);
    println!("=====================================================");

    let (tel_to_beyin_tx, tel_to_beyin_rx) =
        mpsc::channel::<GelenTelemetri>(TEL_CHANNEL_BUF);
    let (beyin_to_tel_tx, beyin_to_tel_rx) =
        mpsc::channel::<GidenTelemetri>(TEL_CHANNEL_BUF);
    let (imu_tx, imu_rx) = watch::channel(ImuVeri::default());
    let (gps_tx, gps_rx) = watch::channel(GpsVeri::default());
    let (motor_tx, mut motor_rx) = mpsc::channel::<MotorVeri>(MOTOR_CHANNEL_BUF);

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
        sonuc = nav_handle => {
            eprintln!("nav_task sonlandı: {:?}", sonuc);
        }
        sonuc = motor_handle => {
            eprintln!("Motor görevi sonlandı: {:?}", sonuc);
        }
        sonuc = gps_handle => {
            eprintln!("GPS görevi sonlandı: {:?}", sonuc);
        }
        sonuc = imu_handle => {
            eprintln!("IMU görevi sonlandı: {:?}", sonuc);
        }
        sonuc = telemetri_handle => {
            eprintln!("Telemetri görevi sonlandı: {:?}", sonuc);
        }
        sonuc = tokio::signal::ctrl_c() => {
            println!("Ctrl+C alındı: {:?}", sonuc);
        }
    }

    println!("Sistem kapanıyor.");
}
