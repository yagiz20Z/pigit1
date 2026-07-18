use std::env;

use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;
use tokio::time::{Duration, MissedTickBehavior, interval, sleep};

use crate::motorlar::MotorKontrol;
use crate::sensorler;
use crate::telemetri;
use crate::veri_tipleri::{GelenTelemetri, GidenTelemetri, GpsVeri, ImuVeri, MotorVeri};

// Donanım portları bu modülde tutulur. İstenirse ortam değişkenleriyle
// değiştirilir: IDA_TEL_PORT, IDA_MOTOR_PORT, IDA_GPS_PORT, IDA_IMU_PORT.
const DEFAULT_TEL_PORT: &str = "/dev/ttyUSB0";
const DEFAULT_MOTOR_PORT: &str = "/dev/ttyUSB1";
// Son doğrulanan eşleşme: GPS=ACM1, IMU=ACM0. Kalıcı kullanımda
// /dev/serial/by-id yollarını ortam değişkenleriyle vermek daha güvenlidir.
const DEFAULT_GPS_PORT: &str = "/dev/ttyACM1";
const DEFAULT_IMU_PORT: &str = "/dev/ttyACM0";

const TEL_BAUD_RATE: u32 = 57_600;
const MOTOR_BAUD_RATE: u32 = 115_200;
const GPS_BAUD_RATE: u32 = 115_200;
const IMU_BAUD_RATE: u32 = 115_200;

const TEL_CHANNEL_BUF: usize = 100;
const MOTOR_GONDERIM_ARALIGI: Duration = Duration::from_millis(50);

fn port_oku(env_adi: &str, varsayilan: &str) -> String {
    env::var(env_adi).unwrap_or_else(|_| varsayilan.to_string())
}

/// Beynin yalnızca ihtiyaç duyduğu veri kanalları.
/// Beyin port, baudrate veya UART ayrıntılarını bilmez.
pub struct BeyinKanallari {
    pub imu_rx: watch::Receiver<ImuVeri>,
    pub gps_rx: watch::Receiver<GpsVeri>,
    /// Motor komutunda kuyruk tutulmaz; yalnızca en güncel değer saklanır.
    pub motor_tx: watch::Sender<MotorVeri>,
    pub yki_komut_rx: mpsc::Receiver<GelenTelemetri>,
    pub yki_telemetri_tx: mpsc::Sender<GidenTelemetri>,
}

/// main.rs'nin izlediği haberleşme görevleri.
pub struct HaberlesmeGorevleri {
    pub gps: JoinHandle<()>,
    pub imu: JoinHandle<()>,
    pub motor: JoinHandle<()>,
    pub telemetri: JoinHandle<()>,
}

/// GPS, IMU, YKİ telemetrisi ve STM motor UART katmanını başlatır.
/// Karar üretmez; yalnızca veriyi taşır ve bağlantıları yeniden kurar.
pub fn baslat() -> (BeyinKanallari, HaberlesmeGorevleri) {
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

    let (tel_to_beyin_tx, tel_to_beyin_rx) = mpsc::channel::<GelenTelemetri>(TEL_CHANNEL_BUF);
    let (beyin_to_tel_tx, beyin_to_tel_rx) = mpsc::channel::<GidenTelemetri>(TEL_CHANNEL_BUF);
    let (imu_tx, imu_rx) = watch::channel(ImuVeri::default());
    let (gps_tx, gps_rx) = watch::channel(GpsVeri::default());
    let (motor_tx, mut motor_rx) = watch::channel(MotorVeri::default());

    let gps = tokio::spawn(async move {
        sensorler::m8n::gps_task(gps_port, GPS_BAUD_RATE, gps_tx).await;
    });

    let imu = tokio::spawn(async move {
        sensorler::bno085::imu_task(imu_port, IMU_BAUD_RATE, imu_tx).await;
    });

    let motor = tokio::spawn(async move {
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

            // Yeni/açılmış bağlantıda eski bir hareket komutunu doğrudan uygulama.
            // Önce mutlaka sıfır gönder, ardından güncel watch değerine geç.
            if let Err(e) = motor_kontrol.sifirla().await {
                eprintln!("STM ilk güvenli sıfırlama başarısız: {e}");
                sleep(Duration::from_millis(500)).await;
                continue;
            }

            let mut gonderim_tick = interval(MOTOR_GONDERIM_ARALIGI);
            gonderim_tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
            // interval'in ilk anlık tick'ini tüket; sıfır paketinden sonra 50 ms bırak.
            gonderim_tick.tick().await;

            'baglanti: loop {
                tokio::select! {
                    degisti = motor_rx.changed() => {
                        if degisti.is_err() {
                            let _ = motor_kontrol.sifirla().await;
                            println!("Motor komut kanalı kapandı.");
                            return;
                        }
                    }
                    _ = gonderim_tick.tick() => {}
                }

                // Ref değerini await öncesinde klonla; yalnızca en güncel komut gönderilir.
                let motor_komutu = motor_rx.borrow().clone();
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
                    break 'baglanti;
                }
            }

            sleep(Duration::from_millis(500)).await;
        }
    });

    let telemetri = tokio::spawn(async move {
        if let Err(e) =
            telemetri::telemetri_task(tel_port, TEL_BAUD_RATE, tel_to_beyin_tx, beyin_to_tel_rx)
                .await
        {
            eprintln!("Telemetri görevi sonlandı: {e}");
        }
    });

    (
        BeyinKanallari {
            imu_rx,
            gps_rx,
            motor_tx,
            yki_komut_rx: tel_to_beyin_rx,
            yki_telemetri_tx: beyin_to_tel_tx,
        },
        HaberlesmeGorevleri {
            gps,
            imu,
            motor,
            telemetri,
        },
    )
}
