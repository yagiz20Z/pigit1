use std::mem;
use std::time::{SystemTime, UNIX_EPOCH};

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc;
use tokio::time::{Duration, sleep, timeout};
use tokio_serial::SerialPortBuilderExt;

use crate::veri_tipleri::{LidarNokta, LidarVeri};

const BAGLANTI_YENIDEN_DENEME: Duration = Duration::from_secs(1);
const KOMUT_TIMEOUT: Duration = Duration::from_secs(2);
const BAYT_TIMEOUT: Duration = Duration::from_secs(2);

/// RPLidar görevi henüz ana haberleşme modülünde başlatılmıyor. Sensör kodu
/// gönderildiğinde port/baud bilgisiyle haberlesme.rs içine bağlanabilir.
pub async fn lidar_task(port_adi: String, baud_rate: u32, tx: mpsc::Sender<LidarVeri>) {
    loop {
        println!("Lidar portu açılmaya çalışılıyor: {}", port_adi);
        let mut port = match tokio_serial::new(&port_adi, baud_rate).open_native_async() {
            Ok(port) => {
                println!("Lidar bağlantısı kuruldu: {}", port_adi);
                port
            }
            Err(e) => {
                eprintln!("Lidar portu açılamadı: {e}");
                sleep(BAGLANTI_YENIDEN_DENEME).await;
                continue;
            }
        };

        let durdur = [0xA5, 0x25];
        let _ = timeout(KOMUT_TIMEOUT, port.write_all(&durdur)).await;
        sleep(Duration::from_millis(50)).await;

        let baslat = [0xA5, 0x20];
        match timeout(KOMUT_TIMEOUT, port.write_all(&baslat)).await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                eprintln!("Lidar başlatma yazma hatası: {e}");
                sleep(BAGLANTI_YENIDEN_DENEME).await;
                continue;
            }
            Err(_) => {
                eprintln!("Lidar başlatma komutu zaman aşımı.");
                sleep(BAGLANTI_YENIDEN_DENEME).await;
                continue;
            }
        }

        let mut desc = [0u8; 7];
        match timeout(KOMUT_TIMEOUT, port.read_exact(&mut desc)).await {
            Ok(Ok(_)) => {}
            Ok(Err(e)) => {
                eprintln!("Lidar descriptor okuma hatası: {e}");
                sleep(BAGLANTI_YENIDEN_DENEME).await;
                continue;
            }
            Err(_) => {
                eprintln!("Lidar descriptor zaman aşımı.");
                sleep(BAGLANTI_YENIDEN_DENEME).await;
                continue;
            }
        }

        let mut buf = [0u8; 5];
        let mut bufid = 0usize;
        let mut anlik_tur: Vec<LidarNokta> = Vec::with_capacity(1500);

        'baglanti: loop {
            match timeout(BAYT_TIMEOUT, port.read_exact(&mut buf[bufid..bufid + 1])).await {
                Ok(Ok(_)) => {
                    bufid += 1;
                }
                Ok(Err(e)) => {
                    eprintln!("Lidar USB okuma hatası: {e}");
                    break 'baglanti;
                }
                Err(_) => {
                    eprintln!("Lidar veri zaman aşımı; port yeniden açılacak.");
                    break 'baglanti;
                }
            }

            if bufid != 5 {
                continue;
            }

            let check1 = buf[0] & 0x01;
            let check2 = (buf[0] >> 1) & 0x01;
            let check3 = buf[1] & 0x01;

            if check1 != check2 && check3 == 1 {
                let start_flag = check1 == 1;
                if start_flag && !anlik_tur.is_empty() {
                    let simdiki_zaman = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_millis() as u64;
                    let noktalar = mem::replace(&mut anlik_tur, Vec::with_capacity(1500));
                    let total_paket = LidarVeri {
                        noktalar,
                        zaman_ms: simdiki_zaman,
                    };

                    if tx.send(total_paket).await.is_err() {
                        println!("Lidar alıcısı kapandı.");
                        return;
                    }
                }

                let kalite = buf[0] >> 2;
                let aci_raw = ((buf[2] as u16) << 7) | ((buf[1] as u16) >> 1);
                let aci_derece = aci_raw as f32 / 64.0;
                let mesafe_raw = ((buf[4] as u16) << 8) | buf[3] as u16;
                let mesafe_mm = mesafe_raw as f32 / 4.0;

                if mesafe_mm > 0.0 && aci_derece.is_finite() && mesafe_mm.is_finite() {
                    anlik_tur.push(LidarNokta {
                        aci: aci_derece,
                        mesafe_mm,
                        kalite,
                    });
                }
                bufid = 0;
            } else {
                buf.copy_within(1..5, 0);
                bufid = 4;
            }
        }

        sleep(BAGLANTI_YENIDEN_DENEME).await;
    }
}
