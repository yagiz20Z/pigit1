use tokio::io::AsyncReadExt;
use tokio::sync::watch;
use tokio::time::{sleep, Duration};
use tokio_serial::SerialPortBuilderExt;

use crate::sensorler::bno085::DurumParse::{HeaderF, HeaderS};
use crate::veri_tipleri::ImuVeri;

enum DurumParse {
    HeaderF,
    HeaderS,
}

pub async fn imu_task(
    port_adi: String,
    baud_rate: u32,
    tx: watch::Sender<ImuVeri>,
) {
    loop {
        println!("IMU portu açılmaya çalışılıyor: {}", port_adi);

        let mut usb_port = match tokio_serial::new(&port_adi, baud_rate).open_native_async() {
            Ok(port) => {
                println!("IMU bağlantısı kuruldu: {}", port_adi);
                port
            }
            Err(e) => {
                eprintln!("IMU portu açılamadı: {e}. 1 saniye sonra tekrar denenecek.");
                sleep(Duration::from_secs(1)).await;
                continue;
            }
        };

        let mut buf = [0u8; 1];
        let mut bucket = [0u8; 45];
        let mut durum = DurumParse::HeaderF;

        'baglanti: loop {
            match durum {
                HeaderF => {
                    if let Err(e) = usb_port.read_exact(&mut buf).await {
                        eprintln!("IMU okuma bağlantı hatası: {e}");
                        break 'baglanti;
                    }

                    if buf[0] == 0xAA {
                        durum = DurumParse::HeaderS;
                    }
                }
                HeaderS => {
                    if let Err(e) = usb_port.read_exact(&mut buf).await {
                        eprintln!("IMU okuma bağlantı hatası: {e}");
                        break 'baglanti;
                    }

                    if buf[0] == 0xBB {
                        if let Err(e) = usb_port.read_exact(&mut bucket).await {
                            eprintln!("IMU paketi yarım kaldı: {e}");
                            break 'baglanti;
                        }

                        let mut calc_checksum = 0u8;
                        for byte in &bucket[..44] {
                            calc_checksum ^= *byte;
                        }

                        if calc_checksum == bucket[44] {
                            let fill_struct = |i: usize| -> f32 {
                                f32::from_le_bytes([
                                    bucket[i],
                                    bucket[i + 1],
                                    bucket[i + 2],
                                    bucket[i + 3],
                                ])
                            };

                            let paket = ImuVeri {
                                roll: fill_struct(0),
                                pitch: fill_struct(4),
                                yaw: fill_struct(8),
                                gx: fill_struct(12),
                                gy: fill_struct(16),
                                gz: fill_struct(20),
                                ax: fill_struct(24),
                                ay: fill_struct(28),
                                az: fill_struct(32),
                                zaman_ms: u64::from_le_bytes(
                                    bucket[36..44].try_into().unwrap(),
                                ),
                            };

                            if tx.send(paket).is_err() {
                                println!("IMU alıcısı kapandı.");
                                return;
                            }
                        }

                        durum = DurumParse::HeaderF;
                    } else if buf[0] == 0xAA {
                        durum = DurumParse::HeaderS;
                    } else {
                        durum = DurumParse::HeaderF;
                    }
                }
            }
        }

        eprintln!("IMU bağlantısı koptu. Yeniden bağlanılıyor...");
        sleep(Duration::from_secs(1)).await;
    }
}
