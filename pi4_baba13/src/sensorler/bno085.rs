use tokio::io::AsyncReadExt;
use tokio::sync::watch;
use tokio::time::{Duration, sleep, timeout};
use tokio_serial::SerialPortBuilderExt;

use crate::sensorler::bno085::DurumParse::{HeaderF, HeaderS};
use crate::veri_tipleri::ImuVeri;

const HEADER_BAYT_TIMEOUT: Duration = Duration::from_secs(5);
const PAKET_BAYT_TIMEOUT: Duration = Duration::from_secs(1);

enum DurumParse {
    HeaderF,
    HeaderS,
}

pub async fn imu_task(port_adi: String, baud_rate: u32, tx: watch::Sender<ImuVeri>) {
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
        let mut gecerli_paket: u64 = 0;
        let mut crc_hatasi: u64 = 0;

        'baglanti: loop {
            match durum {
                HeaderF => {
                    match timeout(HEADER_BAYT_TIMEOUT, usb_port.read_exact(&mut buf)).await {
                        Ok(Ok(_)) => {}
                        Ok(Err(e)) => {
                            eprintln!("IMU okuma bağlantı hatası: {e}");
                            break 'baglanti;
                        }
                        Err(_) => {
                            eprintln!(
                                "[IMU USB SESSİZ] 5 saniyedir veri yok; port yeniden açılacak."
                            );
                            break 'baglanti;
                        }
                    }

                    if buf[0] == 0xAA {
                        durum = DurumParse::HeaderS;
                    }
                }
                HeaderS => {
                    match timeout(PAKET_BAYT_TIMEOUT, usb_port.read_exact(&mut buf)).await {
                        Ok(Ok(_)) => {}
                        Ok(Err(e)) => {
                            eprintln!("IMU ikinci header okuma hatası: {e}");
                            break 'baglanti;
                        }
                        Err(_) => {
                            eprintln!("IMU ikinci header zaman aşımı; port yeniden açılacak.");
                            break 'baglanti;
                        }
                    }

                    if buf[0] == 0xBB {
                        match timeout(PAKET_BAYT_TIMEOUT, usb_port.read_exact(&mut bucket)).await {
                            Ok(Ok(_)) => {}
                            Ok(Err(e)) => {
                                eprintln!("IMU paketi yarım kaldı: {e}");
                                break 'baglanti;
                            }
                            Err(_) => {
                                eprintln!("IMU paket gövdesi zaman aşımı; port yeniden açılacak.");
                                break 'baglanti;
                            }
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
                                zaman_ms: u64::from_le_bytes(bucket[36..44].try_into().unwrap()),
                            };

                            gecerli_paket = gecerli_paket.wrapping_add(1);

                            if gecerli_paket == 1 || gecerli_paket % 20 == 0 {
                                println!(
                                    "[IMU OK] port={} paket={} crc_hata={} roll={:.2} pitch={:.2} yaw={:.2} gyro=({:.2},{:.2},{:.2}) accel=({:.2},{:.2},{:.2}) pico_ms={}",
                                    port_adi,
                                    gecerli_paket,
                                    crc_hatasi,
                                    paket.roll,
                                    paket.pitch,
                                    paket.yaw,
                                    paket.gx,
                                    paket.gy,
                                    paket.gz,
                                    paket.ax,
                                    paket.ay,
                                    paket.az,
                                    paket.zaman_ms,
                                );
                            }

                            if tx.send(paket).is_err() {
                                println!("IMU alıcısı kapandı.");
                                return;
                            }
                        } else {
                            crc_hatasi = crc_hatasi.wrapping_add(1);
                            if crc_hatasi == 1 || crc_hatasi % 20 == 0 {
                                eprintln!(
                                    "[IMU CRC HATA] port={} hata={} hesap={:02X} gelen={:02X}. Portlar ters olabilir.",
                                    port_adi, crc_hatasi, calc_checksum, bucket[44],
                                );
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
