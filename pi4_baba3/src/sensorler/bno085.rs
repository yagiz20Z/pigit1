use tokio::io::AsyncReadExt;
use tokio::sync::watch;
use tokio::time::{sleep, timeout, Duration};
use tokio_serial::SerialPortBuilderExt;

use crate::veri_tipleri::ImuVeri;

#[derive(Clone, Copy)]
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
                println!("IMU USB bağlantısı kuruldu: {}", port_adi);
                println!("IMU için 0xAA 0xBB başlıklı 47 baytlık paket bekleniyor.");
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
        let mut crc_hata = 0u64;
        let mut gecerli_paket = 0u64;

        'baglanti: loop {
            match durum {
                DurumParse::HeaderF => {
                    match timeout(Duration::from_secs(3), usb_port.read_exact(&mut buf)).await {
                        Ok(Ok(_)) => {
                            if buf[0] == 0xAA {
                                durum = DurumParse::HeaderS;
                            }
                        }
                        Ok(Err(e)) => {
                            eprintln!("IMU USB okuma hatası: {e}");
                            break 'baglanti;
                        }
                        Err(_) => {
                            eprintln!("[IMU UYARI] Port açık fakat 3 saniyedir Pico'dan tek bayt gelmedi.");
                        }
                    }
                }
                DurumParse::HeaderS => {
                    match timeout(Duration::from_secs(1), usb_port.read_exact(&mut buf)).await {
                        Ok(Ok(_)) => {}
                        Ok(Err(e)) => {
                            eprintln!("IMU USB okuma hatası: {e}");
                            break 'baglanti;
                        }
                        Err(_) => {
                            eprintln!("[IMU UYARI] Paket başlığının ikinci baytı zaman aşımına uğradı.");
                            durum = DurumParse::HeaderF;
                            continue;
                        }
                    }

                    if buf[0] == 0xBB {
                        match timeout(Duration::from_secs(1), usb_port.read_exact(&mut bucket)).await {
                            Ok(Ok(_)) => {}
                            Ok(Err(e)) => {
                                eprintln!("IMU paketi yarım kaldı: {e}");
                                break 'baglanti;
                            }
                            Err(_) => {
                                eprintln!("[IMU UYARI] 45 baytlık paket gövdesi tamamlanmadı.");
                                durum = DurumParse::HeaderF;
                                continue;
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
                                zaman_ms: u64::from_le_bytes(
                                    bucket[36..44].try_into().unwrap(),
                                ),
                            };

                            gecerli_paket = gecerli_paket.wrapping_add(1);
                            if gecerli_paket == 1 {
                                println!("[IMU OK] İlk geçerli BNO085 paketi alındı.");
                            }

                            if tx.send(paket).is_err() {
                                println!("IMU alıcısı kapandı.");
                                return;
                            }
                        } else {
                            crc_hata = crc_hata.wrapping_add(1);
                            eprintln!(
                                "[IMU CRC HATA] hesaplanan=0x{:02X}, gelen=0x{:02X}, toplam_hata={}",
                                calc_checksum,
                                bucket[44],
                                crc_hata
                            );
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
