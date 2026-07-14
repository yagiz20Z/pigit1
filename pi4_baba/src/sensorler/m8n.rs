use tokio::io::AsyncReadExt;
use tokio::sync::watch;
use tokio::time::{sleep, Duration};
use tokio_serial::SerialPortBuilderExt;

use crate::sensorler::m8n::DParse::{HFirst, HSec};
use crate::veri_tipleri::GpsVeri;

enum DParse {
    HFirst,
    HSec,
}

pub async fn gps_task(
    port_adi: String,
    baud_rate: u32,
    tx: watch::Sender<GpsVeri>,
) {
    loop {
        println!("GPS portu açılmaya çalışılıyor: {}", port_adi);

        let mut usb_port = match tokio_serial::new(&port_adi, baud_rate).open_native_async() {
            Ok(port) => {
                println!("GPS bağlantısı kuruldu: {}", port_adi);
                port
            }
            Err(e) => {
                eprintln!("GPS portu açılamadı: {e}. 1 saniye sonra tekrar denenecek.");
                sleep(Duration::from_secs(1)).await;
                continue;
            }
        };

        let mut buf = [0u8; 1];
        let mut bucket = [0u8; 31];
        let mut durum = DParse::HFirst;

        'baglanti: loop {
            match durum {
                HFirst => {
                    if let Err(e) = usb_port.read_exact(&mut buf).await {
                        eprintln!("GPS okuma bağlantı hatası: {e}");
                        break 'baglanti;
                    }

                    if buf[0] == 0xAA {
                        durum = DParse::HSec;
                    }
                }
                HSec => {
                    if let Err(e) = usb_port.read_exact(&mut buf).await {
                        eprintln!("GPS okuma bağlantı hatası: {e}");
                        break 'baglanti;
                    }

                    if buf[0] == 0xBB {
                        if let Err(e) = usb_port.read_exact(&mut bucket).await {
                            eprintln!("GPS paketi yarım kaldı: {e}");
                            break 'baglanti;
                        }

                        let mut calc_checksum = 0u8;
                        for byte in &bucket[..30] {
                            calc_checksum ^= *byte;
                        }

                        if calc_checksum == bucket[30] {
                            let fill_struct = |i: usize| -> i32 {
                                i32::from_le_bytes([
                                    bucket[i],
                                    bucket[i + 1],
                                    bucket[i + 2],
                                    bucket[i + 3],
                                ])
                            };

                            let paket = GpsVeri {
                                algi_boyut: bucket[0],
                                uydu_sayi: bucket[1],
                                boylam: fill_struct(2),
                                enlem: fill_struct(6),
                                yukseklik_mm: fill_struct(10),
                                hiz: fill_struct(14),
                                yonelim: fill_struct(18),
                                zaman_ms: u64::from_le_bytes(
                                    bucket[22..30].try_into().unwrap(),
                                ),
                            };

                            if tx.send(paket).is_err() {
                                println!("GPS alıcısı kapandı.");
                                return;
                            }
                        }

                        durum = DParse::HFirst;
                    } else if buf[0] == 0xAA {
                        durum = DParse::HSec;
                    } else {
                        durum = DParse::HFirst;
                    }
                }
            }
        }

        eprintln!("GPS bağlantısı koptu. Yeniden bağlanılıyor...");
        sleep(Duration::from_secs(1)).await;
    }
}
