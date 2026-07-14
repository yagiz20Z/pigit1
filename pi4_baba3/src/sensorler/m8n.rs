use tokio::io::AsyncReadExt;
use tokio::sync::watch;
use tokio::time::{sleep, timeout, Duration};
use tokio_serial::SerialPortBuilderExt;

use crate::veri_tipleri::GpsVeri;

#[derive(Clone, Copy)]
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
                println!("GPS USB bağlantısı kuruldu: {}", port_adi);
                println!("GPS için 0xAA 0xBB başlıklı 33 baytlık paket bekleniyor.");
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
        let mut crc_hata = 0u64;
        let mut gecerli_paket = 0u64;

        'baglanti: loop {
            match durum {
                DParse::HFirst => {
                    match timeout(Duration::from_secs(3), usb_port.read_exact(&mut buf)).await {
                        Ok(Ok(_)) => {
                            if buf[0] == 0xAA {
                                durum = DParse::HSec;
                            }
                        }
                        Ok(Err(e)) => {
                            eprintln!("GPS USB okuma hatası: {e}");
                            break 'baglanti;
                        }
                        Err(_) => {
                            eprintln!("[GPS UYARI] Port açık fakat 3 saniyedir Pico'dan tek bayt gelmedi.");
                        }
                    }
                }
                DParse::HSec => {
                    match timeout(Duration::from_secs(1), usb_port.read_exact(&mut buf)).await {
                        Ok(Ok(_)) => {}
                        Ok(Err(e)) => {
                            eprintln!("GPS USB okuma hatası: {e}");
                            break 'baglanti;
                        }
                        Err(_) => {
                            eprintln!("[GPS UYARI] Paket başlığının ikinci baytı zaman aşımına uğradı.");
                            durum = DParse::HFirst;
                            continue;
                        }
                    }

                    if buf[0] == 0xBB {
                        match timeout(Duration::from_secs(1), usb_port.read_exact(&mut bucket)).await {
                            Ok(Ok(_)) => {}
                            Ok(Err(e)) => {
                                eprintln!("GPS paketi yarım kaldı: {e}");
                                break 'baglanti;
                            }
                            Err(_) => {
                                eprintln!("[GPS UYARI] 31 baytlık paket gövdesi tamamlanmadı.");
                                durum = DParse::HFirst;
                                continue;
                            }
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

                            gecerli_paket = gecerli_paket.wrapping_add(1);
                            if gecerli_paket == 1 {
                                println!("[GPS OK] İlk geçerli M8N paketi alındı.");
                            }

                            if tx.send(paket).is_err() {
                                println!("GPS alıcısı kapandı.");
                                return;
                            }
                        } else {
                            crc_hata = crc_hata.wrapping_add(1);
                            eprintln!(
                                "[GPS CRC HATA] hesaplanan=0x{:02X}, gelen=0x{:02X}, toplam_hata={}",
                                calc_checksum,
                                bucket[30],
                                crc_hata
                            );
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
