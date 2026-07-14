use std::error::Error;

use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::sync::mpsc;
use tokio::time::{sleep, Duration};
use tokio_serial::SerialPortBuilderExt;

use crate::veri_tipleri::*;

fn parse_yk_komut(payload: &str) -> Option<GelenTelemetri> {
    let data = payload.strip_prefix("CMD:")?;
    let mut parts = data.splitn(2, ':');
    let command_type = parts.next()?;
    let args = parts.next();

    match command_type {
        "START" => Some(GelenTelemetri::GoreviBaslat),
        "STOP" => Some(GelenTelemetri::AcilDurdur),
        "MOD" => {
            let mod_id = args?.parse::<u8>().ok()?;
            Some(GelenTelemetri::ModDegistir(AracMod::from_u8(mod_id)))
        }
        "MAN" => {
            let mut vals = args?.split(',');
            let gaz = vals.next()?.parse::<f32>().ok()?;
            let aci = vals.next()?.parse::<f32>().ok()?;
            Some(GelenTelemetri::ManuelKontrol(gaz, aci))
        }
        "ROTA" => {
            let mut noktalar = Vec::new();

            for nokta_str in args?.split(';') {
                let mut koordinatlar = nokta_str.split(',');
                let lat = koordinatlar.next()?.parse::<f64>().ok()?;
                let lon = koordinatlar.next()?.parse::<f64>().ok()?;
                noktalar.push((lat, lon));
            }

            if noktalar.is_empty() {
                None
            } else {
                Some(GelenTelemetri::RotaBelirle(noktalar))
            }
        }
        _ => None,
    }
}

pub async fn telemetri_task(
    port_adi: String,
    baud_rate: u32,
    tx_yki: mpsc::Sender<GelenTelemetri>,
    mut rx_yki: mpsc::Receiver<GidenTelemetri>,
) -> Result<(), Box<dyn Error + Send + Sync>> {
    loop {
        println!("Telemetri portu açılmaya çalışılıyor: {}", port_adi);

        let tel_port = match tokio_serial::new(&port_adi, baud_rate).open_native_async() {
            Ok(port) => {
                println!("Telemetri bağlantısı kuruldu: {}", port_adi);
                port
            }
            Err(e) => {
                eprintln!("Telemetri portu açılamadı: {e}. 1 saniye sonra tekrar denenecek.");
                sleep(Duration::from_secs(1)).await;
                continue;
            }
        };

        let (okur, mut yazar) = tokio::io::split(tel_port);
        let mut satirlar = BufReader::new(okur).lines();

        'baglanti: loop {
            tokio::select! {
                gelen = satirlar.next_line() => {
                    match gelen {
                        Ok(Some(line)) => {
                            let temiz = line.trim();

                            if let Some((payload, cs)) = temiz.split_once('*') {
                                if calc_checksum(payload) == cs.to_uppercase() {
                                    if let Some(komut) = parse_yk_komut(payload) {
                                        if tx_yki.send(komut).await.is_err() {
                                            return Ok(());
                                        }
                                    }
                                } else {
                                    eprintln!("Hatalı telemetri checksum: {}", temiz);
                                }
                            }
                        }
                        Ok(None) => {
                            eprintln!("Telemetri bağlantısı kapandı.");
                            break 'baglanti;
                        }
                        Err(e) => {
                            eprintln!("Telemetri okuma hatası: {e}");
                            break 'baglanti;
                        }
                    }
                }
                giden = rx_yki.recv() => {
                    let Some(telemetri) = giden else {
                        return Ok(());
                    };

                    let (nav_str, mot_str) = telemetri.to_rf_strings();

                    if let Err(e) = yazar.write_all(nav_str.as_bytes()).await {
                        eprintln!("Telemetri NAV yazma hatası: {e}");
                        break 'baglanti;
                    }

                    if let Err(e) = yazar.write_all(mot_str.as_bytes()).await {
                        eprintln!("Telemetri MOT yazma hatası: {e}");
                        break 'baglanti;
                    }

                    if let Err(e) = yazar.flush().await {
                        eprintln!("Telemetri flush hatası: {e}");
                        break 'baglanti;
                    }
                }
            }
        }

        eprintln!("Telemetri bağlantısı koptu. Yeniden bağlanılıyor...");
        sleep(Duration::from_secs(1)).await;
    }
}
