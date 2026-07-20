use std::error::Error;
use std::time::Instant;

use tokio::io::{AsyncBufReadExt, AsyncWrite, AsyncWriteExt, BufReader};
use tokio::sync::mpsc;
use tokio::time::{Duration, MissedTickBehavior, interval, sleep};
use tokio_serial::SerialPortBuilderExt;

use crate::veri_tipleri::*;

/// YKİ CMD:PING kullanıyorsa bu süreden uzun sessizlik RF kopması sayılır.
/// PING hiç kullanılmayan arayüzlerde seri bağlantı ve geçerli komutlar yeterlidir.
const HEARTBEAT_ZAMAN_ASIMI: Duration = Duration::from_secs(4);
const BAGLANTI_KONTROL_ARALIGI: Duration = Duration::from_millis(500);
const MAX_ROTA_NOKTASI: usize = 100;

fn koordinat_gecerli(lat: f64, lon: f64) -> bool {
    lat.is_finite()
        && lon.is_finite()
        && (-90.0..=90.0).contains(&lat)
        && (-180.0..=180.0).contains(&lon)
        && !(lat == 0.0 && lon == 0.0)
}

fn command_name(payload: &str) -> &str {
    payload
        .strip_prefix("CMD:")
        .and_then(|rest| rest.split(':').next())
        .unwrap_or("UNKNOWN")
}

fn parse_yk_komut(payload: &str) -> Option<GelenTelemetri> {
    let data = payload.strip_prefix("CMD:")?;
    let mut parts = data.splitn(2, ':');
    let command_type = parts.next()?;
    let args = parts.next();

    match command_type {
        "START" => Some(GelenTelemetri::GoreviBaslat),
        "STOP" => Some(GelenTelemetri::AcilDurdur),
        "PING" => Some(GelenTelemetri::TelemetriHeartbeat),
        "DR" if args == Some("RESET") => Some(GelenTelemetri::DeadReckoningSifirla),
        "HOME" => {
            let mut vals = args?.split(',');
            let lat = vals.next()?.parse::<f64>().ok()?;
            let lon = vals.next()?.parse::<f64>().ok()?;

            if vals.next().is_some() || !koordinat_gecerli(lat, lon) {
                None
            } else {
                Some(GelenTelemetri::EvKonumuBelirle(lat, lon))
            }
        }
        "MOD" => {
            let mod_id = args?.parse::<u8>().ok()?;
            Some(GelenTelemetri::ModDegistir(AracMod::from_u8(mod_id)))
        }
        "MAN" => {
            let mut vals = args?.split(',');
            let ileri = vals.next()?.parse::<f32>().ok()?;
            let yatay_girdi = vals.next()?.parse::<f32>().ok()?;

            if vals.next().is_some()
                || !ileri.is_finite()
                || !yatay_girdi.is_finite()
                || !(0.0..=1.0).contains(&ileri)
            {
                return None;
            }

            // Yeni protokol -1..1 normalize yatay değer kullanır. Eski arayüzün
            // -90..90 derece göndermesi de geriye uyumluluk için kabul edilir.
            let yatay = if (-1.0..=1.0).contains(&yatay_girdi) {
                yatay_girdi
            } else if (-90.0..=90.0).contains(&yatay_girdi) {
                yatay_girdi / 90.0
            } else {
                return None;
            };

            Some(GelenTelemetri::ManuelKontrol(ileri, yatay))
        }
        "MAP" => {
            let mut vals = args?.split(',');
            let esleme = MotorEsleme {
                sol: vals.next()?.parse::<u8>().ok()?,
                ileri1: vals.next()?.parse::<u8>().ok()?,
                sag: vals.next()?.parse::<u8>().ok()?,
                ileri2: vals.next()?.parse::<u8>().ok()?,
            };

            if vals.next().is_some() || !esleme.gecerli() {
                None
            } else {
                Some(GelenTelemetri::MotorEslemeDegistir(esleme))
            }
        }
        "ROTA" => {
            let mut noktalar = Vec::new();

            for nokta_str in args?.split(';') {
                if noktalar.len() >= MAX_ROTA_NOKTASI {
                    return None;
                }

                let mut koordinatlar = nokta_str.split(',');
                let lat = koordinatlar.next()?.parse::<f64>().ok()?;
                let lon = koordinatlar.next()?.parse::<f64>().ok()?;

                if koordinatlar.next().is_some() || !koordinat_gecerli(lat, lon) {
                    return None;
                }

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


fn komut_cevaplari(komut: &GelenTelemetri) -> Vec<String> {
    match komut {
        GelenTelemetri::RotaBelirle(noktalar) => {
            let rota = noktalar
                .iter()
                .map(|(lat, lon)| format!("{lat:.7},{lon:.7}"))
                .collect::<Vec<_>>()
                .join(";");
            vec![format!("STATE:ROTA:{rota}"), "ACK:ROTA".to_string()]
        }
        GelenTelemetri::ModDegistir(_) => vec!["ACK:MOD".to_string()],
        GelenTelemetri::GoreviBaslat => vec!["ACK:START".to_string()],
        GelenTelemetri::AcilDurdur => vec!["ACK:STOP".to_string()],
        GelenTelemetri::MotorEslemeDegistir(_) => vec!["ACK:MAP".to_string()],
        GelenTelemetri::EvKonumuBelirle(_, _) => vec!["ACK:HOME".to_string()],
        GelenTelemetri::DeadReckoningSifirla => vec!["ACK:DR".to_string()],
        // 25 Hz'e kadar gelebilen manuel paketler ve heartbeat için ACK trafiği üretme.
        GelenTelemetri::ManuelKontrol(_, _)
        | GelenTelemetri::TelemetriHeartbeat
        | GelenTelemetri::TelemetriBaglandi
        | GelenTelemetri::TelemetriKoptu => Vec::new(),
    }
}

async fn cevaplari_gonder<W>(yazar: &mut W, cevaplar: &[String]) -> std::io::Result<()>
where
    W: AsyncWrite + Unpin,
{
    for payload in cevaplar {
        let paket = format!("{}*{}\n", payload, calc_checksum(payload));
        yazar.write_all(paket.as_bytes()).await?;
    }
    if !cevaplar.is_empty() {
        yazar.flush().await?;
    }
    Ok(())
}

async fn baglanti_durumu_gonder(
    tx_yki: &mpsc::Sender<GelenTelemetri>,
    durum: GelenTelemetri,
) -> bool {
    tx_yki.send(durum).await.is_ok()
}

pub async fn telemetri_task(
    port_adi: String,
    baud_rate: u32,
    tx_yki: mpsc::Sender<GelenTelemetri>,
    mut rx_yki: mpsc::Receiver<GidenTelemetri>,
) -> Result<(), Box<dyn Error + Send + Sync>> {
    let mut kopuk_bildirildi = false;

    loop {
        // Bağlantı yokken oluşmuş eski NAV/MOT paketlerini at. Yeniden bağlantıda
        // yalnızca güncel telemetri iletilir.
        while rx_yki.try_recv().is_ok() {}

        println!("Telemetri portu açılmaya çalışılıyor: {}", port_adi);

        let tel_port = match tokio_serial::new(&port_adi, baud_rate).open_native_async() {
            Ok(port) => {
                println!("Telemetri seri portu açıldı: {}", port_adi);
                port
            }
            Err(e) => {
                eprintln!("Telemetri portu açılamadı: {e}. 1 saniye sonra tekrar denenecek.");
                if !kopuk_bildirildi {
                    if !baglanti_durumu_gonder(&tx_yki, GelenTelemetri::TelemetriKoptu).await {
                        return Ok(());
                    }
                    kopuk_bildirildi = true;
                }
                sleep(Duration::from_secs(1)).await;
                continue;
            }
        };

        // Fiziksel USB/seri portun açılması tek başına RF bağlantısı sayılmaz.
        // İlk geçerli kontrol paketi (ROTA/MOD/START/PING vb.) oturumu kurar.
        // Böylece PING göndermeyen arayüzlerde ilk görev paketleri reddedilmez.
        let mut bu_oturum_bagli = false;
        let mut son_ping: Option<Instant> = None;
        println!("RF bağlantısı için ilk geçerli CMD paketi bekleniyor...");

        let (okur, mut yazar) = tokio::io::split(tel_port);
        let mut satirlar = BufReader::new(okur).lines();
        let mut kontrol_tick = interval(BAGLANTI_KONTROL_ARALIGI);
        kontrol_tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
        kontrol_tick.tick().await;

        'baglanti: loop {
            tokio::select! {
                gelen = satirlar.next_line() => {
                    match gelen {
                        Ok(Some(line)) => {
                            let temiz = line.trim();

                            let Some((payload, cs)) = temiz.split_once('*') else {
                                eprintln!("Checksum alanı olmayan telemetri paketi reddedildi: {}", temiz);
                                continue;
                            };

                            if calc_checksum(payload) != cs.to_uppercase() {
                                eprintln!("Hatalı telemetri checksum: {}", temiz);
                                continue;
                            }

                            let Some(komut) = parse_yk_komut(payload) else {
                                eprintln!("Geçersiz telemetri komutu reddedildi: {}", payload);
                                continue;
                            };
                            let cevaplar = komut_cevaplari(&komut);

                            let heartbeat = matches!(&komut, GelenTelemetri::TelemetriHeartbeat);
                            let acil_stop = matches!(&komut, GelenTelemetri::AcilDurdur);

                            if heartbeat {
                                son_ping = Some(Instant::now());
                            }

                            // STOP her zaman işlenir fakat tek başına "bağlantı kuruldu"
                            // sayılmaz. Diğer ilk geçerli komut, PING olmasa da oturumu kurar.
                            if !bu_oturum_bagli && !acil_stop {
                                if !baglanti_durumu_gonder(
                                    &tx_yki,
                                    GelenTelemetri::TelemetriBaglandi,
                                )
                                .await
                                {
                                    return Ok(());
                                }

                                kopuk_bildirildi = false;
                                bu_oturum_bagli = true;
                                println!(
                                    "Telemetri RF oturumu ilk geçerli komutla kuruldu: {}",
                                    command_name(payload),
                                );
                            }

                            if tx_yki.send(komut).await.is_err() {
                                return Ok(());
                            }
                            if let Err(e) = cevaplari_gonder(&mut yazar, &cevaplar).await {
                                eprintln!("Telemetri cevap yazma hatası: {e}");
                                break 'baglanti;
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

                    // RF heartbeat kurulmadan telemetri yazma; eski/boş seri bağlantıya
                    // sürekli veri basarak kontrol kanalını maskeleme.
                    if !bu_oturum_bagli {
                        continue;
                    }

                    let (nav_str, mot_str, dr_str) = telemetri.to_rf_strings();

                    if let Err(e) = yazar.write_all(nav_str.as_bytes()).await {
                        eprintln!("Telemetri NAV yazma hatası: {e}");
                        break 'baglanti;
                    }

                    if let Err(e) = yazar.write_all(mot_str.as_bytes()).await {
                        eprintln!("Telemetri MOT yazma hatası: {e}");
                        break 'baglanti;
                    }

                    if let Err(e) = yazar.write_all(dr_str.as_bytes()).await {
                        eprintln!("Telemetri DR yazma hatası: {e}");
                        break 'baglanti;
                    }

                    if let Err(e) = yazar.flush().await {
                        eprintln!("Telemetri flush hatası: {e}");
                        break 'baglanti;
                    }
                }
                _ = kontrol_tick.tick() => {
                    // PING protokolünü kullanan bir oturumda watchdog aktiftir.
                    // Oturum boyunca hiç PING gelmediyse yalnız seri okuma/yazma hatası
                    // bağlantıyı düşürür; otonom görev 4 saniyede yanlışlıkla kesilmez.
                    let zaman_asimi = son_ping
                        .as_ref()
                        .map(|ping| ping.elapsed() > HEARTBEAT_ZAMAN_ASIMI)
                        .unwrap_or(false);

                    if zaman_asimi {
                        eprintln!(
                            "Telemetri heartbeat zaman aşımı ({} ms). RF bağlantısı koptu kabul edildi.",
                            HEARTBEAT_ZAMAN_ASIMI.as_millis()
                        );
                        break 'baglanti;
                    }
                }
            }
        }

        if !kopuk_bildirildi {
            if !baglanti_durumu_gonder(&tx_yki, GelenTelemetri::TelemetriKoptu).await {
                return Ok(());
            }
            kopuk_bildirildi = true;
        }

        eprintln!("Telemetri bağlantısı koptu. Güvenli dönüş sürerken yeniden bağlanılıyor...");
        sleep(Duration::from_secs(1)).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manuel_normalize_ve_derece_kabul_edilir() {
        match parse_yk_komut("CMD:MAN:0.75,0.5") {
            Some(GelenTelemetri::ManuelKontrol(ileri, yatay)) => {
                assert!((ileri - 0.75).abs() < f32::EPSILON);
                assert!((yatay - 0.5).abs() < f32::EPSILON);
            }
            other => panic!("Beklenmeyen sonuç: {other:?}"),
        }

        match parse_yk_komut("CMD:MAN:0.75,45.0") {
            Some(GelenTelemetri::ManuelKontrol(_, yatay)) => {
                assert!((yatay - 0.5).abs() < f32::EPSILON);
            }
            other => panic!("Beklenmeyen sonuç: {other:?}"),
        }
    }

    #[test]
    fn gecersiz_rota_reddedilir() {
        assert!(parse_yk_komut("CMD:ROTA:91.0,29.0").is_none());
        assert!(parse_yk_komut("CMD:ROTA:41.0,29.0,fazla").is_none());
        assert!(parse_yk_komut("CMD:ROTA:0.0,0.0").is_none());
    }

    #[test]
    fn ping_cozumlenir() {
        assert!(matches!(
            parse_yk_komut("CMD:PING"),
            Some(GelenTelemetri::TelemetriHeartbeat)
        ));
    }


    #[test]
    fn komut_adi_cikarilir() {
        assert_eq!(command_name("CMD:ROTA:41.0,29.0"), "ROTA");
        assert_eq!(command_name("CMD:START"), "START");
    }

    #[test]
    fn dead_reckoning_reset_cozumlenir() {
        assert!(matches!(
            parse_yk_komut("CMD:DR:RESET"),
            Some(GelenTelemetri::DeadReckoningSifirla)
        ));
    }
}
