use std::error::Error;
use std::time::Instant;

use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::sync::mpsc;
use tokio::time::{Duration, MissedTickBehavior, interval, sleep};
use tokio_serial::SerialPortBuilderExt;

use crate::veri_tipleri::*;

/// YKİ bu süreden daha uzun süre CMD:PING göndermezse RF bağlantısı kopmuş kabul edilir.
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

        // Fiziksel USB/seri portun açılması RF bağlantısı sayılmaz. Bağlantı ancak
        // geçerli CMD:PING alındığında kurulmuş kabul edilir.
        let mut bu_oturum_bagli = false;
        let mut son_ping: Option<Instant> = None;
        let oturum_acilis = Instant::now();
        println!("RF bağlantısı için CMD:PING bekleniyor...");

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

                            if matches!(&komut, GelenTelemetri::TelemetriHeartbeat) {
                                son_ping = Some(Instant::now());

                                if !bu_oturum_bagli {
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
                                    println!("Telemetri RF bağlantısı CMD:PING ile kuruldu.");
                                }

                                // Beyin heartbeat içeriğini kullanmasa da bağlantı olayı olarak ilet.
                                if tx_yki.send(komut).await.is_err() {
                                    return Ok(());
                                }
                            } else if matches!(&komut, GelenTelemetri::AcilDurdur) {
                                // Acil STOP güvenli yönde bir komuttur; heartbeat kurulmamış
                                // olsa bile geçerli checksum ile gelirse her zaman işle.
                                if tx_yki.send(komut).await.is_err() {
                                    return Ok(());
                                }
                            } else if matches!(
                                &komut,
                                GelenTelemetri::ModDegistir(AracMod::Manuel)
                            ) {
                                // STOP, heartbeat yokken de kabul edildiği için eski kodda şu
                                // kilit oluşabiliyordu: araç AcilDurum'a giriyor fakat MOD:0,
                                // PING gelmeden reddedildiği için kullanıcı acilden çıkamıyordu.
                                // MOD:0 motor hareketi üretmez; yalnızca güvenli manuel moda
                                // geçiş isteğidir. Bu nedenle geçerli checksum'lu MOD:0 komutu
                                // oturumu yeniden kurabilir. Ardından MAN komutları normal akıştan
                                // geçer; PING gelmezse 4 saniyelik watchdog yine bağlantıyı keser.
                                son_ping = Some(Instant::now());

                                if !bu_oturum_bagli {
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
                                        "Acil durumdan çıkış: CMD:MOD:0 ile telemetri oturumu yeniden kuruldu."
                                    );
                                }

                                if tx_yki.send(komut).await.is_err() {
                                    return Ok(());
                                }
                            } else if bu_oturum_bagli {
                                if tx_yki.send(komut).await.is_err() {
                                    return Ok(());
                                }
                            } else {
                                eprintln!("CMD:PING gelmeden kontrol komutu reddedildi: {}", payload);
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
                    let zaman_asimi = son_ping
                        .as_ref()
                        .map(|ping| ping.elapsed() > HEARTBEAT_ZAMAN_ASIMI)
                        .unwrap_or_else(|| oturum_acilis.elapsed() > HEARTBEAT_ZAMAN_ASIMI);

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
    fn dead_reckoning_reset_cozumlenir() {
        assert!(matches!(
            parse_yk_komut("CMD:DR:RESET"),
            Some(GelenTelemetri::DeadReckoningSifirla)
        ));
    }
}
