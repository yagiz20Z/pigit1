use std::error::Error;
use std::time::Instant;

use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::sync::mpsc;
use tokio::time::{interval, sleep, Duration, MissedTickBehavior};
use tokio_serial::SerialPortBuilderExt;

use crate::veri_tipleri::*;

/// YKİ `CMD:PING` göndermeye başladıktan sonra bu süre boyunca yeni bir
/// geçerli paket gelmezse RF bağlantısı kopmuş kabul edilir.
const HEARTBEAT_ZAMAN_ASIMI: Duration = Duration::from_secs(4);
const BAGLANTI_KONTROL_ARALIGI: Duration = Duration::from_millis(500);

fn parse_yk_komut(payload: &str) -> Option<GelenTelemetri> {
    let data = payload.strip_prefix("CMD:")?;
    let mut parts = data.splitn(2, ':');
    let command_type = parts.next()?;
    let args = parts.next();

    match command_type {
        "START" => Some(GelenTelemetri::GoreviBaslat),
        "STOP" => Some(GelenTelemetri::AcilDurdur),
        "PING" => Some(GelenTelemetri::TelemetriHeartbeat),
        "HOME" => {
            let mut vals = args?.split(',');
            let lat = vals.next()?.parse::<f64>().ok()?;
            let lon = vals.next()?.parse::<f64>().ok()?;

            if vals.next().is_some()
                || !lat.is_finite()
                || !lon.is_finite()
                || !(-90.0..=90.0).contains(&lat)
                || !(-180.0..=180.0).contains(&lon)
                || (lat == 0.0 && lon == 0.0)
            {
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
            let yatay = vals.next()?.parse::<f32>().ok()?;
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
    // İlk PING alındıktan sonra RF bağlantısı heartbeat ile izlenir. Bu bilgi
    // seri port yeniden açılsa bile korunur; böylece yalnızca USB portunun açık
    // olması yanlışlıkla "RF yeniden bağlandı" sayılmaz.
    let mut heartbeat_watchdog_aktif = false;
    let mut son_gecerli_yki_paketi: Option<Instant> = None;
    let mut kopuk_bildirildi = false;

    loop {
        // Bağlantı yokken oluşmuş eski NAV/MOT paketlerini at. Beyin tarafı
        // bloklanmadan dönüş hesabına devam eder; yeniden bağlanınca güncel paket gider.
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

        // Eski YKİ heartbeat kullanmıyorsa fiziksel portun açılması bağlantı kabul edilir.
        // Heartbeat daha önce devreye girdiyse gerçek bir geçerli YKİ paketi beklenir.
        let mut bu_oturum_bagli_bildirildi = false;
        let oturum_acilis = Instant::now();
        if !heartbeat_watchdog_aktif {
            if !baglanti_durumu_gonder(&tx_yki, GelenTelemetri::TelemetriBaglandi).await {
                return Ok(());
            }
            kopuk_bildirildi = false;
            bu_oturum_bagli_bildirildi = true;
            println!("Telemetri bağlantısı kuruldu: {}", port_adi);
        } else {
            println!("RF heartbeat bekleniyor...");
        }

        let (okur, mut yazar) = tokio::io::split(tel_port);
        let mut satirlar = BufReader::new(okur).lines();
        let mut kontrol_tick = interval(BAGLANTI_KONTROL_ARALIGI);
        kontrol_tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
        // interval'in ilk anlık tick'ini tüket; RF paketine gelmesi için gerçek bir
        // kontrol aralığı bırak.
        kontrol_tick.tick().await;

        'baglanti: loop {
            tokio::select! {
                gelen = satirlar.next_line() => {
                    match gelen {
                        Ok(Some(line)) => {
                            let temiz = line.trim();

                            if let Some((payload, cs)) = temiz.split_once('*') {
                                if calc_checksum(payload) == cs.to_uppercase() {
                                    if let Some(komut) = parse_yk_komut(payload) {
                                        son_gecerli_yki_paketi = Some(Instant::now());

                                        if matches!(&komut, GelenTelemetri::TelemetriHeartbeat) {
                                            heartbeat_watchdog_aktif = true;
                                        }

                                        if !bu_oturum_bagli_bildirildi {
                                            if !baglanti_durumu_gonder(
                                                &tx_yki,
                                                GelenTelemetri::TelemetriBaglandi,
                                            )
                                            .await
                                            {
                                                return Ok(());
                                            }
                                            kopuk_bildirildi = false;
                                            bu_oturum_bagli_bildirildi = true;
                                            println!("Telemetri RF bağlantısı yeniden kuruldu.");
                                        }

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
                _ = kontrol_tick.tick() => {
                    if heartbeat_watchdog_aktif {
                        let zaman_asimi = if bu_oturum_bagli_bildirildi {
                            son_gecerli_yki_paketi
                                .as_ref()
                                .map(|son_paket| son_paket.elapsed() > HEARTBEAT_ZAMAN_ASIMI)
                                .unwrap_or(true)
                        } else {
                            // Seri port açıldı fakat RF tarafından henüz geçerli paket
                            // gelmedi. Yeniden bağlanma için tam timeout süresi tanı.
                            oturum_acilis.elapsed() > HEARTBEAT_ZAMAN_ASIMI
                        };

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
