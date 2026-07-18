use std::time::{SystemTime, UNIX_EPOCH};

use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::sync::watch;
use tokio::time::{Duration, Instant, sleep, timeout};
use tokio_serial::SerialPortBuilderExt;

use crate::veri_tipleri::GpsVeri;

const ILK_NMEA_TIMEOUT: Duration = Duration::from_secs(4);
const NMEA_SESSIZ_TIMEOUT: Duration = Duration::from_secs(5);
const YENIDEN_DENE: Duration = Duration::from_secs(1);

#[derive(Debug)]
struct NmeaDurum {
    fix_tipi: u8,
    uydu_sayisi: u8,
    enlem: Option<i32>,
    boylam: Option<i32>,
    yukseklik_mm: i32,
    hiz_mm_s: i32,
    yonelim_1e5_derece: i32,
}

impl Default for NmeaDurum {
    fn default() -> Self {
        Self {
            fix_tipi: 1,
            uydu_sayisi: 0,
            enlem: None,
            boylam: None,
            yukseklik_mm: 0,
            hiz_mm_s: 0,
            yonelim_1e5_derece: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CumleSonucu {
    /// Geçerli NMEA cümlesi, ancak GpsVeri yayınlamak gerekmiyor.
    Gecerli,
    /// Konum/hız/yön değişti; mevcut durum yayınlanabilir.
    Yayinla,
    /// Satır NMEA değil veya checksum/alanları bozuk.
    Gecersiz,
}

fn simdiki_zaman_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn nmea_checksum_gecerli(satir: &str) -> bool {
    let satir = satir.trim();
    if !satir.starts_with('$') {
        return false;
    }

    let Some((govde, gelen_hex)) = satir[1..].split_once('*') else {
        // Bazı USB dönüştürücü/test kaynakları checksum eklemeyebilir.
        // Gerçek M8N çıktısında checksum vardır; eksikse yine de parser çalışabilir.
        return true;
    };

    let gelen_hex = gelen_hex.trim();
    if gelen_hex.len() < 2 {
        return false;
    }

    let Ok(gelen) = u8::from_str_radix(&gelen_hex[..2], 16) else {
        return false;
    };

    let hesaplanan = govde.bytes().fold(0u8, |toplam, bayt| toplam ^ bayt);
    hesaplanan == gelen
}

fn nmea_alanlari(satir: &str) -> Option<Vec<&str>> {
    if !nmea_checksum_gecerli(satir) {
        return None;
    }

    let govde = satir.trim().strip_prefix('$')?;
    let govde = govde.split_once('*').map(|(sol, _)| sol).unwrap_or(govde);
    Some(govde.split(',').collect())
}

fn koordinat_1e7(raw: &str, yon: &str) -> Option<i32> {
    if raw.is_empty() || yon.is_empty() {
        return None;
    }

    let deger = raw.parse::<f64>().ok()?;
    if !deger.is_finite() || deger < 0.0 {
        return None;
    }

    let derece = (deger / 100.0).floor();
    let dakika = deger - derece * 100.0;
    if !(0.0..60.0).contains(&dakika) {
        return None;
    }

    let mut ondalik = derece + dakika / 60.0;
    match yon {
        "S" | "W" => ondalik = -ondalik,
        "N" | "E" => {},
        _ => return None,
    }

    let sinir = match yon {
        "N" | "S" => 90.0,
        "E" | "W" => 180.0,
        _ => return None,
    };
    if ondalik.abs() > sinir {
        return None;
    }

    let olcekli = (ondalik * 10_000_000.0).round();
    if olcekli < i32::MIN as f64 || olcekli > i32::MAX as f64 {
        return None;
    }
    Some(olcekli as i32)
}

fn metreyi_mm(metre: &str) -> Option<i32> {
    let metre = metre.parse::<f64>().ok()?;
    if !metre.is_finite() {
        return None;
    }
    let mm = (metre * 1000.0).round();
    Some(mm.clamp(i32::MIN as f64, i32::MAX as f64) as i32)
}

fn knot_mm_s(knot: &str) -> Option<i32> {
    let knot = knot.parse::<f64>().ok()?;
    if !knot.is_finite() || knot < 0.0 {
        return None;
    }
    Some((knot * 0.514_444 * 1000.0).round().clamp(0.0, i32::MAX as f64) as i32)
}

fn kmh_mm_s(kmh: &str) -> Option<i32> {
    let kmh = kmh.parse::<f64>().ok()?;
    if !kmh.is_finite() || kmh < 0.0 {
        return None;
    }
    Some((kmh / 3.6 * 1000.0).round().clamp(0.0, i32::MAX as f64) as i32)
}

fn yonelim_1e5(derece: &str) -> Option<i32> {
    let derece = derece.parse::<f64>().ok()?;
    if !derece.is_finite() {
        return None;
    }
    let normalize = derece.rem_euclid(360.0);
    Some((normalize * 100_000.0).round() as i32)
}

fn gga_isle(alan: &[&str], durum: &mut NmeaDurum) -> CumleSonucu {
    // $GNGGA,zaman,lat,N,lon,E,fix_quality,sat,hdop,alt,M,...
    if alan.len() < 10 {
        return CumleSonucu::Gecersiz;
    }

    let fix_quality = alan[6].parse::<u8>().unwrap_or(0);
    durum.uydu_sayisi = alan[7].parse::<u8>().unwrap_or(durum.uydu_sayisi);

    if let Some(alt) = metreyi_mm(alan[9]) {
        durum.yukseklik_mm = alt;
    }

    if fix_quality == 0 {
        durum.fix_tipi = 1;
        return CumleSonucu::Yayinla;
    }

    let (Some(enlem), Some(boylam)) = (
        koordinat_1e7(alan[2], alan[3]),
        koordinat_1e7(alan[4], alan[5]),
    ) else {
        return CumleSonucu::Gecersiz;
    };

    durum.enlem = Some(enlem);
    durum.boylam = Some(boylam);

    // GGA yalnız başına 2B/3B ayrımını kesin vermez. Güvenlik için 2 kabul edilir;
    // otonomiye izin veren 3B bilgisi GSA cümlesinden gelmelidir.
    if durum.fix_tipi < 2 {
        durum.fix_tipi = 2;
    }

    CumleSonucu::Yayinla
}

fn rmc_isle(alan: &[&str], durum: &mut NmeaDurum) -> CumleSonucu {
    // $GNRMC,zaman,A,lat,N,lon,E,speed_knots,course,date,...
    if alan.len() < 9 {
        return CumleSonucu::Gecersiz;
    }

    if let Some(hiz) = knot_mm_s(alan[7]) {
        durum.hiz_mm_s = hiz;
    }
    if let Some(yon) = yonelim_1e5(alan[8]) {
        durum.yonelim_1e5_derece = yon;
    }

    if alan[2] != "A" {
        durum.fix_tipi = 1;
        return CumleSonucu::Yayinla;
    }

    let (Some(enlem), Some(boylam)) = (
        koordinat_1e7(alan[3], alan[4]),
        koordinat_1e7(alan[5], alan[6]),
    ) else {
        return CumleSonucu::Gecersiz;
    };

    durum.enlem = Some(enlem);
    durum.boylam = Some(boylam);
    if durum.fix_tipi < 2 {
        durum.fix_tipi = 2;
    }

    CumleSonucu::Yayinla
}

fn gsa_isle(alan: &[&str], durum: &mut NmeaDurum) -> CumleSonucu {
    // $GNGSA,A,3,...  -> alan[2] fix type: 1=no, 2=2D, 3=3D
    if alan.len() < 3 {
        return CumleSonucu::Gecersiz;
    }
    if let Ok(fix) = alan[2].parse::<u8>() {
        durum.fix_tipi = fix.clamp(1, 3);
        CumleSonucu::Gecerli
    } else {
        CumleSonucu::Gecersiz
    }
}

fn vtg_isle(alan: &[&str], durum: &mut NmeaDurum) -> CumleSonucu {
    // $GNVTG,course,T,,M,speed_knots,N,speed_kmh,K,...
    if alan.len() < 8 {
        return CumleSonucu::Gecersiz;
    }

    let mut degisti = false;
    if let Some(yon) = yonelim_1e5(alan[1]) {
        durum.yonelim_1e5_derece = yon;
        degisti = true;
    }
    if let Some(hiz) = kmh_mm_s(alan[7]).or_else(|| knot_mm_s(alan[5])) {
        durum.hiz_mm_s = hiz;
        degisti = true;
    }

    if degisti {
        CumleSonucu::Yayinla
    } else {
        CumleSonucu::Gecerli
    }
}

fn nmea_isle(satir: &str, durum: &mut NmeaDurum) -> CumleSonucu {
    let Some(alan) = nmea_alanlari(satir) else {
        return CumleSonucu::Gecersiz;
    };
    let Some(tur) = alan.first().copied() else {
        return CumleSonucu::Gecersiz;
    };

    // Standart NMEA konuşmacıları: GP=GPS, GN=çoklu GNSS, GL=GLONASS,
    // GA=Galileo, GB/BD=BeiDou. Yanlış baudda oluşabilecek rastgele `$...`
    // parçalarının bağlantı bulundu sanılmasını engeller.
    let standart_konusmaci = tur.len() == 5
        && matches!(
            tur.get(..2),
            Some("GP") | Some("GN") | Some("GL") | Some("GA") | Some("GB") | Some("BD")
        );
    if !standart_konusmaci {
        return CumleSonucu::Gecersiz;
    }

    if tur.ends_with("GGA") {
        gga_isle(&alan, durum)
    } else if tur.ends_with("RMC") {
        rmc_isle(&alan, durum)
    } else if tur.ends_with("GSA") {
        gsa_isle(&alan, durum)
    } else if tur.ends_with("VTG") {
        vtg_isle(&alan, durum)
    } else {
        // GSV/GLL/TXT gibi tanınan ama kullanılmayan NMEA cümleleri de USB ve
        // baudrate'in doğru olduğuna kanıttır.
        CumleSonucu::Gecerli
    }
}

fn gps_paketi(durum: &NmeaDurum) -> GpsVeri {
    GpsVeri {
        algi_boyut: durum.fix_tipi,
        uydu_sayi: durum.uydu_sayisi,
        boylam: durum.boylam.unwrap_or(0),
        enlem: durum.enlem.unwrap_or(0),
        yukseklik_mm: durum.yukseklik_mm,
        hiz: durum.hiz_mm_s,
        yonelim: durum.yonelim_1e5_derece,
        zaman_ms: simdiki_zaman_ms(),
    }
}

fn baud_adaylari(istenen: u32) -> Vec<u32> {
    let mut sonuc = Vec::with_capacity(6);
    for baud in [istenen, 9_600, 115_200, 38_400, 57_600, 19_200, 4_800] {
        if !sonuc.contains(&baud) {
            sonuc.push(baud);
        }
    }
    sonuc
}

/// NEO-M8N'i Pico/STM olmadan doğrudan Linux USB seri portundan okur.
///
/// Beklenen veri: standart NMEA satırları (`$GNGGA`, `$GNRMC`, `$GNGSA`,
/// `$GNVTG`; `$GP...` konuşmacı kimliği de kabul edilir).
///
/// `baud_rate` ilk denenen hızdır. Geçerli NMEA bulunamazsa yaygın GPS
/// hızları otomatik denenir. Başarılı hız bağlantı koptuğunda ilk sıraya alınır.
pub async fn gps_task(port_adi: String, baud_rate: u32, tx: watch::Sender<GpsVeri>) {
    let mut tercih_edilen_baud = baud_rate;

    loop {
        let mut baglanti_kuruldu = false;

        for aktif_baud in baud_adaylari(tercih_edilen_baud) {
            println!(
                "GPS USB/NMEA portu açılmaya çalışılıyor: {} @ {} baud",
                port_adi, aktif_baud
            );

            let usb_port = match tokio_serial::new(&port_adi, aktif_baud).open_native_async() {
                Ok(port) => port,
                Err(e) => {
                    eprintln!("GPS portu açılamadı: {e}");
                    break;
                }
            };

            let mut reader = BufReader::new(usb_port);
            let mut satir = String::with_capacity(160);
            let mut durum = NmeaDurum::default();
            let ilk_veri_son_an = Instant::now() + ILK_NMEA_TIMEOUT;
            let mut gecerli_nmea_goruldu = false;
            let mut gecerli_cumle: u64 = 0;
            let mut gecersiz_cumle: u64 = 0;
            let mut yayin_sayaci: u64 = 0;

            'baglanti: loop {
                satir.clear();

                let okuma_timeout = if gecerli_nmea_goruldu {
                    NMEA_SESSIZ_TIMEOUT
                } else {
                    let kalan = ilk_veri_son_an.saturating_duration_since(Instant::now());
                    if kalan.is_zero() {
                        eprintln!(
                            "[GPS NMEA YOK] {} @ {} baud hızında geçerli NMEA bulunamadı.",
                            port_adi, aktif_baud
                        );
                        break 'baglanti;
                    }
                    kalan.min(Duration::from_secs(1))
                };

                match timeout(okuma_timeout, reader.read_line(&mut satir)).await {
                    Ok(Ok(0)) => {
                        eprintln!("GPS USB bağlantısı kapandı: {}", port_adi);
                        break 'baglanti;
                    }
                    Ok(Ok(_)) => {}
                    Ok(Err(e)) => {
                        eprintln!("GPS USB okuma hatası: {e}");
                        break 'baglanti;
                    }
                    Err(_) if gecerli_nmea_goruldu => {
                        eprintln!(
                            "[GPS USB SESSIZ] {} saniyedir NMEA yok; port yeniden açılacak.",
                            NMEA_SESSIZ_TIMEOUT.as_secs()
                        );
                        break 'baglanti;
                    }
                    Err(_) => continue,
                }

                let sonuc = nmea_isle(&satir, &mut durum);
                match sonuc {
                    CumleSonucu::Gecersiz => {
                        gecersiz_cumle = gecersiz_cumle.wrapping_add(1);
                        if gecersiz_cumle == 1 || gecersiz_cumle % 50 == 0 {
                            let gorunen = satir.trim().chars().take(100).collect::<String>();
                            eprintln!(
                                "[GPS NMEA HATA] baud={} hata={} satir={:?}",
                                aktif_baud, gecersiz_cumle, gorunen
                            );
                        }
                    }
                    CumleSonucu::Gecerli | CumleSonucu::Yayinla => {
                        if !gecerli_nmea_goruldu {
                            println!(
                                "GPS doğrudan USB/NMEA bağlantısı kuruldu: {} @ {} baud",
                                port_adi, aktif_baud
                            );
                            gecerli_nmea_goruldu = true;
                            baglanti_kuruldu = true;
                            tercih_edilen_baud = aktif_baud;
                        }
                        gecerli_cumle = gecerli_cumle.wrapping_add(1);
                    }
                }

                if sonuc == CumleSonucu::Yayinla {
                    let paket = gps_paketi(&durum);
                    yayin_sayaci = yayin_sayaci.wrapping_add(1);

                    if yayin_sayaci == 1 || yayin_sayaci % 5 == 0 {
                        println!(
                            "[GPS USB OK] port={} baud={} paket={} nmea={} hata={} fix={} uydu={} enlem={:.7} boylam={:.7} alt_mm={} hiz_mm_s={} yon_deg={:.2}",
                            port_adi,
                            aktif_baud,
                            yayin_sayaci,
                            gecerli_cumle,
                            gecersiz_cumle,
                            paket.algi_boyut,
                            paket.uydu_sayi,
                            paket.enlem as f64 / 10_000_000.0,
                            paket.boylam as f64 / 10_000_000.0,
                            paket.yukseklik_mm,
                            paket.hiz,
                            paket.yonelim as f64 / 100_000.0,
                        );
                    }

                    if tx.send(paket).is_err() {
                        println!("GPS alıcısı kapandı.");
                        return;
                    }
                }
            }

            if baglanti_kuruldu {
                // Doğru baud bulundu fakat bağlantı koptu; diğer baudları gereksiz yere
                // dolaşmadan aynı hızı yeniden dene.
                break;
            }
        }

        sleep(YENIDEN_DENE).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gga_koordinatlarini_cevirir() {
        let mut durum = NmeaDurum::default();
        let sonuc = nmea_isle(
            "$GNGGA,123519,4101.2345,N,02858.7654,E,1,10,0.8,12.3,M,0.0,M,,",
            &mut durum,
        );
        assert_eq!(sonuc, CumleSonucu::Yayinla);
        assert_eq!(durum.fix_tipi, 2);
        assert_eq!(durum.uydu_sayisi, 10);
        assert_eq!(durum.yukseklik_mm, 12_300);
        assert_eq!(durum.enlem, Some(410_205_750));
        assert_eq!(durum.boylam, Some(289_794_233));
    }

    #[test]
    fn rmc_hiz_ve_yonu_cevirir() {
        let mut durum = NmeaDurum::default();
        let sonuc = nmea_isle(
            "$GNRMC,123520,A,4101.2345,N,02858.7654,E,10.0,90.0,180726,,,A",
            &mut durum,
        );
        assert_eq!(sonuc, CumleSonucu::Yayinla);
        assert_eq!(durum.hiz_mm_s, 5_144);
        assert_eq!(durum.yonelim_1e5_derece, 9_000_000);
    }

    #[test]
    fn gsa_fix_tipini_okur() {
        let mut durum = NmeaDurum::default();
        assert_eq!(
            nmea_isle("$GNGSA,A,3,,,,,,,,,,,,,1.2,0.8,0.9", &mut durum),
            CumleSonucu::Gecerli
        );
        assert_eq!(durum.fix_tipi, 3);
    }
}
