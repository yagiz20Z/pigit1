use std::io::{Error, ErrorKind};
use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::time::timeout;
use tokio_serial::{SerialPortBuilderExt, SerialStream};

const STM_ACK: u8 = 0xAC;

pub struct MotorKontrol {
    port: SerialStream,
    paket_sayaci: u64,
    ack_sayaci: u64,
}

impl MotorKontrol {
    pub fn new_port(port_name: &str, baud_rate: u32) -> tokio_serial::Result<Self> {
        let port = tokio_serial::new(port_name, baud_rate).open_native_async()?;

        Ok(Self {
            port,
            paket_sayaci: 0,
            ack_sayaci: 0,
        })
    }

    fn calc_crc8(data: &[u8]) -> u8 {
        let mut crc = 0x00;

        for &byte in data {
            crc ^= byte;

            for _ in 0..8 {
                if (crc & 0x80) != 0 {
                    crc = crc.wrapping_shl(1) ^ 0x8C;
                } else {
                    crc = crc.wrapping_shl(1);
                }
            }
        }

        crc
    }

    pub async fn set_speeds(
        &mut self,
        iskeleon: u16,
        iskelearka: u16,
        sancakon: u16,
        sancakarka: u16,
    ) -> std::io::Result<()> {
        let io = iskeleon.clamp(0, 1000);
        let ia = iskelearka.clamp(0, 1000);
        let so = sancakon.clamp(0, 1000);
        let sa = sancakarka.clamp(0, 1000);

        let mut bucket = [0u8; 11];
        bucket[0] = 0xAA;
        bucket[1] = 0x55;
        bucket[2..4].copy_from_slice(&io.to_be_bytes());
        bucket[4..6].copy_from_slice(&ia.to_be_bytes());
        bucket[6..8].copy_from_slice(&so.to_be_bytes());
        bucket[8..10].copy_from_slice(&sa.to_be_bytes());
        bucket[10] = Self::calc_crc8(&bucket[..10]);

        self.port.write_all(&bucket).await?;
        self.port.flush().await?;
        self.paket_sayaci = self.paket_sayaci.wrapping_add(1);

        // STM geçerli paketi aldığında 0xAC gönderir. ACK gelmese bile
        // komut gönderimi durdurulmaz; yalnızca bağlantı teşhisi yapılır.
        let mut ack = [0u8; 1];
        match timeout(Duration::from_millis(20), self.port.read_exact(&mut ack)).await {
            Ok(Ok(_)) if ack[0] == STM_ACK => {
                self.ack_sayaci = self.ack_sayaci.wrapping_add(1);
            }
            Ok(Ok(_)) => {
                if self.paket_sayaci % 20 == 1 {
                    eprintln!("STM'den beklenmeyen cevap geldi: 0x{:02X}", ack[0]);
                }
            }
            Ok(Err(e)) => return Err(e),
            Err(_) => {
                if self.paket_sayaci % 20 == 1 {
                    eprintln!(
                        "STM ACK gelmedi. TX gönderildi; STM TX -> dönüştürücü RX hattını kontrol et."
                    );
                }
            }
        }

        if self.paket_sayaci % 20 == 1 {
            println!(
                "STM TX paket={} ACK={} PWM=[{}, {}, {}, {}] RAW={:02X?}",
                self.paket_sayaci, self.ack_sayaci, io, ia, so, sa, bucket,
            );
        }

        Ok(())
    }

    pub async fn sifirla(&mut self) -> std::io::Result<()> {
        self.set_speeds(0, 0, 0, 0).await
    }

    #[allow(dead_code)]
    pub async fn ack_bekle(&mut self) -> std::io::Result<()> {
        let mut ack = [0u8; 1];

        match timeout(Duration::from_secs(1), self.port.read_exact(&mut ack)).await {
            Ok(Ok(_)) if ack[0] == STM_ACK => Ok(()),
            Ok(Ok(_)) => Err(Error::new(
                ErrorKind::InvalidData,
                format!("Beklenmeyen STM cevabı: 0x{:02X}", ack[0]),
            )),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(Error::new(ErrorKind::TimedOut, "STM ACK zaman aşımı")),
        }
    }
}
