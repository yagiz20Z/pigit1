use std::env;
use std::error::Error;
use std::io::{Error as IoError, ErrorKind};
use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::time::{sleep, timeout};
use tokio_serial::SerialPortBuilderExt;

const DEFAULT_MOTOR_PORT: &str = "/dev/ttyUSB1";
const MOTOR_BAUD_RATE: u32 = 115_200;
const STM_ACK: u8 = 0xAC;

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

fn pwm_arg(args: &[String], index: usize) -> u16 {
    args.get(index)
        .and_then(|deger| deger.parse::<u16>().ok())
        .unwrap_or(0)
        .clamp(0, 1000)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    let port_adi = args.get(1).map(String::as_str).unwrap_or(DEFAULT_MOTOR_PORT);

    // Varsayılan 0 değerleri ESC'lere 1000 us gönderir.
    let pwms = [
        pwm_arg(&args, 2),
        pwm_arg(&args, 3),
        pwm_arg(&args, 4),
        pwm_arg(&args, 5),
    ];

    let mut paket = [0u8; 11];
    paket[0] = 0xAA;
    paket[1] = 0x55;

    for (index, pwm) in pwms.iter().enumerate() {
        paket[2 + index * 2..4 + index * 2].copy_from_slice(&pwm.to_be_bytes());
    }

    paket[10] = calc_crc8(&paket[..10]);

    println!("STM portu : {}", port_adi);
    println!("PWM       : {:?}", pwms);
    println!("Paket     : {:02X?}", paket);

    let mut port = tokio_serial::new(port_adi, MOTOR_BAUD_RATE).open_native_async()?;

    for deneme in 1..=5 {
        port.write_all(&paket).await?;
        port.flush().await?;
        println!("Deneme {} TX: {:02X?}", deneme, paket);

        let mut ack = [0u8; 1];

        match timeout(Duration::from_secs(1), port.read_exact(&mut ack)).await {
            Ok(Ok(_)) if ack[0] == STM_ACK => {
                println!("Deneme {} RX ACK: 0xAC - BAŞARILI", deneme);
            }
            Ok(Ok(_)) => {
                return Err(IoError::new(
                    ErrorKind::InvalidData,
                    format!("STM beklenmeyen cevap verdi: 0x{:02X}", ack[0]),
                )
                .into());
            }
            Ok(Err(e)) => return Err(e.into()),
            Err(_) => {
                return Err(IoError::new(
                    ErrorKind::TimedOut,
                    "STM'den 1 saniye içinde ACK gelmedi",
                )
                .into());
            }
        }

        sleep(Duration::from_millis(250)).await;
    }

    println!("UART testi tamamlandı: 5/5 paket STM tarafından alındı.");
    Ok(())
}
