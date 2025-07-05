//! Driver library for interfacing with the MFRC522 contact-less communication IC.
//!
//! The MFRC522 is a *Proximity Coupling Device* (PCD) and communicates with a
//! *Proximity Integrated Circuit Card* (PICC). The main purpose of the MFRC522 is to give the
//! connected device the ability to read and write data from/to the card.
//!
//! This library does not concern itself with reading or writing data to a PICC.  It is only concerned with
//! retrieving the identification number (UID) from a PICC reliably.  Currently, the library only
//! works for 4 byte UID's, as those are the most common cards on [Amazon](https://amazon.com).)
//!
//! The MFRC522 supports 3 communication interfaces: SPI, I2C, and UART.  _Only SPI communication
//! is implemented in this crate._
//!
//! This crate also includes a command line tool for your MFRC522 card, `rppal-mfrc522`.
//! Install this crate with `cargo install rppal-mfrc522`to get access to the tool.
//! Use the `--help` command line argument to get assistance.  The tool also allows you to
//! enable/disable specific pins and the reset pin for the card and read UID's from cards.
//!
//! # Quickstart
//! ```rust
//! use rppal::spi::{Spi, SlaveSelect, Bus, Mode};
//! use rppal_mfrc522::Mfrc522;
//!
//! const RESET_PIN_BCM = 22;
//! let mut reset_pin = Gpio::new()?.get(RESET_PIN_BCM)?.into_output();
//! let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0)?;
//! let mut mfrc522 = Mfrc522::new(&mut spi);
//!
//! // Toggle the reset pin to reset the chip
//! reset_pin.set_low();
//! thread::sleep(time::Duration::from_millis(50));
//! reset_pin.set_high();
//! thread::sleep(time::Duration::from_millis(50));
//!
//! //
//! mfrc522.reset().unwrap();
//!
//! // Getting the manufacturer version of the MFRC522 chip is a quick way to check that
//! // communication is working
//! println!("{:#04x}", mfrc522.version().unwrap());
//!
//! let uid = mfrc522.uid(Duration::from_millis(250).unwrap());
//!
//! println!("{:#010x}", uid.to_u32());
//! ```

#![deny(unsafe_code, missing_docs)]

mod log_macros;
mod mfrc522;
mod picc;
mod register;

pub use crate::mfrc522::Mfrc522;
use clap::{Parser, ValueEnum};
use core::fmt::Arguments;
use ctrlc;
use rppal::{
    gpio::Gpio,
    spi::{Bus, Mode, SlaveSelect, Spi},
};
use simple_cancelation_token::CancelationToken;
use std::{error::Error, time::Duration};
use std::{thread, time};

/// Logging trait for the [RppalMfrc522Tool].
pub trait RppalMfrc522Log {
    /// Normal program output, usually goes to `stdout``
    fn output(self: &Self, args: Arguments);
    /// Warning output, normally goes to `stderr``
    fn warning(self: &Self, args: Arguments);
    /// Error output, normally goes to `stderr`
    fn error(self: &Self, args: Arguments);
}

/// A tool for testing an MFRC522 device connected over an available SPI interface on the command line
pub struct RppalMfrc522Tool<'a> {
    log: &'a dyn RppalMfrc522Log,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
#[repr(u8)]
enum BcmPin {
    Pin1 = 1,
    Pin2,
    Pin3,
    Pin4,
    Pin5,
    Pin6,
    Pin7,
    Pin8,
    Pin9,
    Pin10,
    Pin11,
    Pin12,
    Pin13,
    Pin14,
    Pin15,
    Pin16,
    Pin17,
    Pin18,
    Pin19,
    Pin20,
    Pin21,
    Pin22,
    Pin23,
    Pin24,
    Pin25,
    Pin26,
    Pin27,
}

#[derive(Parser)]
#[clap(version, about, long_about = None)]
struct Cli {
    /// Disable colors in output
    #[arg(long = "no-color", short = 'n', env = "NO_CLI_COLOR")]
    no_color: bool,
    #[arg(long = "high", short = '1')]
    high_pins: Vec<BcmPin>,
    #[arg(long = "low", short = '0')]
    low_pins: Vec<BcmPin>,
    #[arg(long = "reset", short = 'r')]
    reset_pin: BcmPin,
}

impl<'a> RppalMfrc522Tool<'a> {
    /// Create a new [RppalMfrc522Tool] with a logger
    pub fn new(log: &'a dyn RppalMfrc522Log) -> RppalMfrc522Tool<'a> {
        RppalMfrc522Tool { log }
    }

    /// Run the tool with the given command line arguments
    pub fn run(
        self: &mut Self,
        args: impl IntoIterator<Item = std::ffi::OsString>,
    ) -> Result<(), Box<dyn Error>> {
        let cli = match Cli::try_parse_from(args) {
            Ok(m) => m,
            Err(err) => {
                output!(self.log, "{}", err.to_string());
                return Ok(());
            }
        };

        let mut reset_pin = Gpio::new()?.get(cli.reset_pin as u8)?.into_output();

        reset_pin.set_reset_on_drop(false);

        reset_pin.set_low();
        thread::sleep(time::Duration::from_millis(100));

        for bcm_pin in cli.low_pins {
            let mut pin = Gpio::new()?.get(bcm_pin as u8)?.into_output();

            pin.set_reset_on_drop(false);
            pin.set_low();
        }

        for bcm_pin in cli.high_pins {
            let mut pin = Gpio::new()?.get(bcm_pin as u8)?.into_output();

            pin.set_reset_on_drop(false);
            pin.set_high();
        }

        reset_pin.set_high();
        thread::sleep(time::Duration::from_millis(50));

        let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0)?;
        let mut mfrc522 = Mfrc522::new(&mut spi);

        mfrc522.reset()?;
        mfrc522.set_antenna_gain(register::RxGain::DB48)?;

        println!("Reader Mfg Version: {:#04x}", mfrc522.version()?);

        let token = CancelationToken::new();
        let token_clone = token.clone();

        ctrlc::set_handler(move || {
            eprintln!(" received, stopping...");
            token_clone.cancel();
            ()
        })?;

        loop {
            match mfrc522.uid(Duration::from_millis(250)) {
                Ok(uid) => println!("{:#010x}        ", uid.to_u32()),
                Err(_) => println!("No card detected"),
            };

            thread::sleep(time::Duration::from_millis(250));

            if token.is_canceled() {
                break;
            }

            print!("{}", termion::cursor::Up(1));
        }

        reset_pin.set_low();

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_test() {
        struct TestLogger;

        impl TestLogger {
            fn new() -> TestLogger {
                TestLogger {}
            }
        }

        impl RppalMfrc522Log for TestLogger {
            fn output(self: &Self, _args: Arguments) {}
            fn warning(self: &Self, _args: Arguments) {}
            fn error(self: &Self, _args: Arguments) {}
        }

        let logger = TestLogger::new();
        let mut tool = RppalMfrc522Tool::new(&logger);
        let args: Vec<std::ffi::OsString> = vec!["".into(), "--help".into()];

        tool.run(args).unwrap();
    }
}
