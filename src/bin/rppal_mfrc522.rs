use clap::{Parser, ValueEnum};
use core::fmt::Arguments;
use ctrlc;
use rppal::{
    gpio::Gpio,
    spi::{Bus, Mode, SlaveSelect, Spi},
};
use rppal_mfrc522::{Mfrc522, RxGain};
use simple_cancelation_token::CancelationToken;
use std::{error::Error, time::Duration};
use std::{thread, time};
use termion::color;

/// Write formatted output to the `output` method of a logger
#[macro_export]
macro_rules! output {
  ($log: expr, $fmt: expr) => {
    $log.output(format_args!($fmt))
  };
  ($log: expr, $fmt: expr, $($args: tt)+) => {
    $log.output(format_args!($fmt, $($args)+))
  };
}

/// Write formatted output to the `warning` method of a logger
#[macro_export]
macro_rules! warning {
  ($log: expr, $fmt: expr) => {
    $log.warning(format_args!($fmt))
  };
  ($log: expr, $fmt: expr, $($args: tt)+) => {
    $log.warning(format_args!($fmt, $($args)+))
  };
}

/// Write formatted output to the `error` method of a logger
#[macro_export]
macro_rules! error {
  ($log: expr, $fmt: expr) => {
    $log.error(format_args!($fmt))
  };
  ($log: expr, $fmt: expr, $($args: tt)+) => {
    $log.error(format_args!($fmt, $($args)+))
  };
}

struct RppalMfrc522Logger;

impl RppalMfrc522Logger {
    fn new() -> RppalMfrc522Logger {
        RppalMfrc522Logger {}
    }
}

/// Logging trait for the [RppalMfrc522Tool].
pub trait RppalMfrc522Log {
    /// Normal program output, usually goes to `stdout``
    fn output(self: &Self, args: Arguments);
    /// Warning output, normally goes to `stderr``
    fn warning(self: &Self, args: Arguments);
    /// Error output, normally goes to `stderr`
    fn error(self: &Self, args: Arguments);
}

impl RppalMfrc522Log for RppalMfrc522Logger {
    fn output(self: &Self, args: Arguments) {
        println!("{}", args);
    }
    fn warning(self: &Self, args: Arguments) {
        eprintln!("{}warning: {}", color::Fg(color::Yellow), args);
    }
    fn error(self: &Self, args: Arguments) {
        eprintln!("{}error: {}", color::Fg(color::Red), args);
    }
}

fn main() {
    let logger = RppalMfrc522Logger::new();

    if let Err(error) = RppalMfrc522Tool::new(&logger).run(std::env::args_os()) {
        error!(logger, "{}", error);
        std::process::exit(1);
    }
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
        mfrc522.set_antenna_gain(RxGain::DB48)?;

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
