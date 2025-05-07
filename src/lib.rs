mod log_macros;
mod mfrc522;

use clap::Parser;
use core::fmt::Arguments;
use std::error::Error;
use mfrc522::Mfrc522;
use rppal::{spi::{Mode, Bus, SlaveSelect, Spi}, gpio::Gpio};
use std::{thread, time};

pub trait RppalMfrc522Log {
    fn output(self: &Self, args: Arguments);
    fn warning(self: &Self, args: Arguments);
    fn error(self: &Self, args: Arguments);
}

pub struct RppalMfrc522Tool<'a> {
    log: &'a dyn RppalMfrc522Log,
}

#[derive(Parser)]
#[clap(version, about, long_about = None)]
struct Cli {
    /// Disable colors in output
    #[arg(long = "no-color", short = 'n', env = "NO_CLI_COLOR")]
    no_color: bool,
}

const GPIO_SELECTOR_A: u8 = 17;
const GPIO_SELECTOR_B: u8 = 27;
const GPIO_SELECTOR_C: u8 = 22;
const GPIO_RESET: u8 = 20;

impl<'a> RppalMfrc522Tool<'a> {
    pub fn new(log: &'a dyn RppalMfrc522Log) -> RppalMfrc522Tool<'a> {
        RppalMfrc522Tool { log }
    }

    pub fn run(
        self: &mut Self,
        args: impl IntoIterator<Item = std::ffi::OsString>,
    ) -> Result<(), Box<dyn Error>> {
        let _cli = match Cli::try_parse_from(args) {
            Ok(m) => m,
            Err(err) => {
                output!(self.log, "{}", err.to_string());
                return Ok(());
            }
        };

        let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 8_000_000, Mode::Mode0)?;

        let mut pin_a = Gpio::new()?.get(GPIO_SELECTOR_A)?.into_output();
        let mut pin_b = Gpio::new()?.get(GPIO_SELECTOR_B)?.into_output();
        let mut pin_c = Gpio::new()?.get(GPIO_SELECTOR_C)?.into_output();
        let mut reset_pin = Gpio::new()?.get(GPIO_RESET)?.into_output();

        pin_a.set_low();
        pin_b.set_low();
        pin_c.set_low();
        reset_pin.set_low();
        thread::sleep(time::Duration::from_millis(10));
        reset_pin.set_high();

        let mut mfrc522 = Mfrc522::new(&mut spi);

        mfrc522.reset()?;

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
