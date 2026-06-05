use clap::{Parser, ValueEnum};
use ctrlc;
use rppal::{
    gpio::Gpio,
    spi::{Bus, Mode, SlaveSelect, Spi},
};
use rppal_mfrc522::{Mfrc522, RxGain};
use simple_cancelation_token::CancelationToken;
use std::{error::Error, time::Duration};
use std::{thread, time};

fn main() {
    if let Err(error) = RppalMfrc522Tool::new().run(std::env::args_os()) {
        eprintln!("{}", error);
        std::process::exit(1);
    }
}

/// A tool for testing an MFRC522 device connected over an available SPI interface on the command line
pub struct RppalMfrc522Tool {}

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

    #[arg(long = "clock", short = 'c', default_value = "1_000_000")]
    clock_speed: u32,
}

impl RppalMfrc522Tool {
    /// Create a new [RppalMfrc522Tool]
    pub fn new() -> RppalMfrc522Tool {
        RppalMfrc522Tool {}
    }

    /// Run the tool with the given command line arguments
    pub fn run(
        self: &mut Self,
        args: impl IntoIterator<Item = std::ffi::OsString>,
    ) -> Result<(), Box<dyn Error>> {
        let cli = match Cli::try_parse_from(args) {
            Ok(m) => m,
            Err(err) => {
                eprintln!("{}", err.to_string());
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

        let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, cli.clock_speed, Mode::Mode0)?;
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
                Ok(uid) => print!(
                    "UID: {:#010x}         \nATQA: {:#06x}  SAK: {:#02x}   \n",
                    uid.to_u32(),
                    uid.atqa.to_u16(),
                    uid.sak.to_u8()
                ),
                Err(_) => print!("No card detected     \n                          \n"),
            };

            thread::sleep(time::Duration::from_millis(250));

            if token.is_canceled() {
                break;
            }

            print!("{}", termion::cursor::Up(2));
        }

        print!("\n\n");
        reset_pin.set_low();

        Ok(())
    }
}
