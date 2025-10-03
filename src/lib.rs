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
//! use rppal::{gpio::Gpio, spi::{Spi, SlaveSelect, Bus, Mode}};
//! use rppal_mfrc522::Mfrc522;
//! use std::time::Duration;
//! use std::thread;
//!
//! const RESET_PIN_BCM: u8 = 22;
//! let mut reset_pin = Gpio::new().unwrap().get(RESET_PIN_BCM).unwrap().into_output();
//! let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
//! let mut mfrc522 = Mfrc522::new(&mut spi);
//!
//! // Toggle the reset pin to reset the chip
//! reset_pin.set_low();
//! thread::sleep(Duration::from_millis(50));
//! reset_pin.set_high();
//! thread::sleep(Duration::from_millis(50));
//!
//! //
//! mfrc522.reset().unwrap();
//!
//! // Getting the manufacturer version of the MFRC522 chip is a quick way to check that
//! // communication is working
//! println!("{:#04x}", mfrc522.version().unwrap());
//!
//! match mfrc522.uid(Duration::from_millis(250)) {
//!     Ok(uid) => {
//!         println!("{:#010x}", uid.to_u32());
//!     },
//!     Err(err) => {
//!         println!("Error reading UID: {}", err);
//!     }
//! }
//!
//! ```

#![deny(unsafe_code, missing_docs)]

mod mfrc522;
pub mod picc;
pub mod register;

pub use crate::mfrc522::Mfrc522;
pub use crate::picc::*;
pub use crate::register::*;
