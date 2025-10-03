# RPPAL MFRC522

[![coverage](https://shields.io/endpoint?url=https://raw.githubusercontent.com/jlyonsmith/rppal_mfrc522/main/coverage.json)](https://github.com/jlyonsmith/rppal_mfrc522/blob/main/coverage.json)
[![Crates.io](https://img.shields.io/crates/v/rppal_mfrc522.svg)](https://crates.io/crates/rppal_mfrc522)
[![Docs.rs](https://docs.rs/rppal_mfrc522/badge.svg)](https://docs.rs/rppal_mfrc522)

## Details

This is a crate for controlling MFRC522 based RFID boards. It is specifically designed to work with [`rppal`][1] over an SPI interface. My goal for this library was to be able to easily, reliably and synchronously read the 4-byte UID from PICC cards, key fobs and stickers when building escape room puzzles.

I was motivated to build this crate because I sadly could not get the [`mfrc522`][2] crate to work.  Specifically the `select()` call seems broken.  I was able to get the very old Python code in [`mfrc522-python`][3] working.  So I reproduced the Python code in Rust, then merged in some of the nicer design elements of the [`mfrc522`][2] crate.

In the process I tried to fully document what the code is doing in comments to remove a lot of the mystery about the MFRC522 chip configuration and interfacing.  Wherever possible I linked to the MFRC522 datasheet section so you can read about it yourself.  I also included the PDF's for the MFRC522 and PICC cards in the repository.

What is implemented:

- SPI communication with an MFRC522 board using `rppal::spi::Spi`.
- `uid()` function to read card identifiers
- `version()` function to read the board manufacturing identifier
- Proper division of timeouts across internal calls

What does not work, because I don't currently need it:

- Collision detection of multiple cards
- Reading/writing the PICC card data

## Command Line Tool

The crate includes a command line program for testing called `rppal-mfrc522`. Run it with the `--help` argument to see options. Comes in really handy for debugging and testing.

## Development

You can use the [`cross`][4] crate for cross compilation to the `aarch64-unknown-linux-gnu` Raspberry Pi 64-bit platform. To develop I use a RPi 5 with 16GB RAM and an external USB NVMe drive.  I use the [Zed](https://zed.dev/) editor and edit the files remotely from a Macbook.

---

[1]: https://crates.io/crates/rppal
[2]: https://crates.io/crates/mfrc522
[3]: https://pypi.org/project/mfrc522-python
[4]: https://crates.io/crates/cross
