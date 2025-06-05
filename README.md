# RPPAL MFRC522

[![coverage](https://shields.io/endpoint?url=https://raw.githubusercontent.com/jlyonsmith/rppal_mfrc522/main/coverage.json)](https://github.com/jlyonsmith/rppal_mfrc522/blob/main/coverage.json)
[![Crates.io](https://img.shields.io/crates/v/rppal_mfrc522.svg)](https://crates.io/crates/rppal_mfrc522)
[![Docs.rs](https://docs.rs/rppal_mfrc522/badge.svg)](https://docs.rs/rppal_mfrc522)

## Summary

This is a crate that for controlling MFRC522 based RFID boards.  It uses a lot of code and ideas from the excellent [`mfrc522`][2] crate but is specifically designed to work with [`rppal`][1] over an SPI interface, and is only focused on reading ID's and collision detection.

## Details

What works:

- [ ] SPI communication with an MFRC522 board
- [ ] Internal state management

[1]: https://crates.io/crates/rppal
[2]: https://crates.io/crates/mfrc522