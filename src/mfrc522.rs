use crate::{picc::*, register::*};
use rppal::spi::{self, Segment, Spi};
use std::thread;
use std::time::{Duration, Instant};
use thiserror::Error;

/// The unique identifier returned by a PICC
pub enum Uid {
    /// Single sized UID, 4 bytes long
    Single(GenericUid<4>),
    /// Double sized UID, 7 bytes long
    Double(GenericUid<7>),
    /// Triple sized UID, 10 bytes long
    Triple(GenericUid<10>),
}

impl Uid {
    /// Get the UID as a byte slice
    pub fn as_bytes(&self) -> &[u8] {
        match self {
            Uid::Single(u) => u.as_bytes(),
            Uid::Double(u) => u.as_bytes(),
            Uid::Triple(u) => u.as_bytes(),
        }
    }

    /// Get the type of the PICC that returned the UID
    pub fn get_type(&self) -> PiccType {
        match self {
            Uid::Single(u) => u.get_type(),
            Uid::Double(u) => u.get_type(),
            Uid::Triple(u) => u.get_type(),
        }
    }
}

/// An identifier that is generic over the size.
///
/// This is used internally in the [Uid] enum.
pub struct GenericUid<const T: usize>
where
    [u8; T]: Sized,
{
    /// The UID can have 4, 7 or 10 bytes.
    bytes: [u8; T],
    /// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
    sak: Sak,
}

impl<const T: usize> GenericUid<T> {
    /// Create a GenericUid from a byte array and a SAK byte.
    ///
    /// You shouldn't typically need to use this function as an end-user.
    /// Instead use the [Uid] returned by the [select](Mfrc522::select) function.
    pub fn new(bytes: [u8; T], sak_byte: u8) -> Self {
        Self {
            bytes,
            sak: Sak::from(sak_byte),
        }
    }

    /// Get the underlying bytes of the UID
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Is the PICC compliant?
    pub fn is_compliant(&self) -> bool {
        self.sak.is_compliant()
    }

    /// Get the type of the PICC
    pub fn get_type(&self) -> PiccType {
        self.sak.get_type()
    }
}

/// Answer To reQuest type A
pub struct AtqA {
    bytes: [u8; 2],
}

// MFRC522 chip frequency
const MFRC_FREQ: f64 = 13.56e6;
// See Section 9.3.3.10 - this is the desired MFRC522 countdown timer tick frequency
const TICK_FREQ: f64 = 1999.7;
// The inverse calculation to get the desired prescale value
static PRESCALE: f64 = (MFRC_FREQ - TICK_FREQ) / (2.0 * TICK_FREQ);
// The desired MFRC522 countdown timer interval
const TIMER_INTERVAL: f64 = 0.015;
// Our safety time when waiting blocking and waiting for the card
const SAFETY_TIMER_INTERVAL: f64 = 0.025;
// Max size of FIFO buffer
const MAX_FIFO_BYTES: usize = 64;
// The length of a standard MIFARE card ID in bytes
const CARD_ID_BYTE_LENGTH: usize = 4;

#[derive(Debug, Error)]
pub enum Mfrc522Error {
    #[error("error reading card")]
    Transceive(u8),
    #[error("safety timeout reading card")]
    SafetyTimeout,
    #[error("Card not found")]
    CardNotFound,
    #[error("SPI error")]
    SpiError(#[from] spi::Error),
}

pub struct Mfrc522<'a> {
    spi: &'a mut Spi,
}

impl Mfrc522<'_> {
    pub fn new(spi: &mut Spi) -> Mfrc522 {
        Mfrc522 { spi }
    }

    /// Write a register from the SPI interface
    fn write(&mut self, reg: Register, data: u8) -> Result<(), Mfrc522Error> {
        // See Section 8.1.2.2 for details of writing to the Mifare registers over SPI

        // No zero needed to terminate the data when writing
        let write_buffer = [((reg as u8) << 1) & 0x7e, data];

        self.spi.write(&write_buffer)?;

        Ok(())
    }

    /// Write multiple bytes to a register on the SPI interface, keeping the Slave
    /// Select (SS) line active until all buffers have been transferred
    fn write_many(&mut self, reg: Register, buf: &[u8]) -> Result<(), Mfrc522Error> {
        let address_buf = [(reg as u8) << 1];
        let segments = [Segment::with_write(&address_buf), Segment::with_write(buf)];

        self.spi.transfer_segments(&segments)?;

        Ok(())
    }

    /// Read a register from the SPI interface
    fn read(&mut self, reg: Register) -> Result<u8, Mfrc522Error> {
        // See Section 8.1.2.1 for details of reading from the Mifare registers over SPI

        // The zero byte terminates the register addresses when reading.
        // You can read from more than one register in a single operation, but we don't
        let write_buffer = [(((reg as u8) << 1) & 0x7e) | 0x80, 0];
        let mut read_buffer = [0u8; 2];

        // Transfer will only receive as much as was sent
        self.spi.transfer(&mut read_buffer, &write_buffer)?;

        // The result is in the second byte, not the first
        Ok(read_buffer[1])
    }

    /// Read multilple bytes from a register on the SPI interface, keeping Slave Select line
    /// active until all transfers are complete.
    fn read_many<'b>(
        &mut self,
        reg: Register,
        buf: &'b mut [u8],
    ) -> Result<&'b [u8], Mfrc522Error> {
        // See Section 8.1.2.1 for why we have to initialize the read buffer like this
        // and Section 8.1.2.3 for why we shift the register address
        let address = ((reg as u8) << 1) | 0x80;
        // TODO @john: We could use stackalloc crate to avoid the head allocation here
        let mut tx_buf = buf.to_vec();
        for byte in tx_buf.iter_mut() {
            *byte = address;
        }
        if let Some(byte) = tx_buf.last_mut() {
            *byte = 0;
        }

        let address_buf = [address];
        let segments = [
            Segment::with_write(&address_buf),
            Segment::new(buf, &tx_buf),
        ];
        self.spi.transfer_segments(&segments)?;

        Ok(buf)
    }

    /// Read-modify-write a register on the SPI interface
    fn rmw<F>(&mut self, reg: Register, func: F) -> Result<(), Mfrc522Error>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read(reg)?;
        let new_value = func(value);
        self.write(reg, new_value)?;

        Ok(())
    }

    /// Reset the reader
    pub fn reset(&mut self) -> Result<(), Mfrc522Error> {
        // See Section 9.3.1.2 - soft reset the chip, setting all registers to defaults
        self.write(Register::CommandReg, Command::SoftReset.into())?;

        let prescale_bytes = (PRESCALE as u16).to_be_bytes();

        // See Section 9.3.3.10:
        // TAuto=1 - timer starts automatically at the end of the transmission in all communication modes at all speeds
        // TGated=0 - timer is not gated by pins MFIN or AUX1
        // TAutoRestart=0 - set IRQ bit instead of restarting timer
        // TPrescaler_Hi=hh - high 4-bits of prescaler value
        self.write(Register::TModeReg, 0x80 | (prescale_bytes[0] & 0xF))?;
        // TPreScalerLo=ll - low bits of prescaler value
        // e.g. 0xd3e = 3390, so f_timer = 13560000 / (2 * 3390 + 1) ~= 1999.7Hz.  1/1999.7 ~= 0.5ms per timer tick
        self.write(Register::TPrescalerReg, prescale_bytes[1])?;

        let timer_ticks = (TIMER_INTERVAL / (1.0 / TICK_FREQ)).ceil() as u16;
        let timer_tick_bytes = timer_ticks.to_be_bytes();

        // See Section 9.3.3.11 - timer reload value
        // e.g. with prescaler interval of 0.5ms, reload time with 30 gives 15ms timeout
        self.write(Register::TReloadRegLow, timer_tick_bytes[1])?;
        self.write(Register::TReloadRegHigh, timer_tick_bytes[0])?;

        // See Section 9.3.2.6 and https://ieeexplore.ieee.org/document/9786078
        // ForceASK100=1 - Force 100% ASK (Amplitude Shift Keying) modulation always
        self.write(Register::TxASKReg, 0b0100_0000)?;

        // See Section 9.3.2.2 - preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
        // MSBFirst=0 - do not calc CRC with MSB first
        // TxWaitRF=1 - transmitter can only bestarted if RF field is generated
        // PolMFin=0 - MFIN is active LOW
        // CRCPreset=11 - CRC preset value is 0xFFFF
        self.write(Register::ModeReg, 0b0011_1101)?;

        // See Section 9.3.2.5
        // Tx2RFEn=1 and Tx1RFEn=1 - output 13.56MHz carrier signal on TX1 and TX2
        self.rmw(Register::TxControlReg, |value| (value | 0x03))?; // Turn on the antenna
        Ok(())
    }

    pub fn get_version(&mut self) -> Result<u8, Mfrc522Error> {
        Ok(self.read(Register::VersionReg)?)
    }

    pub fn read_card_id(&mut self) -> Result<u64, Mfrc522Error> {
        // Section 9.3.1.14 - we want 7 bits of the last byte transmitted
        self.write(Register::BitFramingReg, 0x07)?;

        // Communicate with any nearby card and request it to prepare for anti-collision detection
        self.transceive(&[PiccCommand::ReqA.into()])?;

        // Mifare spec identification and selection takes 2.5ms to settle without collision
        thread::sleep(Duration::from_micros(2500));

        // Section 9.3.1.14 - we want 8 bits of the last byte transmitted
        self.write(Register::BitFramingReg, 0)?;

        // Anticollision detection, which actually returns the card id
        let (read_bytes, _) = self.transceive(&[PiccCommand::SelCl1.into(), 0x20])?;

        // The card is waiting for selection; release it and put it back in the request state
        self.transceive(&[PiccCommand::HltA.into(), 0x50, 0x00])?;

        if read_bytes.len() < CARD_ID_BYTE_LENGTH {
            return Err(Mfrc522Error::CardNotFound);
        }

        Ok(read_bytes.iter().fold(0u64, |uid, n| uid * 256 + *n as u64))
    }

    fn transceive(&mut self, send_data: &[u8]) -> Result<(Vec<u8>, usize), Mfrc522Error> {
        // See Section 9.3.1.3 - enable all IRQ's except HiAlertlEn
        self.write(Register::ComlEnReg, 0b11110111)?;

        // See Section 9.3.1.5 - clear all IRQ bits
        self.rmw(Register::ComIrqReg, |value| value & !0x80)?;

        // See Section 9.3.1.11 - clear FIFO buffer
        self.rmw(Register::FIFOLevelReg, |value| value | 0x80)?;

        // See Section 9.3.1.2 - idle, canceling outstand commands
        self.write(Register::CommandReg, Command::Idle.into())?;

        // See Section 9.3.1.10 - Write output data to 64 byte FIFO buffer
        assert!(send_data.len() <= 64);

        for i in 0..send_data.len() {
            self.write(Register::FIFODataReg, send_data[i])?;
        }

        // Send the data to any listening PICC card
        self.write(Register::CommandReg, Command::Transceive.into())?;

        // See Section 9.3.1.14 - start data transmission
        self.rmw(Register::BitFramingReg, |value| value | 0x80)?;

        // Wait for IRQ bits to indicate timeout or success
        let start_instant = Instant::now();
        let wait_duration = Duration::from_secs_f64(SAFETY_TIMER_INTERVAL);
        let mut timed_out = false;

        loop {
            // See Section 9.3.1.5 - exit on RxlRq, IdlelRq or TimerIRq
            let irq_bits = self.read(Register::ComIrqReg)?;

            // Exit if either the receive completes or the chip timer counts down
            if (irq_bits & 0b0011_0001u8) != 0 {
                break;
            }

            // Exit if our safety time expires
            if Instant::now().duration_since(start_instant) > wait_duration {
                timed_out = true;
                break;
            }
        }

        // Acknowledge the data
        self.rmw(Register::BitFramingReg, |value| value & !0x80)?;

        // If we hit the safety time out, abort
        // TODO @john: if we hit the chip timer, abort too?
        if timed_out {
            return Err(Mfrc522Error::SafetyTimeout);
        }

        let err = self.read(Register::ErrorReg)? & 0b0001_1011;

        if err != 0 {
            return Err(Mfrc522Error::Transceive(err));
        }

        let mut num_fifo_bytes = self.read(Register::FIFOLevelReg)? as usize;

        num_fifo_bytes = usize::min(num_fifo_bytes, MAX_FIFO_BYTES);

        let mut valid_last_bits = (self.read(Register::ControlReg)? & 0x07) as usize;

        if valid_last_bits == 0 {
            // Per Section 9.3.1.13 - the whole last byte is valid
            valid_last_bits = 8;
        }

        let mut read_data = Vec::<u8>::with_capacity(MAX_FIFO_BYTES);

        for _ in 0..4 {
            read_data.push(self.read(Register::FIFODataReg)?);
        }

        Ok((read_data, valid_last_bits))
    }
}
