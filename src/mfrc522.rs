use crate::{picc::*, register::*};
use rppal::spi::{self, Segment, Spi};
use std::thread;
use std::time::{Duration, Instant};
use thiserror::Error;

/// A 4-byte card id
pub struct Uid
where
    [u8; 4]: Sized,
{
    /// The UID can have 4, 7 or 10 bytes.
    bytes: [u8; 4],
    /// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
    sak: PiccSak,
}

impl Uid {
    /// Create a Uid from a byte array and a SAK value
    pub fn new(bytes: [u8; 4], sak: PiccSak) -> Self {
        Self { bytes, sak }
    }

    /// Get the underlying bytes of the UID
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Get the UID as an u32
    pub fn to_u32(&self) -> u32 {
        ((self.bytes[0] as u32) << 0)
            + ((self.bytes[1] as u32) << 8)
            + ((self.bytes[2] as u32) << 16)
            + ((self.bytes[3] as u32) << 24)
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
    pub bytes: [u8; 2],
}

// MFRC522 chip frequency
const MFRC_FREQ: f64 = 13.56e6;
// See Section 9.3.3.10 - this is the desired MFRC522 countdown timer tick frequency
const TICK_FREQ: f64 = 1999.7;
// The inverse calculation to get the desired prescale value
static PRESCALE: f64 = (MFRC_FREQ - TICK_FREQ) / (2.0 * TICK_FREQ);
// The desired MFRC522 countdown timer interval
const TIMER_INTERVAL: f64 = 0.015;

/// Errors that can occur from the crate
#[derive(Debug, Error)]
pub enum Mfrc522Error {
    #[error("Operation timed out")]
    Timeout,
    #[error("Operation was not acknowledged")]
    Nak,
    #[error("Card not found")]
    CardNotFound,
    #[error("SPI error")]
    SpiError(#[from] spi::Error),
    #[error("FIFO buffer overflow")]
    FifoOverflow,
    #[error("Incomplete frame was received")]
    IncompleteFrame,
    #[error("Reader protocol error")]
    ReaderBadProtocol,
    #[error("Reader parity error")]
    ReaderParity,
    #[error("Reader CRC error")]
    ReaderCrc,
    #[error("Reader collision")]
    ReaderCollision,
    #[error("Reader overflow")]
    ReaderOverflow,
    #[error("Reader overheating")]
    ReaderOverheating,
    #[error("Reader bad write")]
    ReaderBadWrite,
    #[error("Non standard anti-collision protocol")]
    Proprietary,
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

    /// Resets the reader
    ///
    /// Should be called before first time use and to reset a card that has been woken up from being disabled.
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

    /// Returns the version reported by the MFRC522.
    ///
    /// Common values are 0x91, 0x92 for original Mifare cards, but clones may have values such as 0xb2 and others. So don't
    /// use this value for anything other than a quick check that you can communicate with the reader.
    pub fn version(&mut self) -> Result<u8, Mfrc522Error> {
        Ok(self.read(Register::VersionReg)?)
    }

    /// Sets the antenna gain of the receiver
    ///
    /// Setting this to a higher value will increase the sensitivity of the antenna.
    pub fn set_antenna_gain(&mut self, gain: RxGain) -> Result<(), Mfrc522Error> {
        self.write(Register::RFCfgReg, gain.into())
    }

    /// Request card to execute a command
    fn command(&mut self, command: Command) -> Result<(), Mfrc522Error> {
        self.write(Register::CommandReg, command.into())
    }

    /// Sends command to enter HALT state
    pub fn hlta(&mut self, timeout: Duration) -> Result<(), Mfrc522Error> {
        let start_instant = Instant::now();
        let mut buffer: [u8; 4] = [PiccCommand::HltA as u8, 0, 0, 0];
        let crc = self.calculate_crc(&buffer[..2], timeout)?;
        buffer[2..].copy_from_slice(&crc);

        // If the PICC responds with any modulation during a period of 1 ms
        // after the end of the frame containing the HLTA command,
        // this response is interpreted as 'not acknowledge',
        // hence the reversed logic here.
        match self.transceive(&buffer, 0, timeout.saturating_sub(start_instant.elapsed())) {
            Err(Mfrc522Error::Timeout) => Ok(()),
            Ok(_) => Err(Mfrc522Error::Nak),
            Err(e) => Err(e),
        }
    }

    /// Sends a REQuest type A to nearby PICCs
    pub fn reqa(&mut self, timeout: Duration) -> Result<AtqA, Mfrc522Error> {
        // NOTE REQA is a short frame (7 bits)
        let fifo_data = self.transceive(&[PiccCommand::ReqA as u8], 7, timeout)?;

        if fifo_data.buffer.len() != 2 || fifo_data.valid_bits != 0 {
            Err(Mfrc522Error::IncompleteFrame)
        } else {
            Ok(AtqA {
                bytes: fifo_data.buffer.try_into().unwrap(),
            })
        }
    }

    /// Read the UID of any available card
    pub fn uid(&mut self, timeout: Duration) -> Result<Uid, Mfrc522Error> {
        let start_instant = Instant::now();
        let _atqa = self.reqa(timeout)?;

        if timeout.saturating_sub(start_instant.elapsed()) > Duration::ZERO {
            // Mifare spec identification and selection takes 2.5ms to settle without collision
            thread::sleep(Duration::from_micros(2500));
        }

        // Anticollision detection, which actually returns the card id
        let fifo_data = self.transceive(
            &[PiccCommand::SelCl1 as u8, 0x20],
            0,
            timeout.saturating_sub(start_instant.elapsed()),
        )?;

        if fifo_data.buffer.len() != 5 || fifo_data.valid_bits != 0 {
            return Err(Mfrc522Error::IncompleteFrame);
        }

        let uid = Uid {
            sak: PiccSak::from(fifo_data.buffer[0]),
            bytes: fifo_data.buffer[1..=4].try_into().unwrap(),
        };

        // The card is waiting for selection; release it and put it back in the request state
        // See MF1S503x Section 8.2.4
        self.hlta(timeout.saturating_sub(start_instant.elapsed()))?;

        Ok(uid)
    }

    /// Transmit and receive data to a PICC card
    fn transceive(
        &mut self,
        tx_buffer: &[u8],
        tx_last_bits: u8,
        timeout: Duration,
    ) -> Result<FifoData, Mfrc522Error> {
        // See Section 9.3.1.10 - Write output data to 64 byte FIFO buffer
        assert!(tx_buffer.len() <= 64);

        // Stop any ongoing command
        self.command(Command::Idle)?;

        // Clear all interrupt flags
        self.write(Register::ComIrqReg, 0x7f)?;

        // Flush FIFO buffer. See Section 9.3.1.11
        self.write(Register::FIFOLevelReg, 1 << 7)?;

        // Write data to transmit to the FIFO buffer
        self.write_many(Register::FIFODataReg, tx_buffer)?;

        // Signal transceive
        self.command(Command::Transceive)?;

        // Configure bit framing and start transmission
        self.write(Register::BitFramingReg, (1 << 7) | (tx_last_bits & 0b0111))?;

        let start_instant = Instant::now();

        /// ComIrqReg: timer decrements the timer value in register TCounterValReg to zero
        const TIMER_IRQ: u8 = 1 << 0;
        // ComIrqReg: an error bit in ErrorReg is set
        const ERR_IRQ: u8 = 1 << 1;
        // ComIrqReg: TODO
        const IDLE_IRQ: u8 = 1 << 4;
        // ComIrqReg: receiver detected the end of a valid data stream
        const RX_IRQ: u8 = 1 << 5;

        loop {
            // See Section 9.3.1.5 - exit on RxlRq, IdlelRq or TimerIRq
            let irq = self.read(Register::ComIrqReg)?;

            if irq & (RX_IRQ | ERR_IRQ | IDLE_IRQ) != 0 {
                break;
            } else if irq & TIMER_IRQ != 0 || start_instant.elapsed() >= timeout {
                return Err(Mfrc522Error::Timeout);
            }
        }

        self.check_error_register()?;

        let num_bytes = self.read(Register::FIFOLevelReg)? as usize;
        let mut buffer = vec![0; num_bytes];
        let mut valid_bits = 0;

        if num_bytes > 0 {
            self.read_many(Register::FIFODataReg, &mut buffer[0..num_bytes])?;
            valid_bits = (self.read(Register::ControlReg)? & 0x07) as usize;
        }

        Ok(FifoData { buffer, valid_bits })
    }

    fn calculate_crc(&mut self, data: &[u8], timeout: Duration) -> Result<[u8; 2], Mfrc522Error> {
        // stop any ongoing command
        self.command(Command::Idle)?;

        /// DivIrqReg: CalcCRC command is active and all data is processed
        const CRC_IRQ: u8 = 1 << 2;

        // clear the CRC_IRQ interrupt flag
        self.write(Register::DivIrqReg, CRC_IRQ)?;

        // Flush FIFO buffer. See Section 9.3.1.11
        self.write(Register::FIFOLevelReg, 1 << 7)?;

        // write data to transmit to the FIFO buffer
        self.write_many(Register::FIFODataReg, data)?;

        self.command(Command::CalcCRC)?;

        // Wait for the CRC calculation to complete.
        let mut irq;
        let start_instant = Instant::now();

        loop {
            irq = self.read(Register::DivIrqReg)?;

            if irq & CRC_IRQ != 0 {
                self.command(Command::Idle)?;
                let crc = [
                    self.read(Register::CRCResultRegLow)?,
                    self.read(Register::CRCResultRegHigh)?,
                ];

                return Ok(crc);
            }

            if start_instant.elapsed() > timeout {
                return Err(Mfrc522Error::Timeout);
            }
        }
    }

    fn check_error_register(&mut self) -> Result<(), Mfrc522Error> {
        let err = self.read(Register::ErrorReg)?;

        // ErrorReg: set to logic 1 if the SOF is incorrect
        pub const PROTOCOL_ERR: u8 = 1 << 0;
        // ErrorReg: parity check failed
        pub const PARITY_ERR: u8 = 1 << 1;
        // ErrorReg: the RxModeReg register’s RxCRCEn bit is set and the CRC calculation fails
        pub const CRC_ERR: u8 = 1 << 2;
        // ErrorReg: a bit-collision is detected
        pub const COLL_ERR: u8 = 1 << 3;
        // ErrorReg: the host or a MFRC522’s internal state machine (e.g. receiver) tries to
        // write data to the FIFO buffer even though it is already full
        pub const BUFFER_OVFL: u8 = 1 << 4;
        // ErrorReg: internal temperature sensor detects overheating
        pub const TEMP_ERR: u8 = 1 << 6;
        // ErrorReg: data is written into the FIFO buffer by the host during the MFAuthent
        // command or if data is written into the FIFO buffer by the host during the
        // time between sending the last bit on the RF interface and receiving the
        // last bit on the RF interface
        pub const WR_ERR: u8 = 1 << 7;

        if err & PROTOCOL_ERR != 0 {
            Err(Mfrc522Error::ReaderBadProtocol)
        } else if err & PARITY_ERR != 0 {
            Err(Mfrc522Error::ReaderParity)
        } else if err & CRC_ERR != 0 {
            Err(Mfrc522Error::ReaderCrc)
        } else if err & COLL_ERR != 0 {
            Err(Mfrc522Error::ReaderCollision)
        } else if err & BUFFER_OVFL != 0 {
            Err(Mfrc522Error::ReaderOverflow)
        } else if err & TEMP_ERR != 0 {
            Err(Mfrc522Error::ReaderOverheating)
        } else if err & WR_ERR != 0 {
            Err(Mfrc522Error::ReaderBadWrite)
        } else {
            Ok(())
        }
    }
}

/// Data read from the internal FIFO buffer
#[derive(Debug, PartialEq)]
pub struct FifoData {
    /// The contents of the FIFO buffer
    pub buffer: Vec<u8>,
    /// The number of valid bits in the last byte
    pub valid_bits: usize,
}
