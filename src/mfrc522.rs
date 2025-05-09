use crate::{picc::PiccCommand, register::*};
use rppal::spi::Spi;
use std::time::{Duration, Instant};
use std::thread;
use thiserror::Error;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

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

#[derive(Debug, Error)]
pub enum Mfrc522Error {
    #[error("error reading card")]
    Transceive(u8),
    #[error("safety timeout reading card")]
    SafetyTimeout,
    #[error("UID is not in expected size of 5 bytes")]
    BadUidSize,
}

pub struct Mfrc522<'a> {
    spi: &'a mut Spi,
}

impl Mfrc522<'_> {
    pub fn new(spi: &mut Spi) -> Mfrc522 {
        Mfrc522 { spi }
    }

    fn write(&mut self, reg: Register, data: u8) -> Result<()> {
        // See Section 8.1.2.2 for details of writing to the Mifare registers over SPI

        // No zero needed to terminate the data when writing
        let write_buffer = [((reg as u8) << 1) & 0x7e, data];

        self.spi.write(&write_buffer)?;

        Ok(())
    }

    fn read(&mut self, reg: Register) -> Result<u8> {
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

    fn read_write(&mut self, reg: Register, func: impl FnOnce(u8) -> u8) -> Result<()> {
        let value = self.read(reg)?;
        let new_value = func(value);
        self.write(reg, new_value)?;

        Ok(())
    }

    pub fn reset(&mut self) -> Result<()> {
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
        self.read_write(Register::TxControlReg, |value| (value | 0x03))?; // Turn on the antenna
        Ok(())
    }

    pub fn get_version(&mut self) -> Result<u8> {
        Ok(self.read(Register::VersionReg)?)
    }

    pub fn read_card_id(&mut self) -> Result<u64> {
		// Section 9.3.1.14 - we want 7 bits of the last byte transmitted
        // TODO @john - Why do we only want the last 7 bits for this request?
        self.write(Register::BitFramingReg, 0x07)?;

        // Communicate with nearby card and request it to prepare for anti-collision detection
        self.transceive_with_card(&[PiccCommand::ReqA.into()])?;

		// Mifare spec identification and selection takes 2.5ms to settle without collision
		thread::sleep(Duration::from_micros(2500));

        // Section 9.3.1.14 - we want 8 bits of the last byte transmitted
        // TODO @john - Again, why? Where is this defined?
        self.write(Register::BitFramingReg, 0)?;

        // Anticollision detection, which actually returns the card id
        let (read_bytes, _) = self.transceive_with_card(&[PiccCommand::SelCl1.into(), 0x20])?;

		// The card is waiting for selection; release it and put it back in the request state
		self.transceive_with_card(&[PiccCommand::HltA.into(), 0x50, 0x00])?;

		if read_bytes.len() != 5 {
			return Err(Box::new(Mfrc522Error::BadUidSize))
		}

		Ok(read_bytes.iter().fold(0u64, |uid, n| uid * 256 + *n as u64))
    }

    fn transceive_with_card(&mut self, send_data: &[u8]) -> Result<(Vec<u8>, usize)> {
        // See Section 9.3.1.3 - enable all IRQ's except HiAlertlEn
        self.write(Register::ComlEnReg, 0b11110111)?;

        // See Section 9.3.1.5 - clear all IRQ bits
        self.read_write(Register::ComIrqReg, |value| value & !0x80)?;

        // See Section 9.3.1.11 - clear FIFO buffer
        self.read_write(Register::FIFOLevelReg, |value| value | 0x80)?;

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
        self.read_write(Register::BitFramingReg, |value| value | 0x80)?;

        // Wait for IRQ bits to indicate timeout or success
        let start_instant = Instant::now();
        let wait_duration = Duration::from_secs_f64(SAFETY_TIMER_INTERVAL);
        let mut timed_out = false;

        loop {
            // See Section 9.3.1.5 - exit on RxlRq, IdlelRq or TimerIRq
            let irq_bits = self.read(Register::ComIrqReg)?;

			// Exit if either the receive completes or the chip timer counts down
            if (irq_bits & 0b0011_0001u8) != 0  {
                break;
            }

			// Exit if our safety time expires
            if Instant::now().duration_since(start_instant) > wait_duration {
                timed_out = true;
                break;
            }
        }

		// Acknowledge the data
		self.read_write(Register::BitFramingReg, |value| value & !0x80)?;

        // If we hit the safety time out, abort
		// TODO @john: if we hit the chip timer, abort too?
        if timed_out {
            return Err(Box::new(Mfrc522Error::SafetyTimeout));
        }

        let err = self.read(Register::ErrorReg)? & 0b0001_1011;

		if err != 0 {
			return Err(Box::new(Mfrc522Error::Transceive(err)));
		}

        let mut num_fifo_bytes = self.read(Register::FIFOLevelReg)? as usize;

		num_fifo_bytes = usize::min(usize::max(num_fifo_bytes, 1), MAX_FIFO_BYTES);

        let mut valid_last_bits = (self.read(Register::ControlReg)? & 0x07) as usize;

        if valid_last_bits == 0 {
            // Per Section 9.3.1.13 - the whole last byte is valid
            valid_last_bits = 8;
        }

        let mut read_data = Vec::<u8>::with_capacity(num_fifo_bytes);

        for _ in 0..num_fifo_bytes {
            read_data.push(self.read(Register::FIFODataReg)?);
        }

        Ok((read_data, valid_last_bits))
    }
}
