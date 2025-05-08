use crate::register::*;
use rppal::spi::Spi;
use std::error::Error;

pub struct Mfrc522<'a> {
    spi: &'a mut Spi,
}

impl Mfrc522<'_> {
    pub fn new(spi: &mut Spi) -> Mfrc522 {
        Mfrc522 { spi }
    }

    fn write_register(&mut self, reg: Register, val: u8) -> Result<(), Box<dyn Error>> {
        let write_buffer = [((reg as u8) << 1) & 0x7e, val];

        self.spi.write(&write_buffer)?;
        Ok(())
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, Box<dyn Error>> {
        let write_buffer = [(((reg as u8) << 1) & 0x7e) | 0x80, 0];
        let mut read_buffer = [0u8; 2];

        self.spi.transfer(&mut read_buffer, &write_buffer)?;

        Ok(read_buffer[1])
    }

    fn read_write_register(
        &mut self,
        reg: Register,
        func: impl FnOnce(u8) -> u8,
    ) -> Result<(), Box<dyn Error>> {
        let value = self.read_register(reg)?;

        let new_value = func(value);
        self.write_register(reg, new_value)?;

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Box<dyn Error>> {
        self.write_register(Register::CommandReg, Command::SoftReset as u8)?; // reset chip
        self.write_register(Register::TModeReg, 0x8d)?; // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
        self.write_register(Register::TPrescalerReg, 0x3e)?; // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
        self.write_register(Register::TReloadRegLow, 30)?; // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
        self.write_register(Register::TReloadRegHigh, 0)?;
        self.write_register(Register::TxASKReg, 0x40)?; // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
        self.write_register(Register::ModeReg, 0x3d)?; // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
        self.read_write_register(Register::TxControlReg, |value| (value | 0x03))?; // Turn on the antenna
        Ok(())
    }

    pub fn version(&mut self) -> Result<u8, Box<dyn Error>> {
        Ok(self.read_register(Register::VersionReg)?)
    }
}
