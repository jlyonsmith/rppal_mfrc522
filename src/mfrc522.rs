use rppal::spi::{Spi};
use std::error::Error;

pub struct Mfrc522<'a> {
    spi: &'a mut Spi
}

impl Mfrc522<'_> {
    pub fn new(spi: &mut Spi) -> Mfrc522 {
        Mfrc522 {
            spi
        }
    }

    fn write_register(&mut self, reg: Register, val: u8) -> rppal::spi::Result<usize> {
        let write_buffer = [((reg as u8) << 1) & 0x7e, val];

        self.spi.write(&write_buffer)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, Box<dyn Error>> {
        let write_buffer = [(((reg as u8) << 1) & 0x7e) | 0x80, 0];
        let mut read_buffer = [0u8; 1];
        self.spi.transfer(&mut read_buffer, &write_buffer)?;

        Ok(read_buffer[0])
    }

    pub fn antenna_on(&mut self) -> Result<(), Box<dyn Error>>  {
        let val = self.read_register(Register::TxControlReg)?;

        self.write_register(Register::TxControlReg, val | 0x03)?;

        Ok(())
      }

    pub fn antenna_off(&mut self) -> Result<(), Box<dyn Error>>  {
        let val = self.read_register(Register::TxControlReg)?;

        self.write_register(Register::TxControlReg, val & !0x03)?;

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.write_register(Register::CommandReg, Command::SoftReset as u8)?; // reset chip
        self.write_register(Register::TModeReg, 0x8d)?; // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
        self.write_register(Register::TPrescalerReg, 0x3e)?; // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
        self.write_register(Register::TReloadRegL, 30)?; // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
        self.write_register(Register::TReloadRegH, 0)?;
        self.write_register(Register::TxAutoReg, 0x40)?; // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
        self.write_register(Register::ModeReg, 0x3d)?; // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
        self.antenna_on()?; // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)

        Ok(())
    }

    // Find card, read card type
    // TagType - Returns the card type
    // 0x4400 = Mifare_UltraLight
    // 0x0400 = Mifare_One (S50)
    // 0x0200 = Mifare_One (S70)
    // 0x0800 = Mifare_Pro (X)
    // 0x4403 = Mifare_DESFire
    fn find_card() -> Result<{
      self.write_register(Register::BitFramingReg, 0x07);

      let tagType = [CMD.PICC_REQIDL];
      let response = this.to_card(Command::Transeive, tagType)?;

      if (response.bit_size != 0x10) {
          response.status = ERROR;
      }

      return { status: response.status, bitSize: response.bitSize };
    }

  // Anti-collision detection, get uid (serial number) of found card
  // 4 byte are for the card to return the serial number, the fifth bit is for the check bit
  get_uid() {
    this.write_register(Register::BitFramingReg, 0x00);

    let uid = [CMD.PICC_ANTICOLL, 0x20];
    let response = this.toCard(CMD.PCD_TRANSCEIVE, uid);

    if (response.status) {
      let uidCheck = 0;
      for (let i = 0; i < 4; i++) {
        uidCheck = uidCheck ^ response.data[i];
      }
      if (uidCheck != response.data[4]) {
        response.status = ERROR;
      }
    }
    return { status: response.status, data: response.data };
  }

  fn to_card(command: Command, bitsToSend) {
    let data = [];
    let bitSize = 0;
    let status = ERROR;
    let irqEn = 0x00;
    let waitIRq = 0x00;

    if (command == CMD.PCD_AUTHENT) {
      irqEn = 0x12;
      waitIRq = 0x10;
    }
    if (command == CMD.PCD_TRANSCEIVE) {
      irqEn = 0x77;
      waitIRq = 0x30;
    }
    this.writeRegister(CMD.CommIEnReg, irqEn | 0x80); //Interrupt request is enabled
    this.clearRegisterBitMask(CMD.CommIrqReg, 0x80); //Clears all interrupt request bits
    this.setRegisterBitMask(CMD.FIFOLevelReg, 0x80); //FlushBuffer=1, FIFO initialization
    this.writeRegister(CMD.CommandReg, CMD.PCD_IDLE); // Stop calculating CRC for new content in the FIFO.
    //Write data to the FIFO
    for (let i = 0; i < bitsToSend.length; i++) {
      this.writeRegister(CMD.FIFODataReg, bitsToSend[i]);
    }
    //Excuting command
    this.writeRegister(CMD.CommandReg, command);
    if (command == CMD.PCD_TRANSCEIVE) {
      this.setRegisterBitMask(CMD.BitFramingReg, 0x80); //StartSend=1,transmission of data starts
    }
    //Wait for the received data to complete
    let i = 2000; //According to the clock frequency adjustment, operation M1 card maximum waiting time 25ms
    let n = 0;
    do {
      n = this.readRegister(CMD.CommIrqReg);
      i--;
    } while (i != 0 && !(n & 0x01) && !(n & waitIRq));

    this.clearRegisterBitMask(CMD.BitFramingReg, 0x80); //StartSend=0
    if (i != 0) {
      if ((this.readRegister(CMD.ErrorReg) & 0x1b) == 0x00) {
        //BufferOvfl Collerr CRCErr ProtecolErr
        status = OK;
        if (n & irqEn & 0x01) {
          status = ERROR;
        }
        if (command == CMD.PCD_TRANSCEIVE) {
          n = this.readRegister(CMD.FIFOLevelReg);
          let lastBits = this.readRegister(CMD.ControlReg) & 0x07;
          if (lastBits) {
            bitSize = (n - 1) * 8 + lastBits;
          } else {
            bitSize = n * 8;
          }
          if (n == 0) {
            n = 1;
          }
          if (n > 16) {
            n = 16;
          }
          //Reads the data received in the FIFO
          for (let i = 0; i < n; i++) {
            data.push(this.readRegister(CMD.FIFODataReg));
          }
        }
      } else {
        status = ERROR;
      }
    }
    return { status: status, data: data, bitSize: bitSize };
  }
}

#[allow(dead_code)]
#[repr(u8)]
enum Register {
    Reserved00 = 0x00,
    CommandReg = 0x01,
    CommIEnReg = 0x02,
    DivlEnReg = 0x03,
    CommIrqReg = 0x04,
    DivIrqReg = 0x05,
    ErrorReg = 0x06,
    Status1Reg = 0x07,
    Status2Reg = 0x08,
    FIFODataReg = 0x09,
    FIFOLevelReg = 0x0A,
    WaterLevelReg = 0x0B,
    ControlReg = 0x0C,
    BitFramingReg = 0x0D,
    CollReg = 0x0E,
    Reserved01 = 0x0F,

    Reserved10 = 0x10,
    ModeReg = 0x11,
    TxModeReg = 0x12,
    RxModeReg = 0x13,
    TxControlReg = 0x14,
    TxAutoReg = 0x15,
    TxSelReg = 0x16,
    RxSelReg = 0x17,
    RxThresholdReg = 0x18,
    DemodReg = 0x19,
    Reserved11 = 0x1A,
    Reserved12 = 0x1B,
    MifareReg = 0x1C,
    Reserved13 = 0x1D,
    Reserved14 = 0x1E,
    SerialSpeedReg = 0x1F,

    Reserved20 = 0x20,
    CRCResultRegM = 0x21,
    CRCResultRegL = 0x22,
    Reserved21 = 0x23,
    ModWidthReg = 0x24,
    Reserved22 = 0x25,
    RFCfgReg = 0x26,
    GsNReg = 0x27,
    CWGsPReg = 0x28,
    ModGsPReg = 0x29,
    TModeReg = 0x2A,
    TPrescalerReg = 0x2B,
    TReloadRegH = 0x2C,
    TReloadRegL = 0x2D,
    TCounterValueRegH = 0x2E,
    TCounterValueRegL = 0x2F,

    Reserved30 = 0x30,
    TestSel1Reg = 0x31,
    TestSel2Reg = 0x32,
    TestPinEnReg = 0x33,
    TestPinValueReg = 0x34,
    TestBusReg = 0x35,
    AutoTestReg = 0x36,
    VersionReg = 0x37,
    AnalogTestReg = 0x38,
    TestDAC1Reg = 0x39,
    TestDAC2Reg = 0x3A,
    TestADCReg = 0x3B,
    Reserved31 = 0x3C,
    Reserved32 = 0x3D,
    Reserved33 = 0x3E,
    Reserved34 = 0x3F
}

#[allow(dead_code)]
#[repr(u8)]
enum Command {
    Idle = 0x00,
    AuthEnt = 0x0E,
    Receive = 0x08,
    Transmit = 0x04,
    Transeive = 0x0C,
    SoftReset = 0x0F,
    CalcCrc = 0x03,
}
