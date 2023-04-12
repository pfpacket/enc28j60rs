#![allow(dead_code)]

use kernel::{prelude::*, spi};

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Bank {
    Bank0 = 0,
    Bank1 = 1,
    Bank2 = 2,
    Bank3 = 3,
    Common = 4,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub(crate) struct ControlRegisterU8 {
    bank: Bank,
    addr: u8,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub(crate) struct ControlRegisterU16 {
    low: ControlRegisterU8,
    high: ControlRegisterU8,
}

pub(crate) const EIE: ControlRegisterU8 = ControlRegisterU8 {
    bank: Bank::Common,
    addr: 0x1b,
};
pub(crate) const EIR: ControlRegisterU8 = ControlRegisterU8 {
    bank: Bank::Common,
    addr: 0x1c,
};
pub(crate) const ESTAT: ControlRegisterU8 = ControlRegisterU8 {
    bank: Bank::Common,
    addr: 0x1d,
};
pub(crate) const ECON2: ControlRegisterU8 = ControlRegisterU8 {
    bank: Bank::Common,
    addr: 0x1e,
};
pub(crate) const ECON1: ControlRegisterU8 = ControlRegisterU8 {
    bank: Bank::Common,
    addr: 0x1f,
};
#[allow(non_snake_case)]
mod ECON1 {
    pub(crate) const BSEL0: u8 = 0b00000001;
    pub(crate) const BSEL1: u8 = 0b00000010;
}

pub(crate) const ERDPTL: ControlRegisterU8 = ControlRegisterU8 {
    bank: Bank::Bank0,
    addr: 0x00,
};
pub(crate) const ERDPTH: ControlRegisterU8 = ControlRegisterU8 {
    bank: Bank::Bank0,
    addr: 0x01,
};
pub(crate) const ERDPT: ControlRegisterU16 = ControlRegisterU16 {
    low: ERDPTL,
    high: ERDPTH,
};

pub(crate) const EREVID: ControlRegisterU8 = ControlRegisterU8 {
    bank: Bank::Bank3,
    addr: 0x12,
};

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub(crate) enum Command {
    Rcr = 0x0,
    Rbm = 0x3a,
    Wcr = 0x2,
    Wbm = 0x7a,
    Bfs = 0x4,
    Bfc = 0x5,
    Src = 0xff,
}

pub(crate) fn write_u8(
    spidev: &spi::Device,
    command: Command,
    reg: ControlRegisterU8,
    data: u8,
) -> Result {
    let tx_buf = [(command as u8) << 5 | reg.addr, data];

    spidev.write(&tx_buf)
}

pub(crate) fn read_u8(
    spidev: &spi::Device,
    command: Command,
    reg: ControlRegisterU8,
) -> Result<u8> {
    let tx_buf = [(command as u8) << 5 | reg.addr];
    let mut rx_buf = [0u8; 1];

    spidev.write_then_read(&tx_buf, &mut rx_buf)?;

    Ok(rx_buf[0])
}

pub(crate) fn set_bank(spidev: &spi::Device, bank: Bank) -> Result {
    if bank == Bank::Common {
        Ok(())
    } else {
        write_u8(spidev, Command::Bfc, ECON1, ECON1::BSEL1 | ECON1::BSEL0)?;
        write_u8(spidev, Command::Bfs, ECON1, bank as u8)
    }
}

pub(crate) trait Register {
    type Size;

    fn bank(&self) -> Bank;
    fn read(&self, _: &spi::Device, _: Command) -> Result<Self::Size>;
    fn write(&self, _: &spi::Device, _: Command, data: Self::Size) -> Result;
}

impl Register for ControlRegisterU8 {
    type Size = u8;

    fn bank(&self) -> Bank {
        self.bank
    }

    fn read(&self, spidev: &spi::Device, command: Command) -> Result<Self::Size> {
        read_u8(spidev, command, *self)
    }

    fn write(&self, spidev: &spi::Device, command: Command, data: Self::Size) -> Result {
        write_u8(spidev, command, *self, data)
    }
}

impl Register for ControlRegisterU16 {
    type Size = u16;

    // There are no registers whose lower and higher registers are located in different banks.
    fn bank(&self) -> Bank {
        self.low.bank
    }

    fn read(&self, spidev: &spi::Device, command: Command) -> Result<Self::Size> {
        let low: u8 = self.low.read(spidev, command)?;
        let high: u8 = self.high.read(spidev, command)?;

        Ok((high as u16) << 8 | low as u16)
    }

    fn write(&self, spidev: &spi::Device, command: Command, data: Self::Size) -> Result {
        self.low.write(spidev, command, data as u8)?;
        self.high.write(spidev, command, (data >> 8) as _)
    }
}
