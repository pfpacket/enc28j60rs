// SPDX-License-Identifier: GPL-2.0
#![allow(dead_code)]

use core::ops::{BitAnd, BitOr};
use kernel::{prelude::*, spi};
use register::*;

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub(crate) enum Bank {
    Bank0 = 0,
    Bank1 = 1,
    Bank2 = 2,
    Bank3 = 3,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) struct ControlRegisterU8 {
    // None = common registers
    bank: Option<Bank>,
    addr: u8,
}

impl ControlRegisterU8 {
    const fn new(bank: Option<Bank>, addr: u8) -> Self {
        Self { bank, addr }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) struct ControlRegisterU16 {
    low: ControlRegisterU8,
    high: ControlRegisterU8,
}

impl ControlRegisterU16 {
    const fn new(low: ControlRegisterU8, high: ControlRegisterU8) -> Self {
        assert!(
            match (low.bank, high.bank) {
                (Some(low), Some(high)) => low as u8 == high as _,
                (None, None) => true,
                _ => false,
            },
            "The banks of low and high registers must be the same"
        );

        Self { low, high }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) struct PhyRegister {
    pub(crate) addr: u8,
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) enum Command {
    // Read Control Register (RCR)
    Rcr = 0x0,
    // Write Control Register (WCR)
    Wcr = 0x2,
    // Bit Field Set (BFS)
    Bfs = 0x4,
    // Bit Field Clear (BFC)
    Bfc = 0x5,
    // Read Buffer Memory (RBM)
    Rbm = 0x3a,
    // Write Buffer Memory (WBM)
    Wbm = 0x7a,
    // System Reset Command (SRC)
    Src = 0xff,
}

pub(crate) fn set_bank(spidev: &spi::Device, bank: Bank) -> Result {
    ECON1.write(spidev, Command::Bfc, econ1::BSEL1 | econ1::BSEL0)?;
    ECON1.write(spidev, Command::Bfs, bank as _)
}

pub(crate) trait Register: Copy {
    type Size: Copy
        + Clone
        + PartialEq
        + Eq
        + BitAnd<Output = Self::Size>
        + BitOr<Output = Self::Size>;

    fn bank(&self) -> Option<Bank>;
    fn read(&self, _: &spi::Device, _: Command) -> Result<Self::Size>;
    fn write(&self, _: &spi::Device, _: Command, data: Self::Size) -> Result;
}

impl Register for ControlRegisterU8 {
    type Size = u8;

    fn bank(&self) -> Option<Bank> {
        self.bank
    }

    fn read(&self, spidev: &spi::Device, command: Command) -> Result<Self::Size> {
        let tx_buf = [(command as u8) << 5 | self.addr];
        let mut rx_buf = [0u8; 1];

        spidev.write_then_read(&tx_buf, &mut rx_buf)?;

        Ok(rx_buf[0])
    }

    fn write(&self, spidev: &spi::Device, command: Command, data: Self::Size) -> Result {
        let tx_buf = [(command as u8) << 5 | self.addr, data];

        spidev.write(&tx_buf)
    }
}

impl Register for ControlRegisterU16 {
    type Size = u16;

    fn bank(&self) -> Option<Bank> {
        // static assetion in `new` ensures the banks of low and high regs are the same
        self.low.bank
    }

    fn read(&self, spidev: &spi::Device, command: Command) -> Result<Self::Size> {
        let low = self.low.read(spidev, command)?;
        let high = self.high.read(spidev, command)?;

        Ok((high as u16) << 8 | low as u16)
    }

    fn write(&self, spidev: &spi::Device, command: Command, data: Self::Size) -> Result {
        self.low.write(spidev, command, data as u8)?;
        self.high.write(spidev, command, (data >> 8) as u8)
    }
}

pub(crate) mod register {
    use super::{Bank, ControlRegisterU16, ControlRegisterU8, PhyRegister};

    //
    // Common Bank
    //
    pub(crate) const EIE: ControlRegisterU8 = ControlRegisterU8::new(None, 0x1b);
    pub(crate) mod eie {
        pub(crate) const RXERIE: u8 = 0b00000001;
        pub(crate) const TXERIE: u8 = 0b00000010;
        pub(crate) const TXIE: u8 = 0b00001000;
        pub(crate) const LINKIE: u8 = 0b00010000;
        pub(crate) const DMAIE: u8 = 0b00100000;
        pub(crate) const PKTIE: u8 = 0b01000000;
        pub(crate) const INTIE: u8 = 0b10000000;
    }

    pub(crate) const EIR: ControlRegisterU8 = ControlRegisterU8::new(None, 0x1c);
    pub(crate) const ESTAT: ControlRegisterU8 = ControlRegisterU8::new(None, 0x1d);

    pub(crate) const ECON2: ControlRegisterU8 = ControlRegisterU8::new(None, 0x1e);
    pub(crate) mod econ2 {
        pub(crate) const AUTO_INC: u8 = 0b10000000;
    }

    pub(crate) const ECON1: ControlRegisterU8 = ControlRegisterU8::new(None, 0x1f);
    pub(crate) mod econ1 {
        pub(crate) const BSEL0: u8 = 0b00000001;
        pub(crate) const BSEL1: u8 = 0b00000010;
        pub(crate) const RXEN: u8 = 0b00000100;
        pub(crate) const TXRTS: u8 = 0b00001000;
        pub(crate) const CSUMEN: u8 = 0b00010000;
        pub(crate) const DMAST: u8 = 0b00100000;
        pub(crate) const RXRST: u8 = 0b01000000;
        pub(crate) const TXRST: u8 = 0b10000000;
    }

    //
    // Bank 1
    //
    pub(crate) const ERDPTL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x00);
    pub(crate) const ERDPTH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x01);
    pub(crate) const ERDPT: ControlRegisterU16 = ControlRegisterU16::new(ERDPTL, ERDPTH);

    pub(crate) const EWRPTL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x02);
    pub(crate) const EWRPTH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x03);
    pub(crate) const EWRPT: ControlRegisterU16 = ControlRegisterU16::new(EWRPTL, EWRPTH);

    pub(crate) const ETXSTL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x04);
    pub(crate) const ETXSTH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x05);
    pub(crate) const ETXST: ControlRegisterU16 = ControlRegisterU16::new(ETXSTL, ETXSTH);

    pub(crate) const ETXNDL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x06);
    pub(crate) const ETXNDH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x07);
    pub(crate) const ETXND: ControlRegisterU16 = ControlRegisterU16::new(ETXNDL, ETXNDH);

    pub(crate) const ERXSTL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x08);
    pub(crate) const ERXSTH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x09);
    pub(crate) const ERXST: ControlRegisterU16 = ControlRegisterU16::new(ERXSTL, ERXSTH);

    pub(crate) const ERXNDL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x0a);
    pub(crate) const ERXNDH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x0b);
    pub(crate) const ERXND: ControlRegisterU16 = ControlRegisterU16::new(ERXNDL, ERXNDH);

    pub(crate) const ERXRDPTL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x0c);
    pub(crate) const ERXRDPTH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x0d);
    pub(crate) const ERXRDPT: ControlRegisterU16 = ControlRegisterU16::new(ERXRDPTL, ERXRDPTH);

    pub(crate) const ERXWRPTL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x0e);
    pub(crate) const ERXWRPTH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank0), 0x0f);
    pub(crate) const ERXWRPT: ControlRegisterU16 = ControlRegisterU16::new(ERXWRPTL, ERXWRPTH);

    pub(crate) const ERXFCON: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank1), 0x18);
    pub(crate) mod erxfcon {
        // Brodcast Filter Enable bit
        pub(crate) const BCEN: u8 = 0b00000001;
        // Multicast Filter Enable bit
        pub(crate) const MCEN: u8 = 0b00000010;
        // Hash Table Filter Enable bit
        pub(crate) const HTEN: u8 = 0b00000100;
        // Magic Packet Filter Enable bit
        pub(crate) const MPEN: u8 = 0b00001000;
        // Pattern Match Filter Enable bit
        pub(crate) const PMEN: u8 = 0b00010000;
        // Post-Filter CRC Check Enable bit
        pub(crate) const CRCEN: u8 = 0b00100000;
        // AND/OR Filter Select bit
        pub(crate) const ANDOR: u8 = 0b01000000;
        // Unicast Filter Enable bit
        pub(crate) const UCEN: u8 = 0b10000000;
    }

    pub(crate) const EPKTCNT: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank1), 0x19);

    //
    // Bank 2
    //
    pub(crate) const MACON1: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x00);
    pub(crate) mod macon1 {
        // MAC Receive Enable bit
        pub(crate) const MARXEN: u8 = 0b00000001;
        // Pass All Received Frames Enable bit
        pub(crate) const PASSALL: u8 = 0b00000010;
        // Pause Control Frames Reception Enable bit
        pub(crate) const RXPAUS: u8 = 0b00000100;
        // Pause Control Frame Transmission Enable bit
        pub(crate) const TXPAUS: u8 = 0b00001000;
    }

    pub(crate) const MACON3: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x02);
    pub(crate) mod macon3 {
        pub(crate) const FULLDPX: u8 = 0b00000001;
        pub(crate) const FRMLNEN: u8 = 0b00000010;
        pub(crate) const HFRMEN: u8 = 0b00000100;
        pub(crate) const PHDREN: u8 = 0b00001000;
        pub(crate) const TXCRCEN: u8 = 0b00010000;
        pub(crate) const PADCFG0: u8 = 0b00100000;
        pub(crate) const PADCFG1: u8 = 0b01000000;
        pub(crate) const PADCFG2: u8 = 0b10000000;
        pub(crate) const PADCFG_60: u8 = 0b00100000;
    }

    pub(crate) const MACON4: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x03);
    pub(crate) mod macon4 {
        pub(crate) const DEFER: u8 = 0b01000000;
    }

    pub(crate) const MABBIPG: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x04);

    pub(crate) const MAIPGL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x06);
    pub(crate) const MAIPGH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x07);
    pub(crate) const MAIPG: ControlRegisterU16 = ControlRegisterU16::new(MAIPGL, MAIPGH);

    pub(crate) const MAMXFLL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x0a);
    pub(crate) const MAMXFLH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x0b);
    pub(crate) const MAMXFL: ControlRegisterU16 = ControlRegisterU16::new(MAMXFLL, MAMXFLH);

    pub(crate) const MICMD: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x12);
    pub(crate) mod micmd {
        pub(crate) const MIISCAN: u8 = 0b00000010;
        pub(crate) const MIIRD: u8 = 0b00000001;
    }

    pub(crate) const MIREGADR: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x14);

    pub(crate) const MIWRL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x16);
    pub(crate) const MIWRH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x17);
    pub(crate) const MIWR: ControlRegisterU16 = ControlRegisterU16::new(MIWRL, MIWRH);

    pub(crate) const MIRDL: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x18);
    pub(crate) const MIRDH: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x19);
    pub(crate) const MIRD: ControlRegisterU16 = ControlRegisterU16::new(MIRDL, MIRDH);

    //
    // Bank 3
    //
    pub(crate) const MAADR5: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank3), 0x00);
    pub(crate) const MAADR6: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank3), 0x01);
    pub(crate) const MAADR3: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank3), 0x02);
    pub(crate) const MAADR4: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank3), 0x03);
    pub(crate) const MAADR1: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank3), 0x04);
    pub(crate) const MAADR2: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank3), 0x05);

    pub(crate) const MISTAT: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank3), 0x0a);
    pub(crate) mod mistat {
        pub(crate) const BUSY: u8 = 0b00000001;
        pub(crate) const SCAN: u8 = 0b00000010;
        pub(crate) const INVALID: u8 = 0b00000100;
    }

    pub(crate) const EREVID: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank3), 0x12);

    //
    // PHY registers
    //
    pub(crate) const PHCON1: PhyRegister = PhyRegister { addr: 0x00 };
    pub(crate) mod phcon1 {
        // PHY Duplex Mode bit
        // 1 = Full-Duplex mode
        // 0 = Half-Duplex mode
        pub(crate) const PDPXMD: u16 = 0x0100;
        pub(crate) const PHCON1_PPWRSV: u16 = 0x0800;
        pub(crate) const PHCON1_PLOOPBK: u16 = 0x4000;
    }

    pub(crate) const PHSTAT1: PhyRegister = PhyRegister { addr: 0x01 };
    pub(crate) const PHHID1: PhyRegister = PhyRegister { addr: 0x02 };
    pub(crate) const PHHID2: PhyRegister = PhyRegister { addr: 0x03 };
    pub(crate) const PHCON2: PhyRegister = PhyRegister { addr: 0x10 };
    pub(crate) const PHSTAT2: PhyRegister = PhyRegister { addr: 0x11 };
    pub(crate) const PHIE: PhyRegister = PhyRegister { addr: 0x12 };
    pub(crate) const PHIR: PhyRegister = PhyRegister { addr: 0x13 };
    pub(crate) const PHLCON: PhyRegister = PhyRegister { addr: 0x14 };
}
