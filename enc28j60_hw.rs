// SPDX-License-Identifier: GPL-2.0
#![allow(dead_code)]

use core::ops::{BitAnd, BitOr};
use kernel::{prelude::*, spi};

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
    eth: bool,
}

impl ControlRegisterU8 {
    const fn new(bank: Option<Bank>, addr: u8) -> Self {
        Self {
            bank,
            addr,
            eth: false,
        }
    }

    const fn eth(bank: Option<Bank>, addr: u8) -> Self {
        Self {
            eth: true,
            ..Self::new(bank, addr)
        }
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
    Wcr = 0x40,
    // Bit Field Set (BFS)
    Bfs = 0x80,
    // Bit Field Clear (BFC)
    Bfc = 0xa0,
    // Read Buffer Memory (RBM)
    Rbm = 0x3a,
    // Write Buffer Memory (WBM)
    Wbm = 0x7a,
    // System Reset Command (SRC)
    Src = 0xff,
}

pub(crate) trait Register: Copy {
    type Size: Copy
        + Clone
        + PartialEq
        + Eq
        + core::fmt::LowerHex
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
        let tx_buf = [(command as u8) | self.addr, 0];
        let mut rx_buf = [0u8; 2];
        let rx_len = if self.eth { 1 } else { 2 };

        spidev.write_then_read(&tx_buf[..1], &mut rx_buf[..rx_len])?;

        Ok(rx_buf[rx_len - 1])
    }

    fn write(&self, spidev: &spi::Device, command: Command, data: Self::Size) -> Result {
        let tx_buf = [(command as u8) | self.addr, data];

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
    pub(crate) const EIE: ControlRegisterU8 = ControlRegisterU8::eth(None, 0x1b);
    pub(crate) mod eie {
        pub(crate) const INTIE: u8 = 0x80;
        pub(crate) const PKTIE: u8 = 0x40;
        pub(crate) const DMAIE: u8 = 0x20;
        pub(crate) const LINKIE: u8 = 0x10;
        pub(crate) const TXIE: u8 = 0x08;
        pub(crate) const TXERIE: u8 = 0x02;
        pub(crate) const RXERIE: u8 = 0x01;
    }

    pub(crate) const EIR: ControlRegisterU8 = ControlRegisterU8::eth(None, 0x1c);
    pub(crate) mod eir {
        pub(crate) const PKTIF: u8 = 0x40;
        pub(crate) const DMAIF: u8 = 0x20;
        pub(crate) const LINKIF: u8 = 0x10;
        pub(crate) const TXIF: u8 = 0x08;
        pub(crate) const TXERIF: u8 = 0x02;
        pub(crate) const RXERIF: u8 = 0x01;
    }

    pub(crate) const ESTAT: ControlRegisterU8 = ControlRegisterU8::eth(None, 0x1d);
    pub(crate) mod estat {
        pub(crate) const INT: u8 = 0x80;
        pub(crate) const LATECOL: u8 = 0x10;
        pub(crate) const RXBUSY: u8 = 0x04;
        pub(crate) const TXABRT: u8 = 0x02;
        pub(crate) const CLKRDY: u8 = 0x01;
    }

    pub(crate) const ECON2: ControlRegisterU8 = ControlRegisterU8::eth(None, 0x1e);
    pub(crate) mod econ2 {
        pub(crate) const AUTOINC: u8 = 0x80;
        pub(crate) const PKTDEC: u8 = 0x40;
        pub(crate) const PWRSV: u8 = 0x20;
        pub(crate) const VRPS: u8 = 0x08;
    }

    pub(crate) const ECON1: ControlRegisterU8 = ControlRegisterU8::eth(None, 0x1f);
    pub(crate) mod econ1 {
        pub(crate) const TXRST: u8 = 0x80;
        pub(crate) const RXRST: u8 = 0x40;
        pub(crate) const DMAST: u8 = 0x20;
        pub(crate) const CSUMEN: u8 = 0x10;
        pub(crate) const TXRTS: u8 = 0x08;
        pub(crate) const RXEN: u8 = 0x04;
        pub(crate) const BSEL1: u8 = 0x02;
        pub(crate) const BSEL0: u8 = 0x01;
    }

    //
    // Bank 0
    //
    pub(crate) const ERDPTL: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x00);
    pub(crate) const ERDPTH: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x01);
    pub(crate) const ERDPT: ControlRegisterU16 = ControlRegisterU16::new(ERDPTL, ERDPTH);

    pub(crate) const EWRPTL: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x02);
    pub(crate) const EWRPTH: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x03);
    pub(crate) const EWRPT: ControlRegisterU16 = ControlRegisterU16::new(EWRPTL, EWRPTH);

    pub(crate) const ETXSTL: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x04);
    pub(crate) const ETXSTH: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x05);
    pub(crate) const ETXST: ControlRegisterU16 = ControlRegisterU16::new(ETXSTL, ETXSTH);

    pub(crate) const ETXNDL: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x06);
    pub(crate) const ETXNDH: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x07);
    pub(crate) const ETXND: ControlRegisterU16 = ControlRegisterU16::new(ETXNDL, ETXNDH);

    pub(crate) const ERXSTL: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x08);
    pub(crate) const ERXSTH: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x09);
    pub(crate) const ERXST: ControlRegisterU16 = ControlRegisterU16::new(ERXSTL, ERXSTH);

    pub(crate) const ERXNDL: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x0a);
    pub(crate) const ERXNDH: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x0b);
    pub(crate) const ERXND: ControlRegisterU16 = ControlRegisterU16::new(ERXNDL, ERXNDH);

    pub(crate) const ERXRDPTL: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x0c);
    pub(crate) const ERXRDPTH: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x0d);
    pub(crate) const ERXRDPT: ControlRegisterU16 = ControlRegisterU16::new(ERXRDPTL, ERXRDPTH);

    pub(crate) const ERXWRPTL: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x0e);
    pub(crate) const ERXWRPTH: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank0), 0x0f);
    pub(crate) const ERXWRPT: ControlRegisterU16 = ControlRegisterU16::new(ERXWRPTL, ERXWRPTH);

    //
    // Bank 1
    //
    pub(crate) const ERXFCON: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank1), 0x18);
    pub(crate) mod erxfcon {
        // Unicast Filter Enable bit
        pub(crate) const UCEN: u8 = 0x80;
        // AND/OR Filter Select bit
        pub(crate) const ANDOR: u8 = 0x40;
        // Post-Filter CRC Check Enable bit
        pub(crate) const CRCEN: u8 = 0x20;
        // Pattern Match Filter Enable bit
        pub(crate) const PMEN: u8 = 0x10;
        // Magic Packet Filter Enable bit
        pub(crate) const MPEN: u8 = 0x08;
        // Hash Table Filter Enable bit
        pub(crate) const HTEN: u8 = 0x04;
        // Multicast Filter Enable bit
        pub(crate) const MCEN: u8 = 0x02;
        // Brodcast Filter Enable bit
        pub(crate) const BCEN: u8 = 0x01;
    }

    pub(crate) const EPKTCNT: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank1), 0x19);

    //
    // Bank 2
    //
    pub(crate) const MACON1: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x00);
    pub(crate) mod macon1 {
        pub(crate) const LOOPBK: u8 = 0x10;
        // Pause Control Frame Transmission Enable bit
        pub(crate) const TXPAUS: u8 = 0x08;
        // Pause Control Frames Reception Enable bit
        pub(crate) const RXPAUS: u8 = 0x04;
        // Pass All Received Frames Enable bit
        pub(crate) const PASSALL: u8 = 0x02;
        // MAC Receive Enable bit
        pub(crate) const MARXEN: u8 = 0x01;
    }

    pub(crate) const MACON3: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x02);
    pub(crate) mod macon3 {
        pub(crate) const PADCFG2: u8 = 0x80;
        pub(crate) const PADCFG1: u8 = 0x40;
        pub(crate) const PADCFG0: u8 = 0x20;
        pub(crate) const TXCRCEN: u8 = 0x10;
        pub(crate) const PHDRLEN: u8 = 0x08;
        pub(crate) const HFRMLEN: u8 = 0x04;
        pub(crate) const FRMLNEN: u8 = 0x02;
        pub(crate) const FULDPX: u8 = 0x01;
    }

    pub(crate) const MACON4: ControlRegisterU8 = ControlRegisterU8::new(Some(Bank::Bank2), 0x03);
    pub(crate) mod macon4 {
        pub(crate) const DEFER: u8 = 1 << 6;
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
        pub(crate) const MIISCAN: u8 = 0x02;
        pub(crate) const MIIRD: u8 = 0x01;
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
        pub(crate) const NVALID: u8 = 0x04;
        pub(crate) const SCAN: u8 = 0x02;
        pub(crate) const BUSY: u8 = 0x01;
    }

    pub(crate) const EREVID: ControlRegisterU8 = ControlRegisterU8::eth(Some(Bank::Bank3), 0x12);

    //
    // PHY registers
    //
    pub(crate) const PHCON1: PhyRegister = PhyRegister { addr: 0x00 };
    pub(crate) mod phcon1 {
        pub(crate) const PRST: u16 = 0x8000;
        pub(crate) const PLOOPBK: u16 = 0x4000;
        pub(crate) const PPWRSV: u16 = 0x0800;

        // PHY Duplex Mode bit
        // 1 = Full-Duplex mode
        // 0 = Half-Duplex mode
        pub(crate) const PDPXMD: u16 = 0x0100;
    }

    pub(crate) const PHSTAT1: PhyRegister = PhyRegister { addr: 0x01 };
    pub(crate) mod phstat1 {
        pub(crate) const PFDPX: u16 = 0x1000;
        pub(crate) const PHDPX: u16 = 0x0800;
        pub(crate) const LLSTAT: u16 = 0x0004;
        pub(crate) const JBSTAT: u16 = 0x0002;
    }

    pub(crate) const PHID1: PhyRegister = PhyRegister { addr: 0x02 };
    pub(crate) const PHID2: PhyRegister = PhyRegister { addr: 0x03 };

    pub(crate) const PHCON2: PhyRegister = PhyRegister { addr: 0x10 };
    pub(crate) mod phcon2 {
        pub(crate) const FRCLINK: u16 = 0x4000;
        pub(crate) const TXDIS: u16 = 0x2000;
        pub(crate) const JABBER: u16 = 0x0400;
        pub(crate) const HDLDIS: u16 = 0x0100;
    }

    pub(crate) const PHSTAT2: PhyRegister = PhyRegister { addr: 0x11 };
    pub(crate) mod phstat2 {
        pub(crate) const TXSTAT: u16 = 1 << 13;
        pub(crate) const RXSTAT: u16 = 1 << 12;
        pub(crate) const COLSTAT: u16 = 1 << 11;
        pub(crate) const LSTAT: u16 = 1 << 10;
        pub(crate) const DPXSTAT: u16 = 1 << 9;
        pub(crate) const PLRITY: u16 = 1 << 5;
    }

    pub(crate) const PHIE: PhyRegister = PhyRegister { addr: 0x12 };
    pub(crate) mod phie {
        pub(crate) const PLNKIE: u16 = 1 << 4;
        pub(crate) const PGEIE: u16 = 1 << 1;
    }

    pub(crate) const PHIR: PhyRegister = PhyRegister { addr: 0x13 };
    pub(crate) mod phir {
        pub(crate) const PLNKIF: u16 = 1 << 4;
        pub(crate) const PGEIF: u16 = 1 << 1;
    }
    pub(crate) const PHLCON: PhyRegister = PhyRegister { addr: 0x14 };
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct TxStatusVector {
    pub(crate) byte_count: u16,
    pub(crate) status1: u16,
    pub(crate) total_bytes_transmitted: u16,
    pub(crate) status2: u8,
}

impl TxStatusVector {
    pub(crate) const fn size() -> usize {
        7
    }

    pub(crate) fn new(data: &[u8; Self::size()]) -> TxStatusVector {
        TxStatusVector {
            byte_count: ((data[1] as u16) << 8) | data[0] as u16,
            status1: ((data[3] as u16) << 8) | data[2] as u16,
            total_bytes_transmitted: ((data[5] as u16) << 8) | data[4] as u16,
            status2: data[6],
        }
    }
}

#[repr(packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct RxStatusVector {
    pub(crate) next_ptr: u16,
    pub(crate) byte_count: u16,
    pub(crate) status: u16,
}

impl RxStatusVector {
    pub(crate) const fn size() -> usize {
        core::mem::size_of::<Self>()
    }

    pub(crate) fn new(data: &[u8; Self::size()]) -> Self {
        Self {
            next_ptr: ((data[1] as u16) << 8) | data[0] as u16,
            byte_count: ((data[3] as u16) << 8) | data[2] as u16,
            status: ((data[5] as u16) << 8) | data[4] as u16,
        }
    }

    pub(crate) fn status(&self, mask: RsvStatus) -> bool {
        self.status & mask as u16 != 0
    }
}

#[repr(u16)]
pub(crate) enum RsvStatus {
    LongDropEvent = 1,
    CarrierEvent = 1 << 2,
    CrcError = 1 << 4,
    LengthCheckError = 1 << 5,
    LengthOutOfRange = 1 << 6,
    RxOk = 1 << 7,
    RxMulticast = 1 << 8,
    RxBroadcast = 1 << 9,
    DribbleNibble = 1 << 10,
    RxControlFrame = 1 << 11,
    RxPauseFrame = 1 << 12,
    RxUnknownOpcode = 1 << 13,
    RxTypeVlan = 1 << 14,
}
