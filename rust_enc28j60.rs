// SPDX-License-Identifier: GPL-2.0

use {
    core::time::Duration,
    kernel::{
        bindings,
        device::RawDevice,
        driver, irq, module_spi_driver, net, of,
        prelude::*,
        spi,
        sync::{smutex::Mutex, Arc},
        types::ForeignOwnable,
    },
};

mod enc28j60;
use enc28j60::register::*;
use enc28j60::*;

#[allow(non_upper_case_globals)]
const from_dev: fn(&dyn RawDevice) -> kernel::device::Device = kernel::device::Device::from_dev;

type BufferRange = core::ops::RangeInclusive<u16>;
const RXFIFO_INIT: BufferRange = 0x0000..=0x19ff;
const TXFIFO_INIT: BufferRange = 0x1a00..=0x1fff;

const ENC28J60_LAMPS_MODE: u16 = 0x3476;

const ETH_MAX_FRAME_LEN: u16 = 1518;

struct EncInner {
    bank: Bank,
    spidev: spi::Device,
    netdev_reg: Option<net::Registration<EncAdapter>>,
    irq: Option<irq::ThreadedRegistration<EncAdapter>>,
    next_packet_ptr: u16,
}

impl EncInner {
    fn read<T: Register>(&mut self, reg: T, command: Command) -> Result<T::Size> {
        self.switch_bank(reg.bank())?;
        reg.read(&self.spidev, command)
    }

    fn write<T: Register>(&mut self, reg: T, command: Command, data: T::Size) -> Result {
        self.switch_bank(reg.bank())?;
        reg.write(&self.spidev, command, data)
    }

    fn switch_bank(&mut self, bank: Option<Bank>) -> Result {
        match bank {
            Some(bank) if self.bank != bank => set_bank(&self.spidev, bank),
            _ => Ok(()),
        }
    }

    fn wait_for_ready<T: Register>(
        &mut self,
        reg: T,
        mask: <T as Register>::Size,
        val: <T as Register>::Size,
    ) -> Result {
        kernel::delay::coarse_sleep(Duration::from_millis(1));

        while self.read(reg, Command::Rcr)? & mask != val {
            kernel::delay::coarse_sleep(Duration::from_millis(1));
        }

        Ok(())
    }

    fn read_phy(&mut self, reg: PhyRegister) -> Result<u16> {
        // Set the PHY register address
        self.write(MIREGADR, Command::Wcr, reg.addr)?;

        // Start reading the PHY register
        self.write(MICMD, Command::Wcr, micmd::MIIRD)?;

        self.wait_for_ready(MISTAT, mistat::BUSY, 0)?;

        self.write(MICMD, Command::Wcr, 0)?;

        self.read(MIRD, Command::Rcr)
    }

    fn write_phy(&mut self, reg: PhyRegister, data: u16) -> Result {
        // Set the PHY register address
        self.write(MIREGADR, Command::Wcr, reg.addr)?;

        // Start reading the PHY register
        self.write(MIWR, Command::Wcr, data)?;

        self.wait_for_ready(MISTAT, mistat::BUSY, 0)
    }

    fn init_hardware(&mut self) -> Result {
        self.spidev.write(&[Command::Src as u8])?;
        kernel::delay::coarse_sleep(Duration::from_millis(2));

        self.write(ECON1, Command::Wcr, 0x0)?;
        self.bank = Bank::Bank0;

        let revision = self.read(EREVID, Command::Rcr)?;
        dev_info!(from_dev(&self.spidev), "ENC28J60 EREVID = {}\n", revision);
        if revision == 0 || revision == 0xff {
            return Err(ENODEV);
        }

        // Enable address auto increment
        self.write(ECON2, Command::Wcr, econ2::AUTO_INC)?;

        self.init_rxfifo(&RXFIFO_INIT)?;
        self.init_txfifo(&TXFIFO_INIT)?;

        // Default filter mode
        self.write(
            ERXFCON,
            Command::Wcr,
            erxfcon::UCEN | erxfcon::CRCEN | erxfcon::BCEN,
        )?;

        // - Enable RX setup
        self.write(
            MACON1,
            Command::Wcr,
            macon1::MARXEN | macon1::RXPAUS | macon1::TXPAUS,
        )?;

        // - Enable full duplex mode
        self.write(
            MACON3,
            Command::Wcr,
            macon3::FULLDPX | macon3::FRMLNEN | macon3::TXCRCEN | macon3::PADCFG_60,
        )?;
        self.write(MAIPGL, Command::Wcr, 0x12)?;
        self.write(MABBIPG, Command::Wcr, 0x15)?;

        self.write(MAMXFL, Command::Wcr, ETH_MAX_FRAME_LEN)?;

        self.write_phy(PHCON1, phcon1::PDPXMD)?;
        self.write_phy(PHCON2, 0x0)?;

        dev_info!(from_dev(&self.spidev), "Hardware initialized\n");

        Ok(())
    }

    fn erxrdpt_workaround(next_packet_ptr: u16, range: &BufferRange) -> u16 {
        next_packet_ptr
            .checked_sub(1)
            .and_then(|v| if range.contains(&v) { Some(v) } else { None })
            .unwrap_or_else(|| *range.end())
    }

    fn init_rxfifo(&mut self, range: &BufferRange) -> Result {
        if range.is_empty()
            || !RXFIFO_INIT.contains(range.start())
            || !RXFIFO_INIT.contains(range.end())
        {
            return Err(EINVAL);
        }

        self.next_packet_ptr = *range.start();

        // RX buffer start pointer
        self.write(ERXST, Command::Wcr, *range.start())?;

        // RX read pointer
        let erxrdpt = Self::erxrdpt_workaround(self.next_packet_ptr, range);
        self.write(ERXRDPT, Command::Wcr, erxrdpt)?;

        // RX buffer end pointer
        self.write(ERXND, Command::Wcr, *range.end())
    }

    fn init_txfifo(&mut self, range: &BufferRange) -> Result {
        if range.is_empty()
            || !TXFIFO_INIT.contains(range.start())
            || !TXFIFO_INIT.contains(range.end())
        {
            return Err(EINVAL);
        }

        // TX buffer start pointer
        self.write(ETXST, Command::Wcr, *range.start())?;

        // TX buffer end pointer
        self.write(ETXND, Command::Wcr, *range.end())
    }

    fn set_random_macaddr(&mut self) -> Result {
        if let Some(ref netdev_reg) = self.netdev_reg {
            (*netdev_reg.dev_get()).eth_hw_addr_random();
            self.set_hw_macaddr()?;
        }
        Ok(())
    }

    fn set_hw_macaddr(&mut self) -> Result {
        if let Some(ref netdev_reg) = self.netdev_reg {
            let netdev = netdev_reg.dev_get();
            let dev_addr = netdev.device_address();

            self.write(MAADR6, Command::Wcr, dev_addr[0])?;
            self.write(MAADR5, Command::Wcr, dev_addr[1])?;
            self.write(MAADR4, Command::Wcr, dev_addr[2])?;
            self.write(MAADR3, Command::Wcr, dev_addr[3])?;
            self.write(MAADR2, Command::Wcr, dev_addr[4])?;
            self.write(MAADR1, Command::Wcr, dev_addr[5])?;

            dev_info!(from_dev(&self.spidev), "Set MAC address: {:?}\n", dev_addr);
        }

        Ok(())
    }
}

struct EncAdapter {
    inner: Mutex<EncInner>,
}

impl driver::DeviceRemoval for EncAdapter {
    fn device_remove(&self) {
        let (irq, netdev_reg) = {
            let mut inner = self.inner.lock();
            dev_info!(from_dev(&inner.spidev), "device removal\n");
            (
                core::mem::replace(&mut inner.irq, None),
                core::mem::replace(&mut inner.netdev_reg, None),
            )
        };

        drop(irq);
        drop(netdev_reg);
    }
}

impl EncAdapter {
    fn try_new(spidev: spi::Device) -> Result<Self> {
        let mut enc = EncInner {
            bank: Bank::Bank0,
            spidev,
            netdev_reg: None,
            irq: None,
            next_packet_ptr: 0,
        };

        enc.init_hardware()?;

        Ok(EncAdapter {
            inner: Mutex::new(enc),
        })
    }
}

#[vtable]
impl net::DeviceOperations for EncAdapter {
    type Data = Arc<EncAdapter>;

    fn open(dev: &net::Device, data: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> Result {
        let mut enc = data.inner.lock();
        dev_info!(from_dev(&enc.spidev), "netdev::open called\n");

        // Enable IRQ
        enc.write(EIE, Command::Bfs, eie::INTIE | eie::PKTIE)?;

        // Start RX
        enc.write(ECON1, Command::Bfs, econ1::RXEN)
    }

    fn stop(dev: &net::Device, data: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> Result {
        let mut enc = data.inner.lock();
        dev_info!(from_dev(&enc.spidev), "netdev::stop called\n");

        // Enable IRQ
        enc.write(EIE, Command::Bfc, eie::INTIE | eie::PKTIE)?;

        // Start RX
        enc.write(ECON1, Command::Bfc, econ1::RXEN)?;

        if let Some(netdev_reg) = enc.netdev_reg.as_ref() {
            let netdev = netdev_reg.dev_get();
            netdev.netif_stop_queue();
            netdev.netif_carrier_off();
        }

        Ok(())
    }

    fn start_xmit(
        skb: &net::SkBuff,
        dev: &net::Device,
        data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
    ) -> net::NetdevTx {
        //pr_info!("netdev::start_xmit called\n");
        net::NetdevTx::Busy
    }
}

impl irq::ThreadedHandler for EncAdapter {
    type Data = Arc<EncAdapter>;

    fn handle_threaded_irq(data: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> irq::Return {
        dev_info!(from_dev(&data.inner.lock().spidev), "IRQ handler called\n");
        irq::Return::Handled
    }
}

type EncIdInfo = ();

impl spi::Driver for EncAdapter {
    kernel::define_of_id_table! {EncIdInfo, [
        (of::DeviceId::Compatible(b"microchip,enc28j60"), None),
    ]}

    kernel::define_spi_id_table! {EncIdInfo, [
        (spi::DeviceId(b"enc28j60"), None),
    ]}

    type Data = Arc<EncAdapter>;

    type IdInfo = EncIdInfo;

    fn probe(
        spidev: spi::Device,
        _of_id_info: Option<&Self::IdInfo>,
        _spi_id_info: Option<&Self::IdInfo>,
    ) -> Result<Self::Data> {
        dev_info!(
            from_dev(&spidev),
            "of={:?} spi={:?}\n",
            _of_id_info,
            _spi_id_info
        );

        let irq = spidev.get_irq();
        let enc = Arc::try_new(EncAdapter::try_new(spidev)?)?;

        {
            let mut inner = enc.inner.lock();

            let netdev_reg = net::Registration::try_new(&inner.spidev)?;
            let netdev = netdev_reg.dev_get();
            inner.netdev_reg = Some(netdev_reg);

            inner.set_random_macaddr()?;

            inner.irq = Some(irq::ThreadedRegistration::try_new(
                irq as _,
                enc.clone(),
                irq::flags::SHARED,
                fmt!("enc28j60_{irq}"),
            )?);

            netdev.set_if_port(bindings::IF_PORT_10BASET as _);
            netdev.set_irq(irq);

            inner.netdev_reg.as_mut().unwrap().register(enc.clone())?;
        }

        Ok(enc)
    }

    fn remove(spidev: spi::Device, _data: &Self::Data) {
        dev_info!(from_dev(&spidev), "rust spi driver remove\n");
    }

    fn shutdown(spidev: spi::Device, _data: <Self::Data as ForeignOwnable>::Borrowed<'_>) {
        dev_info!(from_dev(&spidev), "rust spi driver shutdown\n");
    }
}

module_spi_driver! {
    type: EncAdapter,
    name: "rust_enc28j60",
    description: "ENC28J60 ethernet driver written in Rust",
    license: "GPL",
}
