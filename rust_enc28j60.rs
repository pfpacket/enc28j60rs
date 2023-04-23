// SPDX-License-Identifier: GPL-2.0

use {
    core::fmt::Debug,
    core::time::Duration,
    kernel::{
        bindings,
        device::RawDevice,
        driver, irq, module_spi_driver, net, of,
        prelude::*,
        spi,
        sync::{smutex::Mutex, Arc, UniqueArc},
        types::ForeignOwnable,
        workqueue,
    },
};

mod enc28j60_hw;
use enc28j60_hw::register::*;
use enc28j60_hw::*;

type BufferRange = core::ops::RangeInclusive<u16>;
const RXFIFO_INIT: BufferRange = 0x0000..=0x19ff;
const TXFIFO_INIT: BufferRange = 0x1a00..=0x1fff;

const ENC28J60_LAMPS_MODE: u16 = 0x3476;
const ETH_MAX_FRAME_LEN: u16 = 1518;

#[allow(non_upper_case_globals)]
const from_dev: fn(&dyn RawDevice) -> kernel::device::Device = kernel::device::Device::from_dev;

struct Enc28j60Driver {
    bank: Bank,
    spidev: spi::Device,
    netdev_reg: Option<net::Registration<Enc28j60Adapter>>,
    irq: Option<irq::ThreadedRegistration<Enc28j60Adapter>>,
    hw_enabled: bool,
    next_packet_ptr: u16,
    skb: Option<ARef<net::SkBuff>>,
}

impl Enc28j60Driver {
    fn netdev(&self) -> ARef<net::Device> {
        // netdev_reg is not None until `spi::Driver` gets removed
        self.netdev_reg.as_ref().unwrap().dev_get()
    }

    fn switch_bank<T: Register>(&mut self, reg: T) -> Result {
        match reg.bank() {
            Some(bank) if self.bank != bank => {
                ECON1.write(&self.spidev, Command::Bfc, econ1::BSEL1 | econ1::BSEL0)?;
                ECON1.write(&self.spidev, Command::Bfs, bank as _)?;
                self.bank = bank;
            }
            _ => {}
        }
        Ok(())
    }

    fn read<T: Register + Debug>(&mut self, reg: T, command: Command) -> Result<T::Size> {
        self.switch_bank(reg)?;
        let value = reg.read(&self.spidev, command)?;
        dev_info!(from_dev(&self.spidev), "Read: {:?}={:04x}\n", reg, value);
        Ok(value)
    }

    fn write<T: Register + Debug>(&mut self, reg: T, command: Command, data: T::Size) -> Result {
        self.switch_bank(reg)?;
        dev_info!(from_dev(&self.spidev), "Write: {:?}={:04x}\n", reg, data);
        reg.write(&self.spidev, command, data)
        //dev_info!(from_dev(&self.spidev), "Read: {:?}={:02x}\n", reg, self.read(reg, Command::Rcr)?);
        //Ok(())
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

    fn wait_for_ready<T: Register + Debug>(
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

    fn check_link_status(&mut self) -> Result {
        let phstat2 = self.read_phy(PHSTAT2)?;

        dev_info!(from_dev(&self.spidev), "PHSTAT2={:04x}\n", phstat2);

        if phstat2 & phstat2::LSTAT > 0 {
            self.netdev().netif_carrier_on();
            let duplex = phstat2 & phstat2::DPXSTAT > 0;
            dev_info!(
                from_dev(&self.spidev),
                "link up ({})\n",
                if duplex { "Full Duplex" } else { "Half Duplex" }
            );
        } else {
            dev_info!(from_dev(&self.spidev), "link down\n");
            self.netdev().netif_carrier_off();
        }

        Ok(())
    }

    fn enable_hardware(&mut self) -> Result {
        self.write_phy(PHIE, phie::PGEIE | phie::PLNKIE)?;

        self.write(
            EIR,
            Command::Bfc,
            eir::DMAIF | eir::LINKIF | eir::TXIF | eir::TXERIF | eir::RXERIF | eir::PKTIF,
        )?;
        self.write(
            EIE,
            Command::Wcr,
            eie::INTIE | eie::PKTIE | eie::LINKIE | eie::TXIE | eie::TXERIE | eie::RXERIE,
        )?;

        // Start RX
        self.write(ECON1, Command::Bfs, econ1::RXEN)?;

        self.hw_enabled = true;

        Ok(())
    }

    fn disable_hardware(&mut self) -> Result {
        self.write(EIE, Command::Wcr, 0x0)?;
        self.write(ECON1, Command::Bfc, econ1::RXEN)?;
        self.hw_enabled = false;
        Ok(())
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

        self.write_phy(PHLCON, ENC28J60_LAMPS_MODE)?;

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

    fn set_random_macaddr(&mut self, netdev: &net::Device) -> Result {
        netdev.eth_hw_addr_random();
        self.set_hw_macaddr(netdev)
    }

    fn set_hw_macaddr(&mut self, netdev: &net::Device) -> Result {
        let dev_addr = netdev.device_address();

        dev_info!(from_dev(&self.spidev), "Set MAC address: {:?}\n", dev_addr);

        self.write(MAADR1, Command::Wcr, dev_addr[0])?;
        self.write(MAADR2, Command::Wcr, dev_addr[1])?;
        self.write(MAADR3, Command::Wcr, dev_addr[2])?;
        self.write(MAADR4, Command::Wcr, dev_addr[3])?;
        self.write(MAADR5, Command::Wcr, dev_addr[4])?;
        self.write(MAADR6, Command::Wcr, dev_addr[5])?;

        Ok(())
    }
}

struct Enc28j60Adapter {
    driver: Mutex<Enc28j60Driver>,
    workqueue: workqueue::BoxedQueue,
    irq_work: workqueue::Work,
    tx_work: workqueue::Work,
}

unsafe impl Send for Enc28j60Adapter {}
unsafe impl Sync for Enc28j60Adapter {}

impl Enc28j60Adapter {
    fn try_new(spidev: spi::Device) -> Result<Arc<Self>> {
        let mut driver = Enc28j60Driver {
            bank: Bank::Bank0,
            spidev,
            netdev_reg: None,
            irq: None,
            hw_enabled: false,
            next_packet_ptr: 0,
            skb: None,
        };

        driver.init_hardware()?;

        let adapter = UniqueArc::try_new(Enc28j60Adapter {
            driver: Mutex::new(driver),
            workqueue: workqueue::Queue::try_new(fmt!("enc28j60_wq"))?,
            // SAFETY: Initialized immediately in the following statements.
            irq_work: unsafe { workqueue::Work::new() },
            tx_work: unsafe { workqueue::Work::new() },
        })?;
        kernel::init_work_item_adapter!(Enc28j60IrqWork, &adapter);
        kernel::init_work_item_adapter!(Enc28j60TxWork, &adapter);

        Ok(adapter.into())
    }

    fn request_irq(self: &Arc<Self>) -> Result {
        let mut driver = self.driver.lock();

        let irq = driver.spidev.get_irq() as _;
        driver.irq = Some(irq::ThreadedRegistration::try_new(
            irq,
            self.clone(),
            irq::flags::SHARED,
            fmt!("enc28j60_{irq}"),
        )?);

        Ok(())
    }

    fn register_netdev(self: &Arc<Self>) -> Result {
        let mut driver = self.driver.lock();
        let mut netdev_reg = net::Registration::try_new(&driver.spidev)?;

        let netdev = netdev_reg.dev_get();
        driver.set_random_macaddr(&netdev)?;
        netdev.set_if_port(bindings::IF_PORT_10BASET as _);
        netdev.set_irq(driver.spidev.get_irq());

        netdev_reg.register(self.clone())?;
        driver.netdev_reg = Some(netdev_reg);
        Ok(())
    }
}

impl driver::DeviceRemoval for Enc28j60Adapter {
    fn device_remove(&self) {
        drop({
            let mut driver = self.driver.lock();
            driver.irq.take()
        });

        self.workqueue.flush();

        drop({
            let mut driver = self.driver.lock();
            driver.netdev_reg.take()
        });
    }
}

impl irq::ThreadedHandler for Enc28j60Adapter {
    type Data = Arc<Self>;

    fn handle_threaded_irq(adapter: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> irq::Return {
        adapter
            .workqueue
            .enqueue_adapter::<Enc28j60IrqWork>(adapter.into());

        irq::Return::Handled
    }
}

#[vtable]
impl net::DeviceOperations for Enc28j60Adapter {
    type Data = Arc<Enc28j60Adapter>;

    fn open(_dev: &net::Device, adapter: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> Result {
        let mut driver = adapter.driver.lock();
        dev_info!(from_dev(&driver.spidev), "netdev::open called\n");

        driver.init_hardware()?;
        driver.enable_hardware()?;
        driver.check_link_status()?;

        driver.netdev().netif_start_queue();

        Ok(())
    }

    // Don't use `netdev_reg` as it might be None
    fn stop(dev: &net::Device, adapter: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> Result {
        let mut driver = adapter.driver.lock();
        dev_info!(from_dev(&driver.spidev), "netdev::stop called\n");

        driver.disable_hardware()?;

        dev.netif_stop_queue();

        Ok(())
    }

    fn start_xmit(
        skb: &net::SkBuff,
        dev: &net::Device,
        adapter: <Self::Data as ForeignOwnable>::Borrowed<'_>,
    ) -> net::NetdevTx {
        dev.netif_stop_queue();
        pr_info!("netdev::start_xmit called\n");

        {
            let buf: ARef<net::SkBuff> = skb.into();
            let mut driver = adapter.driver.lock();
            driver.skb = Some(buf);
        }

        adapter
            .workqueue
            .enqueue_adapter::<Enc28j60TxWork>(adapter.into());

        net::NetdevTx::Ok
    }
}

struct Enc28j60TxWork;

kernel::impl_work_adapter!(Enc28j60TxWork, Enc28j60Adapter, tx_work, |adapter| {
    let _ = move || -> Result {
        let driver = adapter.driver.lock();
        dev_info!(from_dev(&driver.spidev), "Tx Handler\n");
        Ok(())
    };
});

struct Enc28j60IrqWork;

kernel::impl_work_adapter!(Enc28j60IrqWork, Enc28j60Adapter, irq_work, |adapter| {
    let _ = move || -> Result {
        let mut driver = adapter.driver.lock();

        // Stop further interrupts
        driver.write(EIE, Command::Bfc, eie::INTIE)?;

        let eir = driver.read(EIR, Command::Rcr)?;

        dev_info!(from_dev(&driver.spidev), "IRQ EIR={:02x}\n", eir);

        driver.check_link_status()?;

        // Clear IRQ flags
        driver.write(
            EIR,
            Command::Bfc,
            eir::DMAIF | eir::LINKIF | eir::TXIF | eir::TXERIF | eir::RXERIF | eir::PKTIF,
        )?;

        // Enable interrupts
        driver.write(EIE, Command::Bfs, eie::INTIE)?;
        Ok(())
    }();
});

type IdInfo = ();

impl spi::Driver for Enc28j60Adapter {
    kernel::define_of_id_table! {IdInfo, [
        (of::DeviceId::Compatible(b"microchip,enc28j60"), None),
    ]}

    kernel::define_spi_id_table! {IdInfo, [
        (spi::DeviceId(b"enc28j60"), None),
    ]}

    type IdInfo = IdInfo;

    type Data = Arc<Enc28j60Adapter>;

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

        let adapter = Enc28j60Adapter::try_new(spidev)?;
        adapter.request_irq()?;
        adapter.register_netdev()?;

        Ok(adapter)
    }

    fn remove(spidev: spi::Device, _data: &Self::Data) {
        dev_info!(from_dev(&spidev), "rust spi driver remove\n");
    }

    fn shutdown(spidev: spi::Device, _data: <Self::Data as ForeignOwnable>::Borrowed<'_>) {
        dev_info!(from_dev(&spidev), "rust spi driver shutdown\n");
    }
}

module_spi_driver! {
    type: Enc28j60Adapter,
    name: "rust_enc28j60",
    description: "ENC28J60 ethernet driver written in Rust",
    license: "GPL",
}
