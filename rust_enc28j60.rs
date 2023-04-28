// SPDX-License-Identifier: GPL-2.0
use {
    core::time::Duration,
    kernel::{
        bindings,
        device::RawDevice,
        driver, irq, module_spi_driver, net, of,
        prelude::*,
        spi,
        sync::{smutex::Mutex, Arc, SpinLock, UniqueArc},
        types::ForeignOwnable,
        workqueue,
    },
};

mod enc28j60_hw;
use enc28j60_hw::register::*;
use enc28j60_hw::*;

type FifoRange = core::ops::RangeInclusive<u16>;
const RXFIFO_INIT: FifoRange = 0x0000..=0x19ff;
const TXFIFO_INIT: FifoRange = 0x1a00..=0x1fff;

const ENC28J60_LAMPS_MODE: u16 = 0x3476;
const ETH_MAX_FRAME_LEN: u16 = 1518;

#[allow(non_upper_case_globals)]
const from_dev: fn(&dyn RawDevice) -> kernel::device::Device = kernel::device::Device::from_dev;

struct Enc28j60Driver {
    bank: Bank,
    spidev: spi::Device,
    netdev_reg: Option<net::Registration<Enc28j60Adapter>>,
    irq: Option<irq::ThreadedRegistration<Enc28j60Adapter>>,
    next_packet_ptr: u16,
    xfer_buf: [u8; 4 + ETH_MAX_FRAME_LEN as usize],
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

    fn read<T: Register>(&mut self, reg: T) -> Result<T::Size> {
        self.switch_bank(reg)?;
        reg.read(&self.spidev, Command::Rcr)
    }

    fn write<T: Register>(&mut self, reg: T, command: Command, data: T::Size) -> Result {
        self.switch_bank(reg)?;
        reg.write(&self.spidev, command, data)
    }

    fn read_buffer(&mut self, addr: u16, rx_buf: &mut [u8]) -> Result {
        self.write(ERDPT, Command::Wcr, addr)?;

        let tx_buf = [Command::Rbm as _];
        self.spidev.write_then_read(&tx_buf, rx_buf)
    }

    fn write_buffer(&mut self, tx_buf: &[u8]) -> Result {
        let buf = &mut self.xfer_buf[..tx_buf.len() + 1];
        buf[0] = Command::Wbm as _;
        buf[1..].copy_from_slice(&tx_buf);

        self.spidev.write(buf)
    }

    fn read_tsv(&mut self) -> Result<TxStatusVector> {
        let tsv_ptr = self.read(ETXND)? + 1;

        let mut tsv = [0u8; TxStatusVector::size()];
        self.read_buffer(tsv_ptr, &mut tsv)?;
        Ok(TxStatusVector::new(&tsv))
    }

    fn read_phy(&mut self, reg: PhyRegister) -> Result<u16> {
        self.write(MIREGADR, Command::Wcr, reg.addr)?;
        self.write(MICMD, Command::Wcr, micmd::MIIRD)?;
        self.wait_for_ready(MISTAT, mistat::BUSY, 0)?;
        self.write(MICMD, Command::Wcr, 0)?;
        self.read(MIRD)
    }

    fn write_phy(&mut self, reg: PhyRegister, data: u16) -> Result {
        self.write(MIREGADR, Command::Wcr, reg.addr)?;
        self.write(MIWR, Command::Wcr, data)?;
        self.wait_for_ready(MISTAT, mistat::BUSY, 0)
    }

    fn wait_for_ready<T: Register>(
        &mut self,
        reg: T,
        mask: <T as Register>::Size,
        val: <T as Register>::Size,
    ) -> Result {
        while (self.read(reg)? & mask) != val {
            kernel::delay::coarse_sleep(Duration::from_millis(1));
        }

        Ok(())
    }

    fn check_link_status(&mut self) -> Result {
        let phstat2 = self.read_phy(PHSTAT2)?;

        if (phstat2 & phstat2::LSTAT) != 0 {
            self.netdev().netif_carrier_on();
            let duplex = (phstat2 & phstat2::DPXSTAT) != 0;
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

        self.write(ECON1, Command::Bfs, econ1::RXEN)?;

        Ok(())
    }

    fn disable_hardware(&mut self) -> Result {
        self.write(EIE, Command::Wcr, 0x0)?;
        self.write(ECON1, Command::Bfc, econ1::RXEN)?;
        Ok(())
    }

    fn init_hardware(&mut self) -> Result {
        self.spidev.write(&[Command::Src as u8])?;
        kernel::delay::coarse_sleep(Duration::from_millis(2));

        self.write(ECON1, Command::Wcr, 0x0)?;
        self.bank = Bank::Bank0;

        match self.read(EREVID)? {
            0x0 | 0xff => return Err(ENODEV),
            _ => {}
        };

        self.write(ECON2, Command::Wcr, econ2::AUTOINC)?;

        self.init_rxfifo(&RXFIFO_INIT)?;
        self.init_txfifo(&TXFIFO_INIT)?;

        self.write(
            ERXFCON,
            Command::Wcr,
            erxfcon::UCEN | erxfcon::CRCEN | erxfcon::BCEN,
        )?;

        self.write(
            MACON1,
            Command::Wcr,
            macon1::MARXEN | macon1::RXPAUS | macon1::TXPAUS,
        )?;

        self.write(
            MACON3,
            Command::Wcr,
            macon3::FULDPX | macon3::FRMLNEN | macon3::TXCRCEN | macon3::PADCFG0,
        )?;
        self.write(MAIPG, Command::Wcr, 0x12)?;
        self.write(MABBIPG, Command::Wcr, 0x15)?;

        self.write(MAMXFL, Command::Wcr, ETH_MAX_FRAME_LEN)?;

        self.write_phy(PHLCON, ENC28J60_LAMPS_MODE)?;

        self.write_phy(PHCON1, phcon1::PDPXMD)?;
        self.write_phy(PHCON2, 0x0)?;

        dev_info!(from_dev(&self.spidev), "Hardware initialized\n");

        Ok(())
    }

    fn erxrdpt_workaround(next_packet_ptr: u16, range: &FifoRange) -> u16 {
        next_packet_ptr
            .checked_sub(1)
            .and_then(|v| if range.contains(&v) { Some(v) } else { None })
            .unwrap_or_else(|| *range.end())
    }

    fn init_rxfifo(&mut self, range: &FifoRange) -> Result {
        if range.is_empty()
            || !RXFIFO_INIT.contains(range.start())
            || !RXFIFO_INIT.contains(range.end())
        {
            return Err(EINVAL);
        }

        self.next_packet_ptr = *range.start();

        self.write(ERXST, Command::Wcr, *range.start())?;

        let erxrdpt = Self::erxrdpt_workaround(self.next_packet_ptr, range);
        self.write(ERXRDPT, Command::Wcr, erxrdpt)?;

        self.write(ERXND, Command::Wcr, *range.end())
    }

    fn init_txfifo(&mut self, range: &FifoRange) -> Result {
        if range.is_empty()
            || !TXFIFO_INIT.contains(range.start())
            || !TXFIFO_INIT.contains(range.end())
        {
            return Err(EINVAL);
        }

        self.write(ETXST, Command::Wcr, *range.start())?;
        self.write(ETXND, Command::Wcr, *range.end())
    }

    fn set_random_macaddr(&mut self, netdev: &net::Device) -> Result {
        netdev.eth_hw_addr_random();
        self.set_hw_macaddr(netdev)
    }

    fn set_hw_macaddr(&mut self, netdev: &net::Device) -> Result {
        let dev_addr = netdev.device_address();

        dev_info!(
            from_dev(&self.spidev),
            "MAC address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\n",
            dev_addr[0],
            dev_addr[1],
            dev_addr[2],
            dev_addr[3],
            dev_addr[4],
            dev_addr[5]
        );

        for (reg, addr) in [MAADR1, MAADR2, MAADR3, MAADR4, MAADR5, MAADR6]
            .iter()
            .zip(dev_addr)
        {
            self.write(*reg, Command::Wcr, *addr)?;
        }

        Ok(())
    }

    fn handle_rx(&mut self) -> Result<bool> {
        let packet_count = self.read(EPKTCNT)?;
        if packet_count == 0 {
            return Ok(false);
        }

        for _ in 0..packet_count {
            self.handle_rx_packet()?;
            self.write(ECON2, Command::Bfs, econ2::PKTDEC)?;
        }

        Ok(true)
    }

    fn handle_rx_packet(&mut self) -> Result {
        let mut rsv = [0; RxStatusVector::size()];
        self.read_buffer(self.next_packet_ptr, &mut rsv)?;
        let rsv = RxStatusVector::new(&rsv);

        if !rsv.status(RsvStatus::RxOk) || rsv.byte_count > ETH_MAX_FRAME_LEN {
            dev_err!(
                from_dev(&self.spidev),
                "RX failed: {:?} Crc={} LengthCheckError={}\n",
                rsv,
                rsv.status(RsvStatus::CrcError),
                rsv.status(RsvStatus::LengthCheckError)
            );
        } else {
            let netdev = self.netdev();
            let skb = netdev.alloc_skb_ip_align(rsv.byte_count as _)?;
            let room = skb.put(rsv.byte_count as _);

            let read_ptr = Self::next_rx_start_ptr(self.next_packet_ptr);
            self.read_buffer(read_ptr, room)?;

            skb.set_protocol(skb.eth_type_trans(&netdev));
            netdev.netif_rx(&skb);
        }

        self.next_packet_ptr = rsv.next_ptr;
        let erxrdpt = Self::erxrdpt_workaround(rsv.next_ptr, &RXFIFO_INIT);
        self.write(ERXRDPT, Command::Wcr, erxrdpt)?;

        Ok(())
    }

    fn next_rx_start_ptr(ptr: u16) -> u16 {
        let rsv_end = ptr + RxStatusVector::size() as u16;
        if RXFIFO_INIT.contains(&rsv_end) {
            rsv_end
        } else {
            rsv_end - (RXFIFO_INIT.end() - RXFIFO_INIT.start() + 1)
        }
    }
}

struct Enc28j60Adapter {
    driver: Mutex<Enc28j60Driver>,
    workqueue: workqueue::BoxedQueue,
    irq_work: workqueue::Work,
    tx_work: workqueue::Work,
    tx_skb: SpinLock<Option<ARef<net::SkBuff>>>,
}

// SAFETY:
//  - `Send` for `workqueue::Work` and `ARef<SkBuff>`.
//     The types lack Send due to holding raw pointers.
//     It's safe to send them to other threads.
//     Dereferencing raw pointers is unsafe anyway.
//  - `Sync` for `workqueue::Work`.
//     The type lacks Sync due to holding raw pointers.
//     `Work` wraps `work_struct` which is a thread-safe type.
unsafe impl Send for Enc28j60Adapter {}
unsafe impl Sync for Enc28j60Adapter {}

impl Enc28j60Adapter {
    fn try_new(spidev: spi::Device) -> Result<Arc<Self>> {
        let mut driver = Enc28j60Driver {
            bank: Bank::Bank0,
            spidev,
            netdev_reg: None,
            irq: None,
            next_packet_ptr: 0,
            xfer_buf: [0; ETH_MAX_FRAME_LEN as usize + 4],
        };

        driver.init_hardware()?;

        let mut adapter = UniqueArc::try_new(Enc28j60Adapter {
            driver: Mutex::new(driver),
            workqueue: workqueue::Queue::try_new(fmt!("enc28j60_wq"))?,
            // SAFETY: Initialized immediately in the following statements.
            irq_work: unsafe { workqueue::Work::new() },
            tx_work: unsafe { workqueue::Work::new() },
            tx_skb: unsafe { SpinLock::new(None) },
        })?;
        kernel::init_work_item_adapter!(IrqWorkHandler, &adapter);
        kernel::init_work_item_adapter!(TxWorkHandler, &adapter);
        kernel::spinlock_init!(
            unsafe { Pin::new_unchecked(&mut adapter.tx_skb) },
            "enc_skb"
        );

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

#[vtable]
impl net::DeviceOperations for Enc28j60Adapter {
    type Data = Arc<Enc28j60Adapter>;

    fn open(dev: &net::Device, adapter: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> Result {
        let mut driver = adapter.driver.lock();

        driver.disable_hardware()?;
        driver.init_hardware()?;
        driver.set_hw_macaddr(dev)?;
        driver.enable_hardware()?;
        driver.check_link_status()?;

        dev.netif_start_queue();

        Ok(())
    }

    // Don't use `netdev_reg` as it might be None
    fn stop(dev: &net::Device, adapter: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> Result {
        let mut driver = adapter.driver.lock();

        dev.netif_stop_queue();

        driver.disable_hardware()?;

        Ok(())
    }

    fn start_xmit(
        skb: &net::SkBuff,
        dev: &net::Device,
        adapter: <Self::Data as ForeignOwnable>::Borrowed<'_>,
    ) -> net::NetdevTx {
        dev.netif_stop_queue();

        *adapter.tx_skb.lock_irqdisable() = Some(skb.into());

        adapter
            .workqueue
            .enqueue_adapter::<TxWorkHandler>(adapter.into());

        net::NetdevTx::Ok
    }
}

impl irq::ThreadedHandler for Enc28j60Adapter {
    type Data = Arc<Self>;

    fn handle_threaded_irq(adapter: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> irq::Return {
        adapter
            .workqueue
            .enqueue_adapter::<IrqWorkHandler>(adapter.into());

        irq::Return::Handled
    }
}

struct IrqWorkHandler;

kernel::impl_work_adapter!(IrqWorkHandler, Enc28j60Adapter, irq_work, |adapter| {
    let _ = move || -> Result {
        let mut driver = adapter.driver.lock();

        driver.write(EIE, Command::Bfc, eie::INTIE)?;

        let mut iteration = false;
        while {
            let eir = driver.read(EIR)?;

            if eir & eir::DMAIF != 0 {
                iteration = true;
                driver.write(EIR, Command::Bfc, eir::DMAIF)?;
            }

            if eir & eir::LINKIF != 0 {
                iteration = true;
                driver.check_link_status()?;
                let _ = driver.read_phy(PHIR)?;
            }

            if eir & eir::TXIF != 0 && eir & eir::TXERIF == 0 {
                iteration = true;
                let _ = adapter.tx_skb.lock().take();

                driver.write(ECON1, Command::Bfc, econ1::TXRTS)?;
                driver.netdev().netif_wake_queue();
                driver.write(EIR, Command::Bfc, eir::TXIF)?;
            }

            if eir & eir::TXERIF != 0 {
                iteration = true;
                let _ = adapter.tx_skb.lock().as_mut().take();

                let tsv = driver.read_tsv()?;
                dev_err!(from_dev(&driver.spidev), "TX failed: {:?}\n", tsv);

                driver.write(ECON1, Command::Bfs, econ1::TXRTS)?;
                driver.write(ECON1, Command::Bfc, econ1::TXRTS)?;
                driver.init_txfifo(&TXFIFO_INIT)?;

                driver.netdev().netif_wake_queue();
                driver.write(EIR, Command::Bfc, eir::TXERIF | eir::TXIF)?;
            }

            if eir & eir::RXERIF != 0 {
                iteration = true;
                driver.write(EIR, Command::Bfc, eir::RXERIF)?;
            }

            if driver.handle_rx()? {
                iteration = true;
            }

            iteration
        } {
            iteration = false;
        }

        driver.write(EIE, Command::Bfs, eie::INTIE)
    }();
});

struct TxWorkHandler;

kernel::impl_work_adapter!(TxWorkHandler, Enc28j60Adapter, tx_work, |adapter| {
    let _ = move || -> Result {
        // DeviceOperations::start_xmit stores the RX `SkBuff`
        let skb = adapter.tx_skb.lock().as_ref().unwrap().clone();
        let skb_data = skb.head_data();

        let mut driver = adapter.driver.lock();

        driver.write(EWRPT, Command::Wcr, *TXFIFO_INIT.start())?;
        driver.write(
            ETXND,
            Command::Wcr,
            TXFIFO_INIT.start() + skb_data.len() as u16,
        )?;

        driver.write_buffer(&[0])?;
        driver.write_buffer(skb_data)?;

        driver.write(ECON1, Command::Bfs, econ1::TXRTS)
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
        let adapter = Enc28j60Adapter::try_new(spidev)?;
        adapter.request_irq()?;
        adapter.register_netdev()?;

        Ok(adapter)
    }

    fn remove(spidev: spi::Device, _data: &Self::Data) {
        dev_info!(from_dev(&spidev), "enc28j60rs SPI remove\n");
    }

    fn shutdown(spidev: spi::Device, _data: <Self::Data as ForeignOwnable>::Borrowed<'_>) {
        dev_info!(from_dev(&spidev), "enc28j60rs SPI shutdown\n");
    }
}

module_spi_driver! {
    type: Enc28j60Adapter,
    name: "enc28j60rs",
    author: "Ryo Munakata <ryomnktml@gmail.com>",
    description: "ENC28J60 ethernet driver in Rust",
    license: "GPL",
}
