// SPDX-License-Identifier: GPL-2.0

use kernel::{
    device::RawDevice,
    driver,
    types::ForeignOwnable,
    module_spi_driver,
    of,
    prelude::*,
    spi,
    sync::smutex::Mutex,
};
use core::sync::atomic::{AtomicU8, Ordering};

mod enc28j60;

const to_dev: fn(&dyn RawDevice) -> kernel::device::Device = kernel::device::Device::from_dev;

struct EncInner {
    bank: AtomicU8,
    spidev: spi::Device,
}

impl EncInner {
    fn read<T: enc28j60::Register>(&mut self, reg: T, command: enc28j60::Command) -> Result<T::Size> {
        let bank = reg.bank();
        if self.bank.load(Ordering::Relaxed) != bank as _ {
            enc28j60::set_bank(&self.spidev, bank)?;
            self.bank.store(bank as _, Ordering::Relaxed);
        }

        reg.read(&self.spidev, command)
    }

    fn soft_reset(&self) -> Result {
        self.spidev.write(&[enc28j60::Command::SRC as u8])
    }
}

struct EncAdapter {
    inner: Mutex<EncInner>,
}

impl driver::DeviceRemoval for EncAdapter {
    fn device_remove(&self) {}
}

impl EncAdapter {
    fn try_new(spidev: spi::Device) -> Result<Self> {
        let mut enc = EncInner {
            bank: AtomicU8::new(enc28j60::Bank::Bank0 as _),
            spidev,
        };

        let revision = enc.read(enc28j60::EREVID, enc28j60::Command::RCR)?;
        dev_info!(to_dev(&enc.spidev), "ENC28J60 EREVID = {}\n", revision);
        if revision == 0 || revision == 0xff {
            return Err(EINVAL);
        }

        enc.soft_reset()?;
        Ok(EncAdapter { inner: Mutex::new(enc) })
    }
}

impl spi::Driver for EncAdapter {
    kernel::define_of_id_table! {(), [
        (of::DeviceId::Compatible(b"microchip,enc28j60"), None),
    ]}

    type Data = Box<EncAdapter>;

    fn probe(spidev: spi::Device, _id_info: Option<&Self::IdInfo>) -> Result<Self::Data> {
        Ok(Box::try_new(EncAdapter::try_new(spidev)?)?)
    }

    fn remove(_spidev: spi::Device, _data: &Self::Data) {
        pr_info!("rust spi driver remove\n");
    }

    fn shutdown(_spidev: spi::Device, _data: <Self::Data as ForeignOwnable>::Borrowed<'_>) {
        pr_info!("rust spi driver shutdown\n");
    }
}

module_spi_driver! {
    type: EncAdapter,
    name: "rust_enc28j60",
    description: "ENC28J60 ethernet driver written in Rust",
    license: "GPL",
}
