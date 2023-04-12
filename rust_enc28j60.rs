// SPDX-License-Identifier: GPL-2.0

use {
    core::time::Duration,
    kernel::{
        device::RawDevice, driver, module_spi_driver, of, prelude::*, spi, sync::smutex::Mutex,
        types::ForeignOwnable,
    },
};

mod enc28j60;

const to_dev: fn(&dyn RawDevice) -> kernel::device::Device = kernel::device::Device::from_dev;

struct EncInner {
    bank: enc28j60::Bank,
    spidev: spi::Device,
}

impl EncInner {
    fn read<T: enc28j60::Register>(
        &mut self,
        reg: T,
        command: enc28j60::Command,
    ) -> Result<T::Size> {
        let bank = reg.bank();
        if self.bank != bank {
            enc28j60::set_bank(&self.spidev, bank)?;
            self.bank = bank;
        }

        reg.read(&self.spidev, command)
    }

    fn soft_reset(&self) -> Result {
        self.spidev.write(&[enc28j60::Command::Src as u8])?;
        kernel::delay::coarse_sleep(Duration::from_millis(2));
        Ok(())
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
            bank: enc28j60::Bank::Bank0,
            spidev,
        };

        let revision = enc.read(enc28j60::EREVID, enc28j60::Command::Rcr)?;
        dev_info!(to_dev(&enc.spidev), "ENC28J60 EREVID = {}\n", revision);
        if revision == 0 || revision == 0xff {
            return Err(EINVAL);
        }

        enc.soft_reset()?;
        Ok(EncAdapter {
            inner: Mutex::new(enc),
        })
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
