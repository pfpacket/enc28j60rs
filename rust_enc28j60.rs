// SPDX-License-Identifier: GPL-2.0

use kernel::{module_platform_driver, of, platform, prelude::*, spi};

module_platform_driver! {
    type: EncAdapter,
    name: "rust_enc28j60",
    license: "GPL",
}

struct EncAdapter;

impl platform::Driver for EncAdapter {
    kernel::define_of_id_table! {(), [
        (of::DeviceId::Compatible(b"microchip,enc28j60"), None),
    ]}

    fn probe(_dev: &mut platform::Device, _id_info: Option<&Self::IdInfo>) -> Result<> {
        pr_err!("enc28j60 rust driver platform probe\n");
        Ok(())
    }
}

//impl spi::SpiMethods for EncAdapter {
//    // Let's say you only want a probe and remove method, no shutdown.
//    kernel::declare_spi_methods!(probe, remove);
//
//    /// Define your probe and remove methods. If you don't, default implementations
//    /// will be used instead. These default implementations do *not* correspond to the
//    /// kernel's default implementations! If you wish to use the kernel's default
//    /// SPI functions implementations, do not declare them using the `declare_spi_methods`
//    /// macro. For example, here our driver will use the kernel's `shutdown` method.
//    fn probe(spi_dev: spi::SpiDevice) -> Result {
//        pr_info!("enc28j60 rust driver spi probe\n");
//
//        Ok(())
//    }
//
//    fn remove(spi_dev: spi::SpiDevice) {}
//}
