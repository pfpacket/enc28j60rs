# enc28j60rs
ENC28J60 Linux ethernet driver written in Rust.

Tested with Raspberry Pi 4 Model B + [Linux kernel 6.2.8](https://github.com/pfpacket/linux-rpi-rust/tree/rust-netdev) + Raspberry Pi OS AArch64.

# Kernel
The forked Raspberry Pi kernel with Rust SPI/netdev support: https://github.com/pfpacket/linux-rpi-rust/tree/rust-netdev

# How to build
The kernel tree `KDIR` in `Makefile` requires the Rust metadata to be available. Then:
```
$ make
```
