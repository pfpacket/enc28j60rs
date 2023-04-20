# SPDX-License-Identifier: GPL-2.0

KDIR ?= /home/devm33/pkgbuild/linux-rpi-rust-git/src/linux/
ARCH=arm64

ifeq ($(ARCH), arm)
CROSS_COMPILE=arm-linux-gnueabihf-
else ifeq ($(ARCH), arm64)
CROSS_COMPILE=aarch64-linux-gnu-
else
$(error Unsupported architecture: $(ARCH))
endif

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) LLVM=1 -C $(KDIR) M=$$PWD

macro_expand: clean
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) LLVM=1 -C $(KDIR) M=$$PWD RUSTFLAGS_MODULE="-Zunpretty=expanded"

clean:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) LLVM=1 -C $(KDIR) M=$$PWD clean

.PHONY: macro_expand clean
