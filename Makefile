# SPDX-License-Identifier: GPL-2.0
#
# Makefile for gps/broadcom/bcm47765 devices
#

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)


KBUILD_OPTIONS += CONFIG_BCM_GPS_SPI_DRIVER=m
KBUILD_OPTIONS += CONFIG_BCM_GPS_PPS_DRIVER=m

EXTRA_CFLAGS += -DCONFIG_BCM_GPS_SPI_DRIVER

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" modules

modules_install:
	@echo "$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install"
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean $(KBUILD_OPTIONS)
