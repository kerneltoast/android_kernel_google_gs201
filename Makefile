# SPDX-License-Identifier: GPL-2.0
#
# Makefile for nfc devices
#

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_NFC_ST21NFC=m CONFIG_NFC_ST21NFC_NO_CRYSTAL=y \
		  CONFIG_ESE_ST54=m CONFIG_ESE_ST33=m

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
