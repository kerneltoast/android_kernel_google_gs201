# SPDX-License-Identifier: GPL-2.0
#
# Makefile for nfc devices
#

obj-$(CONFIG_NFC_ST21NFC)	+= st21nfc.o
obj-$(CONFIG_ESE_ST54)		+= ese/

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_NFC_ST21NFC=m CONFIG_NFC_ST21NFC_NO_CRYSTAL=y \
		  CONFIG_ESE_ST54=m CONFIG_ESE_ST33=m

ccflags-y := -I$(KERNEL_SRC)/../google-modules/nfc

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) W=1 $(@)
