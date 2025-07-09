# SPDX-License-Identifier: GPL-2.0
#
# Makefile for bigwave
#

obj-$(CONFIG_BIGWAVE) += bigwave.o
bigwave-$(CONFIG_BIGWAVE) += bigo.o bigo_pm.o bigo_io.o bigo_of.o bigo_iommu.o bigo_prioq.o
bigwave-$(ENABLE_SLC) += bigo_slc.o
bigwave-$(CONFIG_DEBUG_FS) += bigo_debug.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_BIGWAVE=m ENABLE_SLC=y

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

EXTRA_CFLAGS += -I$(KERNEL_SRC)/../private/google-modules/video/gchips/include
EXTRA_CFLAGS += -DENABLE_SLC=1

modules modules_install headers_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" \
	$(@)

modules_install: headers_install
