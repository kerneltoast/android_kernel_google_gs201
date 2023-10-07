# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for GCIP framework.
#

CONFIG_GCIP		?= m
obj-$(CONFIG_GCIP)	+= gcip.o

gcip-objs		:= gcip-alloc-helper.o \
			   gcip-dma-fence.o \
			   gcip-domain-pool.o \
			   gcip-firmware.o \
			   gcip-image-config.o \
			   gcip-kci.o \
			   gcip-mailbox.o \
			   gcip-mem-pool.o \
			   gcip-pm.o \
			   gcip-telemetry.o \
			   gcip-thermal.o

CURRENT_DIR=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))

ccflags-y += -I$(CURRENT_DIR)/../../include

ifdef CONFIG_GCIP_TEST
obj-y	+= unittests/
include $(srctree)/drivers/gcip/unittests/Makefile.include
$(call include_test_path, $(gcip-objs))
endif

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)
