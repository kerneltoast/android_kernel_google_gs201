# SPDX-License-Identifier: GPL-2.0-only

subdir-ccflags-y += \
		-I$(KERNEL_SRC)/../private/google-modules/bms

obj-$(CONFIG_GOOGLE_BCL) += google_bcl.o
google_bcl-y			+= google_bcl_core.o
google_bcl-y			+= google_bcl_irq_mon.o
google_bcl-y			+= google_bcl_sysfs.o
google_bcl-y			+= google_bcl_util.o
google_bcl-y			+= google_bcl_qos.o
google_bcl-y			+= google_bcl_debugfs.o
google_bcl-y			+= max77759_vdroop.o
google_bcl-y			+= max77779_vdroop.o
google_bcl-y			+= google_bcl_data_logging.o
# ifeq ($(filter y, $(CONFIG_REGULATOR_S2MPG10)),)
ifneq ($(filter m,$(CONFIG_REGULATOR_S2MPG12)$(CONFIG_REGULATOR_S2MPG14)),)
	google_bcl-y			+= google_bcl_votable.o
endif

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/bms/Module.symvers
EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/bms/misc/Module.symvers

ifneq ($(wildcard $(OUT_DIR)/../private/google-modules/radio/samsung/*/Module.symvers),)
EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/radio/samsung/*/Module.symvers
endif

KBUILD_OPTIONS += CONFIG_GOOGLE_BCL=m
include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
