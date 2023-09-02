# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Cirrus Logic haptic driver.
#

input-cs40l26-i2c-objs := cs40l26.o cs40l26-tables.o cs40l26-sysfs.o cs40l26-i2c.o
input-cs40l26-spi-objs := cs40l26.o cs40l26-tables.o cs40l26-sysfs.o cs40l26-spi.o
snd-soc-cs40l26-objs := cs40l26-codec.o

obj-$(CONFIG_INPUT_CS40L26_I2C)	+= input-cs40l26-i2c.o
obj-$(CONFIG_INPUT_CS40L26_SPI)	+= input-cs40l26-spi.o
obj-$(CONFIG_CIRRUS_FIRMWARE_CL_DSP)	+= cl_dsp.o
obj-$(CONFIG_SND_SOC_CS40L26)		+= snd-soc-cs40l26.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_INPUT_CS40L26_I2C=m \
		  CONFIG_CIRRUS_FIRMWARE_CL_DSP=m \
		  CONFIG_SND_SOC_CS40L26=m

EXTRA_CFLAGS += -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS += -DCONFIG_INPUT_CS40L26_ATTR_UNDER_BUS
EXTRA_CFLAGS += -DCONFIG_GOOG_CUST

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
		$(KBUILD_OPTIONS) \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
		W=1 $(@)

