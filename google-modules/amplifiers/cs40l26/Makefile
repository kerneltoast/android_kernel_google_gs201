# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Cirrus Logic haptic driver.
#

cl_dsp-core-objs := cl_dsp.o cl_dsp-debugfs.o
cs40l26-core-objs := cs40l26.o cs40l26-tables.o cs40l26-sysfs.o cs40l26-debugfs.o
snd-soc-cs40l26-objs := cs40l26-codec.o

obj-$(CONFIG_CL_DSP)		+= cl_dsp-core.o
obj-$(CONFIG_INPUT_CS40L26)	+= cs40l26-core.o
obj-$(CONFIG_INPUT_CS40L26_I2C)	+= cs40l26-i2c.o
obj-$(CONFIG_INPUT_CS40L26_SPI)	+= cs40l26-spi.o
obj-$(CONFIG_SND_SOC_CS40L26)	+= snd-soc-cs40l26.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_CL_DSP=m \
		  CONFIG_INPUT_CS40L26=m \
		  CONFIG_INPUT_CS40L26_I2C=m \
		  CONFIG_SND_SOC_CS40L26=m

EXTRA_CFLAGS += -DDYNAMIC_DEBUG_MODULE

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
		$(KBUILD_OPTIONS) \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
		KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" \
		W=1 $(@)

