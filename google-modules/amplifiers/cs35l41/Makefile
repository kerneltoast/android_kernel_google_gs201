# SPDX-License-Identifier: GPL-2.0
snd-soc-cs35l41-objs := cs35l41.o cs35l41-tables.o
snd-soc-cs35l41-spi-objs := cs35l41-spi.o
snd-soc-cs35l41-i2c-objs := cs35l41-i2c.o

obj-$(CONFIG_SND_SOC_CS35L41)		+= snd-soc-cs35l41.o
obj-$(CONFIG_SND_SOC_CS35L41_SPI)	+= snd-soc-cs35l41-spi.o
obj-$(CONFIG_SND_SOC_CS35L41_I2C)	+= snd-soc-cs35l41-i2c.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

EXTRA_CFLAGS += -I$(srctree)/$(src)/include/
EXTRA_CFLAGS += -I$(KERNEL_SRC)/../private/google-modules/amplifiers/snd_soc_wm_adsp
EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/amplifiers/snd_soc_wm_adsp/Module.symvers

KBUILD_OPTIONS += CONFIG_SND_SOC_CS35L41=m \
		  CONFIG_SND_SOC_CS35L41_SPI=m \
		  CONFIG_SND_SOC_CS35L41_I2C=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
