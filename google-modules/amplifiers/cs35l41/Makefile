# SPDX-License-Identifier: GPL-2.0
snd-soc-cs35l41-objs := cs35l41.o cs35l41-tables.o
snd-soc-cs35l41-spi-objs := cs35l41-spi.o
snd-soc-cs35l41-i2c-objs := cs35l41-i2c.o
snd-soc-wm-adsp-objs := wm_adsp.o

obj-$(CONFIG_SND_SOC_CS35L41)		+= snd-soc-cs35l41.o
obj-$(CONFIG_SND_SOC_CS35L41_SPI)	+= snd-soc-cs35l41-spi.o
obj-$(CONFIG_SND_SOC_CS35L41_I2C)	+= snd-soc-cs35l41-i2c.o
obj-$(CONFIG_SND_SOC_WM_ADSP)		+= snd-soc-wm-adsp.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
CUSTOM_INCLUDE ?= -I$(shell pwd)/include/

KBUILD_OPTIONS += CONFIG_SND_SOC_CS35L41=m \
		  CONFIG_SND_SOC_CS35L41_SPI=m \
		  CONFIG_SND_SOC_CS35L41_I2C=m \
		  CONFIG_SND_SOC_WM_ADSP=m \
		  ccflags-y='$(CUSTOM_INCLUDE)'

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) W=1 $(@)
