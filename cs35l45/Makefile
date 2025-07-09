# SPDX-License-Identifier: GPL-2.0
snd-soc-cs35l45-objs := cs35l45.o cs35l45-tables.o
snd-soc-cs35l45-spi-objs := cs35l45-spi.o
snd-soc-cs35l45-i2c-objs := cs35l45-i2c.o
snd-soc-cs35l45-wm-adsp-objs := wm_adsp.o
cs35l45-cs-dsp-objs := cs_dsp.o

obj-$(CONFIG_SND_SOC_CS35L45)		+= snd-soc-cs35l45.o
obj-$(CONFIG_SND_SOC_CS35L45_SPI)	+= snd-soc-cs35l45-spi.o
obj-$(CONFIG_SND_SOC_CS35L45_I2C)	+= snd-soc-cs35l45-i2c.o
obj-$(CONFIG_SND_SOC_CS35L45_WM_ADSP)	+= snd-soc-cs35l45-wm-adsp.o
obj-$(CONFIG_CS35L45_CS_DSP)		+= cs35l45-cs-dsp.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

EXTRA_CFLAGS += -I$(KERNEL_SRC)/$(M)/include/

KBUILD_OPTIONS += CONFIG_SND_SOC_CS35L45=m \
		  CONFIG_SND_SOC_CS35L45_SPI=m \
		  CONFIG_SND_SOC_CS35L45_I2C=m \
		  CONFIG_SND_SOC_CS35L45_WM_ADSP=m \
		  CONFIG_CS35L45_CS_DSP=m \

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
