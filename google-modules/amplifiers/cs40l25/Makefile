# SPDX-License-Identifier: GPL-2.0
#
# Makefile for misc devices that really don't fit anywhere else.
#
haptics-cs40l2x-objs := cs40l2x.o cs40l2x-tables.o
obj-$(CONFIG_HAPTICS_CS40L2X)	+= haptics-cs40l2x.o

snd_soc_cs40l2x-objs := snd-soc-codec-cs40l2x.o
obj-$(CONFIG_SND_SOC_CS40L2X)	+= snd_soc_cs40l2x.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
CUSTOM_INCLUDE ?= -I$(shell pwd)/include/

KBUILD_OPTIONS += CONFIG_HAPTICS_CS40L2X=m \
		  CONFIG_SND_SOC_CS40L2X=m \
		  ccflags-y='$(CUSTOM_INCLUDE)'

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) W=1 $(@)




