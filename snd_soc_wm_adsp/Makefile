# SPDX-License-Identifier: GPL-2.0
#
snd-soc-wm-adsp-objs := wm_adsp.o

obj-$(CONFIG_SND_SOC_WM_ADSP)	+= snd-soc-wm-adsp.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_SND_SOC_WM_ADSP=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) W=1 $(@)
