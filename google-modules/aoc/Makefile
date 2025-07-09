# SPDX-License-Identifier: GPL-2.0

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS	+= CONFIG_AOC_DRIVER=m CONFIG_WC_MBOX=m \
		  CONFIG_AOC_CHAR_DRIVER=m CONFIG_AOC_CHAN_DRIVER=m \
		  CONFIG_AOC_CONTROL_DRIVER=m CONFIG_AOC_UWB_DRIVER=m \
		  CONFIG_AOC_TBN_DRIVER=m CONFIG_AOC_UNIT_TEST_DRIVER=m \

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
