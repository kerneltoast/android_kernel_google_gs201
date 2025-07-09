# SPDX-License-Identifier: GPL-2.0-or-later
# Makefile for the focaltech touchscreen drivers.

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_FTS=m
EXTRA_CFLAGS	+= -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_PANEL_BRIDGE
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_TBN
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_HEATMAP
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_OFFLOAD
EXTRA_SYMBOLS	+= $(OUT_DIR)/../private/google-modules/touch/common/Module.symvers

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" \
	$(@)

