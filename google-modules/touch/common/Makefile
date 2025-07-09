# SPDX-License-Identifier: GPL-2.0

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

EXTRA_CFLAGS	+= -DDYNAMIC_DEBUG_MODULE
EXTRA_SYMBOLS	+= $(OUT_DIR)/../private/google-modules/aoc/Module.symvers
EXTRA_SYMBOLS	+= $(OUT_DIR)/../private/google-modules/display/common/gs_drm/Module.symvers
EXTRA_SYMBOLS	+= $(OUT_DIR)/../private/google-modules/display/samsung/Module.symvers

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install headers_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

modules_install: headers_install
