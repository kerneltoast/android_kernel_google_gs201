# SPDX-License-Identifier: GPL-2.0

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
O ?= $(OUT_DIR)
OBJTREE ?= $(O)/$(M)

EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/bms/misc/Module.symvers
EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/trusty/Module.symvers

modules modules_install headers_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

modules_install: modules_preinstall headers_install

include $(KERNEL_SRC)/$(M)/Makefile.preinstall
