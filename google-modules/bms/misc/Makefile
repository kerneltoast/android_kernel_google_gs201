# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Google Battery Management System
#

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

EXTRA_CFLAGS += -DDYNAMIC_DEBUG_MODULE

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" $(@)

print-%:
	@echo $* = $($*)

value-%:
	@echo $($*)
