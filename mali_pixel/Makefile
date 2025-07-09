# SPDX-License-Identifier: GPL-2.0
#
# Copyright 2021 Google LLC
#

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_MALI_MEMORY_GROUP_MANAGER=m
KBUILD_OPTIONS += CONFIG_MALI_PRIORITY_CONTROL_MANAGER=m
KBUILD_OPTIONS += CONFIG_MALI_PROTECTED_MEMORY_ALLOCATOR=m
KBUILD_OPTIONS += CONFIG_MALI_PIXEL_STATS=m
KBUILD_OPTIONS += CONFIG_MALI_PIXEL_GPU_SLC=y

KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any

EXTRA_CFLAGS += -I$(M)
EXTRA_CFLAGS += -I$(M)/../common/include

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
