# SPDX-License-Identifier: GPL-2.0

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/fingerprint/goodix/fps_touch_handler
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/gs/soc/include

EXTRA_SYMBOLS	+= $(OUT_DIR)/../private/google-modules/touch/common/Module.symvers

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" \
	$(@)
