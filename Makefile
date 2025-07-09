KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_LWIS=m

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

KBUILD_CFLAGS += -Wall -Werror

ifeq ($(CONFIG_GCOV_KERNEL),y)
    KBUILD_CFLAGS += $(call cc-option, -ftest-coverage)
    KBUILD_CFLAGS += $(call cc-option, -fprofile-arcs)
    EXTRA_CFLAGS += -DGCOV_PROFILE=1
endif

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
