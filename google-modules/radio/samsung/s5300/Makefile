KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KCFLAGS += -I$(KERNEL_SRC)/../private/google-modules/soc/gs/include
KCFLAGS += -I$(KERNEL_SRC)/../private/google-modules/soc/gs/include/uapi

EXTRA_SYMBOLS += $(O)/../private/google-modules/soc/gs/Module.symvers
EXTRA_SYMBOLS += $(O)/../private/google-modules/bms/misc/Module.symvers

EXTRA_CFLAGS+="-Wno-missing-prototypes"

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
