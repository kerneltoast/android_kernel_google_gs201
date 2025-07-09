M ?= $(shell pwd)

KBASE_PATH_RELATIVE = $(M)

EXTRA_CFLAGS += -Werror

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
