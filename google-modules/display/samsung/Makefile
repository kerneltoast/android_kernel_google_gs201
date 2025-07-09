M ?= $(shell pwd)
O ?= $(OUT_DIR)
OBJTREE ?= $(O)/$(M)

KBASE_PATH_RELATIVE = $(M)

EXTRA_CFLAGS += -DDYNAMIC_DEBUG_MODULE=1

EXTRA_CFLAGS += -Werror

include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

EXTRA_SYMBOLS  += $(OUT_DIR)/../private/google-modules/bms/misc/Module.symvers
EXTRA_SYMBOLS  += $(OUT_DIR)/../private/google-modules/hdcp/samsung/Module.symvers
EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/display/common/gs_drm/Module.symvers

modules modules_install headers_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

modules_install: headers_install
