obj-$(CONFIG_TOUCHSCREEN_TBN)		+= touch_bus_negotiator.o
obj-$(CONFIG_TOUCHSCREEN_HEATMAP)	+= heatmap.o
obj-$(CONFIG_TOUCHSCREEN_OFFLOAD)	+= touch_offload.o
obj-$(CONFIG_GOOG_TOUCH_INTERFACE)	+= goog_touch_interface.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_TBN=m
KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_HEATMAP=m
KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_OFFLOAD=m
KBUILD_OPTIONS	+= CONFIG_GOOG_TOUCH_INTERFACE=m
EXTRA_CFLAGS	+= -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/touch/common/include
EXTRA_CFLAGS    += -I$(KERNEL_SRC)/../google-modules/display

modules clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(@)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(@)

headers_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(@)
