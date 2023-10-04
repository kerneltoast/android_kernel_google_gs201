obj-$(CONFIG_AUDIOMETRICS)     += audiometrics.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
CUSTOM_INCLUDE ?= -I$(shell pwd)/include/

KBUILD_OPTIONS += CONFIG_AUDIOMETRICS=m

headers-y += uapi/audiometrics_api.h

#		  ccflags-y='$(CUSTOM_INCLUDE)'

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) W=1 $(@)