#
# ASoC-based USB driver
#
obj-$(CONFIG_AOC_DRIVER) += aoc_usb_driver.o
aoc_usb_driver-objs := aoc_usb_dev.o xhci_hooks_impl_whi.o usb_hooks_impl_whi.o snd_usb_audio_hook_impl_whi.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
KBUILD_OPTIONS += CONFIG_AOC_DRIVER=m
EXTRA_CFLAGS=-I$(KERNEL_SRC)/../google-modules/aoc_ipc -I$(KERNEL_SRC)/../google-modules/aoc -I$(KERNEL_SRC)/drivers/usb/host -I$(KERNEL_SRC)/sound/usb
KBUILD_EXTRA_SYMBOLS=$(OUT_DIR)/../google-modules/aoc/Module.symvers\
		     $(OUT_DIR)/../google-modules/aoc/alsa/Module.symvers

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(KBUILD_EXTRA_SYMBOLS)" $(@)
