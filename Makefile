obj-$(CONFIG_WC_MBOX) += mailbox-wc.o

obj-$(CONFIG_AOC_DRIVER)	+= aoc_core.o
aoc_core-objs := aoc.o ../aoc_ipc/aoc_ipc_core.o aoc_firmware.o ion_physical_heap.o

obj-$(CONFIG_AOC_CHAR_DRIVER)   += aoc_char_dev.o
obj-$(CONFIG_AOC_CONTROL_DRIVER)   += aoc_control_dev.o
obj-$(CONFIG_AOC_CHAN_DRIVER)   += aoc_channel_dev.o
obj-$(CONFIG_AOC_UWB_DRIVER)   += aoc_uwb_platform_drv.o aoc_uwb_service_dev.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_AOC_DRIVER=m CONFIG_WC_MBOX=m \
		  CONFIG_AOC_CHAR_DRIVER=m CONFIG_AOC_CHAN_DRIVER=m \
		  CONFIG_AOC_CONTROL_DRIVER=m CONFIG_AOC_UWB_DRIVER=m \

ccflags-y := -I$(KERNEL_SRC)/../google-modules/aoc_ipc
headers-y := uapi/aoc.h

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) W=1 $(@)
