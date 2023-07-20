# SPDX-License-Identifier: GPL-2.0
#
# Makefile for EdgeTPU framework and dependent drivers.
#

obj-$(CONFIG_JANEIRO)		+= janeiro.o

ccflags-y       += -DGIT_REPO_TAG=\"Not\ a\ git\ repository\"

edgetpu-objs	:= edgetpu-async.o edgetpu-dmabuf.o edgetpu-iremap-pool.o \
		   edgetpu-kci.o edgetpu-mailbox.o edgetpu-mapping.o \
		   edgetpu-sw-watchdog.o edgetpu-telemetry.o \
		   edgetpu-firmware-util.o edgetpu-firmware.o \
		   edgetpu-domain-pool.o

janeiro-objs	:= janeiro-core.o janeiro-debug-dump.o janeiro-device-group.o \
		   janeiro-device.o janeiro-firmware.o janeiro-fs.o \
		   janeiro-iommu.o janeiro-platform.o janeiro-pm.o \
		   janeiro-thermal.o janeiro-usage-stats.o janeiro-wakelock.o \
		   janeiro-external.o \
		   $(edgetpu-objs)


KBUILD_OPTIONS += CONFIG_JANEIRO=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)
