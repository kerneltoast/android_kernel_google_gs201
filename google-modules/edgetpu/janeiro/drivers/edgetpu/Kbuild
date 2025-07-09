obj-m	+= janeiro.o
# Use the absolute path of this Makefile to get the source directory.
CURRENT_DIR=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ccflags-y	+= -DCONFIG_EDGETPU_TELEMETRY_TRACE=1 -I$(CURRENT_DIR)/include

GIT_PATH=$(srctree)/$(src)
ifeq ($(shell git -C $(GIT_PATH) rev-parse --is-inside-work-tree),true)
	GIT_REPO_STATE=$(shell (git -C $(GIT_PATH) status --porcelain | grep -q .) && echo -dirty)
	ccflags-y	+= -DGIT_REPO_TAG=\"$(shell git -C $(GIT_PATH) rev-parse --short HEAD)$(GIT_REPO_STATE)\"
else
	ccflags-y	+= -DGIT_REPO_TAG=\"Not\ a\ git\ repository\"
endif
ccflags-y	+= -I$(KERNEL_SRC)/../private/google-modules/power/mitigation -DCONFIG_GOOGLE_BCL

edgetpu-objs	:= edgetpu-mailbox.o edgetpu-telemetry.o edgetpu-mapping.o edgetpu-dmabuf.o edgetpu-async.o edgetpu-iremap-pool.o edgetpu-sw-watchdog.o edgetpu-firmware.o edgetpu-firmware-util.o edgetpu-domain-pool.o

mobile-objs	:= edgetpu-kci.o


janeiro-y	:= janeiro-device.o janeiro-device-group.o janeiro-fs.o janeiro-core.o janeiro-platform.o janeiro-firmware.o janeiro-thermal.o janeiro-pm.o janeiro-usage-stats.o janeiro-iommu.o janeiro-wakelock.o janeiro-external.o $(mobile-objs) $(edgetpu-objs)

CFLAGS_janeiro-fs.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-core.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-device.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-device-group.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-firmware.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-iommu.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-platform.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-pm.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-thermal.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-usage-stats.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-wakelock.o := -DCONFIG_JANEIRO=1
CFLAGS_janeiro-external.o := -DCONFIG_JANEIRO=1
