# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for GCIP framework.
#
# Arguments
#   GMODULE_SRC_PATH: The path of directory containing source of google-modules.
#                     (default: $(KERNEL_SRC)/../private/google-modules)

CONFIG_GCIP		?= m
obj-$(CONFIG_GCIP)	+= gcip.o

gcip-objs		:= gcip-alloc-helper.o \
			   gcip-devfreq.o \
			   gcip-dma-fence.o \
			   gcip-domain-pool.o \
			   gcip-fault-injection.o \
			   gcip-fence-array.o \
			   gcip-fence.o \
			   gcip-firmware.o \
			   gcip-image-config.o \
			   gcip-iommu-reserve.o \
			   gcip-iommu.o \
			   gcip-kci.o \
			   gcip-mailbox.o \
			   gcip-mem-pool.o \
			   gcip-pm.o \
			   gcip-resource-accessor.o \
			   gcip-slc.o \
			   gcip-telemetry.o \
			   gcip-thermal.o \
			   gcip-usage-stats.o

gcip-objs		+= iif/iif-dma-fence.o \
			   iif/iif-fence-table.o \
			   iif/iif-fence.o \
			   iif/iif-manager.o \
			   iif/iif-signaler-submission-waiter.o \
			   iif/iif-sync-file.o

CURRENT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# TODO(b/336717718): Remove path of embedded IIF
ccflags-y	+= -I$(CURRENT_DIR)/../../include \
		   -I$(CURRENT_DIR)/iif/include \

# Set the path of google-modules sources if not defined.
GMODULE_SRC_PATH ?= $(KERNEL_SRC)/../private/google-modules

ifneq ($(wildcard $(GMODULE_SRC_PATH)/iif/include),)
ccflags-y	+= -I$(GMODULE_SRC_PATH)/iif/include
endif



modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
