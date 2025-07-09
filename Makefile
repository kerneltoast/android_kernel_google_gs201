# SPDX-License-Identifier: GPL-2.0
#
# Makefile for GXP driver.
#
# Arguments
#   GMODULE_OUT_PATH: The path of directory containing built google-modules.
#                     This directory is expected to contain *.ko and
#                     Module.symvers files per module. If $(OUT_DIR) is
#                     defined, it will be set to
#                     "$(OUT_DIR)/../private/google-modules".
#
#   GMODULE_SRC_PATH: The path of directory containing source of google-modules.
#                     (default: $(KERNEL_SRC)/../private/google-modules)

GXP_CHIP := AMALTHEA
CONFIG_$(GXP_CHIP) ?= m
GCIP_DIR := gcip-kernel-driver/drivers/gcip
CURRENT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

obj-$(CONFIG_$(GXP_CHIP)) += gxp.o

gxp-objs += \
		gxp-client.o \
		gxp-core-telemetry.o \
		gxp-dci.o \
		gxp-debug-dump.o \
		gxp-dma-fence.o \
		gxp-dma-iommu.o \
		gxp-dmabuf.o \
		gxp-domain-pool.o \
		gxp-doorbell.o \
		gxp-eventfd.o \
		gxp-firmware-data.o \
		gxp-firmware-loader.o \
		gxp-firmware.o \
		gxp-lpm.o \
		gxp-mailbox-manager.o \
		gxp-mailbox.o \
		gxp-mapping.o \
		gxp-mb-notification.o \
		gxp-monitor.o \
		gxp-pm.o \
		gxp-thermal.o \
		gxp-trace.o \
		gxp-vd.o

gxp-mcu-objs := \
		gxp-devfreq.o \
		gxp-kci.o \
		gxp-mcu-firmware.o \
		gxp-mcu-fs.o \
		gxp-mcu-platform.o \
		gxp-mcu-telemetry.o \
		gxp-mcu.o \
		gxp-uci.o \
		gxp-usage-stats.o

gsx01-objs := \
		gxp-bpm.o \
		gxp-cmu.o \
		gxp-gsx01-mailbox.o \
		gxp-gsx01-ssmt.o \
		mobile-soc-gsx01.o

ifeq ($(GXP_CHIP),AMALTHEA)

gxp-objs += \
		$(gsx01-objs) \
		amalthea-platform.o \
		amalthea-pm.o \

EDGETPU_CHIP := janeiro

endif

ifeq ($(CONFIG_$(GXP_CHIP)),m)

gxp-objs += $(GCIP_DIR)/gcip.o

endif

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
# TODO(b/368532067): Remove once tpu-ext.h is properly landed
-include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include
M ?= $(shell pwd)

# Obtain the current git commit hash for logging on probe
GIT_PATH=$(shell cd $(KERNEL_SRC); readlink -e $(M))
GIT_BIN=/usr/bin/git
GIT=$(GIT_BIN) -C $(GIT_PATH)
ifeq ($(shell $(GIT) rev-parse --is-inside-work-tree),true)
        GIT_REPO_STATE=$(shell ($(GIT) --work-tree=$(GIT_PATH) status --porcelain | grep -q .) && echo -dirty)
        ccflags-y       += -DGIT_REPO_TAG=\"$(shell $(GIT) rev-parse --short HEAD)$(GIT_REPO_STATE)\"
endif

# If building via make directly, specify target platform by adding
#     "GXP_PLATFORM=<target>"
# With one of the following values:
#     - SILICON
#     - ZEBU
#     - IP_ZEBU
#     - GEM5
# Defaults to building for SILICON if not otherwise specified.
GXP_PLATFORM ?= SILICON

gxp-flags := -DCONFIG_GXP_$(GXP_PLATFORM) -DCONFIG_$(GXP_CHIP)=1 \
	     -I$(CURRENT_DIR)/include -I$(CURRENT_DIR)/gcip-kernel-driver/include
# TODO(b/336717718): Remove path of embedded IIF
gxp-flags += -I$(CURRENT_DIR)/gcip-kernel-driver/drivers/gcip/iif/include

ccflags-y += $(EXTRA_CFLAGS) $(gxp-flags)
# Flags needed for external modules.
ccflags-y += -DCONFIG_GOOGLE_BCL

KBUILD_OPTIONS += GXP_CHIP=$(GXP_CHIP) GXP_PLATFORM=$(GXP_PLATFORM)

# Set google-modules source and out paths if not defined.
GMODULE_SRC_PATH ?= $(KERNEL_SRC)/../private/google-modules
GMODULE_OUT_PATH ?= $(OUT_DIR)/../private/google-modules

ifneq ($(wildcard $(GMODULE_SRC_PATH)/edgetpu/$(EDGETPU_CHIP)/drivers/edgetpu/include),)
ccflags-y     += -I$(GMODULE_SRC_PATH)/edgetpu/$(EDGETPU_CHIP)/drivers/edgetpu/include
endif

# Access TPU driver's exported symbols.
ifneq ($(wildcard $(GMODULE_OUT_PATH)/edgetpu/$(EDGETPU_CHIP)/drivers/edgetpu/Module.symvers),)
EXTRA_SYMBOLS += $(GMODULE_OUT_PATH)/edgetpu/$(EDGETPU_CHIP)/drivers/edgetpu/Module.symvers
endif

ifneq ($(wildcard $(GMODULE_OUT_PATH)/soc/gs/drivers/soc/google/gsa/Module.symvers),)
EXTRA_SYMBOLS += $(GMODULE_OUT_PATH)/soc/gs/drivers/soc/google/gsa/Module.symvers
endif

ifneq ($(GXP_POWER_MITIGATION), false)
ifneq ($(wildcard $(GMODULE_SRC_PATH)/power/mitigation),)
ccflags-y     += -I$(GMODULE_SRC_PATH)/power/mitigation
endif
ifneq ($(wildcard $(GMODULE_OUT_PATH)/power/mitigation/Module.symvers),)
EXTRA_SYMBOLS += $(GMODULE_OUT_PATH)/power/mitigation/Module.symvers
endif
endif # GXP_POWER_MITIGATION

ifneq ($(wildcard $(GMODULE_SRC_PATH)/iif/include),)
ccflags-y     += -I$(GMODULE_SRC_PATH)/iif/include
endif

ifneq ($(wildcard $(GMODULE_OUT_PATH)/iif/Module.symvers),)
EXTRA_SYMBOLS += $(GMODULE_OUT_PATH)/iif/Module.symvers
endif

ifneq ($(wildcard $(GMODULE_SRC_PATH)/perf/include),)
ccflags-y     += -I$(GMODULE_SRC_PATH)/perf/include
endif

ifneq ($(wildcard $(GMODULE_OUT_PATH)/perf/google_pm_qos_Module.symvers),)
EXTRA_SYMBOLS += $(GMODULE_OUT_PATH)/perf/google_pm_qos_Module.symvers
endif

modules modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(M)/$(GCIP_DIR) gcip.o
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M)/$(GCIP_DIR) $(@)
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)
