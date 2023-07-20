# SPDX-License-Identifier: GPL-2.0
#
# Makefile for GXP driver.
#

obj-$(CONFIG_GXP) += gxp.o

gxp-objs +=	\
		gxp-bpm.o \
		gxp-client.o \
		gxp-debug-dump.o \
		gxp-debugfs.o \
		gxp-dmabuf.o \
		gxp-domain-pool.o \
		gxp-doorbell.o \
		gxp-eventfd.o \
		gxp-firmware.o \
		gxp-firmware-data.o \
		gxp-hw-mailbox-driver.o \
		gxp-lpm.o \
		gxp-mailbox.o \
		gxp-mapping.o \
		gxp-mb-notification.o \
		gxp-platform.o \
		gxp-range-alloc.o \
		gxp-pm.o \
		gxp-telemetry.o \
		gxp-thermal.o \
		gxp-vd.o \
		gxp-wakelock.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

# Obtain the current git commit hash for logging on probe
ccflags-y += -DGIT_REPO_TAG=\"Not\ a\ git\ repository\"

# If building via make directly, specify target platform by adding
#     "GXP_PLATFORM=<target>"
# With one of the following values:
#     - CLOUDRIPPER
#     - ZEBU
#     - IP_ZEBU
# Defaults to building for CLOUDRIPPER if not otherwise specified.
GXP_PLATFORM ?= CLOUDRIPPER
GXP_CHIP ?= AMALTHEA

# Setup which version of the gxp-dma interface is used.
# For gem5, need to adopt dma interface without aux domain.
ifeq ($(GXP_PLATFORM), GEM5)
	gxp-objs += gxp-dma-iommu-gem5.o
else
	gxp-objs += gxp-dma-iommu.o
endif

ccflags-y += -DCONFIG_GXP_$(GXP_PLATFORM) -DCONFIG_$(GXP_CHIP)=1 \
	     -I$(M)/include -I$(srctree)/drivers/gxp/include

KBUILD_OPTIONS += CONFIG_GXP=m GXP_CHIP=AMALTHEA

ifdef CONFIG_GXP_TEST
subdir-ccflags-y        += -Wall -Werror -I$(srctree)/drivers/gxp/include
obj-y           += unittests/
include $(srctree)/drivers/gxp/unittests/Makefile.include
$(call include_test_path, $(gxp-objs))
endif

# Access TPU driver's exported symbols.
KBUILD_EXTRA_SYMBOLS += ../google-modules/edgetpu/janeiro/drivers/edgetpu/Module.symvers

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)
