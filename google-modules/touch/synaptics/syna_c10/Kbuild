
# SPDX-License-Identifier: GPL-2.0-or-later

TCM_CORE=tcm/

ccflags-y += -I$(srctree)/$(src)
ccflags-y += -I$(srctree)/$(src)/$(TCM_CORE)
ccflags-y += -I$(srctree)/../private/google-modules/display
ccflags-y += -I$(srctree)/../private/google-modules/display/samsung/include/uapi
ccflags-y += -I$(srctree)/../private/google-modules/touch/common
ccflags-y += -I$(srctree)/../private/google-modules/touch/common/include
ccflags-y += -I$(srctree)/../private/google-modules/touch/synaptics/
ccflags-y += -I$(srctree)/../private/google-modules/touch/synaptics/tcm

obj-$(CONFIG_TOUCHSCREEN_SYNA_TCM2) = syna_touch.o
syna_touch-objs += \
			syna_tcm2.o \
			$(TCM_CORE)synaptics_touchcom_core_v1.o \
			$(TCM_CORE)synaptics_touchcom_core_v2.o \
			$(TCM_CORE)synaptics_touchcom_func_base.o \
			$(TCM_CORE)synaptics_touchcom_func_touch.o \
			$(TCM_CORE)synaptics_touchcom_func_reflash.o \
			$(TCM_CORE)synaptics_touchcom_func_romboot.o \
			syna_tcm2_platform_spi.o \
			syna_tcm2_sysfs.o \
			syna_tcm2_testing.o
