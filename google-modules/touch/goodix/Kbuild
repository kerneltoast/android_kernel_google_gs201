# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(srctree)/../private/google-modules/display
ccflags-y += -I$(srctree)/../private/google-modules/display/include/uapi
ccflags-y += -I$(srctree)/../private/google-modules/touch/common
ccflags-y += -I$(srctree)/../private/google-modules/touch/common/include

obj-$(CONFIG_TOUCHSCREEN_GOODIX_BRL) = goodix_brl_touch.o
goodix_brl_touch-objs += \
	goodix_brl_fwupdate.o \
	goodix_brl_hw.o \
	goodix_brl_i2c.o \
	goodix_brl_spi.o \
	goodix_cfg_bin.o \
	goodix_ts_core.o \
	goodix_ts_gesture.o \
	goodix_ts_inspect.o \
	goodix_ts_tools.o \
	goodix_ts_utils.o \
	goodix_ts_proc.o \
	touch_apis.o \
	touch_mf_mode.o
