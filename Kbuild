# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(srctree)/$(src)/include
ccflags-y += -I$(srctree)/../private/google-modules/display
ccflags-y += -I$(srctree)/../private/google-modules/display/common/include
ccflags-y += -I$(srctree)/../private/google-modules/display/samsung/include/uapi
ccflags-y += -I$(srctree)/../private/google-modules/aoc
obj-$(CONFIG_TOUCHSCREEN_TBN)		+= touch_bus_negotiator.o
obj-$(CONFIG_TOUCHSCREEN_HEATMAP)	+= heatmap.o
obj-$(CONFIG_TOUCHSCREEN_OFFLOAD)	+= touch_offload.o
obj-$(CONFIG_GOOG_TOUCH_INTERFACE)	+= goog_touch_interface.o

