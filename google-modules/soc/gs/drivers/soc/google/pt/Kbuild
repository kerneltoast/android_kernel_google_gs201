# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the PT API
#

# Needed for the trace points
CFLAGS_pt_trace_points.o = -I$(srctree)/$(src)

obj-$(CONFIG_SLC_PARTITION_MANAGER) += slc_pt.o
slc_pt-$(CONFIG_SLC_PARTITION_MANAGER) += pt.o pt_trace_points.o
obj-$(CONFIG_SLC_PARTITION_MANAGER) += slc_dummy.o
obj-$(CONFIG_GS_SLC_ACPM)	+= slc_acpm.o slc_pmon.o
