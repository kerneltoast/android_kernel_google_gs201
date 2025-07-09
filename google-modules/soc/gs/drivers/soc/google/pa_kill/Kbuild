# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Procative Kill

# Needed for the trace points
CFLAGS_pa_kill_core.o = -I$(srctree)/$(src)

obj-$(CONFIG_PROACTIVE_KILL) += pa_kill.o

pa_kill-y                    := pa_kill_core.o pa_kill_sysfs.o
