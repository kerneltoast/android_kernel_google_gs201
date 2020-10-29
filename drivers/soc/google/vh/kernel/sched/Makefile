## SPDX-License-Identifier: GPL-2.0-only

EXTRA_CFLAGS += -I$(srctree)

# vendor sched module
obj-$(CONFIG_VH_SCHED) += vh_sched.o
vh_sched-y += fair.o init.o cpufreq_schedutil.o

# vendor sched tracing module
obj-$(CONFIG_VH_SCHED) += sched_tp.o
