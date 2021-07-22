## SPDX-License-Identifier: GPL-2.0-only

EXTRA_CFLAGS += -I$(srctree)

# vendor sched module
obj-$(CONFIG_VH_SCHED) += vh_sched.o
vh_sched-y += core.o fair.o init.o sysfs_node.o rt.o cpufreq_gov.o

# vendor sched tracing module
obj-$(CONFIG_VH_SCHED) += sched_tp.o

obj-$(CONFIG_VH_PREEMPTIRQ_TRACEPOINTS) += vh_preemptirq_long.o
vh_preemptirq_long-y = init_preemptirq_long.o preemptirq_long.o
