## SPDX-License-Identifier: GPL-2.0-only

EXTRA_CFLAGS += -I$(srctree)

obj-$(CONFIG_PIXEL_METRICS) += pixel_metrics.o
pixel_metrics-y += init_metrics.o perf_metrics.o thermal_metrics.o
pixel_metrics-$(CONFIG_CPUIDLE_METRICS) += cpuidle_metrics.o
