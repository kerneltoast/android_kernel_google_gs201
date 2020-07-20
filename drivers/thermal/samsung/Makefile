# SPDX-License-Identifier: GPL-2.0-only
#
# Samsung thermal specific Makefile
#
obj-$(CONFIG_EXYNOS_THERMAL)			+= exynos_thermal.o
exynos_thermal-y				:= exynos_tmu.o

obj-$(CONFIG_GS101_THERMAL_V2)			+= gs101_thermal.o
gs101_thermal-y					+= gs101_tmu_v2.o
gs101_thermal-y					+= exynos_acpm_tmu.o
gs101_thermal-$(CONFIG_EXYNOS_CPU_THERMAL)	+= exynos_cpu_cooling.o
gs101_thermal-$(CONFIG_ISP_THERMAL)		+= isp_cooling.o

obj-$(CONFIG_GPU_THERMAL)			+= gpu_cooling.o
