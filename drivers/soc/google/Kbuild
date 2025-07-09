# SPDX-License-Identifier: GPL-2.0-only

ccflags-y := -DDYNAMIC_DEBUG_MODULE

obj-$(CONFIG_GS_CHIPID)			+= gs-chipid.o
obj-$(CONFIG_CAL_IF)            	+= cal-if/
obj-$(CONFIG_EXYNOS_PMU_IF)		+= exynos-pmu-if.o
obj-$(CONFIG_EXYNOS_CPUHP)		+= exynos-cpuhp.o
obj-$(CONFIG_EXYNOS_PD)			+= exynos-pd.o
obj-$(CONFIG_EXYNOS_PD_DBG)		+= exynos-pd-dbg.o
obj-$(CONFIG_EXYNOS_PD_EL3)		+= exynos-pd_el3.o
obj-$(CONFIG_EXYNOS_PD_HSI0)		+= exynos-pd_hsi0.o
obj-$(CONFIG_EXYNOS_PM)			+= exynos-pm.o

# CPU Power Management
obj-$(CONFIG_EXYNOS_CPUPM)		+= exynos-cpupm.o

obj-$(CONFIG_ECT)			+= ect_parser.o

# ACPM
obj-$(CONFIG_GS_ACPM)			+= acpm/

obj-$(CONFIG_EXYNOS_DVFS_MANAGER)       += exynos-dm.o

obj-$(CONFIG_EXYNOS_PM_QOS)		+= exynos_pm_qos.o

obj-$(CONFIG_SLC_PARTITION_MANAGER)	+= pt/

# GSA
obj-$(CONFIG_GSA)			+= gsa/

obj-$(CONFIG_EXYNOS_BCM_DBG)		+= bcm_dbg.o
bcm_dbg-$(CONFIG_EXYNOS_BCM_DBG)	+= exynos-bcm_dbg.o exynos-bcm_dbg-dt.o exynos-bcm_dbg-ppmu.o
obj-$(CONFIG_EXYNOS_BCM_DBG_DUMP)	+= exynos-bcm_dbg-dump.o

# GNSSIF
obj-$(CONFIG_EXYNOS_GNSS_IF_SPI) += gnssif_spi/

obj-y				    	+= debug/

# Emerald Hill
obj-$(CONFIG_GOOGLE_EH) += eh/

# GCMA
obj-$(CONFIG_GCMA) += gcma/

# Proactive Kill
obj-$(CONFIG_PROACTIVE_KILL) += pa_kill/

# Smart Readahead
obj-$(CONFIG_SMRA) += smra/

# S2MPU
obj-$(CONFIG_GS_S2MPU)			+= s2mpu/

obj-$(CONFIG_HARDLOCKUP_DEBUG)		+= hardlockup-debug.o

obj-$(CONFIG_HARDLOCKUP_WATCHDOG)	+= hardlockup-watchdog.o

obj-$(CONFIG_SYSRQ_HOOK)	+= sysrq-hook.o

# vh/kernel/sched
obj-$(CONFIG_VH_KERNEL)			+= vh/kernel/

obj-$(CONFIG_PIXEL_STAT)		+= pixel_stat/

obj-$(CONFIG_DBGCORE_DUMP)		+= dbgcore-dump.o

obj-$(CONFIG_EXYNOS_SECURE_LOG) 	+= exynos-seclog.o

obj-$(CONFIG_PKVM_S2MPU)		+= pkvm-s2mpu/pkvm-s2mpu/

obj-$(CONFIG_PKVM_S2MPU_V9) 		+= pkvm-s2mpu/pkvm-s2mpu-v9/

# perf_trace_counters
obj-$(CONFIG_PERF_TRACE_CTRS)		+= perf_trace_counters.o

obj-$(CONFIG_BOOT_CONTROL)	 	+= boot_control/

obj-$(CONFIG_KERNEL_TOP)		+= kernel-top.o

# Modem control interfaces
obj-$(CONFIG_GOOGLE_MODEMCTL)		+= modemctl/
