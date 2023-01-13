#
#  Makefile for Debug
#

ccflags-y := -DDYNAMIC_DEBUG_MODULE

obj-$(CONFIG_DEBUG_SNAPSHOT) 		:= dss.o
dss-y 					+= debug-snapshot.o \
						debug-snapshot-log.o\
						debug-snapshot-utils.o \
						debug-snapshot-dpm.o cache.o
obj-$(CONFIG_DEBUG_SNAPSHOT_BUILT_IN)		+= debug-snapshot-built.o
obj-$(CONFIG_DEBUG_SNAPSHOT_SFRDUMP)		+= debug-snapshot-sfrdump.o
obj-$(CONFIG_DEBUG_SNAPSHOT_QUICKDUMP)		+= debug-snapshot-qd.o
obj-$(CONFIG_DEBUG_SNAPSHOT_DEBUG_KINFO)	+= debug-snapshot-debug-kinfo.o

obj-$(CONFIG_EXYNOS_EHLD)		+= ehld.o
ehld-y					:= exynos-ehld.o \
						exynos-ehld-ipc.o

obj-$(CONFIG_EXYNOS_DEBUG_TEST)		+= exynos-debug-test.o

obj-$(CONFIG_PIXEL_DEBUG_TEST)		+= pixel-debug-test.o

obj-$(CONFIG_EXYNOS_ITMON) 		+= itmon.o
itmon-$(CONFIG_SOC_GS101) 		+= gs101-itmon.o
itmon-$(CONFIG_SOC_GS201) 		+= gs201-itmon.o

obj-$(CONFIG_EXYNOS_CORESIGHT)		+= exynos-coresight.o

obj-$(CONFIG_EXYNOS_ECC_HANDLER)	+= exynos-ecc-handler.o

obj-$(CONFIG_EXYNOS_CORESIGHT_ETM)	+= exynos-coresight-etm.o

obj-$(CONFIG_EXYNOS_ADV_TRACER) 	+= exynos-adv-tracer.o

obj-$(CONFIG_EXYNOS_ADV_TRACER_S2D)	+= exynos-adv-tracer-s2d.o

obj-$(CONFIG_PIXEL_BOOT_METRICS)	+= pixel-boot-metrics.o

obj-$(CONFIG_GS_SJTAG)			+= sjtag-driver.o

obj-$(CONFIG_ETM2DRAM)			+= etm2dram.o
