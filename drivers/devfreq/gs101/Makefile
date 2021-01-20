# Exynos DEVFREQ Drivers

ccflags-y := -DDYNAMIC_DEBUG_MODULE

obj-$(CONFIG_ARM_EXYNOS_DEVFREQ)		+= exynos_devfreq.o
exynos_devfreq-$(CONFIG_ARM_EXYNOS_DEVFREQ)	+= gs101-devfreq.o governor_simpleinteractive.o
exynos_devfreq-$(CONFIG_EXYNOS_ALT_DVFS)	+= gs101-ppc.o
