#
#  Makefile for Debug
#

obj-$(CONFIG_DEBUG_SNAPSHOT) 		:= dss.o
dss-y 					+= debug-snapshot.o \
						debug-snapshot-log.o\
						debug-snapshot-utils.o \
						debug-snapshot-dpm.o cache.o
obj-$(CONFIG_DEBUG_SNAPSHOT_BUILT_IN)	+= debug-snapshot-built.o
obj-$(CONFIG_DEBUG_SNAPSHOT_SFRDUMP)	+= debug-snapshot-sfrdump.o
obj-$(CONFIG_DEBUG_SNAPSHOT_QUICKDUMP)	+= debug-snapshot-qd.o

obj-$(CONFIG_EXYNOS_DEBUG_TEST)		+= exynos-debug-test.o

obj-$(CONFIG_PIXEL_DEBUG_TEST)		+= pixel-debug-test.o

obj-$(CONFIG_EXYNOS_ITMON) 		+= gs101-itmon.o
