# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the i2c bus drivers.
#

# Embedded system I2C/SMBus host controller drivers
obj-$(CONFIG_I2C_EXYNOS5_GS)	+= i2c-exynos5.o

# Other I2C/SMBus bus drivers
obj-$(CONFIG_I2C_ACPM)		+= i2c-acpm.o

ccflags-$(CONFIG_I2C_DEBUG_BUS) := -DDEBUG
