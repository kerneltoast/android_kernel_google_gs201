# SPDX-License-Identifier: GPL-2.0

subdir-ccflags-y += \
		-I$(srctree)/$(src)/include \
		-I$(srctree)/$(src)/include/uapi \

obj-y += drivers/phy/

obj-y += drivers/pinctrl/

ifeq ($(CONFIG_SOC_ZUMA),y)
obj-y += drivers/pci/controller/dwc/
else
obj-y += drivers/pci/controller/dwc-whi/
endif

obj-y += drivers/clk/gs/

obj-y += drivers/dma/

obj-y += drivers/soc/google/

obj-y += drivers/regulator/

obj-y += drivers/tty/serial/

obj-y += drivers/char/hw_random/

obj-y += drivers/iommu/

obj-y += drivers/gpu/

obj-y += drivers/misc/

obj-y += drivers/mfd/

obj-y += drivers/dma-buf/heaps/

obj-y += drivers/ufs/

obj-y += drivers/spi/

obj-y += drivers/spmi/

obj-y += drivers/usb/

obj-y += drivers/input/

obj-y += drivers/rtc/

obj-y += drivers/i2c/busses/

obj-y += drivers/media/platform/

obj-y += drivers/power/

obj-y += drivers/thermal/

obj-y += drivers/watchdog/

obj-y += drivers/cpufreq/

obj-y += drivers/clocksource/

ifeq ($(CONFIG_SOC_ZUMA),y)
obj-y += drivers/devfreq/google/
else
obj-y += drivers/devfreq-whi/google/
endif

obj-y += drivers/performance/gs_perf_mon/

obj-y += drivers/performance/lat_governors/

obj-y += drivers/iio/

obj-y += drivers/bts/

obj-y += drivers/block/zram/

obj-y += drivers/pwm/

obj-y += drivers/video/backlight/
