lwis-objs := lwis_device.o
lwis-objs += lwis_device_dpm.o
lwis-objs += lwis_device_i2c.o
lwis-objs += lwis_device_ioreg.o
lwis-objs += lwis_device_slc.o
lwis-objs += lwis_device_spi.o
lwis-objs += lwis_device_test.o
lwis-objs += lwis_device_top.o
lwis-objs += lwis_clock.o
lwis-objs += lwis_gpio.o
lwis-objs += lwis_i2c.o
lwis-objs += lwis_spi.o
lwis-objs += lwis_interrupt.o
lwis-objs += lwis_ioctl.o
lwis-objs += lwis_ioctl_past.o
lwis-objs += lwis_ioreg.o
lwis-objs += lwis_periodic_io.o
lwis-objs += lwis_phy.o
lwis-objs += lwis_pinctrl.o
lwis-objs += lwis_regulator.o
lwis-objs += lwis_transaction.o
lwis-objs += lwis_event.o
lwis-objs += lwis_buffer.o
lwis-objs += lwis_util.o
lwis-objs += lwis_debug.o
lwis-objs += lwis_io_entry.o
lwis-objs += lwis_allocator.o
lwis-objs += lwis_version.o
lwis-objs += lwis_fence.o
lwis-objs += lwis_bus_manager.o
lwis-objs += lwis_bus_scheduler.o
lwis-objs += lwis_io_buffer.o

# Anchorage specific files
ifeq ($(CONFIG_SOC_GS101), y)
lwis-objs += platform/anchorage/lwis_platform_anchorage.o
lwis-objs += platform/anchorage/lwis_platform_anchorage_dma.o
endif

# Busan specific files
ifeq ($(CONFIG_SOC_GS201), y)
lwis-objs += platform/busan/lwis_platform_busan.o
lwis-objs += platform/busan/lwis_platform_busan_dma.o
endif

# Casablanca specific files
ifeq ($(CONFIG_SOC_ZUMA), y)
lwis-objs += platform/casablanca/lwis_platform_casablanca.o
lwis-objs += platform/casablanca/lwis_platform_casablanca_dma.o
endif

# Device tree specific file
ifeq ($(CONFIG_OF), y)
lwis-objs += lwis_dt.o
endif

obj-$(CONFIG_LWIS) += lwis.o

ccflags-y += -I$(abspath $(KERNEL_SRC)/$(M)) -I$(abspath $(KBUILD_SRC)/drivers/soc/google)
