# SPDX-License-Identifier: GPL-2.0

subdir-ccflags-y += -I$(srctree)/drivers/dma

#dmaengine debug flags
subdir-ccflags-$(CONFIG_DMADEVICES_DEBUG)  := -DDEBUG
subdir-ccflags-$(CONFIG_DMADEVICES_VDEBUG) += -DVERBOSE_DEBUG

#devices
obj-$(CONFIG_PL330_DMA_GS) += pl330.o
obj-$(CONFIG_SAMSUNG_DMADEV) += samsung-dma.o
