/* SPDX-License-Identifier: GPL-2.0 */
/*
 * xhci-goog-dma.h - DMA coherent driver for xHCI.
 *
 * Copyright (c) 2024 Google LLC
 *
 * Author: Howard Yen <howardyen@google.com>
 */
#ifndef _XHCI_GOOG_DMA_H
#define _XHCI_GOOG_DMA_H

#define XHCI_GOOG_DMA_RMEM_SRAM 0
#define XHCI_GOOG_DMA_RMEM_DRAM 1
#define XHCI_GOOG_DMA_RMEM_MAX  2

struct xhci_goog_dma_coherent_mem {
	void			*virt_base;
	dma_addr_t		device_base;
	unsigned long		pfn_base;
	int			size;
	unsigned long		*bitmap;
	spinlock_t		spinlock;
	bool			use_dev_dma_pfn_offset;
};

struct xhci_goog_dma_coherent_mem **(*get_dma_coherent_mem)(struct device *dev);
void (*put_dma_coherent_mem)(struct device *dev);

int xhci_goog_rmem_setup_latecall(struct device *dev);
void xhci_goog_setup_dma_ops(struct device *dev);
void xhci_goog_restore_dma_ops(struct device *dev);

int xhci_goog_register_get_cb(
	struct xhci_goog_dma_coherent_mem **(*get_dma_coherent_mem_cb)(struct device *dev));
void xhci_goog_unregister_get_cb(void);

int xhci_goog_register_put_cb(void (*put_dma_coherent_mem_cb)(struct device *dev));
void xhci_goog_unregister_put_cb(void);

#endif 	/* _XHCI_GOOG_DMA_H */
