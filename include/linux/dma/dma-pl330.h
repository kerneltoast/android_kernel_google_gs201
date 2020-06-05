/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 */

#ifndef __DMA_PL330_H_
#define __DMA_PL330_H_ __FILE__

#include <linux/dmaengine.h>

/*
 * PL330 can assign any channel to communicate with
 * any of the peripherals attached to the DMAC.
 * For the sake of consistency across client drivers,
 * We keep the channel names unchanged and only add
 * missing peripherals are added.
 * Order is not important since DMA PL330 API driver
 * use these just as IDs.
 */
enum dma_ch {
	DMACH_MAX = 0,
};

struct s3c2410_dma_client {
	char	*name;
};

static inline bool samsung_dma_has_circular(void)
{
	return true;
}

static inline bool samsung_dma_is_dmadev(void)
{
	return true;
}

static inline bool samsung_dma_has_infiniteloop(void)
{
	return true;
}

struct samsung_dma_req {
	enum dma_transaction_type cap;
	struct s3c2410_dma_client *client;
};

struct samsung_dma_prep {
	enum dma_transaction_type cap;
	enum dma_transfer_direction direction;
	dma_addr_t buf;
	unsigned long period;
	unsigned long len;
	void (*fp)(void *data);
	void *fp_param;
	unsigned int infiniteloop;
};

struct samsung_dma_config {
	enum dma_transfer_direction direction;
	enum dma_slave_buswidth width;
	u32 maxburst;
	dma_addr_t fifo;
};

struct samsung_dma_ops {
	unsigned long (*request)(enum dma_ch ch, struct samsung_dma_req *param,
				 struct device *dev, char *ch_name);
	int (*release)(unsigned long ch, void *param);
	int (*config)(unsigned long ch, struct samsung_dma_config *param);
	int (*prepare)(unsigned long ch, struct samsung_dma_prep *param);
	int (*trigger)(unsigned long ch);
	int (*started)(unsigned long ch);
	int (*getposition)(unsigned long ch, dma_addr_t *src, dma_addr_t *dst);
	int (*flush)(unsigned long ch);
	int (*stop)(unsigned long ch);
	int (*debug)(unsigned long ch);
};

/*
 * samsung_dma_get_ops
 * get the set of samsung dma operations
 */
#if IS_ENABLED(CONFIG_SAMSUNG_DMADEV)
extern void *samsung_dmadev_get_ops(void);
extern void *s3c_dma_get_ops(void);

static inline void *samsung_dma_get_ops(void)
{
	if (samsung_dma_is_dmadev())
		return samsung_dmadev_get_ops();
	else
		return s3c_dma_get_ops();
}
#else
#define samsung_dma_get_ops() NULL
#endif

#endif	/* __DMA_PL330_H_ */

