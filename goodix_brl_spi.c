/*
 * Goodix Touchscreen Driver
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#ifdef CONFIG_GOOG_TOUCH_INTERFACE
#include <goog_touch_interface.h>
#endif

#include "goodix_ts_core.h"
#define TS_DRIVER_NAME "gtx8_spi"

#define SPI_TRANS_PREFIX_LEN 1
#define REGISTER_WIDTH 4
#define SPI_READ_DUMMY_LEN 4
#define SPI_READ_PREFIX_LEN                                                    \
	(SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH + SPI_READ_DUMMY_LEN)
#define SPI_WRITE_PREFIX_LEN (SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH)
#define SPI_PREALLOC_RX_BUF_SIZE 4096 + SPI_READ_PREFIX_LEN
#define SPI_PREALLOC_TX_BUF_SIZE 4096 + SPI_WRITE_PREFIX_LEN

#define SPI_WRITE_FLAG 0xF0
#define SPI_READ_FLAG 0xF1

/**
 * goodix_spi_read_bra- read device register through spi bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: read buffer
 * @len: bytes to read
 * return: 0 - read ok, < 0 - spi transter error
 */
static int goodix_spi_read_bra(struct device *dev, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct spi_device *spi = to_spi_device(dev);
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	struct goodix_bus_interface *bus = cd->bus;
	u8 *rx_buf = NULL;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	int buf_len = SPI_READ_PREFIX_LEN + len;

	mutex_lock(&bus->mutex);

	if (buf_len <= SPI_PREALLOC_RX_BUF_SIZE &&
		buf_len <= SPI_PREALLOC_TX_BUF_SIZE) {
		rx_buf = bus->rx_buf;
		tx_buf = bus->tx_buf;
		memset(tx_buf, 0, buf_len);
	} else {
		rx_buf = kzalloc(buf_len, GFP_KERNEL);
		if (!rx_buf) {
			ts_err("alloc rx_buf failed, size:%d", buf_len);
			ret = -ENOMEM;
			goto err_alloc_rx_buf;
		}

		tx_buf = kzalloc(buf_len, GFP_KERNEL);
		if (!tx_buf) {
			ts_err("alloc tx_buf failed, size:%d", buf_len);
			ret = -ENOMEM;
			goto err_alloc_tx_buf;
		}
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	/*spi_read tx_buf format: 0xF1 + addr(4bytes) + data*/
	tx_buf[0] = SPI_READ_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	tx_buf[5] = 0xFF;
	tx_buf[6] = 0xFF;
	tx_buf[7] = 0xFF;
	tx_buf[8] = 0xFF;

	xfers.tx_buf = tx_buf;
	xfers.rx_buf = rx_buf;
	xfers.len = buf_len;
	xfers.cs_change = 0;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		ts_err("spi transfer error:%d", ret);
		goto err_spi_transfer;
	}
	memcpy(data, &rx_buf[SPI_READ_PREFIX_LEN], len);

err_spi_transfer:
	if (tx_buf != bus->tx_buf)
		kfree(tx_buf);
err_alloc_tx_buf:
	if (rx_buf != bus->rx_buf)
		kfree(rx_buf);
err_alloc_rx_buf:
	mutex_unlock(&bus->mutex);
	return ret;
}

static int goodix_spi_read(struct device *dev, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct spi_device *spi = to_spi_device(dev);
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	struct goodix_bus_interface *bus = cd->bus;
	u8 *rx_buf = NULL;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	int buf_len = SPI_READ_PREFIX_LEN - 1 + len;

	if (bus->dma_mode_enabled && buf_len >= 64)
		buf_len = ALIGN(buf_len, 4);

	mutex_lock(&bus->mutex);

	if (buf_len <= SPI_PREALLOC_RX_BUF_SIZE &&
		buf_len <= SPI_PREALLOC_TX_BUF_SIZE) {
		rx_buf = bus->rx_buf;
		tx_buf = bus->tx_buf;
		memset(tx_buf, 0, buf_len);
	} else {
		rx_buf = kzalloc(buf_len, GFP_KERNEL);
		if (!rx_buf) {
			ts_err("alloc rx_buf failed, size:%d", buf_len);
			ret = -ENOMEM;
			goto err_alloc_rx_buf;
		}

		tx_buf = kzalloc(buf_len, GFP_KERNEL);
		if (!tx_buf) {
			ts_err("alloc tx_buf failed, size:%d", buf_len);
			ret = -ENOMEM;
			goto err_alloc_tx_buf;
		}
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	/*spi_read tx_buf format: 0xF1 + addr(4bytes) + data*/
	tx_buf[0] = SPI_READ_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	tx_buf[5] = 0xFF;
	tx_buf[6] = 0xFF;
	tx_buf[7] = 0xFF;

	xfers.tx_buf = tx_buf;
	xfers.rx_buf = rx_buf;
	xfers.len = buf_len;
	xfers.cs_change = 0;
	if (bus->dma_mode_enabled)
		xfers.bits_per_word = buf_len >= 64 ? 32 : 8;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		ts_err("spi transfer error:%d", ret);
		goto err_spi_transfer;
	}
	memcpy(data, &rx_buf[SPI_READ_PREFIX_LEN - 1], len);

err_spi_transfer:
	if (tx_buf != bus->tx_buf)
		kfree(tx_buf);
err_alloc_tx_buf:
	if (rx_buf != bus->rx_buf)
		kfree(rx_buf);
err_alloc_rx_buf:
	mutex_unlock(&bus->mutex);
	return ret;
}

/* [GOOG]
 * This SPI transaction will direct read into `struct goodix_rx_package`.
 * And, the package are comprised of `SPI prefix header` + `data`.
 */
static int goodix_spi_read_fast(struct device *dev, unsigned int addr,
	struct goodix_rx_package *package, unsigned int len)
{
	struct spi_device *spi = to_spi_device(dev);
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	struct goodix_bus_interface *bus = cd->bus;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	int buf_len = SPI_READ_PREFIX_LEN - 1 + len;

	if (bus->dma_mode_enabled && buf_len >= 64)
		buf_len = ALIGN(buf_len, 4);

	if (buf_len <= SPI_PREALLOC_TX_BUF_SIZE) {
		tx_buf = bus->tx_buf;
	} else {
		tx_buf = kzalloc(buf_len, GFP_KERNEL);
		if (!tx_buf) {
			ts_err("alloc tx_buf failed, size:%d", buf_len);
			return -ENOMEM;
		}
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	mutex_lock(&bus->mutex);

	/*spi_read tx_buf format: 0xF1 + addr(4bytes) + data*/
	tx_buf[0] = SPI_READ_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	tx_buf[5] = 0xFF;
	tx_buf[6] = 0xFF;
	tx_buf[7] = 0xFF;

	xfers.tx_buf = tx_buf;
	xfers.rx_buf = package->header;
	xfers.len = buf_len;
	xfers.cs_change = 0;
	if (bus->dma_mode_enabled)
		xfers.bits_per_word = buf_len >= 64 ? 32 : 8;
	spi_message_add_tail(&xfers, &spi_msg);

	ret = spi_sync(spi, &spi_msg);

	mutex_unlock(&bus->mutex);

	if (ret < 0) {
		ts_err("spi transfer error:%d", ret);
		goto err_spi_transfer;
	}

err_spi_transfer:
	if (tx_buf != bus->tx_buf)
		kfree(tx_buf);
	return ret;
}

/**
 * goodix_spi_write- write device register through spi bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: write buffer
 * @len: bytes to write
 * return: 0 - write ok; < 0 - spi transter error.
 */
static int goodix_spi_write(struct device *dev, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct spi_device *spi = to_spi_device(dev);
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	struct goodix_bus_interface *bus = cd->bus;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	int buf_len = SPI_WRITE_PREFIX_LEN + len;

	if (bus->dma_mode_enabled && buf_len >= 64)
		buf_len = ALIGN(buf_len, 4);

	if (buf_len <= SPI_PREALLOC_TX_BUF_SIZE) {
		tx_buf = bus->tx_buf;
	} else {
		tx_buf = kzalloc(buf_len, GFP_KERNEL);
		if (!tx_buf) {
			ts_err("alloc tx_buf failed, size:%d", buf_len);
			return -ENOMEM;
		}
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	mutex_lock(&bus->mutex);

	tx_buf[0] = SPI_WRITE_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	memcpy(&tx_buf[SPI_WRITE_PREFIX_LEN], data, len);
	xfers.tx_buf = tx_buf;
	xfers.len = buf_len;
	xfers.cs_change = 0;
	if (bus->dma_mode_enabled)
		xfers.bits_per_word = buf_len >= 64 ? 32 : 8;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);

	mutex_unlock(&bus->mutex);

	if (ret < 0)
		ts_err("spi transfer error:%d", ret);

	if (tx_buf != bus->tx_buf)
		kfree(tx_buf);
	return ret;
}

static int goodix_spi_probe(struct spi_device *spi)
{
	struct goodix_device_resource *dev_res;
	int ret = 0;

	ts_info("%s: goodix spi probe in", __func__);

	/* init spi_device */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->rt = true; /* [GOOG] */

	ret = spi_setup(spi);
	if (ret) {
		ts_err("failed set spi mode, %d", ret);
		return ret;
	}

	dev_res = kzalloc(sizeof(*dev_res), GFP_KERNEL);
	if (!dev_res)
		return -ENOMEM;

	/* get ic type */
	ret = goodix_get_ic_type(spi->dev.of_node, &dev_res->bus);
	if (ret < 0)
		goto err_get_ic_type;

	dev_res->bus.bus_type = GOODIX_BUS_TYPE_SPI;
	dev_res->bus.dev = &spi->dev;
	if (dev_res->bus.ic_type == IC_TYPE_BERLIN_A) {
		dev_res->bus.read = goodix_spi_read_bra;
	} else {
		dev_res->bus.read = goodix_spi_read;
		dev_res->bus.read_fast = goodix_spi_read_fast;
	}
	dev_res->bus.write = goodix_spi_write;

/* [GOOG]
 * Move goodix_device_register() after `dev_res->bus.dev` assigned.
 * This will help to set the `struct device *dev` early.
 */
	goodix_device_register(dev_res);

	dev_res->bus.rx_buf = kzalloc(SPI_PREALLOC_RX_BUF_SIZE, GFP_KERNEL);
	dev_res->bus.tx_buf = kzalloc(SPI_PREALLOC_TX_BUF_SIZE, GFP_KERNEL);
	if (!dev_res->bus.rx_buf || !dev_res->bus.tx_buf) {
		ret = -ENOMEM;
		goto err_pdev;
	}
/*~[GOOG] */
	mutex_init(&dev_res->bus.mutex);

	dev_res->bus.dma_mode_enabled = false;
#ifdef CONFIG_GOOG_TOUCH_INTERFACE
	dev_res->bus.dma_mode_enabled = goog_check_spi_dma_enabled(spi);
	ts_info("dma_mode: %s\n", dev_res->bus.dma_mode_enabled ? "enabled" : "disabled");
#endif

	// platform device init
	dev_res->pdev.name = GOODIX_CORE_DRIVER_NAME;
	dev_res->pdev.id = dev_res->id;
	dev_res->pdev.num_resources = 0;

	/* register platform device, then the goodix_ts_core
	 * module will probe the touch device.
	 */
	ret = platform_device_register(&dev_res->pdev);
	if (ret) {
		ts_err("failed register goodix platform device, %d", ret);
		goto err_pdev;
	}
	ts_info("spi probe out");
	return 0;

err_pdev:
	kfree(dev_res->bus.rx_buf);
	kfree(dev_res->bus.tx_buf);
err_get_ic_type:
	kfree(dev_res);
	ts_info("spi probe out, %d", ret);
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void goodix_spi_remove(struct spi_device *spi)
{
	/* goodix_ts_core_exit() will unregister device(s)
	   platform_device_unregister(dev_res->pdev); */
}
#else
static int goodix_spi_remove(struct spi_device *spi)
{
	/* goodix_ts_core_exit() will unregister device(s)
	   platform_device_unregister(dev_res->pdev); */
	return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id spi_matches[] = {
	{
		.compatible = "goodix,brl-a",
	},
	{
		.compatible = "goodix,brl-b",
	},
	{
		.compatible = "goodix,brl-b,gt7986",
	},
	{
		.compatible = "goodix,brl-d",
	},
	{
		.compatible = "goodix,nottingham",
	},
	{},
};
MODULE_DEVICE_TABLE(of, spi_matches);
#endif

static const struct spi_device_id spi_id_table[] = {
	{ TS_DRIVER_NAME, 0 },
	{},
};

static struct spi_driver goodix_spi_driver = {
	.driver = {
		.name = TS_DRIVER_NAME,
		//.owner = THIS_MODULE,
		.of_match_table = spi_matches,
	},
	.id_table = spi_id_table,
	.probe = goodix_spi_probe,
	.remove = goodix_spi_remove,
};

int goodix_spi_bus_init(void)
{
	ts_info("Goodix spi driver init");
	return spi_register_driver(&goodix_spi_driver);
}

void goodix_spi_bus_exit(void)
{
	ts_info("Goodix spi driver exit");
	spi_unregister_driver(&goodix_spi_driver);
}
