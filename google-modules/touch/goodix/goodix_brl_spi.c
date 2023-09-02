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

static struct platform_device *goodix_pdev;
struct goodix_bus_interface goodix_spi_bus;

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
	u8 *rx_buf = NULL;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	int buf_len = SPI_READ_PREFIX_LEN + len;

	mutex_lock(&goodix_spi_bus.mutex);

	if (buf_len <= SPI_PREALLOC_RX_BUF_SIZE &&
		buf_len <= SPI_PREALLOC_TX_BUF_SIZE) {
		rx_buf = goodix_spi_bus.rx_buf;
		tx_buf = goodix_spi_bus.tx_buf;
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
	if (tx_buf != goodix_spi_bus.tx_buf)
		kfree(tx_buf);
err_alloc_tx_buf:
	if (rx_buf != goodix_spi_bus.rx_buf)
		kfree(rx_buf);
err_alloc_rx_buf:
	mutex_unlock(&goodix_spi_bus.mutex);
	return ret;
}

static int goodix_spi_read(struct device *dev, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 *rx_buf = NULL;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	int buf_len = SPI_READ_PREFIX_LEN - 1 + len;

	if (goodix_spi_bus.dma_mode_enabled && buf_len >= 64) {
		buf_len = ALIGN(buf_len, 4);
	}

	mutex_lock(&goodix_spi_bus.mutex);

	if (buf_len <= SPI_PREALLOC_RX_BUF_SIZE &&
		buf_len <= SPI_PREALLOC_TX_BUF_SIZE) {
		rx_buf = goodix_spi_bus.rx_buf;
		tx_buf = goodix_spi_bus.tx_buf;
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
	if (goodix_spi_bus.dma_mode_enabled) xfers.bits_per_word = buf_len >= 64 ? 32 : 8;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		ts_err("spi transfer error:%d", ret);
		goto err_spi_transfer;
	}
	memcpy(data, &rx_buf[SPI_READ_PREFIX_LEN - 1], len);

err_spi_transfer:
	if (tx_buf != goodix_spi_bus.tx_buf)
		kfree(tx_buf);
err_alloc_tx_buf:
	if (rx_buf != goodix_spi_bus.rx_buf)
		kfree(rx_buf);
err_alloc_rx_buf:
	mutex_unlock(&goodix_spi_bus.mutex);
	return ret;
}

static int goodix_spi_read_fast(struct device *dev, unsigned int addr,
	struct goodix_rx_package *package, unsigned int len)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	int buf_len = SPI_READ_PREFIX_LEN - 1 + len;

	if (goodix_spi_bus.dma_mode_enabled && buf_len >= 64) {
		buf_len = ALIGN(buf_len, 4);
	}

	if (buf_len <= SPI_PREALLOC_TX_BUF_SIZE) {
		tx_buf = goodix_spi_bus.tx_buf;
	} else {
		tx_buf = kzalloc(buf_len, GFP_KERNEL);
		if (!tx_buf) {
			ts_err("alloc tx_buf failed, size:%d", buf_len);
			return -ENOMEM;
		}
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	mutex_lock(&goodix_spi_bus.mutex);

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
	if (goodix_spi_bus.dma_mode_enabled) xfers.bits_per_word = buf_len >= 64 ? 32 : 8;
	spi_message_add_tail(&xfers, &spi_msg);

	ret = spi_sync(spi, &spi_msg);

	mutex_unlock(&goodix_spi_bus.mutex);

	if (ret < 0) {
		ts_err("spi transfer error:%d", ret);
		goto err_spi_transfer;
	}

err_spi_transfer:
	if (tx_buf != goodix_spi_bus.tx_buf)
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
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	int buf_len = SPI_WRITE_PREFIX_LEN + len;

	if (goodix_spi_bus.dma_mode_enabled && buf_len >= 64) {
		buf_len = ALIGN(buf_len, 4);
	}

	if (buf_len <= SPI_PREALLOC_TX_BUF_SIZE) {
		tx_buf = goodix_spi_bus.tx_buf;
	} else {
		tx_buf = kzalloc(buf_len, GFP_KERNEL);
		if (!tx_buf) {
			ts_err("alloc tx_buf failed, size:%d", buf_len);
			return -ENOMEM;
		}
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	mutex_lock(&goodix_spi_bus.mutex);

	tx_buf[0] = SPI_WRITE_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	memcpy(&tx_buf[SPI_WRITE_PREFIX_LEN], data, len);
	xfers.tx_buf = tx_buf;
	xfers.len = buf_len;
	xfers.cs_change = 0;
	if (goodix_spi_bus.dma_mode_enabled) xfers.bits_per_word = buf_len >= 64 ? 32 : 8;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);

	mutex_unlock(&goodix_spi_bus.mutex);

	if (ret < 0)
		ts_err("spi transfer error:%d", ret);

	if (tx_buf != goodix_spi_bus.tx_buf)
		kfree(tx_buf);
	return ret;
}

static void goodix_pdev_release(struct device *dev)
{
	ts_info("goodix pdev released");
	kfree(goodix_pdev);
}

static int goodix_spi_probe(struct spi_device *spi)
{
	int ret = 0;

	ts_info("goodix spi probe in");

	/* init spi_device */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->rt = true;

	ret = spi_setup(spi);
	if (ret) {
		ts_err("failed set spi mode, %d", ret);
		return ret;
	}

	/* get ic type */
	ret = goodix_get_ic_type(spi->dev.of_node, &goodix_spi_bus);
	if (ret < 0)
		return ret;

	goodix_spi_bus.bus_type = GOODIX_BUS_TYPE_SPI;
	goodix_spi_bus.dev = &spi->dev;
	if (goodix_spi_bus.ic_type == IC_TYPE_BERLIN_A) {
		goodix_spi_bus.read = goodix_spi_read_bra;
	} else {
		goodix_spi_bus.read = goodix_spi_read;
		goodix_spi_bus.read_fast = goodix_spi_read_fast;
	}
	goodix_spi_bus.write = goodix_spi_write;

	goodix_spi_bus.rx_buf = kzalloc(SPI_PREALLOC_RX_BUF_SIZE, GFP_KERNEL);
	if (!goodix_spi_bus.rx_buf) {
		return -ENOMEM;
	}

	goodix_spi_bus.tx_buf = kzalloc(SPI_PREALLOC_TX_BUF_SIZE, GFP_KERNEL);
	if (!goodix_spi_bus.tx_buf) {
		ret = -ENOMEM;
		goto err_alloc_tx_buf;
	}

	mutex_init(&goodix_spi_bus.mutex);

	goodix_spi_bus.dma_mode_enabled = false;
#ifdef CONFIG_GOOG_TOUCH_INTERFACE
	goodix_spi_bus.dma_mode_enabled = goog_check_spi_dma_enabled(spi);
	ts_info("dma_mode: %s\n", goodix_spi_bus.dma_mode_enabled ? "enabled" : "disabled");
#endif

	/* ts core device */
	goodix_pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if (!goodix_pdev) {
		ret = -ENOMEM;
		goto err_alloc_pdev;
	}

	goodix_pdev->name = GOODIX_CORE_DRIVER_NAME;
	goodix_pdev->id = 0;
	goodix_pdev->num_resources = 0;
	/*
	 * you can find this platform dev in
	 * /sys/devices/platform/goodix_ts.0
	 * goodix_pdev->dev.parent = &client->dev;
	 */
	goodix_pdev->dev.platform_data = &goodix_spi_bus;
	goodix_pdev->dev.release = goodix_pdev_release;

	/* register platform device, then the goodix_ts_core
	 * module will probe the touch device.
	 */
	ret = platform_device_register(goodix_pdev);
	if (ret) {
		ts_err("failed register goodix platform device, %d", ret);
		goto err_register_platform_device;
	}
	ts_info("spi probe out");
	return 0;

err_register_platform_device:
	kfree(goodix_pdev);
err_alloc_pdev:
	kfree(goodix_spi_bus.tx_buf);
err_alloc_tx_buf:
	kfree(goodix_spi_bus.rx_buf);

	ts_info("spi probe out, %d", ret);
	return ret;
}

static int goodix_spi_remove(struct spi_device *spi)
{
	platform_device_unregister(goodix_pdev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id spi_matches[] = {
	{
		.compatible = "goodix,brl-a",
	},
	{
		.compatible = "goodix,brl-b",
	},
	{
		.compatible = "goodix,brl-d",
	},
	{
		.compatible = "goodix,nottingham",
	},
	{},
};
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
