// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "include/gnss.h"
#include "gnss_spi.h"


int gnss_spi_send(char *buff, unsigned int size, char *rx_buff)
{
	int ret = 0;
	struct spi_message msg;
	struct spi_transfer tx;

	memset(&tx, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);
	tx.len = size;
	tx.tx_buf = buff;
	tx.bits_per_word = SPI_BITS_PER_WORD;
	if (rx_buff)
		tx.rx_buf = rx_buff;
	spi_message_add_tail(&tx, &msg);
	ret = spi_sync(gnss_if.spi, &msg);
	if (ret < 0)
		gif_err("spi_sync() error:%d\n", ret);

	return ret;
}
EXPORT_SYMBOL(gnss_spi_send);

int gnss_spi_recv(char *buff, unsigned int size)
{
	int ret = 0;
	struct spi_message msg;
	struct spi_transfer rx;

	memset(&rx, 0, sizeof(struct spi_transfer));
	rx.len = size;
	rx.rx_buf = buff;
	rx.bits_per_word = SPI_BITS_PER_WORD;
	spi_message_init(&msg);
	spi_message_add_tail(&rx, &msg);
	ret = spi_sync(gnss_if.spi, &msg);
	if (ret < 0)
		gif_err("spi_sync() error:%d\n", ret);

	return ret;

}
EXPORT_SYMBOL(gnss_spi_recv);

static int gnss_spi_probe(struct spi_device *spi)
{
	int ret = 0;

	gif_info("+++\n");
	mutex_init(&gnss_if.lock);

	if (spi_setup(spi)) {
		gif_err("ERR! spi_setup fail\n");
		ret = -EINVAL;
		goto err_setup;
	}
	spi_set_drvdata(spi, &gnss_if);
	gnss_if.spi = spi;
	gif_info("---\n");

	return 0;

err_setup:
	mutex_destroy(&gnss_if.lock);

	return ret;
}

static void gnss_spi_remove (struct spi_device *spi)
{
	struct gnss_spi *gnss_if = spi_get_drvdata(spi);

	mutex_destroy(&gnss_if->lock);
}

static const struct of_device_id gnss_spi_dt_match[] = {
	{ .compatible = "samsung,gnss-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, gnss_spi_dt_match);

static struct spi_driver gnss_spi_driver = {
	.probe = gnss_spi_probe,
	.remove = gnss_spi_remove,
	.driver = {
		.name = "gnss_spi",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gnss_spi_dt_match),
		.suppress_bind_attrs = true,
	},
};
module_spi_driver(gnss_spi_driver);

MODULE_DESCRIPTION("Exynos SPI driver for GNSS communication");
MODULE_LICENSE("GPL");

