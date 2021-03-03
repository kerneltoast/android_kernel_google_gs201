// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2020 Google LLC
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fsnotify.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/uaccess.h>

#include <trace/hooks/pstore.h>

#define RAMLOG_LAST_RSE_NAME	"bl_old_log"
#define RAMLOG_CUR_RSE_NAME	"bl_log"

/*
 * Header structure must be byte-packed, since the table is provided by
 * bootloader.
 */
struct bldr_log_header {
	__u64 i;
	__u64 size;
} __packed;

static void *bl_last_log_buf, *bl_cur_log_buf;
static size_t bl_last_log_buf_size, bl_cur_log_buf_size;

static int bldr_log_check_header(const struct bldr_log_header *header, size_t bldr_log_size)
{
	if (bldr_log_size <= sizeof(*header) || bldr_log_size - sizeof(*header) < header->size)
		return -EINVAL;
	return 0;
}

static void bldr_log_parser(struct device *dev, const void *bldr_log, size_t bldr_log_size,
			    char *bldr_log_buf, size_t *bldr_log_buf_size)
{
	const struct bldr_log_header *header = bldr_log;
	u64 offset = header->i % header->size;

	if (bldr_log_check_header(header, bldr_log_size)) {
		dev_warn(dev, "invalid bldr_log header in %p\n", bldr_log);
		*bldr_log_buf_size = 0;
		return;
	}

	if (header->i > header->size) {
		char *bldr_log_buf_ptr = bldr_log_buf;
		size_t bldr_log_bottom_size, bldr_log_top_size;

		/* Bottom half part  */
		bldr_log_bottom_size = bldr_log_size - sizeof(*header) - offset;
		memcpy(bldr_log_buf_ptr, bldr_log + sizeof(*header) + offset, bldr_log_bottom_size);
		bldr_log_buf_ptr += bldr_log_bottom_size;

		/* Top half part  */
		bldr_log_top_size = offset;
		memcpy(bldr_log_buf_ptr, bldr_log + sizeof(*header), bldr_log_top_size);

		*bldr_log_buf_size = bldr_log_bottom_size + bldr_log_top_size;
	} else {
		*bldr_log_buf_size = offset;

		memcpy(bldr_log_buf, bldr_log + sizeof(*header), *bldr_log_buf_size);
	}
}

/**
 * Read last bootloader logs, kernel logs, current bootloader logs in order.
 *
 * Handle reads that overlap different regions so the file appears like one
 * contiguous file to the reader.
 */
static ssize_t bldr_log_read(char __user *userbuf, size_t count, loff_t *ppos,
			     const void *lastk_buf, ssize_t lastk_size)
{
	loff_t pos;
	ssize_t total_len = 0;
	ssize_t len;
	int i;

	struct {
		const char *buf;
		const ssize_t size;
	} log_regions[] = {
		{ .buf = bl_last_log_buf,	.size = bl_last_log_buf_size },
		{ .buf = lastk_buf,		.size = lastk_size },
		{ .buf = bl_cur_log_buf,	.size = bl_cur_log_buf_size },
	};

	pos = *ppos;
	if (pos < 0)
		return -EINVAL;

	if (!count)
		return 0;

	for (i = 0; i < ARRAY_SIZE(log_regions); ++i) {
		if (pos < log_regions[i].size && log_regions[i].buf) {
			len = simple_read_from_buffer(userbuf, count, &pos,
						      log_regions[i].buf,
						      log_regions[i].size);
			if (len < 0)
				return len;
			count -= len;
			userbuf += len;
			total_len += len;
		}
		pos -= log_regions[i].size;
		if (pos < 0)
			break;
	}

	*ppos += total_len;
	return total_len;
}

static ssize_t bldr_log_total_size(void)
{
	return bl_last_log_buf_size + bl_cur_log_buf_size;
}

static int bldr_log_setup(struct device *dev, phys_addr_t bldr_phy_addr, size_t bldr_log_size,
			  void **log_buf, size_t *log_size)
{
	void __iomem *bldr_base;
	int ret = 0;

	bldr_base = ioremap_cache(bldr_phy_addr, bldr_log_size);
	if (!bldr_base) {
		dev_err(dev, "failed to map bootloader log buffer\n");
		return -ENOMEM;
	}

	/* allocate memory matching bootloader region size */
	*log_buf = kmalloc(bldr_log_size, GFP_KERNEL);
	if (!*log_buf) {
		ret = -ENOMEM;
		goto _unmap;
	}

	bldr_log_parser(dev, bldr_base, bldr_log_size, *log_buf, log_size);
	dev_info(dev, "bootloader log: buf_size:%zu, parsed:%zu\n", bldr_log_size, *log_size);

	/* free extra memory by reallocation */
	*log_buf = krealloc(*log_buf, *log_size, GFP_KERNEL);
	if (!*log_buf)
		*log_size = 0;

_unmap:
	iounmap(bldr_base);

	return ret;
}

/*
 * interface function: adjusts console file length
 */
static void bldr_log_if_console_mkfile(void *data, loff_t *i_size)
{
	*i_size += bldr_log_total_size();
}

/*
 * interface function: handles console file read
 */
static void bldr_log_if_console_read(void *data, void __user *to, size_t count, loff_t *ppos,
				     const void *from, size_t available, ssize_t *ret)
{
	*ret = bldr_log_read(to, count, ppos, from, available);
}

static int __init bldr_log_probe(struct platform_device *pdev)
{
	struct resource temp_res;
	int num_reg = 0;
	int rc = 0;

	while (of_address_to_resource(pdev->dev.of_node, num_reg, &temp_res) == 0) {
		if (!strcmp(temp_res.name, RAMLOG_LAST_RSE_NAME))
			bldr_log_setup(&pdev->dev, temp_res.start, resource_size(&temp_res),
				       &bl_last_log_buf, &bl_last_log_buf_size);
		else if (!strcmp(temp_res.name, RAMLOG_CUR_RSE_NAME))
			bldr_log_setup(&pdev->dev, temp_res.start, resource_size(&temp_res),
				       &bl_cur_log_buf, &bl_cur_log_buf_size);
		else
			dev_warn(&pdev->dev, "unknown bldr resource %s\n", temp_res.name);

		num_reg++;
	}

	if (!num_reg) {
		dev_warn(&pdev->dev, "can't find address resource\n");
		return 0;
	}

	rc = register_trace_android_vh_pstore_console_mkfile(bldr_log_if_console_mkfile, NULL);
	if (rc) {
		dev_err(&pdev->dev, "unable to register console mkfile hook\n");
		return rc;
	}
	rc = register_trace_android_vh_pstore_console_read(bldr_log_if_console_read, NULL);
	if (rc)
		dev_err(&pdev->dev, "unable to register console read hook\n");

	return rc;
}

static int __exit bldr_log_remove(struct platform_device *pdev)
{
	kfree(bl_last_log_buf);
	kfree(bl_cur_log_buf);
	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "google,bldr_log" },
	{}
};

static struct platform_driver bldr_log_driver = {
	.driver		= {
		.name	= "bldr_log",
		.of_match_table	= dt_match,
	},
	.remove		= __exit_p(bldr_log_remove),
};

module_platform_driver_probe(bldr_log_driver, bldr_log_probe);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Oleg Matcovschi <omatcovschi@google.com>");
MODULE_DESCRIPTION("Pstore bootloader logs driver");
