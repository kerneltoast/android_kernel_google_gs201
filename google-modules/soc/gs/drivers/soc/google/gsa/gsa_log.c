// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Google LLC.
 */
#include <asm/page.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include "gsa_log.h"

#define GSA_LOG_MAGIC 'GSAL'
#define GSA_LOG_SIZE  0x800

/*
 * struct gsa_log_hdr - GSA log header
 * @magic: should be set to %GSA_LOG_MAGIC
 * @size: log buffer size
 * @tail: offset of location in log buffer where next character should be
 *        written. Normally should contain 0 as a head/tail separator.
 *
 * In memory this structure prepends GSA log body.
 */
struct gsa_log_hdr {
	uint32_t magic;
	uint32_t size;
	uint32_t tail;
} __packed;

struct gsa_log_mem {
	struct gsa_log_hdr hdr;
	uint8_t body[0];
} __packed;

struct gsa_log {
	struct device *dev;
	void __iomem *base_main;
	void __iomem *base_intermediate;
};

struct gsa_log *gsa_log_init(struct platform_device *pdev)
{
	struct device_node *np;
	struct gsa_log *log;
	struct reserved_mem *rmem;
	struct device *dev = &pdev->dev;

	/* Check if we have log in the device tree; it is not required. */
	np = of_parse_phandle(dev->of_node, "log-region", 0);
	if (!np) {
		dev_info(dev, "log region not configured\n");
		return NULL;
	}

	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem) {
		dev_err(dev, "configured log region invalid\n");
		return ERR_PTR(-ENODEV);
	}

	if (rmem->size % (GSA_LOG_SIZE * 2)) {
		dev_err(dev, "log size not multiple of expected size %d", (GSA_LOG_SIZE * 2));
		return ERR_PTR(-EINVAL);
	}

	log = devm_kzalloc(dev, sizeof(*log), GFP_KERNEL);
	if (!log)
		return ERR_PTR(-ENOMEM);

	log->dev = dev;

	/* map main log region */
	log->base_main = devm_ioremap(dev, rmem->base, rmem->size);
	if (IS_ERR(log->base_main)) {
		dev_err(dev, "ioremap failed (%d)\n", (int)PTR_ERR(log->base_main));
		return log->base_main;
	}

	/* map intermediate reset log region */
	log->base_intermediate = log->base_main + GSA_LOG_SIZE;

	return log;
}

ssize_t gsa_log_read(struct gsa_log *log, bool intermediate, char *buf)
{
	uint32_t magic;
	size_t size;
	size_t tail;
	size_t offset = 0;
	void *base;
	struct gsa_log_mem *gsa_log_mem_base;

	if (intermediate) {
		base = log->base_intermediate;
	} else {
		base = log->base_main;
	}

	gsa_log_mem_base = (struct gsa_log_mem *)base;
	magic = readl(&gsa_log_mem_base->hdr.magic);
	size = readl(&gsa_log_mem_base->hdr.size);
	tail = readl(&gsa_log_mem_base->hdr.tail);

	if (magic != GSA_LOG_MAGIC
			|| size != (GSA_LOG_SIZE - sizeof(struct gsa_log_hdr))
			|| tail >= size) {
		dev_err(log->dev, "log is corrupted\n");
		return 0;
	}

	/*
	 * 'tail' points to the end of the current log. There are two
	 * possibilities about the rest of the buffer after 'tail':
	 *  1. The buffer is empty; we ignore it.
	 *  2. The buffer has older log data; we need to copy it first.
	 *
	 * Check to see if the byte after tail is null to discriminate
	 * which case the log is in:
	 */
	if ((tail + 1) != size && readb(&gsa_log_mem_base->body[tail + 1])) {
		memcpy_fromio(buf, &gsa_log_mem_base->body[tail + 1], size - tail - 1);
		offset += size - tail - 1;
	}

	/* Copy the newer log data (bytes up to 'tail') */
	memcpy_fromio(&buf[offset], gsa_log_mem_base->body, tail);
	buf[offset + tail] = '\0';

	return offset + tail;
}

MODULE_LICENSE("GPL v2");
