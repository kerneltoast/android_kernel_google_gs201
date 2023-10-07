/*
 * Google LWIS I/O Entry Implementation
 *
 * Copyright (c) 2021 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-ioentry: " fmt

#include <linux/delay.h>
#include <linux/preempt.h>

#include "lwis_io_entry.h"
#include "lwis_util.h"

int lwis_io_entry_poll(struct lwis_device *lwis_dev, struct lwis_io_entry *entry)
{
	uint64_t val, start;
	uint64_t timeout_ms = entry->read_assert.timeout_ms;
	int ret = 0;

	/* Only read and check once if in_irq() */
	if (in_irq()) {
		timeout_ms = 0;
	}

	/* Read until getting the expected value or timeout */
	val = ~entry->read_assert.val;
	start = ktime_to_ms(lwis_get_time());
	while (val != entry->read_assert.val) {
		ret = lwis_io_entry_read_assert(lwis_dev, entry);
		if (ret == 0) {
			break;
		}
		if (ktime_to_ms(lwis_get_time()) - start > timeout_ms) {
			dev_err(lwis_dev->dev, "Polling timed out: block %d offset 0x%llx\n",
				entry->read_assert.bid, entry->read_assert.offset);
			return -ETIMEDOUT;
		}
		/* Sleep for 1ms */
		usleep_range(1000, 1000);
	}
	return ret;
}

int lwis_io_entry_read_assert(struct lwis_device *lwis_dev, struct lwis_io_entry *entry)
{
	uint64_t val;
	int ret = 0;

	ret = lwis_device_single_register_read(lwis_dev, entry->read_assert.bid,
					       entry->read_assert.offset, &val,
					       lwis_dev->native_value_bitwidth);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to read registers: block %d offset 0x%llx\n",
			entry->read_assert.bid, entry->read_assert.offset);
		return ret;
	}
	if ((val & entry->read_assert.mask) == (entry->read_assert.val & entry->read_assert.mask)) {
		return 0;
	}
	return -EINVAL;
}
