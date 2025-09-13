// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS I/O Entry Implementation
 *
 * Copyright (c) 2021 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-ioentry: " fmt

#include <linux/delay.h>
#include <linux/preempt.h>

#include "lwis_io_entry.h"
#include "lwis_util.h"

/* Allow 20us range in usleep */
#define USLEEP_RANGE_DELTA_SHORT 2
#define USLEEP_RANGE_DELTA 20

int lwis_io_entry_poll(struct lwis_device *lwis_dev, struct lwis_io_entry *entry, bool is_short)
{
	uint64_t val, start;
	uint64_t timeout_ms = entry->read_assert.timeout_ms;
	int ret = 0;
	int64_t process_time_ms = 0;

	/* Only read and check once if in_hardirq() */
	if (in_hardirq())
		timeout_ms = 0;

	/* Read until getting the expected value or timeout */
	val = ~entry->read_assert.val;
	start = ktime_to_ms(lwis_get_time());
	while (val != entry->read_assert.val) {
		ret = lwis_io_entry_read_assert(lwis_dev, entry);
		if (ret == 0)
			break;

		if (ktime_to_ms(lwis_get_time()) - start > timeout_ms) {
			ret = lwis_io_entry_read_assert(lwis_dev, entry);
			if (ret == 0)
				break;

			dev_err(lwis_dev->dev, "Polling timed out: block %d offset 0x%llx\n",
				entry->read_assert.bid, entry->read_assert.offset);
			return -ETIMEDOUT;
		}

		if (is_short) {
			/* Sleep for 10us */
			usleep_range(10, 10 + USLEEP_RANGE_DELTA_SHORT);
		} else {
			/* Sleep for 1ms */
			usleep_range(1000, 1000 + USLEEP_RANGE_DELTA);
		}
	}

	process_time_ms = ktime_to_ms(lwis_get_time()) - start;

	if (process_time_ms > DEFAULT_POLLING_TIMEOUT_MS)
		dev_info(lwis_dev->dev, "IO entry polling processed %lld ms", process_time_ms);

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
	if ((val & entry->read_assert.mask) == (entry->read_assert.val & entry->read_assert.mask))
		return 0;

	return -EINVAL;
}

int lwis_io_entry_wait(struct lwis_device *lwis_dev, struct lwis_io_entry *entry)
{
	if (entry->wait_us == 0)
		return 0;

	if (entry->wait_us <= MAX_WAIT_TIME) {
		usleep_range(entry->wait_us, entry->wait_us + USLEEP_RANGE_DELTA);
		return 0;
	}
	dev_warn(lwis_dev->dev, "Sleep time should be within 0us ~ %dus\n", MAX_WAIT_TIME);
	return -EINVAL;
}
