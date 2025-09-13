// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC service library
 *
 * Copyright (c) 2019-2023 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "aoc.h"

extern enum AOC_FW_STATE aoc_state;
extern struct aoc_control_block *aoc_control;

static void signal_aoc(struct mbox_chan *channel);

ssize_t aoc_service_read_timeout(struct aoc_service_dev *dev, uint8_t *buffer,
				 size_t count, long timeout)
{
	struct aoc_prvdata *prvdata;
	aoc_service *service;

	size_t msg_size;
	int service_number;
	long ret = 1;

	if (!dev || !buffer || !count)
		return -EINVAL;

	if (dev->dead)
		return -ENODEV;

	prvdata = dev_get_drvdata(dev->dev.parent);
	if (!prvdata)
		return -ENODEV;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE) {
		ret = -EBUSY;
		goto err;
	}

	service_number = dev->service_index;
	service = service_at_index(prvdata, dev->service_index);

	if (!aoc_is_valid_dram_address(prvdata, service)) {
		WARN_ONCE(1, "aoc service %d has invalid DRAM region", service_number);
		ret = -ENODEV;
		goto err;
	}

	if (aoc_service_message_slots(service, AOC_UP) == 0) {
		ret = -EBADF;
		goto err;
	}

	if (!aoc_service_can_read_message(service, AOC_UP)) {
		set_bit(service_number, prvdata->read_blocked_mask);
		ret = wait_event_interruptible_timeout(
			dev->read_queue,
			aoc_state != AOC_STATE_ONLINE || dev->dead ||
				aoc_service_can_read_message(service, AOC_UP),
			timeout);
		clear_bit(service_number, prvdata->read_blocked_mask);
	}

	if (dev->dead || (aoc_state != AOC_STATE_ONLINE)) {
		ret = -ENODEV;
		goto err;
	}

	if (ret < 0)
		goto err;

	/* AoC timed out */
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto err;
	}

	if (!aoc_service_is_ring(service) &&
	    count < aoc_service_current_message_size(service, prvdata->ipc_base,
						     AOC_UP)) {
		ret = -EFBIG;
		goto err;
	}

	msg_size = count;
	aoc_service_read_message(service, prvdata->ipc_base, AOC_UP, buffer,
				 &msg_size);

err:
	atomic_dec(&prvdata->aoc_process_active);

	if (ret < 0)
		return ret;

	return msg_size;
}
EXPORT_SYMBOL_GPL(aoc_service_read_timeout);

ssize_t aoc_service_write(struct aoc_service_dev *dev, const uint8_t *buffer,
			  size_t count, bool block)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;

	aoc_service *service;
	int service_number;
	int interrupt = dev->mbox_index;
	int ret = 0;

	if (!dev || !buffer || !count)
		return -EINVAL;

	if (dev->dead)
		return -ENODEV;

	if (aoc_state != AOC_STATE_ONLINE)
		return -ENODEV;

	BUG_ON(!dev->dev.parent);

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);
	if (!prvdata)
		return -ENODEV;

	if (interrupt >= prvdata->aoc_mbox_channels)
		return -EINVAL;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE) {
		ret = -EBUSY;
		goto err;
	}

	service_number = dev->service_index;
	service = service_at_index(prvdata, service_number);

	if (!aoc_is_valid_dram_address(prvdata, service)) {
		WARN_ONCE(1, "aoc service %d has invalid DRAM region", service_number);
		ret = -ENODEV;
		goto err;
	}

	if (aoc_service_message_slots(service, AOC_DOWN) == 0) {
		ret = -EBADF;
		goto err;
	}

	if (count > aoc_service_message_size(service, AOC_DOWN)) {
		ret = -EFBIG;
		goto err;
	}

	if (!aoc_service_can_write_message(service, AOC_DOWN)) {
		if (!block) {
			ret = -EAGAIN;
			goto err;
		}

		set_bit(service_number, prvdata->write_blocked_mask);
		ret = wait_event_interruptible(dev->write_queue,
			aoc_state != AOC_STATE_ONLINE || dev->dead ||
				aoc_service_can_write_message(service, AOC_DOWN));
		clear_bit(service_number, prvdata->write_blocked_mask);
	}

	if (dev->dead) {
		ret = -ENODEV;
		goto err;
	}

	if (aoc_state != AOC_STATE_ONLINE) {
		ret = -ENODEV;
		goto err;
	}

	/*
	 * The wait can fail if the AoC goes offline in the middle of a
	 * blocking write, so check again after the wait
	 */
	if (ret != 0) {
		ret = -EAGAIN;
		goto err;
	}

	ret = aoc_service_write_message(service, prvdata->ipc_base, AOC_DOWN,
					buffer, count);

	if (!aoc_service_is_ring(service) || aoc_ring_is_push(service))
		signal_aoc(prvdata->mbox_channels[interrupt].channel);
err:
	atomic_dec(&prvdata->aoc_process_active);
	if (ret < 0)
		return ret;

	return count;
}
EXPORT_SYMBOL_GPL(aoc_service_write);

ssize_t aoc_service_write_timeout(struct aoc_service_dev *dev, const uint8_t *buffer,
				  size_t count, long timeout)
{
	struct aoc_prvdata *prvdata;

	aoc_service *service;
	int service_number;
	int interrupt = dev->mbox_index;
	long ret = 1;

	if (!dev || !buffer || !count)
		return -EINVAL;

	if (dev->dead)
		return -ENODEV;

	prvdata = dev_get_drvdata(dev->dev.parent);
	if (!prvdata)
		return -ENODEV;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE) {
		ret = -EBUSY;
		goto err;
	}

	service_number = dev->service_index;
	service = service_at_index(prvdata, service_number);

	if (!aoc_is_valid_dram_address(prvdata, service)) {
		WARN_ONCE(1, "aoc service %d has invalid DRAM region", service_number);
		ret = -ENODEV;
		goto err;
	}

	if (aoc_service_message_slots(service, AOC_DOWN) == 0) {
		ret = -EBADF;
		goto err;
	}

	if (count > aoc_service_message_size(service, AOC_DOWN)) {
		ret = -EFBIG;
		goto err;
	}

	if (!aoc_service_can_write_message(service, AOC_DOWN)) {
		set_bit(service_number, prvdata->write_blocked_mask);
		ret = wait_event_interruptible_timeout(
			dev->write_queue,
			aoc_state != AOC_STATE_ONLINE || dev->dead ||
				aoc_service_can_write_message(service, AOC_DOWN),
			timeout);
		clear_bit(service_number, prvdata->write_blocked_mask);
	}

	if (dev->dead || aoc_state != AOC_STATE_ONLINE) {
		ret = -ENODEV;
		goto err;
	}

	if (ret < 0)
		goto err;

	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto err;
	}

	ret = aoc_service_write_message(service, prvdata->ipc_base, AOC_DOWN,
					buffer, count);

	if (!aoc_service_is_ring(service) || aoc_ring_is_push(service))
		signal_aoc(prvdata->mbox_channels[interrupt].channel);

err:
	atomic_dec(&prvdata->aoc_process_active);

	if (ret < 0)
		return ret;

	return count;
}
EXPORT_SYMBOL_GPL(aoc_service_write_timeout);

int aoc_service_can_read(struct aoc_service_dev *dev)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);
	service = service_at_index(prvdata, dev->service_index);

	if (aoc_service_message_slots(service, AOC_UP) == 0)
		return 0;

	return aoc_service_can_read_message(service, AOC_UP);
}
EXPORT_SYMBOL_GPL(aoc_service_can_read);

int aoc_service_can_write(struct aoc_service_dev *dev)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);
	service = service_at_index(prvdata, dev->service_index);

	if (aoc_service_message_slots(service, AOC_DOWN) == 0)
		return 0;

	return aoc_service_can_write_message(service, AOC_DOWN);
}
EXPORT_SYMBOL_GPL(aoc_service_can_write);

void aoc_service_set_read_blocked(struct aoc_service_dev *dev)
{
	int service_number;
	struct device *parent = dev->dev.parent;
	struct aoc_prvdata *prvdata = dev_get_drvdata(parent);

	service_number = dev->service_index;
	set_bit(service_number, prvdata->read_blocked_mask);
}
EXPORT_SYMBOL_GPL(aoc_service_set_read_blocked);

int aoc_num_services(void)
{
	return aoc_fw_ready() ? le32_to_cpu(aoc_control->services) : 0;
}
EXPORT_SYMBOL_GPL(aoc_num_services);

aoc_service *service_at_index(struct aoc_prvdata *prvdata,
					    unsigned int index)
{
	if (!aoc_fw_ready() || index > aoc_num_services())
		return NULL;

	return (((uint8_t *)prvdata->ipc_base) + aoc_control->services_offset +
		(le32_to_cpu(aoc_control->service_size) * index));
}
EXPORT_SYMBOL_GPL(service_at_index);

struct aoc_service_dev *service_dev_at_index(struct aoc_prvdata *prvdata,
							unsigned int index)
{
	if (!aoc_fw_ready() || index > aoc_num_services() || aoc_state != AOC_STATE_ONLINE)
		return NULL;

	return prvdata->services[index];
}
EXPORT_SYMBOL_GPL(service_dev_at_index);

bool validate_service(struct aoc_prvdata *prv, int i)
{
	struct aoc_ipc_service_header *hdr = service_at_index(prv, i);
	struct device *dev = prv->dev;

	if (!aoc_is_valid_dram_address(prv, hdr)) {
		dev_err(dev, "service %d is not in DRAM region\n", i);
		return false;
	}

	if (hdr->regions[0].slots == 0 && hdr->regions[1].slots == 0) {
		dev_err(dev, "service %d is not readable or writable\n", i);

		return false;
	}

	if (aoc_service_is_ring(hdr) &&
	    (hdr->regions[0].slots > 1 || hdr->regions[1].slots > 1)) {
		dev_err(dev, "service %d has invalid ring slot configuration\n",
			i);

		return false;
	}

	return true;
}
EXPORT_SYMBOL_GPL(validate_service);

bool aoc_service_flush_read_data(struct aoc_service_dev *dev)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;
	size_t slots;

	if (!dev)
		return false;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);

	service = service_at_index(prvdata, dev->service_index);

	slots = aoc_service_slots_available_to_read(service, AOC_UP);
	if (slots == 0)
		return false;

	aoc_service_advance_read_index(service, AOC_UP, slots);
	return true;
}
EXPORT_SYMBOL_GPL(aoc_service_flush_read_data);

ssize_t aoc_service_read(struct aoc_service_dev *dev, uint8_t *buffer,
			 size_t count, bool block)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;

	size_t msg_size;
	int service_number;
	int ret = 0;
	bool was_full;
	int interrupt = dev->mbox_index;

	if (!dev || !buffer || !count)
		return -EINVAL;

	if (dev->dead)
		return -ENODEV;

	if (aoc_state != AOC_STATE_ONLINE)
		return -EBUSY;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);
	if (!prvdata)
		return -ENODEV;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE) {
		ret = -EBUSY;
		goto err;
	}

	service_number = dev->service_index;
	service = service_at_index(prvdata, dev->service_index);

	if (!aoc_is_valid_dram_address(prvdata, service)) {
		WARN_ONCE(1, "aoc service %d has invalid DRAM region", service_number);
		ret = -ENODEV;
		goto err;
	}

	if (aoc_service_message_slots(service, AOC_UP) == 0) {
		ret = -EBADF;
		goto err;
	}

	if (!aoc_service_can_read_message(service, AOC_UP)) {
		if (!block) {
			ret = -EAGAIN;
			goto err;
		}

		set_bit(service_number, prvdata->read_blocked_mask);
		ret = wait_event_interruptible(dev->read_queue,
			aoc_state != AOC_STATE_ONLINE || dev->dead ||
				aoc_service_can_read_message(service, AOC_UP));
		clear_bit(service_number, prvdata->read_blocked_mask);
	}

	if (dev->dead) {
		ret = -ENODEV;
		goto err;
	}

	if (aoc_state != AOC_STATE_ONLINE) {
		ret = -ENODEV;
		goto err;
	}

	/*
	 * The wait can fail if the AoC goes offline in the middle of a
	 * blocking read, so check again after the wait
	 */
	if (ret != 0) {
		ret = -EAGAIN;
		goto err;
	}

	if (!aoc_service_is_ring(service) &&
	    count < aoc_service_current_message_size(service, prvdata->ipc_base,
						     AOC_UP)) {
		ret = -EFBIG;
		goto err;
	}

	msg_size = count;
	was_full = !aoc_service_can_write_message(service, AOC_UP);

	aoc_service_read_message(service, prvdata->ipc_base, AOC_UP, buffer,
				 &msg_size);

	/*
	 * If the service queue was full right before reading, signal AoC that
	 * there is now space available to write.
	 */
	if (was_full)
		signal_aoc(prvdata->mbox_channels[interrupt].channel);
err:
	atomic_dec(&prvdata->aoc_process_active);
	if (ret < 0)
		return ret;

	return msg_size;
}
EXPORT_SYMBOL_GPL(aoc_service_read);

bool aoc_online_state(struct aoc_service_dev *dev)
{
	struct aoc_prvdata *prvdata;

	if (!dev)
		return false;

	prvdata = dev_get_drvdata(dev->dev.parent);
	if (!prvdata)
		return false;

	if (aoc_state != AOC_STATE_ONLINE)
		return false;
	return true;
}
EXPORT_SYMBOL_GPL(aoc_online_state);

void aoc_service_set_write_blocked(struct aoc_service_dev *dev)
{
	int service_number;
	struct device *parent = dev->dev.parent;
	struct aoc_prvdata *prvdata = dev_get_drvdata(parent);

	service_number = dev->service_index;
	set_bit(service_number, prvdata->write_blocked_mask);
}
EXPORT_SYMBOL_GPL(aoc_service_set_write_blocked);

wait_queue_head_t *aoc_service_get_read_queue(struct aoc_service_dev *dev)
{
	return &dev->read_queue;
}
EXPORT_SYMBOL_GPL(aoc_service_get_read_queue);

wait_queue_head_t *aoc_service_get_write_queue(struct aoc_service_dev *dev)
{
	return &dev->write_queue;
}
EXPORT_SYMBOL_GPL(aoc_service_get_write_queue);

static void signal_aoc(struct mbox_chan *channel)
{
	mbox_send_message(channel, NULL);
}
