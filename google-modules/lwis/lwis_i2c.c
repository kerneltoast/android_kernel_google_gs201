// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS I2C Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-i2c: " fmt

#include "lwis_i2c.h"
#include "lwis_trace.h"
#include "lwis_util.h"

#include <linux/bits.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>

#define I2C_DEVICE_NAME "LWIS_I2C"

/* Max bit width for register and data that is supported by this
 * driver currently
 */
#define MIN_OFFSET_BITS 8
#define MAX_OFFSET_BITS 16
#define MIN_DATA_BITS 8
#define MAX_DATA_BITS 32

/* I2C target address bytes is 7-bit + 1 ack bit */
#define I2C_TARGET_ADDR_BYTES 1

static inline bool check_bitwidth(const int bitwidth, const int min, const int max)
{
	return (bitwidth >= min) && (bitwidth <= max) && ((bitwidth % 8) == 0);
}

static int perform_read_transfer(struct i2c_client *client, struct i2c_msg *msg, uint64_t offset,
				 int offset_size_bytes, struct lwis_device *lwis_dev)
{
	int ret = 0;
	u8 *wbuf = msg[0].buf;

	const int num_msg = 2;
	char trace_name[LWIS_MAX_NAME_STRING_LEN];

	scnprintf(trace_name, LWIS_MAX_NAME_STRING_LEN, "i2c_read_%s", lwis_dev->name);

	lwis_value_to_be_buf(offset, wbuf, offset_size_bytes);
	LWIS_ATRACE_FUNC_INT_BEGIN(lwis_dev, trace_name,
				   msg[0].len + msg[1].len + num_msg * I2C_TARGET_ADDR_BYTES);
	ret = i2c_transfer(client->adapter, msg, num_msg);
	LWIS_ATRACE_FUNC_INT_END(lwis_dev, trace_name,
				 msg[0].len + msg[1].len + num_msg * I2C_TARGET_ADDR_BYTES);
	return (ret == num_msg) ? 0 : ret;
}

static int perform_write_transfer(struct i2c_client *client, struct i2c_msg *msg, uint64_t offset,
				  int offset_size_bytes, int value_size_bytes, uint64_t value,
				  struct lwis_device *lwis_dev)
{
	int ret = 0;
	u8 *buf = msg->buf;

	const int num_msg = 1;
	char trace_name[LWIS_MAX_NAME_STRING_LEN];

	scnprintf(trace_name, LWIS_MAX_NAME_STRING_LEN, "i2c_write_%s", lwis_dev->name);

	lwis_value_to_be_buf(offset, buf, offset_size_bytes);
	lwis_value_to_be_buf(value, buf + offset_size_bytes, value_size_bytes);
	LWIS_ATRACE_FUNC_INT_BEGIN(lwis_dev, trace_name,
				   msg[0].len + num_msg * I2C_TARGET_ADDR_BYTES);
	ret = i2c_transfer(client->adapter, msg, num_msg);
	LWIS_ATRACE_FUNC_INT_END(lwis_dev, trace_name,
				 msg[0].len + num_msg * I2C_TARGET_ADDR_BYTES);
	return (ret == num_msg) ? 0 : ret;
}

static int perform_write_batch_transfer(struct i2c_client *client, struct i2c_msg *msg,
					uint64_t offset, int offset_size_bytes,
					int value_size_bytes, uint8_t *value_buf,
					struct lwis_device *lwis_dev)
{
	int ret = 0;
	u8 *buf = msg->buf;

	const int num_msg = 1;
	char trace_name[LWIS_MAX_NAME_STRING_LEN];

	scnprintf(trace_name, LWIS_MAX_NAME_STRING_LEN, "i2c_write_batch_%s", lwis_dev->name);

	lwis_value_to_be_buf(offset, buf, offset_size_bytes);
	memcpy(buf + offset_size_bytes, value_buf, value_size_bytes);

	LWIS_ATRACE_FUNC_INT_BEGIN(lwis_dev, trace_name,
				   msg[0].len + num_msg * I2C_TARGET_ADDR_BYTES);
	ret = i2c_transfer(client->adapter, msg, num_msg);
	LWIS_ATRACE_FUNC_INT_END(lwis_dev, trace_name,
				 msg[0].len + num_msg * I2C_TARGET_ADDR_BYTES);
	return (ret == num_msg) ? 0 : ret;
}

int lwis_i2c_set_state(struct lwis_i2c_device *i2c, const char *state_str)
{
	int ret;
	struct pinctrl_state *state;

	if (!i2c || !i2c->state_pinctrl) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}

	if (!i2c->set_master_pinctrl_state)
		return 0;

	state = pinctrl_lookup_state(i2c->state_pinctrl, state_str);
	if (IS_ERR_OR_NULL(state)) {
		dev_err(i2c->base_dev.dev, "State %s not found (%ld)\n", state_str, PTR_ERR(state));
		return PTR_ERR(state);
	}

	ret = pinctrl_select_state(i2c->state_pinctrl, state);
	if (ret) {
		dev_err(i2c->base_dev.dev, "Error selecting state %s (%d)\n", state_str, ret);
		return ret;
	}

	return 0;
}

static int i2c_read(struct lwis_i2c_device *i2c, uint64_t offset, uint64_t *value)
{
	int ret = 0;
	u8 *wbuf;
	u8 *rbuf;
	struct i2c_client *client;
	struct i2c_msg msg[2];
	unsigned int offset_bits;
	unsigned int offset_bytes;
	unsigned int value_bits;
	unsigned int value_bytes;

	if (!i2c || !i2c->client) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}
	client = i2c->client;

	offset_bits = i2c->base_dev.native_addr_bitwidth;
	offset_bytes = offset_bits / BITS_PER_BYTE;
	if (!check_bitwidth(offset_bits, MIN_OFFSET_BITS, MAX_OFFSET_BITS)) {
		dev_err(i2c->base_dev.dev, "Invalid offset bitwidth %d\n", offset_bits);
		return -EINVAL;
	}

	value_bits = i2c->base_dev.native_value_bitwidth;
	value_bytes = value_bits / BITS_PER_BYTE;
	if (!check_bitwidth(value_bits, MIN_DATA_BITS, MAX_DATA_BITS)) {
		dev_err(i2c->base_dev.dev, "Invalid value bitwidth %d\n", value_bits);
		return -EINVAL;
	}

	wbuf = kmalloc(offset_bytes, GFP_KERNEL);
	if (!wbuf)
		return -ENOMEM;

	rbuf = kmalloc(value_bytes, GFP_KERNEL);
	if (!rbuf) {
		ret = -ENOMEM;
		goto error_rbuf_alloc;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = offset_bytes;
	msg[0].buf = wbuf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = value_bytes;
	msg[1].buf = rbuf;

	ret = perform_read_transfer(client, msg, offset, offset_bytes, &i2c->base_dev);

	if (ret) {
		dev_err(i2c->base_dev.dev, "I2C Read failed: Offset 0x%llx (%d)\n", offset, ret);
		goto error_transfer;
	}

	*value = lwis_be_buf_to_value(rbuf, value_bytes);

error_transfer:
	kfree(rbuf);
error_rbuf_alloc:
	kfree(wbuf);

	return ret;
}

static int i2c_write(struct lwis_i2c_device *i2c, uint64_t offset, uint64_t value)
{
	int ret;
	u8 *buf;
	struct i2c_client *client;
	struct i2c_msg msg;
	unsigned int offset_bits;
	unsigned int value_bits;
	unsigned int offset_bytes;
	unsigned int value_bytes;
	int msg_bytes;

	if (!i2c || !i2c->client) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}
	client = i2c->client;

	if (i2c->base_dev.is_read_only) {
		dev_err(i2c->base_dev.dev, "Device is read only\n");
		return -EPERM;
	}

	offset_bits = i2c->base_dev.native_addr_bitwidth;
	offset_bytes = offset_bits / BITS_PER_BYTE;
	if (!check_bitwidth(offset_bits, MIN_OFFSET_BITS, MAX_OFFSET_BITS)) {
		dev_err(i2c->base_dev.dev, "Invalid offset bitwidth %d\n", offset_bits);
		return -EINVAL;
	}

	value_bits = i2c->base_dev.native_value_bitwidth;
	value_bytes = value_bits / BITS_PER_BYTE;
	if (!check_bitwidth(value_bits, MIN_DATA_BITS, MAX_DATA_BITS)) {
		dev_err(i2c->base_dev.dev, "Invalid value bitwidth %d\n", value_bits);
		return -EINVAL;
	}

	msg_bytes = offset_bytes + value_bytes;
	buf = kmalloc(msg_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = msg_bytes;

	ret = perform_write_transfer(client, &msg, offset, offset_bytes, value_bytes, value,
				     &i2c->base_dev);

	if (ret) {
		dev_err(i2c->base_dev.dev, "I2C Write failed: Offset 0x%llx Value 0x%llx (%d)\n",
			offset, value, ret);
	}

	kfree(buf);

	return ret;
}

static int i2c_read_batch(struct lwis_i2c_device *i2c, uint64_t start_offset, uint8_t *read_buf,
			  int read_buf_size)
{
	int ret = 0;
	uint8_t *wbuf;
	struct i2c_client *client;
	struct i2c_msg msg[2];
	unsigned int offset_bits;
	unsigned int offset_bytes;

	if (!i2c || !i2c->client) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}
	client = i2c->client;

	offset_bits = i2c->base_dev.native_addr_bitwidth;
	offset_bytes = offset_bits / BITS_PER_BYTE;
	if (!check_bitwidth(offset_bits, MIN_OFFSET_BITS, MAX_OFFSET_BITS)) {
		dev_err(i2c->base_dev.dev, "Invalid offset bitwidth %d\n", offset_bits);
		return -EINVAL;
	}

	wbuf = kmalloc(offset_bytes, GFP_KERNEL);
	if (!wbuf)
		return -ENOMEM;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = offset_bytes;
	msg[0].buf = wbuf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = read_buf_size;
	msg[1].buf = read_buf;

	ret = perform_read_transfer(client, msg, start_offset, offset_bytes, &i2c->base_dev);

	if (ret) {
		dev_err(i2c->base_dev.dev, "I2C Read Batch failed: Start Offset 0x%llx (%d)\n",
			start_offset, ret);
	}

	kfree(wbuf);
	return ret;
}

static int i2c_write_batch(struct lwis_i2c_device *i2c, uint64_t start_offset, uint8_t *write_buf,
			   int write_buf_size)
{
	int ret;
	uint8_t *buf;
	struct i2c_client *client;
	struct i2c_msg msg;
	unsigned int offset_bits;
	unsigned int offset_bytes;
	int msg_bytes;

	if (!i2c || !i2c->client) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}
	client = i2c->client;

	if (i2c->base_dev.is_read_only) {
		dev_err(i2c->base_dev.dev, "Device is read only\n");
		return -EPERM;
	}

	offset_bits = i2c->base_dev.native_addr_bitwidth;
	offset_bytes = offset_bits / BITS_PER_BYTE;
	if (!check_bitwidth(offset_bits, MIN_OFFSET_BITS, MAX_OFFSET_BITS)) {
		dev_err(i2c->base_dev.dev, "Invalid offset bitwidth %d\n", offset_bits);
		return -EINVAL;
	}

	msg_bytes = offset_bytes + write_buf_size;
	buf = kmalloc(msg_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = msg_bytes;

	ret = perform_write_batch_transfer(client, &msg, start_offset, offset_bytes, write_buf_size,
					   write_buf, &i2c->base_dev);

	if (ret) {
		dev_err(i2c->base_dev.dev, "I2C Write Batch failed: Start Offset 0x%llx (%d)\n",
			start_offset, ret);
	}

	kfree(buf);

	return ret;
}

int lwis_i2c_io_entry_rw(struct lwis_i2c_device *i2c, struct lwis_io_entry *entry)
{
	int ret;
	uint64_t reg_value;

	if (!entry) {
		dev_err(i2c->base_dev.dev, "IO entry is NULL.\n");
		return -EINVAL;
	}

	if (entry->type == LWIS_IO_ENTRY_READ)
		return i2c_read(i2c, entry->rw.offset, &entry->rw.val);

	if (entry->type == LWIS_IO_ENTRY_WRITE)
		return i2c_write(i2c, entry->rw.offset, entry->rw.val);

	if (entry->type == LWIS_IO_ENTRY_MODIFY) {
		ret = i2c_read(i2c, entry->mod.offset, &reg_value);
		if (ret)
			return ret;

		reg_value &= ~entry->mod.val_mask;
		reg_value |= entry->mod.val_mask & entry->mod.val;
		return i2c_write(i2c, entry->mod.offset, reg_value);
	}
	if (entry->type == LWIS_IO_ENTRY_READ_BATCH) {
		return i2c_read_batch(i2c, entry->rw_batch.offset, entry->rw_batch.buf,
				      entry->rw_batch.size_in_bytes);
	}
	if (entry->type == LWIS_IO_ENTRY_WRITE_BATCH) {
		return i2c_write_batch(i2c, entry->rw_batch.offset, entry->rw_batch.buf,
				       entry->rw_batch.size_in_bytes);
	}
	dev_err(i2c->base_dev.dev, "Invalid IO entry type: %d\n", entry->type);
	return -EINVAL;
}
