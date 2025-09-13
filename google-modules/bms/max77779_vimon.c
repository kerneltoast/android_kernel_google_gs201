/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023, Google Inc
 *
 * MAX77779 BATTVIMON management
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/reboot.h>
#include <linux/list.h>
#include <linux/genalloc.h>

#include "google_bms.h"
#include "max77779.h"
#include "max77779_vimon.h"

#define MAX77779_BVIM_bvim_trig_0_6_MASK (MAX77779_BVIM_bvim_trig_SPR_7_MASK - 1)
#define MAX77779_BVIM_bvim_trig_8_15_MASK (~((1 << (MAX77779_BVIM_bvim_trig_SPR_7_SHIFT + 1)) - 1))

#define MAX77779_VIMON_MAX_CLIENT 32
#define MAX77779_VIMON_PAGE_MASK 3

#define VIMON_DBG_TEMP_BUFFER_SZ	32
#define VIMON_DBG_CLIENT_MAX_OUTPUT	4

struct vimon_client_info {
	u16 mask;
	int count;
	void *private_data;
	struct vimon_client_callbacks *client_cb;
	struct list_head list;
};

static void on_debug_sample_ready(void *private, const enum vimon_trigger_source reason,
				  const u16 *data, const size_t len)
{
	struct max77779_vimon_data *vimon = private;
	size_t i = 0, pos = 0;
	const size_t count = len / sizeof(u16);
	char temp[VIMON_DBG_TEMP_BUFFER_SZ];

	dev_info(vimon->dev, "reason: %d, data: %p, len: %ld\n", reason, data, count);

	for (i = 0; i < count; i++) {
		pos += scnprintf(&temp[pos], VIMON_DBG_TEMP_BUFFER_SZ - pos, "%04x ", data[i]);

		if (((i + 1) % VIMON_DBG_CLIENT_MAX_OUTPUT) == 0) {
			dev_info(vimon->dev, " %s\n", temp);
			pos = 0;
		}
	}

	if (pos > 0)
		dev_info(vimon->dev, " %s\n", temp);
}

static void on_debug_removed(void *private)
{
	struct max77779_vimon_data *vimon = private;

	dev_dbg(vimon->dev, "on_removed_impl called\n");
}

static bool debug_extra_trigger(void *private, const u16 *data, const size_t len)
{
	struct max77779_vimon_data *vimon = private;

	dev_dbg(vimon->dev, "extra_trigger_impl called\n");
	return false;
}

static struct vimon_client_callbacks debug_cb_impl = {
	.on_sample_ready = on_debug_sample_ready,
	.on_removed = on_debug_removed,
	.extra_trigger = debug_extra_trigger,
};

static LIST_HEAD(vimon_clients);
static struct gen_pool *vimon_cache_pool;
static void *vimon_cache_mem;

static inline int max77779_vimon_reg_read(struct max77779_vimon_data *data, unsigned int reg,
					  unsigned int *val)
{
	return regmap_read(data->regmap, reg, val);
}

static inline int max77779_vimon_reg_write(struct max77779_vimon_data *data, unsigned int reg,
					   unsigned int val)
{
	return regmap_write(data->regmap, reg, val);
}

static inline int max77779_vimon_reg_update(struct max77779_vimon_data *data, unsigned int reg,
					    unsigned int mask, unsigned int val)
{
	return regmap_update_bits(data->regmap, reg, mask, val);
}

/* 0 not running, !=0 running, <0 error */
static int max77779_vimon_is_running(struct max77779_vimon_data *data)
{
	unsigned int running = 0;
	int ret;

	ret = max77779_vimon_reg_read(data, MAX77779_BVIM_CTRL, &running);

	if (ret < 0)
		return ret;

	return !!(running & MAX77779_BVIM_CTRL_BVIMON_TRIG_MASK);
}

static int max77779_vimon_direct_is_running(struct max77779_vimon_data *data)
{
	unsigned int running = 0;
	int ret;

	ret = data->direct_reg_read(data, MAX77779_BVIM_CTRL, &running);

	if (ret < 0)
		return ret;

	return !!(running & MAX77779_BVIM_CTRL_BVIMON_TRIG_MASK);
}

/* vimon_update_callback_mask needs to be protected by vimon_cb_lock */
static void vimon_update_callback_mask(struct max77779_vimon_data *data)
{
	u16 new_mask = 0;
	struct vimon_client_info *client;
	int ret;

	list_for_each_entry(client, &vimon_clients, list)
		new_mask |= client->mask;

	/* bit7 is used for SPR */
	new_mask = (new_mask & MAX77779_BVIM_bvim_trig_0_6_MASK) |
		   ((new_mask << 1) & MAX77779_BVIM_bvim_trig_8_15_MASK);

	if (data->trigger_src == new_mask)
		return;

	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_bvim_trig, new_mask);
	if (ret) {
		dev_err(data->dev, "Failed to configure vimon trig(%d)\n", ret);
		return;
	}

	data->trigger_src = new_mask;
}

int vimon_register_callback(struct device *dev, const u16 mask, const int count, void *private,
			    struct vimon_client_callbacks *cb)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);
	struct vimon_client_info *client;
	int ret;

	if (!vimon_cache_pool)
		return -ENOMEM;

	client = (struct vimon_client_info *)
		  gen_pool_alloc(vimon_cache_pool, sizeof(struct vimon_client_info));
	if (!client)
		return -ENOMEM;

	client->mask = mask;
	client->count = count;
	client->private_data = private;
	client->client_cb = cb;

	pm_stay_awake(data->dev);

	mutex_lock(&data->vimon_lock);
	mutex_lock(&data->vimon_cb_lock);

	list_add(&client->list, &vimon_clients);
	vimon_update_callback_mask(data);

	/* TODO: consider delay based on response of b/383420815 */
	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_CTRL,
				       MAX77779_BVIM_CTRL_BVIMON_TRIG_MASK);
	if (ret)
		dev_err(data->dev, "Failed to configure BVIM enable(%d)\n", ret);

	mutex_unlock(&data->vimon_cb_lock);
	mutex_unlock(&data->vimon_lock);

	pm_relax(data->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(vimon_register_callback);

/*
 * vimon_unregister_callback
 * - on_removed will not be called: a client request removing callback explicitly.
 */
void vimon_unregister_callback(struct device *dev, struct vimon_client_callbacks *cb)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);
	struct vimon_client_info *client, *temp_node;

	pm_stay_awake(data->dev);

	mutex_lock(&data->vimon_lock);
	mutex_lock(&data->vimon_cb_lock);

	list_for_each_entry_safe(client, temp_node, &vimon_clients, list) {
		if (client->client_cb == cb) {
			list_del(&client->list);
			gen_pool_free(vimon_cache_pool, (unsigned long)client,
				      sizeof(struct vimon_client_info));
		}
	}

	vimon_update_callback_mask(data);

	mutex_unlock(&data->vimon_cb_lock);
	mutex_unlock(&data->vimon_lock);

	pm_relax(data->dev);
}
EXPORT_SYMBOL_GPL(vimon_unregister_callback);

/* requires mutex_lock(&data->vimon_lock); */
static int vimon_is_running(struct max77779_vimon_data *data)
{
	return data->state > MAX77779_VIMON_IDLE;
}

int max77779_external_vimon_reg_read(struct device *dev, uint16_t reg, void *val, int len)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);

	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_raw_read(data->regmap, reg, val, len);
}
EXPORT_SYMBOL_GPL(max77779_external_vimon_reg_read);

int max77779_external_vimon_reg_write(struct device *dev, uint16_t reg, const void *val, int len)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);

	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_raw_write(data->regmap, reg, val, len);
}
EXPORT_SYMBOL_GPL(max77779_external_vimon_reg_write);

int max77779_external_vimon_read_buffer(struct device *dev, uint16_t *buff, size_t *count,
					size_t buff_max)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);
	int ret = 0;
	int copy_count;

	if (!data)
		return -ENODEV;


	copy_count = data->buf_len;

	if (buff_max < data->buf_len)
		copy_count = buff_max;

	memcpy(buff, data->buf, copy_count);
	*count = copy_count;

	return ret;
}
EXPORT_SYMBOL_GPL(max77779_external_vimon_read_buffer);

int max77779_external_vimon_enable(struct device *dev, bool enable)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);
	int ret, reg;

	if (!data)
		return -ENODEV;

	mutex_lock(&data->vimon_lock);

	ret = max77779_vimon_reg_read(data, MAX77779_BVIM_CTRL, &reg);
	if (ret < 0) {
		mutex_unlock(&data->vimon_lock);
		return -EIO;
	}

	reg = _max77779_bvim_ctrl_bvimon_trig_set(reg, enable);
	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_CTRL, reg);
	if (reg < 0) {
		mutex_unlock(&data->vimon_lock);
		return -EIO;
	}

	ret = max77779_vimon_reg_read(data, MAX77779_BVIM_INT_STS, &reg);
	if (ret < 0) {
		mutex_unlock(&data->vimon_lock);
		return -EIO;
	}

	reg = _max77779_bvim_int_sts_bvim_samples_rdy_set(reg, enable);
	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_INT_STS, reg);
	if (ret < 0) {
		mutex_unlock(&data->vimon_lock);
		return -EIO;
	}

	data->state = enable ? MAX77779_VIMON_IDLE : MAX77779_VIMON_DISABLED;

	mutex_unlock(&data->vimon_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(max77779_external_vimon_enable);

static int max77779_vimon_start(struct max77779_vimon_data *data, uint16_t config)
{
	int ret;

	mutex_lock(&data->vimon_lock);

	ret = max77779_vimon_reg_update(data, MAX77779_BVIM_bvim_cfg, config, config);
	if (ret)
		goto vimon_start_exit;

	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_CTRL,
				       MAX77779_BVIM_CTRL_BVIMON_TRIG_MASK);
	if (ret == 0)
		data->state = MAX77779_VIMON_RUNNING;

vimon_start_exit:
	mutex_unlock(&data->vimon_lock);

	return ret;
}

static int max77779_vimon_direct_stop(struct max77779_vimon_data *data)
{
	return data->direct_reg_write(data, MAX77779_BVIM_CTRL, 0);
}

static int max77779_vimon_set_config(struct max77779_vimon_data *data, uint16_t mask)
{
	return max77779_vimon_reg_write(data, MAX77779_BVIM_bvim_cfg, mask);
}

/*
 * BattVIMon's Buffer: (1024-32) bytes
 * -page[0:2] 256byts, page[3]:224(256-32)
 * -ranges
 *   page0: [0x000:0x07F]
 *   page1: [0x080:0x0FF]  ---> 0x80:0xFF
 *   page2: [0x100:0x17F]
 *   page3: [0x180:0x1EF]
 */
static ssize_t max77779_vimon_access_buffer(struct max77779_vimon_data *data, size_t offset,
					    size_t addr_len, uint16_t *buffer, bool toread)
{
	unsigned int target_addr;
	int ret = -1;
	size_t addr_sz;
	size_t rw_sz;
	unsigned int page;
	size_t start = offset;
	const char* type = toread ? "read" : "write";

	/* valid range: 0 - (1024-32) */
	if (offset + addr_len > 992) {
		dev_err(data->dev, "Failed to %s BVIM's buffer: out of range\n", type);
		return -EINVAL;
	}
	page = MAX77779_VIMON_PAGE_MASK & (offset >> 7);
	target_addr = MAX77779_VIMON_OFFSET_BASE + (offset & 0x7F);

	while (addr_len > 0) {
		/*
		 * page = offset / 128
		 * addr_sz   = 256 - (offset % 256)
		 * target_addr = 0x80 + (offset % 256)
		 */

		if (page == MAX77779_VIMON_PAGE_CNT - 1)
			addr_sz = MAX77779_VIMON_LAST_PAGE_SIZE -
				  (target_addr - MAX77779_VIMON_OFFSET_BASE);
		else
			addr_sz = MAX77779_VIMON_PAGE_SIZE -
				  (target_addr - MAX77779_VIMON_OFFSET_BASE);

		if (addr_len < addr_sz)
			addr_sz = addr_len;

		ret = regmap_write(data->regmap, MAX77779_BVIM_PAGE_CTRL, page);
		if (ret < 0) {
			dev_err(data->dev, "page write failed: page: %i\n", page);
			return ret;
		}

		rw_sz = addr_sz * MAX77779_VIMON_BYTES_PER_ENTRY;

		if (toread)
			ret = regmap_raw_read(data->regmap, target_addr, buffer, rw_sz);
		else
			ret = regmap_raw_write(data->regmap, target_addr, buffer, rw_sz);

		if (ret < 0) {
			dev_err(data->dev, "regmap_raw_read or write failed: %d\n", ret);
			return ret;
		}

		offset += addr_sz;
		buffer += addr_sz;
		addr_len -= addr_sz;
		page = MAX77779_VIMON_PAGE_MASK & (page + 1);
		target_addr = MAX77779_VIMON_OFFSET_BASE;
	}

	return (offset - start) * MAX77779_VIMON_BYTES_PER_ENTRY;
}

static ssize_t bvim_cfg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = -1;
	unsigned int val=-1;
	struct max77779_vimon_data *data = dev_get_drvdata(dev);

	ret = max77779_vimon_reg_read(data, MAX77779_BVIM_bvim_cfg, &val);

	if (ret <0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static void max77779_vimon_handle_data(struct work_struct *work)
{
	struct max77779_vimon_data *data = container_of(work, struct max77779_vimon_data,
							read_data_work.work);
	unsigned int bvim_rfap, bvim_rs, rts, rsc, bvim_osc, smpl_start_add;
	int ret, rd_addr_cnt, rd_bytes;
	struct vimon_client_info *client, *temp_node;
	bool trigger_callback, update_callback_mask;
	enum vimon_trigger_source reason;

	pm_stay_awake(data->dev);
	mutex_lock(&data->vimon_lock);

	if (data->state != MAX77779_VIMON_DATA_AVAILABLE) {
		ret = -ENODATA;
		goto vimon_handle_data_exit;
	}

	ret = max77779_vimon_reg_read(data, MAX77779_BVIM_bvim_rfap, &bvim_rfap);
	if (ret)
		goto vimon_handle_data_exit;

	ret = max77779_vimon_reg_read(data, MAX77779_BVIM_bvim_rs, &bvim_rs);
	if (ret)
		goto vimon_handle_data_exit;

	rsc = _max77779_bvim_bvim_rs_rsc_get(bvim_rs);
	if (rsc > MAX77779_VIMON_SMPL_CNT)
		rsc = MAX77779_VIMON_SMPL_CNT;

	rd_bytes = rsc * MAX77779_VIMON_BYTES_PER_ENTRY * MAX77779_VIMON_ENTRIES_PER_VI_PAIR;

	rd_addr_cnt = rsc * MAX77779_VIMON_ENTRIES_PER_VI_PAIR;

	ret = max77779_vimon_access_buffer(data, bvim_rfap, rd_addr_cnt, data->buf, true);
	if (ret < 0)
		goto vimon_handle_data_exit;

	data->buf_len = ret;

	ret = max77779_vimon_reg_read(data, MAX77779_BVIM_bvim_sts, &bvim_osc);
	if (ret)
		goto vimon_handle_data_exit;

	bvim_osc = _max77779_bvim_bvim_sts_bvim_osc_get(bvim_osc);

	ret = max77779_vimon_reg_read(data, MAX77779_BVIM_smpl_math, &smpl_start_add);
	if (ret)
		goto vimon_handle_data_exit;

	smpl_start_add = _max77779_bvim_smpl_math_smpl_start_add_get(smpl_start_add);


	rts = (bvim_rs & MAX77779_BVIM_bvim_rs_bvim_rts_MASK ) >>
	      MAX77779_BVIM_bvim_rs_bvim_rts_SHIFT;

	mutex_lock(&data->vimon_cb_lock);
	update_callback_mask = false;

	list_for_each_entry_safe(client, temp_node, &vimon_clients, list) {
		trigger_callback = false;
		if (client->mask & BIT(rts)) {
			trigger_callback = true;
			reason = BIT(rts);
		} else if (client->client_cb->extra_trigger &&
			   client->mask & VIMON_CLIENT_REQUEST &&
			   client->client_cb->extra_trigger(client->private_data, data->buf,
							    rd_bytes)) {
			trigger_callback = true;
			reason = VIMON_CLIENT_REQUEST;
		}

		if (trigger_callback) {
			client->client_cb->on_sample_ready(client->private_data, reason, data->buf,
							   rd_bytes);
			if (client->count != VIMON_CLIENT_ALWAYS_RUN)
				client->count--;
		}
		if (client->count == 0) {
			client->client_cb->on_removed(client->private_data);

			list_del(&client->list);
			gen_pool_free(vimon_cache_pool, (unsigned long)client,
				      sizeof(struct vimon_client_info));
			update_callback_mask = true;
		}
	}

	if (update_callback_mask)
		vimon_update_callback_mask(data);

	mutex_unlock(&data->vimon_cb_lock);

vimon_handle_data_exit:

	if (ret)
		dev_dbg(data->dev, "Failed to handle data: (%d).\n", ret);

	data->state = MAX77779_VIMON_IDLE;

	/* TODO: consider continuous mode - b/376772385 */
	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_CTRL,
				       MAX77779_BVIM_CTRL_BVIMON_TRIG_MASK);
	if (ret)
		dev_err(data->dev, "Failed to configure vimon trig\n");

	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_MASK, 0);
	if (ret)
		dev_err(data->dev, "Failed to clear BVIM_MASK(%d).\n", ret);

	mutex_unlock(&data->vimon_lock);
	pm_relax(data->dev);
}

static ssize_t bvim_cfg_store(struct device *dev, struct device_attribute *attr, const char* buf,
			      size_t count)
{
	int ret = -1;
	unsigned int val = -1;
        struct max77779_vimon_data *data = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

        ret = max77779_vimon_reg_write(data, MAX77779_BVIM_bvim_cfg, val);
        return ret < 0 ? ret : count;
}

DEVICE_ATTR(bvim_cfg, 0660, bvim_cfg_show, bvim_cfg_store);

static ssize_t latest_buff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int idx;
	ssize_t count = 0;
	uint16_t rdback;
	struct max77779_vimon_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->vimon_lock);
	for (idx = 0; idx < data->buf_len / MAX77779_VIMON_BYTES_PER_ENTRY; idx++) {
		rdback = data->buf[idx];
		count += sysfs_emit_at(buf, count, "%#x\n", rdback);
	}
	mutex_unlock(&data->vimon_lock);

	return count;
}
static DEVICE_ATTR_RO(latest_buff);

static struct attribute *max77779_vimon_attrs[] = {
	&dev_attr_bvim_cfg.attr,
	&dev_attr_latest_buff.attr,
	NULL,
};

static const struct attribute_group max77779_vimon_attr_grp = {
	.attrs = max77779_vimon_attrs,
};

/* -- debug --------------------------------------------------------------- */
static int max77779_vimon_debug_start(void *d, u64 *val)
{
	struct max77779_vimon_data *data = d;
	int ret;

	ret = max77779_vimon_start(data, MAX77779_BVIM_bvim_cfg_cnt_run_MASK);
	if (ret)
		return ret;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_start_fops, max77779_vimon_debug_start, NULL, "%02llx\n");

static int max77779_vimon_debug_reg_read(void *d, u64 *val)
{
	struct max77779_vimon_data *data = d;
	int ret, reg;

	ret = regmap_read(data->regmap, data->debug_reg_address, &reg);
	if (ret == 0)
		*val = reg & 0xffff;

	return ret;
}

static int max77779_vimon_debug_reg_write(void *d, u64 val)
{
	struct max77779_vimon_data *data = d;

	return regmap_write(data->regmap, data->debug_reg_address, val & 0xffff);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_reg_rw_fops, max77779_vimon_debug_reg_read,
			max77779_vimon_debug_reg_write, "%04llx\n");

static ssize_t max77779_vimon_show_reg_all(struct file *filp, char __user *buf, size_t count,
					   loff_t *ppos)
{
	struct max77779_vimon_data *data = filp->private_data;
	u32 reg_address;
	char *tmp;
	int ret = 0, len = 0;
	int regread;

	if (*ppos)
		return 0;

	if (!data->regmap)
		return -EIO;

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for (reg_address = 0; reg_address <= 0x7F; reg_address++) {
		ret = regmap_read(data->regmap, reg_address, &regread);
		if (ret < 0)
			continue;

		len += scnprintf(tmp + len, PAGE_SIZE - len, "%02x: %04x\n", reg_address,
				regread & 0xffff);
	}

	if (len > 0)
		len = simple_read_from_buffer(buf, count, ppos, tmp, len);

	kfree(tmp);

	return len;
}
BATTERY_DEBUG_ATTRIBUTE(debug_vimon_all_reg_fops, max77779_vimon_show_reg_all, NULL);

static ssize_t max77779_vimon_show_buff_all(struct file *filp, char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct max77779_vimon_data *data = filp->private_data;
	char *tmp;
	uint16_t *vals;
	int ret;
	int len = 0;
	int i;
	const size_t last_readback_size = MAX77779_VIMON_LAST_PAGE_SIZE *
					  MAX77779_VIMON_BYTES_PER_ENTRY;
	const size_t readback_size = MAX77779_VIMON_PAGE_SIZE * MAX77779_VIMON_BYTES_PER_ENTRY;
	int readback_cnt;

	if (*ppos)
		return 0;

	if (!data->regmap)
		return -EIO;

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	vals = kcalloc(MAX77779_VIMON_PAGE_SIZE, sizeof(uint16_t), GFP_KERNEL);

	mutex_lock(&data->vimon_lock);
	ret = regmap_write(data->regmap, MAX77779_BVIM_PAGE_CTRL, data->debug_buffer_page);
	if (ret < 0)
		goto vimon_show_buff_exit;

	if (data->debug_buffer_page < MAX77779_VIMON_PAGE_CNT - 1) {
		ret = regmap_raw_read(data->regmap, MAX77779_VIMON_OFFSET_BASE, vals,
				      readback_size);
		readback_cnt = MAX77779_VIMON_PAGE_SIZE;
	} else {
		ret = regmap_raw_read(data->regmap, MAX77779_VIMON_OFFSET_BASE, vals,
				      last_readback_size);
		readback_cnt = MAX77779_VIMON_LAST_PAGE_SIZE;
	}

	if (ret < 0)
		goto vimon_show_buff_exit;

	for (i = 0; i < readback_cnt; i++)
		len += scnprintf(tmp + len, PAGE_SIZE - len, "%02x: %04x\n",
				 data->debug_buffer_page * MAX77779_VIMON_PAGE_SIZE + i, vals[i]);

	if (len > 0)
		len = simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));

	ret = len;

vimon_show_buff_exit:
	mutex_unlock(&data->vimon_lock);

	kfree(tmp);
	kfree(vals);

	return ret;
}
BATTERY_DEBUG_ATTRIBUTE(debug_vimon_all_buff_fops, max77779_vimon_show_buff_all, NULL);

static ssize_t max77779_vimon_debug_monitor(struct file *filp, const char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct max77779_vimon_data *data = filp->private_data;
	char temp_buf[VIMON_DBG_TEMP_BUFFER_SZ];
	int mask = 0, cnt = 0, ret;

	ret = simple_write_to_buffer(temp_buf, VIMON_DBG_TEMP_BUFFER_SZ, ppos, buf, count);
	if (ret < 0)
		return ret;

	if (sscanf(temp_buf, "%d %d", &mask, &cnt) != 2) {
		dev_err(data->dev, "invalid argument: format should be mask counter\n");
		return -EINVAL;
	}

	ret = vimon_register_callback(data->dev, (u16)mask, cnt, data, &debug_cb_impl);
	if (ret) {
		dev_err(data->dev, "failed to register vimon client callback(%d)\n", ret);
		return ret;
	}

	dev_info(data->dev, "registered debug callback mask(%04x), count(%d)\n", mask, cnt);

	return (ssize_t)count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_monitor_fops, NULL, max77779_vimon_debug_monitor);

static int max77779_vimon_debug_buff_page_read(void *d, u64 *val)
{
	struct max77779_vimon_data *data = d;

	*val = data->debug_buffer_page;

	return 0;
}

static int max77779_vimon_debug_buff_page_write(void *d, u64 val)
{
	struct max77779_vimon_data *data = d;

	if (val >= MAX77779_VIMON_PAGE_CNT)
		return -EINVAL;

	data->debug_buffer_page = (u8)val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_buff_page_rw_fops, max77779_vimon_debug_buff_page_read,
			max77779_vimon_debug_buff_page_write, "%llu\n");

bool max77779_vimon_is_reg(struct device *dev, unsigned int reg)
{
	return reg >= 0 && reg <= MAX77779_VIMON_SIZE;
}
EXPORT_SYMBOL_GPL(max77779_vimon_is_reg);

static int max77779_vimon_init_fs(struct max77779_vimon_data *data)
{
	int ret = -1;

	ret = sysfs_create_group(&data->dev->kobj, &max77779_vimon_attr_grp);
	if (ret < 0) {
		dev_err(data->dev, "Failed to create sysfs group ret:%d\n", ret);
		return ret;
	}

	data->de = debugfs_create_dir("max77779_vimon", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_u32("address", 0600, data->de, &data->debug_reg_address);
	debugfs_create_file("data", 0600, data->de, data, &debug_reg_rw_fops);
	debugfs_create_file("registers", 0444, data->de, data, &debug_vimon_all_reg_fops);

	debugfs_create_file("start", 0600, data->de, data, &debug_start_fops);
	debugfs_create_file("buffer", 0444, data->de, data, &debug_vimon_all_buff_fops);
	debugfs_create_file("buffer_page", 0600, data->de, data, &debug_buff_page_rw_fops);
	debugfs_create_bool("run_in_offmode", 0644, data->de, &data->run_in_offmode);

	debugfs_create_file("debug_monitor", 0444, data->de, data, &debug_monitor_fops);

	return 0;
}

static int max77779_vimon_reboot_notifier(struct notifier_block *nb,
					  unsigned long val, void *v)
{
	struct max77779_vimon_data *data =
		container_of(nb, struct max77779_vimon_data, reboot_notifier);
	int running;

	running = max77779_vimon_direct_is_running(data);
	if (running < 0)
		dev_err(data->dev, "cannot read VIMON HW state (%d)\n", running);
	if (running || vimon_is_running(data))
		dev_warn(data->dev, "vimon state HW=%d SW=%d\n",
			 running, data->state);

	/* stop the HW, warn on inconsistency betwee HW and SW state */
	if (!data->run_in_offmode && running) {
		int ret;

		ret = max77779_vimon_direct_stop(data);
		if (ret < 0)
			dev_err(data->dev, "cannot stop vimon acquisition\n");
	}

	return NOTIFY_OK;
}

/* IRQ */
static irqreturn_t max77779_vimon_irq(int irq, void *ptr)
{
	struct max77779_vimon_data *data = ptr;
	int ret;

	if (data->state <= MAX77779_VIMON_DISABLED)
		return IRQ_HANDLED;

	if (data->state >= MAX77779_VIMON_DATA_AVAILABLE)
		goto vimon_rearm_interrupt;

	data->state = MAX77779_VIMON_DATA_AVAILABLE;

	ret = regmap_write(data->regmap, MAX77779_BVIM_MASK,
			   MAX77779_BVIM_MASK_BVIM_Samples_Rdy_m_MASK);
	if (ret)
		dev_err(data->dev, "Failed to set BVIM_MASK (%d).", ret);

	schedule_delayed_work(&data->read_data_work,
			      msecs_to_jiffies(MAX77779_VIMON_DATA_RETRIEVE_DELAY));

vimon_rearm_interrupt:

	ret = regmap_write(data->regmap, MAX77779_BVIM_INT_STS,
			   MAX77779_BVIM_INT_STS_BVIM_Samples_Rdy_MASK);
	if (ret)
		dev_err(data->dev, "Failed to clear INT_STS (%d).", ret);


	return IRQ_HANDLED;
}

/*
 * Initialization requirements
 * struct max77779_vimon_data *data
 * - dev
 * - regmap
 * - irq
 */
int max77779_vimon_init(struct max77779_vimon_data *data)
{
	struct device *dev = data->dev;
	unsigned int running;
	uint16_t cfg_mask = 0;
	uint16_t cfg_mask_lower_bits = 0;
	unsigned long min_alloc_order;
	int ret;

	/* VIMON can be used to profile battery drain during reboot */
	running = max77779_vimon_is_running(data);
	if (running)
		dev_warn(data->dev, "VIMON is already running (%d)\n", running);
	mutex_init(&data->vimon_lock);
	mutex_init(&data->vimon_cb_lock);

	/* configure collected sample count with MAX77779_VIMON_SMPL_CNT */
	cfg_mask = MAX77779_BVIM_bvim_cfg_vioaok_stop_MASK |
		   MAX77779_BVIM_bvim_cfg_top_fault_stop_MASK;

	cfg_mask_lower_bits = _max77779_bvim_bvim_cfg_smpl_n_set(cfg_mask_lower_bits,
								 MAX77779_VIMON_SMPL_CNT);

	cfg_mask |= cfg_mask_lower_bits;

	ret = max77779_vimon_set_config(data, cfg_mask);
	if (ret) {
		dev_err(dev, "Failed to configure vimon\n");
		return ret;
	}

	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_bvim_trig, 0);
	if (ret) {
		dev_err(dev, "Failed to configure vimon trig\n");
		return ret;
	}

	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_CTRL,
				       MAX77779_BVIM_CTRL_BVIMON_TRIG_MASK);
	if (ret) {
		dev_err(dev, "Failed to configure BVIM enable\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "max77779,max_cnt", &data->max_cnt);
	if (ret)
		data->max_cnt = MAX77779_VIMON_DEFAULT_MAX_CNT;

	ret = of_property_read_u32(dev->of_node, "max77779,max_triggers", &data->max_cnt);
	if (ret)
		data->max_triggers = MAX77779_VIMON_DEFAULT_MAX_TRIGGERS;

	data->buf_size = sizeof(*data->buf) * data->max_cnt * data->max_triggers * 2;
	if (!data->buf_size) {
		dev_err(dev, "max_cnt=%d, max_cnt=%d invalid buf_size\n",
			data->max_cnt, data->max_triggers);
		return -EINVAL;
	}
	data->buf = devm_kzalloc(dev, data->buf_size, GFP_KERNEL);
	if (!data->buf)
		return -ENOMEM;

	INIT_DELAYED_WORK(&data->read_data_work, max77779_vimon_handle_data);

	if (data->irq){
		ret = devm_request_threaded_irq(data->dev, data->irq, NULL,
				max77779_vimon_irq,
				IRQF_TRIGGER_LOW | IRQF_SHARED | IRQF_ONESHOT,
				"max77779_vimon", data);
		if (ret < 0)
			dev_warn(dev, "Failed to get irq thread.\n");
	} else {
		dev_warn(dev, "irq not setup\n");
	}

	ret = max77779_vimon_init_fs(data);
	if (ret < 0)
		dev_warn(dev, "Failed to initialize debug fs\n");

	/* turn off vimon on reboot */
	data->reboot_notifier.notifier_call = max77779_vimon_reboot_notifier;
	ret = register_reboot_notifier(&data->reboot_notifier);
	if (ret)
		dev_err(data->dev, "failed to register reboot notifier\n");

	ret = max77779_vimon_reg_write(data, MAX77779_BVIM_MASK, 0);
	if (ret)
		dev_err(data->dev, "Failed to unmask INT (%d).\n", ret);

	data->state = MAX77779_VIMON_IDLE;
	dev_info(data->dev, "buf_size=%lu\n", data->buf_size);

	INIT_LIST_HEAD(&vimon_clients);

	min_alloc_order = fls_long(sizeof(struct vimon_client_info) - 1);
	vimon_cache_pool = gen_pool_create(min_alloc_order, -1);
	if (vimon_cache_pool) {
		const size_t mem_size = MAX77779_VIMON_MAX_CLIENT * BIT(min_alloc_order);

		vimon_cache_mem = kzalloc(mem_size, GFP_KERNEL);
		if (!vimon_cache_mem) {
			gen_pool_destroy(vimon_cache_pool);
			vimon_cache_pool = NULL;
		} else {
			gen_pool_add(vimon_cache_pool, (unsigned long)vimon_cache_mem,
				     mem_size, -1);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_vimon_init);


void max77779_vimon_remove(struct max77779_vimon_data *data)
{
	unsigned int running;
	struct vimon_client_info *client, *temp_node;

	mutex_lock(&data->vimon_cb_lock);

	list_for_each_entry_safe(client, temp_node, &vimon_clients, list) {
		client->client_cb->on_removed(client->private_data);

		list_del(&client->list);
		gen_pool_free(vimon_cache_pool, (unsigned long)client,
			      sizeof(struct vimon_client_info));
	}

	mutex_unlock(&data->vimon_cb_lock);

	if (vimon_cache_pool) {
		gen_pool_destroy(vimon_cache_pool);
		kfree(vimon_cache_mem);
	}

	running = max77779_vimon_is_running(data);
	if (running < 0)
		dev_err(data->dev, "cannot read VIMON HW state (%d)\n", running);
	if (running || vimon_is_running(data))
		dev_warn(data->dev, "vimon state HW=%d SW=%d\n",
			 running, data->state);

	if (data->de)
		debugfs_remove(data->de);
	if (data->irq)
		free_irq(data->irq, data);
}
EXPORT_SYMBOL_GPL(max77779_vimon_remove);

MODULE_DESCRIPTION("max77779 VIMON Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_AUTHOR("Chungro Lee <chungro@google.com>");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_AUTHOR("Hiroshi Akiyama <hiroshiakiyama@google.com>");
MODULE_LICENSE("GPL");
