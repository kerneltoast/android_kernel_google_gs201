/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include "gbms_power_supply.h"
#include "gbms_storage.h"

#define MAX77729_UIC_HW_REV                  0x00
#define MAX77729_UIC_FW_REV                  0x01
#define MAX77729_UIC_INT                     0x02
#define MAX77729_CC_INT                      0x03
#define MAX77729_PD_INT                      0x04
#define MAX77729_VDM_INT                     0x05
#define MAX77729_USBC_STATUS1                0x06
#define MAX77729_USBC_STATUS2                0x07
#define MAX77729_BC_STATUS                   0x08
#define MAX77729_UIC_FW_REV2                 0x09
#define MAX77729_CC_STATUS1                  0x0a
#define MAX77729_CC_STATUS2                  0x0b
#define MAX77729_PD_STATUS1                  0x0c
#define MAX77729_PD_STATUS2                  0x0d
#define MAX77729_UIC_INT_M                   0x0e
#define MAX77729_CC_INT_M                    0x0f
#define MAX77729_PD_INT_M                    0x10

#define MAX77729_AP_DATAOUT0                 0x21
#define MAX77729_AP_DATAOUT1                 0x22
#define MAX77729_AP_DATAOUT32                0x41
#define MAX77729_AP_DATAIN0                  0x51
#define MAX77729_AP_DATAIN1                  0x52

#define MAX77729_UIC_BC_CTRL1_CONFIG_READ  0x01
#define MAX77729_UIC_BC_CTRL1_CONFIG_WRITE 0x02

#define NOAUTOIBUS_SHIFT                        5
#define WAIT_STEP_MS                           10
#define WAIT_MAX_TRIES                         10

#define MAX77729_UIC_INT_APCMDRESI           0x80
#define MAX77729_UIC_INT_SYSMSGI             0x40
#define MAX77729_UIC_INT_VBUSDETI            0x20
#define MAX77729_UIC_INT_VBADCI              0x10
#define MAX77729_UIC_INT_DCDTMOI             0x08
#define MAX77729_UIC_INT_FAKEVBUSI           0x04
#define MAX77729_UIC_INT_CHGTYPI             0x02
#define MAX77729_UIC_INT_UIDADCI             0x01

#define MAX77729_BC_STATUS_VBUSDET           0x80

#define NAI_DWELL_TIME                       3000
#define BC_CTRL1_DWELL_TIME                  1000
#define BC_CTRL1_DEFAULT                     0xe5
#define OP_CC_CTRL_WRITE                     0x0c
#define OP_CC_CTRL_CCSRCSNK                  0x10

#define CHGTYP_MASK                          GENMASK(1, 0)
#define CHGTYP_NONE                          0x0
#define CHGTYP_SDP                           0x1
#define CHGTYP_CDP                           0x2
#define CHGTYP_DCP                           0x3

#define OP_GPIOX_READ                        0x23
#define OP_GPIOX_WRITE                       0x24
#define GPIO_DIR_OUT                         1
#define GPIO_DIR_IN                          0
#define GPIO_OUT_HI                          1
#define GPIO_OUT_LO                          0

#define OP_CC_CTRL3_READ                     0xF
#define CC_LP_MODE                           BIT(4)
#define OP_CC_CTRL3_WRITE                    0x10

#define MAX77729_STORAGE_SIZE	8
#define MAX77729_STORAGE_BASE	(MAX77729_AP_DATAOUT0 + MAX77729_STORAGE_SIZE)

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define MAX77729_INT_DEFAULT_MASK (MAX77729_UIC_INT_VBUSDETI |	\
				   MAX77729_UIC_INT_APCMDRESI |	\
				   MAX77729_UIC_INT_CHGTYPI |	\
				   MAX77729_UIC_INT_DCDTMOI)

struct max77729_uic_data {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	int irq;
	int irq_gpio;
	uint8_t bc_ctrl1;
	uint8_t cmd_pending;
	struct delayed_work noautoibus_work;
	struct completion cmd_done;
	struct mutex io_lock;
	struct power_supply *usb_psy;
	struct mutex gpio_lock;
	struct mutex cc_ctrl3_lock;
	bool probe_done;

	struct dentry *de;
};

/* silence warning */
extern int max77729_disable_water_detection(struct i2c_client *client);
extern int max77729_gpio_set(struct i2c_client *client, unsigned int gpio,
		      bool dir_out, bool out_hi);


static bool max77729_uic_is_reg(struct device *dev, unsigned int reg)
{
	int ret;

	switch (reg) {
	case 0x00 ... 0x71:
		ret = true;
		break;
	default:
		ret = false;
		break;
	}
	return ret;
}

static const struct regmap_config max77729_uic_regmap_cfg = {
	.name = "max77729_uic",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = 0x71,
	.readable_reg = max77729_uic_is_reg,
	.volatile_reg = max77729_uic_is_reg,
};

static inline int max77729_uic_read(struct regmap *uic_regmap,
				    int addr, u8 *val, int len)
{
	int rc;

	if (!uic_regmap)
		return -ENXIO;

	rc = regmap_bulk_read(uic_regmap, addr, val, len);
	if (rc < 0) {
		pr_err("regmap_read failed for address %04x rc=%d\n",
			addr, rc);
		return rc;
	}

	return 0;
}

static inline int max77729_uic_write(struct regmap *uic_regmap, int addr,
				     const u8 *val, int len)
{
	int rc;

	if (!uic_regmap)
		return -ENXIO;

	rc = regmap_bulk_write(uic_regmap, addr, val, len);
	if (rc < 0) {
		pr_err("regmap_write failed for address %04x rc=%d\n",
			addr, rc);
		return rc;
	}

	return 0;
}

static inline int max77729_uic_wait(struct i2c_client *client)
{
	struct max77729_uic_data *data = i2c_get_clientdata(client);
	unsigned long rc;

	rc = wait_for_completion_timeout(&data->cmd_done,
		msecs_to_jiffies(BC_CTRL1_DWELL_TIME));
	if (!rc) {
		dev_err(data->dev, "timeout waiting for cmd 0x%02x\n",
			data->cmd_pending);
		return -ETIME;
	}
	return 0;
}

/* Adopted from one of the earlier patches from Jim */
static int max77729_uic_opcode_read(struct i2c_client *client,
				    uint8_t op, uint8_t *val)
{
	struct max77729_uic_data *data = i2c_get_clientdata(client);
	int rc;
	uint8_t buf[2];

	mutex_lock(&data->io_lock);
	buf[0] = MAX77729_AP_DATAOUT0;
	buf[1] = op;
	rc = i2c_master_send(client, buf, 2);
	if (rc < 0)
		goto opcode_read_out;
	buf[0] = MAX77729_AP_DATAOUT32;
	buf[1] = 0x00;
	rc = i2c_master_send(client, buf, 2);
	if (rc < 0)
		goto opcode_read_out;

	rc = max77729_uic_wait(client);
	if (rc < 0)
		goto opcode_read_out;

	buf[0] = MAX77729_AP_DATAIN1;
	rc = i2c_master_send(client, buf, 1);
	if (rc < 0)
		goto opcode_read_out;
	rc = i2c_master_recv(client, buf, 1);
	if (rc < 0)
		goto opcode_read_out;

	*val = buf[0];

opcode_read_out:
	mutex_unlock(&data->io_lock);
	return rc;
}

static int max77729_uic_opcode_write(struct i2c_client *client,
				     uint8_t op, uint8_t val)
{
	struct max77729_uic_data *data = i2c_get_clientdata(client);
	int rc;
	uint8_t buf[2];

	mutex_lock(&data->io_lock);
	data->cmd_pending = op;
	reinit_completion(&data->cmd_done);

	/* write updated result */
	buf[0] = MAX77729_AP_DATAOUT0;
	buf[1] = op;
	rc = i2c_master_send(client, buf, 2);
	if (rc < 0)
		goto opcode_write_out;
	buf[0] = MAX77729_AP_DATAOUT1;
	buf[1] = val;
	rc = i2c_master_send(client, buf, 2);
	if (rc < 0)
		goto opcode_write_out;
	buf[0] = MAX77729_AP_DATAOUT32;
	buf[1] = 0x00;
	rc = i2c_master_send(client, buf, 2);
	if (rc < 0)
		goto opcode_write_out;

	rc = max77729_uic_wait(client);

	data->cmd_pending = 0;
opcode_write_out:
	mutex_unlock(&data->io_lock);

	return rc;
}

int max77729_disable_water_detection(struct i2c_client *client)
{
	uint8_t ctrl3_write, ctrl3_readback;
	struct max77729_uic_data *data = i2c_get_clientdata(client);

	if (!data || !data->probe_done)
		return -EAGAIN;

	mutex_lock(&data->cc_ctrl3_lock);
	max77729_uic_opcode_read(client, OP_CC_CTRL3_READ,
				 &ctrl3_write);
	ctrl3_write = ctrl3_write & ~CC_LP_MODE;

	max77729_uic_opcode_write(client, OP_CC_CTRL3_WRITE,
				  ctrl3_write);
	max77729_uic_opcode_read(client, OP_CC_CTRL3_READ,
				 &ctrl3_readback);
	mutex_unlock(&data->cc_ctrl3_lock);

	return ctrl3_write == ctrl3_readback ? 0 : -1;
}
EXPORT_SYMBOL_GPL(max77729_disable_water_detection);

int max77729_gpio_set(struct i2c_client *client, unsigned int gpio,
		      bool dir_out, bool out_hi)
{
	uint8_t gpio_dir_val = 0, gpio_out_val = 0, gpio_val = 0;
	uint8_t gpio_dir_offset = 0, gpio_current_setting = 0,
		gpio_read_back;
	struct max77729_uic_data *data = i2c_get_clientdata(client);

	if (!data || !data->probe_done)
		return -EAGAIN;

	gpio_dir_val = dir_out ? GPIO_DIR_OUT : GPIO_DIR_IN;
	if (dir_out)
		gpio_out_val = out_hi ? GPIO_OUT_HI : GPIO_OUT_LO;

	gpio_dir_offset = (gpio - 1) * 2;

	gpio_val = ((gpio_out_val << 1) | gpio_dir_val) << gpio_dir_offset;

	mutex_lock(&data->gpio_lock);
	max77729_uic_opcode_read(client, OP_GPIOX_READ,
				 &gpio_current_setting);

	/* Leave other gpios in the same state */
	gpio_current_setting &= (0xFF & ~(3 << gpio_dir_offset));
	gpio_current_setting |= gpio_val;

	max77729_uic_opcode_write(client, OP_GPIOX_WRITE,
				  gpio_current_setting);

	max77729_uic_opcode_read(client, OP_GPIOX_READ,
				 &gpio_read_back);
	mutex_unlock(&data->gpio_lock);

	return gpio_current_setting == gpio_read_back ? 0 : -1;
}
EXPORT_SYMBOL_GPL(max77729_gpio_set);

/* TODO: implement this */
static int max77729_uic_noautoibus_get(struct i2c_client *client)
{
	return 1;
}

static int max77729_uic_noautoibus_set(struct i2c_client *client)
{
	struct max77729_uic_data *data = i2c_get_clientdata(client);
	uint8_t bc_stat = 0;
	int ret;

	ret = max77729_uic_read(data->regmap, MAX77729_BC_STATUS, &bc_stat, 1);
	if (ret < 0)
		return ret;

	if (bc_stat & MAX77729_BC_STATUS_VBUSDET)
		max77729_uic_opcode_write(data->client, OP_CC_CTRL_WRITE,
					  OP_CC_CTRL_CCSRCSNK);

	max77729_uic_opcode_write(data->client,
				  MAX77729_UIC_BC_CTRL1_CONFIG_WRITE,
				  data->bc_ctrl1);
	return 0;
}

static inline void max77729_cmd_complete(struct i2c_client *client)
{
	struct max77729_uic_data *data = i2c_get_clientdata(client);
	int rc;
	uint8_t buf[1];

	buf[0] = MAX77729_AP_DATAIN0;
	rc = i2c_master_send(client, buf, 1);
	if (rc < 0)
		return;
	rc = i2c_master_recv(client, buf, 1);
	if (rc < 0)
		return;

	if (buf[0] == data->cmd_pending)
		complete_all(&data->cmd_done);
}

static void max77729_report_chgtype(struct max77729_uic_data *data)
{
	union power_supply_propval val = { 0 };
	enum power_supply_usb_type usb_type;
	uint8_t bc_status = 0;
	int ret;

	ret = max77729_uic_read(data->regmap, MAX77729_BC_STATUS, &bc_status,
				  1);
	dev_info(data->dev, "report_chgtype bc_status:%x ret:%d\n",
		 bc_status, ret);
	if (ret < 0)
		return;

	switch (bc_status & CHGTYP_MASK) {
	case CHGTYP_NONE:
		usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	case CHGTYP_SDP:
		usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case CHGTYP_CDP:
		usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case CHGTYP_DCP:
		usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	}

	val.intval = usb_type;
	ret = power_supply_set_property(data->usb_psy,
					POWER_SUPPLY_PROP_USB_TYPE,
					&val);
	if (ret)
		dev_err(data->dev, "BC12: usb_psy update failed (%d)", ret);
}

static irqreturn_t max77729_uic_irq(int irq, void *client)
{
	struct max77729_uic_data *data = i2c_get_clientdata(client);
	uint8_t pd_int = 0, cc_int = 0, vdm_int = 0, uic_int;
	int ret;

	/* clear any ints we don't care about */
	max77729_uic_read(data->regmap, MAX77729_PD_INT, &pd_int, 1);
	max77729_uic_read(data->regmap, MAX77729_CC_INT, &cc_int, 1);
	max77729_uic_read(data->regmap, MAX77729_VDM_INT, &vdm_int, 1);

	ret = max77729_uic_read(data->regmap, MAX77729_UIC_INT, &uic_int, 1);
	if (ret < 0) {
		dev_err_ratelimited(data->dev,
			"failed to read register 0x%02x\n", MAX77729_UIC_INT);
		return IRQ_NONE;
	}
	if (!uic_int)
		return IRQ_NONE;

	if (uic_int & MAX77729_UIC_INT_VBUSDETI)
		mod_delayed_work(system_wq, &data->noautoibus_work,
				 msecs_to_jiffies(NAI_DWELL_TIME));

	if (uic_int & MAX77729_UIC_INT_APCMDRESI)
		max77729_cmd_complete(client);

	if (uic_int & MAX77729_UIC_INT_CHGTYPI) {
		dev_info(data->dev, "BC1.2 CHGTYPI\n");
		max77729_report_chgtype(data);
	}

	if (uic_int & MAX77729_UIC_INT_DCDTMOI)
		dev_err(data->dev, "BC1.2 DCDTMO\n");

	return IRQ_HANDLED;
}

static uint8_t max77729_get_bc_ctrl1(struct device *dev)
{
	struct property *np;
	uint32_t val;
	uint8_t cfg = BC_CTRL1_DEFAULT;

	np = of_find_property(dev->of_node, "bc1_config", NULL);
	if (np) {
		if (!of_property_read_u32(dev->of_node, "bc1_config", &val))
			cfg = val & 0xff;
	}
	return cfg;
}

static void max77729_noautoibus_worker(struct work_struct *work)
{
	struct max77729_uic_data *data = container_of(to_delayed_work(work),
			struct max77729_uic_data, noautoibus_work);
	int ret, nai = -1;

	ret = max77729_uic_noautoibus_set(data->client);
	if (ret == 0)
		nai = max77729_uic_noautoibus_get(data->client);
	if (nai <= 0)
		mod_delayed_work(system_wq, &data->noautoibus_work,
				 msecs_to_jiffies(BC_CTRL1_DWELL_TIME));

	dev_err(data->dev, "NoAutoIbus WORK ret = %d, nai=%d\n", ret, nai);
}


#ifdef CONFIG_DEBUG_FS
static int max77729_dbg_set_noautoibus(void *d, u64 val)
{
	struct max77729_uic_data *data = d;
	int ret, nai = -1;

	ret = max77729_uic_noautoibus_set(data->client);
	if (ret == 0)
		nai = max77729_uic_noautoibus_get(data->client);

	dev_err(data->dev, "NoAutoIbus = %d\n", nai);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(max77729_noautoibus_fops, NULL,
			max77729_dbg_set_noautoibus, "%llu\n");

static int dbg_init_fs(struct max77729_uic_data *data)
{
	data->de = debugfs_create_dir("max77729_maxq", NULL);
	if (!data->de)
		return -EINVAL;

	debugfs_create_file("noautoibus", 0644, data->de, data,
				&max77729_noautoibus_fops);
	return 0;
}

#else
static int dbg_init_fs(struct max77729_chgr_data *data)
{
	return 0;
}
#endif

static int max77729_uic_storage_iter(int index, gbms_tag_t *tag, void *ptr)
{
	if (index < 0 || index > (GBMS_TAG_RRS7 - GBMS_TAG_RRS0))
		return -ENOENT;
	*tag = GBMS_TAG_RRS0 + index;
	return 0;
}

static int max77729_uic_storage_read(gbms_tag_t tag, void *buff, size_t size,
				     void *ptr)
{
	const int base = MAX77729_STORAGE_BASE + tag - GBMS_TAG_RRS0;
	struct max77729_uic_data *data = ptr;
	int ret;

	if (tag < GBMS_TAG_RRS0 || tag > GBMS_TAG_RRS7)
		return -ENOENT;
	if ((tag + size - 1) > GBMS_TAG_RRS7)
		return -ERANGE;

	ret = max77729_uic_read(data->regmap, base, buff, size);
	if (ret < 0)
		ret = -EIO;
	return ret;
}

static int max77729_uic_storage_write(gbms_tag_t tag, const void *buff,
				      size_t size, void *ptr)
{
	const int base = MAX77729_STORAGE_BASE + tag - GBMS_TAG_RRS0;
	struct max77729_uic_data *data = ptr;
	int ret;

	if (tag < GBMS_TAG_RRS0 || tag > GBMS_TAG_RRS7)
		return -ENOENT;
	if ((tag + size - 1) > GBMS_TAG_RRS7)
		return -ERANGE;

	ret = max77729_uic_write(data->regmap, base, buff, size);
	if (ret < 0)
		ret = -EIO;
	return ret;
}

static struct gbms_storage_desc max77729_uic_storage_dsc = {
	.iter = max77729_uic_storage_iter,
	.read = max77729_uic_storage_read,
	.write = max77729_uic_storage_write,
};

static int max77729_uic_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct max77729_uic_data *data;
	struct device *dev = &client->dev;
	struct power_supply *usb_psy;
	const char *usb_psy_name;
	uint8_t irq_mask = 0x0; /* unmask all */
	uint8_t hw_rev = 0;
	int ret;

	usb_psy_name = of_get_property(dev->of_node, "usb-psy-name", NULL);
	if (!usb_psy_name) {
		dev_err(dev, "usb-psy-name not set\n");
		return -EINVAL;
	}

	usb_psy = power_supply_get_by_name(usb_psy_name);
	if (!usb_psy) {
		dev_err(&client->dev, "usb psy not up, retrying....\n");
		return -EPROBE_DEFER;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	i2c_set_clientdata(client, data);
	data->client = client;
	data->usb_psy = usb_psy;

	data->regmap = devm_regmap_init_i2c(client, &max77729_uic_regmap_cfg);
	if (IS_ERR(data->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		ret = -EINVAL;
		goto exit;
	}

	/* check chip is available */
	ret = max77729_uic_read(data->regmap, MAX77729_UIC_HW_REV, &hw_rev, 1);
	if (ret < 0) {
		dev_err(dev, "device not available\n");
		return ret;
	}

	init_completion(&data->cmd_done);
	INIT_DELAYED_WORK(&data->noautoibus_work, max77729_noautoibus_worker);
	mutex_init(&data->io_lock);
	mutex_init(&data->gpio_lock);
	mutex_init(&data->cc_ctrl3_lock);

	data->bc_ctrl1 = max77729_get_bc_ctrl1(dev);

	data->irq_gpio = of_get_named_gpio(dev->of_node,
					   "max77729,irq-gpio", 0);
	if (data->irq_gpio < 0) {
		dev_err(dev, "failed get irq_gpio\n");
		return -EINVAL;
	}
	client->irq = gpio_to_irq(data->irq_gpio);

	ret = devm_request_threaded_irq(data->dev, client->irq, NULL,
					max77729_uic_irq,
					IRQF_TRIGGER_LOW |
					IRQF_SHARED |
					IRQF_ONESHOT,
					"maxq", client);
	if (ret == 0)
		enable_irq_wake(client->irq);

	/* handle pending interrupts, unmask the interesting ones */
	max77729_uic_irq(-1, client);
	ret = max77729_uic_write(data->regmap, MAX77729_UIC_INT_M,
				 &irq_mask, 1);
	if (ret < 0)
		dev_err(dev, "cannot reset irq mask %d", ret);
	/* report port type since the irq might have been cleared in BL */
	max77729_report_chgtype(data);

	/* TODO: move at the beginning of probe */
	ret = gbms_storage_register(&max77729_uic_storage_dsc,
				    "max77729uic", data);

	dev_info(dev, "maxq: hw_rev=%x mask=%x st=%d init_work done\n",
		 hw_rev, irq_mask, ret);

	if (ret == -EBUSY)
		ret = 0;

	dbg_init_fs(data);
	data->probe_done = true;
exit:
	return ret;
}

static void max77729_uic_remove(struct i2c_client *client)
{
	struct max77729_uic_data *data = i2c_get_clientdata(client);

	cancel_delayed_work(&data->noautoibus_work);
}

static const struct of_device_id max77729_uic_of_match_table[] = {
	{ .compatible = "maxim,max77729uic"},
	{},
};
MODULE_DEVICE_TABLE(of, max77729_uic_of_match_table);

static const struct i2c_device_id max77729_uic_id[] = {
	{"max77729_uic", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77729_uic_id);

#if defined CONFIG_PM
static int max77729_uic_pm_suspend(struct device *dev)
{
	return 0; /* TODO */
}

static int max77729_uic_pm_resume(struct device *dev)
{
	return 0; /* TODO */
}
#endif

static const struct dev_pm_ops max77729_uic_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(
		max77729_uic_pm_suspend,
		max77729_uic_pm_resume)
};

static struct i2c_driver max77729_uic_i2c_driver = {
	.driver = {
		.name = "max77729-uic",
		.owner = THIS_MODULE,
		.of_match_table = max77729_uic_of_match_table,
#ifdef CONFIG_PM
		.pm = &max77729_uic_pm_ops,
#endif
	},
	.id_table = max77729_uic_id,
	.probe = max77729_uic_probe,
	.remove = max77729_uic_remove,
};

module_i2c_driver(max77729_uic_i2c_driver);
MODULE_DESCRIPTION("Maxim 77729 UIC driver");
MODULE_AUTHOR("Jim Wylder jwylder@google.com");
MODULE_LICENSE("GPL");
