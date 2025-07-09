// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2023 Google LLC
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>

#include "max77779_pmic.h"
#include "max77779_i2cm.h"

static int max77779_i2cm_done(struct max77779_i2cm_info *info,
		unsigned int *status)
{
	unsigned int timeout;

	timeout = msecs_to_jiffies(info->completion_timeout_ms);
	if (!wait_for_completion_timeout(&info->xfer_done, timeout)) {
		dev_err(info->dev, "Xfer timed out.\n");
		return -ETIMEDOUT;
	}

	return regmap_read(info->regmap, MAX77779_I2CM_STATUS, status);
}

static irqreturn_t max777x9_i2cm_irq(int irq, void *ptr)
{
	struct max77779_i2cm_info *info = ptr;
	unsigned int val;
	int err;

	err = regmap_read(info->regmap, MAX77779_I2CM_INTERRUPT, &val);
	if (err) {
		dev_err(info->dev, "Failed to read Interrupt (%d).\n",
				err);
		return IRQ_NONE;
	}
	if (DONEI_GET(val))
		complete(&info->xfer_done);

	/* clear interrupt */
	regmap_write(info->regmap, MAX77779_I2CM_INTERRUPT,
			ERRI_SET(1) | DONEI_SET(1));
	return IRQ_HANDLED;
}

static inline void set_regval(struct max77779_i2cm_info *info,
		unsigned int reg, unsigned int val)
{
	u8 val8 = (u8)val;

	if (reg > I2CM_MAX_REGISTER) {
		dev_err(info->dev, "reg too large %#04x\n", reg);
		return;
	}
	info->reg_vals[reg] = val8;
}

static int max77779_i2cm_xfer(struct i2c_adapter *adap,
		struct i2c_msg *msgs, int num_msgs)
{
	struct max77779_i2cm_info *info =
			container_of(adap, struct max77779_i2cm_info, adap);
	struct regmap *regmap = info->regmap;
	unsigned int txdata_cnt = 0;
	unsigned int tx_data_buffer = MAX77779_I2CM_TX_BUFFER_0;
	unsigned int rxdata_cnt = 0;
	unsigned int rx_data_buffer = MAX77779_I2CM_RX_BUFFER_0;
	unsigned int cmd = 0;
	int i, j;
	int err = 0;
	unsigned int status; /* result of status register */
	uint8_t status_err;

	set_regval(info, MAX77779_I2CM_INTERRUPT, DONEI_SET(1) | ERRI_SET(1));
	set_regval(info, MAX77779_I2CM_INTMASK, ERRIM_SET(0) | DONEIM_SET(0));
	set_regval(info, MAX77779_I2CM_TIMEOUT, info->timeout);
	set_regval(info, MAX77779_I2CM_CONTROL,
			I2CEN_SET(1) | CLOCK_SPEED_SET(info->speed));
	set_regval(info, MAX77779_I2CM_SLADD, SID_SET(msgs[0].addr));

	/* parse message into regval buffer */
	for (i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];

		if (msg->flags & I2C_M_RD) {
			rxdata_cnt += msg->len;
			if (rxdata_cnt  > MAX77779_I2CM_MAX_READ) {
				dev_err(info->dev, "read too large %d > %d\n",
						rxdata_cnt,
						MAX77779_I2CM_MAX_READ);
				return -EINVAL;
			}

			cmd |= I2CMREAD_SET(1);
		} else {
			txdata_cnt += msg->len;
			if (txdata_cnt  > MAX77779_I2CM_MAX_WRITE) {
				dev_err(info->dev, "write too large %d > %d\n",
						txdata_cnt,
						MAX77779_I2CM_MAX_WRITE);
				return -EINVAL;
			}
			cmd |= I2CMWRITE_SET(1);
			for (j = 0; j < msg->len; j++) {
				u8 buf = msg->buf[j];

				set_regval(info, tx_data_buffer, buf);
				tx_data_buffer++;
			}
		}
	}

	set_regval(info, MAX77779_I2CM_TXDATA_CNT, txdata_cnt);

	err = regmap_raw_write(regmap, MAX77779_I2CM_INTERRUPT,
			&info->reg_vals[MAX77779_I2CM_INTERRUPT],
			tx_data_buffer - MAX77779_I2CM_INTERRUPT);
	if (err) {
		dev_err(info->dev, "regmap_raw_write returned %d\n", err);
		goto xfer_done;
	}

	set_regval(info, MAX77779_I2CM_RXDATA_CNT,
			rxdata_cnt > 0 ? rxdata_cnt - 1 : 0);
	set_regval(info, MAX77779_I2CM_CMD, cmd);

	err = regmap_raw_write(regmap, MAX77779_I2CM_RXDATA_CNT,
			&info->reg_vals[MAX77779_I2CM_RXDATA_CNT],
			2);
	if (err) {
		dev_err(info->dev, "regmap_raw_write returned %d\n", err);
		goto xfer_done;
	}

	err = max77779_i2cm_done(info, &status);
	if (err)
		goto xfer_done;
	status_err = ERROR_GET(status);                 /* bit */
	if (I2CM_ERR_ADDRESS_NACK(status_err))          /*  2  */
		err = -ENXIO;
	else if (I2CM_ERR_DATA_NACK(status_err))        /*  3  */
		err = -ENXIO;
	else if (I2CM_ERR_RX_FIFO_NA(status_err))       /*  4  */
		err = -ENOBUFS;
	else if (I2CM_ERR_TIMEOUT(status_err))          /*  1  */
		err = -ETIMEDOUT;
	else if (I2CM_ERR_START_OUT_SEQ(status_err))    /*  5  */
		err = -EBADMSG;
	else if (I2CM_ERR_STOP_OUT_SEQ(status_err))     /*  6  */
		err = -EBADMSG;
	else if (I2CM_ERR_ARBITRATION_LOSS(status_err)) /*  0  */
		err = -EAGAIN;
	if (err) {
		dev_err(info->dev, "I2CM status Error (%#04x).\n", status_err);
		goto xfer_done;
	}

	if (!rxdata_cnt) /* nothing to read we are done. */
		goto xfer_done;

	err = regmap_raw_read(regmap, MAX77779_I2CM_RX_BUFFER_0,
			&info->reg_vals[MAX77779_I2CM_RX_BUFFER_0], rxdata_cnt);
	if (err) {
		dev_err(info->dev, "Error reading = %d\n", err);
		goto xfer_done;
	}

	rx_data_buffer = MAX77779_I2CM_RX_BUFFER_0;
	for (i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];

		if (msg->flags & I2C_M_RD) {
			for (j = 0; j < msg->len; j++) {
				msg->buf[j] = info->reg_vals[rx_data_buffer];
				rx_data_buffer++;
			}
		}
	}

xfer_done:
	set_regval(info, MAX77779_I2CM_INTERRUPT, DONEI_SET(1) | ERRI_SET(1));
	set_regval(info, MAX77779_I2CM_INTMASK, ERRIM_SET(1) | DONEIM_SET(1));

	regmap_raw_write(regmap, MAX77779_I2CM_INTERRUPT,
			&info->reg_vals[MAX77779_I2CM_INTERRUPT], 2);

	if (err) {
		dev_err(info->dev, "Xfer Error (%d)\n", err);
		return err;
	}

	return num_msgs;
}

static u32 max77779_i2cm_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_BYTE |
	       I2C_FUNC_SMBUS_BYTE_DATA |
	       I2C_FUNC_SMBUS_WORD_DATA |
	       I2C_FUNC_SMBUS_BLOCK_DATA|
	       I2C_FUNC_SMBUS_I2C_BLOCK |
	       I2C_FUNC_I2C;
}

static const struct i2c_algorithm max77779_i2cm_algorithm = {
	.master_xfer		= max77779_i2cm_xfer,
	.functionality		= max77779_i2cm_func,
};

static const struct i2c_adapter_quirks max77779_i2cm_quirks = {
	.flags = I2C_AQ_COMB_WRITE_THEN_READ |
		 I2C_AQ_NO_ZERO_LEN |
		 I2C_AQ_NO_REP_START,
	.max_num_msgs = 2,
	.max_write_len = MAX77779_I2CM_MAX_WRITE,
	.max_read_len = MAX77779_I2CM_MAX_READ,
	.max_comb_1st_msg_len = MAX77779_I2CM_MAX_WRITE,
	.max_comb_2nd_msg_len = MAX77779_I2CM_MAX_READ
};

int max77779_i2cm_init(struct max77779_i2cm_info *info)
{
	struct device *dev = info->dev;
	int err = 0;

	if (!IS_ENABLED(CONFIG_OF))
		return -EINVAL;

	/* Device Tree Setup */
	err = of_property_read_u32(dev->of_node, "max77779,timeout",
			&info->timeout);
	if (err || (info->timeout > MAX77779_MAX_TIMEOUT)) {
		dev_warn(dev, "Invalid max77779,timeout set to max.\n");
		info->timeout = MAX77779_TIMEOUT_DEFAULT;
	}

	err = of_property_read_u32(dev->of_node, "max77779,speed",
			&info->speed);
	if (err || (info->speed > MAX77779_MAX_SPEED)) {
		dev_warn(dev, "Invalid max77779,speed - set to min.\n");
		info->speed = MAX77779_SPEED_DEFAULT;
	}

	err = of_property_read_u32(dev->of_node,
			"max77779,completion_timeout_ms",
			&info->completion_timeout_ms);
	if (err)
		info->completion_timeout_ms =
			MAX77779_COMPLETION_TIMEOUT_MS_DEFAULT;

	init_completion(&info->xfer_done);

	if (info->irq) {
		err = devm_request_threaded_irq(info->dev, info->irq, NULL,
				max777x9_i2cm_irq,
				IRQF_TRIGGER_LOW | IRQF_SHARED | IRQF_ONESHOT,
				"max777x9_i2cm", info);
		if (err < 0) {
			dev_err(dev, "Failed to get irq thread.\n");
		} else {
			/*
			* write I2CM_MASK to disable interrupts, they
			* will be enabled during xfer.
			*/
			err = regmap_write(info->regmap, MAX77779_I2CM_INTERRUPT,
					DONEI_SET(1) | ERRI_SET(1));
			if (err) {
				dev_err(dev, "Failed to setup interrupts.\n");
				return -EIO;
			}
		}
	}

	/* setup the adapter */
	strscpy(info->adap.name, "max77779-i2cm", sizeof(info->adap.name));
	info->adap.owner   = THIS_MODULE;
	info->adap.algo    = &max77779_i2cm_algorithm;
	info->adap.retries = 2;
	info->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	info->adap.dev.of_node = dev->of_node;
	info->adap.algo_data = info;
	info->adap.dev.parent = info->dev;
	info->adap.nr = -1;
	info->adap.quirks = &max77779_i2cm_quirks;

	err = i2c_add_numbered_adapter(&info->adap);
	if (err < 0)
		dev_err(dev, "failed to add bus to i2c core\n");

	return err;
}
EXPORT_SYMBOL_GPL(max77779_i2cm_init);

void max77779_i2cm_remove(struct max77779_i2cm_info *info)
{
	devm_kfree(info->dev, info);
}
EXPORT_SYMBOL_GPL(max77779_i2cm_remove);

MODULE_DESCRIPTION("Maxim 77779 I2C Bridge Driver");
MODULE_AUTHOR("Jim Wylder <jwylder@google.com>");
MODULE_LICENSE("GPL");
