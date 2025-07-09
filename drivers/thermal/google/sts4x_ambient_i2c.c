// SPDX-License-Identifier: GPL-2.0
/*
 * sts4x_ambient_i2c.c - STS4x I2C Driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/thermal.h>

/* STS4X Temperature Range. */
#define STS4X_MIN_TEMPERATURE	-40000
#define STS4X_MAX_TEMPERATURE	125000

/* Poll intervals (in milliseconds). */
#define STS4X_MIN_POLL_INTERVAL	2000

/* I2C command delays (in microseconds). */
#define STS4X_MEAS_DELAY_HPM	8300	// t_MEAS,h
#define STS4X_DELAY_EXTRA	10000

/* Command Bytes. */
#define STS4X_CMD_MEASURE_TEMP	0b11111101
#define STS4X_CMD_RESET		0b10010100

/* Command Lengths. */
#define STS4X_CMD_LEN		1
#define STS4X_CRC8_LEN		1
#define STS4X_WORD_LEN		2
#define STS4X_RESPONSE_LENGTH	3

/* STS4X Cyclic Redundancy Check Properties. */
#define STS4X_CRC8_POLYNOMIAL	0x31
#define STS4X_CRC8_INIT		0xff

DECLARE_CRC8_TABLE(sts4x_crc8_table);

static const struct i2c_device_id sts4x_id[] = {
	{ "sts4a", 0 },
	{ "sts4b", 0 },
	{ "sts4c", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sts4x_id);

static const struct of_device_id sts4x_of_match[] = {
	{ .compatible = "sensirion,sts4a" },
	{ .compatible = "sensirion,sts4b" },
	{ .compatible = "sensirion,sts4c" },
	{ }
};
MODULE_DEVICE_TABLE(of, sts4x_of_match);

/**
 * struct sts4x_data - All the data required to operate an STS4X chip.
 * @client: the i2c client associated with the STS4X.
 * @tzd: thermal zone struct associated with the STS4x.
 * @lock: a mutex that is used to prevent parallel access to the i2c client.
 * @last_updated: the previous time that the STS4X was polled.
 * @temp: the latest temperature value received from the STS4X.
 */
struct sts4x_data {
	struct i2c_client		*client;
	struct thermal_zone_device	*tzd;
	struct mutex			lock;	/* atomic read data updates */
	bool				valid;	/* validity of field below */
	long				last_updated;	/* in jiffies */
	s32				temp;
};

/**
 * sts4x_get_temp() - read & parse the raw temp from the STS4X in milli degrees.
 * @tzd:  &thermal zone device backing this sts4x sensor.
 * @temp: Temperature of the thermal zone, in milli degrees celsius.
 * Return: 0 if successful or a negative error code on failure.
 */
static int sts4x_get_temp(struct thermal_zone_device *tzd, int *temp)
{
	int ret = 0;
	u16 t_ticks;
	unsigned long next_update;
	struct sts4x_data *data;
	u8 crc;
	u8 cmd[STS4X_CMD_LEN] = {STS4X_CMD_MEASURE_TEMP};
	u8 raw_data[STS4X_RESPONSE_LENGTH];

	if (tzd == NULL || tzd->devdata == NULL)
		return -EINVAL;

	data = tzd->devdata;

	mutex_lock(&data->lock);
	next_update = data->last_updated +
		      msecs_to_jiffies(STS4X_MIN_POLL_INTERVAL);

	if (data->valid && time_before_eq(jiffies, next_update))
		goto update_temp;

	ret = i2c_master_send(data->client, cmd, STS4X_CMD_LEN);
	if (ret < 0)
		goto unlock;

	usleep_range(STS4X_MEAS_DELAY_HPM, STS4X_MEAS_DELAY_HPM + STS4X_DELAY_EXTRA);

	ret = i2c_master_recv(data->client, raw_data, STS4X_RESPONSE_LENGTH);
	if (ret != STS4X_RESPONSE_LENGTH) {
		if (ret >= 0)
			ret = -ENODATA;
		goto unlock;
	}

	t_ticks = raw_data[0] << 8 | raw_data[1];
	crc = crc8(sts4x_crc8_table, &raw_data[0], STS4X_WORD_LEN, STS4X_CRC8_INIT);
	if (crc != raw_data[2]) {
		dev_err(&data->client->dev, "data integrity check failed\n");
		ret = -EIO;
		goto unlock;
	}

	data->temp = ((21875 * (int32_t)t_ticks) >> 13) - 45000;
	data->last_updated = jiffies;
	data->valid = true;
	ret = 0;

update_temp:
	*temp = clamp_val(data->temp, STS4X_MIN_TEMPERATURE, STS4X_MAX_TEMPERATURE);
unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static struct thermal_zone_device_ops sts4x_tzd_ops = {
	.get_temp = sts4x_get_temp,
};

static int sts4x_probe(struct i2c_client *client,
		       const struct i2c_device_id *sts4x_id)
{
	struct sts4x_data *data;
	u8 cmd[] = {STS4X_CMD_RESET};
	int err, ret;

	/*
	 * we require full i2c support since the sts4x uses multi-byte read and
	 * writes as well as multi-byte commands which are not supported by
	 * the smbus protocol.
	 */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Failed to start STS4X due to incompatible i2c support");
		return -EOPNOTSUPP;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;

	// Register thermal zone.
	data->tzd = thermal_zone_device_register("ambient",
				0, 0, data, &sts4x_tzd_ops, NULL, 0, 0);
	if (IS_ERR(data->tzd)) {
		err = PTR_ERR(data->tzd);
		dev_err(&client->dev, "Failed to register ambient thermal zone: %d", err);
		return err;
	}

	ret = thermal_zone_device_enable(data->tzd);
	if (ret) {
		dev_err(&client->dev, "Failed to enable ambient thermal zone ret=%d", ret);
		thermal_zone_device_unregister(data->tzd);
		return ret;
	}

	mutex_init(&data->lock);

	crc8_populate_msb(sts4x_crc8_table, STS4X_CRC8_POLYNOMIAL);

	ret = i2c_master_send(client, cmd, STS4X_CMD_LEN);
	if (ret < 0)
		return ret;
	if (ret != STS4X_CMD_LEN)
		return -EIO;
	return 0;
}

static void sts4x_remove(struct i2c_client *client)
{
	struct sts4x_data *data = i2c_get_clientdata(client);

	thermal_zone_device_unregister(data->tzd);
}

static struct i2c_driver sts4x_driver = {
	.driver = {
		.name = "sts4x",
		.of_match_table = sts4x_of_match,
	},
	.id_table	= sts4x_id,
	.probe		= sts4x_probe,
	.remove		= sts4x_remove,
};

module_i2c_driver(sts4x_driver);

MODULE_DESCRIPTION("Sensirion STS4X temperature sensor driver");
MODULE_AUTHOR("S Ashwin Balaji <sashwinbalaji@google.com>");
MODULE_LICENSE("GPL");
