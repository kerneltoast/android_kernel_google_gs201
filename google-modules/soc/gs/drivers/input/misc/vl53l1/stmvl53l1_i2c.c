// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file stmvl53l1_i2c.c  vl53l1 linux native i2c interface
 *
 */
#include "stmvl53l1.h"
#include "stmvl53l1-i2c.h"
#include <linux/delay.h>
#include <linux/i2c.h>

#define WRITE_MULTIPLE_CHUNK_MAX	32

static inline void st_gettimeofday(struct timespec64 *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_nsec = now.tv_nsec;
}

/**
 * compute elapsed time in micro sec based on do_gettimeofday
 * @param tv pointer to start time_val
 * @return time elapsed in  micro seconde
 */
static uint32_t tv_elapsed_us(struct timespec64 *tv)
{
	struct timespec64 now;

	st_gettimeofday(&now);
	return (now.tv_sec - tv->tv_sec) * 1000000 +
	       (now.tv_nsec - tv->tv_nsec) / 1000;
}

#if STMVL53L1_LOG_CCI_TIMING
#define cci_access_var struct timespec64 cci_log_start_tv
#define cci_access_start() st_gettimeofday(&cci_log_start_tv)
#define cci_access_over(dev, fmt, ...) \
	dev_dbg(dev, "cci_timing %d us" fmt "\n", \
		tv_elapsed_us(&cci_log_start_tv), ##__VA_ARGS__)
#else
#define cci_access_var
#define cci_access_start(...) (void)0
#define cci_access_over(...) (void)0
#endif

VL53L1_Error VL53L1_GetTickCount(VL53L1_Dev_t *pdev, uint32_t *ptime_ms)
{
	(void)pdev;
	(void)ptime_ms;
	WARN_ON(1);
	return 0;
}

/**
 * compute elapsed time in milli sec based on do_gettimeofday
 * @param tv pointer to start time_val
 * @return time elapsed in milli seconde
 */
static uint32_t tv_elapsed_ms(struct timespec64 *tv)
{
	struct timespec64 now;

	st_gettimeofday(&now);
	return (now.tv_sec - tv->tv_sec) * 1000 +
	       (now.tv_nsec - tv->tv_nsec) / 1000000;
}

static int cci_write(struct stmvl53l1_data *dev, int index,
		uint8_t *data, uint16_t len)
{
	uint8_t buffer[STMVL53L1_MAX_CCI_XFER_SZ + 2];
	struct i2c_msg msg;
	struct i2c_data *i2c_client_obj = (struct i2c_data *)dev->client_object;
	struct i2c_client *client = (struct i2c_client *)i2c_client_obj->client;
	int rc;

	cci_access_var;
	if (len > STMVL53L1_MAX_CCI_XFER_SZ || len == 0) {
		dev_err(&client->dev, "%s: invalid len %d\n", __func__, len);
		return -1;
	}
	cci_access_start();
	/* build up little endian index in buffer */
	buffer[0] = (index >> 8) & 0xFF;
	buffer[1] = (index >> 0) & 0xFF;
	/* copy write data to buffer after index  */
	memcpy(buffer + 2, data, len);
	/* set i2c msg */
	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buffer;
	msg.len = len + 2;

	rc = i2c_transfer(client->adapter, &msg, 1);
	if (rc != 1) {
		dev_err(&client->dev,
			"wr i2c_transfer err:%d, index 0x%x len %d\n",
			rc, index, len);
	}
	cci_access_over(&client->dev, "rd status %d long %d ", rc != 1, len);
	return rc != 1;
}

static int cci_read(struct stmvl53l1_data *dev, int index,
		uint8_t *data, uint16_t len)
{
	uint8_t buffer[2];
	struct i2c_msg msg[2];
	struct i2c_data *i2c_client_obj = (struct i2c_data *)dev->client_object;
	struct i2c_client *client = (struct i2c_client *)i2c_client_obj->client;
	int rc;

	cci_access_var;
	if (len > STMVL53L1_MAX_CCI_XFER_SZ || len == 0) {
		dev_err(&client->dev, "%s: invalid len %d\n", __func__, len);
		return -1;
	}
	cci_access_start();

	/* build up little endian index in buffer */
	buffer[0] = (index >> 8) & 0xFF;
	buffer[1] = (index >> 0) & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;	/* Write */
	msg[0].buf = buffer;
	msg[0].len = 2;
	/* read part of the i2c transaction */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD | client->flags;
	msg[1].buf = data;
	msg[1].len = len;

	rc = i2c_transfer(client->adapter, msg, 2);
	if (rc != 2) {
		dev_err(&client->dev,
			"i2c_transfer :%d, @%x index 0x%x len %d\n",
			rc, client->addr, index, len);
	}
	cci_access_over(&client->dev, "wr len %d status %d", rc != 2, len);
	return rc != 2;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV pdev, uint16_t index, uint8_t data)
{
	struct stmvl53l1_data *dev;

	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);

	return cci_write(dev, index, &data, 1);

}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV pdev, uint16_t index, uint8_t *pdata)
{
	struct stmvl53l1_data *dev;

	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);

	return cci_read(dev, index, pdata, 1) ?
		VL53L1_ERROR_CONTROL_INTERFACE : VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV pdev, uint16_t index, uint16_t data)
{
	VL53L1_Error status;
	uint8_t buffer[2];
	struct stmvl53l1_data *dev =
		(struct stmvl53l1_data *)container_of(
			pdev, struct stmvl53l1_data, stdev);
	struct i2c_data *i2c_data = (struct i2c_data *)dev->client_object;

	/* Split 16-bit word into MS and L*  stmvl53l1 FlightSense sensor */

	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data & 0x00FF);
	dev_dbg(&i2c_data->client->dev, " @%x d= %x  => [ %x , %x ] \n",
		index, data, buffer[0], buffer[1]);
	status = VL53L1_WriteMulti(pdev, index, buffer, 2);

	return status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV pdev, uint16_t index, uint16_t *pdata)
{
	VL53L1_Error status;
	uint8_t buffer[2];

	status = VL53L1_ReadMulti(pdev, index, buffer, 2);

	*pdata = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];

	return status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV pdev, uint16_t index, uint32_t data)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t buffer[4];

	/* Split 32-bit word into MS ... LS bytes */
	buffer[0] = (uint8_t)(data >> 24);
	buffer[1] = (uint8_t)((data & 0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((data & 0x0000FF00) >> 8);
	buffer[3] = (uint8_t)(data & 0x000000FF);

	status = VL53L1_WriteMulti(pdev, index, buffer, 4);

	return status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV pdev, uint16_t index, uint32_t *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t buffer[4];

	status = VL53L1_ReadMulti(pdev, index, buffer, 4);

	*pdata = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) +
		 ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];

	return status;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV pdev, uint16_t index,
		uint8_t *pdata, uint32_t count)
{
	struct stmvl53l1_data *dev;
	uint32_t chunk_size = WRITE_MULTIPLE_CHUNK_MAX;
	VL53L1_Error status;
	uint32_t i;
	uint16_t hostaddr = index;

	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);

	for (i = 0; i < count; i += chunk_size) {
		status = (cci_write(dev, hostaddr, &pdata[i],
			min(chunk_size, (count - i))) ?
			VL53L1_ERROR_CONTROL_INTERFACE : VL53L1_ERROR_NONE);
		if (status != VL53L1_ERROR_NONE)
			break;
		hostaddr += chunk_size;
	}

	return status;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV pdev, uint16_t index,
		uint8_t *pdata, uint32_t count)
{
	struct stmvl53l1_data *dev;

	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);

	return cci_read(dev, index, pdata, count) ?
		VL53L1_ERROR_CONTROL_INTERFACE : VL53L1_ERROR_NONE;
}

static int is_time_over(struct timespec64 *tv, uint32_t msec)
{
	return tv_elapsed_ms(tv) >= msec;
}

VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV pdev,
		uint32_t timeout_ms,
		uint16_t index,
		uint8_t value,
		uint8_t mask, uint32_t poll_delay_ms)
{
	struct timespec64 start_tv;
	struct stmvl53l1_data *dev;
	int rc, time_over;
	uint8_t rd_val;
	struct i2c_data *data;

	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);
	data = (struct i2c_data *)dev->client_object;

	st_gettimeofday(&start_tv);
	do {
		rc = cci_read(dev, index, &rd_val, 1);
		if (rc)
			return VL53L1_ERROR_CONTROL_INTERFACE;
		if ((rd_val & mask) == value) {
			dev_dbg(&data->client->dev, "poll in %d us\n",
				tv_elapsed_us(&start_tv));
			return VL53L1_ERROR_NONE;
		}
		dev_dbg(&data->client->dev, "poll @%x %x & %d != %x\n",
			index, rd_val, mask, value);
		time_over = is_time_over(&start_tv, timeout_ms);
		if (!time_over)
			msleep(poll_delay_ms);
	} while (!time_over);
	dev_err(&data->client->dev, "time over %d ms\n", timeout_ms);
	return VL53L1_ERROR_TIME_OUT;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_DEV pdev, int32_t wait_us)
{
	struct stmvl53l1_data *data;

	data = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);

	if (!data->is_delay_allowed)
		return VL53L1_ERROR_PLATFORM_SPECIFIC_START;

	/* follow Documentation/timers/timers-howto.txt recommendations */
	if (wait_us < 10)
		udelay(wait_us);
	else if (wait_us < 20000)
		usleep_range(wait_us, wait_us + 1);
	else
		msleep(wait_us / 1000);

	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_DEV pdev, int32_t wait_ms)
{
	return VL53L1_WaitUs(pdev, wait_ms * 1000);
}
