// SPDX-License-Identifier: GPL-2.0
/*
 * s2mpg15_spmic_thermal.c S2MPG15 Sub-PMIC thermistor driver
 *
 * Copyright (c) 2022, Google LLC. All rights reserved.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/kthread.h>
#include <linux/mfd/samsung/s2mpg15.h>
#include <linux/mfd/samsung/s2mpg15-register.h>
#include <soc/google/thermal_metrics.h>
#include "thermal_core.h"

#define GTHERM_CHAN_NUM 8
#define SENSOR_MAX_WAIT_TIME_MS 64000
#define SENSOR_WAIT_SLEEP_MS 50
#define NTC_UPDATE_MIN_DELAY_US 100
#define NTC_UPDATE_MAX_DELAY_US 10000
#define SPMIC_TEMPERATURE_BUF_LEN 5
#define SPMIC_ERR_READING_IGNORE_TIME_MSEC 30000
#define HW_LPF_RESET_VALUE 0x80

struct spmic_temperature_log {
	ktime_t time;
	int temperature;
	int raw;
	u8 raw_reg[S2MPG15_METER_NTC_BUF];
	int read_ct;
};

static const int stats_thresholds[MAX_SUPPORTED_THRESHOLDS] = {
			0, 39000, 43000, 45000, 46500, 52000, 55000, 70000};
struct s2mpg15_spmic_thermal_sensor {
	struct s2mpg15_spmic_thermal_chip *chip;
	struct thermal_zone_device *tzd;
	unsigned int adc_chan;
	bool thr_triggered; // TODO: remove?
	int emul_temperature;
	int ot_irq;
	int ut_irq;
	int log_ct;
	struct spmic_temperature_log temp_log[SPMIC_TEMPERATURE_BUF_LEN];
	tr_handle tr_handle;
};

struct s2mpg15_spmic_thermal_chip {
	struct device *dev;
	struct i2c_client *meter_i2c;
	struct i2c_client *mt_trim_i2c;
	struct mutex adc_chan_lock;
	struct s2mpg15_dev *iodev;
	struct s2mpg15_spmic_thermal_sensor sensor[GTHERM_CHAN_NUM];
	u8 adc_chan_en;
	u8 enable_hw_lpf[GTHERM_CHAN_NUM];
	u8 stats_en;
	bool filter_spurious_samples;
	struct kobject *kobjs[GTHERM_CHAN_NUM];
	struct kthread_worker *wq;
	struct kthread_delayed_work wait_sensor_work;
	bool sensors_ready;
};
static struct s2mpg15_spmic_thermal_chip *spmic_thermal_chip;

/* ODPM driver is the only known client for now
*  might want to scale this to handle more than 1 client
*  in the future.
*/

/**
 * struct s2mpg15_spmic_thermal_map_pt - Map data representation for ADC
 * @volt: Represent the ADC voltage data.
 * @temp: Represent the temperature for given volt.
 */
struct adc_map_pt {
	int volt;
	int temp;
};

/*
 * voltage to temperature table organized descending in voltage,
 * ascending in temperature.
 * TODO: confirm the mapping
 */
static const struct adc_map_pt s2mpg15_adc_map[] = {
	{ 0xF8D, -26428 }, { 0xF6A, -21922 }, { 0xF29, -15958 },
	{ 0xEE4, -11060 }, { 0xE9D, -6890 },  { 0xE3F, -2264 },
	{ 0xDBF, 2961 },   { 0xD33, 7818 },   { 0xC97, 12525 },
	{ 0xBF5, 16945 },  { 0xB3A, 21623 },  { 0xA42, 27431 },
	{ 0x7F1, 40631 },  { 0x734, 44960 },  { 0x66B, 49757 },
	{ 0x5A3, 54854 },  { 0x4EE, 59898 },  { 0x446, 65076 },
	{ 0x43A, 65779 },  { 0x430, 65856 },  { 0x3C3, 69654 },
	{ 0x3BD, 69873 },  { 0x33B, 74910 },  { 0x2BB, 80691 },
	{ 0x259, 85844 },  { 0x206, 90915 },  { 0x1CE, 94873 },
	{ 0x191, 99720 },  { 0x160, 104216 }, { 0x12E, 109531 },
	{ 0xF9, 116445 },  { 0xD7, 121600 },  { 0x9F, 131839 },
};

/*
 * Convert the input voltage to a temperature using linear interpretation
 * from the lookup table.
 */
static int s2mpg15_map_volt_temp(int input)
{
	int low = 0;
	int high = ARRAY_SIZE(s2mpg15_adc_map) - 1;
	int mid = 0;

	if (s2mpg15_adc_map[low].volt <= input)
		return s2mpg15_adc_map[low].temp;
	else if (s2mpg15_adc_map[high].volt >= input)
		return s2mpg15_adc_map[high].temp;

	/* Binary search, value will be between index low and low - 1 */
	while (low <= high) {
		mid = (low + high) / 2;
		if (s2mpg15_adc_map[mid].volt < input)
			high = mid - 1;
		else if (s2mpg15_adc_map[mid].volt > input)
			low = mid + 1;
		else
			return s2mpg15_adc_map[mid].temp;
	}

	return s2mpg15_adc_map[low].temp +
	       mult_frac(s2mpg15_adc_map[low - 1].temp - s2mpg15_adc_map[low].temp,
			 input - s2mpg15_adc_map[low].volt,
			 s2mpg15_adc_map[low - 1].volt - s2mpg15_adc_map[low].volt);
}

/*
 * Convert the temperature to voltage to a temperature using linear interpretation
 * from the lookup table.
 */
static int s2mpg15_map_temp_volt(int input)
{
	int low = 0;
	int high = ARRAY_SIZE(s2mpg15_adc_map) - 1;
	int mid = 0;

	if (s2mpg15_adc_map[low].temp >= input)
		return s2mpg15_adc_map[low].volt;
	else if (s2mpg15_adc_map[high].temp <= input)
		return s2mpg15_adc_map[high].volt;

	/* Binary search, value will between index low and low - 1 */
	while (low <= high) {
		mid = (low + high) / 2;
		if (s2mpg15_adc_map[mid].temp < input)
			low = mid + 1;
		else if (s2mpg15_adc_map[mid].temp > input)
			high = mid - 1;
		else
			return s2mpg15_adc_map[mid].volt;
	}
	return s2mpg15_adc_map[low].volt +
	       mult_frac(s2mpg15_adc_map[low - 1].volt - s2mpg15_adc_map[low].volt,
			 input - s2mpg15_adc_map[low].temp,
			 s2mpg15_adc_map[low - 1].temp - s2mpg15_adc_map[low].temp);
}

static int s2mpg15_spmic_thermal_read_raw(struct s2mpg15_spmic_thermal_sensor *s, int *raw)
{
	struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal = s->chip;
	u8 data_buf[S2MPG15_METER_NTC_BUF];
	u8 reg = S2MPG15_METER_LPF_DATA_NTC0_1 + S2MPG15_METER_NTC_BUF * s->adc_chan;

	int ret = s2mpg15_bulk_read(s2mpg15_spmic_thermal->meter_i2c, reg, S2MPG15_METER_NTC_BUF,
								data_buf);
	if (ret)
		return ret;

	*raw = data_buf[0] + ((data_buf[1] & 0xf) << 8);

	// All 0 usually means not ready
	if (*raw == 0)
		return -EBUSY;

	return ret;
}
/*
 * s2mpg15_spmic_disable_hw_lpf() - disable hw low-pass filter.
 * @s2mpg15_spmic_thermal: sub pmic chip
 *
 * disable hardware low pass filter.
 *
 * Return: register write status.
*/

static int s2mpg15_spmic_disable_hw_lpf(
			struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal)
{
	int ret_code = 0, i;
	struct device *dev = s2mpg15_spmic_thermal->dev;

	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		u8 reg = S2MPG15_METER_NTC_LPF_C0_0 + i;

		dev_dbg(dev, "Disabling sensor %d  LPF\n", i);
		ret_code = s2mpg15_write_reg(s2mpg15_spmic_thermal->meter_i2c,
			reg, HW_LPF_RESET_VALUE);
		if (ret_code) {
			dev_warn(dev, "Disable LPF for chan %d failed with %x\n", i, ret_code);
			break;
		}
	}
	return ret_code;
}

/*
 * Configure NTC channels in thermistor engine.
 */
static int s2mpg15_spmic_set_ntc_channels(
	struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal, u8 adc_ch_en)
{
	int ret = 0;
	struct device *dev = s2mpg15_spmic_thermal->dev;
	struct i2c_client *meter_i2c = s2mpg15_spmic_thermal->meter_i2c;

	// disable lpf before meter sw reset
	s2mpg15_spmic_disable_hw_lpf(s2mpg15_spmic_thermal);

	usleep_range(NTC_UPDATE_MIN_DELAY_US, NTC_UPDATE_MAX_DELAY_US);

	dev_info(dev, "Applying NTC... disabling odpm [s2mpg15]\n");
	/* workaround suggested in b/238678825 for NTC channel update */
	ret = s2mpg15_write_reg(meter_i2c, S2MPG15_METER_CTRL3, 0x00);
	if (ret)
		goto err;

	usleep_range(NTC_UPDATE_MIN_DELAY_US, NTC_UPDATE_MAX_DELAY_US);
	ret = s2mpg15_write_reg(meter_i2c, S2MPG15_METER_CTRL5, 0x4F);
	if (ret)
		goto err;

	usleep_range(NTC_UPDATE_MIN_DELAY_US, NTC_UPDATE_MAX_DELAY_US);
	ret = s2mpg15_write_reg(meter_i2c, S2MPG15_METER_CTRL3, adc_ch_en);
	if (ret)
		goto err;

	msleep(SENSOR_WAIT_SLEEP_MS);

	dev_info(dev, "Set NTC channels (adc_ch_en : 0x%x)\n", adc_ch_en);

	/* b/228112807, we need to re-enable the meter for the odpm. */
	ret = s2mpg15_update_reg(meter_i2c, S2MPG15_METER_CTRL1,
				 METER_EN_MASK, METER_EN_MASK);
	if (ret)
		dev_info(dev, "Failed to re-enable odpm [s2mpg15] :(\n");
	else
		dev_info(dev, "Re-enabled odpm [s2mpg15] :)\n");

	return ret;

err:
	s2mpg15_write_reg(meter_i2c, S2MPG15_METER_CTRL3, 0x00);
	dev_err(dev, "Failed to set NTC channels (adc_ch_en : 0x%x)\n", adc_ch_en);
	return ret;
}

/*
 * Wait for sensors to be ready. Enable HW LPF after the sensors are ready.
 * Continue with enabling HW LPF and updating thermal zone if the sensors
 * are not ready by the end of max wait time.
 */
static int s2mpg15_spmic_wait_for_sensors_ready(
			struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal, u8 adc_chan_en)
{
	struct device *dev = s2mpg15_spmic_thermal->dev;
	int raw, ret_code = 0, i, j = SENSOR_MAX_WAIT_TIME_MS / SENSOR_WAIT_SLEEP_MS;
	const unsigned long enabled_channels = adc_chan_en;

	for_each_set_bit(i, &enabled_channels, GTHERM_CHAN_NUM) {
		int ret = 0;
		u8 reg = S2MPG15_METER_NTC_LPF_C0_0 + i;

		/* Wait for longest refresh period */
		do {
			ret = s2mpg15_spmic_thermal_read_raw(&s2mpg15_spmic_thermal->sensor[i],
											&raw);
			dev_dbg(dev, "Sensor %d raw:0x%x ret:%d\n", i, raw, ret);
			if (ret != -EBUSY)
				break;
			dev_info(dev, "Sensor %d not ready, retry...\n", i);
			msleep(SENSOR_WAIT_SLEEP_MS);
		} while (j-- >= 0);

		if (ret == -EBUSY) {
			dev_err(dev, "Sensor %d timeout, give up...\n", i);
			ret_code = ret;
		}
		/* Enable HW LPF */
		mutex_lock(&s2mpg15_spmic_thermal->adc_chan_lock);
		dev_dbg(dev, "updating sensor %d  LPF CO to 0x%x\n",
			i, s2mpg15_spmic_thermal->enable_hw_lpf[i]);
		ret_code = s2mpg15_write_reg(s2mpg15_spmic_thermal->meter_i2c, reg,
			s2mpg15_spmic_thermal->enable_hw_lpf[i]);
		if (ret_code)
			dev_warn(dev, "Sensor %d LPF write fail with ret:%d\n", i, ret_code);
		mutex_unlock(&s2mpg15_spmic_thermal->adc_chan_lock);
		thermal_zone_device_update(s2mpg15_spmic_thermal->sensor[i].tzd,
					   THERMAL_EVENT_UNSPECIFIED);
	}
	/* set up spmic driver info for API access*/
	spmic_thermal_chip = s2mpg15_spmic_thermal;
	s2mpg15_spmic_thermal->sensors_ready = true;
	return ret_code;
}

/*
 * s2mpg15_spmic_set_hw_lpf() - enable/disable hw low-pass filter.
 * @enable: 1: Enable, 0: Disable
 *
 * API to enable or disable hardware low pass filter.
 *
 * Return : register write status if driver ready, -ENODEV if not ready.
*/
int s2mpg15_spmic_set_hw_lpf(bool enable) {
	int ret = 0;
	/*
	* We are not using a mutex/counter to guard the critical section
	* and we can evaluate it later if needed.
	*/
	if (!spmic_thermal_chip) {
		pr_err("%s:spmic thermal driver not ready \n", __func__);
		return -ENODEV;
	}
	if (enable) {
		ret = s2mpg15_spmic_wait_for_sensors_ready(
			spmic_thermal_chip,spmic_thermal_chip->adc_chan_en);
		if (ret)
			spmic_thermal_chip->adc_chan_en = 0x00;
	} else {
		ret = s2mpg15_spmic_disable_hw_lpf(spmic_thermal_chip);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg15_spmic_set_hw_lpf);

/*
 * s2mpg15_spmic_thermal_ready()
 *
 * API for client driver to query spmic driver if it is ready.
 *
 * Return: true if spmic driver is ready, false if not.
*/
bool s2mpg15_spmic_thermal_ready(void) {
	if (spmic_thermal_chip)
		return true;
	return false;
}
EXPORT_SYMBOL_GPL(s2mpg15_spmic_thermal_ready);

/*
 * s2mpg15_spmic_temp_update: Update the temperature history.
 */
static void s2mpg15_spmic_temp_update(
		struct s2mpg15_spmic_thermal_sensor *s,
		int temp, int raw, u8 *buf)
{
	int ct = s->log_ct - 1;

	if (ct < 0)
		ct += SPMIC_TEMPERATURE_BUF_LEN;
	if (s->temp_log[ct].temperature == temp) {
		s->temp_log[ct].read_ct++;
		return;
	}
	ct = s->log_ct;
	s->temp_log[ct].time = ktime_get_real();
	s->temp_log[ct].raw = raw;
	s->temp_log[ct].raw_reg[0] = buf[0];
	s->temp_log[ct].raw_reg[1] = buf[1];
	s->temp_log[ct].temperature = temp;
	s->temp_log[ct].read_ct = 1;
	s->log_ct = (s->log_ct + 1) % SPMIC_TEMPERATURE_BUF_LEN;
}

/**
 * dump_thermal_history() - Dump the last TEMPERATURE_BUF_LEN readings.
 * @s: s2mpg15_spmic_thermal_sensor with abnormality
 *
 * Read the latest sensor reading and print it.
 * Print the last 5 distinct temp readings
 */
static void dump_thermal_history(const struct s2mpg15_spmic_thermal_sensor *s)
{
	struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal = s->chip;
	struct device *dev = s2mpg15_spmic_thermal->dev;
	int ret = 0, ct = 0, i = 0;
	u8 data_buf[S2MPG15_METER_NTC_BUF * GTHERM_CHAN_NUM];

	dev_info(dev, "Dump thermal reading of %s", s->tzd->type);
	ret = s2mpg15_bulk_read(s2mpg15_spmic_thermal->meter_i2c,
					S2MPG15_METER_LPF_DATA_NTC0_1,
					S2MPG15_METER_NTC_BUF * GTHERM_CHAN_NUM,
					data_buf);
	if (!ret) {
		for (ct = 0; ct < S2MPG15_METER_NTC_BUF * GTHERM_CHAN_NUM; ct = ct + 2)
			dev_info(dev, "0x%x 0x%x\n", data_buf[ct], data_buf[ct + 1]);
	}

	for (i = 0; i < SPMIC_TEMPERATURE_BUF_LEN; i++) {
		ct = (s->log_ct + i) % SPMIC_TEMPERATURE_BUF_LEN;
		dev_info(dev, "%lld: Raw:%d, temp:%d, Reg:%#x %#x count:%d",
				s->temp_log[ct].time, s->temp_log[ct].raw,
				s->temp_log[ct].temperature, s->temp_log[ct].raw_reg[0],
				s->temp_log[ct].raw_reg[1], s->temp_log[ct].read_ct
		);
	}
}

/**
 * get_filtered_temp() - Get the updated temperature after filtering abnormality.
 * @s: s2mpg15_spmic_thermal_sensor whose temp to be filtered.
 *
 * If current abnormal temp is observed only once or if observed continuously for less
 * than SPMIC_ERR_READING_IGNORE_TIME_MSEC, return prev valid temperature.
 * Else, return the current abnormal temp.
 *
 * Return: Filtered temperature
 */
static int get_filtered_temp(const struct s2mpg15_spmic_thermal_sensor *s)
{
	struct device *dev = s->chip->dev;
	int valid_temp_idx = 0, last_temp_idx = 0;

	 /* Get the previous valid reading. */
	valid_temp_idx = s->log_ct - 2;
	if (valid_temp_idx < 0)
		valid_temp_idx += SPMIC_TEMPERATURE_BUF_LEN;

	/* Get the error reading index to get the first occurrence. */
	last_temp_idx = s->log_ct - 1;
	if (last_temp_idx < 0)
		last_temp_idx += SPMIC_TEMPERATURE_BUF_LEN;

	if (s->temp_log[last_temp_idx].read_ct == 1 ||
		ktime_ms_delta(ktime_get_real(), s->temp_log[last_temp_idx].time) <
			SPMIC_ERR_READING_IGNORE_TIME_MSEC) {
		dev_dbg(dev, "Filtering spurious reading for sensor %s.\n", s->tzd->type);
		return s->temp_log[valid_temp_idx].temperature;
	}
	dev_err(dev, "%s spurious reading persisted %d msec. Reading actual value.",
				s->tzd->type, SPMIC_ERR_READING_IGNORE_TIME_MSEC);
	return s->temp_log[last_temp_idx].temperature;
}
/*
 * Get temperature for given tz.
 */
static int s2mpg15_spmic_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct s2mpg15_spmic_thermal_sensor *s = tz->devdata;
	struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal = s->chip;
	int raw, ret = 0, metrics_ret = 0;
	u8 mask = 0x1;
	u8 data_buf[S2MPG15_METER_NTC_BUF];
	u8 reg = S2MPG15_METER_LPF_DATA_NTC0_1 +
		 S2MPG15_METER_NTC_BUF * s->adc_chan;

	if (!s2mpg15_spmic_thermal->sensors_ready)
		return -EAGAIN;

	mutex_lock(&s2mpg15_spmic_thermal->adc_chan_lock);
	if (s->emul_temperature)
		goto emul_temp_exit;

	if (!(s2mpg15_spmic_thermal->adc_chan_en & (mask << s->adc_chan))) {
		ret = -EIO;
		goto err_exit;
	}

	ret = s2mpg15_bulk_read(s2mpg15_spmic_thermal->meter_i2c, reg,
				S2MPG15_METER_NTC_BUF, data_buf);
	if (ret)
		goto err_exit;

	raw = data_buf[0] + ((data_buf[1] & 0xf) << 8);
	*temp = s2mpg15_map_volt_temp(raw);

	s2mpg15_spmic_temp_update(s, *temp, raw, data_buf);

#if IS_ENABLED(CONFIG_PIXEL_METRICS)
	/* Enabled for all tz. */
	if (s2mpg15_spmic_thermal->stats_en & (mask << s->adc_chan)) {
		metrics_ret = temp_residency_stats_update(s->tr_handle, *temp);
		/* Filter the spurious reading. */
		if (s2mpg15_spmic_thermal->filter_spurious_samples && unlikely(metrics_ret == EXTREME_HIGH_TEMP)) {
			dump_thermal_history(s);
			*temp = get_filtered_temp(s);
		}
	}
#endif

	mutex_unlock(&s2mpg15_spmic_thermal->adc_chan_lock);
	return ret;

emul_temp_exit:
	*temp = s->emul_temperature;

err_exit:
	mutex_unlock(&s2mpg15_spmic_thermal->adc_chan_lock);
	return ret;
}

/*
 * Set monitor window for given tz.
 */
static int s2mpg15_spmic_thermal_set_trips(struct thermal_zone_device *tz, int low_temp,
					 int high_temp)
{
	struct s2mpg15_spmic_thermal_sensor *s = tz->devdata;
	struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal = s->chip;
	struct device *dev = s2mpg15_spmic_thermal->dev;
	int emul_temp, low_volt, high_volt, ret = 0;
	u8 low_raw, high_raw;

	if (!s2mpg15_spmic_thermal->sensors_ready)
		return -EAGAIN;

	/* Set threshold to extreme value when emul_temp set */
	emul_temp = s->emul_temperature;
	if (emul_temp) {
		high_temp = INT_MAX;
		low_temp = INT_MIN;
	}

	low_volt = s2mpg15_map_temp_volt(low_temp);
	low_raw = low_volt >> 4 & 0xFF;
	ret = s2mpg15_write_reg(s2mpg15_spmic_thermal->meter_i2c,
				S2MPG15_METER_NTC_UT_WARN0 + s->adc_chan, low_raw);
	if (ret)
		return ret;

	high_volt = s2mpg15_map_temp_volt(high_temp);
	high_raw = high_volt >> 4 & 0xFF;
	ret = s2mpg15_write_reg(s2mpg15_spmic_thermal->meter_i2c,
				S2MPG15_METER_NTC_OT_WARN0 + s->adc_chan, high_raw);
	if (ret)
		return ret;

	dev_dbg_ratelimited(dev,
		"low_temp(mdegC):%d, high_temp(mdegC):%d low_adc:%d high_adc:%d\n",
		low_temp, high_temp, low_raw, high_raw);

	return 0;
}

static int
s2mpg15_spmic_thermal_set_hot_trip(struct s2mpg15_spmic_thermal_sensor *s, int temp)
{
	int ret = 0;
	struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal = s->chip;
	u8 raw = s2mpg15_map_temp_volt(temp) >> 4 & 0xFF;
	struct device *dev = s2mpg15_spmic_thermal->dev;

	if (temp == THERMAL_TEMP_INVALID)
		return -EINVAL;

	ret = s2mpg15_write_reg(s2mpg15_spmic_thermal->meter_i2c,
						S2MPG15_METER_NTC_OT_FAULT0 + s->adc_chan, raw);
	dev_info(dev, "Set sensor %d hot trip(mdegC):%d, ret:%d\n", s->adc_chan, temp, ret);

	return ret;
}

/*
 * Set temperature threshold for given tz, only critical threshold will be
 * programmed as shutdown threshold.
 */
static int s2mpg15_spmic_thermal_set_trip_temp(struct thermal_zone_device *tz, int trip, int temp)
{
	struct s2mpg15_spmic_thermal_sensor *s = tz->devdata;
	const struct thermal_trip *trip_points;
	int ret = 0;

	if (!s->chip->sensors_ready)
		return -EAGAIN;

	trip_points = of_thermal_get_trip_points(s->tzd);
	if (!trip_points)
		return -EINVAL;

	if (trip_points[trip].type != THERMAL_TRIP_HOT)
		return ret;

	/* Use THERMAL_TRIP_HOT for HW thermal shutdown */
	ret = s2mpg15_spmic_thermal_set_hot_trip(s, temp);

	return ret;
}

/*
 * Set emulation temperture for given tz.
 */
static int s2mpg15_spmic_thermal_set_emul_temp(struct thermal_zone_device *tz, int temp)
{
	struct s2mpg15_spmic_thermal_sensor *sensor = tz->devdata;
	int ret = 0;
	u8 value = 0, mask = 0x1;

	if (!sensor->chip->sensors_ready)
		return -EAGAIN;

	mutex_lock(&sensor->chip->adc_chan_lock);
	if (sensor->chip->adc_chan_en & (mask << sensor->adc_chan)) {
		ret = s2mpg15_read_reg(sensor->chip->meter_i2c, S2MPG15_METER_CTRL3, &value);
		if (ret)
			goto err;

		if (temp)
			value &= ~(mask << sensor->adc_chan);
		else
			value |= mask << sensor->adc_chan;

		ret = s2mpg15_spmic_set_ntc_channels(sensor->chip, value);
		if (ret)
			goto err;
	}
	sensor->emul_temperature = temp;

	mutex_unlock(&sensor->chip->adc_chan_lock);
	return s2mpg15_spmic_wait_for_sensors_ready(sensor->chip, value);

err:
	mutex_unlock(&sensor->chip->adc_chan_lock);
	return ret;
}

static void
s2mpg15_spmic_thermal_init(struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal)
{
	int i;

	mutex_init(&s2mpg15_spmic_thermal->adc_chan_lock);

	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		s2mpg15_spmic_thermal->sensor[i].chip = s2mpg15_spmic_thermal;
		s2mpg15_spmic_thermal->sensor[i].adc_chan = i;
		s2mpg15_spmic_thermal->sensor[i].log_ct = 0;
	}
}

static struct thermal_zone_device_ops s2mpg15_spmic_thermal_ops = {
	.get_temp = s2mpg15_spmic_thermal_get_temp,
	.set_trips = s2mpg15_spmic_thermal_set_trips,
	.set_trip_temp = s2mpg15_spmic_thermal_set_trip_temp,
	.set_emul_temp = s2mpg15_spmic_thermal_set_emul_temp,
};

static ssize_t
tz_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_zone_device *tzd = to_thermal_zone(dev);

	thermal_zone_device_update(tzd, THERMAL_EVENT_UNSPECIFIED);

	return sysfs_emit(buf, "%d\n", tzd->temperature);
}

static DEVICE_ATTR_RO(tz_temp);

static int s2mpg15_spmic_thermal_get_hot_temp(struct thermal_zone_device *tzd)
{
	int ntrips;
	const struct thermal_trip *trips;
	int i;

	ntrips = of_thermal_get_ntrips(tzd);
	if (ntrips <= 0)
		return THERMAL_TEMP_INVALID;

	trips = of_thermal_get_trip_points(tzd);
	if (!trips)
		return THERMAL_TEMP_INVALID;

	for (i = 0; i < ntrips; i++) {
		if (of_thermal_is_trip_valid(tzd, i) && trips[i].type == THERMAL_TRIP_HOT)
			return trips[i].temperature;
	}

	return THERMAL_TEMP_INVALID;
}

/*
 * IRQ handler.
 */
static irqreturn_t s2mpg15_spmic_thermal_irq(int irq, void *data)
{
	struct s2mpg15_spmic_thermal_chip *chip = data;
	struct device *dev = chip->dev;
	int i;

	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		if ((chip->sensor[i].ot_irq == irq) || (chip->sensor[i].ut_irq == irq)) {
			dev_info_ratelimited(dev, "PMIC_THERM[%d] IRQ, %d ot_irq:%d\n", i,
					     irq, (chip->sensor[i].ot_irq == irq));
			thermal_zone_device_update(chip->sensor[i].tzd,
						   THERMAL_EVENT_UNSPECIFIED);
			return IRQ_HANDLED;
		}
	}
	WARN(1, "Bad IRQ in thermal %d", irq);
	return IRQ_NONE;
}

static void s2mpg15_spmic_thermal_wait_sensor_work(struct kthread_work *work)
{
	struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal =
		container_of(work, struct s2mpg15_spmic_thermal_chip, wait_sensor_work.work);
	struct device *dev = s2mpg15_spmic_thermal->dev;
	u8 adc_chan_en = s2mpg15_spmic_thermal->adc_chan_en;
	int ret;

	mutex_lock(&s2mpg15_spmic_thermal->adc_chan_lock);
	ret = s2mpg15_spmic_set_ntc_channels(s2mpg15_spmic_thermal, adc_chan_en);
	if (ret)
		goto err;
	mutex_unlock(&s2mpg15_spmic_thermal->adc_chan_lock);

	s2mpg15_spmic_wait_for_sensors_ready(s2mpg15_spmic_thermal, adc_chan_en);
	return;

err:
	dev_err(dev, "Failed to set NTC channels during initialization\n");
	s2mpg15_spmic_thermal->adc_chan_en = 0x00;
	mutex_unlock(&s2mpg15_spmic_thermal->adc_chan_lock);
}

/*
 * Unregister thermal zones.
 */
static void
s2mpg15_spmic_thermal_unregister_tzd(struct s2mpg15_spmic_thermal_chip *chip)
{
	int i;
	struct device *dev = chip->dev;

	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		dev_info(dev, "Unregistering %d sensor\n", i);
		devm_thermal_of_zone_unregister(chip->dev,
						       chip->sensor[i].tzd);
	}
}

/*
 * Register thermal zones.
 */
static int
s2mpg15_spmic_thermal_register_tzd(struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal)
{
	unsigned int i;
	struct thermal_zone_device *tzd;
	struct device *dev = s2mpg15_spmic_thermal->dev;
	u8 mask = 0x1;
	int temp, ret = 0;

	for (i = 0; i < GTHERM_CHAN_NUM; i++, mask <<= 1) {
		dev_info(dev, "Registering %d sensor\n", i);
		tzd = devm_thermal_of_zone_register(s2mpg15_spmic_thermal->dev, i,
							   &s2mpg15_spmic_thermal->sensor[i],
							   &s2mpg15_spmic_thermal_ops);

		if (IS_ERR(tzd)) {
			dev_err(dev,
				"Error registering thermal zone:%ld for channel:%d\n",
				PTR_ERR(tzd), i);
			return -EINVAL;
		}
		s2mpg15_spmic_thermal->sensor[i].tzd = tzd;
		if (s2mpg15_spmic_thermal->adc_chan_en & mask)
			thermal_zone_device_enable(tzd);
		else
			thermal_zone_device_disable(tzd);

		ret = device_create_file(&tzd->device, &dev_attr_tz_temp);
		if (ret) {
			dev_err(dev,
				"Error creating tz_temp node for thermal zone:%ld for channel:%d\n",
				PTR_ERR(tzd), i);
			return ret;
		}

		temp = s2mpg15_spmic_thermal_get_hot_temp(tzd);
		s2mpg15_spmic_thermal_set_hot_trip(&s2mpg15_spmic_thermal->sensor[i], temp);
	}

	return ret;
}

static const struct of_device_id s2mpg15_spmic_thermal_match_table[] = {
	{ .compatible = "google,s2mpg15-spmic-thermal" },
	{}
};
MODULE_DEVICE_TABLE(of, s2mpg15_spmic_thermal_match_table);

static int s2mpg15_spmic_thermal_get_dt_data(struct platform_device *pdev,
					   struct s2mpg15_spmic_thermal_chip *s2mpg15_spmic_thermal)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const struct of_device_id *id;

	if (!node)
		return -EINVAL;

	id = of_match_node(s2mpg15_spmic_thermal_match_table, node);
	if (!id)
		return -EINVAL;

	if (of_property_read_u8(node, "adc_chan_en",
				&s2mpg15_spmic_thermal->adc_chan_en)) {
		dev_err(dev, "Cannot read adc_chan_en\n");
		return -EINVAL;
	}

	if (of_property_read_u8(node, "stats_en",
				&s2mpg15_spmic_thermal->stats_en)) {
		dev_info(dev, "Cannot read stats_en\n");
		s2mpg15_spmic_thermal->stats_en = 0;
	}

    s2mpg15_spmic_thermal->filter_spurious_samples =
        of_property_read_bool(node, "filter_spurious_samples");
	/* property to enable HW LPF*/
	if (of_property_read_u8_array(node, "enable_hw_lpf",
				&s2mpg15_spmic_thermal->enable_hw_lpf[0],
				GTHERM_CHAN_NUM) < 0) {
		dev_dbg(dev, "lpf_weight property not defined\n");
		memset(&s2mpg15_spmic_thermal->enable_hw_lpf[0], HW_LPF_RESET_VALUE,
			sizeof(s2mpg15_spmic_thermal->enable_hw_lpf));
	}
    return 0;
}

static ssize_t
adc_chan_en_show(struct device *dev, struct device_attribute *devattr,
		 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s2mpg15_spmic_thermal_chip *chip = platform_get_drvdata(pdev);
	int ret;
	u8 value;

	if (!chip->sensors_ready)
		return -EAGAIN;

	mutex_lock(&chip->adc_chan_lock);
	ret = s2mpg15_read_reg(chip->meter_i2c, S2MPG15_METER_CTRL3, &value);
	mutex_unlock(&chip->adc_chan_lock);

	return ret ? ret : sysfs_emit(buf, "0x%02X\n", value);
}

static ssize_t
adc_chan_en_store(struct device *dev, struct device_attribute *devattr,
		  const char *buf, size_t count)
{
	int i, ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct s2mpg15_spmic_thermal_chip *chip = platform_get_drvdata(pdev);
	u8 value, mask = 0x1;

	if (!chip->sensors_ready)
		return -EAGAIN;

	ret = sscanf(buf, "%hhx", &value);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&chip->adc_chan_lock);
	ret = s2mpg15_spmic_set_ntc_channels(chip, value);
	if (ret)
		goto err;

	chip->adc_chan_en = value;
	mutex_unlock(&chip->adc_chan_lock);

	ret = s2mpg15_spmic_wait_for_sensors_ready(chip, chip->adc_chan_en);
	if (ret) {
		chip->adc_chan_en = 0x00;
		return ret;
	}

	for (i = 0; i < GTHERM_CHAN_NUM; i++, mask <<= 1) {
		if (chip->adc_chan_en & mask)
			thermal_zone_device_enable(chip->sensor[i].tzd);
		else
			thermal_zone_device_disable(chip->sensor[i].tzd);
	}

	return count;

err:
	chip->adc_chan_en = 0x00;
	mutex_unlock(&chip->adc_chan_lock);
	return ret;
}

static ssize_t enable_hw_lpf_show(struct device *dev,
				    struct device_attribute *devattr,
				    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s2mpg15_spmic_thermal_chip *chip = platform_get_drvdata(pdev);
	int i, len = 0;

	for (i = 0; i < GTHERM_CHAN_NUM; i++)
		len += sysfs_emit_at(buf, len, "%d ", chip->enable_hw_lpf[i]);
	len += sysfs_emit_at(buf, len, "\n");

	return len;
}

static ssize_t enable_hw_lpf_store(struct device *dev,
				    struct device_attribute *devattr,
				    const char *buf,
				    size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct device *device = &pdev->dev;
	struct s2mpg15_spmic_thermal_chip *chip = platform_get_drvdata(pdev);
	char **argv;
	int argc, i, ret = 0;
	u8 val;

	argv = argv_split(GFP_KERNEL, buf, &argc);
	if (!argv) {
		ret = -ENOMEM;
		goto out;
	}

	if (argc != GTHERM_CHAN_NUM) {
		ret = -EINVAL;
		dev_err(device, "invalid args count, ret=%d,"
			"argc=%d, GTHERM_CHAN_NUM=%d",
			ret, argc, GTHERM_CHAN_NUM);
		goto free_arg_out;
	}
	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		ret = kstrtou8(argv[i], 0 , &val);
		if (ret) {
			dev_err(device,"parse error ,ret=%d",ret);
			goto free_arg_out;
		}
		dev_dbg(device, "Update hw lpf weight[%d] "
			"from %d to %d\n", i, chip->enable_hw_lpf[i], val);
		chip->enable_hw_lpf[i] = val;
	}
	ret = count;
free_arg_out:
	argv_free(argv);
out:
	return ret;
}

static DEVICE_ATTR_RW(adc_chan_en);
static DEVICE_ATTR_RW(enable_hw_lpf);

static struct attribute *s2mpg15_spmic_dev_attrs[] = {
	&dev_attr_adc_chan_en.attr,
	&dev_attr_enable_hw_lpf.attr,
	NULL
};

ATTRIBUTE_GROUPS(s2mpg15_spmic_dev);

static int s2mpg15_spmic_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg15_spmic_thermal_chip *chip;
	int ret = 0;
	struct s2mpg15_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg15_platform_data *pdata;
	int irq_base, i = 0;
	int irq_count = 0;
	u8 mask = 0x01;
	__maybe_unused char thermal_group[] = "spmic";

	chip = devm_kzalloc(&pdev->dev, sizeof(struct s2mpg15_spmic_thermal_chip),
			    GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (!iodev) {
		dev_err(dev, "Failed to get parent s2mpg15_dev\n");
		return -EINVAL;
	}
	pdata = iodev->pdata;
	if (!pdata) {
		dev_err(dev, "Failed to get s2mpg15_platform_data\n");
		return -EINVAL;
	}

	chip->dev = dev;
	chip->meter_i2c = iodev->meter;
	chip->mt_trim_i2c = iodev->mt_trim;
	chip->iodev = iodev;
	chip->sensors_ready = false;
	ret = s2mpg15_spmic_thermal_get_dt_data(pdev, chip);
	if (ret) {
		dev_err(dev, "s2mpg15_spmic_thermal get dt data failed\n");
		return ret;
	}

	irq_base = pdata->irq_base;
	if (irq_base <= 0) {
		dev_err(dev, "Failed to get irq base %d\n", irq_base);
		ret = -ENODEV;
		goto fail;
	}

	s2mpg15_spmic_thermal_init(chip);

	/* Set sampling rate */
	s2mpg15_update_reg(chip->meter_i2c, S2MPG15_METER_CTRL1,
			   NTC_0P15625HZ << NTC_SAMP_RATE_SHIFT,
			   NTC_SAMP_RATE_MASK);

	ret = s2mpg15_spmic_thermal_register_tzd(chip);
	if (ret) {
		dev_err(dev, "Failed to register with of thermal\n");
		goto disable_ntc;
	}

	/* Setup IRQ and register for residency stats */
	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		int ret;
#if IS_ENABLED(CONFIG_PIXEL_METRICS)
		struct thermal_zone_device *tzd = chip->sensor[i].tzd;
		tr_handle tr_stats_handle;
		int num_stats_thresholds =  ARRAY_SIZE(stats_thresholds);
#endif
		chip->sensor[i].ot_irq =
			irq_base + S2MPG15_IRQ_NTC_WARN_OT_CH1_INT7 + i;

		chip->sensor[i].ut_irq =
			irq_base + S2MPG15_IRQ_NTC_WARN_UT_CH1_INT8 + i;

		ret = devm_request_threaded_irq(&pdev->dev, chip->sensor[i].ot_irq,
						NULL, s2mpg15_spmic_thermal_irq,
						0, "PMIC_THERM_IRQ", chip);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to request NTC[%d] IRQ: %d: %d\n", i,
				chip->sensor[i].ot_irq, ret);
			goto free_irq_tz;
		}
		irq_count++;

		ret = devm_request_threaded_irq(&pdev->dev, chip->sensor[i].ut_irq,
						NULL, s2mpg15_spmic_thermal_irq,
						0, "PMIC_THERM_IRQ", chip);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to request NTC[%d] IRQ: %d: %d\n", i,
				chip->sensor[i].ut_irq, ret);
			goto free_irq_tz;
		}
		irq_count++;

		if (!(chip->stats_en & (mask << i)))
			continue;

#if IS_ENABLED(CONFIG_PIXEL_METRICS)
		tr_stats_handle = register_temp_residency_stats(tzd->type, thermal_group);
		if (tr_stats_handle < 0) {
			dev_err(&pdev->dev,
				"Failed to register for temperature residency stats. ret: %d\n",
				tr_stats_handle);
			goto free_irq_tz;
		}

		ret = temp_residency_stats_set_thresholds(tr_stats_handle,
					stats_thresholds, num_stats_thresholds);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to set stats_thresholds. ret = %d\n", ret);
			goto free_irq_tz;
		}

		chip->sensor[i].tr_handle = tr_stats_handle;
#endif
	}
	chip->wq = kthread_create_worker(0, "spmic-init");
	if (IS_ERR_OR_NULL(chip->wq)) {
		ret = PTR_ERR(chip->wq);
		goto free_irq_tz;
	}

	kthread_init_delayed_work(&chip->wait_sensor_work, s2mpg15_spmic_thermal_wait_sensor_work);

	platform_set_drvdata(pdev, chip);

	dev_info(dev, "probe done, now wait for sensors\n");
	kthread_mod_delayed_work(chip->wq, &chip->wait_sensor_work, 0);

	return ret;

free_irq_tz:
	if (irq_count & 0x01)
		devm_free_irq(&pdev->dev, chip->sensor[i].ot_irq, chip);

	while (--i >= 0) {
		devm_free_irq(&pdev->dev, chip->sensor[i].ot_irq, chip);
		devm_free_irq(&pdev->dev, chip->sensor[i].ut_irq, chip);
	}
disable_ntc:
	s2mpg15_spmic_set_ntc_channels(chip, 0x00);
fail:
	return ret;
}

static int s2mpg15_spmic_thermal_remove(struct platform_device *pdev)
{
	int i;
	__maybe_unused u8 mask = 0x01;
	struct s2mpg15_spmic_thermal_chip *chip = platform_get_drvdata(pdev);

	mutex_lock(&chip->adc_chan_lock);
	s2mpg15_spmic_set_ntc_channels(chip, 0x00);
	chip->adc_chan_en = 0x00;
	mutex_unlock(&chip->adc_chan_lock);

	kthread_cancel_delayed_work_sync(&chip->wait_sensor_work);
	kthread_destroy_worker(chip->wq);
	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		devm_free_irq(&pdev->dev, chip->sensor[i].ot_irq, chip);
		devm_free_irq(&pdev->dev, chip->sensor[i].ut_irq, chip);

#if IS_ENABLED(CONFIG_PIXEL_METRICS)
		if (chip->stats_en & (mask << i))
			unregister_temp_residency_stats(chip->sensor[i].tr_handle);
#endif
	}
	s2mpg15_spmic_thermal_unregister_tzd(chip);
	spmic_thermal_chip = NULL;
	return 0;
}

static struct platform_driver s2mpg15_spmic_thermal_driver = {
	.driver = {
		.name = "s2mpg15-spmic-thermal",
		.dev_groups = s2mpg15_spmic_dev_groups,
		.owner = THIS_MODULE,
	},
	.probe = s2mpg15_spmic_thermal_probe,
	.remove = s2mpg15_spmic_thermal_remove,
};
module_platform_driver(s2mpg15_spmic_thermal_driver);

MODULE_DESCRIPTION("Google LLC S2MPG15 SPMIC Thermal Driver");
MODULE_AUTHOR("Sayanna Chandula <sayanna@google.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:google,s2mpg15_thermal");
