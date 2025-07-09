// SPDX-License-Identifier: GPL-2.0
/*
 * gs101_spmic_thermal.c gsoc101 SPMIC thermistor driver
 *
 * Copyright (c) 2020, Google LLC. All rights reserved.
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
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg11-register.h>

#include <thermal_core.h>

#if IS_ENABLED(CONFIG_PIXEL_METRICS)
#include <soc/google/thermal_metrics.h>
static const int stats_thresholds[MAX_SUPPORTED_THRESHOLDS] = {
			0, 39000, 43000, 45000, 46500, 52000, 55000, 70000};
#endif

#define GTHERM_CHAN_NUM 8
#define SENSOR_WAIT_SLEEP_MS 50

struct gs101_spmic_thermal_sensor {
	struct gs101_spmic_thermal_chip *chip;
	struct thermal_zone_device *tzd;
	unsigned int adc_chan;
	int emul_temperature;
	int irq;
#if IS_ENABLED(CONFIG_PIXEL_METRICS)
	tr_handle tr_handle;
#endif
};

struct gs101_spmic_thermal_chip {
	struct device *dev;
	struct i2c_client *i2c;
	struct s2mpg11_dev *iodev;
	struct gs101_spmic_thermal_sensor sensor[GTHERM_CHAN_NUM];
	u8 adc_chan_en;
	u8 stats_en;
	struct kobject *kobjs[GTHERM_CHAN_NUM];
	struct kthread_worker *wq;
	struct kthread_delayed_work wait_sensor_work;
	bool sensors_ready;
};

/**
 * struct gs101_spmic_thermal_map_pt - Map data representation for ADC
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
 */
static const struct adc_map_pt gs101_adc_map[] = {
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
static int gs101_map_volt_temp(int input)
{
	int low = 0;
	int high = ARRAY_SIZE(gs101_adc_map) - 1;
	int mid = 0;

	if (gs101_adc_map[low].volt <= input)
		return gs101_adc_map[low].temp;
	else if (gs101_adc_map[high].volt >= input)
		return gs101_adc_map[high].temp;

	/* Binary search, value will be between index low and low - 1 */
	while (low <= high) {
		mid = (low + high) / 2;
		if (gs101_adc_map[mid].volt < input)
			high = mid - 1;
		else if (gs101_adc_map[mid].volt > input)
			low = mid + 1;
		else
			return gs101_adc_map[mid].temp;
	}

	return gs101_adc_map[low].temp +
	       mult_frac(gs101_adc_map[low - 1].temp - gs101_adc_map[low].temp,
			 input - gs101_adc_map[low].volt,
			 gs101_adc_map[low - 1].volt - gs101_adc_map[low].volt);
}

/*
 * Convert the temperature to voltage to a temperature using linear interpretation
 * from the lookup table.
 */
static int gs101_map_temp_volt(int input)
{
	int low = 0;
	int high = ARRAY_SIZE(gs101_adc_map) - 1;
	int mid = 0;

	if (gs101_adc_map[low].temp >= input)
		return gs101_adc_map[low].volt;
	else if (gs101_adc_map[high].temp <= input)
		return gs101_adc_map[high].volt;

	/* Binary search, value will between index low and low - 1 */
	while (low <= high) {
		mid = (low + high) / 2;
		if (gs101_adc_map[mid].temp < input)
			low = mid + 1;
		else if (gs101_adc_map[mid].temp > input)
			high = mid - 1;
		else
			return gs101_adc_map[mid].volt;
	}
	return gs101_adc_map[low].volt +
	       mult_frac(gs101_adc_map[low - 1].volt - gs101_adc_map[low].volt,
			 input - gs101_adc_map[low].temp,
			 gs101_adc_map[low - 1].temp - gs101_adc_map[low].temp);
}

static int gs101_spmic_thermal_read_raw(struct gs101_spmic_thermal_sensor *s, int *raw)
{
	struct gs101_spmic_thermal_chip *gs101_spmic_thermal = s->chip;
	u8 data_buf[S2MPG11_METER_NTC_BUF];
	u8 reg = S2MPG11_METER_LPF_DATA_NTC0_1 + S2MPG11_METER_NTC_BUF * s->adc_chan;

	int ret = s2mpg11_bulk_read(gs101_spmic_thermal->i2c, reg, S2MPG11_METER_NTC_BUF, data_buf);
	if (ret)
		return ret;

	*raw = data_buf[0] + ((data_buf[1] & 0xf) << 8);

	// All 0 usually means not ready
	if (*raw == 0)
		return -EBUSY;

	return ret;
}

/*
 * Get temperature for given tz.
 */
static int gs101_spmic_thermal_get_temp(struct thermal_zone_device *tz,
					int *temp)
{
	struct gs101_spmic_thermal_sensor *s = tz->devdata;
	struct gs101_spmic_thermal_chip *gs101_spmic_thermal = s->chip;
	int emul_temp;
	int raw, ret = 0;
	u8 mask = 0x1;

	if (!gs101_spmic_thermal->sensors_ready)
		return -EAGAIN;

	emul_temp = s->emul_temperature;
	if (emul_temp) {
		*temp = emul_temp;
		goto end;
	}

	if (!(gs101_spmic_thermal->adc_chan_en & (mask << s->adc_chan)))
		return -EIO;

	ret = gs101_spmic_thermal_read_raw(s, &raw);
	if (ret)
		return ret;

	*temp = gs101_map_volt_temp(raw);

end:
#if IS_ENABLED(CONFIG_PIXEL_METRICS)
	if (gs101_spmic_thermal->stats_en & (mask << s->adc_chan))
		temp_residency_stats_update(s->tr_handle, *temp);
#endif

	return ret;
}

/*
 * Set monitor window for given tz.
 */
static int gs101_spmic_thermal_set_trips(struct thermal_zone_device *tz,
					 int low_temp, int high_temp)
{
	struct gs101_spmic_thermal_sensor *s = tz->devdata;
	struct gs101_spmic_thermal_chip *gs101_spmic_thermal = s->chip;
	struct device *dev = gs101_spmic_thermal->dev;
	int emul_temp, low_volt, ret = 0;
	u8 raw;

	if (!gs101_spmic_thermal->sensors_ready)
		return -EAGAIN;

	/* Set threshold to extreme value when emul_temp set */
	emul_temp = s->emul_temperature;
	if (emul_temp) {
		high_temp = INT_MAX;
		low_temp = INT_MIN;
	}

	/*
	 * Ignore low_temp, and assuming trips are
	 * configured with passive for polling
	 */
	low_volt = gs101_map_temp_volt(high_temp);

	raw = low_volt >> 4 & 0xFF;
	ret = s2mpg11_write_reg(gs101_spmic_thermal->i2c,
				S2MPG11_METER_NTC_L_WARN0 + s->adc_chan, raw);

	dev_dbg_ratelimited(dev,
			    "low_temp(mdegC):%d, high_temp(mdegC):%d adc:%d ret:%d\n",
			    low_temp, high_temp, raw, ret);

	return ret;
}

static int gs101_spmic_thermal_set_hot_trip(struct gs101_spmic_thermal_sensor *s, int temp)
{
	int ret = 0;
	struct gs101_spmic_thermal_chip *gs101_spmic_thermal = s->chip;
	u8 raw = gs101_map_temp_volt(temp) >> 4 & 0xFF;
	struct device *dev = gs101_spmic_thermal->dev;

	if (!gs101_spmic_thermal->sensors_ready)
		return -EAGAIN;

	if (temp == THERMAL_TEMP_INVALID)
		return -EINVAL;
	ret = s2mpg11_write_reg(gs101_spmic_thermal->i2c, S2MPG11_METER_NTC_H_WARN0 + s->adc_chan,
				raw);
	dev_info(dev, "Set sensor %d hot trip(mdegC):%d, ret:%d\n", s->adc_chan, temp, ret);

	return ret;
}

/*
 * Set temperature threshold for given tz, only critical threshold will be
 * programmed as shutdown threshold.
 */
static int gs101_spmic_thermal_set_trip_temp(struct thermal_zone_device *tz,
					     int trip, int temp)
{
	struct gs101_spmic_thermal_sensor *s = tz->devdata;
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
	ret = gs101_spmic_thermal_set_hot_trip(s, temp);

	return ret;
}

/*
 * Set emulation temperture for given tz.
 */
static int gs101_spmic_thermal_set_emul_temp(struct thermal_zone_device *tz,
					     int temp)
{
	struct gs101_spmic_thermal_sensor *sensor = tz->devdata;
	int ret;
	u8 value, mask = 0x1;

	if (!sensor->chip->sensors_ready)
		return -EAGAIN;

	if (sensor->chip->adc_chan_en & (mask << sensor->adc_chan)) {
		ret = s2mpg11_read_reg(sensor->chip->i2c, S2MPG11_METER_CTRL3, &value);
		if (ret)
			return ret;

		if (temp)
			value &= ~(mask << sensor->adc_chan);
		else
			value |= mask << sensor->adc_chan;

		ret = s2mpg11_write_reg(sensor->chip->i2c, S2MPG11_METER_CTRL3, value);
		if (ret)
			return ret;
	}
	sensor->emul_temperature = temp;
	return 0;
}

static void
gs101_spmic_thermal_init(struct gs101_spmic_thermal_chip *gs101_spmic_thermal)
{
	int i;

	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		gs101_spmic_thermal->sensor[i].chip = gs101_spmic_thermal;
		gs101_spmic_thermal->sensor[i].adc_chan = i;
	}
}

static struct thermal_zone_device_ops gs101_spmic_thermal_ops = {
	.get_temp = gs101_spmic_thermal_get_temp,
	.set_trips = gs101_spmic_thermal_set_trips,
	.set_trip_temp = gs101_spmic_thermal_set_trip_temp,
	.set_emul_temp = gs101_spmic_thermal_set_emul_temp,
};

static ssize_t channel_temp_show(struct kobject *kobj,
			 struct kobj_attribute *attr, char *buf)
{
	struct thermal_zone_device *tzd = to_thermal_zone(kobj_to_dev(
			kobj->parent));

	thermal_zone_device_update(tzd, THERMAL_EVENT_UNSPECIFIED);

	return sysfs_emit(buf, "%d\n", tzd->temperature);
}

static struct kobj_attribute channel_temp_attr = __ATTR_RO(channel_temp);

static int gs101_spmic_thermal_get_hot_temp(struct thermal_zone_device *tzd)
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
 * Wait for sensors to be ready.
 */
static void gs101_spmic_thermal_wait_sensor(struct kthread_work *work)
{
	struct gs101_spmic_thermal_chip *gs101_spmic_thermal =
		container_of(work, struct gs101_spmic_thermal_chip, wait_sensor_work.work);
	struct device *dev = gs101_spmic_thermal->dev;
	u8 mask = 0x1;
	int raw, ret, i, j = 64000 / SENSOR_WAIT_SLEEP_MS;

	for (i = 0; i < GTHERM_CHAN_NUM; i++, mask <<= 1) {
		if (!(gs101_spmic_thermal->adc_chan_en & mask))
			continue;
		/* Wait for longest refresh period */
		while (j--) {
			ret = gs101_spmic_thermal_read_raw(&gs101_spmic_thermal->sensor[i], &raw);
			dev_info(dev, "Sensor %d raw:0x%x\n", i, raw);
			if (ret != -EBUSY)
				break;
			dev_info(dev, "Sensor %d not ready, retry...\n", i);
			msleep(SENSOR_WAIT_SLEEP_MS);
		}

		if (j < 0)
			dev_warn(dev, "Sensor %d timeout, give up...\n", i);

		thermal_zone_device_enable(gs101_spmic_thermal->sensor[i].tzd);
	}
	gs101_spmic_thermal->sensors_ready = true;
}

/*
 * Register thermal zones.
 */
static int gs101_spmic_thermal_register_tzd(struct gs101_spmic_thermal_chip *gs101_spmic_thermal)
{
	unsigned int i;
	struct thermal_zone_device *tzd;
	struct device *dev = gs101_spmic_thermal->dev;
	int temp;
	int ret = 0;

	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		dev_info(dev, "Registering channel %u\n", i);
		tzd = devm_thermal_of_zone_register(gs101_spmic_thermal->dev,
						    i,
						    &gs101_spmic_thermal->sensor[i],
						    &gs101_spmic_thermal_ops);

		if (IS_ERR(tzd)) {
			dev_err(dev,
				"Error registering thermal zone:%ld for channel:%u\n",
				PTR_ERR(tzd), i);
			continue;
		}
		gs101_spmic_thermal->sensor[i].tzd = tzd;
		/*
		 * Disable thermal zone until we have other resources ready like
		 * interrupt, stats, etc.
		 */
		thermal_zone_device_disable(tzd);
		gs101_spmic_thermal->kobjs[i] =
			kobject_create_and_add("adc_channel", &tzd->device.kobj);
		ret = sysfs_create_file(gs101_spmic_thermal->kobjs[i], &channel_temp_attr.attr);
		if (ret < 0)
			dev_err(dev,
				"Error creating sysfs file for thermal zone:%ld for channel:%u\n",
				PTR_ERR(tzd), i);
		temp = gs101_spmic_thermal_get_hot_temp(tzd);
		gs101_spmic_thermal_set_hot_trip(&gs101_spmic_thermal->sensor[i], temp);
	}
	return 0;
}

/*
 * IRQ handler.
 */
static irqreturn_t gs101_spmic_thermal_irq(int irq, void *data)
{
	struct gs101_spmic_thermal_chip *chip = data;
	struct device *dev = chip->dev;
	int i;

	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		if (chip->sensor[i].irq == irq) {
			dev_info_ratelimited(dev, "PMIC_THERM[%d] IRQ, %d\n", i,
					     irq);
			thermal_zone_device_update(chip->sensor[i].tzd,
						   THERMAL_EVENT_UNSPECIFIED);
			return IRQ_HANDLED;
		}
	}
	WARN(1, "Bad IRQ in thermal %d", irq);
	return IRQ_NONE;
}

/*
 * Unregister thermal zones.
 */
static void
gs101_spmic_thermal_unregister_tzd(struct gs101_spmic_thermal_chip *chip)
{
	int i;
	struct device *dev = chip->dev;

	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		dev_info(dev, "Unregistering %d sensor\n", i);
		sysfs_remove_file(chip->kobjs[i], &channel_temp_attr.attr);
		kobject_put(chip->kobjs[i]);
	}
}

static const struct of_device_id gs101_spmic_thermal_match_table[] = {
	{ .compatible = "google,gs101-spmic-thermal" },
	{}
};
MODULE_DEVICE_TABLE(of, gs101_spmic_thermal_match_table);

static int gs101_spmic_thermal_get_dt_data(struct platform_device *pdev,
					   struct gs101_spmic_thermal_chip *gs101_spmic_thermal)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const struct of_device_id *id;

	if (!node)
		return -EINVAL;

	id = of_match_node(gs101_spmic_thermal_match_table, node);
	if (!id)
		return -EINVAL;

	if (of_property_read_u8(node, "adc_chan_en",
				&gs101_spmic_thermal->adc_chan_en)) {
		dev_info(dev, "Cannot read adc_chan_en\n");
		return -EINVAL;
	}

	if (of_property_read_u8(node, "stats_en",
				&gs101_spmic_thermal->stats_en)) {
		dev_info(dev, "Cannot read stats_en\n");
		gs101_spmic_thermal->stats_en = 0;
	}

	return 0;
}

/*
 * Enable NTC thermistor engine.
 */
static int
gs101_spmic_set_enable(struct gs101_spmic_thermal_chip *gs101_spmic_thermal,
		       bool on)
{
	int ret = 0;
	struct device *dev = gs101_spmic_thermal->dev;

	/* WAR EVT0, ADC_EN should be set as '1' */
	/* if any channel of NTC is set to be '1' */
	if (on && gs101_spmic_thermal->iodev->pmic_rev == S2MPG11_EVT0) {
		ret = s2mpg11_update_reg(gs101_spmic_thermal->i2c,
					 S2MPG11_METER_CTRL1, METER_EN_MASK,
					 METER_EN_MASK);
		if (ret) {
			dev_err(dev, "Cannot update meter_on for EVT0\n");
			return ret;
		}
	}

	if (on) {
		/* WAR EVT0 disable IC power shutdown reaching NTC_H_WARN threshold */
		if (gs101_spmic_thermal->iodev->pmic_rev == S2MPG11_EVT0) {
			ret = s2mpg11_write_reg(gs101_spmic_thermal->iodev->trim,
						0xEA, 0x02);
			if (ret) {
				dev_err(dev,
					"Cannot disable IC power down for EVT0\n");
				return ret;
			}
		}
		ret = s2mpg11_write_reg(gs101_spmic_thermal->i2c,
					S2MPG11_METER_CTRL3,
					gs101_spmic_thermal->adc_chan_en);
		if (ret) {
			dev_err(dev, "Cannot enable NTC engine\n");
		} else {
			dev_info(dev, "Enabled NTC channels: 0x%x\n",
				 gs101_spmic_thermal->adc_chan_en);
		}
	} else {
		ret = s2mpg11_write_reg(gs101_spmic_thermal->i2c,
					S2MPG11_METER_CTRL3, 0x00);
		if (ret) {
			dev_err(dev, "Cannot disable NTC\n");
		} else {
			dev_info(dev, "Disabled NTC channels\n");
		}
	}

	return ret;
}

static ssize_t
adc_chan_en_show(struct device *dev, struct device_attribute *devattr,
		 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gs101_spmic_thermal_chip *chip = platform_get_drvdata(pdev);
	int ret;
	u8 value;

	if (!chip->sensors_ready)
		return -EAGAIN;

	ret = s2mpg11_read_reg(chip->i2c, S2MPG11_METER_CTRL3, &value);

	return ret ? ret : scnprintf(buf, PAGE_SIZE, "0x%02X\n", value);
}

static ssize_t
adc_chan_en_store(struct device *dev, struct device_attribute *devattr,
		  const char *buf, size_t count)
{
	int i, ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct gs101_spmic_thermal_chip *chip = platform_get_drvdata(pdev);
	u8 value, mask = 0x1;

	if (!chip->sensors_ready)
		return -EAGAIN;

	ret = sscanf(buf, "%hhx", &value);
	if (ret != 1)
		return ret;

	ret = s2mpg11_write_reg(chip->i2c, S2MPG11_METER_CTRL3, value);
	if (ret)
		return ret;

	chip->adc_chan_en = value;

	for (i = 0; i < GTHERM_CHAN_NUM; i++, mask <<= 1) {
		if (chip->adc_chan_en & mask)
			thermal_zone_device_enable(chip->sensor[i].tzd);
		else
			thermal_zone_device_disable(chip->sensor[i].tzd);
	}

	return count;
}

static DEVICE_ATTR_RW(adc_chan_en);

static struct attribute *gs101_spmic_dev_attrs[] = {
	&dev_attr_adc_chan_en.attr,
	NULL
};

ATTRIBUTE_GROUPS(gs101_spmic_dev);

static int gs101_spmic_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs101_spmic_thermal_chip *chip;
	int ret = 0;
	struct s2mpg11_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg11_platform_data *pdata;
	int irq_base, i;
	u8 mask = 0x01;
	char __maybe_unused thermal_group[] = "spmic";
	chip = devm_kzalloc(&pdev->dev, sizeof(struct gs101_spmic_thermal_chip),
			    GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (!iodev) {
		dev_err(dev, "Failed to get parent s2mpg11_dev\n");
		ret = -EINVAL;
		goto fail;
	}
	pdata = iodev->pdata;
	if (!pdata) {
		dev_err(dev, "Failed to get s2mpg11_platform_data\n");
		ret = -EINVAL;
		goto fail;
	}

	chip->dev = dev;
	chip->i2c = iodev->meter;
	chip->iodev = iodev;

	ret = gs101_spmic_thermal_get_dt_data(pdev, chip);
	if (ret) {
		dev_err(dev, "gs101_spmic_thermal get dt data failed\n");
		goto fail;
	}

	irq_base = pdata->irq_base;
	if (!irq_base) {
		dev_err(dev, "Failed to get irq base %d\n", irq_base);
		ret = -ENODEV;
		goto fail;
	}

	gs101_spmic_thermal_init(chip);

	/* w/a for reversed NTC_LPF_DATA */
	/* 0x00 is the highest threshold for NTC values, not 0xff  */
	if (iodev->pmic_rev >= S2MPG11_EVT1) {
		for (i = S2MPG11_METER_NTC_L_WARN0;
		     i <= S2MPG11_METER_NTC_H_WARN7; i++)
			s2mpg11_write_reg(chip->i2c, i, 0x00);
	}
	/* Set sampling rate */
	s2mpg11_update_reg(chip->i2c, S2MPG11_METER_CTRL1,
			   NTC_0P15625HZ << NTC_SAMP_RATE_SHIFT,
			   NTC_SAMP_RATE_MASK);

	ret = gs101_spmic_set_enable(chip, true);
	if (ret) {
		dev_err(dev, "Failed to enable NTC engine\n");
		goto fail;
	}

	ret = gs101_spmic_thermal_register_tzd(chip);
	if (ret) {
		dev_err(dev, "Failed to register with of thermal\n");
		goto disable_ntc;
	}

	/* Setup IRQ and stats collection*/
	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		int ret;
#if IS_ENABLED(CONFIG_PIXEL_METRICS)
		struct thermal_zone_device *tzd = chip->sensor[i].tzd;
		tr_handle tr_stats_handle;
		int num_stats_thresholds =  ARRAY_SIZE(stats_thresholds);
#endif

		chip->sensor[i].irq =
			irq_base + S2MPG11_IRQ_NTC_WARN_CH1_INT6 + i;

		ret = devm_request_threaded_irq(&pdev->dev, chip->sensor[i].irq,
						NULL, gs101_spmic_thermal_irq,
						0, "PMIC_THERM_IRQ", chip);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to request NTC[%d] IRQ: %d: %d\n", i,
				chip->sensor[i].irq, ret);
			goto free_irq_tz;
		}

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

	kthread_init_delayed_work(&chip->wait_sensor_work, gs101_spmic_thermal_wait_sensor);

	platform_set_drvdata(pdev, chip);

	dev_info(dev, "probe done, now wait for sensors\n");
	kthread_mod_delayed_work(chip->wq, &chip->wait_sensor_work, 0);

	return ret;

free_irq_tz:
	while (--i >= 0) {
		devm_free_irq(&pdev->dev, chip->sensor[i].irq, chip);
	}
	gs101_spmic_thermal_unregister_tzd(chip);
disable_ntc:
	gs101_spmic_set_enable(chip, false);
fail:
	devm_kfree(&pdev->dev, chip);
	return ret;
}

static int gs101_spmic_thermal_remove(struct platform_device *pdev)
{
	int i;
	u8 __maybe_unused mask = 0x01;
	struct gs101_spmic_thermal_chip *chip = platform_get_drvdata(pdev);

	kthread_cancel_delayed_work_sync(&chip->wait_sensor_work);
	kthread_destroy_worker(chip->wq);
	for (i = 0; i < GTHERM_CHAN_NUM; i++) {
		devm_free_irq(&pdev->dev, chip->sensor[i].irq, chip);

#if IS_ENABLED(CONFIG_PIXEL_METRICS)
		if (chip->stats_en & (mask << i))
			unregister_temp_residency_stats(chip->sensor[i].tr_handle);
#endif
	}
	gs101_spmic_thermal_unregister_tzd(chip);
	gs101_spmic_set_enable(chip, false);

	return 0;
}

static const struct platform_device_id gs101_spmic_thermal_id_table[] = {
	{
		.name = "gs101-spmic-thermal",
	},
	{},
};

static struct platform_driver gs101_spmic_thermal_driver = {
	.driver = {
		.name = "gs101-spmic-thermal",
		.dev_groups = gs101_spmic_dev_groups,
		.owner = THIS_MODULE,
	},
	.probe = gs101_spmic_thermal_probe,
	.remove = gs101_spmic_thermal_remove,
	.id_table = gs101_spmic_thermal_id_table,
};
module_platform_driver(gs101_spmic_thermal_driver);

MODULE_DESCRIPTION("Google LLC GS101 SPMIC Thermal Driver");
MODULE_AUTHOR("Wei Wang <wvw@google.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:google,gs101_thermal");
