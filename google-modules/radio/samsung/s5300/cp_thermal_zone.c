// SPDX-License-Identifier: GPL-2.0
/*
 * CP thermal zone driver.
 *
 * Copyright (c) 2020, Google LLC. All rights reserved.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/thermal.h>

#define MAX_CP_TEMP_SENSOR 9

struct cp_temp_sensor {
	struct thermal_zone_device *tzd;
	bool valid;
	int temp;
};

struct cp_temp_manager {
	u32 num_sensors;
	struct cp_temp_sensor sensor[MAX_CP_TEMP_SENSOR];
};

static ssize_t cp_temp_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t count)
{
	int ret = 0;
	struct cp_temp_manager *temp_manager = dev_get_drvdata(dev);
	u32 index;
	int temp, trigger;

	if (!temp_manager) {
		dev_err(dev, "Could not get CP temperature manager");
		return -EINVAL;
	}

	ret = sscanf(buf, "%d %d %d", &index, &trigger, &temp);
	if (ret != 3) {
		dev_err(dev, "Invalid CP temperature update");
		return -EINVAL;
	}

	if (index >= temp_manager->num_sensors) {
		dev_err(dev, "Invalid CP temperature sensor index - %d", index);
		return -EINVAL;
	}

	if (!temp_manager->sensor[index].valid && temp_manager->sensor[index].tzd)
		thermal_zone_device_enable(temp_manager->sensor[index].tzd);

	temp_manager->sensor[index].valid = true;
	temp_manager->sensor[index].temp = temp;

	if (trigger && temp_manager->sensor[index].tzd) {
		dev_info_ratelimited(dev, "Update CP temperature sensor %d", index);
		thermal_zone_device_update(temp_manager->sensor[index].tzd,
					   THERMAL_EVENT_UNSPECIFIED);
	}

	return count;
}

static DEVICE_ATTR_WO(cp_temp);

static struct attribute *cp_temp_attrs[] = {
	&dev_attr_cp_temp.attr,
	NULL,
};

static const struct attribute_group cp_temp_group = {
	.attrs = cp_temp_attrs,
};

static int cp_sensor_get_temp(struct thermal_zone_device *tz, int *temp)
{
	int ret = 0;
	struct cp_temp_sensor *s = tz->devdata;

	if (s && s->valid)
		*temp = s->temp;
	else
		ret = -EINVAL;

	return ret;
}

static struct thermal_zone_device_ops cp_thermal_zone_ops = { .get_temp = cp_sensor_get_temp };

static int cp_thermal_zone_probe(struct platform_device *pdev)
{
	int ret = 0;
	unsigned int i;
	struct cp_temp_manager *temp_manager;
	struct thermal_zone_device *tzd;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;

	temp_manager = devm_kzalloc(dev, sizeof(struct cp_temp_manager), GFP_KERNEL);

	if (of_property_read_u32(np, "num_sensors", &temp_manager->num_sensors)) {
		dev_err(dev, "Cannot read number of CP sensors");
		ret = -EINVAL;
		goto fail;
	}
	if (temp_manager->num_sensors > MAX_CP_TEMP_SENSOR) {
		dev_err(dev, "Invalid number of CP temp sensor - %d", temp_manager->num_sensors);
		ret = -EINVAL;
		goto fail;
	}

	if (sysfs_create_group(&dev->kobj, &cp_temp_group)) {
		dev_err(dev, "Error creating sysfs node for CP temperature");
		ret = -EINVAL;
		goto fail;
	}

	for (i = 0; i < temp_manager->num_sensors; ++i) {
		dev_info(dev, "Registering CP temp sensor %d", i);
		tzd = devm_thermal_of_zone_register(dev, i,
						    &temp_manager->sensor[i],
						    &cp_thermal_zone_ops);
		if (IS_ERR(tzd)) {
			dev_err(dev, "Error registering CP temperature sensor %d", i);
			continue;
		}
		temp_manager->sensor[i].tzd = tzd;
		thermal_zone_device_disable(tzd);
	}
	platform_set_drvdata(pdev, temp_manager);
fail:
	return ret;
}

static int cp_thermal_zone_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id cp_thermal_zone_match[] = {
	{
		.compatible = "google,gs101-cp-thermal",
	},
	{},
};
MODULE_DEVICE_TABLE(of, cp_thermal_zone_match);

static struct platform_driver cp_thermal_zone_driver = {
	.driver = {
		.name = "gs101-cp-thermal-zone",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cp_thermal_zone_match),
	},
	.probe = cp_thermal_zone_probe,
	.remove = cp_thermal_zone_remove,
};
module_platform_driver(cp_thermal_zone_driver);

MODULE_DESCRIPTION("Google LLC CP Thermal Zone Driver");
MODULE_AUTHOR("Eddie Tashjian <etashjian@google.com>");
MODULE_LICENSE("GPL");
