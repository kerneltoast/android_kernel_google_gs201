// SPDX-License-Identifier: GPL-2.0
/*
 * Platform thermal driver for GXP.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/acpm_dvfs.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/version.h>

#include "gxp-internal.h"
#include "gxp-pm.h"
#include "gxp-thermal.h"
#include "gxp-lpm.h"

/*
 * Value comes from internal measurement
 * b/229623553
 */
static struct gxp_state_pwr state_pwr_map[] = {
	{1155000, 78},
	{975000, 58},
	{750000, 40},
	{560000, 27},
	{373000, 20},
	{268000, 16},
	{178000, 13},
};

static int gxp_get_max_state(struct thermal_cooling_device *cdev,
			     unsigned long *state)
{
	struct gxp_thermal_manager *thermal = cdev->devdata;

	if (!thermal->gxp_num_states)
		return -EIO;

	*state = thermal->gxp_num_states - 1;
	return 0;
}

/*
 * Set cooling state.
 */
static int gxp_set_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long cooling_state)
{
	int ret = 0;
	struct gxp_thermal_manager *thermal = cdev->devdata;
	struct device *dev = thermal->gxp->dev;
	unsigned long pwr_state;

	if (cooling_state >= thermal->gxp_num_states) {
		dev_err(dev, "%s: invalid cooling state %lu\n", __func__,
			cooling_state);
		return -EINVAL;
	}

	mutex_lock(&thermal->lock);
	cooling_state = max(thermal->sysfs_req, cooling_state);
	if (cooling_state >= ARRAY_SIZE(state_pwr_map)) {
		dev_err(dev, "Unsupported cooling state: %lu\n", cooling_state);
		ret = -EINVAL;
		goto out;
	}
	pwr_state = state_pwr_map[cooling_state].state;
	dev_dbg(dev, "setting policy %ld\n", pwr_state);
	if (cooling_state != thermal->cooling_state) {
#ifdef CONFIG_GXP_CLOUDRIPPER
		ret = exynos_acpm_set_policy(AUR_DVFS_DOMAIN,
			pwr_state < aur_power_state2rate[AUR_UUD] ?
			aur_power_state2rate[AUR_UUD] :
			pwr_state);
#endif
		if (ret) {
			dev_err(dev,
				"error setting gxp cooling policy: %d\n", ret);
			goto out;
		}
		thermal->cooling_state = cooling_state;
		gxp_pm_set_thermal_limit(thermal->gxp, pwr_state);
	} else {
		ret = -EALREADY;
	}

out:
	mutex_unlock(&thermal->lock);
	return ret;
}

static int gxp_get_cur_state(struct thermal_cooling_device *cdev,
			     unsigned long *state)
{
	int ret = 0;
	struct gxp_thermal_manager *thermal = cdev->devdata;

	mutex_lock(&thermal->lock);
	*state = thermal->cooling_state;
	if (*state >= thermal->gxp_num_states) {
		dev_err(thermal->gxp->dev,
			"Unknown cooling state: %lu, resetting\n", *state);
		ret = -EINVAL;
		goto out;
	}
out:
	mutex_unlock(&thermal->lock);
	return ret;
}

static int gxp_state2power_internal(unsigned long state, u32 *power,
				    struct gxp_thermal_manager *thermal)
{
	int i;

	for (i = 0; i < thermal->gxp_num_states; i++) {
		if (state == state_pwr_map[i].state) {
			*power = state_pwr_map[i].power;
			return 0;
		}
	}
	dev_err(thermal->gxp->dev, "Unknown state req for: %lu\n", state);
	*power = 0;
	return -EINVAL;
}

static int gxp_get_requested_power(struct thermal_cooling_device *cdev,
				   u32 *power)
{
	unsigned long power_state;
	struct gxp_thermal_manager *cooling = cdev->devdata;

	power_state = exynos_acpm_get_rate(AUR_DVFS_DOMAIN, 0);
	return gxp_state2power_internal(power_state, power, cooling);
}

/* TODO(b/213272324): Move state2power table to dts */
static int gxp_state2power(struct thermal_cooling_device *cdev,
			       unsigned long state, u32 *power)
{
	struct gxp_thermal_manager *thermal = cdev->devdata;

	if (state >= thermal->gxp_num_states) {
		dev_err(thermal->gxp->dev, "%s: invalid state: %lu\n", __func__,
			state);
		return -EINVAL;
	}

	return gxp_state2power_internal(state_pwr_map[state].state, power,
					thermal);
}

static int gxp_power2state(struct thermal_cooling_device *cdev,
			       u32 power, unsigned long *state)
{
	int i, penultimate_throttle_state;
	struct gxp_thermal_manager *thermal = cdev->devdata;

	*state = 0;
	/* Less than 2 state means we cannot really throttle */
	if (thermal->gxp_num_states < 2)
		return thermal->gxp_num_states == 1 ? 0 : -EIO;

	penultimate_throttle_state = thermal->gxp_num_states - 2;
	/*
	 * argument "power" is the maximum allowed power consumption in mW as
	 * defined by the PID control loop. Check for the first state that is
	 * less than or equal to the current allowed power. state_pwr_map is
	 * descending, so lowest power consumption is last value in the array
	 * return lowest state even if it consumes more power than allowed as
	 * not all platforms can handle throttling below an active state
	 */
	for (i = penultimate_throttle_state; i >= 0; --i) {
		if (power < state_pwr_map[i].power) {
			*state = i + 1;
			break;
		}
	}
	return 0;
}

static struct thermal_cooling_device_ops gxp_cooling_ops = {
	.get_max_state = gxp_get_max_state,
	.get_cur_state = gxp_get_cur_state,
	.set_cur_state = gxp_set_cur_state,
	.get_requested_power = gxp_get_requested_power,
	.state2power = gxp_state2power,
	.power2state = gxp_power2state,
};

static void gxp_thermal_exit(struct gxp_thermal_manager *thermal)
{
	if (!IS_ERR_OR_NULL(thermal->cdev))
		thermal_cooling_device_unregister(thermal->cdev);
}

static void devm_gxp_thermal_release(struct device *dev, void *res)
{
	struct gxp_thermal_manager *thermal = res;

	gxp_thermal_exit(thermal);
}

static ssize_t
user_vote_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev =
			container_of(dev, struct thermal_cooling_device,
				     device);
	struct gxp_thermal_manager *cooling = cdev->devdata;

	if (!cooling)
		return -ENODEV;

	return sysfs_emit(buf, "%lu\n", cooling->sysfs_req);
}

static ssize_t user_vote_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct thermal_cooling_device *cdev =
			container_of(dev, struct thermal_cooling_device,
				     device);
	struct gxp_thermal_manager *cooling = cdev->devdata;
	int ret;
	unsigned long state;

	if (!cooling)
		return -ENODEV;

	ret = kstrtoul(buf, 0, &state);
	if (ret)
		return ret;

	if (state >= cooling->gxp_num_states)
		return -EINVAL;

	mutex_lock(&cdev->lock);
	cooling->sysfs_req = state;
	cdev->updated = false;
	mutex_unlock(&cdev->lock);
	thermal_cdev_update(cdev);
	return count;
}

static DEVICE_ATTR_RW(user_vote);

static int
gxp_thermal_cooling_register(struct gxp_thermal_manager *thermal, char *type)
{
	struct device_node *cooling_node = NULL;

	thermal->op_data = NULL;
	thermal->gxp_num_states = ARRAY_SIZE(state_pwr_map);

	mutex_init(&thermal->lock);
	cooling_node = of_find_node_by_name(NULL, GXP_COOLING_NAME);

	/* TODO: Change this to fatal error once dts change is merged */
	if (!cooling_node)
		dev_warn(thermal->gxp->dev, "failed to find cooling node\n");
	/* Initialize the cooling state as 0, means "no cooling" */
	thermal->cooling_state = 0;
	thermal->cdev = thermal_of_cooling_device_register(
		cooling_node, type, thermal, &gxp_cooling_ops);
	if (IS_ERR(thermal->cdev))
		return PTR_ERR(thermal->cdev);

	return device_create_file(&thermal->cdev->device, &dev_attr_user_vote);
}

static int cooling_init(struct gxp_thermal_manager *thermal, struct device *dev)
{
	int err;
	struct dentry *d;

	d = debugfs_create_dir("cooling", thermal->gxp->d_entry);
	/* don't let debugfs creation failure abort the init procedure */
	if (IS_ERR_OR_NULL(d))
		dev_warn(dev, "failed to create debug fs for cooling");
	thermal->cooling_root = d;

	err = gxp_thermal_cooling_register(thermal, GXP_COOLING_NAME);
	if (err) {
		dev_err(dev, "failed to initialize external cooling\n");
		gxp_thermal_exit(thermal);
		return err;
	}
	return 0;
}

struct gxp_thermal_manager
*gxp_thermal_init(struct gxp_dev *gxp)
{
	struct device *dev = gxp->dev;
	struct gxp_thermal_manager *thermal;
	int err;

	thermal = devres_alloc(devm_gxp_thermal_release, sizeof(*thermal),
				GFP_KERNEL);
	if (!thermal)
		return ERR_PTR(-ENOMEM);

	thermal->gxp = gxp;
	err = cooling_init(thermal, dev);
	if (err) {
		devres_free(thermal);
		return ERR_PTR(err);
	}

	devres_add(dev, thermal);
	return thermal;
}
