// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thermal management support for GCIP devices.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/thermal.h>

#include <gcip/gcip-config.h>
#include <gcip/gcip-pm.h>
#include <gcip/gcip-thermal.h>

#define OF_DATA_NUM_MAX (GCIP_THERMAL_MAX_NUM_STATES * 2)

#define to_cdev(dev) container_of(dev, struct thermal_cooling_device, device)
#define to_gcip_thermal(dev) ((struct gcip_thermal *)to_cdev(dev)->devdata)

/* Struct for state to rate and state to power mappings. */
struct gcip_rate_pwr {
	unsigned long rate;
	u32 power;
};

static struct gcip_rate_pwr state_map[GCIP_THERMAL_MAX_NUM_STATES] = { 0 };

static int gcip_thermal_get_max_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct gcip_thermal *thermal = cdev->devdata;

	if (!thermal->num_states)
		return -ENODEV;

	*state = thermal->num_states - 1;

	return 0;
}

static int gcip_thermal_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct gcip_thermal *thermal = cdev->devdata;

	mutex_lock(&thermal->lock);
	*state = thermal->state;
	mutex_unlock(&thermal->lock);

	return 0;
}

static int gcip_thermal_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct gcip_thermal *thermal = cdev->devdata;
	int i, ret = 0;

	if (state >= thermal->num_states) {
		dev_err(thermal->dev, "Invalid thermal cooling state %lu\n", state);
		return -EINVAL;
	}

	mutex_lock(&thermal->lock);

	thermal->vote[GCIP_THERMAL_COOLING_DEVICE] = state;
	for (i = 0; i < GCIP_THERMAL_MAX_NUM_VOTERS; i++)
		state = max(state, thermal->vote[i]);

	if (state == thermal->state)
		goto out;

	if (!gcip_pm_get_if_powered(thermal->pm, false)) {
		ret = thermal->set_rate(thermal->data, state_map[state].rate);
		gcip_pm_put_async(thermal->pm);
	}

	if (ret)
		dev_err(thermal->dev, "Failed to set thermal cooling state: %d\n", ret);
	else
		thermal->state = state;
out:
	mutex_unlock(&thermal->lock);

	return ret;
}

static int gcip_thermal_rate2power_internal(struct gcip_thermal *thermal, unsigned long rate,
					    u32 *power)
{
	int i;

	for (i = 0; i < thermal->num_states; i++) {
		if (rate == state_map[i].rate) {
			*power = state_map[i].power;
			return 0;
		}
	}

	dev_err(thermal->dev, "Unknown rate for: %lu\n", rate);
	*power = 0;

	return -EINVAL;
}

static int gcip_thermal_get_requested_power(struct thermal_cooling_device *cdev, u32 *power)
{
	struct gcip_thermal *thermal = cdev->devdata;
	unsigned long rate;
	int ret;

	if (gcip_pm_get_if_powered(thermal->pm, false)) {
		*power = 0;
		return 0;
	}

	mutex_lock(&thermal->lock);

	ret = thermal->get_rate(thermal->data, &rate);

	mutex_unlock(&thermal->lock);
	gcip_pm_put_async(thermal->pm);

	if (ret)
		return ret;

	return gcip_thermal_rate2power_internal(thermal, rate, power);
}

static int gcip_thermal_state2power(struct thermal_cooling_device *cdev, unsigned long state,
				    u32 *power)
{
	struct gcip_thermal *thermal = cdev->devdata;

	if (state >= thermal->num_states) {
		dev_err(thermal->dev, "Invalid state: %lu\n", state);
		return -EINVAL;
	}

	return gcip_thermal_rate2power_internal(thermal, state_map[state].rate, power);
}

static int gcip_thermal_power2state(struct thermal_cooling_device *cdev, u32 power,
				    unsigned long *state)
{
	struct gcip_thermal *thermal = cdev->devdata;

	if (!thermal->num_states)
		return -ENODEV;

	/*
	 * Argument "power" is the maximum allowed power consumption in mW as defined by the PID
	 * control loop. Checks for the first state that is less than or equal to the current
	 * allowed power. state_map is descending, so lowest power consumption is last value in the
	 * array. Returns lowest state even if it consumes more power than allowed as not all
	 * platforms can handle throttling below an active state.
	 */
	for (*state = 0; *state < thermal->num_states; (*state)++)
		if (power >= state_map[*state].power)
			return 0;

	*state = thermal->num_states - 1;

	return 0;
}

static const struct thermal_cooling_device_ops gcip_thermal_ops = {
	.get_max_state = gcip_thermal_get_max_state,
	.get_cur_state = gcip_thermal_get_cur_state,
	.set_cur_state = gcip_thermal_set_cur_state,
	.get_requested_power = gcip_thermal_get_requested_power,
	.state2power = gcip_thermal_state2power,
	.power2state = gcip_thermal_power2state,
};

/* This API was removed, but Android still uses it to update thermal request. */
void thermal_cdev_update(struct thermal_cooling_device *cdev);

static void gcip_thermal_update(struct gcip_thermal *thermal)
{
	struct thermal_cooling_device *cdev = thermal->cdev;

	cdev->updated = false;

	thermal_cdev_update(cdev);
}

static ssize_t user_vote_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gcip_thermal *thermal = to_gcip_thermal(dev);
	ssize_t ret;

	if (!thermal)
		return -ENODEV;

	mutex_lock(&thermal->lock);
	ret = sysfs_emit(buf, "%lu\n", thermal->vote[GCIP_THERMAL_SYSFS]);
	mutex_unlock(&thermal->lock);

	return ret;
}

static ssize_t user_vote_store(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct gcip_thermal *thermal = to_gcip_thermal(dev);
	unsigned long state;
	int ret;

	if (!thermal)
		return -ENODEV;

	ret = kstrtoul(buf, 0, &state);
	if (ret)
		return ret;

	if (state >= thermal->num_states)
		return -EINVAL;

	mutex_lock(&thermal->lock);
	thermal->vote[GCIP_THERMAL_SYSFS] = state;
	mutex_unlock(&thermal->lock);

	gcip_thermal_update(thermal);

	return count;
}

static DEVICE_ATTR_RW(user_vote);

static int gcip_thermal_rate2state(struct gcip_thermal *thermal, unsigned long rate)
{
	int i;

	for (i = 0; i < thermal->num_states; i++) {
		if (state_map[i].rate <= rate)
			return i;
	}

	/* Returns lowest state on an invalid input. */
	return thermal->num_states - 1;
}

static int gcip_thermal_notifier(struct notifier_block *nb, unsigned long rate, void *nb_data)
{
	struct gcip_thermal *thermal = container_of(nb, struct gcip_thermal, nb);
	unsigned long state = gcip_thermal_rate2state(thermal, rate);

	dev_dbg(thermal->dev, "Thermal notifier req original: %lu, state: %lu\n", rate, state);

	mutex_lock(&thermal->lock);
	thermal->vote[GCIP_THERMAL_NOTIFIER_BLOCK] = state;
	mutex_unlock(&thermal->lock);

	gcip_thermal_update(thermal);

	return NOTIFY_OK;
}

struct notifier_block *gcip_thermal_get_notifier_block(struct gcip_thermal *thermal)
{
	if (IS_ERR_OR_NULL(thermal))
		return NULL;

	return &thermal->nb;
}

static ssize_t state2power_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gcip_thermal *thermal = to_gcip_thermal(dev);
	ssize_t count = 0;
	int i;
	u32 power;

	for (i = 0; i < thermal->num_states; i++) {
		gcip_thermal_state2power(thermal->cdev, i, &power);
		count += sysfs_emit_at(buf, count, "%u ", power);
	}
	count += sysfs_emit_at(buf, count, "\n");

	return count;
}

static DEVICE_ATTR_RO(state2power_table);

void gcip_thermal_destroy(struct gcip_thermal *thermal)
{
	if (IS_ERR_OR_NULL(thermal))
		return;

	debugfs_remove_recursive(thermal->dentry);
	thermal_cooling_device_unregister(thermal->cdev);
	devm_kfree(thermal->dev, thermal);
}

static int gcip_thermal_enable_get(void *data, u64 *val)
{
	struct gcip_thermal *thermal = (struct gcip_thermal *)data;

	mutex_lock(&thermal->lock);
	*val = thermal->enabled;
	mutex_unlock(&thermal->lock);

	return 0;
}

static int gcip_thermal_enable_set(void *data, u64 val)
{
	struct gcip_thermal *thermal = (struct gcip_thermal *)data;
	int ret = 0;

	mutex_lock(&thermal->lock);

	if (thermal->enabled != (bool)val) {
		/*
		 * If the device is not powered, the value will be restored by
		 * gcip_thermal_restore_on_powering in next fw boot.
		 */
		if (!gcip_pm_get_if_powered(thermal->pm, false)) {
			ret = thermal->control(thermal->data, val);
			gcip_pm_put_async(thermal->pm);
		}

		if (!ret) {
			thermal->enabled = val;
			dev_info_ratelimited(thermal->dev, "%s thermal control",
					     thermal->enabled ? "Enable" : "Disable");
		} else {
			dev_err(thermal->dev, "Failed to %s thermal control: %d ",
				val ? "enable" : "disable", ret);
		}
	}

	mutex_unlock(&thermal->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_gcip_thermal_enable, gcip_thermal_enable_get, gcip_thermal_enable_set,
			 "%llu\n");

static int gcip_thermal_parse_dvfs_table(struct gcip_thermal *thermal)
{
	int row_size, col_size, tbl_size, i;
	int of_data_int_array[OF_DATA_NUM_MAX];

	if (of_property_read_u32_array(thermal->dev->of_node, GCIP_THERMAL_TABLE_SIZE_NAME,
				       of_data_int_array, 2))
		goto error;

	row_size = of_data_int_array[0];
	col_size = of_data_int_array[1];
	tbl_size = row_size * col_size;
	if (row_size > GCIP_THERMAL_MAX_NUM_STATES) {
		dev_err(thermal->dev, "Too many states\n");
		goto error;
	}

	if (tbl_size > OF_DATA_NUM_MAX)
		goto error;

	if (of_property_read_u32_array(thermal->dev->of_node, GCIP_THERMAL_TABLE_NAME,
				       of_data_int_array, tbl_size))
		goto error;

	thermal->num_states = row_size;
	for (i = 0; i < row_size; ++i) {
		int idx = col_size * i;

		state_map[i].rate = of_data_int_array[idx];
		state_map[i].power = of_data_int_array[idx + 1];
	}

	return 0;

error:
	dev_err(thermal->dev, "Failed to parse DVFS table\n");

	return -EINVAL;
}

static int gcip_thermal_cooling_register(struct gcip_thermal *thermal, const char *type,
					 const char *node_name)
{
	struct device_node *node = NULL;
	int ret;

	ret = gcip_thermal_parse_dvfs_table(thermal);
	if (ret)
		return ret;

	if (node_name)
		node = of_find_node_by_name(NULL, node_name);
	if (!node)
		dev_warn(thermal->dev, "Failed to find thermal cooling node: %s\n",
			 node_name ? node_name : "");

	thermal->cdev = thermal_of_cooling_device_register(node, type, thermal, &gcip_thermal_ops);
	/* OK to call it even if @node is NULL. */
	of_node_put(node);
	if (IS_ERR(thermal->cdev))
		return PTR_ERR(thermal->cdev);

	ret = device_create_file(&thermal->cdev->device, &dev_attr_user_vote);
	if (ret)
		thermal_cooling_device_unregister(thermal->cdev);

	ret = device_create_file(&thermal->cdev->device, &dev_attr_state2power_table);
	if (ret)
		thermal_cooling_device_unregister(thermal->cdev);

	return ret;
}

struct gcip_thermal *gcip_thermal_create(const struct gcip_thermal_args *args)
{
	struct gcip_thermal *thermal;
	int ret;

	if (!args->dev || !args->get_rate || !args->set_rate || !args->control)
		return ERR_PTR(-EINVAL);

	thermal = devm_kzalloc(args->dev, sizeof(*thermal), GFP_KERNEL);
	if (!thermal)
		return ERR_PTR(-ENOMEM);

	thermal->dev = args->dev;
	thermal->nb.notifier_call = gcip_thermal_notifier;
	thermal->pm = args->pm;
	thermal->enabled = true;
	thermal->data = args->data;
	thermal->get_rate = args->get_rate;
	thermal->set_rate = args->set_rate;
	thermal->control = args->control;

	mutex_init(&thermal->lock);

	ret = gcip_thermal_cooling_register(thermal, args->type, args->node_name);
	if (ret) {
		dev_err(args->dev, "Failed to initialize external thermal cooling\n");
		devm_kfree(args->dev, thermal);
		return ERR_PTR(ret);
	}

	thermal->dentry = debugfs_create_dir("cooling", args->dentry);
	/* Don't let debugfs creation failure abort the init procedure. */
	if (IS_ERR_OR_NULL(thermal->dentry))
		dev_warn(args->dev, "Failed to create debugfs for thermal cooling");
	else
		debugfs_create_file("enable", 0660, thermal->dentry, thermal,
				    &fops_gcip_thermal_enable);

	return thermal;
}

int gcip_thermal_suspend_device(struct gcip_thermal *thermal)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(thermal))
		return 0;

	mutex_lock(&thermal->lock);

	/*
	 * Always sets as suspended even when the request cannot be handled for unknown reasons
	 * because we still want to prevent the client from using device.
	 */
	thermal->device_suspended = true;
	if (!gcip_pm_get_if_powered(thermal->pm, false)) {
		ret = thermal->set_rate(thermal->data, 0);
		gcip_pm_put_async(thermal->pm);
	}

	mutex_unlock(&thermal->lock);

	return ret;
}

int gcip_thermal_resume_device(struct gcip_thermal *thermal)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(thermal))
		return 0;

	mutex_lock(&thermal->lock);

	if (!gcip_pm_get_if_powered(thermal->pm, false)) {
		ret = thermal->set_rate(thermal->data, state_map[thermal->state].rate);
		gcip_pm_put_async(thermal->pm);
	}

	/*
	 * Unlike gcip_thermal_suspend_device(), only sets the device as resumed if the request is
	 * fulfilled.
	 */
	if (!ret)
		thermal->device_suspended = false;

	mutex_unlock(&thermal->lock);

	return ret;
}

bool gcip_thermal_is_device_suspended(struct gcip_thermal *thermal)
{
	if (IS_ERR_OR_NULL(thermal))
		return false;

	return thermal->device_suspended;
}

int gcip_thermal_restore_on_powering(struct gcip_thermal *thermal)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(thermal))
		return 0;

	mutex_lock(&thermal->lock);

	if (!thermal->enabled)
		ret = thermal->control(thermal->data, thermal->enabled);
	else if (thermal->device_suspended)
		ret = thermal->set_rate(thermal->data, 0);
	else if (thermal->state)
		/* Skips state 0 since it's the default thermal state. */
		ret = thermal->set_rate(thermal->data, state_map[thermal->state].rate);

	mutex_unlock(&thermal->lock);

	return ret;
}
