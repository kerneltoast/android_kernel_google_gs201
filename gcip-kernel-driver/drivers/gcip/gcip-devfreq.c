// SPDX-License-Identifier: GPL-2.0-only
/*
 * Devfreq support for GCIP devices.
 *
 * Copyright (C) 2024 Google LLC
 */

#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/units.h>

#include <gcip/gcip-devfreq.h>

/* Callback to set the device operating frequency. */
static int gcip_devfreq_target_no_ops(struct device *dev, unsigned long *freq, u32 flags)
{
	/*
	 * Setting device frequency for GCIP devices from devfreq is not in scope. Devfreq
	 * framework is adapted only for clamping the [min_freq , max_freq] range for the
	 * GCIP device. Refer go/gcip-devfreq for more details.
	 */
	return 0;
}

static void gcip_devfreq_remove_frequency_table(struct gcip_devfreq *gdevfreq)
{
	if (gdevfreq->dvfs_devfreq_freqs_num)
		dev_pm_opp_remove_all_dynamic(gdevfreq->dev);
}

static int gcip_devfreq_register_frequency_table(struct gcip_devfreq *gdevfreq)
{
	int i, ret;

	/* Get the frequency table from the driver. */
	gdevfreq->dvfs_devfreq_freqs_num = gdevfreq->ops->get_freq_table(
		gdevfreq->data, gdevfreq->dvfs_devfreq_freqs_khz, GCIP_DEVFREQ_MAX_DVFS_FREQ_NUM);
	if (!gdevfreq->dvfs_devfreq_freqs_num) {
		dev_err(gdevfreq->dev, "No frequency table provided by the driver.");
		return -EINVAL;
	}

	for (i = 0; i < gdevfreq->dvfs_devfreq_freqs_num; i++) {
		/*
		 * Passing zero frequency value will not throw error for dev_pm_opp_add() but will
		 * cause the failure in upstream devfreq initialization later. Hence return error if
		 * a zero frequency value is encountered.
		 */
		if (!gdevfreq->dvfs_devfreq_freqs_khz[i]) {
			dev_err(gdevfreq->dev, "Zero frequency value not allowed.\n");
			ret = -EINVAL;
			goto out;
		}

		/*
		 * dev_pm_opp_add() takes frequency in Hz and voltage in micro Volt as an argument.
		 * For gcip usage, voltage argument is not used and hence passed as zero.
		 */
		ret = dev_pm_opp_add(
			gdevfreq->dev,
			(unsigned long)HZ_PER_KHZ * gdevfreq->dvfs_devfreq_freqs_khz[i], 0);
		if (ret) {
			dev_err(gdevfreq->dev, "Registering %d kHz returned with error: %d.\n",
				gdevfreq->dvfs_devfreq_freqs_khz[i], ret);
			goto out;
		}
	}
	return 0;
out:
	gcip_devfreq_remove_frequency_table(gdevfreq);
	return ret;
}

/*
 * Returns the minimum valid frequency for the device registered in the OPP table. Returns the
 * theoretical minimum frequency of 0 in case OPP lib fails to give the minimum valid frequency for
 * the device.
 */
static unsigned long find_available_min_freq(struct device *dev)
{
	struct dev_pm_opp *opp;
	unsigned long min_freq = 0;

	/* Updates `min_freq` to the matching ceil opp frequency in success scenario. */
	opp = dev_pm_opp_find_freq_ceil(dev, &min_freq);
	if (IS_ERR(opp))
		dev_warn(dev, "Failed to find the min frequency limit:%ld.\n", PTR_ERR(opp));
	else
		dev_pm_opp_put(opp);

	return min_freq;
}

/*
 * Returns the maximum valid frequency for the device registered in the OPP table. Returns the
 * theoretical maximum frequency of ULONG_MAX in case OPP lib fails to give the maximum valid
 * frequency for the device.
 */
static unsigned long find_available_max_freq(struct device *dev)
{
	struct dev_pm_opp *opp;
	unsigned long max_freq = ULONG_MAX;

	/* Updates `max_freq` to the matching floor opp frequency in success scenario. */
	opp = dev_pm_opp_find_freq_floor(dev, &max_freq);
	if (IS_ERR(opp))
		dev_warn(dev, "Failed to find the max frequency limit:%ld.\n", PTR_ERR(opp));
	else
		dev_pm_opp_put(opp);

	return max_freq;
}

/**
 * gcip_devfreq_get_min_max_freq_range() - Get the [min_freq_hz, max_freq_hz] range.
 * @gdevfreq:    the devfreq instance.
 * @min_freq_hz: the min frequency.
 * @max_freq_hz: the max frequency.
 *
 * For calculating the frequency range, this function takes into consideration all the
 * constraints. Updates the min and max frequency limits to match what would have reflected
 * on reading the {min,max}_freq devfreq sysfs nodes.
 */
static void gcip_devfreq_get_min_max_freq_range(struct gcip_devfreq *gdevfreq,
						unsigned long *min_freq_hz,
						unsigned long *max_freq_hz)
{
	s32 qos_min_freq, qos_max_freq;

	mutex_lock(&gdevfreq->min_max_range_lock);

	/*
	 * Get min amd max frequency limit from OPP table. Unlikely but in case of failure to
	 * retrieve the min/max frequency from OPP table, theoretical min/max value of 0/ULONG_MAX
	 * would be returned. Then user provided DEV_PM_QOS_MIN_FREQUENCY and
	 * DEV_PM_QOS_MAX_FREQUENCY values will take precedence as seen in below logic.
	 */
	*min_freq_hz = find_available_min_freq(gdevfreq->dev);
	*max_freq_hz = find_available_max_freq(gdevfreq->dev);

	/* Get constraints from PM QoS. */
	qos_min_freq = dev_pm_qos_read_value(gdevfreq->dev, DEV_PM_QOS_MIN_FREQUENCY);
	qos_max_freq = dev_pm_qos_read_value(gdevfreq->dev, DEV_PM_QOS_MAX_FREQUENCY);

	*min_freq_hz = max(*min_freq_hz, (unsigned long)HZ_PER_KHZ * qos_min_freq);
	*max_freq_hz = min(*max_freq_hz, (unsigned long)HZ_PER_KHZ * qos_max_freq);

	if (*min_freq_hz > *max_freq_hz)
		*min_freq_hz = *max_freq_hz;

	mutex_unlock(&gdevfreq->min_max_range_lock);
}

static int gcip_devfreq_update_freq_range(struct gcip_devfreq *gdevfreq)
{
	unsigned long min_freq_hz, max_freq_hz;
	unsigned long nearest_ceil_min_freq_hz, neareset_floor_max_freq_hz;
	struct dev_pm_opp *opp;

	/*
	 * Update `min_freq_hz` and `max_freq_hz` with the values that will get reflected on
	 * {min,max}_freq sysfs node. [min_freq_hz, max_freq_hz] is supposed to be a valid range
	 * between the smallest and the largest frequency registered in the OPP library. For
	 * instance if registered frequencies are {100, 500, 1000} then valid range can be
	 * [250, 780], [578, 650] etc i.e. following 3 conditions will be followed:
	 * a. min_freq_hz >= smallest registered frequency
	 * b. max_freq_hz <= largest registered frequency
	 * c. min_freq_hz <= max_freq_hz
	 */
	gcip_devfreq_get_min_max_freq_range(gdevfreq, &min_freq_hz, &max_freq_hz);
	nearest_ceil_min_freq_hz = min_freq_hz;
	neareset_floor_max_freq_hz = max_freq_hz;

	/* Round up the min_freq to the nearest valid frequency. */
	opp = dev_pm_opp_find_freq_ceil(gdevfreq->dev, &nearest_ceil_min_freq_hz);
	if (IS_ERR(opp)) {
		dev_warn(gdevfreq->dev, "Failed to find OPP. Requested minfreq: %lu, err: %ld.\n",
			 min_freq_hz, PTR_ERR(opp));
		return PTR_ERR(opp);
	}
	dev_pm_opp_put(opp);

	/* Round down the max_freq to the nearest valid frequency. */
	opp = dev_pm_opp_find_freq_floor(gdevfreq->dev, &neareset_floor_max_freq_hz);
	if (IS_ERR(opp)) {
		dev_warn(gdevfreq->dev, "Failed to find OPP. Requested maxfreq: %lu, err: %ld.\n",
			 max_freq_hz, PTR_ERR(opp));
		return PTR_ERR(opp);
	}
	dev_pm_opp_put(opp);

	/*
	 * Boundary scenario where updated [min_freq_hz, max_freq_hz] happens to be a valid range
	 * between the smallest and the largest registered frequency but this range happens to be
	 * between two continuous discrete frequencies defined for the device. For example
	 * registered discrete frequencies are {100,500,1000}. User writes 200 on the min_freq node
	 * and 300 on the max_freq node. While reading back the {min,max}_freq nodes, user will get
	 * the frequency limit to be [200,300] but there is no valid frequency with in this range.
	 * Nearest ceil frequency of 200 would be 500 and nearest floor frequency of 300 would be
	 * 100. Resultant [500,100] range would be invalid and hence skip forwarding frequency range
	 * to the IP driver in such cases.
	 */
	if (nearest_ceil_min_freq_hz > neareset_floor_max_freq_hz) {
		dev_warn(gdevfreq->dev, "No valid frequency within [%lu, %lu] frequency range.\n",
			 min_freq_hz, max_freq_hz);
		return -ERANGE;
	}

	gdevfreq->ops->update_min_max_freq_range(gdevfreq->data,
						 nearest_ceil_min_freq_hz / HZ_PER_KHZ,
						 neareset_floor_max_freq_hz / HZ_PER_KHZ);
	return 0;
}

/* Notifier callback for DEV_PM_QOS_MIN_FREQUENCY update. */
static int gcip_devfreq_update_min_freq(struct notifier_block *nb, unsigned long event, void *data)
{
	struct gcip_devfreq *gdevfreq = container_of(nb, struct gcip_devfreq, min_freq_nb);
	int ret;

	ret = gcip_devfreq_update_freq_range(gdevfreq);
	if (ret)
		dev_warn(gdevfreq->dev, "Frequency range not forwarded to device (Err: %d).\n",
			 ret);

	return NOTIFY_OK;
}

/* Notifier callback for DEV_PM_QOS_MAX_FREQUENCY update. */
static int gcip_devfreq_update_max_freq(struct notifier_block *nb, unsigned long event, void *data)
{
	struct gcip_devfreq *gdevfreq = container_of(nb, struct gcip_devfreq, max_freq_nb);
	int ret;

	ret = gcip_devfreq_update_freq_range(gdevfreq);
	if (ret)
		dev_warn(gdevfreq->dev, "Frequency range not forwarded to device (Err: %d).\n",
			 ret);

	return NOTIFY_OK;
}

static int gcip_devfreq_register(struct gcip_devfreq *gdevfreq)
{
	struct devfreq_dev_profile *profile;
	int ret;

	profile = devm_kzalloc(gdevfreq->dev, sizeof(*profile), GFP_KERNEL);
	if (!profile) {
		ret = -ENOMEM;
		goto out;
	}

	/* Populate the profile fields. */
	profile->polling_ms = 0;
	profile->target = gcip_devfreq_target_no_ops;
	profile->get_cur_freq = gdevfreq->ops->get_cur_freq;
	gdevfreq->profile = profile;

	ret = gcip_devfreq_register_frequency_table(gdevfreq);

	if (ret)
		goto free_profile_struct;

	/* Register the notifier for DEV_PM_QOS_MIN_FREQUENCY. */
	gdevfreq->min_freq_nb.notifier_call = gcip_devfreq_update_min_freq;
	ret = dev_pm_qos_add_notifier(gdevfreq->dev, &gdevfreq->min_freq_nb,
				      DEV_PM_QOS_MIN_FREQUENCY);

	if (ret) {
		dev_err(gdevfreq->dev, "failed to add min_freq notifier:%d.", ret);
		goto remove_opp_table;
	}

	/* Register the notifier for DEV_PM_QOS_MAX_FREQUENCY. */
	gdevfreq->max_freq_nb.notifier_call = gcip_devfreq_update_max_freq;
	ret = dev_pm_qos_add_notifier(gdevfreq->dev, &gdevfreq->max_freq_nb,
				      DEV_PM_QOS_MAX_FREQUENCY);

	if (ret) {
		dev_err(gdevfreq->dev, "failed to add max_freq notifier:%d.", ret);
		goto remove_min_nb;
	}

	mutex_init(&gdevfreq->min_max_range_lock);

	gdevfreq->devfreq =
		devm_devfreq_add_device(gdevfreq->dev, profile, DEVFREQ_GOV_POWERSAVE, NULL);
	if (IS_ERR(gdevfreq->devfreq)) {
		dev_err(gdevfreq->dev, "failed to add devfreq device: %ld\n",
			PTR_ERR(gdevfreq->devfreq));
		ret = PTR_ERR(gdevfreq->devfreq);
		goto remove_max_nb;
	}
	return 0;

remove_max_nb:
	dev_pm_qos_remove_notifier(gdevfreq->dev, &gdevfreq->max_freq_nb, DEV_PM_QOS_MAX_FREQUENCY);
remove_min_nb:
	dev_pm_qos_remove_notifier(gdevfreq->dev, &gdevfreq->min_freq_nb, DEV_PM_QOS_MIN_FREQUENCY);
remove_opp_table:
	gcip_devfreq_remove_frequency_table(gdevfreq);
free_profile_struct:
	devm_kfree(gdevfreq->dev, gdevfreq->profile);
out:
	return ret;
}

static void gcip_devfreq_unregister(struct gcip_devfreq *gdevfreq)
{
	devm_devfreq_remove_device(gdevfreq->dev, gdevfreq->devfreq);

	mutex_destroy(&gdevfreq->min_max_range_lock);
	dev_pm_qos_remove_notifier(gdevfreq->dev, &gdevfreq->max_freq_nb, DEV_PM_QOS_MAX_FREQUENCY);
	dev_pm_qos_remove_notifier(gdevfreq->dev, &gdevfreq->min_freq_nb, DEV_PM_QOS_MIN_FREQUENCY);

	gcip_devfreq_remove_frequency_table(gdevfreq);

	devm_kfree(gdevfreq->dev, gdevfreq->profile);
}

struct gcip_devfreq *gcip_devfreq_create(const struct gcip_devfreq_args *args)
{
	struct gcip_devfreq *gdevfreq;
	int ret;

	if (!args->dev)
		return ERR_PTR(-EINVAL);

	if (!args->ops->update_min_max_freq_range || !args->ops->get_cur_freq ||
	    !args->ops->get_freq_table) {
		dev_err(args->dev, "Required callbacks are not provided by the driver.");
		return ERR_PTR(-EINVAL);
	}

	gdevfreq = devm_kzalloc(args->dev, sizeof(*gdevfreq), GFP_KERNEL);
	if (!gdevfreq)
		return ERR_PTR(-ENOMEM);

	/* Populate the gdevfreq fields. */
	gdevfreq->dev = args->dev;
	gdevfreq->data = args->data;
	gdevfreq->ops = args->ops;

	ret = gcip_devfreq_register(gdevfreq);
	if (ret)
		goto out;

	return gdevfreq;
out:
	devm_kfree(gdevfreq->dev, gdevfreq);
	return ERR_PTR(ret);
}

void gcip_devfreq_destroy(struct gcip_devfreq *gdevfreq)
{
	gcip_devfreq_unregister(gdevfreq);
	devm_kfree(gdevfreq->dev, gdevfreq);
}
