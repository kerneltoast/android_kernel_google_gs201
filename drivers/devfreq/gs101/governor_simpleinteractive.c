// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/devfreq.h>
#include <linux/math64.h>
#include <soc/google/exynos_pm_qos.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/time.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/pm_opp.h>
#include "../governor.h"

#include <linux/devfreq.h>
#include <soc/google/exynos-devfreq.h>

static int devfreq_simple_interactive_notifier(struct notifier_block *nb, unsigned long val,
					       void *v)
{
	struct devfreq_notifier_block *devfreq_nb;

	devfreq_nb = container_of(nb, struct devfreq_notifier_block, nb);

	mutex_lock(&devfreq_nb->df->lock);
	update_devfreq(devfreq_nb->df);
	mutex_unlock(&devfreq_nb->df->lock);

	return NOTIFY_OK;
}

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
#define NEXTBUF(x, b)	do { if (++(x) > &(b)[LOAD_BUFFER_MAX - 1]) (x) = (b); } while (0)
#define POSTBUF(x, b)	((x) = ((--(x) < (b)) ?				\
			&(b)[LOAD_BUFFER_MAX - 1] : (x)))
static unsigned long update_load(struct devfreq_dev_status *stat,
				 struct devfreq_simple_interactive_data *data)
{
	struct devfreq_alt_load *ptr;
	struct devfreq_alt_dvfs_data *alt_data = &data->alt_data;
	unsigned int targetload;
	unsigned long freq;
	int i;
	struct exynos_profile_data *profile_data = (struct exynos_profile_data *)stat->private_data;

	if (!profile_data->total_time)
		return stat->current_frequency;
	for (i = 0; i < alt_data->num_target_load - 1 &&
	     stat->current_frequency >= alt_data->target_load[i + 1]; i += 2)
		;

	targetload = alt_data->target_load[i];

	/* if frequency is changed then reset the load */
	if (!stat->current_frequency ||
	    stat->current_frequency != data->prev_freq) {
		alt_data->rear = alt_data->front;
		alt_data->front->delta = 0;
		alt_data->total = 0;
		alt_data->busy = 0;
		if (alt_data->max_load >= targetload)
			alt_data->max_load = targetload;
		else
			alt_data->max_load = 0;

		alt_data->max_spent = 0;
		alt_data->min_load = targetload;
	}
	ptr = alt_data->front;
	ptr->delta += profile_data->delta_time;
	alt_data->max_spent += profile_data->delta_time;
	alt_data->total += profile_data->total_time;
	alt_data->busy += profile_data->busy_time;

	/* if too short time, then not counting */
	if (ptr->delta > alt_data->min_sample_time * NSEC_PER_MSEC) {
		NEXTBUF(alt_data->front, alt_data->buffer);
		alt_data->front->delta = 0;

		if (alt_data->front == alt_data->rear)
			NEXTBUF(alt_data->rear, alt_data->buffer);
		ptr->load = alt_data->total ?
				    alt_data->busy * 1000 / alt_data->total :
				    0;
		alt_data->busy = 0;
		alt_data->total = 0;

		/* if ptr load is higher than pervious or too small load */
		if (alt_data->max_load <= ptr->load) {
			alt_data->min_load = ptr->load;
			alt_data->max_spent = 0;
			alt_data->max_load = ptr->load;
			goto out;
		} else if (ptr->load < alt_data->min_load) {
			alt_data->min_load = ptr->load;
			if (ptr->load < alt_data->tolerance) {
				alt_data->max_load = ptr->load;
				alt_data->max_spent = 0;
				data->governor_freq = 0;
				return 0;
			}
		}
	}

	/* new max load */
	if (alt_data->max_spent > alt_data->hold_sample_time * NSEC_PER_MSEC) {
		unsigned long long spent = 0;
		/* if not valid data, then skip */
		if (alt_data->front == ptr) {
			spent += ptr->delta;
			POSTBUF(ptr, alt_data->buffer);
		}
		alt_data->max_load = ptr->load;
		alt_data->max_spent = spent;
		/* if there is downtrend, then reflect current load */
		if (ptr->load > alt_data->min_load + alt_data->tolerance) {
			alt_data->min_load = ptr->load;
			spent += ptr->delta;
			POSTBUF(ptr, alt_data->buffer);
			for (; spent < alt_data->hold_sample_time *
			     NSEC_PER_MSEC && ptr != alt_data->rear;
			     POSTBUF(ptr, alt_data->buffer)) {
				if (alt_data->max_load < ptr->load) {
					alt_data->max_load = ptr->load;
					alt_data->max_spent = spent;
				} else if (alt_data->min_load > ptr->load) {
					alt_data->min_load = ptr->load;
				}
				spent += ptr->delta;
			}
		} else {
			alt_data->min_load = ptr->load;
		}
	}
out:
	/* a few measurement */
	if (alt_data->max_load == targetload || alt_data->total)
		freq = data->governor_freq;
	else
		freq = alt_data->max_load * stat->current_frequency / targetload;

	/* Limit the change to 50% ~ 200% */
	freq = min(freq, stat->current_frequency << 1);
	freq = max(freq, stat->current_frequency >> 1);

	if (alt_data->max_load > alt_data->hispeed_load &&
	    alt_data->hispeed_freq > freq)
		freq = alt_data->hispeed_freq;

	data->governor_freq = freq;

	return freq;
}
#endif

static int devfreq_simple_interactive_func(struct devfreq *df,
					   unsigned long *freq)
{
	struct devfreq_simple_interactive_data *data = df->data;
	unsigned long exynos_pm_qos_min = 0;
	unsigned long exynos_pm_qos_max = INT_MAX;
	int delay_check = 0;
	int delay_time = 0;
	int i = 0;
	struct dev_pm_opp *limit_opp;
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	struct exynos_devfreq_data *exynos_df =
	       container_of(data, struct exynos_devfreq_data, simple_interactive_data);
	struct devfreq_dev_status *stat;
	int err;
#endif

	if (!data) {
		pr_err("%s: failed to find governor data\n", __func__);
		return -ENODATA;
	}

	exynos_pm_qos_min = exynos_pm_qos_request(data->pm_qos_class);
	if (data->pm_qos_class_max) {
		exynos_pm_qos_max = exynos_pm_qos_request(data->pm_qos_class_max);
		limit_opp = devfreq_recommended_opp(df->dev.parent, &exynos_pm_qos_max,
						    DEVFREQ_FLAG_LEAST_UPPER_BOUND);
		if (IS_ERR(limit_opp)) {
			pr_err("%s: failed to limit by max frequency\n", __func__);
			return PTR_ERR(limit_opp);
		}
		dev_pm_opp_put(limit_opp);
	}

	*freq = exynos_pm_qos_min;

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	stat = &df->last_status;

	if (df->profile->get_dev_status) {
		err = devfreq_update_stats(df);
		if (err)
			return err;
		*freq = max(*freq, update_load(stat, data));
	}

#endif
	if (!data->use_delay_time)
		goto out;

	if (data->prev_freq != df->previous_freq) {
		for (i = 0; i < data->ndelay_time - 1 &&
		     *freq >= data->delay_time[i + 1]; i += 2)
			;

		/* unit of delay time should be 10msec */
		delay_check = data->delay_time[i] % DELAY_TIME_RANGE;
		delay_time = delay_check ?
				data->delay_time[i] - delay_check + DELAY_TIME_RANGE :
				data->delay_time[i];

		data->freq_timer.expires = data->changed_time +
			msecs_to_jiffies(delay_time);
	}

	if (exynos_pm_qos_max > df->previous_freq && *freq < df->previous_freq &&
	    time_after(data->freq_timer.expires, jiffies)) {
		*freq = df->previous_freq;
		if (!timer_pending(&data->freq_timer))
			/* timer is bound to cpu0 */
			add_timer_on(&data->freq_timer, BOUND_CPU_NUM);

		goto out;
	} else if (timer_pending(&data->freq_timer)) {
		del_timer_sync(&data->freq_timer);
	}

	data->changed_time = jiffies;

out:
	/*
	 * save current frequency and time
	 * to use when update_devfreq is called next
	 */
	data->prev_freq = df->previous_freq;
	*freq = min(exynos_pm_qos_max, *freq);
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	if (df->profile->get_dev_status && !exynos_df->suspend_flag) {
		unsigned long expires = jiffies;

		mod_timer(&data->freq_timer, expires +
			msecs_to_jiffies(data->alt_data.min_sample_time * 2));
		if (*freq > exynos_df->min_freq) {
			/* timer is bound to cpu0 */
			mod_timer(&data->freq_slack_timer, expires +
				  msecs_to_jiffies(data->alt_data.hold_sample_time));
		} else if (timer_pending(&data->freq_slack_timer)) {
			del_timer(&data->freq_slack_timer);
		}

	} else if (exynos_df->suspend_flag) {
		del_timer_sync(&data->freq_timer);
		del_timer(&data->freq_slack_timer);
	}
#endif

	return 0;
}

static int devfreq_change_freq_task(void *data)
{
	struct devfreq *df = data;

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		schedule();

		set_current_state(TASK_RUNNING);

		mutex_lock(&df->lock);
		update_devfreq(df);
		mutex_unlock(&df->lock);
	}

	return 0;
}

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
static void alt_dvfs_nop_timer(struct timer_list *timer)
{
}
#endif

/*timer callback function send a signal */
static void simple_interactive_timer(struct timer_list *timer)
{
	struct devfreq_simple_interactive_data *gov_data = from_timer(gov_data, timer, freq_timer);

	wake_up_process(gov_data->change_freq_task);
}

static int devfreq_simple_interactive_register_notifier(struct devfreq *df)
{
	int ret;
	struct devfreq_simple_interactive_data *data = df->data;

	if (!data)
		return -EINVAL;

	data->nb.df = df;
	data->nb.nb.notifier_call = devfreq_simple_interactive_notifier;

	ret = exynos_pm_qos_add_notifier(data->pm_qos_class, &data->nb.nb);
	if (ret < 0)
		goto err1;

	if (data->pm_qos_class_max) {
		data->nb_max.df = df;
		data->nb_max.nb.notifier_call = devfreq_simple_interactive_notifier;

		ret = exynos_pm_qos_add_notifier(data->pm_qos_class_max, &data->nb_max.nb);
		if (ret < 0) {
			exynos_pm_qos_remove_notifier(data->pm_qos_class, &data->nb.nb);
			goto err2;
		}
	}

	/* timer of governor for delay time initialize */
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	timer_setup(&data->freq_timer, simple_interactive_timer, TIMER_DEFERRABLE);
	timer_setup(&data->freq_slack_timer, alt_dvfs_nop_timer, 0);
#else
	timer_setup(&data->freq_timer, simple_interactive_timer, 0);
#endif

	data->change_freq_task = kthread_create(devfreq_change_freq_task, df, "simpleinteractive");

	if (IS_ERR(data->change_freq_task)) {
		pr_err("%s: failed kthread_create for simpleinteractive governor\n", __func__);
		ret = PTR_ERR(data->change_freq_task);

		destroy_timer_on_stack(&data->freq_timer);
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
		destroy_timer_on_stack(&data->freq_slack_timer);
#endif
		exynos_pm_qos_remove_notifier(data->pm_qos_class, &data->nb.nb);
		if (data->pm_qos_class_max)
			exynos_pm_qos_remove_notifier(data->pm_qos_class_max, &data->nb_max.nb);

		goto err2;
	}

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	if (df->profile->get_dev_status) {
		data->freq_timer.expires = jiffies +
			msecs_to_jiffies(data->alt_data.min_sample_time * 2);
		add_timer(&data->freq_timer);
		data->freq_slack_timer.expires = jiffies +
			msecs_to_jiffies(data->alt_data.hold_sample_time);
		add_timer_on(&data->freq_slack_timer, BOUND_CPU_NUM);
	}

#else
	kthread_bind(data->change_freq_task, BOUND_CPU_NUM);
#endif

	wake_up_process(data->change_freq_task);
	return 0;

err2:
	kfree((void *)&data->nb_max.nb);

err1:
	kfree((void *)&data->nb.nb);

	return ret;
}

static int devfreq_simple_interactive_unregister_notifier(struct devfreq *df)
{
	int ret;
	struct devfreq_simple_interactive_data *data = df->data;

	if (!data)
		return -EINVAL;

	if (data->pm_qos_class_max) {
		ret = exynos_pm_qos_remove_notifier(data->pm_qos_class_max, &data->nb_max.nb);
		if (ret < 0)
			goto err;
	}

	ret = exynos_pm_qos_remove_notifier(data->pm_qos_class, &data->nb.nb);

	destroy_timer_on_stack(&data->freq_timer);
	kthread_stop(data->change_freq_task);

err:
	return ret;
}

static int devfreq_simple_interactive_handler(struct devfreq *devfreq,
					      unsigned int event, void *data)
{
	int ret;

	switch (event) {
	case DEVFREQ_GOV_START:
		ret = devfreq_simple_interactive_register_notifier(devfreq);
		if (ret)
			return ret;
		break;

	case DEVFREQ_GOV_STOP:
		ret = devfreq_simple_interactive_unregister_notifier(devfreq);
		if (ret)
			return ret;
		break;

	default:
		break;
	}

	return 0;
}

static struct devfreq_governor devfreq_simple_interactive = {
	.name = "interactive",
	.get_target_freq = devfreq_simple_interactive_func,
	.event_handler = devfreq_simple_interactive_handler,
};

int devfreq_simple_interactive_init(void)
{
	return devfreq_add_governor(&devfreq_simple_interactive);
}

MODULE_LICENSE("GPL");
