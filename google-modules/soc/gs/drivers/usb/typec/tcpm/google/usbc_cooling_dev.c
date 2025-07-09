// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021, Google LLC
 *
 * Type-C port cooling device.
 */

#include <linux/alarmtimer.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/thermal.h>

#include "usb_icl_voter.h"
#include "usb_thermal_voter.h"

#define USBC_COOLING_DEV_VOTER		"USBC_COOLING_DEV_VOTER"

#define TOGGLE_DISABLE_VOTABLE		"TOGGLE_DISABLE"

enum {
	ENABLE_TOGGLE,
	DISABLE_TOGGLE,
};

/* Vote value doesn't matter. Only status matters. */
#define DISABLE_TOGGLE_VOTE                    1

enum {
	DISABLE_ICL_VOTE,
	ENABLE_ICL_VOTE,
};

#define THROTTLE_USBIN_LIMIT			0

enum {
	USB_NO_LIMIT = 0,
	USB_COOLING_DEV_THROTTLE = 1,
	USB_MAX_THROTTLE_STATE = USB_COOLING_DEV_THROTTLE,
};

struct usb_port_cooling_dev_stats {
	int trip_time;
	int hysteresis_time;
	int cleared_time;
};

struct usb_port_cooling_dev_info {
	struct device *dev;
	/* Votable to limit USBIN current limit */
	struct gvotable_election *usb_icl_votable;
	/* Votable to disable USB-C port */
	struct gvotable_election *toggle_disable_votable;
	/* Votable that signals whether USB is suspended or resumed */
	struct gvotable_election *usb_throttle_votable;
	struct usb_port_cooling_dev_stats stats;
	/* usb-port cooling device */
	struct thermal_cooling_device *cooling_dev;

	/* Wakeup alarm for implementing unplug requirement */
	struct alarm unplug_alarm;

	/* Extcon dev to infer usb connected status */
	struct extcon_dev *edev;
	/* Notifier block to infer connected status */
	struct notifier_block connected_nb;
	/* Set when an accessory or TA is connected. Current state of the port */
	bool usb_connected;
	/* Completion to wait for connected notification */
	struct completion connected_completion;

	/* Set when USB-C port is disabled */
	bool usb_suspended;

	/* boot_time when usb was plugged */
	ktime_t plug_time_since_boot;
	/* boot_time when usb-port cooling device started throttling */
	ktime_t trip_time_since_boot;
	/* boot_time when usb-port cooling device stopped throttling */
	ktime_t hysteresis_time_since_boot;
	/* boot time when USB-C was unplugged when cooling device stopped throttling */
	ktime_t unplug_time_since_boot;

	/* Throttle state of the usb-port cooling device */
	unsigned long throttle_state;

	u32 cooling_device_unplug_interval_sec;
	u32 cooling_device_polling_interval_ms;
	u32 cooling_device_connected_interval_ms;

	/* Single threaded workqueue for processing cooling device events */
	struct workqueue_struct *wq;
};

struct usb_cdev_event {
	struct work_struct work;
	struct usb_port_cooling_dev_info *usb_cdev_info;
};

#define USB_CDEV_ATTR(_name)									\
static ssize_t _name##_show(struct device *dev, struct device_attribute *attr, char *buf)	\
{												\
	struct usb_port_cooling_dev_info *usb_cdev_info = dev_get_drvdata(dev);			\
												\
	return sysfs_emit(buf, "%d\n", usb_cdev_info->stats._name);				\
}												\
static DEVICE_ATTR_RO(_name)

USB_CDEV_ATTR(trip_time);
USB_CDEV_ATTR(hysteresis_time);
USB_CDEV_ATTR(cleared_time);

static struct attribute *usb_cdev_attrs[] = {
	&dev_attr_hysteresis_time.attr,
	&dev_attr_trip_time.attr,
	&dev_attr_cleared_time.attr,
	NULL,
};
ATTRIBUTE_GROUPS(usb_cdev);

static inline ktime_t get_seconds_since_boot(void)
{
	return div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC);
}

static inline int get_dts_vars(struct usb_port_cooling_dev_info *usb_cdev_info)
{
	struct device *dev = usb_cdev_info->dev;
	struct device_node *node = dev->of_node;
	int ret;

	ret = of_property_read_u32(node, "google,usb-cd-polling-interval-ms",
				   &usb_cdev_info->cooling_device_polling_interval_ms);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev,
			"cannot read usb-cd-polling-interval-ms, ret=%d\n",
			ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,usb-cd-unplug-interval-sec",
				   &usb_cdev_info->cooling_device_unplug_interval_sec);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev,
			"cannot read usb-cd-unplug-interval-ms, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,usb-cd-connected-interval-ms",
				   &usb_cdev_info->cooling_device_connected_interval_ms);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev,
			"cannot read usb-cd-connected-interval-ms, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/* Type-C USB port will no longer detect Type-C chargers or accessories */
static int suspend_usb(struct usb_port_cooling_dev_info *usb_cdev_info)
{
	int ret;

	/* Early return if usb is already suspended */
	if (usb_cdev_info->usb_suspended)
		return 0;

	/* disable Type-C port */
	ret = gvotable_cast_vote(usb_cdev_info->toggle_disable_votable,
				 USBC_COOLING_DEV_VOTER,
				 (void *)DISABLE_TOGGLE_VOTE, DISABLE_TOGGLE);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't vote for toggle_disable_votable ret=%d\n",
			ret);
		return ret;
	}

	/* suspend USBIN */
	ret = gvotable_cast_vote(usb_cdev_info->usb_icl_votable, USBC_COOLING_DEV_VOTER,
				 (void *)THROTTLE_USBIN_LIMIT, ENABLE_ICL_VOTE);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't vote for USB ICL ret=%d\n", ret);
		return ret;
	}

	/*
	 * Vote to let TCPCI know that USB has been throttled.
	 * Different from toggle_disable_votable where the toggling is disabled
	 * intermittently.
	 */
	ret = gvotable_cast_vote(usb_cdev_info->usb_throttle_votable, USBC_COOLING_DEV_VOTER,
				 (void *)DISABLE_TOGGLE_VOTE, USB_SUSPENDED);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't vote for usb_throttle_votable ret=%d\n",
			ret);
		return ret;
	}

	dev_warn(usb_cdev_info->dev, "USB suspended\n");
	usb_cdev_info->trip_time_since_boot = get_seconds_since_boot();
	usb_cdev_info->usb_suspended = true;
	return ret;
}

static int resume_usb(struct usb_port_cooling_dev_info *usb_cdev_info)
{
	int ret;

	/* Early return if usb is already running */
	if (!usb_cdev_info->usb_suspended)
		return 0;

	/* Fill out stats so userspace can read them. */
	usb_cdev_info->stats.trip_time =
		(int)(usb_cdev_info->trip_time_since_boot - usb_cdev_info->plug_time_since_boot);
	usb_cdev_info->stats.hysteresis_time =
		(int)(usb_cdev_info->hysteresis_time_since_boot -
		      usb_cdev_info->trip_time_since_boot);
	usb_cdev_info->stats.cleared_time =
		(int)(get_seconds_since_boot() - usb_cdev_info->hysteresis_time_since_boot);

	/* enable USBIN */
	ret = gvotable_cast_vote(usb_cdev_info->usb_icl_votable, USBC_COOLING_DEV_VOTER,
				 (void *)THROTTLE_USBIN_LIMIT, DISABLE_ICL_VOTE);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't un-vote for USB ICL ret=%d\n", ret);
		return ret;
	}

	/*
	 * Vote to let TCPCI know that USB has been resumed.
	 * Different from toggle_disable_votable where the toggling is disabled
	 * intermittently.
	 * Vote before enable toggling so that the CC updates are processed.
	 */
	ret = gvotable_cast_vote(usb_cdev_info->usb_throttle_votable, USBC_COOLING_DEV_VOTER,
				 (void *)DISABLE_TOGGLE_VOTE, USB_RESUMED);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't un-vote for usb_throttle_votable ret=%d\n",
			ret);
		return ret;
	}

	/* enable USB */
	ret = gvotable_cast_vote(usb_cdev_info->toggle_disable_votable, USBC_COOLING_DEV_VOTER,
				 (void *)DISABLE_TOGGLE_VOTE, ENABLE_TOGGLE);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't un-vote for toggle_disable_votable ret=%d\n",
			ret);
		return ret;
	}

	dev_warn(usb_cdev_info->dev, "USB resumed\n");
	/* Notify userspace to read the stats. */
	kobject_uevent(&usb_cdev_info->dev->kobj, KOBJ_CHANGE);

	usb_cdev_info->plug_time_since_boot = 0;
	usb_cdev_info->trip_time_since_boot = 0;
	usb_cdev_info->hysteresis_time_since_boot = 0;
	usb_cdev_info->usb_suspended = false;
	return ret;
}

static int update_usb_plugged_status(struct usb_port_cooling_dev_info *usb_cdev_info)
{
	int ret;
	bool prev_usb_connected;

	ret = extcon_get_state(usb_cdev_info->edev, EXTCON_MECHANICAL);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't get extcon state, ret=%d\n", ret);
		return ret;
	}

	dev_dbg(usb_cdev_info->dev, "extcon state: %d\n", ret);
	prev_usb_connected = usb_cdev_info->usb_connected;
	usb_cdev_info->usb_connected = ret;

	if (usb_cdev_info->usb_connected && !prev_usb_connected)
		usb_cdev_info->plug_time_since_boot = get_seconds_since_boot();

	return ret;
}

/* USB-C cooling device work run in single thread workqueue */
static void usb_cdev_work(struct work_struct *work)
{
	struct usb_cdev_event *event = container_of(work, struct usb_cdev_event, work);
	struct usb_port_cooling_dev_info *usb_cdev_info = event->usb_cdev_info;
	int ret;
	u32 connect_wait_ms = usb_cdev_info->cooling_device_connected_interval_ms;

	/* Port is too hot suspend usb */
	if (usb_cdev_info->throttle_state == USB_COOLING_DEV_THROTTLE) {
		suspend_usb(usb_cdev_info);
		/* Reset hysteresis time. Set when cold */
		usb_cdev_info->hysteresis_time_since_boot = 0;
		/* Cancel unplug alarm, if running */
		alarm_cancel(&usb_cdev_info->unplug_alarm);
		goto free_event;
	}

	/* Update Plug in time. Port is not hot here */
	if (!usb_cdev_info->usb_suspended) {
		update_usb_plugged_status(usb_cdev_info);
		goto free_event;
	}

	/* Port is cold satisfy unplug requirement before re-enabling usb */
	if (!usb_cdev_info->hysteresis_time_since_boot) {
		dev_warn(usb_cdev_info->dev, "Port now cold\n");
		usb_cdev_info->hysteresis_time_since_boot = get_seconds_since_boot();
	}

	reinit_completion(&usb_cdev_info->connected_completion);

	/* Enable USB port to check status */
	ret = gvotable_cast_vote(usb_cdev_info->toggle_disable_votable, USBC_COOLING_DEV_VOTER,
				 (void *)DISABLE_TOGGLE_VOTE, ENABLE_TOGGLE);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't un-vote for toggle_disable_votable ret=%d\n",
			ret);
		goto resched;
	}

	/* Wait for Type-C state machine to detect */
	wait_for_completion_timeout(&usb_cdev_info->connected_completion,
				    msecs_to_jiffies(connect_wait_ms));

	ret = update_usb_plugged_status(usb_cdev_info);
	if (ret < 0)
		goto resched;

	/* Disable port after checking status */
	ret = gvotable_cast_vote(usb_cdev_info->toggle_disable_votable, USBC_COOLING_DEV_VOTER,
				 (void *)DISABLE_TOGGLE_VOTE, DISABLE_TOGGLE);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Couldn't vote for toggle_disable_votable ret=%d\n",
			ret);
		goto resched;
	}

	if (usb_cdev_info->usb_connected) {
		dev_warn(usb_cdev_info->dev, "Port cold. Waiting for unplug\n");
		/* Reset unplug time */
		usb_cdev_info->unplug_time_since_boot = 0;
		goto resched;
	}

	if (!usb_cdev_info->unplug_time_since_boot) {
		dev_warn(usb_cdev_info->dev, "Port cold and unplugged\n");
		usb_cdev_info->unplug_time_since_boot = get_seconds_since_boot();
	}

	/* Re-enable USB when unplug requirement is satisfied */
	if (get_seconds_since_boot() - usb_cdev_info->unplug_time_since_boot >
	    usb_cdev_info->cooling_device_unplug_interval_sec) {
		resume_usb(usb_cdev_info);
		goto free_event;
	}

resched:
	alarm_start_relative(&usb_cdev_info->unplug_alarm,
			     ms_to_ktime(usb_cdev_info->cooling_device_polling_interval_ms));
free_event:
	pm_relax(usb_cdev_info->dev);
	devm_kfree(usb_cdev_info->dev, event);
}

static void usb_cdev_changed(struct usb_port_cooling_dev_info *usb_cdev_info)
{
	struct usb_cdev_event *event;

	event = devm_kzalloc(usb_cdev_info->dev, sizeof(*event), GFP_ATOMIC);
	if (!event) {
		dev_err(usb_cdev_info->dev, "Event missed!");
		return;
	}

	INIT_WORK(&event->work, usb_cdev_work);
	event->usb_cdev_info = usb_cdev_info;
	pm_stay_awake(usb_cdev_info->dev);
	queue_work(usb_cdev_info->wq, &event->work);
}

static int usb_connected_notifier(struct notifier_block *nb, unsigned long action, void *dev)
{
	struct usb_port_cooling_dev_info *usb_cdev_info =
		container_of(nb, struct usb_port_cooling_dev_info, connected_nb);

	dev_dbg(usb_cdev_info->dev, "name=usb evt=%lu %s\n", action, usb_cdev_info->usb_suspended ?
		"Dropping as USB is suspended" : "");

	if (!usb_cdev_info->usb_suspended)
		usb_cdev_changed(usb_cdev_info);
	else
		complete(&usb_cdev_info->connected_completion);

	return NOTIFY_OK;
}

static int usb_cdev_extcon_register(struct usb_port_cooling_dev_info *usb_cdev_info)
{
	int ret;

	if (!of_property_read_bool(usb_cdev_info->dev->of_node, "extcon")) {
		dev_err(usb_cdev_info->dev, "extcon not set\n");
		return -EINVAL;
	}

	usb_cdev_info->edev = extcon_get_edev_by_phandle(usb_cdev_info->dev, 0);
	if (IS_ERR_OR_NULL(usb_cdev_info->edev)) {
		dev_err(usb_cdev_info->dev, "couldn't get extcon\n");
		return -EPROBE_DEFER;
	}

	usb_cdev_info->connected_nb.notifier_call = usb_connected_notifier;
	ret = extcon_register_notifier(usb_cdev_info->edev, EXTCON_MECHANICAL,
				       &usb_cdev_info->connected_nb);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "couldn't register notifier for EXTCON\n");
		return ret;
	}

	return ret;
}

static int usb_get_cur_state(struct thermal_cooling_device *cooling_dev, unsigned long *state)
{
	struct usb_port_cooling_dev_info *usb_cdev_info = cooling_dev->devdata;

	if (!usb_cdev_info)
		return -EINVAL;

	*state = usb_cdev_info->throttle_state;

	return 0;
}

static int usb_get_max_state(struct thermal_cooling_device *cooling_dev, unsigned long *state)
{
	struct usb_port_cooling_dev_info *usb_cdev_info = cooling_dev->devdata;

	if (!usb_cdev_info)
		return -EINVAL;

	*state = USB_MAX_THROTTLE_STATE;

	return 0;
}

static int usb_set_cur_state(struct thermal_cooling_device *cooling_dev, unsigned long state)
{
	struct usb_port_cooling_dev_info *usb_cdev_info = cooling_dev->devdata;
	unsigned long current_state;

	if (!usb_cdev_info)
		return -EINVAL;

	if (state > USB_MAX_THROTTLE_STATE)
		return -EINVAL;

	current_state = usb_cdev_info->throttle_state;
	usb_cdev_info->throttle_state = state;

	if (current_state != state) {
		dev_warn(usb_cdev_info->dev, "usb port cooling dev throttle state=%lu\n",
			 state);
		usb_cdev_changed(usb_cdev_info);
	}
	return 0;
}

static enum alarmtimer_restart unplug_alarm_handler(struct alarm *alarm, ktime_t time)
{
	struct usb_port_cooling_dev_info *usb_cdev_info =
		container_of(alarm, struct usb_port_cooling_dev_info, unplug_alarm);

	usb_cdev_changed(usb_cdev_info);

	return ALARMTIMER_NORESTART;
}

static const struct thermal_cooling_device_ops usb_cdev_ops = {
	.get_max_state = usb_get_max_state,
	.get_cur_state = usb_get_cur_state,
	.set_cur_state = usb_set_cur_state,
};

static int usb_cdev_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct usb_port_cooling_dev_info *usb_cdev_info;
	struct gvotable_election *usb_icl_votable;
	struct gvotable_election *toggle_disable_votable;
	struct gvotable_election *usb_throttle_votable;

	usb_icl_votable = gvotable_election_get_handle(USB_ICL_EL);
	if (IS_ERR_OR_NULL(usb_icl_votable)) {
		dev_err(&pdev->dev, "Couldn't find USB_ICL votable:%ld\n",
			PTR_ERR(usb_icl_votable));
		return -EPROBE_DEFER;
	}

	toggle_disable_votable = gvotable_election_get_handle(TOGGLE_DISABLE_VOTABLE);
	if (IS_ERR_OR_NULL(toggle_disable_votable)) {
		dev_err(&pdev->dev, "Couldn't find DISABLE_POWER_ROLE_SWITCH votable:%ld\n",
			PTR_ERR(toggle_disable_votable));
		return -EPROBE_DEFER;
	}

	usb_throttle_votable = gvotable_election_get_handle(USB_THROTTLE_VOTABLE);
	if (IS_ERR_OR_NULL(usb_throttle_votable)) {
		dev_err(&pdev->dev, "Couldn't find USB_THROTTLE votable:%ld\n",
			PTR_ERR(usb_throttle_votable));
		return -EPROBE_DEFER;
	}

	usb_cdev_info = devm_kzalloc(&pdev->dev, sizeof(*usb_cdev_info), GFP_KERNEL);
	if (!usb_cdev_info)
		return -ENOMEM;

	get_device(&pdev->dev);
	usb_cdev_info->dev = &pdev->dev;
	usb_cdev_info->usb_icl_votable = usb_icl_votable;
	usb_cdev_info->toggle_disable_votable = toggle_disable_votable;
	usb_cdev_info->usb_throttle_votable = usb_throttle_votable;

	ret = get_dts_vars(usb_cdev_info);
	if (ret < 0)
		goto put_dev;

	platform_set_drvdata(pdev, usb_cdev_info);

	usb_cdev_info->wq = create_singlethread_workqueue(dev_name(usb_cdev_info->dev));
	if (!usb_cdev_info->wq) {
		dev_err(usb_cdev_info->dev, "workqueue creation failed\n");
		ret = -ENOMEM;
		goto put_dev;
	}

	alarm_init(&usb_cdev_info->unplug_alarm, ALARM_BOOTTIME, unplug_alarm_handler);
	init_completion(&usb_cdev_info->connected_completion);

	ret = device_init_wakeup(usb_cdev_info->dev, true);
	if (ret < 0) {
		dev_err(usb_cdev_info->dev, "Cannot init wakeup, ret=%d\n", ret);
		goto destroy_wq;
	}

	ret = usb_cdev_extcon_register(usb_cdev_info);
	if (ret < 0)
		goto destroy_wq;

	/* Register cooling device */
	usb_cdev_info->cooling_dev =
		thermal_of_cooling_device_register(dev_of_node(usb_cdev_info->dev), "usbc-port",
						   usb_cdev_info, &usb_cdev_ops);

	if (IS_ERR(usb_cdev_info->cooling_dev)) {
		ret = PTR_ERR(usb_cdev_info->cooling_dev);
		dev_err(usb_cdev_info->dev, "failed to register cooling device: %d\n", ret);
		goto unreg_notifier;
	}

	return 0;

unreg_notifier:
	extcon_unregister_notifier(usb_cdev_info->edev, EXTCON_MECHANICAL,
				   &usb_cdev_info->connected_nb);
destroy_wq:
	destroy_workqueue(usb_cdev_info->wq);
put_dev:
	put_device(&pdev->dev);

	return ret;
}

static int usb_cdev_remove(struct platform_device *pdev)
{
	struct usb_port_cooling_dev_info *usb_cdev_info = platform_get_drvdata(pdev);

	thermal_cooling_device_unregister(usb_cdev_info->cooling_dev);
	extcon_unregister_notifier(usb_cdev_info->edev, EXTCON_MECHANICAL,
				   &usb_cdev_info->connected_nb);
	put_device(&pdev->dev);

	return 0;
}

static const struct of_device_id match_table[] = {
	{
		.compatible = "google,usbc_port_cooling_dev",
	},
	{},
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver usb_cdev_driver = {
	.driver = {
		.name = "google,usbc_port_cooling_dev",
		.owner = THIS_MODULE,
		.of_match_table = match_table,
		.dev_groups = usb_cdev_groups,
	},
	.probe = usb_cdev_probe,
	.remove = usb_cdev_remove,
};

module_platform_driver(usb_cdev_driver);
MODULE_DESCRIPTION("USB port cooling device driver");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL v2");
