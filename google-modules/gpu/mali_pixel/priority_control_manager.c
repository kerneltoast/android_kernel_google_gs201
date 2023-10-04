// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

#include <linux/fs.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/priority_control_manager.h>

/**
 * enum priorities - The different priorities available on the GPU
 */
enum priorities {
	/**
	 * @PRIORITY_RT: Realtime priority
	 */
	PRIORITY_RT = 0,
	/**
	 * @PRIORITY_HIGH: High priority
	 */
	PRIORITY_HIGH,
	/**
	 * @PRIORITY_MED: Medium priority
	 */
	PRIORITY_MED,
	/**
	 * @PRIORITY_LOW: Low priority
	 */
	PRIORITY_LOW,
	/**
	 * @PRIORITY_COUNT: The number of priority classes
	 */
	PRIORITY_COUNT,
};

static const char* priority_name[PRIORITY_COUNT] = {
	"realtime",
	"high",
	"medium",
	"low",
};

/*
 * TODO(b/182907924) Using permissive mode until we have a mechanism to validate priority requests
 * in place.
 */
#define PERMISSIVE_MODE (1)

/**
 * PRIORITY_DEFAULT - The default priority that applications will be set to.
 */
#define PRIORITY_DEFAULT (PRIORITY_MED)

/**
 * pcm_scheduler_priority_check() - Checks in incoming priority request
 *
 * This function returns which priority a context should run at, taking into
 * consideration what the context is requesting.
 *
 * @pcm_dev:            Pointer to the priority control manager.
 * @task:               The task struct of the process requesting the priority
 *                      check.
 * @requested_priority: The priority that the context is requesting.
 *
 * Return: The priority that should be granted to the context.
 */
static int pcm_scheduler_priority_check(struct priority_control_manager_device *pcm_dev,
	struct task_struct *task, int requested_priority)
{
	int ret;
	struct device *dev = pcm_dev->data;
	kuid_t uid = task->cred->uid;

	switch (requested_priority)
	{

	/* For low priority requests, we don't apply any restrictions */
	case PRIORITY_LOW:
	case PRIORITY_MED:
		ret = requested_priority;
		dev_dbg(dev, "UID %d request for %s priority was granted\n",
			__kuid_val(uid), priority_name[requested_priority]);
		break;

	/* Request is for one of the restricted priorities */
	case PRIORITY_HIGH:
	case PRIORITY_RT:
		if (PERMISSIVE_MODE) {
			ret = requested_priority;
			dev_dbg(dev, "UID %d request for %s priority was granted\n",
				__kuid_val(uid), priority_name[requested_priority]);
		} else {
			ret = PRIORITY_DEFAULT;
			dev_warn(dev,
				"UID %d request for %s priority was denied, granted %s instead\n",
				__kuid_val(uid), priority_name[requested_priority],
				priority_name[ret]);
		}
		break;
	default:
		ret = PRIORITY_DEFAULT;
		dev_warn(dev, "UID %d requested an invalid priority (ID: %d), granted %s instead\n",
			__kuid_val(uid), requested_priority, priority_name[ret]);
	}

	return ret;
}

static int priority_control_manager_probe(struct platform_device *pdev)
{
	struct priority_control_manager_device *pcm_dev;

	pcm_dev = kzalloc(sizeof(*pcm_dev), GFP_KERNEL);
	if (!pcm_dev)
		return -ENOMEM;

	pcm_dev->ops.pcm_scheduler_priority_check = pcm_scheduler_priority_check;
	pcm_dev->data = &pdev->dev;

	platform_set_drvdata(pdev, pcm_dev);
	dev_info(&pdev->dev, "Priority control manager probed successfully\n");

	return 0;
}

static int priority_control_manager_remove(struct platform_device *pdev)
{
	struct priority_control_manager_device *pcm_dev = platform_get_drvdata(pdev);

	dev_info(pcm_dev->data, "Priority control manager removed successfully\n");

	return 0;
}

static const struct of_device_id priority_control_manager_dt_ids[] = {
	{ .compatible = "arm,priority-control-manager" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, priority_control_manager_dt_ids);

struct platform_driver priority_control_manager_driver = {
	.probe = priority_control_manager_probe,
	.remove = priority_control_manager_remove,
	.driver = {
		.name = "mali-pcm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(priority_control_manager_dt_ids),
		.suppress_bind_attrs = true,
	}
};
