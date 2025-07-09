// SPDX-License-Identifier: GPL-2.0-only
/* init_metrics.c
 *
 * Support for init metrics
 * Copyright 2022 Google LLC
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <soc/google/cpuidle_metrics.h>

extern int thermal_metrics_init(struct kobject *metrics_kobj);
extern int perf_metrics_init(struct kobject *metrics_kobj);

struct kobject *metrics_kobj;

static int __init metrics_init(void)
{
	metrics_kobj = kobject_create_and_add("metrics", kernel_kobj);
	thermal_metrics_init(metrics_kobj);
	perf_metrics_init(metrics_kobj);
	cpuidle_metrics_init(metrics_kobj);
	return 0;
}
EXPORT_SYMBOL_GPL(metrics_kobj);
module_init(metrics_init);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ziyi Cui <ziyic@google.com>");
