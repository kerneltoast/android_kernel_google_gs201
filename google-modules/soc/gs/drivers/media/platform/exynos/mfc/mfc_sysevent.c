/*
 * drivers/media/platform/exynos/mfc/mfc_sysevent.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#if IS_ENABLED(CONFIG_EXYNOS_SYSTEM_EVENT)

#include <linux/platform_device.h>

#include "mfc_sysevent.h"
#include "mfc_common.h"

int mfc_sysevent_powerup(const struct sysevent_desc *desc)
{
	struct mfc_core *core = (struct mfc_core *)desc->dev->driver_data;

	mfc_core_info("powerup callback\n");

	return 0;
}

int mfc_sysevent_shutdown(const struct sysevent_desc *desc, bool force_stop)
{
	struct mfc_core *core = (struct mfc_core *)desc->dev->driver_data;

	mfc_core_info("shutdown callback\n");

	return 0;
}

void mfc_sysevent_crash_shutdown(const struct sysevent_desc *desc)
{
	struct mfc_core *core = (struct mfc_core *)desc->dev->driver_data;

	mfc_core_info("crash callback\n");
}

static int mfc_core_sysevent_notifier_cb(struct notifier_block *this, unsigned long code,
								void *_cmd)
{
	struct notif_data *notifdata = NULL;
	notifdata = (struct notif_data *) _cmd;

	switch (code) {
	case SYSTEM_EVENT_BEFORE_SHUTDOWN:
		pr_info("%s: %s: %s\n", __func__, notifdata->pdev->name,
				__stringify(SYSTEM_EVENT_BEFORE_SHUTDOWN));
		break;
	case SYSTEM_EVENT_AFTER_SHUTDOWN:
		pr_info("%s: %s: %s\n", __func__, notifdata->pdev->name,
				__stringify(SYSTEM_EVENT_AFTER_SHUTDOWN));
		break;
	case SYSTEM_EVENT_RAMDUMP_NOTIFICATION:
		pr_info("%s: %s: %s\n", __func__, notifdata->pdev->name,
				__stringify(SYSTEM_EVENT_RAMDUMP_NOTIFICATION));
		break;
	case SYSTEM_EVENT_BEFORE_POWERUP:
		if (_cmd) {
			notifdata = (struct notif_data *) _cmd;
			pr_info("%s: %s: %s, crash_status:%d, enable_ramdump:%d\n",
				__func__, notifdata->pdev->name,
				__stringify(SYSTEM_EVENT_BEFORE_POWERUP),
				notifdata->crashed, notifdata->enable_ramdump);
		} else {
			pr_info("%s: %s: %s\n", __func__,
				notifdata->pdev->name,
				__stringify(SYSTEM_EVENT_BEFORE_POWERUP));
		}
		break;
	case SYSTEM_EVENT_AFTER_POWERUP:
		pr_info("%s: %s: %s\n", __func__, notifdata->pdev->name,
				__stringify(SYSTEM_EVENT_AFTER_POWERUP));
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

struct notifier_block mfc_core_nb = {
	.notifier_call = mfc_core_sysevent_notifier_cb,
};
#endif
