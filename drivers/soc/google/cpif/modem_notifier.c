// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 Samsung Electronics.
 *
 */

#include <stdarg.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/notifier.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_notifier.h"

void modem_notify_event(enum modem_event evt, struct modem_ctl *mc)
{
	/* ToDo */
}
EXPORT_SYMBOL(modem_notify_event);

#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
void modem_voice_call_notify_event(enum modem_voice_call_event evt)
{
	/* ToDo */
}
EXPORT_SYMBOL(modem_voice_call_notify_event);
#endif
