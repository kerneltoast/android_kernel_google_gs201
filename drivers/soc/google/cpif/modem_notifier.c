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
#include <linux/modem_notifier.h>

#include "modem_prj.h"
#include "modem_utils.h"

static struct raw_notifier_head modem_event_notifier;
#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
static struct raw_notifier_head modem_voice_call_event_notifier;
#endif

int register_modem_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;

	return raw_notifier_chain_register(&modem_event_notifier, nb);
}
EXPORT_SYMBOL(register_modem_event_notifier);

void modem_notify_event(enum modem_event evt, void *mc)
{
	mif_info("event notify (%d)\n", evt);
	raw_notifier_call_chain(&modem_event_notifier, evt, mc);
}
EXPORT_SYMBOL(modem_notify_event);

#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
int register_modem_voice_call_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;
	return raw_notifier_chain_register(&modem_voice_call_event_notifier, nb);
}

void modem_voice_call_notify_event(enum modem_voice_call_event evt, void *data)
{
	mif_err("voice call event notify (%d) ++\n", evt);
	raw_notifier_call_chain(&modem_voice_call_event_notifier, evt, data);
	mif_err("voice call event notify (%d) --\n", evt);
}
EXPORT_SYMBOL(modem_voice_call_notify_event);
#endif
