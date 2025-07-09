// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 Samsung Electronics.
 * Copyright 2024 Google LLC
 */

#include <linux/module.h>
#include <linux/notifier.h>
#include <soc/google/modem_notifier.h>

static RAW_NOTIFIER_HEAD(modem_event_notifier);
static RAW_NOTIFIER_HEAD(modem_voice_call_event_notifier);
static RAW_NOTIFIER_HEAD(modem_force_crash_notifier);

int register_modem_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;

	return raw_notifier_chain_register(&modem_event_notifier, nb);
}
EXPORT_SYMBOL(register_modem_event_notifier);

void modem_notify_event(enum modem_event evt, void *mc)
{
	/* ToDo */
}
EXPORT_SYMBOL(modem_notify_event);

int register_modem_voice_call_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;
	return raw_notifier_chain_register(&modem_voice_call_event_notifier, nb);
}
EXPORT_SYMBOL(register_modem_voice_call_event_notifier);

void unregister_modem_voice_call_event_notifier(struct notifier_block *nb)
{
	if (nb)
		raw_notifier_chain_unregister(&modem_voice_call_event_notifier, nb);
}
EXPORT_SYMBOL(unregister_modem_voice_call_event_notifier);

void modem_voice_call_notify_event(enum modem_voice_call_event evt, void *data)
{
	raw_notifier_call_chain(&modem_voice_call_event_notifier, evt, data);
}
EXPORT_SYMBOL(modem_voice_call_notify_event);

int register_modem_force_crash_handler(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;
	return raw_notifier_chain_register(&modem_force_crash_notifier, nb);
}
EXPORT_SYMBOL(register_modem_force_crash_handler);

void unregister_modem_force_crash_handler(struct notifier_block *nb)
{
	if (nb)
		raw_notifier_chain_unregister(&modem_force_crash_notifier, nb);
}
EXPORT_SYMBOL(unregister_modem_force_crash_handler);

int modem_force_crash_exit_ext(const char *buf)
{
	return raw_notifier_call_chain(&modem_force_crash_notifier, 0, (void *)buf);
}
EXPORT_SYMBOL(modem_force_crash_exit_ext);

MODULE_LICENSE("GPL");
