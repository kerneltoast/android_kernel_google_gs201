/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2015 Samsung Electronics.
 *
 */

#ifndef __MODEM_NOTIFIER_H__
#define __MODEM_NOTIFIER_H__

/* refer to enum modem_state  */
enum modem_event {
	MODEM_EVENT_RESET	= 1,
	MODEM_EVENT_EXIT,
	MODEM_EVENT_ONLINE	= 4,
	MODEM_EVENT_OFFLINE	= 5,
	MODEM_EVENT_WATCHDOG	= 9,
};

enum modem_voice_call_event {
	MODEM_VOICE_CALL_OFF	= 0,
	MODEM_VOICE_CALL_ON	= 1,
};

void modem_notify_event(enum modem_event evt, struct modem_ctl *mc);

#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
void modem_voice_call_notify_event(enum modem_voice_call_event evt);
#endif

#endif/*__MODEM_NOTIFIER_H__*/
