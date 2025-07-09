/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015-2019, Samsung Electronics.
 * Copyright 2024 Google LLC
 */

#ifndef __GOOGLE_MODEM_NOTIFIER_H__
#define __GOOGLE_MODEM_NOTIFIER_H__

struct notifier_block;

/*
 * Note: these values align with 'enum modem_state' in some Samsung modem
 * drivers.
 */
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

#if IS_ENABLED(CONFIG_GOOGLE_MODEMCTL)

extern int modem_force_crash_exit_ext(const char *buf);
extern int register_modem_force_crash_handler(struct notifier_block *nb);
extern void unregister_modem_force_crash_handler(struct notifier_block *nb);

extern int register_modem_event_notifier(struct notifier_block *nb);
extern void modem_notify_event(enum modem_event evt, void *mc);

extern int register_modem_voice_call_event_notifier(struct notifier_block *nb);
extern void unregister_modem_voice_call_event_notifier(struct notifier_block *nb);
extern void modem_voice_call_notify_event(enum modem_voice_call_event evt, void *data);

#else

static inline int modem_force_crash_exit_ext(const char *buf) { return 0; }
static inline int void register_modem_force_crash_handler(struct notifier_block *nb) {}
static inline void unregister_modem_force_crash_handler(struct notifier_block *nb) {}

static inline int register_modem_event_notifier(struct notifier_block *nb)
{
	return 0;
}
static inline void modem_notify_event(enum modem_event evt, void *mc) {}

static inline int register_modem_voice_call_event_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline void unregister_modem_voice_call_event_notifier(struct notifier_block *nb)
{
}
static inline void modem_voice_call_notify_event(enum modem_voice_call_event evt, void *data) {}
#endif

#endif /* __GOOGLE_MODEM_NOTIFIER_H__ */
