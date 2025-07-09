/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022, Samsung Electronics.
 *
 */

#ifndef __GNSS_UTILS_H__
#define __GNSS_UTILS_H__

#include "gnss_prj.h"

static inline struct wakeup_source *gnssif_wlock_register(struct device *dev,
		const char *name)
{
	struct wakeup_source *ws = NULL;

	ws = wakeup_source_register(dev, name);
	if (ws == NULL) {
		gif_err("%s: wakelock register fail\n", name);
		return NULL;
	}

	return ws;
}

static inline void gnssif_wlock_unregister(struct wakeup_source *ws)
{
	if (ws == NULL) {
		gif_err("wakelock unregister fail\n");
		return;
	}

	wakeup_source_unregister(ws);
}

static inline void gnssif_wlock_lock(struct wakeup_source *ws)
{
	if (ws == NULL) {
		gif_err("wakelock fail\n");
		return;
	}

	__pm_stay_awake(ws);
}

static inline void gnssif_wlock_lock_timeout(struct wakeup_source *ws, long timeout)
{
	if (ws == NULL) {
		gif_err("wakelock timeout fail\n");
		return;
	}

	__pm_wakeup_event(ws, jiffies_to_msecs(timeout));
}

static inline void gnssif_wlock_unlock(struct wakeup_source *ws)
{
	if (ws == NULL) {
		gif_err("wake unlock fail\n");
		return;
	}

	__pm_relax(ws);
}

static inline int gnssif_wlock_active(struct wakeup_source *ws)
{
	if (ws == NULL) {
		gif_err("wakelock active fail\n");
		return 0;
	}

	return ws->active;
}

/* gnss irq */
void gif_configure_irq(struct gnss_irq *irq, unsigned int num, const char *name,
			unsigned long flags);
int gif_request_irq(struct gnss_irq *irq, irq_handler_t isr, void *data);
void gif_enable_irq(struct gnss_irq *irq);
void gif_disable_irq_nosync(struct gnss_irq *irq);
void gif_disable_irq_sync(struct gnss_irq *irq);

#endif /* end of __GNSS_UTILS_H__ */
