/*
 * drivers/media/platform/exynos/mfc/mfc_sysevent.h
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
#include <soc/samsung/sysevent.h>

int mfc_sysevent_powerup(const struct sysevent_desc *desc);
int mfc_sysevent_shutdown(const struct sysevent_desc *desc, bool force_stop);
void mfc_sysevent_crash_shutdown(const struct sysevent_desc *desc);
#else
#define sysevent_get(...)			do {} while (0)
#define sysevent_put(...)			do {} while (0)
#define mfc_sysevent_powerup(...)		do {} while (0)
#define mfc_sysevent_shutdown(...)		do {} while (0)
#define mfc_sysevent_crash_shutdown(...)	do {} while (0)
#endif
