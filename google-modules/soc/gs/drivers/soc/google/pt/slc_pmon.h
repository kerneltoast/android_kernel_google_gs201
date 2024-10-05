/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * slc_pmon.h
 *
 * PMON API for SLC counter management.
 *
 * Copyright 2020 Google LLC
 *
 * Author: paillon@google.com
 */

#ifndef __GOOGLE_SLC_PMON_H__
#define __GOOGLE_SLC_PMON_H__

struct slc_acpm_driver_data;

#if IS_ENABLED(CONFIG_SLC_PMON)

/*
 * Driver entry point: allocates SLC PMON structures and registers
 * perf driver.
 */
extern int slc_pmon_init(struct slc_acpm_driver_data *driver_data,
			 int (*slc_acpm)(struct slc_acpm_driver_data *,
					 unsigned int, unsigned int,
					 unsigned long, uint32_t *));

/*
 * Driver exit point: releases all allocated resources before unregistering.
 */
extern void slc_pmon_exit(void);

#else

static inline int slc_pmon_init(struct slc_acpm_driver_data *driver_data,
				int (*slc_acpm)(struct slc_acpm_driver_data *,
						unsigned int, unsigned int,
						unsigned long, uint32_t *))
{
	return 0;
}
static inline void slc_pmon_exit(void)
{
}

#endif

#endif // __GOOGLE_SLC_PMON_H__
