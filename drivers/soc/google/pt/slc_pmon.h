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

#if IS_ENABLED(CONFIG_SLC_PMON)

struct slc_acpm_driver_data;

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

#define slc_pmon_init(driver_data, acpm_callback) (0)
#define slc_pmon_exit()

#endif

#endif // __GOOGLE_SLC_PMON_H__
