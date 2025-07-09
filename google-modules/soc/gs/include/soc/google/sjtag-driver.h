/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Copyright (c) 2021 Google LLC
 *    Author: Peter Csaszar <pcsaszar@google.com>
 */

#ifndef INCLUDE_SOC_GOOGLE_SJTAG_DRIVER_H_
#define INCLUDE_SOC_GOOGLE_SJTAG_DRIVER_H_

#if IS_ENABLED(CONFIG_GS_SJTAG)
extern int sjtag_is_locked(void);
#else /* !IS_ENABLED(CONFIG_GS_SJTAG) */
static inline int sjtag_is_locked(void)
{
	return 0;
}
#endif
#endif /* INCLUDE_SOC_GOOGLE_SJTAG_DRIVER_H_ */
