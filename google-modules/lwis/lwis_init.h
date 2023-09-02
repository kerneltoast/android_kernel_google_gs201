/*
 * Google LWIS Base Device Driver - Init Function Declarations
 *
 * Copyright (c) 2019 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_INIT_H_
#define LWIS_INIT_H_

/* lwis_device_ioreg.c */
int lwis_ioreg_device_init(void);

/* lwis_device_top.c */
int lwis_top_device_init(void);

/* lwis_device_i2c.c */
int lwis_i2c_device_init(void);

/* lwis_device_slc.c */
int lwis_slc_device_init(void);

/* lwis_device_dpm.c */
int lwis_dpm_device_init(void);

#endif // LWIS_INIT_H_
