/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * include/linux/gsc.h
 *
 * Copyright (C) 2020 Google Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef GSC_H
#define GSC_H

#include <linux/types.h>

#define GSC_IOC_MAGIC		'c'

struct gsc_ioc_tpm_datagram {
	u64 buf;
	u32 len;
	u32 command;
};

#define GSC_IOC_TPM_DATAGRAM	_IOW(GSC_IOC_MAGIC, 1, \
				     struct gsc_ioc_tpm_datagram)
#define GSC_IOC_RESET		_IO(GSC_IOC_MAGIC, 2)

#endif /* GSC_H */
