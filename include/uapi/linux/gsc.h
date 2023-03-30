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
	__u64 buf;
	__u32 len;
	__u32 command;
};

struct gsc_ioc_nos_call_req {
	__u8 app_id;
	__u8 reserved;
	__u16 params;
	__u32 arg_len;
	__u64 buf;
	__u32 reply_len;
	__u32 call_status;
};

#define GSC_IOC_TPM_DATAGRAM	_IOW(GSC_IOC_MAGIC, 1, \
				     struct gsc_ioc_tpm_datagram)
#define GSC_IOC_RESET		_IO(GSC_IOC_MAGIC, 2)
#define GSC_IOC_GSA_NOS_CALL	_IOW(GSC_IOC_MAGIC, 3, \
				     struct gsc_ioc_nos_call_req)

#endif /* GSC_H */
