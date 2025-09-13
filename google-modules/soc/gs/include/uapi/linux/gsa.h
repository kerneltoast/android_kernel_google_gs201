/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * include/linux/gsa.h
 *
 * Copyright (C) 2023 Google Inc
 */

#ifndef GSA_H
#define GSA_H

#include <linux/types.h>

#define GSA_IOC_MAGIC		'g'

struct gsa_ioc_load_app_req {
	__u64 buf;
	__u32 len;
};

#define GSA_IOC_LOAD_APP	_IOW(GSA_IOC_MAGIC, 1, \
				     struct gsa_ioc_load_app_req)

#endif /* GSA_H */
