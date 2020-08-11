/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 Samsung Electronics.
 *
 */

#ifndef __MODEM_DUMP_H__
#define __MODEM_DUMP_H__

#include "modem_prj.h"
#include "link_device_memory.h"

extern int cp_get_log_dump(struct io_device *iod, struct link_device *ld, unsigned long arg);

#endif
