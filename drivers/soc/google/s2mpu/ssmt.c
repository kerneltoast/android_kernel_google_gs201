// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform device driver for S2MPU.
 *
 * Copyright (C) 2020 Google LLC.
 */
#include <linux/io.h>

#include "s2mpu-lib.h"
#include "ssmt.h"

#define SSMT_NS_READ_STREAM_VID(base, sid)	((base) + (0x1000u + (0x4U * (sid))))
#define SSMT_NS_WRITE_STREAM_VID(base, sid)	((base) + (0x1200u + (0x4U * (sid))))

void ssmt_set_vid(struct s2mpu_info *info)
{
	unsigned int i;

	for (i = 0; i < info->sidcount; i++) {
		__raw_writel(info->vid, SSMT_NS_READ_STREAM_VID(info->ssmt_base,
								info->sids[i]));
		__raw_writel(info->vid, SSMT_NS_WRITE_STREAM_VID(info->ssmt_base,
								 info->sids[i]));
	}

	/* make sure all register writes are complete before we return from
	 * this function.
	 */
	wmb();
}
