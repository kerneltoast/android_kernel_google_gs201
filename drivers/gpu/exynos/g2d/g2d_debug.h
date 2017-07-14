/*
 * linux/drivers/gpu/exynos/g2d/g2d_debug.h
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * Contact: Hyesoo Yu <hyesoo.yu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __EXYNOS_G2D_DEBUG_H_
#define __EXYNOS_G2D_DEBUG_H_

struct regs_info {
	int start;
	int size;
	const char *name;
};

enum g2d_stamp_id {
	G2D_STAMP_STATE_ALLOC,
	G2D_STAMP_STATE_BEGIN,
	G2D_STAMP_STATE_PM_RESUME,
	G2D_STAMP_STATE_PUSH,
	G2D_STAMP_STATE_DONE,
	G2D_STAMP_STATE_PM_SUSPEND,
	G2D_STAMP_STATE_FREE,
	G2D_STAMP_STATE_TIMEOUT_FENCE,
	G2D_STAMP_STATE_TIMEOUT_HW,
	G2D_STAMP_STATE_ERR_INT,
	G2D_STAMP_STATE_MMUFAULT,
	G2D_STAMP_STATE_SHUTDOWN_S,
	G2D_STAMP_STATE_SHUTDOWN_E,
	G2D_STAMP_STATE_SUSPEND_S,
	G2D_STAMP_STATE_SUSPEND_E,
	G2D_STAMP_STATE_RESUME_S,
	G2D_STAMP_STATE_RESUME_E,
	G2D_STAMP_STATE_HWFCBUF,
};

void g2d_init_debug(struct g2d_device *dev);
void g2d_destroy_debug(struct g2d_device *dev);
void g2d_stamp_task(struct g2d_task *task, u32 val);
void g2d_dump_task(struct g2d_task *task);
#endif /* __EXYNOS_G2D_HELPER_H_ */
