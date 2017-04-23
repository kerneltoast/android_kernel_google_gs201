/*
 * linux/drivers/gpu/exynos/g2d/g2d_uapi.h
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * Samsung Graphics 2D driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _G2D_UAPI_H_
#define _G2D_UAPI_H_

/*
 * struct g2d_task - description of a layer processing task
 * @version: the version of the format of struct g2d_task_data
 */
struct g2d_task_data {
	__u32			version;
};

#define G2D_IOC_PROCESS		_IOWR('M', 4, struct g2d_task_data)

#endif /* _G2D_UAPI_H_ */


