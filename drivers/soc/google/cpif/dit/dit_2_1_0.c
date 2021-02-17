// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#include "dit_common.h"

static struct dit_ctrl_t *dc;

static int dit_do_suspend(void)
{
	int ret;

	ret = dit_reg_backup_restore(true);
	if (ret) {
		mif_err("reg backup failed ret:%d\n", ret);
		return ret;
	}

	ret = dit_init(NULL, DIT_INIT_DEINIT);
	if (ret) {
		mif_err("deinit failed ret:%d\n", ret);
		return ret;
	}

	return 0;
}

static int dit_do_resume(void)
{
	unsigned int dir;
	int ret;

	ret = dit_init(NULL, DIT_INIT_NORMAL);
	if (ret) {
		mif_err("init failed ret:%d\n", ret);
		for (dir = 0; dir < DIT_DIR_MAX; dir++) {
			if (dit_is_busy(dir))
				mif_err("busy (dir:%d)\n", dir);
		}
		return ret;
	}

	ret = dit_reg_backup_restore(false);
	if (ret) {
		mif_err("reg restore failed ret:%d\n", ret);
		return ret;
	}

	return 0;
}

int dit_ver_create(struct dit_ctrl_t *dc_ptr)
{
	if (unlikely(!dc_ptr))
		return -EPERM;

	dc = dc_ptr;

	dc->do_suspend = dit_do_suspend;
	dc->do_resume = dit_do_resume;

	return 0;
}

