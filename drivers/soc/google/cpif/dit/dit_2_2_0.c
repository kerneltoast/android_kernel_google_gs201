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

static int dit_get_reg_version(u32 *version)
{
	*version = READ_REG_VALUE(dc, DIT_REG_VERSION);

	return 0;
}

int dit_ver_create(struct dit_ctrl_t *dc_ptr)
{
	if (unlikely(!dc_ptr))
		return -EPERM;

	dc = dc_ptr;

	dc->get_reg_version = dit_get_reg_version;
	dc->do_suspend = dit_dummy;
	dc->do_resume = dit_dummy;

	return 0;
}

