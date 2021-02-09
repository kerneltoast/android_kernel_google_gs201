// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#include "dit_common.h"
#include "dit_2_2_0.h"

static struct dit_ctrl_t *dc;

int dit_get_reg_version(u32 *version)
{
	*version = READ_REG_VALUE(dc, DIT_REG_VERSION);

	return 0;
}

int dit_ver_create(struct dit_ctrl_t *dc_ptr)
{
	dc = dc_ptr;

	dc->get_reg_version = dit_get_reg_version;

	return 0;
}

