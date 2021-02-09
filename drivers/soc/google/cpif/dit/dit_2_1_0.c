// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#include "dit_common.h"
#include "dit_2_1_0.h"

static struct dit_ctrl_t *dc;

int dit_ver_create(struct dit_ctrl_t *dc_ptr)
{
	dc = dc_ptr;

	return 0;
}

