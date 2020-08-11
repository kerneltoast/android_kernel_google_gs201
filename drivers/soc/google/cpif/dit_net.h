/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#ifndef __DIT_NET_H__
#define __DIT_NET_H__

#define DIT_NET_DEV_NAME "dit%d"

struct dit_net_priv {
	struct dit_ctrl_t *dc;
};

int dit_net_init(struct dit_ctrl_t *dc);

#endif /* __DIT_NET_H__ */

