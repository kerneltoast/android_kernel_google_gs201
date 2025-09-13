// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Pinctrl Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-pin: " fmt

#include "lwis_pinctrl.h"

#include <linux/kernel.h>

int lwis_pinctrl_set_state(struct pinctrl *pc, char *state_str)
{
	int ret;
	struct pinctrl_state *state;

	if (!pc) {
		pr_err("Cannot find pinctrl instance\n");
		return -ENODEV;
	}

	state = pinctrl_lookup_state(pc, state_str);
	if (IS_ERR_OR_NULL(state)) {
		pr_err("Cannot find mclk state %s\n", state_str);
		return PTR_ERR(state);
	}

	ret = pinctrl_select_state(pc, state);
	if (ret) {
		pr_err("Cannot select state %s\n", state_str);
		return ret;
	}

	return 0;
}
