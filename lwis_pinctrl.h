/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Pinctrl Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_PINCTRL_H_
#define LWIS_PINCTRL_H_

#include <linux/pinctrl/consumer.h>

/*
 * lwis_pinctrl_set_state: Set pinctrl state to the provided state string.
 */
int lwis_pinctrl_set_state(struct pinctrl *pc, char *state_str);

#endif /* LWIS_PINCTRL_H_ */
