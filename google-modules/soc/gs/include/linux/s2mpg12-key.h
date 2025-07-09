/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * s2mpg12-key.h
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * header file including key information of s2mpg12
 */

#ifndef _POWER_KEYS_H
#define _POWER_KEYS_H

struct device;

struct power_keys_button {
	/* Configuration parameters */
	unsigned int code;	/* input event code (KEY_*, SW_*) */
	int gpio;		/* -1 if this key does not support gpio */
	int active_low;
	const char *desc;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
	int wakeup;
	int always_wakeup;	/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	bool can_disable;
	int value;		/* axis value for EV_ABS */
	unsigned int irq;	/* Irq number in case of interrupt keys */
};

struct power_keys_platform_data {
	struct power_keys_button *buttons;
	int nbuttons;
	unsigned int poll_interval;	/* polling interval in msecs for polling driver only */
	unsigned int rep:1;		/* enable input subsystem auto repeat */

	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;		/* input device name */
};

#endif
