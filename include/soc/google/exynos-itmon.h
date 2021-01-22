/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#ifndef EXYNOS_ITMON__H
#define EXYNOS_ITMON__H

struct itmon_notifier {
	char *port;			/* The block to which the client IP belongs */
	char *client;			/* The client's name which problem occurred */
	char *dest;			/* The destination which the client tried to access */
	bool read;			/* Transaction Type */
	unsigned long target_addr;	/* The physical address which the master tried to access */
	unsigned int errcode;		/* The error code which the problem occurred */
	bool onoff;			/* Target Block on/off */
	char *pd_name;			/* Target Block power domain name */
};

#define M_NODE			(0)
#define T_S_NODE		(1)
#define T_M_NODE		(2)
#define S_NODE			(3)
#define NODE_TYPE		(4)

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
extern void itmon_notifier_chain_register(struct notifier_block *n);
extern void itmon_notifier_chain_unregister(struct notifier_block *n);
extern void itmon_enable(bool enabled);
#else
#define itmon_notifier_chain_register(x)		do { } while (0)
#define itmon_notifier_chain_unregister(x)		do { } while (0)
#define itmon_enable(x)					do { } while (0)
#endif
#endif
