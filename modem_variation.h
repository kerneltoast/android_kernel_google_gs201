/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#ifndef __MODEM_VARIATION_H__
#define __MODEM_VARIATION_H__

#include "modem_prj.h"

#define DECLARE_MODEM_INIT(type)					\
	int type ## _init_modemctl_device(				\
				struct modem_ctl *mc,			\
				struct modem_data *pdata)

#define DECLARE_MODEM_UNINIT(type)					\
	void type ## _uninit_modemctl_device(				\
				struct modem_ctl *mc,			\
				struct modem_data *pdata)


#define DECLARE_MODEM_INIT_DUMMY(type)					\
	int type ## _init_modemctl_device(				\
				struct modem_ctl *mc,			\
				struct modem_data *pdata)		\
	{ return 0; }

#define DECLARE_MODEM_UNINIT_DUMMY(type)				\
	void type ## _uninit_modemctl_device(				\
				struct modem_ctl *mc,			\
				struct modem_data *pdata)		\
	{ return; }


#define DECLARE_LINK_INIT()						\
	struct link_device *create_link_device(				\
				struct platform_device *pdev,		\
				u32 link_type)

#define DECLARE_LINK_INIT_DUMMY()					\
	struct link_device *dummy_create_link_device(			\
				struct platform_device *pdev,		\
				u32 link_type)				\
	{ return NULL; }

#define MODEM_INIT_CALL(type)	type ## _init_modemctl_device

#define MODEM_UNINIT_CALL(type) type ## _uninit_modemctl_device

#define LINK_INIT_CALL()	create_link_device
#define LINK_INIT_CALL_DUMMY()	dummy_create_link_device


/**
 * Add extern declaration of modem & link type
 * (CAUTION!!! Every DUMMY function must be declared in modem_variation.c)
 */

/* modem device support */
#if IS_ENABLED(CONFIG_SEC_MODEM_S5000AP)
DECLARE_MODEM_INIT(s5000ap);
DECLARE_MODEM_UNINIT(s5000ap);
#endif

#if IS_ENABLED(CONFIG_SEC_MODEM_S5100)
DECLARE_MODEM_INIT(s5100);
DECLARE_MODEM_UNINIT(s5100);
#endif

/* link device support */
#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM) || IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
DECLARE_LINK_INIT();
#endif

typedef int (*modem_init_call)(struct modem_ctl *, struct modem_data *);
typedef void (*modem_uninit_call)(struct modem_ctl *, struct modem_data *);
typedef struct link_device *(*link_init_call)(struct platform_device *,
						u32 link_type);

int call_modem_init_func(struct modem_ctl *mc, struct modem_data *pdata);
void call_modem_uninit_func(struct modem_ctl *mc, struct modem_data *pdata);

struct link_device *call_link_init_func(struct platform_device *pdev,
					       u32 link_type);

#endif
