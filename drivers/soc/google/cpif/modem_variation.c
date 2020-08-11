// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#include "modem_variation.h"

/* add declaration of modem & link type */
/* modem device support */
DECLARE_MODEM_INIT_DUMMY(dummy)

#if !IS_ENABLED(CONFIG_SEC_MODEM_S5000AP)
DECLARE_MODEM_INIT_DUMMY(s5000ap)
#endif

#if !IS_ENABLED(CONFIG_SEC_MODEM_S5100)
DECLARE_MODEM_INIT_DUMMY(s5100)
#endif

/* link device support */
DECLARE_LINK_INIT_DUMMY()

static modem_init_call modem_init_func[MAX_MODEM_TYPE] = {
	[SEC_S5000AP] = MODEM_INIT_CALL(s5000ap),
	[SEC_S5100] = MODEM_INIT_CALL(s5100),
	[MODEM_TYPE_DUMMY] = MODEM_INIT_CALL(dummy),
};

static link_init_call link_init_func[LINKDEV_MAX] = {
	[LINKDEV_UNDEFINED] = LINK_INIT_CALL_DUMMY(),
	[LINKDEV_SHMEM] = LINK_INIT_CALL(),
	[LINKDEV_PCIE] = LINK_INIT_CALL(),
};

int call_modem_init_func(struct modem_ctl *mc, struct modem_data *pdata)
{
	if (modem_init_func[pdata->modem_type])
		return modem_init_func[pdata->modem_type](mc, pdata);
	else
		return -ENOTSUPP;
}

struct link_device *call_link_init_func(struct platform_device *pdev,
					u32 link_type)
{
	if (link_init_func[link_type])
		return link_init_func[link_type](pdev, link_type);
	else
		return NULL;
}
