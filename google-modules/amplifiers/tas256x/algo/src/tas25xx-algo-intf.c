/*
 * =============================================================================
 * Copyright (c) 2016  Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * File:
 *     tas25xx-algo-intf.c
 *
 * Description:
 *     Wrapper on Interface to algorithm.
 *
 * =============================================================================
 */
#include <sound/soc.h>
#include <linux/types.h>
#include <linux/device.h>
#include "algo/inc/tas25xx-calib.h"
#include "algo/inc/tas_smart_amp_v2.h"
#include "tas25xx-algo-bin-utils.h"
#include "tas25xx-algo-intf.h"

static int s_tas_smartamp_bypass;
static int init_once;
static struct mutex smartpa_algo_lock;

void tas25xx_set_algo_bypass(int bypass)
{
	s_tas_smartamp_bypass = bypass;
}

int tas25xx_get_algo_bypass(void)
{
	return s_tas_smartamp_bypass;
}

void tas25xx_smartamp_alg_intf_init(void)
{
	if (!init_once) {
		mutex_init(&smartpa_algo_lock);
		pr_info("tas mutex init");
		init_once = 1;
	}
}

void tas25xx_smartamp_alg_intf_deinit(void)
{
	if (init_once) {
		mutex_destroy(&smartpa_algo_lock);
		pr_info("tas mutex destroy");
		init_once = 0;
	}
}

/*Wrapper arround set/get parameter,
 *all set/get commands pass through this wrapper
 */
int tas25xx_smartamp_algo_ctrl(u8 *user_data, uint32_t param_id,
		uint8_t get_set, uint32_t length, module_id_t module_id)
{
	int ret = 0;

	if (!s_tas_smartamp_bypass) {
		mutex_lock(&smartpa_algo_lock);
		ret = tas25xx_smartamp_get_set(user_data, param_id, get_set,
				length, module_id);
		mutex_unlock(&smartpa_algo_lock);
	} else {
		pr_info("TI-SmartPA: %s: algo set/get is bypassed\n", __func__);
	}

	return ret;
}
EXPORT_SYMBOL(tas25xx_smartamp_algo_ctrl);
