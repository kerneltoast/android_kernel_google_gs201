/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** File:
**
**
** Description:
**
** =============================================================================
*/

#include "../../inc/tas_smartamp_v2.h"
#ifdef CONFIG_PLATFORM_MTK
#include "algo/platform/mtk/tas_mtk.h"
#endif /*CONFIG_PLATFORM_QCOM*/

int tas25xx_smartamp_get_set(u8 *user_data, uint32_t param_id,
		uint8_t get_set, uint32_t length, module_id_t module_id)
{
	int ret = -EINVALID;
	struct ti_smartpa_data resp_data;
	(void)module_id;

	switch (get_set) {
	case TAS_SET_PARAM:
		memcpy(resp_data.payload, user_data, length);
		ret = ti_smartpa_write((void *)&resp_data, param_id, length);
		break;

	case TAS_GET_PARAM:
		memset(&resp_data, 0, sizeof(resp_data));
		ret = ti_smartpa_read((void *)&resp_data, param_id, length);
		if (ret == 0)
			memcpy(user_data, resp_data.payload, length);
		break;

	default:
		break;
	}

	return ret;
}

