/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support For Battery EEPROM
 *
 * Copyright 2024 Google, LLC
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/nvmem-consumer.h>
#include <linux/module.h>
#include <linux/delay.h>
#include "gbms_storage.h"

#define BATT_EEPROM_TAG_HIST_OFFSET	0x5E
#define BATT_EEPROM_TAG_HIST_LEN	BATT_ONE_HIST_LEN
#define BATT_MAX_HIST_CNT		200
#define BATT_TOTAL_HIST_LEN		(BATT_ONE_HIST_LEN * BATT_MAX_HIST_CNT)
#define BATT_EEPROM_TAG_EXTRA_START	(BATT_EEPROM_TAG_HIST_OFFSET + BATT_TOTAL_HIST_LEN)
/* 0x9BE is the first free with 200 history entries. Write from end */
#define BATT_EEPROM_TAG_FCRU_OFFSET	0x1FE2
#define BATT_EEPROM_TAG_FCRU_LEN	GBMS_FCRU_LEN
#define BATT_EEPROM_TAG_FGST_OFFSET	0x1FE4
#define BATT_EEPROM_TAG_FGST_LEN	1
#define BATT_EEPROM_TAG_AYMD_OFFSET	0x1FE5
#define BATT_EEPROM_TAG_AYMD_LEN	BATT_EEPROM_TAG_XYMD_LEN
#define BATT_EEPROM_TAG_GCFE_OFFSET	0x1FE8
#define BATT_EEPROM_TAG_GCFE_LEN	2
#define BATT_EEPROM_TAG_RAVG_OFFSET	0x1FEA
#define BATT_EEPROM_TAG_RAVG_LEN	2
#define BATT_EEPROM_TAG_RFCN_OFFSET	0x1FEC
#define BATT_EEPROM_TAG_RFCN_LEN	2
#define BATT_EEPROM_TAG_DINF_OFFSET	0x1FEE
#define BATT_EEPROM_TAG_DINF_LEN	GBMS_DINF_LEN
#define BATT_EEPROM_TAG_THAS_OFFSET	0x1FFE
#define BATT_EEPROM_TAG_THAS_LEN	2

int gbee_storage02_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr)
{
	int ret = 0;

	switch (tag) {
	case GBMS_TAG_DINF:
		*addr = BATT_EEPROM_TAG_DINF_OFFSET;
		*count = BATT_EEPROM_TAG_DINF_LEN;
		break;
	case GBMS_TAG_HIST:
		*addr = BATT_EEPROM_TAG_HIST_OFFSET;
		*count = BATT_EEPROM_TAG_HIST_LEN;
		break;
	case GBMS_TAG_GCFE:
		*addr = BATT_EEPROM_TAG_GCFE_OFFSET;
		*count = BATT_EEPROM_TAG_GCFE_LEN;
		break;
	case GBMS_TAG_RAVG:
		*addr = BATT_EEPROM_TAG_RAVG_OFFSET;
		*count = BATT_EEPROM_TAG_RAVG_LEN;
		break;
	case GBMS_TAG_RFCN:
		*addr = BATT_EEPROM_TAG_RFCN_OFFSET;
		*count = BATT_EEPROM_TAG_RFCN_LEN;
		break;
	case GBMS_TAG_THAS:
		*addr = BATT_EEPROM_TAG_THAS_OFFSET;
		*count = BATT_EEPROM_TAG_THAS_LEN;
		break;
	case GBMS_TAG_AYMD:
		*addr = BATT_EEPROM_TAG_AYMD_OFFSET;
		*count = BATT_EEPROM_TAG_AYMD_LEN;
		break;
	case GBMS_TAG_FGST:
		*addr = BATT_EEPROM_TAG_FGST_OFFSET;
		*count = BATT_EEPROM_TAG_FGST_LEN;
		break;
	case GBMS_TAG_FCRU:
		*addr = BATT_EEPROM_TAG_FCRU_OFFSET;
		*count = BATT_EEPROM_TAG_FCRU_LEN;
		break;
	default:
		ret = gbee_storage_info(tag, addr, count, ptr);
		break;
	}

	return ret;
}

int gbee_storage_read_data_02(gbms_tag_t tag, void *data, size_t count,
				  int idx, void *ptr)
{
	struct nvmem_device *nvmem = (struct nvmem_device *)(ptr);
	size_t offset = 0, len = 0;
	int ret;

	switch (tag) {
	case GBMS_TAG_HIST:
		ret = gbee_storage02_info(tag, &offset, &len, ptr);
		break;
	default:
		ret = -ENOENT;
		break;
	}

	if (ret < 0)
		return ret;

	if (!data || !count) {
		if (idx == GBMS_STORAGE_INDEX_INVALID)
			return 0;
		else
			return BATT_MAX_HIST_CNT;
	}

	if (idx < 0)
		return -EINVAL;

	/* index == 0 is ok here */
	if (idx >= BATT_MAX_HIST_CNT)
		return -ENODATA;

	if (len > count)
		return -EINVAL;

	offset += len * idx;

	ret = nvmem_device_read(nvmem, offset, len, data);
	if (ret == 0)
		ret = len;

	return ret;
}
