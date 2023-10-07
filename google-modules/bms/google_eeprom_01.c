/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support For Battery EEPROM
 *
 * Copyright 2021 Google, LLC
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/nvmem-consumer.h>
#include <linux/module.h>
#include <linux/delay.h>
#include "gbms_storage.h"

/* battery health */
#define BATT_EEPROM_TAG_BPST_OFFSET	0x5E
#define BATT_EEPROM_TAG_BPST_LEN	1
/* history data */
#define BATT_EEPROM_TAG_HIST_OFFSET	0x64
#define BATT_EEPROM_TAG_HIST_LEN	BATT_ONE_HIST_LEN

/* Add GBMS_TAG_BPST move down history */
int gbee_storage01_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr)
{
	int ret = 0;

	switch (tag) {
	case GBMS_TAG_BPST:
		*addr = BATT_EEPROM_TAG_BPST_OFFSET;
		*count = BATT_EEPROM_TAG_BPST_LEN;
		break;
	case GBMS_TAG_HIST:
		*addr = BATT_EEPROM_TAG_HIST_OFFSET;
		*count = BATT_EEPROM_TAG_HIST_LEN;
		break;
	default:
		ret = gbee_storage_info(tag, addr, count, ptr);
		break;
	}

	return ret;
}

/* same as defaults in gbee_storage_iter() */
int gbee_storage01_iter(int index, gbms_tag_t *tag, void *ptr)
{
	static const gbms_tag_t keys[] = { GBMS_TAG_BGPN, GBMS_TAG_MINF,
					   GBMS_TAG_DINF, GBMS_TAG_HIST,
					   GBMS_TAG_BRID, GBMS_TAG_SNUM,
					   GBMS_TAG_GMSR, GBMS_TAG_BCNT,
					   GBMS_TAG_CNHS, GBMS_TAG_SELC,
					   GBMS_TAG_CELC, GBMS_TAG_LOTR,
					   GBMS_TAG_BPST, GBMS_TAG_STRD };
	const int max = ARRAY_SIZE(keys);

	if (index < 0 || index >= max)
		return -ENOENT;

	*tag = keys[index];
	return 0;
}

