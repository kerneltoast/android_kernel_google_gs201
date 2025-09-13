/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support For Battery EEPROM
 *
 * Copyright 2018 Google, LLC
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/nvmem-consumer.h>
#include <linux/module.h>
#include <linux/delay.h>
#include "gbms_storage.h"

#define BATT_EEPROM_TAG_MINF_OFFSET	0x00
#define BATT_EEPROM_TAG_MINF_LEN	GBMS_MINF_LEN
#define BATT_EEPROM_TAG_BGPN_OFFSET	0x03
#define BATT_EEPROM_TAG_BGPN_LEN	GBMS_BGPN_LEN
#define BATT_EEPROM_TAG_MYMD_OFFSET	0x0F
#define BATT_EEPROM_TAG_MYMD_LEN	BATT_EEPROM_TAG_XYMD_LEN
#define BATT_EEPROM_TAG_BRID_OFFSET	0x17
#define BATT_EEPROM_TAG_BRID_LEN	1
#define BATT_EEPROM_TAG_STRD_OFFSET	0x1E
#define BATT_EEPROM_TAG_STRD_LEN	12
#define BATT_EEPROM_TAG_ACIM_OFFSET	0x2C
#define BATT_EEPROM_TAG_ACIM_LEN	2
#define BATT_EEPROM_TAG_BCNT_OFFSET	0x2E
#define BATT_EEPROM_TAG_BCNT_LEN	(GBMS_CCBIN_BUCKET_COUNT * 2)
#define BATT_EEPROM_TAG_GMSR_OFFSET	0x42
#define BATT_EEPROM_TAG_GMSR_LEN	GBMS_GMSR_LEN
#define BATT_EEPROM_TAG_LOTR_OFFSET	0x59 // Layout Track
#define BATT_EEPROM_TAG_LOTR_LEN	1
#define BATT_EEPROM_TAG_CNHS_OFFSET	0x5A
#define BATT_EEPROM_TAG_CNHS_LEN	2
#define BATT_EEPROM_TAG_SELC_OFFSET	0x5C
#define BATT_EEPROM_TAG_SELC_LEN	1
#define BATT_EEPROM_TAG_CELC_OFFSET	0x5D
#define BATT_EEPROM_TAG_CELC_LEN	1

#define BATT_EEPROM_TAG_HIST_OFFSET	0x5E
#define BATT_EEPROM_TAG_HIST_LEN	BATT_ONE_HIST_LEN
#define BATT_MAX_HIST_CNT		75
#define BATT_TOTAL_HIST_LEN		(BATT_ONE_HIST_LEN * BATT_MAX_HIST_CNT)

#define BATT_EEPROM_TAG_EXTRA_START	(BATT_EEPROM_TAG_HIST_OFFSET + BATT_TOTAL_HIST_LEN)

// 0x3E2 is the first free with 75 history entries
#define BATT_EEPROM_TAG_FCRU_OFFSET	0x3E2
#define BATT_EEPROM_TAG_FCRU_LEN	GBMS_FCRU_LEN
#define BATT_EEPROM_TAG_FGST_OFFSET	0x3E4
#define BATT_EEPROM_TAG_FGST_LEN 	1
#define BATT_EEPROM_TAG_AYMD_OFFSET	0x3E5
#define BATT_EEPROM_TAG_AYMD_LEN	BATT_EEPROM_TAG_XYMD_LEN
#define BATT_EEPROM_TAG_GCFE_OFFSET	0x3E8
#define BATT_EEPROM_TAG_GCFE_LEN	2
#define BATT_EEPROM_TAG_RAVG_OFFSET	0x3EA
#define BATT_EEPROM_TAG_RAVG_LEN	2
#define BATT_EEPROM_TAG_RFCN_OFFSET	0x3EC
#define BATT_EEPROM_TAG_RFCN_LEN	2
#define BATT_EEPROM_TAG_DINF_OFFSET	0x3EE
#define BATT_EEPROM_TAG_DINF_LEN	GBMS_DINF_LEN
#define BATT_EEPROM_TAG_THAS_OFFSET	0x3FE
#define BATT_EEPROM_TAG_THAS_LEN	2

static struct gbms_storage_desc *gbee_desc;

#define GBEE_GET_NVRAM(ptr) ((struct nvmem_device *)(ptr))
#define GBEE_STORAGE_INFO(tag, addr, count, ptr) \
	(gbee_desc && gbee_desc->info) ? gbee_desc->info(tag, addr, count, ptr) : \
		gbee_storage_info(tag, addr, count, ptr)

/*
 * I2C error when try to write continuous data.
 * Add delay before write to wait previous internal write complete
 * http://b/179235291#comment8
 */
#define BATT_WAIT_INTERNAL_WRITE_MS	1

int gbee_storage_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr)
{
	int ret = 0;

	switch (tag) {
	case GBMS_TAG_MINF:
		*addr = BATT_EEPROM_TAG_MINF_OFFSET;
		*count = BATT_EEPROM_TAG_MINF_LEN;
		break;
	case GBMS_TAG_DINF:
		*addr = BATT_EEPROM_TAG_DINF_OFFSET;
		*count = BATT_EEPROM_TAG_DINF_LEN;
		break;
	case GBMS_TAG_HIST:
		*addr = BATT_EEPROM_TAG_HIST_OFFSET;
		*count = BATT_EEPROM_TAG_HIST_LEN;
		break;
	case GBMS_TAG_SNUM:
	case GBMS_TAG_BGPN:
		*addr = BATT_EEPROM_TAG_BGPN_OFFSET;
		*count = BATT_EEPROM_TAG_BGPN_LEN;
		break;
	case GBMS_TAG_GMSR:
		*addr = BATT_EEPROM_TAG_GMSR_OFFSET;
		*count = BATT_EEPROM_TAG_GMSR_LEN;
		break;
	case GBMS_TAG_BCNT:
		*addr = BATT_EEPROM_TAG_BCNT_OFFSET;
		*count = BATT_EEPROM_TAG_BCNT_LEN;
		break;
	case GBMS_TAG_CNHS:
		*addr = BATT_EEPROM_TAG_CNHS_OFFSET;
		*count = BATT_EEPROM_TAG_CNHS_LEN;
		break;
	case GBMS_TAG_LOTR:
		*addr = BATT_EEPROM_TAG_LOTR_OFFSET;
		*count = BATT_EEPROM_TAG_LOTR_LEN;
		break;
	case GBMS_TAG_SELC:
		*addr = BATT_EEPROM_TAG_SELC_OFFSET;
		*count = BATT_EEPROM_TAG_SELC_LEN;
		break;
	case GBMS_TAG_CELC:
		*addr = BATT_EEPROM_TAG_CELC_OFFSET;
		*count = BATT_EEPROM_TAG_CELC_LEN;
		break;
	case GBMS_TAG_STRD:
		*addr = BATT_EEPROM_TAG_STRD_OFFSET;
		*count = BATT_EEPROM_TAG_STRD_LEN;
		break;
	case GBMS_TAG_ACIM:
		*addr = BATT_EEPROM_TAG_ACIM_OFFSET;
		*count = BATT_EEPROM_TAG_ACIM_LEN;
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
	case GBMS_TAG_MYMD:
		*addr = BATT_EEPROM_TAG_MYMD_OFFSET;
		*count = BATT_EEPROM_TAG_MYMD_LEN;
		break;
	case GBMS_TAG_FCRU:
		*addr = BATT_EEPROM_TAG_FCRU_OFFSET;
		*count = BATT_EEPROM_TAG_FCRU_LEN;
		break;
	case GBMS_TAG_FGST:
		*addr = BATT_EEPROM_TAG_FGST_OFFSET;
		*count = BATT_EEPROM_TAG_FGST_LEN;
		break;
	default:
		ret = -ENOENT;
		break;
	}

	return ret;
}

static int gbee_storage_iter(int index, gbms_tag_t *tag, void *ptr)
{
	static const gbms_tag_t keys[] = { GBMS_TAG_BGPN, GBMS_TAG_MINF,
					   GBMS_TAG_DINF, GBMS_TAG_HIST,
					   GBMS_TAG_BRID, GBMS_TAG_SNUM,
					   GBMS_TAG_GMSR, GBMS_TAG_BCNT,
					   GBMS_TAG_CNHS, GBMS_TAG_SELC,
					   GBMS_TAG_CELC, GBMS_TAG_LOTR,
					   GBMS_TAG_STRD, GBMS_TAG_ACIM,
					   GBMS_TAG_GCFE, GBMS_TAG_RAVG,
					   GBMS_TAG_RFCN, GBMS_TAG_THAS,
					   GBMS_TAG_AYMD, GBMS_TAG_MYMD,
					   GBMS_TAG_FGST, GBMS_TAG_FCRU };
	const int count = ARRAY_SIZE(keys);

	if (index < 0 || index >= count)
		return -ENOENT;

	*tag = keys[index];
	return 0;
}

static int gbee_storage_read(gbms_tag_t tag, void *buff, size_t size, void *ptr)
{
	struct nvmem_device *nvmem = GBEE_GET_NVRAM(ptr);
	size_t offset = 0, len = 0;
	int ret;

	if (tag == GBMS_TAG_BRID) {
		u8 temp;

		if (size != sizeof(u32))
			return -ENOMEM;

		ret = nvmem_device_read(nvmem, BATT_EEPROM_TAG_BRID_OFFSET,
					1, &temp);
		if (ret < 0)
			return ret;

		((u32*)buff)[0] = temp;
		return len;
	}

	ret = GBEE_STORAGE_INFO(tag, &offset, &len, ptr);
	if (ret < 0)
		return ret;
	if (!len)
		return -ENOENT;
	if (len > size)
		return -ENOMEM;

	ret = nvmem_device_read(nvmem, offset, len, buff);
	if (ret == 0)
		ret = len;

	return ret;
}

static bool gbee_storage_is_writable(gbms_tag_t tag)
{
	switch (tag) {
	case GBMS_TAG_DINF:
	case GBMS_TAG_GMSR:
	case GBMS_TAG_BCNT:
	case GBMS_TAG_CNHS:
	case GBMS_TAG_SELC:
	case GBMS_TAG_CELC:
	case GBMS_TAG_BPST:
	case GBMS_TAG_STRD:
	case GBMS_TAG_ACIM:
	case GBMS_TAG_GCFE:
	case GBMS_TAG_RAVG:
	case GBMS_TAG_RFCN:
	case GBMS_TAG_THAS:
	case GBMS_TAG_AYMD:
	case GBMS_TAG_FGST:
	case GBMS_TAG_FCRU:
		return true;
	default:
		return false;
	}

}

static int gbee_storage_write(gbms_tag_t tag, const void *buff, size_t size,
			      void *ptr)
{
	struct nvmem_device *nvmem = ptr;
	size_t offset = 0, len = 0;
	int ret, write_size = 0;

	if (!gbee_storage_is_writable(tag))
		return -ENOENT;

	ret = GBEE_STORAGE_INFO(tag, &offset, &len, ptr);
	if (ret < 0)
		return ret;
	if (size > len)
		return -ENOMEM;

	for (write_size = 0; write_size < size; write_size++) {
		ret = nvmem_device_write(nvmem, write_size + offset, 1,
					 &((char *)buff)[write_size]);
		if (ret < 0)
			return ret;
		msleep(BATT_WAIT_INTERNAL_WRITE_MS);
	}

	ret = size;

	return ret;
}

static int gbee_storage_read_data(gbms_tag_t tag, void *data, size_t count,
				  int idx, void *ptr)
{
	struct nvmem_device *nvmem = GBEE_GET_NVRAM(ptr);
	size_t offset = 0, len = 0;
	int ret;

	switch (tag) {
	case GBMS_TAG_HIST:
		ret = GBEE_STORAGE_INFO(tag, &offset, &len, ptr);
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

static int gbee_storage_write_data(gbms_tag_t tag, const void *data,
				   size_t count, int idx, void *ptr)
{
	struct nvmem_device *nvmem = GBEE_GET_NVRAM(ptr);
	size_t offset = 0, len = 0;
	int ret, write_size = 0;

	switch (tag) {
	case GBMS_TAG_HIST:
		ret = GBEE_STORAGE_INFO(tag, &offset, &len, ptr);
		break;
	default:
		ret = -ENOENT;
		break;
	}

	if (ret < 0)
		return ret;

	if (idx < 0 || !data || !count)
		return -EINVAL;

	/* index == 0 is ok here */
	if (idx >= BATT_MAX_HIST_CNT)
		return -ENODATA;

	if (count > len)
		return -EINVAL;

	offset += len * idx;

	for (write_size = 0; write_size < len; write_size++) {
		ret = nvmem_device_write(nvmem, write_size + offset, 1,
					 &((char *)data)[write_size]);
		if (ret < 0)
			return ret;
		msleep(BATT_WAIT_INTERNAL_WRITE_MS);
	}

	ret = len;

	return ret;
}

static struct gbms_storage_desc gbee_storage_dsc = {
	.info = gbee_storage_info,
	.iter = gbee_storage_iter,
	.read = gbee_storage_read,
	.write = gbee_storage_write,
	.read_data = gbee_storage_read_data,
	.write_data = gbee_storage_write_data,
};

struct gbms_storage_desc gbee_storage01_dsc = {
	.info = gbee_storage01_info,
	.iter = gbee_storage01_iter,
	.read = gbee_storage_read,
	.write = gbee_storage_write,
	.read_data = gbee_storage_read_data,
	.write_data = gbee_storage_write_data,
};

struct gbms_storage_desc gbee_storage02_dsc = {
	.info = gbee_storage02_info,
	.iter = gbee_storage_iter,
	.read = gbee_storage_read,
	.write = gbee_storage_write,
	.read_data = gbee_storage_read_data_02,
	.write_data = gbee_storage_write_data,
};

/* TODO: factor history mechanics out of google battery? */
static int gbms_hist_move(struct nvmem_device *nvmem, int from, int to, int len)
{
	u8 *buff, *p;
	int index, ret;

	buff = kzalloc(len, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	/* move only the entries that are used */
	p = buff;
	for (index = 0; index < BATT_MAX_HIST_CNT; index++) {
		ret = nvmem_device_read(nvmem, from, BATT_ONE_HIST_LEN, p);
		if (ret < 0) {
			pr_err("%s: cannot read history data (%d)\n", __func__, ret);
			goto exit;
		}

		/* verify 1st byte for tempco */
		if (*p == 0xff)
			break;
		/* move to next history entry */
		from += BATT_ONE_HIST_LEN;
		p += BATT_ONE_HIST_LEN;
	}

	/* when the history data is empty */
	if (index == 0)
		goto exit;

	ret = nvmem_device_write(nvmem, to, (BATT_ONE_HIST_LEN * index), buff);
	if (ret < 0)
		pr_err("%s: cannot write history data (%d)\n", __func__, ret);

exit:
	kfree(buff);
	return ret;
}

static int gbms_lotr_update_to_v1(struct nvmem_device *nvmem)
{
	int ret;
	static u8 init_data[5]= { 0 };

	ret = gbms_hist_move(nvmem, 0x5E, 0x64, BATT_TOTAL_HIST_LEN);
	if (ret < 0) {
		pr_err("%s: cannot move history\n", __func__);
		return ret;
		/* TODO: flag this in BPST? */
	}

	ret = nvmem_device_write(nvmem, 0x5E, sizeof(init_data), init_data);
	if (ret != sizeof(init_data)) {
		pr_err("%s: cannot init new fields\n", __func__);
		return ret < 0 ? ret : -EINVAL;
	}

	return ret;
}

static int gbms_lotr_update_to_v2(struct nvmem_device *nvmem)
{
	int ret;
	static u8 clr_val[1] = { 0xff };
	static size_t offset = BATT_EEPROM_TAG_FCRU_OFFSET;
	size_t index, size = 0x3FF - offset;

	/* clear space in old version format */
	for (index = 0; index < size; index++) {
		ret = nvmem_device_write(nvmem, index + offset, 1, clr_val);
		if (ret < 0)
			return ret;
		msleep(BATT_WAIT_INTERNAL_WRITE_MS);
	}

	return ret;
}

/* LOTR is in a fixed position, move  */
static int gbms_lotr_update(struct nvmem_device *nvmem, int lotr_to)
{
	int ret, lotr_from = 0;

	ret = nvmem_device_read(nvmem, BATT_EEPROM_TAG_LOTR_OFFSET,
				BATT_EEPROM_TAG_LOTR_LEN, &lotr_from);
	pr_info("google_bms: lotr_from:%d, lotr_to=%d\n", lotr_from, lotr_to);
	if (ret < 0 || lotr_from == lotr_to)
		return ret;

	if (lotr_to == GBMS_LOTR_V1 && lotr_from == GBMS_LOTR_DEFAULT)
		ret = gbms_lotr_update_to_v1(nvmem);
	else if (lotr_to == GBMS_LOTR_V2 && lotr_from != GBMS_LOTR_V2)
		ret = gbms_lotr_update_to_v2(nvmem);
	else
		return 0;

	/* not block the process, wait to clear again in next boot if fail */
	if (ret < 0)
		return 0;

	/* now write lotr to the right place */
	ret = nvmem_device_write(nvmem, BATT_EEPROM_TAG_LOTR_OFFSET,
				 BATT_EEPROM_TAG_LOTR_LEN, &lotr_to);
	if (ret == BATT_EEPROM_TAG_LOTR_LEN)
		pr_info("%s: lotr migrated %d->%d\n", __func__, lotr_from, lotr_to);

	return ret;
}

static struct gbms_storage_desc *gbms_lotr_2_dsc(int lotr_ver)
{
	switch (lotr_ver) {
	case GBMS_LOTR_V1:
		return &gbee_storage01_dsc;
	case GBMS_LOTR_V2:
		return &gbee_storage02_dsc;
	default:
		return &gbee_storage_dsc;
	}
}

/*
 * Caller will use something like of_nvmem_device_get() to retrieve the
 * nvmem_device instance.
 * TODO: this only supports a singleton but the model can be extended to
 * multiple eeproms passing a structure to gbms_storage_register() and
 * modifying the implementation of GBEE_GET_NVRAM and GBEE_STORAGE_INFO
 * TODO: map nvram cells to tags
 */
int gbee_register_device(const char *name, int lotr, struct nvmem_device *nvram)
{
	int ret;

	gbee_desc = gbms_lotr_2_dsc(lotr);
	if (!gbee_desc)
		return -EINVAL;

	/* convert the layout (if needed) */
	ret = gbms_lotr_update(nvram, lotr);
	if (ret < 0) {
		pr_err("gbee %s update lotr failed, %d\n", name, ret);
		goto error_exit;
	}

	/* watch out for races on gbee_desc */
	ret = gbms_storage_register(gbee_desc, name, nvram);
	if (ret == 0)
		return 0;

error_exit:
	gbee_desc = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(gbee_register_device);

void gbee_destroy_device(void)
{

}
EXPORT_SYMBOL_GPL(gbee_destroy_device);

MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_DESCRIPTION("Google EEPROM");
MODULE_LICENSE("GPL");
