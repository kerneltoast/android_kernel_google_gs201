/*
 ** ============================================================================
 ** Copyright (c) 2016  Texas Instruments Inc.
 **
 ** This program is free software; you can redistribute it and/or modify it
 ** under the terms of the GNU General Public License as published by the Free
 ** Software Foundation; version 2.
 **
 ** This program is distributed in the hope that it will be useful, but WITHOUT
 ** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 ** FITNESS
 ** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 ** details
 **
 ** File:
 **     tas25xx-algo-bin-utils.c
 **
 ** Description:
 **     Algorithm parameter's binary file utility wrapper.
 **
 ** ============================================================================
 */
#include <linux/types.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>

#include "tas25xx-algo-bin-utils.h"

const char *binfilenames[NUM_SPK_TYPES] = {
	"tas25xx_TI_0.bin",
	"tas25xx_TI_1.bin",
	"tas25xx_TI_2.bin",
	"tas25xx_TI_3.bin"
};

static struct bin_file_data_t s_bin = {
	.uses_bin_file = 0,
	.parse_done = 0,
	.profile_id = 0,
	.spk_id = 0,
	.bytes_per_prof_per_channel = 0,
	.number_of_profiles = 0,
	.no_of_channels = 0,
	.single_profile_size = 0,
	.profile_data = NULL,
};

static struct device *s_dev;

void bin_file_set_device(struct device *dev)
{
	s_dev = dev;
}

static inline struct device *bin_file_get_device(void)
{
	return s_dev;
}

/*
 * returns 0 on success, -ve errorcode otherwise
 */
int bin_file_parse_init(void)
{
	int ret = 0;
	char metadata[SA_BIN_MDATA_SZ] = { 0 };
	char header[SA_BIN_HDR_SIZE] = { 0 };
	uint32_t *header_ptr = (uint32_t *)header;
	const struct firmware *fw_entry = NULL;
	struct device *dev;
	int32_t i = 0;
	int bin_data_size;

	int32_t *data;
	int32_t version;
	int32_t time;
	char *ddc;
	char ddcl[64] = { 0 };

	s_bin.uses_bin_file = 1;

	if (s_bin.profile_data)
		return 0;

	dev = bin_file_get_device();
	if (!dev) {
		pr_err("TI-SmartPA BIN: struct device is null\n");
		return -EINVAL;
	}

	if ((s_bin.spk_id >= 0) && (s_bin.spk_id < NUM_SPK_TYPES)) {
		pr_info("TI-SmartPA BIN : Speaker ID:%d,Bin file : %s\n",
			s_bin.spk_id, binfilenames[s_bin.spk_id]);
		ret = request_firmware(&fw_entry,
			binfilenames[s_bin.spk_id], dev);
		if (ret || !fw_entry || !fw_entry->data ||
			(fw_entry->size  < (SA_BIN_MDATA_SZ+SA_BIN_HDR_SIZE))) {
			pr_err("TI-SmartPA BIN : request_firmware failed with error=%d, fw_entry=%p, size=%d\n",
				ret, fw_entry,
				fw_entry ? (int)fw_entry->size : 0);
			return ret;
		}

		memcpy(metadata, fw_entry->data, SA_BIN_MDATA_SZ);

		data = (int32_t *)metadata;
		version = data[0];
		time = data[1];

		ddc = (char *)data;
		while (((71 - i) > 8) && ddc[71 - i]) {
			ddcl[i] = ddc[71 - i];
			i++;
		}

		pr_info("TI-SmartPA BIN: Version = 0x%x\n", version);
		pr_info("TI-SmartPA BIN: Time = %d seconds from epoch time\n",
			time);
		pr_info("TI-SmartPA BIN: DDC Name = %s\n", ddcl);
		pr_info("TI-SmartPA BIN: metadata read !!\n");

		memcpy(header, (char *)(fw_entry->data) + SA_BIN_MDATA_SZ,
			SA_BIN_HDR_SIZE);
		s_bin.number_of_profiles = header_ptr[0];
		s_bin.no_of_channels = (header_ptr[1] >> 24) & 0xFF;
		s_bin.single_profile_size = header_ptr[1] & 0xFFFF;
		s_bin.bytes_per_prof_per_channel =
			s_bin.single_profile_size / s_bin.no_of_channels;

		pr_info("TI-SmartPA BIN: [BIN]: num of prof=%d, num of ch=%d, single profile sz=%d\n",
				s_bin.number_of_profiles,
				s_bin.no_of_channels,
				s_bin.single_profile_size);

		bin_data_size =
			s_bin.number_of_profiles * s_bin.single_profile_size;
		if (bin_data_size == (fw_entry->size - SA_BIN_MDATA_SZ - SA_BIN_HDR_SIZE)) {
			dev_info(dev, "%s ALLOC: %d", __func__, bin_data_size);
			s_bin.profile_data = kmalloc(bin_data_size, GFP_KERNEL);
		} else {
			pr_err("TI-SmartPA BIN : size mismatch bin vs firmware\n");
			s_bin.profile_data = NULL;
		}

		if (!s_bin.profile_data) {
			pr_err("TI-SmartPA BIN : kmalloc failed\n");
			bin_file_parse_deinit();
			ret = -ENOMEM;
		} else {
			char *data_start =
				((char *)(fw_entry->data)) +
				SA_BIN_MDATA_SZ + SA_BIN_HDR_SIZE;

			memcpy(s_bin.profile_data, data_start, bin_data_size);
			s_bin.parse_done = 1;
			pr_info("TI-SmartPA BIN: [BIN]: Read %d bytes successfully !!!\n",
				bin_data_size);
		}

		release_firmware(fw_entry);
	} else {
		pr_err("TI-SmartPA BIN : Invalid speaker ID : %d\n",
			s_bin.spk_id);
		return -EINVAL;
	}

	return ret;
}

/*
 * 1 is success, 0 is false
 */
bool bin_file_parse_status(void)
{
	if (s_bin.uses_bin_file && s_bin.parse_done)
		return true;

	return false;
}

/* parse profile apr */
bool bin_file_get_profile_data(char **buf, int *size)
{
	bool success = false;
	int profile_id;

	if (s_bin.profile_id < 0)
		s_bin.profile_id = 0;

	profile_id = s_bin.profile_id;

	pr_info("TI-SmartPA BIN: [BIN] load_profile %d",
		s_bin.profile_id);

	/*
	 * profile_id [0- g_number_of_profiles)
	 */
	if (!bin_file_parse_init() &&
		(profile_id < s_bin.number_of_profiles) &&
		buf && size) {

		int sz = s_bin.bytes_per_prof_per_channel;
		uint8_t *ptr = (uint8_t *)s_bin.profile_data;

		ptr += (profile_id * s_bin.single_profile_size);
		*buf = ptr;

		if (s_bin.no_of_channels == 2)
			sz += s_bin.bytes_per_prof_per_channel;

		*size = sz;
		success = true;
	} else {
		pr_info("TI-SmartPA BIN:[BIN] error parsing bin file!!!");
	}

	return success;
}

void bin_file_parse_deinit(void)
{
	bool temp = s_bin.uses_bin_file;
	int old_spk_id = s_bin.spk_id;
	int old_profile_id = s_bin.profile_id;

	if (s_bin.profile_data)
		kfree(s_bin.profile_data);

	memset(&s_bin, 0, sizeof(s_bin));

	s_bin.profile_id = old_profile_id;
	s_bin.spk_id = old_spk_id;
	s_bin.uses_bin_file = temp;
}

void bin_file_set_profile_id(int profile_id)
{
	s_bin.profile_id = profile_id;
}

void bin_file_set_spk_id(int spk_id)
{
	bin_file_parse_deinit();
	s_bin.spk_id = spk_id;
}

/*read only functions*/
int bin_file_get_profile_id(void)
{
	return s_bin.profile_id;
}

int bin_file_get_spk_id(void)
{
	return s_bin.spk_id;
}

int bin_file_get_number_of_channels(void)
{
	return s_bin.no_of_channels;
}

bool uses_bin_file(void)
{
	return s_bin.uses_bin_file;
}

int bin_file_get_per_profile_per_channel_param_count(void)
{
	return s_bin.bytes_per_prof_per_channel / sizeof(int);
}

int32_t *bin_file_get_profile_0_data_pointer(void)
{
	if (s_bin.uses_bin_file && s_bin.parse_done)
		return (int32_t *)s_bin.profile_data;

	return NULL;
}

/**
 * custom value start - size
 * custom value 1
 * custom value 2
 */
int32_t bin_file_get_custom_value_by_idx(int index_request, int *value)
{
	int custom_index_start;
	int ret = -EINVAL;
	int32_t *data = bin_file_get_profile_0_data_pointer();

	if (data && value) {
		custom_index_start = data[CUSTOM_VALUE_INDEX_PTR];
		if (custom_index_start &&
			(index_request < data[custom_index_start])) {
			custom_index_start += 1; /* points to data - index 0 */
			*value = data[custom_index_start + index_request];
			ret = 0;
		}
	}

	return ret;
}
