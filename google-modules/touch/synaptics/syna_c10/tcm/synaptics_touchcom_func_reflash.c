// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file synaptics_touchcom_func_reflash.c
 *
 * This file implements the fw reflash related functions of TouchBoot.
 * The declarations are available in synaptics_touchcom_func_reflash.h.
 */

#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_reflash.h"

/**
 * @section: Reflash relevant definitions
 *
 */
#define FLASH_READ_DELAY_MS (10)
#define FLASH_WRITE_DELAY_MS (20)
#define FLASH_ERASE_DELAY_MS (500)

#define BOOT_CONFIG_SIZE 8
#define BOOT_CONFIG_SLOTS 16

#define DO_NONE (0)
#define DO_UPDATE (1)

/**
 * syna_tcm_set_up_flash_access()
 *
 * Enter the bootloader fw if not in the mode.
 * Besides, get the necessary parameters in boot info.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [out] reflash_data: data blob for reflash
 *
 * @return
 *    Result of image file comparison
 */
static int syna_tcm_set_up_flash_access(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data)
{
	int retval;
	unsigned int temp;
	struct tcm_identification_info id_info;
	struct tcm_boot_info *boot_info;
	unsigned int wr_chunk;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	LOGI("Set up flash access\n");

	retval = syna_tcm_identify(tcm_dev, &id_info);
	if (retval < 0) {
		LOGE("Fail to do identification\n");
		return retval;
	}

	/* switch to bootloader mode */
	if (IS_APP_FW_MODE(id_info.mode)) {
		LOGI("Prepare to enter bootloader mode\n");

		retval = syna_tcm_switch_fw_mode(tcm_dev,
				MODE_BOOTLOADER,
				FW_MODE_SWITCH_DELAY_MS);
		if (retval < 0) {
			LOGE("Fail to enter bootloader mode\n");
			return retval;
		}
	}

	if (!IS_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Fail to enter bootloader mode (current: 0x%x)\n",
			tcm_dev->dev_mode);
		return retval;
	}

	boot_info = &tcm_dev->boot_info;

	/* get boot info to set up the flash access */
	retval = syna_tcm_get_boot_info(tcm_dev, boot_info);
	if (retval < 0) {
		LOGE("Fail to get boot info at mode 0x%x\n",
			id_info.mode);
		return retval;
	}

	wr_chunk = tcm_dev->max_wr_size;

	temp = boot_info->write_block_size_words;
	reflash_data->write_block_size = temp * 2;

	LOGI("Write block size: %d (words size: %d)\n",
		reflash_data->write_block_size, temp);

	temp = syna_pal_le2_to_uint(boot_info->erase_page_size_words);
	reflash_data->page_size = temp * 2;

	LOGI("Erase page size: %d (words size: %d)\n",
		reflash_data->page_size, temp);

	temp = syna_pal_le2_to_uint(boot_info->max_write_payload_size);
	reflash_data->max_write_payload_size = temp;

	LOGI("Max write flash data size: %d\n",
		reflash_data->max_write_payload_size);

	if (reflash_data->write_block_size > (wr_chunk - 9)) {
		LOGE("Write block size, %d, greater than chunk space, %d\n",
			reflash_data->write_block_size, (wr_chunk - 9));
		return _EINVAL;
	}

	if (reflash_data->write_block_size == 0) {
		LOGE("Invalid write block size %d\n",
			reflash_data->write_block_size);
		return _EINVAL;
	}

	if (reflash_data->page_size == 0) {
		LOGE("Invalid erase page size %d\n",
			reflash_data->page_size);
		return _EINVAL;
	}

	return 0;
}

/**
 * syna_tcm_compare_image_id_info()
 *
 * Compare the ID information between device and the image file,
 * and then determine the area to be updated.
 * The function should be called after parsing the image file.
 *
 * @param
 *    [ in] tcm_dev:       the device handle
 *    [ in] reflash_data:  data blob for reflash
 *
 * @return
 *    Blocks to be updated
 */
int syna_tcm_compare_image_id_info(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data)
{
	enum update_area result;
	unsigned int idx;
	unsigned int image_fw_id;
	unsigned int device_fw_id;
	unsigned char *image_config_id;
	unsigned char *device_config_id;
	struct app_config_header *header;
	const unsigned char *app_config_data;
	struct block_data *app_config;

	result = UPDATE_NONE;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash_data\n");
		return _EINVAL;
	}

	app_config = &reflash_data->image_info.data[AREA_APP_CONFIG];

	if (app_config->size < sizeof(struct app_config_header)) {
		LOGE("Invalid application config in image file\n");
		return _EINVAL;
	}

	app_config_data = app_config->data;
	header = (struct app_config_header *)app_config_data;

	image_fw_id = syna_pal_le4_to_uint(header->build_id);
	device_fw_id = tcm_dev->packrat_number;

	LOGN("Device firmware ID: %d, image build id: %d\n",
		device_fw_id, image_fw_id);

	if (image_fw_id != device_fw_id) {
		LOGN("Image build ID and device fw ID mismatched\n");
		result = UPDATE_FIRMWARE_CONFIG;
		goto exit;
	}

	image_config_id = header->customer_config_id;
	device_config_id = tcm_dev->app_info.customer_config_id;

	for (idx = 0; idx < MAX_SIZE_CONFIG_ID; idx++) {
		if (image_config_id[idx] != device_config_id[idx]) {
			LOGN("Different Config ID\n");
			result = UPDATE_CONFIG_ONLY;
			goto exit;
		}
	}

	result = UPDATE_NONE;

exit:
	switch (result) {
	case UPDATE_FIRMWARE_CONFIG:
		LOGN("Update firmware and config\n");
		break;
	case UPDATE_CONFIG_ONLY:
		LOGN("Update config only\n");
		break;
	case UPDATE_NONE:
	default:
		LOGN("No need to do reflash\n");
		break;
	}

	return (int)result;
}

/**
 * syna_tcm_check_flash_boot_config()
 *
 * Check whether the same flash address of boot config in between the device
 * and the image file.
 *
 * @param
 *    [ in] boot_config:     block data of boot_config from image file
 *    [ in] boot_info:       data of boot info
 *    [ in] block_size:      max size of write block
 *
 * @return
 *    '0' represent no need to do reflash;
 *    '1' represent a positive result of checking;
 *    otherwise, negative value on error.
 */
static int syna_tcm_check_flash_boot_config(struct block_data *boot_config,
		struct tcm_boot_info *boot_info, unsigned int block_size)
{
	unsigned int start_block;
	unsigned int image_addr;
	unsigned int device_addr;

	if (!boot_config) {
		LOGE("Invalid boot_config block data\n");
		return _EINVAL;
	}

	if (!boot_info) {
		LOGE("Invalid boot_info\n");
		return _EINVAL;
	}

	if (boot_config->size < BOOT_CONFIG_SIZE) {
		LOGE("No valid BOOT_CONFIG size, %d, in image file\n",
			boot_config->size);
		return _EINVAL;
	}

	image_addr = boot_config->flash_addr;

	LOGD("Boot Config address in image file: 0x%x\n", image_addr);

	start_block = VALUE(boot_info->boot_config_start_block);
	device_addr = start_block * block_size;

	LOGD("Boot Config address in device: 0x%x\n", device_addr);

	return DO_NONE;
}

/**
 * syna_tcm_check_flash_app_config()
 *
 * Check whether the same flash address of app config in between the
 * device and the image file.
 *
 * @param
 *    [ in] app_config:      block data of app_config from image file
 *    [ in] app_info:        data of application info
 *    [ in] block_size:      max size of write block
 *
 * @return
 *    '0' represent no need to do reflash;
 *    '1' represent a positive result of checking;
 *    otherwise, negative value on error.
 */
static int syna_tcm_check_flash_app_config(struct block_data *app_config,
		struct tcm_application_info *app_info, unsigned int block_size)
{
	unsigned int temp;
	unsigned int image_addr;
	unsigned int image_size;
	unsigned int device_addr;
	unsigned int device_size;

	if (!app_config) {
		LOGE("Invalid app_config block data\n");
		return _EINVAL;
	}

	if (!app_info) {
		LOGE("Invalid app_info\n");
		return _EINVAL;
	}

	if (app_config->size == 0) {
		LOGD("No APP_CONFIG in image file\n");
		return DO_NONE;
	}

	image_addr = app_config->flash_addr;
	image_size = app_config->size;

	LOGD("App Config address in image file: 0x%x, size: %d\n",
		image_addr, image_size);

	temp = VALUE(app_info->app_config_start_write_block);
	device_addr = temp * block_size;
	device_size = VALUE(app_info->app_config_size);

	LOGD("App Config address in device: 0x%x, size: %d\n",
		device_addr, device_size);

	if (device_addr == 0 && device_size == 0)
		return DO_UPDATE;

	if (image_addr != device_addr)
		LOGW("App Config address mismatch, image:0x%x, dev:0x%x\n",
			image_addr, device_addr);

	if (image_size != device_size)
		LOGW("App Config address size mismatch, image:%d, dev:%d\n",
			image_size, device_size);

	return DO_UPDATE;
}

/**
 * syna_tcm_check_flash_disp_config()
 *
 * Check whether the same flash address of display config in between the
 * device and the image file.
 *
 * @param
 *    [ in] disp_config:     block data of disp_config from image file
 *    [ in] boot_info:       data of boot info
 *    [ in] block_size:      max size of write block
 *
 * @return
 *    '0' represent no need to do reflash;
 *    '1' represent a positive result of checking;
 *    otherwise, negative value on error.
 */
static int syna_tcm_check_flash_disp_config(struct block_data *disp_config,
		struct tcm_boot_info *boot_info, unsigned int block_size)
{
	unsigned int temp;
	unsigned int image_addr;
	unsigned int image_size;
	unsigned int device_addr;
	unsigned int device_size;

	if (!disp_config) {
		LOGE("Invalid disp_config block data\n");
		return _EINVAL;
	}

	if (!boot_info) {
		LOGE("Invalid boot_info\n");
		return _EINVAL;
	}

	/* disp_config area may not be included in all product */
	if (disp_config->size == 0) {
		LOGD("No DISP_CONFIG in image file\n");
		return DO_NONE;
	}

	image_addr = disp_config->flash_addr;
	image_size = disp_config->size;

	LOGD("Disp Config address in image file: 0x%x, size: %d\n",
		image_addr, image_size);

	temp = VALUE(boot_info->display_config_start_block);
	device_addr = temp * block_size;

	temp = VALUE(boot_info->display_config_length_blocks);
	device_size = temp * block_size;

	LOGD("Disp Config address in device: 0x%x, size: %d\n",
		device_addr, device_size);

	if (image_addr != device_addr)
		LOGW("Disp Config address mismatch, image:0x%x, dev:0x%x\n",
			image_addr, device_addr);

	if (image_size != device_size)
		LOGW("Disp Config address size mismatch, image:%d, dev:%d\n",
			image_size, device_size);

	return DO_UPDATE;
}

/**
 * syna_tcm_check_flash_app_code()
 *
 * Check whether the valid size of app firmware in the image file
 *
 * @param
 *    [ in] app_code:      block data of app_code from image file
 *
 * @return
 *    '0' represent no need to do reflash;
 *    '1' represent a positive result of checking;
 *    otherwise, negative value on error.
 */
static int syna_tcm_check_flash_app_code(struct block_data *app_code)
{
	if (!app_code) {
		LOGE("Invalid app_code block data\n");
		return _EINVAL;
	}

	if (app_code->size == 0) {
		LOGD("No %s in image file\n", AREA_ID_STR(app_code->id));
		return _EINVAL;
	}

	return DO_UPDATE;
}

/**
 * syna_tcm_check_flash_openshort()
 *
 * Check whether the valid size of openshort area in the image file
 *
 * @param
 *    [ in] open_short:      block data of open_short from image file
 *
 * @return
 *    '0' represent no need to do reflash;
 *    '1' represent a positive result of checking;
 *    otherwise, negative value on error.
 */
static int syna_tcm_check_flash_openshort(struct block_data *open_short)
{
	if (!open_short) {
		LOGE("Invalid open_short block data\n");
		return _EINVAL;
	}

	/* open_short area may not be included in all product */
	if (open_short->size == 0) {
		LOGD("No %s in image file\n", AREA_ID_STR(open_short->id));
		return DO_NONE;
	}

	return DO_UPDATE;
}

/**
 * syna_tcm_check_flash_app_prod_test()
 *
 * Check whether the valid size of app prod_test area in the image file
 *
 * @param
 *    [ in] prod_test:  block data of app_prod_test from image file
 *
 * @return
 *    '0' represent no need to do reflash;
 *    '1' represent a positive result of checking;
 *    otherwise, negative value on error.
 */
static int syna_tcm_check_flash_app_prod_test(struct block_data *prod_test)
{
	if (!prod_test) {
		LOGE("Invalid app_prod_test block data\n");
		return _EINVAL;
	}

	/* app_prod_test area may not be included in all product */
	if (prod_test->size == 0) {
		LOGD("No %s in image file\n", AREA_ID_STR(prod_test->id));
		return DO_NONE;
	}

	return DO_UPDATE;
}

/**
 * syna_tcm_check_flash_ppdt()
 *
 * Check whether the valid size of ppdt area in the image file
 *
 * @param
 *    [ in] ppdt:      block data of PPDT from image file
 *
 * @return
 *    '0' represent no need to do reflash;
 *    '1' represent a positive result of checking;
 *    otherwise, negative value on error.
 */
static int syna_tcm_check_flash_ppdt(struct block_data *ppdt)
{
	if (!ppdt) {
		LOGE("Invalid ppdt block data\n");
		return _EINVAL;
	}

	/* open_short area may not be included in all product */
	if (ppdt->size == 0) {
		LOGD("No %s in image file\n", AREA_ID_STR(ppdt->id));
		return DO_NONE;
	}

	return DO_UPDATE;
}

/**
 * syna_tcm_check_flash_block()
 *
 * Dispatch to the proper helper to ensure the data of associated block area
 * is correct in between the device and the image file.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] reflash_data: data blob for reflash
 *    [ in] block:        target block area to check
 *
 * @return
 *    '0' represent no need to do reflash;
 *    '1' represent a positive result of checking;
 *    otherwise, negative value on error.
 */
static int syna_tcm_check_flash_block(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		struct block_data *block)
{
	int retval = 0;
	struct tcm_application_info *app_info;
	struct tcm_boot_info *boot_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return DO_NONE;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return DO_NONE;
	}

	if (!block) {
		LOGE("Invalid block data\n");
		return DO_NONE;
	}

	app_info = &tcm_dev->app_info;
	boot_info = &tcm_dev->boot_info;

	switch (block->id) {
	case AREA_APP_CODE:
		retval = syna_tcm_check_flash_app_code(block);
		break;
	case AREA_APP_CONFIG:
		retval = syna_tcm_check_flash_app_config(block, app_info,
				reflash_data->write_block_size);
		break;
	case AREA_BOOT_CONFIG:
		retval = syna_tcm_check_flash_boot_config(block, boot_info,
				reflash_data->write_block_size);
		break;
	case AREA_DISP_CONFIG:
		retval = syna_tcm_check_flash_disp_config(block, boot_info,
				reflash_data->write_block_size);
		break;
	case AREA_OPEN_SHORT_TUNING:
		retval = syna_tcm_check_flash_openshort(block);
		break;
	case AREA_PROD_TEST:
		retval = syna_tcm_check_flash_app_prod_test(block);
		break;
	case AREA_PPDT:
		retval = syna_tcm_check_flash_ppdt(block);
		break;
	default:
		retval = DO_NONE;
		break;
	}

	return retval;
}

/**
 * syna_tcm_get_flash_data_location()
 *
 * Return the address and length of the specified data area
 * in the flash memory.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] area:    specified area in flash memory
 *    [out] addr:    the flash address of the specified area returned
 *    [out] len:     the size of the specified area returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_get_flash_data_location(struct tcm_dev *tcm_dev,
		enum flash_area area, unsigned int *addr, unsigned int *len)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned char payload;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	switch (area) {
	case AREA_CUSTOM_LCM:
		payload = FLASH_LCM_DATA;
		break;
	case AREA_CUSTOM_OEM:
		payload = FLASH_OEM_DATA;
		break;
	case AREA_PPDT:
		payload = FLASH_PPDT_DATA;
		break;
	case AREA_FORCE_TUNING:
		payload = FLASH_FORCE_CALIB_DATA;
		break;
	case AREA_OPEN_SHORT_TUNING:
		payload = FLASH_OPEN_SHORT_TUNING_DATA;
		break;
	default:
		LOGE("Invalid flash area %d\n", area);
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_DATA_LOCATION,
			&payload,
			sizeof(payload),
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_DATA_LOCATION);
		goto exit;
	}

	if (tcm_dev->resp_buf.data_length != 4) {
		LOGE("Invalid data length %d\n",
			tcm_dev->resp_buf.data_length);
		retval = _EINVAL;
		goto exit;
	}

	*addr = syna_pal_le2_to_uint(&tcm_dev->resp_buf.buf[0]);
	*len = syna_pal_le2_to_uint(&tcm_dev->resp_buf.buf[2]);

	retval = 0;

exit:
	return retval;
}

/**
 * syna_tcm_reflash_send_command()
 *
 * Helper to wrap up the write_message() function.
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] command:        given command code
 *    [ in] payload:        payload data, if any
 *    [ in] payload_len:    length of payload data
 *    [ in] delay_ms_resp:  delay time to get the response of command
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_reflash_send_command(struct tcm_dev *tcm_dev,
		unsigned char command, unsigned char *payload,
		unsigned int payload_len, unsigned int delay_ms_resp)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!IS_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in BL mode, 0x%x\n", tcm_dev->dev_mode);
		retval = _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			command,
			payload,
			payload_len,
			&resp_code,
			delay_ms_resp);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n", command);
		goto exit;
	}

exit:
	return retval;
}

/**
 * syna_tcm_read_flash()
 *
 * Implement the bootloader command to read specified data from flash memory.
 *
 * Reads to the protected bootloader code or application code areas will read
 * as 0. If the number of words requested is too large, it may be truncated to
 * an defined maximum read size.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] address:      the address in flash memory to read
 *    [out] rd_data:      data retrieved
 *    [ in] rd_len:       length of data to be read
 *    [ in] rd_delay_ms:  a short delay after the command executed
 *                        set 'DEFAULT_FLASH_READ_DELAY' to use default,
 *                        which is 10 us;
 *                        set '0' or 'RESP_IN_ATTN' to select ATTN-driven.
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_read_flash(struct tcm_dev *tcm_dev,
		unsigned int address, unsigned char *rd_data,
		unsigned int rd_len, unsigned int rd_delay_ms)
{
	int retval = 0;
	unsigned int length_words;
	unsigned int flash_addr_words;
	unsigned char out[6] = { 0 };
	unsigned int delay_ms = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!rd_data) {
		LOGE("Invalid rd_data buffer\n");
		return _EINVAL;
	}

	if (address == 0 || rd_len == 0) {
		LOGE("Invalid flash address and length\n");
		retval = _EINVAL;
		goto exit;
	}

	length_words = rd_len / 2;
	flash_addr_words = address / 2;

	LOGD("Flash address: 0x%x (words: 0x%x), size: %d (words: %d)\n",
		address, flash_addr_words, rd_len, length_words);

	out[0] = (unsigned char)flash_addr_words;
	out[1] = (unsigned char)(flash_addr_words >> 8);
	out[2] = (unsigned char)(flash_addr_words >> 16);
	out[3] = (unsigned char)(flash_addr_words >> 24);
	out[4] = (unsigned char)length_words;
	out[5] = (unsigned char)(length_words >> 8);

	if (rd_delay_ms == DEFAULT_FLASH_READ_DELAY)
		delay_ms = FLASH_READ_DELAY_MS;
	else
		delay_ms = rd_delay_ms;

	if (delay_ms == RESP_IN_ATTN) {
		LOGD("xfer: %d, delay: ATTN-driven\n", length_words);
	} else {
		delay_ms = (delay_ms * length_words) / 1000;
		LOGD("xfer: %d, delay: %d\n", length_words, delay_ms);
	}

	retval = syna_tcm_reflash_send_command(tcm_dev,
			CMD_READ_FLASH,
			out,
			sizeof(out),
			delay_ms);
	if (retval < 0) {
		LOGE("Fail to read flash data from addr 0x%x, size %d\n",
			address, rd_len);
		goto exit;
	}

	if (tcm_dev->resp_buf.data_length != rd_len) {
		LOGE("Fail to read requested length %d, rd_len %d\n",
			tcm_dev->resp_buf.data_length, rd_len);
		retval = _EIO;
		goto exit;
	}

	retval = syna_pal_mem_cpy(rd_data,
			rd_len,
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			rd_len);
	if (retval < 0) {
		LOGE("Fail to copy read data, size %d\n", rd_len);
		goto exit;
	}

exit:
	return retval;
}

/**
 * syna_tcm_read_flash_boot_config()
 *
 * Read the data of boot config area in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] reflash_data: data blob for reflash
 *    [out] rd_data:      buffer used for storing the retrieved data
 *    [ in] rd_delay_us:  delay time to access data in flash memory
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_read_flash_boot_config(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		struct tcm_buffer *rd_data, unsigned int rd_delay_us)
{
	int retval;
	unsigned int temp;
	unsigned int addr = 0;
	unsigned int length = 0;
	struct tcm_boot_info *boot_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	if (!rd_data) {
		LOGE("Invalid read data buffer\n");
		return _EINVAL;
	}

	boot_info = &tcm_dev->boot_info;

	if (IS_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("BOOT_CONFIG not available in app fw mode %d\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	temp = VALUE(boot_info->boot_config_start_block);
	addr = temp * reflash_data->write_block_size;
	length = BOOT_CONFIG_SIZE * BOOT_CONFIG_SLOTS;

	if (addr == 0 || length == 0) {
		LOGE("BOOT_CONFIG data area unavailable\n");
		retval = _EINVAL;
		goto exit;
	}

	LOGD("BOOT_CONFIG address: 0x%x, length: %d\n", addr, length);

	retval = syna_tcm_buf_alloc(rd_data, length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for rd_data buffer\n");
		goto exit;
	}

	retval = syna_tcm_read_flash(tcm_dev, addr, rd_data->buf,
			length, rd_delay_us);
	if (retval < 0) {
		LOGE("Fail to read BOOT_CONFIG area (addr: 0x%x, length: %d)\n",
			addr, length);
		goto exit;
	}

	rd_data->data_length = length;

exit:
	return retval;
}

/**
 * syna_tcm_read_flash_app_config()
 *
 * Read the data of app config area in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] reflash_data: data blob for reflash
 *    [out] rd_data:      buffer used for storing the retrieved data
 *    [ in] rd_delay_us:  delay time to access data in flash memory
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_read_flash_app_config(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		struct tcm_buffer *rd_data, unsigned int rd_delay_us)
{
	int retval;
	unsigned int temp;
	unsigned int addr = 0;
	unsigned int length = 0;
	struct tcm_application_info *app_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	if (!rd_data) {
		LOGE("Invalid read data buffer\n");
		return _EINVAL;
	}

	app_info = &tcm_dev->app_info;

	if (IS_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("APP_CONFIG not available in app fw mode %d\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	temp = VALUE(app_info->app_config_start_write_block);
	addr = temp * reflash_data->write_block_size;
	length = VALUE(app_info->app_config_size);

	if (addr == 0 || length == 0) {
		LOGE("APP_CONFIG data area unavailable\n");
		retval = _EINVAL;
		goto exit;
	}

	LOGD("APP_CONFIG address: 0x%x, length: %d\n", addr, length);

	retval = syna_tcm_buf_alloc(rd_data, length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for rd_data buffer\n");
		goto exit;
	}

	retval = syna_tcm_read_flash(tcm_dev, addr, rd_data->buf,
			length, rd_delay_us);
	if (retval < 0) {
		LOGE("Fail to read APP_CONFIG area (addr: 0x%x, length: %d)\n",
			addr, length);
		goto exit;
	}

	rd_data->data_length = length;

exit:
	return retval;
}

/**
 * syna_tcm_read_flash_disp_config()
 *
 * Read the data of display config area in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:       the device handle
 *    [ in] reflash_data:  data blob for reflash
 *    [out] rd_data:       buffer used for storing the retrieved data
 *    [ in] rd_delay_us:   delay time to access data in flash memory
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_read_flash_disp_config(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		struct tcm_buffer *rd_data, unsigned int rd_delay_us)
{
	int retval;
	unsigned int temp;
	unsigned int addr = 0;
	unsigned int length = 0;
	struct tcm_boot_info *boot_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	if (!rd_data) {
		LOGE("Invalid read data buffer\n");
		return _EINVAL;
	}

	boot_info = &tcm_dev->boot_info;

	if (IS_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("DISP_CONFIG not available in app fw mode %d\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	temp = VALUE(boot_info->display_config_start_block);
	addr = temp * reflash_data->write_block_size;
	temp = VALUE(boot_info->display_config_length_blocks);
	length = temp * reflash_data->write_block_size;

	if (addr == 0 || length == 0) {
		LOGE("DISP_CONFIG data area unavailable\n");
		retval = _EINVAL;
		goto exit;
	}

	LOGD("DISP_CONFIG address: 0x%x, length: %d\n", addr, length);

	retval = syna_tcm_buf_alloc(rd_data, length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for rd_data buffer\n");
		goto exit;
	}

	retval = syna_tcm_read_flash(tcm_dev, addr, rd_data->buf,
			length, rd_delay_us);
	if (retval < 0) {
		LOGE("Fail to read DISP_CONFIG area (addr: 0x%x, length: %d)\n",
			addr, length);
		goto exit;
	}

	rd_data->data_length = length;

exit:
	return retval;
}

/**
 * syna_tcm_read_flash_custom_otp()
 *
 * Read the data of custom OTP area in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] reflash_data: data blob for reflash
 *    [out] rd_data:      buffer used for storing the retrieved data
 *    [ in] rd_delay_us:  delay time to access data in flash memory
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_read_flash_custom_otp(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		struct tcm_buffer *rd_data, unsigned int rd_delay_us)
{
	int retval;
	unsigned int temp;
	unsigned int addr = 0;
	unsigned int length = 0;
	struct tcm_boot_info *boot_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	if (!rd_data) {
		LOGE("Invalid read data buffer\n");
		return _EINVAL;
	}

	boot_info = &tcm_dev->boot_info;

	if (IS_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("CUSTOM_OTP not available in app fw mode %d\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	temp = VALUE(boot_info->custom_otp_start_block);
	addr = temp * reflash_data->write_block_size;
	temp = VALUE(boot_info->custom_otp_length_blocks);
	length = temp * reflash_data->write_block_size;

	if (addr == 0 || length == 0) {
		LOGE("CUSTOM_OTP data area unavailable\n");
		retval = _EINVAL;
		goto exit;
	}

	LOGD("CUSTOM_OTP address: 0x%x, length: %d\n", addr, length);

	retval = syna_tcm_buf_alloc(rd_data, length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for rd_data buffer\n");
		goto exit;
	}

	retval = syna_tcm_read_flash(tcm_dev, addr, rd_data->buf,
			length, rd_delay_us);
	if (retval < 0) {
		LOGE("Fail to read CUSTOM_OTP area (addr: 0x%x, length: %d)\n",
			addr, length);
		goto exit;
	}

	rd_data->data_length = length;

exit:
	return retval;
}

/**
 * syna_tcm_read_flash_custom_data()
 *
 * Read the data of custom data in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:       the device handle
 *    [ in] reflash_data:  data blob for reflash
 *    [ in] address:       address generated by get_flash_data_location()
 *    [ in] size:          size generated by get_flash_data_location()
 *    [out] rd_data:       buffer used for storing the retrieved data
 *    [ in] rd_delay_us:   delay time to access data in flash memory
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_read_flash_custom_data(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		unsigned int address, unsigned int size,
		struct tcm_buffer *rd_data, unsigned int rd_delay_us)
{
	int retval;
	unsigned int addr = 0;
	unsigned int length = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	if (!rd_data) {
		LOGE("Invalid read data buffer\n");
		return _EINVAL;
	}

	if (IS_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Custom data not available in app fw mode %d\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	addr = address * reflash_data->write_block_size;
	length = size * reflash_data->write_block_size;

	if (addr == 0 || length == 0) {
		LOGE("Custom data area unavailable\n");
		retval = _EINVAL;
		goto exit;
	}

	LOGD("Custom data address: 0x%x, length: %d\n", addr, length);

	retval = syna_tcm_buf_alloc(rd_data, length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for rd_data buffer\n");
		goto exit;
	}

	retval = syna_tcm_read_flash(tcm_dev, addr, rd_data->buf,
			length, rd_delay_us);
	if (retval < 0) {
		LOGE("Fail to read custom data (addr: 0x%x, length: %d)\n",
			addr, length);
		goto exit;
	}

	rd_data->data_length = length;

exit:
	return retval;
}
/**
 * syna_tcm_read_flash_area()
 *
 * Entry function to read in the data of specific area in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] area:         flash area to read
 *    [out] data:         buffer storing the retrieved data
 *    [ in] rd_delay_us:  delay time in micro-sec to read words data from flash.
 *                        set 'DEFAULT_FLASH_READ_DELAY' to use default,
 *                        which is 10 us;
 *                        set '0' or 'RESP_IN_ATTN' to select ATTN-driven.
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_read_flash_area(struct tcm_dev *tcm_dev,
		enum flash_area area, struct tcm_buffer *rd_data,
		unsigned int rd_delay_us)
{
	int retval;
	unsigned int addr = 0;
	unsigned int length = 0;
	struct tcm_reflash_data_blob reflash_data;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!rd_data) {
		LOGE("Invalid data buffer\n");
		return _EINVAL;
	}

	switch (area) {
	case AREA_CUSTOM_LCM:
	case AREA_CUSTOM_OEM:
	case AREA_PPDT:
	case AREA_FORCE_TUNING:
	case AREA_OPEN_SHORT_TUNING:
		retval = syna_tcm_get_flash_data_location(tcm_dev,
				area, &addr, &length);
		if (retval < 0) {
			LOGE("Fail to get data location of 0x%x\n", area);
			return retval;
		}
		break;
	default:
		break;
	}

	retval = syna_tcm_set_up_flash_access(tcm_dev,
			&reflash_data);
	if (retval < 0) {
		LOGE("Fail to set up flash access\n");
		return retval;
	}

	syna_tcm_buf_init(&reflash_data.out);

	switch (area) {
	case AREA_BOOT_CONFIG:
		retval = syna_tcm_read_flash_boot_config(tcm_dev,
				&reflash_data, rd_data, rd_delay_us);
		if (retval < 0) {
			LOGE("Fail to get boot config data\n");
			goto exit;
		}
		break;
	case AREA_APP_CONFIG:
		retval = syna_tcm_read_flash_app_config(tcm_dev,
				&reflash_data, rd_data, rd_delay_us);
		if (retval < 0) {
			LOGE("Fail to get app config data\n");
			goto exit;
		}
		break;
	case AREA_DISP_CONFIG:
		retval = syna_tcm_read_flash_disp_config(tcm_dev,
				&reflash_data, rd_data, rd_delay_us);
		if (retval < 0) {
			LOGE("Fail to get disp config data\n");
			goto exit;
		}
		break;
	case AREA_CUSTOM_OTP:
		retval = syna_tcm_read_flash_custom_otp(tcm_dev,
				&reflash_data, rd_data, rd_delay_us);
		if (retval < 0) {
			LOGE("Fail to get custom otp data\n");
			goto exit;
		}
		break;
	case AREA_CUSTOM_LCM:
	case AREA_CUSTOM_OEM:
	case AREA_PPDT:
	case AREA_FORCE_TUNING:
	case AREA_OPEN_SHORT_TUNING:
		retval = syna_tcm_read_flash_custom_data(tcm_dev,
				&reflash_data, addr, length, rd_data,
				rd_delay_us);
		break;
	default:
		LOGE("Invalid data area\n");
		retval = _EINVAL;
		goto exit;
	}

	LOGI("%s read\n", AREA_ID_STR(area));

	retval = 0;

exit:
	retval = syna_tcm_switch_fw_mode(tcm_dev,
			MODE_APPLICATION_FIRMWARE,
			FW_MODE_SWITCH_DELAY_MS);
	if (retval < 0)
		LOGE("Fail to go back to application firmware\n");

	syna_tcm_buf_release(&reflash_data.out);

	return retval;
}

/**
 * syna_tcm_write_flash()
 *
 * Implement the bootloader command to write specified data to flash memory.
 *
 * If the length of the data to write is not an integer multiple of words,
 * the trailing byte will be discarded.  If the length of the data to write
 * is not an integer number of write blocks, it will be zero-padded to the
 * next write block.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] reflash_data: data blob for reflash
 *    [ in] address:      the address in flash memory to write
 *    [ in] wr_data:      data to write
 *    [ in] wr_len:       length of data to write
 *    [ in] wr_delay_ms:  a short delay after the command executed
 *                         set 'DEFAULT_FLASH_WRITE_DELAY' to use default,
 *                         which is 20 us;
 *                         set '0' or 'RESP_IN_ATTN' to select ATTN-driven.
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_write_flash(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		unsigned int address, const unsigned char *wr_data,
		unsigned int wr_len, unsigned int wr_delay_ms)
{
	int retval;
	unsigned int offset;
	unsigned int w_length;
	unsigned int xfer_length;
	unsigned int remaining_length;
	unsigned int flash_address;
	unsigned int block_address;
	unsigned int delay_ms;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	w_length = tcm_dev->max_wr_size - 8;

	w_length = w_length - (w_length % reflash_data->write_block_size);

	w_length = MIN(w_length, reflash_data->max_write_payload_size);

	offset = 0;

	remaining_length = wr_len;

	syna_tcm_buf_lock(&reflash_data->out);

	while (remaining_length) {
		if (remaining_length > w_length)
			xfer_length = w_length;
		else
			xfer_length = remaining_length;

		retval = syna_tcm_buf_alloc(&reflash_data->out,
				xfer_length + 2);
		if (retval < 0) {
			LOGE("Fail to allocate memory for buf.out\n");
			syna_tcm_buf_unlock(&reflash_data->out);
			return retval;
		}

		flash_address = address + offset;
		block_address = flash_address / reflash_data->write_block_size;
		reflash_data->out.buf[0] = (unsigned char)block_address;
		reflash_data->out.buf[1] = (unsigned char)(block_address >> 8);

		retval = syna_pal_mem_cpy(&reflash_data->out.buf[2],
				reflash_data->out.buf_size - 2,
				&wr_data[offset],
				wr_len - offset,
				xfer_length);
		if (retval < 0) {
			LOGE("Fail to copy write data ,size: %d\n",
				xfer_length);
			syna_tcm_buf_unlock(&reflash_data->out);
			return retval;
		}

		if (wr_delay_ms == DEFAULT_FLASH_WRITE_DELAY)
			delay_ms = FLASH_WRITE_DELAY_MS;
		else
			delay_ms = wr_delay_ms;

		if (delay_ms == RESP_IN_ATTN) {
			LOGD("xfer: %d, delay: ATTN-driven\n", xfer_length);
		} else {
			delay_ms = (delay_ms * xfer_length) / 1000;
			LOGD("xfer: %d, delay: %d\n", xfer_length, delay_ms);
		}

		retval = syna_tcm_reflash_send_command(tcm_dev,
				CMD_WRITE_FLASH,
				reflash_data->out.buf,
				xfer_length + 2,
				delay_ms);
		if (retval < 0) {
			LOGE("Fail to write data to flash addr 0x%x, size %d\n",
				flash_address, xfer_length + 2);
			syna_tcm_buf_unlock(&reflash_data->out);
			return retval;
		}

		offset += xfer_length;
		remaining_length -= xfer_length;
	}

	syna_tcm_buf_unlock(&reflash_data->out);

	return 0;
}

/**
 * syna_tcm_write_flash_block()
 *
 * Write data to the target block data area in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:       the device handle
 *    [ in] reflash_data:  data blob for reflash
 *    [ in] area:          target block area to write
 *    [ in] wr_delay_us:   delay time in micro-sec to write block data to flash.
 *                         set 'DEFAULT_FLASH_WRITE_DELAY' to use default,
 *                         which is 20 ms;
 *                         set '0' or 'RESP_IN_ATTN' to select ATTN-driven.
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_write_flash_block(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		struct block_data *block, unsigned int wr_delay_us)
{
	int retval;
	unsigned int size;
	unsigned int flash_addr;
	const unsigned char *data;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	if (!block) {
		LOGE("Invalid block data\n");
		return _EINVAL;
	}

	data = block->data;
	size = block->size;
	flash_addr = block->flash_addr;

	LOGD("Write data to %s - address: 0x%x, size: %d\n",
		AREA_ID_STR(block->id), flash_addr, size);

	if (size == 0) {
		LOGI("No need to update, size = %d\n", size);
		goto exit;
	}

	retval = syna_tcm_write_flash(tcm_dev, reflash_data,
			flash_addr, data, size, wr_delay_us);
	if (retval < 0) {
		LOGE("Fail to write %s to flash (addr: 0x%x, size: %d)\n",
			AREA_ID_STR(block->id), flash_addr, size);
		return retval;
	}

exit:
	LOGN("%s area written\n", AREA_ID_STR(block->id));

	return 0;
}

/**
 * syna_tcm_erase_flash()
 *
 * Implement the bootloader command, which is used to erase the specified
 * blocks of flash memory.
 *
 * Until this command completes, the device may be unresponsive.
 * Therefore, this helper is implemented as a blocked function, and the delay
 * time is set to 200 ms in default.
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] reflash_data:   data blob for reflash
 *    [ in] address:        the address in flash memory to read
 *    [ in] size:           size of data to write
 *    [ in] erase_delay_ms: the delay time to get the resp from mass erase
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_erase_flash(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		unsigned int address, unsigned int size,
		unsigned int erase_delay_ms)
{
	int retval;
	unsigned int page_start = 0;
	unsigned int page_count = 0;
	unsigned char out_buf[4] = {0};
	int size_erase_cmd;

	page_start = address / reflash_data->page_size;

	page_count = syna_pal_ceil_div(size, reflash_data->page_size);

	LOGD("Page start = %d (0x%04x), Page count = %d (0x%04x)\n",
		page_start, page_start, page_count, page_count);

	if ((page_start > 0xff) || (page_count > 0xff)) {
		size_erase_cmd = 4;

		out_buf[0] = (unsigned char)(page_start & 0xff);
		out_buf[1] = (unsigned char)((page_start >> 8) & 0xff);
		out_buf[2] = (unsigned char)(page_count & 0xff);
		out_buf[3] = (unsigned char)((page_count >> 8) & 0xff);
	} else {
		size_erase_cmd = 2;

		out_buf[0] = (unsigned char)(page_start & 0xff);
		out_buf[1] = (unsigned char)(page_count & 0xff);
	}

	retval = syna_tcm_reflash_send_command(tcm_dev,
			CMD_ERASE_FLASH,
			out_buf,
			size_erase_cmd,
			erase_delay_ms);
	if (retval < 0) {
		LOGE("Fail to erase data at flash page 0x%x, count %d\n",
			page_start, page_count);
		return retval;
	}

	return 0;
}

/**
 * syna_tcm_erase_flash_block()
 *
 * Mass erase the target block data area in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] reflash_data: data blob for reflash
 *    [ in] block:        target block area to erase
 *    [ in] delay_ms:     a short delay after the erase command executed
 *                        set 'DEFAULT_FLASH_ERASE_DELAY' to use default,
 *                        which is 500 ms;
 *                        set '0' or 'RESP_IN_ATTN' to select ATTN-driven.
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_erase_flash_block(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		struct block_data *block, unsigned int delay_ms)
{
	int retval;
	unsigned int size;
	unsigned int flash_addr;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	if (!block) {
		LOGE("Invalid block data\n");
		return _EINVAL;
	}

	flash_addr = block->flash_addr;

	size = block->size;

	if (delay_ms == DEFAULT_FLASH_ERASE_DELAY)
		delay_ms = FLASH_ERASE_DELAY_MS;

	LOGD("Erase %s block - address: 0x%x, size: %d\n",
		AREA_ID_STR(block->id), flash_addr, size);

	if (size == 0) {
		LOGI("No need to erase, size = %d\n", size);
		goto exit;
	}

	retval = syna_tcm_erase_flash(tcm_dev, reflash_data,
			flash_addr, size, delay_ms);
	if (retval < 0) {
		LOGE("Fail to erase %s data (addr: 0x%x, size: %d)\n",
			AREA_ID_STR(block->id), flash_addr, size);
		return retval;
	}

exit:
	LOGN("%s area erased\n", AREA_ID_STR(block->id));

	return 0;
}

/**
 * syna_tcm_update_flash_block()
 *
 * Perform the reflash sequence to the target area
 *
 * @param
 *    [ in] tcm_dev:       the device handle
 *    [ in] reflash_data:  data blob for reflash
 *    [ in] block:         target block area to update
 *    [ in] delay_ms:      a short delay time in millisecond to wait for
 *                           the completion of flash access
 *                           for polling, set a value formatted with
 *                           [erase | write];
 *                           for ATTN-driven, set a '0' or 'RESP_IN_ATTN'
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_update_flash_block(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		struct block_data *block, unsigned int delay_ms)
{
	int retval;
	unsigned int erase_delay_ms = (delay_ms >> 16) & 0xFFFF;
	unsigned int wr_blk_delay_ms = delay_ms & 0xFFFF;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash data blob\n");
		return _EINVAL;
	}

	if (!block) {
		LOGE("Invalid block data\n");
		return _EINVAL;
	}

	/* reflash is not needed for the partition */
	retval = syna_tcm_check_flash_block(tcm_dev,
			reflash_data,
			block);
	if (retval < 0) {
		LOGE("Invalid %s area\n", AREA_ID_STR(block->id));
		return retval;
	}

	if (retval == DO_NONE)
		return 0;

	LOGN("Prepare to erase %s area\n", AREA_ID_STR(block->id));

	retval = syna_tcm_erase_flash_block(tcm_dev,
			reflash_data,
			block,
			erase_delay_ms);
	if (retval < 0) {
		LOGE("Fail to erase %s area\n", AREA_ID_STR(block->id));
		return retval;
	}

	LOGN("Prepare to update %s area\n", AREA_ID_STR(block->id));

	retval = syna_tcm_write_flash_block(tcm_dev,
			reflash_data,
			block,
			wr_blk_delay_ms);
	if (retval < 0) {
		LOGE("Fail to write %s area\n", AREA_ID_STR(block->id));
		return retval;
	}

	return 0;
}

/**
 * syna_tcm_do_reflash_tddi()
 *
 * Implement the sequence specific for MODE_TDDI_BOOTLOADER.
 *
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] reflash_data:    misc. data used for fw update
 *    [ in] type:            the area to update
 *    [ in] delay_ms:        a short delay time in millisecond to wait for
 *                           the completion of flash access
 *                           for polling, set a value formatted with
 *                           [erase | write];
 *                           for ATTN-driven, set a '0' or 'RESP_IN_ATTN'
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_do_reflash_tddi(struct tcm_dev *tcm_dev,
	struct tcm_reflash_data_blob *reflash_data,
	enum update_area type, unsigned int delay_ms)
{
	int retval = 0;
	int idx;
	unsigned int erase_delay_ms = (delay_ms >> 16) & 0xFFFF;
	unsigned int wr_blk_delay_ms = delay_ms & 0xFFFF;
	struct image_info *image_info;
	struct block_data *block;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash_data blob\n");
		return _EINVAL;
	}

	image_info = &reflash_data->image_info;

	if (tcm_dev->dev_mode != MODE_TDDI_BOOTLOADER) {
		LOGE("Incorrect bootloader mode, 0x%02x, expected: 0x%02x\n",
			tcm_dev->dev_mode, MODE_TDDI_BOOTLOADER);
		return _EINVAL;
	}

	if (type == UPDATE_NONE)
		goto exit;

	/* Always mass erase all blocks, before writing the data */
	for (idx = 0; idx < AREA_MAX; idx++) {

		block = &image_info->data[idx];

		if (!block->available)
			continue;

		retval = syna_tcm_check_flash_block(tcm_dev,
				reflash_data,
				block);
		if (retval == DO_NONE)
			continue;

		LOGN("Prepare to erase %s area\n", AREA_ID_STR(block->id));

		retval = syna_tcm_erase_flash_block(tcm_dev,
				reflash_data,
				block,
				erase_delay_ms);
		if (retval < 0) {
			LOGE("Fail to erase %s area\n", AREA_ID_STR(block->id));
			goto exit;
		}
	}

	/* Write all the data to flash */
	for (idx = 0; idx < AREA_MAX; idx++) {

		block = &image_info->data[idx];

		if (!block->available)
			continue;

		retval = syna_tcm_check_flash_block(tcm_dev,
				reflash_data,
				block);
		if (retval == DO_NONE)
			continue;

		LOGN("Prepare to update %s area\n", AREA_ID_STR(block->id));

		retval = syna_tcm_write_flash_block(tcm_dev,
			reflash_data,
			block,
			wr_blk_delay_ms);
		if (retval < 0) {
			LOGE("Fail to update %s area\n",
				AREA_ID_STR(block->id));
			goto exit;
		}
	}

exit:
	return retval;
}

/**
 * syna_tcm_do_reflash_generic()
 *
 * Implement the generic sequence of fw update in MODE_BOOTLOADER.
 *
 * Typically, it is applied on most of discrete touch controllers
 *
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] reflash_data:    misc. data used for fw update
 *    [ in] type:            the area to update
 *    [ in] wait_delay_ms:   a short delay time in millisecond to wait for
 *                           the completion of flash access
 *                           for polling, set a value formatted with
 *                           [erase | write];
 *                           for ATTN-driven, set a '0' or 'RESP_IN_ATTN'
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_do_reflash_generic(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data,
		enum update_area type, unsigned int wait_delay_ms)
{
	int retval = 0;
	struct block_data *block;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!reflash_data) {
		LOGE("Invalid reflash_data blob\n");
		return _EINVAL;
	}

	if (tcm_dev->dev_mode != MODE_BOOTLOADER) {
		LOGE("Incorrect bootloader mode, 0x%02x, expected: 0x%02x\n",
			tcm_dev->dev_mode, MODE_BOOTLOADER);
		return _EINVAL;
	}

	switch (type) {
	case UPDATE_FIRMWARE_CONFIG:
		block = &reflash_data->image_info.data[AREA_APP_CODE];

		retval = syna_tcm_update_flash_block(tcm_dev,
				reflash_data,
				block,
				wait_delay_ms);
		if (retval < 0) {
			LOGE("Fail to update application firmware\n");
			goto exit;
		}
		fallthrough;
	case UPDATE_CONFIG_ONLY:
		block = &reflash_data->image_info.data[AREA_APP_CONFIG];

		retval = syna_tcm_update_flash_block(tcm_dev,
				reflash_data,
				block,
				wait_delay_ms);
		if (retval < 0) {
			LOGE("Fail to update application config\n");
			goto exit;
		}
		break;
	case UPDATE_NONE:
		fallthrough;
	default:
		break;
	}
exit:
	return retval;
}

/**
 * syna_tcm_do_fw_update()
 *
 * The entry function to perform fw update upon TouchBoot.
 *
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] image:           binary data to write
 *    [ in] image_size:      size of data array
 *    [ in] wait_delay_ms:   a short delay time in millisecond to wait for
 *                           the completion of flash access
 *                           for polling, set a value formatted with
 *                           [erase | write];
 *                           for ATTN-driven, set a '0' or 'RESP_IN_ATTN'
 *    [ in] force_reflash:   '1' to do reflash anyway
 *                           '0' to compare ID info before doing reflash.
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_do_fw_update(struct tcm_dev *tcm_dev,
		const unsigned char *image, unsigned int image_size,
		unsigned int wait_delay_ms, bool force_reflash)
{
	int retval;
	int retval_reset;
	enum update_area type = UPDATE_NONE;
	struct tcm_reflash_data_blob reflash_data;
	int app_status;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if ((!image) || (image_size == 0)) {
		LOGE("Invalid image data\n");
		return _EINVAL;
	}

	LOGN("Prepare to do reflash\n");

	syna_tcm_buf_init(&reflash_data.out);

	reflash_data.image = image;
	reflash_data.image_size = image_size;
	syna_pal_mem_set(&reflash_data.image_info, 0x00,
		sizeof(struct image_info));

	retval = syna_tcm_parse_fw_image(image, &reflash_data.image_info);
	if (retval < 0) {
		LOGE("Fail to parse firmware image\n");
		return retval;
	}

	LOGN("Start of reflash\n");

	ATOMIC_SET(tcm_dev->firmware_flashing, 1);

	app_status = syna_pal_le2_to_uint(tcm_dev->app_info.status);

	/* to forcedly update the firmware and config
	 *   - flag of 'force_reflash' has been set
	 *   - device stays in bootloader
	 *   - app firmware doesn't run properly
	 */
	force_reflash = force_reflash ||
		(IS_BOOTLOADER_MODE(tcm_dev->dev_mode)) ||
		(IS_APP_FW_MODE(tcm_dev->dev_mode) && (app_status != APP_STATUS_OK));

	if (force_reflash) {
		type = UPDATE_FIRMWARE_CONFIG;
		goto reflash;
	}

	type = (enum update_area)syna_tcm_compare_image_id_info(tcm_dev,
		&reflash_data);

	if (type == UPDATE_NONE)
		goto exit;

reflash:
	syna_tcm_buf_init(&reflash_data.out);

	/* set up flash access, and enter the bootloader mode */
	retval = syna_tcm_set_up_flash_access(tcm_dev, &reflash_data);
	if (retval < 0) {
		LOGE("Fail to set up flash access\n");
		goto exit;
	}

	/* perform the fw update */
	if (tcm_dev->dev_mode == MODE_BOOTLOADER) {
		retval = syna_tcm_do_reflash_generic(tcm_dev,
			&reflash_data,
			type,
			wait_delay_ms);
	} else if (tcm_dev->dev_mode == MODE_TDDI_BOOTLOADER) {
		retval = syna_tcm_do_reflash_tddi(tcm_dev,
			&reflash_data,
			type,
			wait_delay_ms);
	} else {
		LOGE("Incorrect bootloader mode, 0x%02x\n",
			tcm_dev->dev_mode);
		goto reset;
	}

	if (retval < 0) {
		LOGE("Fail to do firmware update\n");
		goto reset;
	}

	LOGN("End of reflash\n");

	retval = 0;
reset:
	retval_reset = syna_tcm_reset(tcm_dev);
	if (retval_reset < 0) {
		LOGE("Fail to do reset, retval_reset = %d\n", retval_reset);
		retval = retval_reset;
		goto exit;
	}

exit:
	ATOMIC_SET(tcm_dev->firmware_flashing, 0);

	syna_tcm_buf_release(&reflash_data.out);

	return retval;
}

