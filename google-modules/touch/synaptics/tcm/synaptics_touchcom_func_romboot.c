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
 * @file synaptics_touchcom_func_romboot.c
 *
 * This file implements the ROM boot-loader related functions.
 * The declarations are available in synaptics_touchcom_func_romboot.h.
 */

#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_romboot.h"

#define JEDEC_STATUS_CHECK_US_MIN 5000
#define JEDEC_STATUS_CHECK_US_MAX 10000

#define BINARY_FILE_MAGIC_VALUE 0xaa55

#define ROMBOOT_FLASH_PAGE_SIZE 256

/**
 * @section: JEDEC flash command set
 */
enum flash_command {
	JEDEC_PAGE_PROGRAM = 0x02,
	JEDEC_READ_STATUS = 0x05,
	JEDEC_WRITE_ENABLE = 0x06,
	JEDEC_CHIP_ERASE = 0xc7,
};


/**
 * syna_tcm_romboot_send_command()
 *
 * Helper to send a packet to ROM bootloader.
 *
 * Please be noted that the given packet must be formatted into the
 * specific structure in order to communicate with flash.
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] out:            data sent out, if any
 *    [ in] out_size:       size of data sent out
 *    [ in] in:             buffer to store the data read in
 *    [ in] in_size:        size of data read in
 *    [ in] delay_ms_resp:  delay time to get the response
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_send_command(struct tcm_dev *tcm_dev,
		unsigned char *out, unsigned int out_size, unsigned char *in,
		unsigned int in_size, unsigned int delay_ms_resp)
{
	int retval;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (out_size < sizeof(struct flash_param)) {
		LOGE("Invalid size of out data, %d, min. size:%d\n",
			out_size, (int)sizeof(struct flash_param));
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SPI_MASTER_WRITE_THEN_READ_EXTENDED,
			out,
			out_size,
			&resp_code,
			delay_ms_resp);
	if (retval < 0) {
		LOGE("Fail to send romboot flash command 0x%02x\n",
			CMD_SPI_MASTER_WRITE_THEN_READ_EXTENDED);
		goto exit;
	}

	LOGD("resp_code: 0x%x, resp length: %d\n",
		resp_code, tcm_dev->resp_buf.data_length);

	if ((in == NULL) || (in_size < tcm_dev->resp_buf.data_length))
		goto exit;

	/* copy resp data to caller */
	retval = syna_pal_mem_cpy((unsigned char *)in,
			in_size,
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			tcm_dev->resp_buf.data_length);
	if (retval < 0) {
		LOGE("Fail to copy resp data to caller\n");
		goto exit;
	}

exit:
	return retval;
}


/**
 * syna_tcm_romboot_multichip_send_command()
 *
 * Send a command code to the ROM bootloader inside the multi-chip device
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] command:        command to send
 *    [ in] out:            additional data sent out, if any
 *    [ in] out_size:       size of data sent out
 *    [ in] in:             buffer to store the data read in
 *    [ in] in_size:        size of data read in
 *    [ in] delay_ms:       delay time to get the response
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_multichip_send_command(struct tcm_dev *tcm_dev,
		unsigned char command, unsigned char *out,
		unsigned int out_size, unsigned char *in,
		unsigned int in_size, unsigned int delay_ms)
{
	int retval;
	unsigned char *payld_buf = NULL;
	unsigned int payld_size;
	struct flash_param flash_param;
	unsigned int offset = (int)sizeof(struct flash_param);

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	syna_pal_mem_set((void *)&flash_param, 0x00, sizeof(flash_param));

	flash_param.read_size[0] = (unsigned char)(in_size & 0xff);
	flash_param.read_size[1] = (unsigned char)(in_size >> 8) & 0xff;

	flash_param.command = command;

	payld_size = offset + out_size;
	if (flash_param.command != 0x00)
		payld_size += 2;

	LOGD("Command: 0x%02x, packet size: %d, wr:%d, rd:%d\n",
		command, payld_size, out_size, in_size);

	payld_buf = syna_pal_mem_alloc(payld_size, sizeof(unsigned char));
	if (!payld_buf) {
		LOGE("Fail to allocate buffer to store flash command\n");
		return _ENOMEM;
	}

	if (flash_param.command != 0x00) {
		payld_buf[offset] = (unsigned char)out_size;
		payld_buf[offset + 1] = (unsigned char)(out_size >> 8);

		if (out_size > 0) {
			retval = syna_pal_mem_cpy(&payld_buf[offset + 2],
					payld_size - offset - 2,
					out,
					out_size,
					out_size);
			if (retval < 0) {
				LOGE("Fail to copy payload to payld_buf\n");
				goto exit;
			}
		}

		LOGD("Packet: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			flash_param.byte0, flash_param.byte1, flash_param.byte2,
			flash_param.read_size[0], flash_param.read_size[1],
			flash_param.command, payld_buf[offset],
			payld_buf[offset + 1]);
	}

	retval = syna_pal_mem_cpy(payld_buf, payld_size,
			&flash_param, sizeof(flash_param), sizeof(flash_param));
	if (retval < 0) {
		LOGE("Fail to copy flash_param header to payld_buf\n");
		goto exit;
	}

	retval = syna_tcm_romboot_send_command(tcm_dev,
			payld_buf,
			payld_size,
			in,
			in_size,
			delay_ms);
	if (retval < 0) {
		LOGE("Fail to write command 0x%x\n", flash_param.command);
		goto exit;
	}

exit:
	syna_pal_mem_free((void *)payld_buf);

	return retval;
}
/**
 * syna_tcm_romboot_multichip_get_resp()
 *
 * To get the response data from ROM bootloader inside the multi-chip device
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] length:         size to read
 *    [ in] resp:           buffer to store the resp data
 *    [ in] resp_size:      size of resp data
 *    [ in] delay_ms:       delay time to get the response
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_multichip_get_resp(struct tcm_dev *tcm_dev,
		unsigned int length, unsigned char *resp,
		unsigned int resp_size, unsigned int delay_ms)
{
	int retval;
	unsigned char *tmp_buf = NULL;
	unsigned int xfer_len;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (resp && (resp_size < length)) {
		LOGE("Invalid buffer size, len:%d, size:%d\n",
			length, resp_size);
		return _EINVAL;
	}

	xfer_len = length + 2;

	tmp_buf = syna_pal_mem_alloc(xfer_len, sizeof(unsigned char));
	if (!tmp_buf) {
		LOGE("Fail to allocate tmp_buf\n");
		return _ENOMEM;
	}

	retval = syna_tcm_romboot_multichip_send_command(tcm_dev,
			CMD_NONE, NULL, 0,
			tmp_buf, xfer_len, delay_ms);
	if (retval < 0) {
		LOGE("Fail to get resp, size: %d\n", xfer_len);
		goto exit;
	}

	if (resp) {
		retval = syna_pal_mem_cpy(resp, resp_size,
				&tmp_buf[1], xfer_len - 1, length);
		if (retval < 0) {
			LOGE("Fail to copy resp data\n");
			goto exit;
		}
	}

exit:
	syna_pal_mem_free((void *)tmp_buf);

	return retval;
}
/**
 * syna_tcm_romboot_multichip_get_status()
 *
 * To poll the status until the completion
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] resp_status:    response status returned
 *    [ in] resp_length:    response length returned
 *    [ in] delay_ms:       delay time to get the response
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_multichip_get_status(struct tcm_dev *tcm_dev,
		unsigned char *resp_status, unsigned int *resp_length,
		unsigned int delay_ms)
{
	int retval;
	unsigned char resp[4] = { 0 };
	int timeout = 0;
	int MAX_TIMEOUT = 1000;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	syna_pal_sleep_ms(delay_ms);

	do {
		retval = syna_tcm_romboot_multichip_send_command(tcm_dev,
				CMD_NONE, NULL, 0,
				resp, 3, delay_ms);
		if (retval < 0) {
			LOGE("Fail to poll the resp\n");
			goto exit;
		}

		LOGD("status: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

		if (resp[0] == 0xff) {
			syna_pal_sleep_ms(100);
			timeout += 100;
			continue;
		} else if (resp[0] == 0x01) {
			*resp_status = resp[0];
			*resp_length = syna_pal_le2_to_uint(&resp[1]);
			goto exit;
		} else {
			LOGE("Invalid resp, %02x %02x %02x\n",
				resp[0], resp[1], resp[2]);
			retval = _EIO;
			goto exit;
		}

	} while (timeout < MAX_TIMEOUT);

	if (timeout >= 500) {
		LOGE("Timeout to get the status\n");
		retval = _EIO;
	}
exit:
	return retval;
}
/**
 * syna_tcm_romboot_multichip_write_flash()
 *
 * Write the given binary data to the flash through the ROM bootloader
 * inside the multi-chip device
 *
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] romboot_data:    data blob for romboot
 *    [ in] address:         the address in flash memory to write
 *    [ in] wr_data:         binary data to write
 *    [ in] wr_len:          length of data to write
 *    [ in] wr_delay_ms:     a short delay after the command executed
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_multichip_write_flash(struct tcm_dev *tcm_dev,
		struct tcm_romboot_data_blob *romboot_data,
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
	unsigned char resp_code = 0;
	unsigned int resp_length = 0;
	unsigned int delay_ms;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	w_length = tcm_dev->max_wr_size - 16;

	w_length = w_length - (w_length % romboot_data->write_block_size);

	w_length = MIN(w_length, romboot_data->max_write_payload_size);

	offset = 0;

	remaining_length = wr_len;

	syna_tcm_buf_lock(&romboot_data->out);

	while (remaining_length) {
		if (remaining_length > w_length)
			xfer_length = w_length;
		else
			xfer_length = remaining_length;

		retval = syna_tcm_buf_alloc(&romboot_data->out,
			xfer_length + 2);
		if (retval < 0) {
			LOGE("Fail to allocate memory for buf.out\n");
			goto exit;
		}

		flash_address = address + offset;
		block_address = flash_address / romboot_data->write_block_size;

		romboot_data->out.buf[0] = (unsigned char)block_address & 0xff;
		romboot_data->out.buf[1] = (unsigned char)(block_address >> 8);

		retval = syna_pal_mem_cpy(&romboot_data->out.buf[2],
				romboot_data->out.buf_size - 2,
				&wr_data[offset],
				wr_len - offset,
				xfer_length);
		if (retval < 0) {
			LOGE("Fail to copy write data ,size: %d\n",
				xfer_length);
			goto exit;
		}

		if (wr_delay_ms == 0)
			delay_ms = ROMBOOT_DELAY_MS;
		else
			delay_ms = wr_delay_ms;

		LOGD("write xfer: %d (remaining: %d)\n",
			xfer_length, remaining_length);

		retval = syna_tcm_romboot_multichip_send_command(tcm_dev,
				CMD_WRITE_FLASH,
				romboot_data->out.buf,
				xfer_length + 2,
				NULL,
				0,
				delay_ms);
		if (retval < 0) {
			LOGE("Fail to write data to flash addr 0x%x, size %d\n",
				flash_address, xfer_length + 2);
			goto exit;
		}

		retval = syna_tcm_romboot_multichip_get_status(tcm_dev,
			&resp_code, &resp_length, ROMBOOT_DELAY_MS);
		if (retval < 0) {
			LOGE("Fail to get the response of command 0x%x\n",
				CMD_WRITE_FLASH);
			goto exit;
		}

		LOGD("status:%02x, data_length:%d\n", resp_code, resp_length);

		if (resp_code != STATUS_OK) {
			LOGE("Invalid response of command %x\n",
				CMD_WRITE_FLASH);
			retval = _EIO;
			goto exit;
		}

		retval = syna_tcm_romboot_multichip_get_resp(tcm_dev,
				resp_length, NULL, 0, ROMBOOT_DELAY_MS);
		if (retval < 0) {
			LOGE("Fail to get the boot info packet\n");
			goto exit;
		}

		offset += xfer_length;
		remaining_length -= xfer_length;
	}

	retval = 0;

exit:
	syna_tcm_buf_unlock(&romboot_data->out);

	return retval;
}
/**
 * syna_tcm_romboot_multichip_erase_flash()
 *
 * Ask the ROM bootloader to erase the flash inside the multi-chip device
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] romboot_data:   data blob for remboot
 *    [ in] address:        the address in flash memory to read
 *    [ in] size:           size of data to write
 *    [ in] erase_delay_ms: the delay time to get the resp from mass erase
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_multichip_erase_flash(struct tcm_dev *tcm_dev,
		struct tcm_romboot_data_blob *romboot_data,
		unsigned int address, unsigned int size,
		unsigned int erase_delay_ms)
{
	int retval;
	unsigned int page_start = 0;
	unsigned int page_count = 0;
	unsigned char out_buf[4] = { 0 };
	unsigned char resp_code = 0;
	unsigned int resp_length = 0;
	int size_erase_cmd;

	page_start = address / romboot_data->page_size;

	page_count = syna_pal_ceil_div(size, romboot_data->page_size);

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

	retval = syna_tcm_romboot_multichip_send_command(tcm_dev,
			CMD_ERASE_FLASH,
			out_buf,
			size_erase_cmd,
			NULL,
			0,
			erase_delay_ms);
	if (retval < 0) {
		LOGE("Fail to erase data at 0x%x (page:0x%x, count:%d)\n",
			address, page_start, page_count);
		return retval;
	}

	retval = syna_tcm_romboot_multichip_get_status(tcm_dev,
			&resp_code, &resp_length, ROMBOOT_DELAY_MS);
	if (retval < 0) {
		LOGE("Fail to get the response of command 0x%x\n",
			CMD_ERASE_FLASH);
		return retval;
	}

	LOGD("status:%02x, data_length:%d\n", resp_code, resp_length);

	if (resp_code != STATUS_OK) {
		LOGE("Invalid response of command %x\n", CMD_WRITE_FLASH);
		retval = _EIO;
		return retval;
	}

	return retval;
}

/**
 * syna_tcm_romboot_multichip_get_boot_info()
 *
 * To request a boot information packet from ROM bootloader inside
 * the multi-chip device
 *
 * @param
 *    [ in] tcm_dev:   the device handle
 *    [out] boot_info: the boot info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_multichip_get_boot_info(struct tcm_dev *tcm_dev,
	struct tcm_boot_info *boot_info)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int resp_data_len = 0;
	unsigned int copy_size;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = syna_tcm_romboot_multichip_send_command(tcm_dev,
		CMD_GET_BOOT_INFO, NULL, 0, NULL, 0, ROMBOOT_DELAY_MS);
	if (retval < 0) {
		LOGE("Fail to run command 0x%x\n", CMD_GET_BOOT_INFO);
		goto exit;
	}

	retval = syna_tcm_romboot_multichip_get_status(tcm_dev,
		&resp_code, &resp_data_len, ROMBOOT_DELAY_MS);
	if (retval < 0) {
		LOGE("Fail to get the response of command 0x%x\n",
			CMD_GET_BOOT_INFO);
		return retval;
	}

	LOGD("status:%02x, data_length:%d\n", resp_code, resp_data_len);

	if (resp_code != STATUS_OK) {
		LOGE("Invalid response of command %x\n", CMD_GET_BOOT_INFO);
		retval = _EIO;
		return retval;
	}

	if (boot_info == NULL)
		goto exit;

	copy_size = MIN(sizeof(struct tcm_boot_info), resp_data_len);

	retval = syna_tcm_romboot_multichip_get_resp(tcm_dev,
		copy_size, (unsigned char *)boot_info,
		sizeof(struct tcm_boot_info), ROMBOOT_DELAY_MS);
	if (retval < 0) {
		LOGE("Fail to get the boot info packet\n");
		return retval;
	}

exit:
	return retval;
}
/**
 * syna_tcm_romboot_preparation()
 *
 * Perform the preparation before doing firmware update of multi-chip device
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [out] romboot_data: data blob for romboot access
 *    [ in] is_multichip: flag to indicate a multichip DUT
 *
 * @return
 *    Result of image file comparison
 */
static int syna_tcm_romboot_preparation(struct tcm_dev *tcm_dev,
		struct tcm_romboot_data_blob *romboot_data, bool is_multichip)
{
	int retval;
	unsigned int temp;
	struct tcm_boot_info *boot_info;
	unsigned int wr_chunk;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!romboot_data) {
		LOGE("Invalid romboot data blob\n");
		return _EINVAL;
	}

	LOGI("Set up preparation, multi-chip: %s\n",
		(is_multichip)?"yes":"no");

	retval = syna_tcm_identify(tcm_dev, NULL);
	if (retval < 0) {
		LOGE("Fail to do identification\n");
		return retval;
	}

	/* switch to bootloader mode */
	if (IS_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGI("Prepare to enter bootloader mode\n");
		if (is_multichip)
			retval = syna_tcm_switch_fw_mode(tcm_dev,
				MODE_MULTICHIP_TDDI_BOOTLOADER,
				FW_MODE_SWITCH_DELAY_MS);
		else
			retval = syna_tcm_switch_fw_mode(tcm_dev,
				MODE_TDDI_BOOTLOADER,
				FW_MODE_SWITCH_DELAY_MS);

		if (retval < 0) {
			LOGE("Fail to enter bootloader mode\n");
			return retval;
		}
	}
	/* switch to rom boot mode */
	if (!IS_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGI("Prepare to enter rom boot mode\n");

		retval = syna_tcm_switch_fw_mode(tcm_dev,
				MODE_ROMBOOTLOADER,
				FW_MODE_SWITCH_DELAY_MS);
		if (retval < 0) {
			LOGE("Fail to enter rom boot mode\n");
			return retval;
		}
	}

	if (!IS_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Device not in romboot mode\n");
		return _EINVAL;
	}

	if (!is_multichip)
		return 0;

	boot_info = &tcm_dev->boot_info;

	retval = syna_tcm_romboot_multichip_get_boot_info(tcm_dev,
			boot_info);
	if (retval < 0) {
		LOGE("Fail to get boot info\n");
		return retval;
	}

	wr_chunk = tcm_dev->max_wr_size;

	temp = boot_info->write_block_size_words;
	romboot_data->write_block_size = temp * 2;

	LOGI("Write block size: %d (words size: %d)\n",
		romboot_data->write_block_size, temp);

	temp = syna_pal_le2_to_uint(boot_info->erase_page_size_words);
	romboot_data->page_size = temp * 2;

	LOGI("Erase page size: %d (words size: %d)\n",
		romboot_data->page_size, temp);

	temp = syna_pal_le2_to_uint(boot_info->max_write_payload_size);
	romboot_data->max_write_payload_size = temp;

	LOGI("Max write flash data size: %d\n",
		romboot_data->max_write_payload_size);

	if (romboot_data->write_block_size > (wr_chunk - 9)) {
		LOGE("Write block size, %d, greater than chunk space, %d\n",
			romboot_data->write_block_size, (wr_chunk - 9));
		return _EINVAL;
	}

	if (romboot_data->write_block_size == 0) {
		LOGE("Invalid write block size %d\n",
			romboot_data->write_block_size);
		return _EINVAL;
	}

	if (romboot_data->page_size == 0) {
		LOGE("Invalid erase page size %d\n",
			romboot_data->page_size);
		return _EINVAL;
	}

	return 0;
}

/**
 * syna_tcm_romboot_jedec_send_command()
 *
 * Send a jedec flash commend to the ROM bootloader
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] flash_command:  flash command to send
 *    [ in] out:            additional data sent out, if any
 *    [ in] out_size:       size of data sent out
 *    [ in] in:             buffer to store the data read in
 *    [ in] in_size:        size of data read in
 *    [ in] delay_ms:       delay time to get the response
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_jedec_send_command(struct tcm_dev *tcm_dev,
		unsigned char flash_command, unsigned char *out,
		unsigned int out_size, unsigned char *in, unsigned int in_size,
		unsigned int delay_ms)
{
	int retval;
	unsigned char *payld_buf = NULL;
	unsigned int payld_size;
	struct flash_param flash_param;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	syna_pal_mem_set((void *)&flash_param, 0x00, sizeof(flash_param));

	flash_param.spi_param = 1;
	flash_param.clk_div = 0x19;

	flash_param.read_size[0] = (unsigned char)(in_size & 0xff);
	flash_param.read_size[1] = (unsigned char)(in_size >> 8) & 0xff;

	flash_param.command = flash_command;

	payld_size = sizeof(struct flash_param) + out_size;

	LOGD("Flash command: 0x%02x, total size: %d, wr: %d, rd: %d\n",
		flash_command, payld_size, out_size, in_size);
	LOGD("Packet: %02x %02x %02x %02x %02x %02x\n",
		flash_param.byte0, flash_param.byte1, flash_param.byte2,
		flash_param.read_size[0], flash_param.read_size[1],
		flash_param.command);

	payld_buf = syna_pal_mem_alloc(payld_size, sizeof(unsigned char));
	if (!payld_buf) {
		LOGE("Fail to allocate buffer to store flash command\n");
		return _ENOMEM;
	}

	retval = syna_pal_mem_cpy(payld_buf, payld_size,
			&flash_param, sizeof(flash_param), sizeof(flash_param));
	if (retval < 0) {
		LOGE("Fail to copy flash_param header to payld_buf\n");
		goto exit;
	}

	if (out && (out_size > 0)) {
		retval = syna_pal_mem_cpy(payld_buf + sizeof(flash_param),
				payld_size - sizeof(flash_param),
				out, out_size, out_size);
		if (retval < 0) {
			LOGE("Fail to copy data to payld_buf\n");
			goto exit;
		}
	}

	retval = syna_tcm_romboot_send_command(tcm_dev,
			payld_buf,
			payld_size,
			in,
			in_size,
			delay_ms);
	if (retval < 0) {
		LOGE("Fail to write flash command 0x%x\n", flash_command);
		goto exit;
	}

exit:
	syna_pal_mem_free((void *)payld_buf);

	return retval;
}

/**
 * syna_tcm_romboot_jedec_get_status()
 *
 * Use jedec command to poll the flash status until the completion
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] delay_ms:       delay time to get the response
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_jedec_get_status(struct tcm_dev *tcm_dev,
		unsigned int delay_ms)
{
	int retval;
	int idx;
	unsigned char status;
	int STATUS_CHECK_RETRY = 50;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	for (idx = 0; idx < STATUS_CHECK_RETRY; idx++) {
		retval = syna_tcm_romboot_jedec_send_command(tcm_dev,
				JEDEC_READ_STATUS,
				NULL,
				0,
				&status,
				sizeof(status),
				delay_ms);
		if (retval < 0) {
			LOGE("Failed to write JEDEC_READ_STATUS\n");
			return retval;
		}

		syna_pal_sleep_us(JEDEC_STATUS_CHECK_US_MIN,
				JEDEC_STATUS_CHECK_US_MAX);
		/* once completed, status = 0 */
		if (!status)
			break;
	}

	if (status)
		retval = _EIO;
	else
		retval = status;

	return retval;
}

/**
 * syna_tcm_romboot_jedec_erase_flash()
 *
 * Ask the ROM bootloader to erase the flash by using the jedec command
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] delay_ms:       delay time to get the response
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_jedec_erase_flash(struct tcm_dev *tcm_dev,
		unsigned int delay_ms)
{
	int retval;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = syna_tcm_romboot_jedec_send_command(tcm_dev,
			JEDEC_WRITE_ENABLE,
			NULL,
			0,
			NULL,
			0,
			delay_ms);
	if (retval < 0) {
		LOGE("Failed to write JEDEC_WRITE_ENABLE\n");
		return retval;
	}

	retval = syna_tcm_romboot_jedec_send_command(tcm_dev,
			JEDEC_CHIP_ERASE,
			NULL,
			0,
			NULL,
			0,
			delay_ms);
	if (retval < 0) {
		LOGE("Failed to write JEDEC_WRITE_ENABLE\n");
		return retval;
	}

	retval = syna_tcm_romboot_jedec_get_status(tcm_dev, delay_ms);
	if (retval < 0)
		LOGE("Fail to get correct status, retval = %d\n", retval);

	return retval;
}

 /**
  * syna_tcm_romboot_jedec_write_flash()
  *
  * Write the given binary data to the flash through the ROM bootloader
  * by using the jedec command
  *
  * @param
  *    [ in] tcm_dev:         the device handle
  *    [ in] address:         the address in flash memory to write
  *    [ in] data:            binary data to write
  *    [ in] data_size:       size of binary data
  *    [ in] delay_ms:        a short delay time in millisecond to wait for
  *                           the completion of flash access
  *
  * @return
  *    on success, 0 or positive value; otherwise, negative value on error.
  */
static int syna_tcm_romboot_jedec_write_flash(struct tcm_dev *tcm_dev,
		unsigned int address, const unsigned char *data,
		unsigned int data_size, unsigned int delay_ms)
{
	int retval = 0;
	unsigned int offset;
	unsigned char buf[ROMBOOT_FLASH_PAGE_SIZE + 3];
	unsigned int remaining_length;
	unsigned int xfer_length;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if ((!data) || (data_size == 0)) {
		LOGE("Invalid image data, no data available\n");
		return _EINVAL;
	}

	remaining_length = data_size;

	offset = 0;

	while (remaining_length) {
		if (remaining_length > ROMBOOT_FLASH_PAGE_SIZE)
			xfer_length = ROMBOOT_FLASH_PAGE_SIZE;
		else
			xfer_length = remaining_length;

		syna_pal_mem_set(buf, 0x00, sizeof(buf));

		retval = syna_tcm_romboot_jedec_send_command(tcm_dev,
				JEDEC_WRITE_ENABLE,
				NULL,
				0,
				NULL,
				0,
				delay_ms);
		if (retval < 0) {
			LOGE("Failed to write JEDEC_WRITE_ENABLE\n");
			goto exit;
		}

		buf[0] = (unsigned char)((address + offset) >> 16);
		buf[1] = (unsigned char)((address + offset) >> 8);
		buf[2] = (unsigned char)(address + offset);

		retval = syna_pal_mem_cpy(&buf[3],
				sizeof(buf) - 3,
				&data[offset],
				data_size - offset,
				xfer_length);
		if (retval < 0) {
			LOGE("Fail to copy data to write, size: %d\n",
				xfer_length);
			goto exit;
		}

		retval = syna_tcm_romboot_jedec_send_command(tcm_dev,
				JEDEC_PAGE_PROGRAM,
				buf,
				sizeof(buf),
				NULL,
				0,
				delay_ms);
		if (retval < 0) {
			LOGE("Failed to write data to addr 0x%x (offset: %x)\n",
				address + offset, offset);
			LOGE("Remaining data %d\n",
				remaining_length);
			goto exit;
		}

		retval = syna_tcm_romboot_jedec_get_status(tcm_dev, delay_ms);
		if (retval < 0) {
			LOGE("Fail to get correct status, retval = %d\n",
				retval);
			goto exit;
		}
		offset += xfer_length;
		remaining_length -= xfer_length;
	}

exit:
	return retval;
}


/**
 * syna_tcm_romboot_erase_flash()
 *
 * The entry function to perform mass erase
 *
 * @param
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] romboot_data:    data blob for remboot
 *    [ in] blk:             the block in flash memory to erase
 *    [ in] delay_ms:        delay time to get the response
 *    [ in] is_multichip:    use multi-chip command packet instead
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_erase_flash(struct tcm_dev *tcm_dev,
		struct tcm_romboot_data_blob *romboot_data,
		struct block_data *blk, unsigned int delay_ms,
		bool is_multichip)
{
	int retval;

	if (!tcm_dev || !romboot_data || !blk)
		return _EINVAL;

	if (is_multichip)
		retval = syna_tcm_romboot_multichip_erase_flash(tcm_dev,
				romboot_data, blk->flash_addr, blk->size,
				delay_ms);
	else
		retval = syna_tcm_romboot_jedec_erase_flash(tcm_dev,
				delay_ms);

	return retval;
}

/**
 * syna_tcm_romboot_write_flash()
 *
 * The entry function to write hex data to flash
 *
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] romboot_data:    data blob for remboot
 *    [ in] blk:             the block in flash memory to update
 *    [ in] delay_ms:        a short delay time in millisecond to wait for
 *                           the completion of flash access
 *    [ in] is_multichip:    use multi-chip command packet instead
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_romboot_write_flash(struct tcm_dev *tcm_dev,
		struct tcm_romboot_data_blob *romboot_data,
		struct block_data *blk, unsigned int delay_ms,
		bool is_multichip)
{
	int retval;

	if (!tcm_dev || !romboot_data || !blk)
		return _EINVAL;

	if (is_multichip)
		retval = syna_tcm_romboot_multichip_write_flash(tcm_dev,
				romboot_data, blk->flash_addr, blk->data,
				blk->size, delay_ms);
	else
		retval = syna_tcm_romboot_jedec_write_flash(tcm_dev,
				blk->flash_addr, blk->data, blk->size,
				delay_ms);

	return retval;
}


/**
 * syna_tcm_romboot_do_ihex_update()
 *
 * The entry function to perform ihex update upon ROM Boot.
 *
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] ihex:            ihex data to write
 *    [ in] ihex_size:       size of ihex data
 *    [ in] flash_size:      size for temporary buffer allocation, which used
 *                           to re-order the flash data.
 *                           in general, (ihex_size + 4K) is preferred size
 *    [ in] len_per_line:    length per line in the ihex file
 *    [ in] delay_ms:        a short delay time in millisecond to wait for
 *                           the completion of flash access
 *    [ in] is_multichip:    flag to indicate a multi-chip product used
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_romboot_do_ihex_update(struct tcm_dev *tcm_dev,
		const unsigned char *ihex, unsigned int ihex_size,
		unsigned int flash_size, unsigned int len_per_line,
		unsigned int delay_ms, bool is_multichip)
{
	int retval;
	struct tcm_romboot_data_blob *romboot_data = NULL;
	struct ihex_info *ihex_info = NULL;
	struct block_data *block;
	unsigned int erase_delay_ms = (delay_ms >> 16) & 0xFFFF;
	unsigned int wr_delay_ms = delay_ms & 0xFFFF;
	unsigned short *header;
	int idx;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if ((!ihex) || (ihex_size == 0)) {
		LOGE("Invalid ihex data\n");
		return _EINVAL;
	}

	if (flash_size == 0)
		flash_size = ihex_size + 4096;

	romboot_data = syna_pal_mem_alloc(1,
			sizeof(struct tcm_romboot_data_blob));
	if (!romboot_data) {
		LOGE("Fail to allocate romboot data blob\n");

		return _ENOMEM;
	}

	romboot_data->bdata = ihex;
	romboot_data->bdata_size = ihex_size;
	syna_pal_mem_set(&romboot_data->ihex_info, 0x00,
		sizeof(struct ihex_info));

	ihex_info = &romboot_data->ihex_info;

	ihex_info->bin = syna_pal_mem_alloc(flash_size,
			sizeof(unsigned char));
	if (!ihex_info->bin) {
		LOGE("Fail to allocate buffer for ihex data\n");
		syna_pal_mem_free((void *)romboot_data);

		return _ENOMEM;
	}

	ihex_info->bin_size = flash_size;

	syna_tcm_buf_init(&romboot_data->out);

	LOGN("Parse ihex file\n");

	/* parse ihex file */
	retval = syna_tcm_parse_fw_ihex((const char *)ihex,
			ihex_size, ihex_info, len_per_line);
	if (retval < 0) {
		LOGE("Fail to parse firmware ihex file\n");
		goto exit;
	}

	if (!is_multichip) {
		header = (unsigned short *)ihex_info->block[0].data;
		if (*header != BINARY_FILE_MAGIC_VALUE) {
			LOGE("Incorrect image header 0x%04X\n", *header);
			goto exit;
		}
	}

	/* set up flash access, and enter the bootloader mode */
	retval = syna_tcm_romboot_preparation(tcm_dev,
			romboot_data,
			is_multichip);
	if (retval < 0) {
		LOGE("Fail to do preparation\n");
		goto reset;
	}

	LOGN("Start of ihex update\n");

	ATOMIC_SET(tcm_dev->firmware_flashing, 1);

	for (idx = 0; idx < IHEX_MAX_BLOCKS; idx++) {

		block = &ihex_info->block[idx];

		if (!block->available)
			continue;

		if (block->size == 0)
			continue;

		/* for single-chip, mass erase will affect the
		 * entire flash memory, so it just takes once
		 */
		if ((idx != 0) && (!is_multichip))
			break;

		retval = syna_tcm_romboot_erase_flash(tcm_dev,
				romboot_data,
				block,
				erase_delay_ms,
				is_multichip);
		if (retval < 0) {
			LOGE("Fail to erase flash\n");
			goto reset;
		}
	}

	LOGN("Flash erased\n");
	LOGN("Start to write all data to flash\n");

	for (idx = 0; idx < IHEX_MAX_BLOCKS; idx++) {

		block = &ihex_info->block[idx];

		if (!block->available)
			continue;

		LOGD("block:%d, addr:0x%x, size:%d\n",
			idx, block->flash_addr, block->size);

		if (block->size == 0)
			continue;

		retval = syna_tcm_romboot_write_flash(tcm_dev,
				romboot_data,
				block,
				wr_delay_ms,
				is_multichip);
		if (retval < 0) {
			LOGE("Fail to write data to addr 0x%x, size:%d\n",
				block->flash_addr, block->size);
			goto reset;
		}

		LOGI("Data written, size:%d\n", block->size);
	}

	LOGN("End of ihex update\n");

	retval = 0;

reset:
	retval = syna_tcm_reset(tcm_dev);
	if (retval < 0) {
		LOGE("Fail to do reset\n");
		goto exit;
	}

exit:
	syna_pal_mem_free((void *)ihex_info->bin);

	ATOMIC_SET(tcm_dev->firmware_flashing, 0);

	syna_tcm_buf_release(&romboot_data->out);

	syna_pal_mem_free((void *)romboot_data);

	return retval;
}

/**
 * syna_tcm_romboot_do_multichip_reflash()
 *
 * The entry function to perform fw update in multi-chip product.
 *
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] image:           image file given
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
int syna_tcm_romboot_do_multichip_reflash(struct tcm_dev *tcm_dev,
		const unsigned char *image, unsigned int image_size,
		unsigned int wait_delay_ms, bool force_reflash)
{
	int retval;
	int idx;
	struct tcm_romboot_data_blob *romboot_data = NULL;
	struct block_data *block;
	struct app_config_header *header;
	unsigned int image_fw_id;
	unsigned char *image_config_id;
	unsigned char *device_config_id;
	unsigned int erase_delay_ms = (wait_delay_ms >> 16) & 0xFFFF;
	unsigned int wr_delay_ms = wait_delay_ms & 0xFFFF;
	bool has_tool_boot_cfg = false;
	bool reflash_required = false;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if ((!image) || (image_size == 0)) {
		LOGE("Invalid image data\n");
		return _EINVAL;
	}

	romboot_data = syna_pal_mem_alloc(1,
			sizeof(struct tcm_romboot_data_blob));
	if (!romboot_data) {
		LOGE("Fail to allocate romboot data blob\n");

		return _ENOMEM;
	}

	LOGN("Prepare to do reflash\n");

	syna_tcm_buf_init(&romboot_data->out);

	romboot_data->bdata = image;
	romboot_data->bdata_size = image_size;

	syna_tcm_buf_init(&romboot_data->out);

	retval = syna_tcm_parse_fw_image(image, &romboot_data->image_info);
	if (retval < 0) {
		LOGE("Fail to parse firmware image\n");
		retval = _EINVAL;
		goto exit;
	}

	block = &romboot_data->image_info.data[AREA_APP_CONFIG];
	if (block->size < sizeof(struct app_config_header)) {
		LOGE("Invalid application config in image file\n");
		retval = _EINVAL;
		goto exit;
	}
	header = (struct app_config_header *)block->data;

	image_fw_id = syna_pal_le4_to_uint(header->build_id);

	LOGN("Device firmware ID: %d, image build id: %d\n",
		tcm_dev->packrat_number, image_fw_id);

	if (image_fw_id != tcm_dev->packrat_number) {
		LOGN("Image build ID and device fw ID mismatched\n");
		reflash_required = true;
	}

	image_config_id = header->customer_config_id;
	device_config_id = tcm_dev->app_info.customer_config_id;

	for (idx = 0; idx < MAX_SIZE_CONFIG_ID; idx++) {
		if (image_config_id[idx] != device_config_id[idx]) {
			LOGN("Different Config ID\n");
			reflash_required = true;
		}
	}
	/* to start the process of firmware update
	 *   - fw ID or config ID is mismatched
	 *   - device stays in rom-bootloader
	 *   - flag of 'force_reflash' has been set
	 */
	reflash_required = reflash_required ||
		(IS_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) ||
		force_reflash;

	if (!reflash_required) {
		LOGN("No need to do reflash\n");
		retval = 0;
		goto exit;
	}

	block = &romboot_data->image_info.data[AREA_TOOL_BOOT_CONFIG];
	has_tool_boot_cfg = block->available;

	/* set up flash access, and enter the bootloader mode */
	retval = syna_tcm_romboot_preparation(tcm_dev, romboot_data, true);
	if (retval < 0) {
		LOGE("Fail to do preparation\n");
		goto reset;
	}

	if (!IS_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Incorrect device mode 0x%02x, expected:0x%02x\n",
			tcm_dev->dev_mode, MODE_ROMBOOTLOADER);
		retval = _EINVAL;
		goto reset;
	}

	LOGN("Start of reflash\n");

	ATOMIC_SET(tcm_dev->firmware_flashing, 1);

	/* Traverse through all blocks in the image file,
	 * then erase the corresponding block area
	 */
	for (idx = 0; idx < AREA_MAX; idx++) {

		block = &romboot_data->image_info.data[idx];

		if (!block->available)
			continue;

		if (idx == AREA_ROMBOOT_APP_CODE)
			continue;

		if ((idx == AREA_BOOT_CONFIG) && has_tool_boot_cfg)
			continue;

		LOGD("Erase %s block - address: 0x%x (%d), size: %d\n",
			AREA_ID_STR(block->id), block->flash_addr,
			block->flash_addr, block->size);

		if (block->size == 0)
			continue;

		if (erase_delay_ms == DEFAULT_FLASH_ERASE_DELAY)
			erase_delay_ms = ROMBOOT_DELAY_MS;

		retval = syna_tcm_romboot_erase_flash(tcm_dev,
				romboot_data,
				block,
				erase_delay_ms,
				true);
		if (retval < 0) {
			LOGE("Fail to erase %s area\n", AREA_ID_STR(block->id));
			goto reset;
		}

		LOGN("%s partition erased\n", AREA_ID_STR(block->id));
	}

	/* Traverse through all blocks in the image file,
	 * and then update the corresponding block area
	 */
	for (idx = 0; idx < AREA_MAX; idx++) {

		LOGD("Prepare to update %s partition\n", AREA_ID_STR(idx));

		block = &romboot_data->image_info.data[idx];

		if (!block->available)
			continue;

		if (idx == AREA_ROMBOOT_APP_CODE)
			continue;

		if ((idx == AREA_BOOT_CONFIG) && has_tool_boot_cfg)
			continue;

		LOGD("Write data to %s - address: 0x%x (%d), size: %d\n",
			AREA_ID_STR(block->id), block->flash_addr,
			block->flash_addr, block->size);

		if (block->size == 0)
			continue;

		if (wr_delay_ms == DEFAULT_FLASH_WRITE_DELAY)
			wr_delay_ms = ROMBOOT_DELAY_MS;

		retval = syna_tcm_romboot_write_flash(tcm_dev,
				romboot_data,
				block,
				wr_delay_ms,
				true);

		if (retval < 0) {
			LOGE("Fail to update %s partition, size: %d\n",
				AREA_ID_STR(block->id), block->size);
			goto reset;
		}

		LOGN("%s written\n", AREA_ID_STR(block->id));
	}

	LOGN("End of reflash\n");

	retval = 0;
reset:
	retval = syna_tcm_reset(tcm_dev);
	if (retval < 0) {
		LOGE("Fail to do reset\n");
		goto exit;
	}

exit:
	ATOMIC_SET(tcm_dev->firmware_flashing, 0);

	syna_tcm_buf_release(&romboot_data->out);

	syna_pal_mem_free(romboot_data);

	return retval;
}


/**
 * syna_tcm_get_romboot_info()
 *
 * Implement the bootloader command code, which is used to request a
 * RomBoot information packet.
 *
 * @param
 *    [ in] tcm_dev:       the device handle
 *    [out] rom_boot_info: the romboot info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_romboot_info(struct tcm_dev *tcm_dev,
	struct tcm_romboot_info *rom_boot_info)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int copy_size = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_ROMBOOT_INFO,
			NULL,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_ROMBOOT_INFO);
		goto exit;
	}

	if (rom_boot_info == NULL)
		goto exit;

	copy_size = MIN(sizeof(struct tcm_romboot_info),
			tcm_dev->resp_buf.data_length);

	/* copy romboot_info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)rom_boot_info,
			sizeof(struct tcm_romboot_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
		copy_size);
	if (retval < 0) {
		LOGE("Fail to copy romboot info to caller\n");
		goto exit;
	}

exit:
	return retval;
}



