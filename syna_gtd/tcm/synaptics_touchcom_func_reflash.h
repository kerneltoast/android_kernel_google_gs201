/* SPDX-License-Identifier: GPL-2.0
 *
 * Synaptics TouchCom touchscreen driver
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

/*
 * @file synaptics_touchcom_func_reflash.h
 *
 * This file declares relevant functions and structures for TouchBoot.
 */

#ifndef _SYNAPTICS_TOUCHCOM_REFLASH_FUNCS_H_
#define _SYNAPTICS_TOUCHCOM_REFLASH_FUNCS_H_

#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base_flash.h"

/*
 * @section: Blocks to be updated
 */
enum update_area {
	UPDATE_NONE = 0,
	UPDATE_FIRMWARE_CONFIG,
	UPDATE_CONFIG_ONLY,
	UPDATE_ALL_BLOCKS,
};

/*
 * @section: Data Type in flash memory
 */
enum flash_data {
	FLASH_LCM_DATA = 1,
	FLASH_OEM_DATA,
	FLASH_PPDT_DATA,
	FLASH_FORCE_CALIB_DATA,
	FLASH_OPEN_SHORT_TUNING_DATA,
};

/*
 * @section: Specific data blob for reflash
 *
 * The structure contains various parameters being used in reflash
 */
struct tcm_reflash_data_blob {
	/* binary data of an image file */
	const unsigned char *image;
	unsigned int image_size;
	/* parsed data based on given image file */
	struct image_info image_info;
	/* standard information for flash access */
	unsigned int page_size;
	unsigned int write_block_size;
	unsigned int max_write_payload_size;
	/* temporary buffer during the reflash */
	struct tcm_buffer out;
};

/*
 * syna_tcm_compare_image_id_info()
 *
 * Compare the ID information between device and the image file,
 * and determine the area to be updated.
 * The function should be called after parsing the image file.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] reflash_data: data blob for reflash
 *
 * @return
 *    Comparison result
 */
int syna_tcm_compare_image_id_info(struct tcm_dev *tcm_dev,
		struct tcm_reflash_data_blob *reflash_data);

/*
 * syna_tcm_read_flash_area()
 *
 * Entry function to read in the data of specific area in the flash memory.
 *
 * @param
 *    [ in] tcm_dev:      the device handle
 *    [ in] area:         flash area to read
 *    [out] data:         buffer storing the retrieved data
 *    [ in] rd_delay_us:  delay time in micro-sec to read words data from flash.
 *                        set '0' to use default time, which is 10 us;
 *                        set 'FORCE_ATTN_DRIVEN' to adopt ATTN-driven.
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_read_flash_area(struct tcm_dev *tcm_dev,
		enum flash_area area, struct tcm_buffer *rd_data,
		unsigned int rd_delay_us);

/*
 * syna_tcm_do_fw_update()
 *
 * The entry function to perform fw update upon TouchBoot.
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
int syna_tcm_do_fw_update(struct tcm_dev *tcm_dev,
		const unsigned char *image, unsigned int image_size,
		unsigned int wait_delay_ms, bool force_reflash);

#endif /* end of _SYNAPTICS_TOUCHCOM_REFLASH_FUNCS_H_ */
