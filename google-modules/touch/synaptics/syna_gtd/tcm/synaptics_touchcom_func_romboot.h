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
 * @file synaptics_touchcom_func_romboot.h
 *
 * This file declares relevant functions and structures for ROM boot-loader.
 */

#ifndef _SYNAPTICS_TOUCHCOM_ROMBOOT_FUNCS_H_
#define _SYNAPTICS_TOUCHCOM_ROMBOOT_FUNCS_H_

#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base_flash.h"


#define ROMBOOT_DELAY_MS (20)

/*
 * @section: Structure to assemble flash command
 */
struct flash_param {
	union {
		struct {
			unsigned char byte0;
			unsigned char byte1;
			unsigned char byte2;
		};
		struct {
			unsigned char spi_param;
			unsigned char clk_div;
			unsigned char mode;
		};
	};
	unsigned char read_size[2];
	unsigned char command;
};

/*
 * @section: Specific data blob for romboot
 *
 * The structure contains various parameters being used in
 * ROM boot control
 */
struct tcm_romboot_data_blob {
	/* binary data to write */
	const unsigned char *bdata;
	unsigned int bdata_size;
	/* parsed data based on given binary data */
	struct ihex_info ihex_info;
	struct image_info image_info;
	/* standard information for flash access */
	unsigned int page_size;
	unsigned int write_block_size;
	unsigned int max_write_payload_size;
	/* temporary buffer during the reflash */
	struct tcm_buffer out;
};


/*
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
	unsigned int delay_ms, bool is_multichip);

/*
 * syna_tcm_romboot_do_multichip_reflash()
 *
 * The entry function to perform fw update with multi-chip product.
 *
 * @param
 *    [ in] tcm_dev:         the device handle
 *    [ in] image:           image file given
 *    [ in] image_size:      size of data array
 *    [ in] wait_delay_ms:   a short delay time in millisecond to wait for
 *                           the completion of flash access
 *                           set [erase_delay_ms | write_delay_ms] for setup;
 *                           set '0' to use default time;
 *                           set 'FORCE_ATTN_DRIVEN' to adopt ATTN-driven.
 *    [ in] force_reflash:   '1' to do reflash anyway
 *                           '0' to compare ID info before doing reflash.
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_romboot_do_multichip_reflash(struct tcm_dev *tcm_dev,
	const unsigned char *image, unsigned int image_size,
	unsigned int wait_delay_ms, bool force_reflash);

/*
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
		struct tcm_romboot_info *rom_boot_info);


#endif /* end of _SYNAPTICS_TOUCHCOM_ROMBOOT_FUNCS_H_ */
