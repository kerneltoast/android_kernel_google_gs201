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
 * @file synaptics_touchcom_func_base_flash.h
 *
 * This file declares the common functions and structures being used in relevant
 * functions of fw update.
 */

#ifndef _SYNAPTICS_TOUCHCOM_PARSE_FW_FILES_H_
#define _SYNAPTICS_TOUCHCOM_PARSE_FW_FILES_H_

#include "synaptics_touchcom_core_dev.h"

/*
 * @section: Some specific definition in the firmware file
 */
#define ID_STRING_SIZE (32)

#define SIZE_WORDS (8)

#define IHEX_RECORD_SIZE (14)

#define IHEX_MAX_BLOCKS (64)

#define IMAGE_FILE_MAGIC_VALUE (0x4818472b)

#define FLASH_AREA_MAGIC_VALUE (0x7c05e516)


/*
 * @section: Helper macros for firmware file parsing
 */
#define CRC32(data, length) \
	(syna_pal_crc32(~0, data, length) ^ ~0)

#define VALUE(value) \
	(syna_pal_le2_to_uint(value))

#define AREA_ID_STR(area) \
	(syna_tcm_get_flash_area_string(area))


/*
 * @section: Area Partitions in firmware
 */
enum flash_area {
	AREA_NONE = 0,
	/* please add the declarations below */

	AREA_BOOT_CODE,
	AREA_BOOT_CONFIG,
	AREA_APP_CODE,
	AREA_APP_CODE_COPRO,
	AREA_APP_CONFIG,
	AREA_PROD_TEST,
	AREA_DISP_CONFIG,
	AREA_F35_APP_CODE,
	AREA_FORCE_TUNING,
	AREA_GAMMA_TUNING,
	AREA_TEMPERATURE_GAMM_TUNING,
	AREA_CUSTOM_LCM,
	AREA_LOOKUP,
	AREA_CUSTOM_OEM,
	AREA_OPEN_SHORT_TUNING,
	AREA_CUSTOM_OTP,
	AREA_PPDT,
	AREA_ROMBOOT_APP_CODE,
	AREA_TOOL_BOOT_CONFIG,

	/* please add the declarations above */
	AREA_MAX,
};
/*
 * @section: String of Area Partitions in firmware
 */
static char *flash_area_str[] = {
	NULL,
	/* please add the declarations below */

	"BOOT_CODE",       /* AREA_BOOT_CODE */
	"BOOT_CONFIG",     /* AREA_BOOT_CONFIG */
	"APP_CODE",        /* AREA_APP_CODE  */
	"APP_CODE_COPRO",  /* AREA_APP_CODE_COPRO */
	"APP_CONFIG",      /* AREA_APP_CONFIG */
	"APP_PROD_TEST",   /* AREA_PROD_TEST */
	"DISPLAY",         /* AREA_DISP_CONFIG  */
	"F35_APP_CODE",    /* AREA_F35_APP_CODE  */
	"FORCE",           /* AREA_FORCE_TUNING */
	"GAMMA",           /* AREA_GAMMA_TUNING */
	"TEMPERATURE_GAMM",/* AREA_TEMPERATURE_GAMM_TUNING */
	"LCM",             /* AREA_CUSTOM_LCM */
	"LOOKUP",          /* AREA_LOOKUP */
	"OEM",             /* AREA_CUSTOM_OEM */
	"OPEN_SHORT",      /* AREA_OPEN_SHORT_TUNING */
	"OTP",             /* AREA_CUSTOM_OTP */
	"PPDT",            /* AREA_PPDT */
	"ROMBOOT_APP_CODE",/* AREA_ROMBOOT_APP_CODE */
	"TOOL_BOOT_CONFIG",/* AREA_TOOL_BOOT_CONFIG */

	/* please add the declarations above */
	NULL
};
/*
 * @section: Header Content of app config defined
 *           in firmware file
 */
struct app_config_header {
	unsigned short magic_value[4];
	unsigned char checksum[4];
	unsigned char length[2];
	unsigned char build_id[4];
	unsigned char customer_config_id[16];
};
/*
 * @section: The Partition Descriptor defined
 *           in firmware file
 */
struct area_descriptor {
	unsigned char magic_value[4];
	unsigned char id_string[16];
	unsigned char flags[4];
	unsigned char flash_addr_words[4];
	unsigned char length[4];
	unsigned char checksum[4];
};
/*
 * @section: Structure for the Data Block defined
 *           in firmware file
 */
struct block_data {
	const unsigned char *data;
	unsigned int size;
	unsigned int flash_addr;
	unsigned char id;
	bool available;
};
/*
 * @section: Structure for the Parsed Image File
 */
struct image_info {
	struct block_data data[AREA_MAX];
};
/*
 * @section: Header of Image File
 *
 * Define the header of firmware image file
 */
struct image_header {
	unsigned char magic_value[4];
	unsigned char num_of_areas[4];
};
/*
 * @section: Structure for the Parsed iHex File
 */
struct ihex_info {
	unsigned int records;
	unsigned char *bin;
	unsigned int bin_size;
	struct block_data block[IHEX_MAX_BLOCKS];
};

/*
 * syna_tcm_get_flash_area_string()
 *
 * Return the string ID of target area in the flash memory
 *
 * @param
 *    [ in] area: target flash area
 *
 * @return
 *    the string ID
 */
static inline char *syna_tcm_get_flash_area_string(enum flash_area area)
{
	if (area < AREA_MAX)
		return (char *)flash_area_str[area];
	else
		return "";
}

/*
 * syna_tcm_save_flash_block_data()
 *
 * Save the block data of flash memory to the corresponding structure.
 *
 * @param
 *    [out] image_info: image info used for storing the block data
 *    [ in] area:       target area
 *    [ in] content:    content of data
 *    [ in] flash_addr: offset of block data
 *    [ in] size:       size of block data
 *    [ in] checksum:   checksum of block data
 *
 * @return
 *     on success, return 0; otherwise, negative value on error.
 */
static int syna_tcm_save_flash_block_data(struct image_info *image_info,
		enum flash_area area, const unsigned char *content,
		unsigned int offset, unsigned int size, unsigned int checksum)
{
	if (!image_info) {
		LOGE("Invalid image_info\n");
		return -ERR_INVAL;
	}

	if (area >= AREA_MAX) {
		LOGE("Invalid flash area\n");
		return -ERR_INVAL;
	}

	if (checksum != CRC32((const char *)content, size)) {
		LOGE("%s checksum error, in image: 0x%x (0x%x)\n",
			AREA_ID_STR(area), checksum,
			CRC32((const char *)content, size));
		return -ERR_INVAL;
	}
	image_info->data[area].size = size;
	image_info->data[area].data = content;
	image_info->data[area].flash_addr = offset;
	image_info->data[area].id = (unsigned char)area;
	image_info->data[area].available = true;

	LOGI("%s area - address:0x%08x (%d), size:%d\n",
		AREA_ID_STR(area), offset, offset, size);

	return 0;
}

/*
 * syna_tcm_get_flash_area_id()
 *
 * Return the corresponding ID of flash area based on the given string
 *
 * @param
 *    [ in] str: string to look for
 *
 *
 * @return
 *    if matching, return the corresponding ID; otherwise, return AREA_MAX.
 */
static enum flash_area syna_tcm_get_flash_area_id(char *str)
{
	int area;
	char *target;
	unsigned int len;

	for (area = AREA_MAX - 1; area >= 0; area--) {
		target = AREA_ID_STR(area);
		len = syna_pal_str_len(target);

		if (syna_pal_str_cmp(str, target, len) == 0)
			return area;
	}

	LOGW("Un-defined area string, %s\n", str);
	return AREA_MAX;
}

/*
 * syna_tcm_parse_fw_image()
 *
 * Parse and analyze the information of each areas from the given
 * firmware image.
 *
 * @param
 *    [ in] image:        image file given
 *    [ in] image_info:   data blob stored the parsed data from an image file
 *
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static inline int syna_tcm_parse_fw_image(const unsigned char *image,
		struct image_info *image_info)
{
	int retval = 0;
	unsigned int idx;
	unsigned int addr;
	unsigned int offset;
	unsigned int length;
	unsigned int checksum;
	unsigned int flash_addr;
	unsigned int magic_value;
	unsigned int num_of_areas;
	struct image_header *header;
	struct area_descriptor *descriptor;
	const unsigned char *content;
	enum flash_area target_area;

	if (!image) {
		LOGE("No image data\n");
		return -ERR_INVAL;
	}

	if (!image_info) {
		LOGE("Invalid image_info blob\n");
		return -ERR_INVAL;
	}

	syna_pal_mem_set(image_info, 0x00, sizeof(struct image_info));

	header = (struct image_header *)image;

	magic_value = syna_pal_le4_to_uint(header->magic_value);
	if (magic_value != IMAGE_FILE_MAGIC_VALUE) {
		LOGE("Invalid image file magic value\n");
		return -ERR_INVAL;
	}

	offset = sizeof(struct image_header);
	num_of_areas = syna_pal_le4_to_uint(header->num_of_areas);

	for (idx = 0; idx < num_of_areas; idx++) {
		addr = syna_pal_le4_to_uint(image + offset);
		descriptor = (struct area_descriptor *)(image + addr);
		offset += 4;

		magic_value = syna_pal_le4_to_uint(descriptor->magic_value);
		if (magic_value != FLASH_AREA_MAGIC_VALUE)
			continue;

		length = syna_pal_le4_to_uint(descriptor->length);
		content = (unsigned char *)descriptor + sizeof(*descriptor);
		flash_addr = syna_pal_le4_to_uint(descriptor->flash_addr_words);
		flash_addr = flash_addr * 2;
		checksum = syna_pal_le4_to_uint(descriptor->checksum);

		target_area = syna_tcm_get_flash_area_id(
			(char *)descriptor->id_string);

		retval = syna_tcm_save_flash_block_data(image_info,
				target_area,
				content,
				flash_addr,
				length,
				checksum);
		if (retval < 0)
			return retval;
	}

	return 0;
}


/*
 * syna_tcm_parse_ihex_line()
 *
 * Parse a line in the ihex file and convert into an actual data
 *
 * @param
 *    [ in] line:         a line of string stored in the ihex file
 *    [out] count:        size of actual data
 *    [out] addr:         address of data located
 *    [out] type:         the type of data belonging
 *    [out] buf:          a buffer to store the converted data
 *    [int] buf_size:     size of buffer
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static inline int syna_tcm_parse_ihex_line(char *line, unsigned int *count,
		unsigned int *addr, unsigned int *type, unsigned char *buf,
		unsigned int buf_size)
{
	const int OFFSET_COUNT = 1;
	const int SIZE_COUNT = 2;
	const int OFFSET_ADDR = OFFSET_COUNT + SIZE_COUNT;
	const int SIZE_ADDR = 4;
	const int OFFSET_TYPE = OFFSET_ADDR + SIZE_ADDR;
	const int SIZE_TYPE = 2;
	const int OFFSET_DATA = OFFSET_TYPE + SIZE_TYPE;
	const int SIZE_DATA = 2;
	unsigned int pos;

	if (!line) {
		LOGE("No string line\n");
		return -ERR_INVAL;
	}

	if ((!buf) || (buf_size == 0)) {
		LOGE("Invalid temporary data buffer\n");
		return -ERR_INVAL;
	}

	*count = syna_pal_hex_to_uint(
			line + OFFSET_COUNT, 2);
	*addr = syna_pal_hex_to_uint(
			line + OFFSET_ADDR, SIZE_ADDR);
	*type = syna_pal_hex_to_uint(
			line + OFFSET_TYPE, SIZE_TYPE);

	if (*count > buf_size) {
		LOGE("Data size mismatched, required:%d, given:%d\n",
			*count, buf_size);
		return -ERR_INVAL;
	}

	for (pos = 0; pos < *count; pos++)
		buf[pos] = (unsigned char)syna_pal_hex_to_uint(
			line + (int)((pos << 1) + OFFSET_DATA),
			SIZE_DATA);

	return 0;
}

/*
 * syna_tcm_parse_fw_ihex()
 *
 * Based on the firmware ihex file given , parse and convert into a binary
 * firmware data to update.
 *
 * @param
 *    [ in] ihex:         original ihex file
 *    [ in] ihex_size:    size of given file
 *    [ in] ihex_info:    data blob stored the parsed data from an ihex file.
 *                        assume the data buffer inside was allocated
 *    [ in] len_per_line: length for a useful data in a line
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static inline int syna_tcm_parse_fw_ihex(const char *ihex, int ihex_size,
		struct ihex_info *ihex_info,  const unsigned int len_per_line)
{
	int retval;
	unsigned int pos;
	unsigned int record;
	char *tmp = NULL;
	unsigned int count;
	unsigned int type;
	unsigned char data[32] = { 0 };
	unsigned int addr;
	unsigned int offset;
	unsigned int prev_addr;
	unsigned int block_idx = 0;

	if (!ihex) {
		LOGE("No ihex data\n");
		return -ERR_INVAL;
	}

	if (!ihex_info) {
		LOGE("Invalid ihex_info blob\n");
		return -ERR_INVAL;
	}

	if ((!ihex_info->bin) || (ihex_info->bin_size == 0)) {
		LOGE("Invalid ihex_info->data\n");
		return -ERR_INVAL;
	}

	tmp = syna_pal_mem_alloc(len_per_line + 1, sizeof(char));
	if (!tmp) {
		LOGE("Fail to allocate temporary buffer\n");
		return -ERR_NOMEM;
	}

	offset = 0;
	addr = 0;
	pos = 0;
	prev_addr = 0;

	ihex_info->records = ihex_size / len_per_line;
	LOGD("records = %d\n", ihex_info->records);

	for (record = 0; record < ihex_info->records; record++) {
		pos = record * len_per_line;
		if ((char)ihex[pos] != ':') {
			LOGE("Invalid string maker at pos %d, marker:%c\n",
				pos, (char)ihex[pos]);
			goto exit;
		}

		retval = syna_pal_mem_cpy(tmp, len_per_line,
				&ihex[pos], ihex_size - pos, len_per_line);
		if (retval < 0) {
			LOGE("Fail to copy a line at pos %d\n", pos);
			goto exit;
		}

		retval = syna_tcm_parse_ihex_line(tmp, &count, &addr, &type,
				data, sizeof(data));
		if (retval < 0) {
			LOGE("Fail to parse line at pos %d\n", pos);
			goto exit;
		}

		if ((((prev_addr + 2) & 0xFFFF) != addr) && (type == 0x00)) {
			block_idx = (record == 0) ? 0 : block_idx + 1;
			if (block_idx >= IHEX_MAX_BLOCKS) {
				LOGE("Invalid block index\n");
				goto exit;
			}

			ihex_info->block[block_idx].flash_addr =
				addr + offset;
			ihex_info->block[block_idx].data =
				&ihex_info->bin[addr + offset];
			ihex_info->block[block_idx].available = true;
		}

		if (type == 0x00) {

			prev_addr = addr;
			addr += offset;

			if (addr >= ihex_info->bin_size) {
				LOGE("No enough size for data0 addr:0x%x(%d)\n",
					addr, addr);
				goto exit;
			}
			ihex_info->bin[addr++] = data[0];

			if (addr >= ihex_info->bin_size) {
				LOGE("No enough size for data1 addr:0x%x(%d)\n",
					addr, addr);
				goto exit;
			}
			ihex_info->bin[addr++] = data[1];

			ihex_info->block[block_idx].size += 2;

		} else if (type == 0x02) {
			offset = (data[0] << 8) + data[1];
			offset <<= 4;
		}
	}

	ihex_info->bin_size = addr; /* the actual size after data reordering */
	LOGN("Size of firmware binary data = %d\n", ihex_info->bin_size);

exit:
	syna_pal_mem_free((void *)tmp);

	return 0;
}


#endif /* end of _SYNAPTICS_TOUCHCOM_PARSE_FW_FILES_H_ */
