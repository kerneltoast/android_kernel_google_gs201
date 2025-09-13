// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC Firmware Loading Support
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) "aoc-fw: " fmt

#define AOC_AUTH_HEADER_MAGIC_VALUE 0x00434F41

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "aoc.h"
#include "aoc_firmware.h"
#include "aoc-interface.h"

struct aoc_auth_header {
	u8 signature[512];
	u8 key[512];
	union {
		struct {
			u32 magic;
			u32 generation;
			u32 rollback_info;
			u32 length;
			u8 flags [16];
			u8 hash [32];
			u8 chip_id [32];
			u8 auth_config [256];
			u8 image_config [256];
		} header_v1;
		struct {
			u32 magic;
			u32 generation;
			u32 rollback_info;
			u32 length;
			u8 flags [16];
			u8 hash [64];
			u8 chip_id [32];
			u8 auth_config [256];
			u8 image_config [256];
		} header_v2;
	};
} __packed;

struct aoc_superbin_header {
	u32 magic;
	u32 release_type;
	u32 _reserved0;
	u32 image_size;
	u32 bootloader_low;
	u32 bootloader_high;
	u32 bootloader_offset;
	u32 bootloader_size;
	u32 uuid_table_offset;
	u32 uuid_table_size;
	u8 version_prefix[8];
	u8 version_string[64];
	u32 section_table_offset;
	u32 section_table_entry_size;
	u32 section_table_entry_count;
	u32 sram_offset;
	u32 repo_info_offset;
	u32 a32_data_offset;
	u32 a32_data_size;
	u32 ff1_data_offset;
	u32 ff1_data_size;
	u32 hifi3z_data_offset;
	u32 hifi3z_data_size;
	u32 crc32;
} __packed;

struct aoc_image_config {
	union {
		struct {
			uint32_t version;
			struct {
				uint32_t fw_start;
				uint32_t fw_size;
				uint32_t privilege_level;
				uint16_t bl_offset;
				uint16_t bl_size;
				uint16_t iommu_offset;
				uint16_t iommu_size;
			};
		};
		uint8_t raw_bytes[256];
	};
};

static u32 aoc_img_header_size(const struct firmware *fw)
{
	if(_aoc_fw_is_signed(fw))
		return AOC_AUTH_HEADER_SIZE;

	return 0UL;
}

static bool region_is_in_firmware(size_t start, size_t length,
				  const struct firmware *fw)
{
	return ((start + length) < (fw->size - aoc_img_header_size(fw)));
}

static const struct aoc_superbin_header *superbin_header(const struct firmware* fw) {
	return (const struct aoc_superbin_header *)(fw->data + aoc_img_header_size(fw));
}

bool _aoc_fw_is_valid(const struct firmware *fw)
{
	const struct aoc_superbin_header *header;
	u32 bootloader_offset;
	u32 uuid_offset, uuid_size;

	if (!fw || !fw->data)
		return false;

	if (!region_is_in_firmware(0, sizeof(*header), fw))
		return false;

	header = superbin_header(fw);
	if (le32_to_cpu(header->magic) != 0xaabbccdd)
		return false;

	/* Validate that the AoC firmware recognizes the messages known at
	 * compile time
	 */

	uuid_offset = le32_to_cpu(header->uuid_table_offset);
	uuid_size = le32_to_cpu(header->uuid_table_size);

	if (!region_is_in_firmware(uuid_offset, uuid_size, fw)) {
		pr_err("invalid method signature region\n");
		return false;
	}

	bootloader_offset = _aoc_fw_bootloader_offset(fw);

	/* The bootloader resides within the FW image, so make sure
	 * that value makes sense
	 */
	if (!region_is_in_firmware(bootloader_offset,
				   le32_to_cpu(header->bootloader_size), fw))
		return false;

	return true;
}

bool _aoc_fw_is_release(const struct firmware *fw)
{
	const struct aoc_superbin_header *header = superbin_header(fw);

	return (le32_to_cpu(header->release_type) == 1);
}

u32 _aoc_fw_get_header_version(const struct firmware *fw)
{
	const struct aoc_auth_header *header = (const struct aoc_auth_header *)(fw->data);
	if ((le32_to_cpu(header->header_v1.generation) == 1) &&
			le32_to_cpu(header->header_v1.magic) == AOC_AUTH_HEADER_MAGIC_VALUE) {
		return 1;
	}

	if ((le32_to_cpu(header->header_v2.generation) == 2) &&
		le32_to_cpu(header->header_v2.magic) == AOC_AUTH_HEADER_MAGIC_VALUE) {
		return 2;
	}

	return 0;
}

bool _aoc_fw_is_signed(const struct firmware *fw)
{
	return _aoc_fw_get_header_version(fw) != 0;
}

struct aoc_image_config *_aoc_fw_image_config(const struct firmware *fw)
{
	const struct aoc_auth_header *header = (const struct aoc_auth_header *)(fw->data);
	u32 header_version = _aoc_fw_get_header_version(fw);

	switch (header_version) {
	case 1:
		return (struct aoc_image_config *)&header->header_v1.image_config;
	case 2:
		return (struct aoc_image_config *)&header->header_v2.image_config;
	default:
		return (struct aoc_image_config *)&header->header_v1.image_config;
	}
}

uint16_t _aoc_fw_iommu_offset(const struct firmware *fw)
{
	struct aoc_image_config *cfg = _aoc_fw_image_config(fw);

	return cfg->iommu_offset;
}

uint16_t _aoc_fw_iommu_size(const struct firmware *fw)
{
	struct aoc_image_config *cfg = _aoc_fw_image_config(fw);

	return cfg->iommu_size;
}

bool _aoc_fw_is_valid_iommu_size(const struct firmware *fw)
{
	struct aoc_image_config *cfg = _aoc_fw_image_config(fw);

	return cfg->iommu_offset < sizeof(*cfg) &&
		(cfg->iommu_offset + cfg->iommu_size) <= sizeof(*cfg) &&
		(cfg->iommu_size % sizeof(struct iommu_entry) == 0);
}

uint16_t _aoc_fw_bl_size(const struct firmware *fw)
{
	struct aoc_image_config *cfg = _aoc_fw_image_config(fw);

	return cfg->bl_size;
}

u32 *_aoc_fw_bl(const struct firmware *fw)
{
	struct aoc_image_config *cfg = _aoc_fw_image_config(fw);

	return (u32 *)((uint8_t *)cfg + cfg->bl_offset);
}

struct iommu_entry *_aoc_fw_iommu_entry(const struct firmware *fw)
{
	struct aoc_image_config *cfg = _aoc_fw_image_config(fw);

	return (struct iommu_entry *)((uint8_t *)cfg + cfg->iommu_offset);
}

bool _aoc_fw_is_compatible(const struct firmware *fw)
{
	const struct aoc_superbin_header *header = superbin_header(fw);
	u32 uuid_offset, uuid_size;

	if (!_aoc_fw_is_release(fw))
		return true;

	uuid_offset = le32_to_cpu(header->uuid_table_offset);
	uuid_size = le32_to_cpu(header->uuid_table_size);

	if (AocInterfaceCheck(fw->data + aoc_img_header_size(fw) + uuid_offset, uuid_size) != 0) {
		pr_err("failed to validate method signature table\n");
		return false;
	}

	return true;
}

u32 _aoc_fw_bootloader_offset(const struct firmware *fw)
{
	const struct aoc_superbin_header *header = superbin_header(fw);
	return le32_to_cpu(header->bootloader_offset);
}

u32 _aoc_fw_ipc_offset(const struct firmware *fw)
{
	const struct aoc_superbin_header *header = superbin_header(fw);
	return le32_to_cpu(header->image_size);
}

/* Returns firmware version, or 0 on error */
const char* _aoc_fw_version(const struct firmware *fw)
{
	const struct aoc_superbin_header *header = superbin_header(fw);
	size_t maxlen = sizeof(header->version_string);

	if (strnlen(header->version_string, maxlen) == maxlen)
		return NULL;

	return (const char *)header->version_string;
}

bool _aoc_fw_commit(const struct firmware *fw, void *dest)
{
	u32 header_size = aoc_img_header_size(fw);
	if (!_aoc_fw_is_valid(fw))
		return false;

	memcpy(dest, fw->data + header_size, fw->size - header_size);
	return true;
}
