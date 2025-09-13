/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Common authenticated image format for Google SoCs
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GCIP_COMMON_IMAGE_HEADER_H__
#define __GCIP_COMMON_IMAGE_HEADER_H__

#include <linux/types.h>

#include "gcip-image-config.h"

#define GCIP_FW_HEADER_SIZE (0x1000)

struct gcip_common_image_sub_header {
	uint32_t magic;
	uint32_t rollbackInfo;
	uint32_t delegate_rollback_info;
	uint32_t length;
	char flags[40];
	char body_hash[64];
	char chip_id[20];
	char auth_config[256];
	struct gcip_image_config image_config;
};

struct gcip_common_image_header {
	uint32_t format_version;
	char root_sig[512];
	char root_pub[512];
	char delegate_pub[512];
	char delegate_policy[96];
	char delegate_sig[512];
	struct gcip_common_image_sub_header sub_header;
};

struct gcip_common_image_legacy_sub_header_common {
	uint32_t magic;
	uint32_t generation;
	uint32_t rollback_info;
	uint32_t length;
	uint8_t flags[16];
};

struct gcip_common_image_legacy_sub_header_gen1 {
	uint8_t body_hash[32];
	uint8_t chip_id[32];
	uint8_t auth_config[256];
	struct gcip_image_config image_config;
};

struct gcip_common_image_legacy_sub_header_gen2 {
	uint8_t body_hash[64];
	uint8_t chip_id[32];
	uint8_t auth_config[256];
	struct gcip_image_config image_config;
};

struct gcip_common_image_legacy_header {
	uint8_t sig[512];
	uint8_t pub[512];
	struct {
		struct gcip_common_image_legacy_sub_header_common common;
		union {
			struct gcip_common_image_legacy_sub_header_gen1 gen1;
			struct gcip_common_image_legacy_sub_header_gen2 gen2;
		};
	};
};

/* Checks if the magic field in the common image header matches the given magic value. */
static inline bool gcip_common_image_check_magic(const u8 *fw_data, uint32_t magic)
{
	const struct gcip_common_image_header *hdr =
		(const struct gcip_common_image_header *)fw_data;
	const struct gcip_common_image_legacy_header *legacy_hdr =
		(const struct gcip_common_image_legacy_header *)fw_data;

	return hdr->sub_header.magic == magic || legacy_hdr->common.magic == magic;
}

/*
 * Returns the image config field from a common image header or NULL if the header has an invalid
 * magic or generation identifier.
 */
static inline const struct gcip_image_config *
gcip_common_image_get_config_from_hdr(const u8 *fw_data, uint32_t magic)
{
	const struct gcip_common_image_header *hdr =
		(const struct gcip_common_image_header *)fw_data;
	const struct gcip_common_image_legacy_header *legacy_hdr =
		(const struct gcip_common_image_legacy_header *)fw_data;

	if (hdr->sub_header.magic == magic) {
		return &hdr->sub_header.image_config;
	} else if (legacy_hdr->common.magic == magic) {
		switch (legacy_hdr->common.generation) {
		case 1:
			return &legacy_hdr->gen1.image_config;
		case 2:
			return &legacy_hdr->gen2.image_config;
		default:
			return NULL;
		}
	}

	return NULL;
}

#endif  /* __GCIP_COMMON_IMAGE_HEADER_H__ */
