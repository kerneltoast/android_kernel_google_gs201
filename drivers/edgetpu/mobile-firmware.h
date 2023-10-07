/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Implements utilities for firmware management of mobile chipsets.
 *
 * Copyright (C) 2021 Google, Inc.
 */
#ifndef __MOBILE_FIRMWARE_H__
#define __MOBILE_FIRMWARE_H__

#include <linux/sizes.h>

#include "edgetpu-internal.h"
#include "edgetpu.h"

#define MAX_IOMMU_MAPPINGS 23
#define MAX_NS_IOMMU_MAPPINGS 5

#define FW_PRIV_LEVEL_GSA		(0)
#define FW_PRIV_LEVEL_TZ		(1)
#define FW_PRIV_LEVEL_NS		(2)

/* mobile FW header size */
#define MOBILE_FW_HEADER_SIZE SZ_4K
/* The offset to the signed firmware header. */
#define MOBILE_HEADER_OFFSET 0x400
/* The offset to image configuration. */
#define MOBILE_IMAGE_CONFIG_OFFSET (MOBILE_HEADER_OFFSET + 0x160)

#define CONFIG_TO_SIZE(a) ((1 << ((a) & 0xFFF)) << 12)

#define CONFIG_TO_MBSIZE(a) (((a) & 0xFFF) << 20)

struct iommu_mapping {
	/* TPU virt address */
	__u32 virt_address;
	/*
	 * contains a 12-bit aligned address and a page-order size into a
	 * 32-bit value i.e. a physical address and size in page order.
	 */
	__u32 image_config_value;
};

/*
 * The image configuration attached to the signed firmware.
 */
struct mobile_image_config {
	__u32 carveout_base;
	__u32 firmware_base;
	__u32 firmware_size;
	struct edgetpu_fw_version firmware_versions;
	__u32 config_version;
	__u32 privilege_level;
	__u32 remapped_region_start;
	__u32 remapped_region_end;
	__u32 num_iommu_mapping;
	struct iommu_mapping mappings[MAX_IOMMU_MAPPINGS];
	__u32 num_ns_iommu_mappings;
	__u32 ns_iommu_mappings[MAX_NS_IOMMU_MAPPINGS];
} __packed;

/*
 * Mobile firmware header.
 */
struct mobile_image_header {
	char sig[512];
	char pub[512];
	int Magic;
	int Generation;
	int RollbackInfo;
	int Length;
	char Flags[16];
	char BodyHash[32];
	char ChipId[32];
	char AuthConfig[256];
	struct mobile_image_config ImageConfig;
};

/* Value of Magic field above: 'TPUF' as a 32-bit LE int */
#define EDGETPU_MOBILE_FW_MAGIC	0x46555054

int edgetpu_mobile_firmware_create(struct edgetpu_dev *etdev);
void edgetpu_mobile_firmware_destroy(struct edgetpu_dev *etdev);

/*
 * Assert or release the reset signal of the TPU's CPU
 * Depending on privilege level, this may be by a direct register write
 * or a call into GSA.
 */
int edgetpu_mobile_firmware_reset_cpu(struct edgetpu_dev *etdev, bool assert_reset);

#endif /* __MOBILE_FIRMWARE_H__ */
