/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Framework for parsing the firmware image configuration.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GCIP_IMAGE_CONFIG_H__
#define __GCIP_IMAGE_CONFIG_H__

#include <asm/page.h>
#include <linux/bits.h>
#include <linux/types.h>

#define GCIP_FW_NUM_VERSIONS 4
#define GCIP_IMG_CFG_MAX_IOMMU_MAPPINGS 22
#define GCIP_IMG_CFG_MAX_NS_IOMMU_MAPPINGS 5

#define GCIP_FW_PRIV_LEVEL_GSA 0
#define GCIP_FW_PRIV_LEVEL_TZ 1
#define GCIP_FW_PRIV_LEVEL_NS 2

/*
 * The image configuration attached to the signed firmware.
 */
struct gcip_image_config {
	__u32 carveout_base;
	__u32 firmware_base;
	__u32 firmware_size;
	__u32 firmware_versions[GCIP_FW_NUM_VERSIONS];
	__u32 config_version;
	__u32 privilege_level;
	__u32 remapped_region_start;
	__u32 remapped_region_size;
	__u32 num_iommu_mappings;
	struct {
		/* Device virtual address */
		__u32 virt_address;
		/*
		 * Encodes a 12-bit aligned address and the corresponding size
		 * into a 32-bit value.
		 * Detailed encoding method is defined in gcip-image-config.c.
		 */
		__u32 image_config_value;
	} iommu_mappings[GCIP_IMG_CFG_MAX_IOMMU_MAPPINGS];
	__u32 remapped_data_start;
	__u32 remapped_data_size;
	__u32 num_ns_iommu_mappings;
	__u32 ns_iommu_mappings[GCIP_IMG_CFG_MAX_NS_IOMMU_MAPPINGS];
} __packed;

#define GCIP_IMAGE_CONFIG_FLAGS_SECURE (1u << 0)

struct gcip_image_config_ops {
	/*
	 * Adds an IOMMU mapping from @daddr to @paddr with size @size.
	 *
	 * It is ensured that there is no overflow on @paddr + @size before calling this function.
	 *
	 * @flags is a bit-field with the following attributes:
	 *   [0:0] - Security. 1 for secure and 0 for non-secure.
	 *   [31:1] - Reserved.
	 *
	 * Returns 0 on success. Otherwise a negative errno.
	 * Mandatory.
	 */
	int (*map)(void *data, dma_addr_t daddr, phys_addr_t paddr, size_t size,
		   unsigned int flags);
	/*
	 * Removes the IOMMU mapping previously added by @map.
	 *
	 * Mandatory.
	 */
	void (*unmap)(void *data, dma_addr_t daddr, size_t size, unsigned int flags);
};

struct gcip_image_config_parser {
	struct device *dev;
	void *data; /* User-specify data, will be passed to ops. */
	const struct gcip_image_config_ops *ops;
	/* The last image config being successfully parsed. */
	struct gcip_image_config last_config;
};

#define GCIP_IMG_CFG_ADDR_SHIFT 12
#define GCIP_IMG_CFG_MB_SHIFT 20
#define GCIP_IMG_CFG_SIZE_MODE_BIT BIT(GCIP_IMG_CFG_ADDR_SHIFT - 1)
#define GCIP_IMG_CFG_SECURE_SIZE_MASK (GCIP_IMG_CFG_SIZE_MODE_BIT - 1u)
#define GCIP_IMG_CFG_NS_SIZE_MASK (GCIP_IMG_CFG_SIZE_MODE_BIT - 1u)
#define GCIP_IMG_CFG_ADDR_MASK ~(BIT(GCIP_IMG_CFG_ADDR_SHIFT) - 1u)

/* For decoding the size of ns_iommu_mappings. */
static inline u32 gcip_ns_config_to_size(u32 cfg)
{
	if (cfg & GCIP_IMG_CFG_SIZE_MODE_BIT)
		return (cfg & GCIP_IMG_CFG_NS_SIZE_MASK) << PAGE_SHIFT;

	return (cfg & GCIP_IMG_CFG_NS_SIZE_MASK) << GCIP_IMG_CFG_MB_SHIFT;
}

/* For decoding the size of iommu_mappings. */
static inline u32 gcip_config_to_size(u32 cfg)
{
	if (cfg & GCIP_IMG_CFG_SIZE_MODE_BIT)
		return (cfg & GCIP_IMG_CFG_SECURE_SIZE_MASK) << PAGE_SHIFT;

	return BIT(cfg & GCIP_IMG_CFG_SECURE_SIZE_MASK) << PAGE_SHIFT;
}

/*
 * Initializes the image configuration parser.
 *
 * @dev is only used for logging.
 * @data will be passed to operations.
 *
 * Returns 0 on success. Returns -EINVAL when any mandatory operations is NULL.
 */
int gcip_image_config_parser_init(struct gcip_image_config_parser *parser,
				  const struct gcip_image_config_ops *ops, struct device *dev,
				  void *data);

/*
 * Parses the image configuration and adds specified IOMMU mappings by calling pre-registered
 * operations.
 *
 * Number of mappings to be added might be different according to the value of
 * @config->privilege_level:
 * - GCIP_FW_PRIV_LEVEL_NS:
 *   Both @iommu_mappings and @ns_iommu_mappings will be added. Because GCIP_FW_PRIV_LEVEL_NS means
 *   the firmware will run in non-secure mode and all transactions will go through the non-secure
 *   IOMMU.
 * - Otherwise:
 *   Only @ns_iommu_mappings are considered. TZ/GSA will be the one who programs secure IOMMU for
 *   those secure IOMMU mappings.
 *
 * Before parsing the newly passed @config, the mappings of the last record (stored by @parser
 * internally) will be reverted. If there is any mapping in the new config fails to be mapped, the
 * reverted last config will be reverted again. i.e. This function will keep the mapping state the
 * same as before calling it on any error happens. But if the IOMMU state is somehow corrupted and
 * hence fails to roll back the reverted last image config, only an error is logged. See the pseudo
 * code below:
 *
 * gcip_image_config_parse(config):
 *   unmap(last_image_config)
 *   if ret = map(config) fails:
 *     LOG("Failed to map image config, rolling back to the last image config.")
 *     if map(last_image_config) fails:
 *       LOG("Failed to roll back the last image config.")
 *     return ret
 *  else:
 *    last_image_config = config
 *  return SUCCESS
 *
 * A special case being considered is if the content of @config is identical to the last
 * successfully parsed image config, this function will return 0 immediately without removing /
 * adding any mapping.
 *
 * Returns 0 on success. Otherwise an errno, which usually would be the one returned by
 * gcip_image_config_ops.map. On error no new mapping specified in @config is added.
 */
int gcip_image_config_parse(struct gcip_image_config_parser *parser,
			    struct gcip_image_config *config);

/*
 * Clears the mappings specified in the last image config.
 *
 * It's valid to call this function without any image config has been successfully parsed, or when
 * the last image config is already cleared. In which case this function works as no-op.
 */
void gcip_image_config_clear(struct gcip_image_config_parser *parser);

/*
 * Returns whether the privilege level specified by @config is non-secure.
 */
static inline bool gcip_image_config_is_ns(struct gcip_image_config *config)
{
	return config->privilege_level == GCIP_FW_PRIV_LEVEL_NS;
}

#endif /* __GCIP_IMAGE_CONFIG_H__ */
