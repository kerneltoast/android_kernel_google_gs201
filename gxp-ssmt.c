// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP SSMT driver.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/platform_device.h>

#include "gxp-config.h"
#include "gxp-internal.h"
#include "gxp-ssmt.h"

static inline void ssmt_set_vid_for_idx(void __iomem *ssmt, uint vid, uint idx)
{
	/* NS_READ_STREAM_VID_<sid> */
	writel(vid, ssmt + 0x1000u + 0x4u * idx);
	/* NS_WRITE_STREAM_VID_<sid> */
	writel(vid, ssmt + 0x1200u + 0x4u * idx);
}

int gxp_ssmt_init(struct gxp_dev *gxp, struct gxp_ssmt *ssmt)
{
	struct platform_device *pdev =
		container_of(gxp->dev, struct platform_device, dev);
	struct resource *r;

	ssmt->gxp = gxp;
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt_idma");
	if (!r) {
		dev_err(gxp->dev, "Failed to find IDMA SSMT register base\n");
		return -EINVAL;
	}

	ssmt->idma_ssmt_base = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(ssmt->idma_ssmt_base)) {
		dev_err(gxp->dev,
			"Failed to map IDMA SSMT register base (%ld)\n",
			PTR_ERR(ssmt->idma_ssmt_base));
		return PTR_ERR(ssmt->idma_ssmt_base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "ssmt_inst_data");
	if (!r) {
		dev_err(gxp->dev,
			"Failed to find instruction/data SSMT register base\n");
		return -EINVAL;
	}

	ssmt->inst_data_ssmt_base = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(ssmt->inst_data_ssmt_base)) {
		dev_err(gxp->dev,
			"Failed to map instruction/data SSMT register base (%ld)\n",
			PTR_ERR(ssmt->inst_data_ssmt_base));
		return PTR_ERR(ssmt->inst_data_ssmt_base);
	}

	return 0;
}

void gxp_ssmt_set_core_vid(struct gxp_ssmt *ssmt, uint core, uint vid)
{
	const u8 sids[] = {
		INST_SID_FOR_CORE(core),
		DATA_SID_FOR_CORE(core),
		IDMA_SID_FOR_CORE(core),
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(sids); i++) {
		ssmt_set_vid_for_idx(ssmt->idma_ssmt_base, vid, sids[i]);
		ssmt_set_vid_for_idx(ssmt->inst_data_ssmt_base, vid, sids[i]);
	}
}

void gxp_ssmt_set_bypass(struct gxp_ssmt *ssmt)
{
	u32 mode;
	uint core, i;

	mode = readl(ssmt->idma_ssmt_base + SSMT_CFG_OFFSET);
	if (mode == SSMT_MODE_CLIENT) {
		for (i = 0; i < MAX_NUM_CONTEXTS; i++) {
			ssmt_set_vid_for_idx(ssmt->idma_ssmt_base, i, i);
			ssmt_set_vid_for_idx(ssmt->inst_data_ssmt_base, i, i);
		}
	} else {
		for (core = 0; core < GXP_NUM_CORES; core++)
			gxp_ssmt_set_core_vid(ssmt, core,
					      SSMT_CLAMP_MODE_BYPASS);
	}
}
