// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP SSMT driver.
 *
 * Copyright (C) 2022-2024 Google LLC
 */

#include <linux/platform_device.h>

#include <gcip/gcip-slc.h>

#include "gxp-config.h"
#include "gxp-gsx01-ssmt.h"
#include "gxp-internal.h"

static inline bool ssmt_is_client_driven(struct gxp_ssmt *ssmt)
{
	return readl(ssmt->idma_ssmt_base + SSMT_CFG_OFFSET) == SSMT_MODE_CLIENT;
}

static inline void ssmt_set_vid_for_idx(void __iomem *ssmt, uint vid, uint idx)
{
	writel(vid, ssmt + SSMT_NS_READ_STREAM_VID_OFFSET + 0x4u * idx);
	writel(vid, ssmt + SSMT_NS_WRITE_STREAM_VID_OFFSET + 0x4u * idx);
}

static inline void ssmt_set_slc_for_sid(void __iomem *ssmt, struct gcip_slc *slc, uint sid)
{
	writel(slc->pid, ssmt + SSMT_NS_READ_PID_OFFSET + 0x4u * sid);
	writel(slc->pid, ssmt + SSMT_NS_WRITE_PID_OFFSET + 0x4u * sid);
	writel(slc->cache, ssmt + SSMT_NS_CACHE_OFFSET + 0x4u * sid);
	writel(slc->r_alloc_override,
	       ssmt + SSMT_NS_READ_ALLOCATE_OVERRIDE_OFFSET + 0x4u * sid);
	writel(slc->w_alloc_override,
	       ssmt + SSMT_NS_WRITE_ALLOCATE_OVERRIDE_OFFSET + 0x4u * sid);
}

void gxp_gsx01_ssmt_set_slc_attr(struct gxp_ssmt *ssmt, struct gcip_slc *slc)
{
	uint sid;

	for (sid = 0; sid < SSMT_NUM_STREAMS; ++sid) {
		ssmt_set_slc_for_sid(ssmt->idma_ssmt_base, slc, sid);
		ssmt_set_slc_for_sid(ssmt->inst_data_ssmt_base, slc, sid);
	}
}

int gxp_gsx01_ssmt_init(struct gxp_dev *gxp, struct gxp_ssmt *ssmt)
{
	struct platform_device *pdev = container_of(gxp->dev, struct platform_device, dev);
	struct resource *r;

	ssmt->gxp = gxp;
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt_idma");
	if (!r) {
		dev_err(gxp->dev, "Failed to find IDMA SSMT register base\n");
		return -EINVAL;
	}

	ssmt->idma_ssmt_base = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(ssmt->idma_ssmt_base)) {
		dev_err(gxp->dev, "Failed to map IDMA SSMT register base (%ld)\n",
			PTR_ERR(ssmt->idma_ssmt_base));
		return PTR_ERR(ssmt->idma_ssmt_base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt_inst_data");
	if (!r) {
		dev_err(gxp->dev, "Failed to find instruction/data SSMT register base\n");
		return -EINVAL;
	}

	ssmt->inst_data_ssmt_base = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(ssmt->inst_data_ssmt_base)) {
		dev_err(gxp->dev, "Failed to map instruction/data SSMT register base (%ld)\n",
			PTR_ERR(ssmt->inst_data_ssmt_base));
		return PTR_ERR(ssmt->inst_data_ssmt_base);
	}

	return 0;
}

void gxp_gsx01_ssmt_set_core_vid(struct gxp_ssmt *ssmt, uint core, uint vid)
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

/*
 * Programs SSMT to always use SCIDs as VIDs.
 * Assumes clamp mode.
 */
static void gxp_gsx01_ssmt_set_bypass(struct gxp_ssmt *ssmt)
{
	uint core;

	for (core = 0; core < GXP_NUM_CORES; core++)
		gxp_gsx01_ssmt_set_core_vid(ssmt, core, SSMT_CLAMP_MODE_BYPASS);
}

void gxp_gsx01_ssmt_activate_scid(struct gxp_ssmt *ssmt, uint scid)
{
	if (ssmt_is_client_driven(ssmt)) {
		ssmt_set_vid_for_idx(ssmt->idma_ssmt_base, scid, scid);
		ssmt_set_vid_for_idx(ssmt->inst_data_ssmt_base, scid, scid);
	} else {
		/*
		 * In clamp mode, we can't configure specific SCID. We can only mark all
		 * transactions as "bypassed" which have all streams to use their SCID as VID.
		 */
		gxp_gsx01_ssmt_set_bypass(ssmt);
	}
}

void gxp_gsx01_ssmt_deactivate_scid(struct gxp_ssmt *ssmt, uint scid)
{
	if (ssmt_is_client_driven(ssmt)) {
		ssmt_set_vid_for_idx(ssmt->idma_ssmt_base, scid, 0);
		ssmt_set_vid_for_idx(ssmt->inst_data_ssmt_base, scid, 0);
	} else {
		dev_warn_once(ssmt->gxp->dev, "Unable to deactivate context on clamp mode");
	}
}
