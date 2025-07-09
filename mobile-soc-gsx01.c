// SPDX-License-Identifier: GPL-2.0-only
/*
 * SoC specific function definitions for GSx01.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#include <linux/acpm_dvfs.h>
#include <linux/slab.h>
#include <linux/units.h>
#include <soc/google/exynos_pm_qos.h>

#include <gcip/gcip-slc.h>

#include "gxp-cmu.h"
#include "gxp-config.h"
#include "gxp-core-telemetry.h"
#include "gxp-firmware.h"
#include "gxp-gsx01-ssmt.h"
#include "gxp-lpm.h"
#include "gxp-pm.h"
#include "mobile-soc-gsx01.h"
#include "mobile-soc.h"

/*
 * Encode INT/MIF values as a 16 bit pair in the 32-bit return value
 * (in units of MHz, to provide enough range)
 */
#define PM_QOS_INT_SHIFT (16)
#define PM_QOS_MIF_MASK (0xFFFF)
#define PM_QOS_FACTOR KHZ_PER_MHZ

/*
 * Encode BCI/DSU values as a 16 bit pair in the 32-bit return value
 * (in units of MHz, to provide enough range)
 */
#define COHERENT_FABRIC_QOS_BCI_SHIFT (16)
#define COHERENT_FABRIC_QOS_DSU_MASK (0xFFFF)
#define COHERENT_FABRIC_QOS_FACTOR KHZ_PER_MHZ

static const s32 aur_memory_state2int_table[] = { 0,
						  AUR_MEM_INT_MIN,
						  AUR_MEM_INT_VERY_LOW,
						  AUR_MEM_INT_LOW,
						  AUR_MEM_INT_HIGH,
						  AUR_MEM_INT_VERY_HIGH,
						  AUR_MEM_INT_MAX };

static const s32 aur_memory_state2mif_table[] = { 0,
						  AUR_MEM_MIF_MIN,
						  AUR_MEM_MIF_VERY_LOW,
						  AUR_MEM_MIF_LOW,
						  AUR_MEM_MIF_HIGH,
						  AUR_MEM_MIF_VERY_HIGH,
						  AUR_MEM_MIF_MAX };

static u64 pm_arg_encode(enum gxp_soc_qos_request qos_request, s32 qos_val_1, s32 qos_val_2)
{
	if (qos_request == MEMORY_INT_QOS_REQ)
		return ((qos_val_1 / PM_QOS_FACTOR) << PM_QOS_INT_SHIFT) |
		       (qos_val_2 / PM_QOS_FACTOR);
	else if (qos_request == COHERENT_FABRIC_QOS_REQ)
		return ((qos_val_1 / COHERENT_FABRIC_QOS_FACTOR) << COHERENT_FABRIC_QOS_BCI_SHIFT) |
		       (qos_val_2 / COHERENT_FABRIC_QOS_FACTOR);
	else
		return 0;
}

static void pm_arg_decode(enum gxp_soc_qos_request qos_request, u64 value, s32 *qos_val_1,
			  s32 *qos_val_2)
{
	if (qos_request == MEMORY_INT_QOS_REQ) {
		*qos_val_1 = (value >> PM_QOS_INT_SHIFT) * PM_QOS_FACTOR;
		*qos_val_2 = (value & PM_QOS_MIF_MASK) * PM_QOS_FACTOR;
	} else if (qos_request == COHERENT_FABRIC_QOS_REQ) {
		*qos_val_1 = (value >> COHERENT_FABRIC_QOS_BCI_SHIFT) * COHERENT_FABRIC_QOS_FACTOR;
		*qos_val_2 = (value & COHERENT_FABRIC_QOS_DSU_MASK) * COHERENT_FABRIC_QOS_FACTOR;
	} else {
		*qos_val_1 = 0;
		*qos_val_2 = 0;
	}
}

#if GXP_ENABLE_PARTIAL_LAP
/*
 * Manually enables partial Local Access Path (LAP) support on platforms where
 * it's not enabled by default.
 *
 * Note: Since the function requires access to the block CSRs, AUR BLK needs to
 * be already on before calling this function.
 */
static void gxp_soc_enable_partial_lap(struct gxp_dev *gxp)
{
	u32 val;

	/*
	 * Enable CNOC to DNOC path in Provino for direct TOP access from Q7
	 * cores. b/258714265.
	 */
	val = gxp_read_32(gxp, FABRIC_IXBAR1_ARL_CTRL_OFFSET);
	val |= FABRIC_IXBAR1_ARL_CTRL_EN;
	gxp_write_32(gxp, FABRIC_IXBAR1_ARL_CTRL_OFFSET, val);
}
#endif /* GXP_ENABLE_PARTIAL_LAP */

void gxp_soc_set_pm_arg_from_state(struct gxp_req_pm_qos_work *work,
				   enum aur_memory_power_state state)
{
	s32 int_val = aur_memory_state2int_table[state];
	s32 mif_val = aur_memory_state2mif_table[state];

	work->pm_value = pm_arg_encode(MEMORY_INT_QOS_REQ, int_val, mif_val);
}

int gxp_soc_pm_set_rate(unsigned int id, unsigned long rate)
{
	return exynos_acpm_set_rate(id, rate);
}

unsigned long gxp_soc_pm_get_rate(struct gxp_dev *gxp, unsigned int id, unsigned long dbg_val)
{
	return exynos_acpm_get_rate(id, dbg_val);
}

void gxp_soc_pm_init(struct gxp_dev *gxp)
{
	exynos_pm_qos_add_request(&gxp->soc_data->int_min, PM_QOS_DEVICE_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&gxp->soc_data->mif_min, PM_QOS_BUS_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&gxp->soc_data->bci_min, PM_QOS_BCI_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&gxp->soc_data->dsu_min, PM_QOS_DSU_THROUGHPUT, 0);
}

void gxp_soc_pm_exit(struct gxp_dev *gxp)
{
	exynos_pm_qos_remove_request(&gxp->soc_data->dsu_min);
	exynos_pm_qos_remove_request(&gxp->soc_data->bci_min);
	exynos_pm_qos_remove_request(&gxp->soc_data->mif_min);
	exynos_pm_qos_remove_request(&gxp->soc_data->int_min);
}

void gxp_soc_pm_set_request(struct gxp_dev *gxp, enum gxp_soc_qos_request qos_request, u64 value)
{
	s32 int_val, mif_val;
	s32 bci_val, dsu_val;

	if (qos_request == MEMORY_INT_QOS_REQ) {
		pm_arg_decode(qos_request, value, &int_val, &mif_val);
		dev_dbg(gxp->dev, "pm_qos request - int = %d mif = %d\n", int_val, mif_val);
		exynos_pm_qos_update_request(&gxp->soc_data->int_min, int_val);
		exynos_pm_qos_update_request(&gxp->soc_data->mif_min, mif_val);
	} else if (qos_request == COHERENT_FABRIC_QOS_REQ) {
		pm_arg_decode(qos_request, value, &bci_val, &dsu_val);
		dev_dbg(gxp->dev, "coherent_fabric_qos request - bci = %d dsu = %d\n", bci_val,
			dsu_val);
		exynos_pm_qos_update_request(&gxp->soc_data->bci_min, bci_val);
		exynos_pm_qos_update_request(&gxp->soc_data->dsu_min, dsu_val);
	} else {
		dev_warn(gxp->dev, "Undefined QoS request: %u.", qos_request);
	}
}

u64 gxp_soc_pm_get_request(struct gxp_dev *gxp, enum gxp_soc_qos_request qos_request)
{
	s32 int_val, mif_val;
	s32 bci_val, dsu_val;

	if (qos_request == MEMORY_INT_QOS_REQ) {
		int_val = exynos_pm_qos_read_req_value(PM_QOS_DEVICE_THROUGHPUT,
						       &gxp->soc_data->int_min);
		mif_val = exynos_pm_qos_read_req_value(PM_QOS_BUS_THROUGHPUT,
						       &gxp->soc_data->mif_min);
		return pm_arg_encode(qos_request, int_val, mif_val);
	} else if (qos_request == COHERENT_FABRIC_QOS_REQ) {
		bci_val = exynos_pm_qos_read_req_value(PM_QOS_BCI_THROUGHPUT,
						       &gxp->soc_data->bci_min);
		dsu_val = exynos_pm_qos_read_req_value(PM_QOS_DSU_THROUGHPUT,
						       &gxp->soc_data->dsu_min);
		return pm_arg_encode(qos_request, bci_val, dsu_val);
	}

	dev_warn(gxp->dev, "Invalid QoS request: %u.", qos_request);
	return 0;
}

void gxp_soc_pm_reset(struct gxp_dev *gxp)
{
	exynos_pm_qos_update_request(&gxp->soc_data->int_min, 0);
	exynos_pm_qos_update_request(&gxp->soc_data->mif_min, 0);
	exynos_pm_qos_update_request(&gxp->soc_data->bci_min, 0);
	exynos_pm_qos_update_request(&gxp->soc_data->dsu_min, 0);
}

int gxp_soc_init(struct gxp_dev *gxp)
{
	int ret;

	gxp->soc_data = devm_kzalloc(gxp->dev, sizeof(struct gxp_soc_data), GFP_KERNEL);
	if (!gxp->soc_data)
		return -ENOMEM;

	ret = gxp_gsx01_ssmt_init(gxp, &gxp->soc_data->ssmt);
	if (ret) {
		dev_err(gxp->dev, "Failed to find SSMT\n");
		return ret;
	}

	ret = gxp_cmu_set_reg_resources(gxp);
	if (ret)
		dev_warn(gxp->dev, "Failed to set CMU resources");

	gcip_slc_debugfs_init(&gxp->soc_data->slc, gxp->dev, gxp->d_entry);
	gxp_cmu_debugfs_init(gxp);

	return 0;
}

void gxp_soc_exit(struct gxp_dev *gxp)
{
	gcip_slc_debugfs_exit(&gxp->soc_data->slc);
}

void gxp_soc_activate_context(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
			      uint core_list)
{
	struct gxp_ssmt *ssmt = &gxp->soc_data->ssmt;
	struct gcip_slc *slc = &gxp->soc_data->slc;
	uint core;

	/* Program VID only when cores are managed by us. */
	if (gxp_is_direct_mode(gxp)) {
		for (core = 0; core < GXP_NUM_CORES; core++)
			if (BIT(core) & core_list) {
				dev_dbg(gxp->dev, "Assign core%u to PASID %d\n", core,
					gdomain->pasid);
				gxp_gsx01_ssmt_set_core_vid(ssmt, core, gdomain->pasid);
			}
	} else {
		gxp_gsx01_ssmt_activate_scid(ssmt, gdomain->pasid);
	}

	if (gcip_slc_is_valid(slc))
		gxp_gsx01_ssmt_set_slc_attr(ssmt, slc);
}

void gxp_soc_deactivate_context(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
				uint core_list)
{
	struct gxp_ssmt *ssmt = &gxp->soc_data->ssmt;
	uint core;

	/* Program VID only when cores are managed by us. */
	if (gxp_is_direct_mode(gxp)) {
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (BIT(core) & core_list)
				gxp_gsx01_ssmt_set_core_vid(ssmt, core, 0);
		}
	} else {
		gxp_gsx01_ssmt_deactivate_scid(ssmt, gdomain->pasid);
	}
}

void gxp_soc_set_iremap_context(struct gxp_dev *gxp)
{
}

void gxp_soc_lpm_init(struct gxp_dev *gxp)
{
	/* Startup TOP's PSM */
	gxp_lpm_init(gxp);

#if GXP_ENABLE_PARTIAL_LAP
	/*
	 * Enable core local access path where applicable.
	 * Called here since it can only be called after the block is turned on.
	 */
	gxp_soc_enable_partial_lap(gxp);
#endif /* GXP_ENABLE_PARTIAL_LAP */
}

void gxp_soc_lpm_destroy(struct gxp_dev *gxp)
{
	/* Shutdown TOP's PSM */
	gxp_lpm_destroy(gxp);
}
