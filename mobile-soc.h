/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SoC-specific function headers.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#ifndef __MOBILE_SOC_H__
#define __MOBILE_SOC_H__

#include "gxp-internal.h"
#include "gxp-pm.h"

enum gxp_soc_qos_request {
	MEMORY_INT_QOS_REQ,
	COHERENT_FABRIC_QOS_REQ,
};

/**
 * gxp_soc_init() - Initialization function for SoC-dependent code.
 * @gxp: The GXP device to be initialized.
 *
 * This function is used to initialize SoC-dependent functions and data structure.
 * gxp->soc_data should be allocated here.
 */
int gxp_soc_init(struct gxp_dev *gxp);

/**
 * gxp_soc_exit() - Cleans up resources allocated in gxp_soc_init().
 * @gxp: The GXP device to be initialized.
 */
void gxp_soc_exit(struct gxp_dev *gxp);

/**
 * gxp_soc_pm_init() - Initializes the pm requests.
 * @gxp: The GXP device which is the container of target pm.
 *
 * The function should be called only once after allocation.
 */
void gxp_soc_pm_init(struct gxp_dev *gxp);

/**
 * gxp_soc_pm_exit() - Finalizess the pm requests.
 * @gxp: The GXP device which is the container of target pm.
 *
 * The function should be called only once before destroy.
 */
void gxp_soc_pm_exit(struct gxp_dev *gxp);

/**
 * gxp_soc_pm_set_request() - Updates the pm requests.
 * @gxp: The GXP device which is the container of target pm.
 * @qos_request: SOC QoS request enum.
 * @value: A 64-bit encoded value.
 */
void gxp_soc_pm_set_request(struct gxp_dev *gxp, enum gxp_soc_qos_request qos_request, u64 value);

/**
 * gxp_soc_pm_get_request() - Retrieves the pm requests.
 * @gxp: The GXP device which is the container of target pm.
 * @qos_request: SOC QoS request enum.
 */
u64 gxp_soc_pm_get_request(struct gxp_dev *gxp, enum gxp_soc_qos_request qos_request);

/**
 * gxp_soc_pm_reset() - Resets the pm requests.
 * @gxp: The GXP device which is the container of target pm.
 *
 * This functions resets the pm status as just initialized.
 */
void gxp_soc_pm_reset(struct gxp_dev *gxp);

/* The set function of pm rate. */
int gxp_soc_pm_set_rate(unsigned int id, unsigned long rate);

/* The get function of pm rate. */
unsigned long gxp_soc_pm_get_rate(struct gxp_dev *gxp, unsigned int id, unsigned long dbg_val);

/**
 * gxp_soc_set_pm_arg_from_state() - Set gxp_req_pm_qos_work according to the given state.
 * @work: The work object to be set.
 * @state: The target state to be set to.
 */
void gxp_soc_set_pm_arg_from_state(struct gxp_req_pm_qos_work *work,
				   enum aur_memory_power_state state);

/**
 * gxp_soc_activate_context() - Assigns cores to PASIDs.
 * @gxp: The GXP device to check the run mode.
 * @gdomain: The IOMMU domain that contains PASID.
 * @core_list: The physical cores to be assigned.
 */
void gxp_soc_activate_context(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
			      uint core_list);
/**
 * gxp_soc_deactivate_context() - Unassigns cores to PASIDs.
 * @gxp: The GXP device to check the run mode.
 * @gdomain: The IOMMU domain that contains PASID.
 * @core_list: The physical cores to be assigned.
 */
void gxp_soc_deactivate_context(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
				uint core_list);

/**
 * gxp_soc_set_iremap_context() - Set context for MCU accesses through remap region.
 * @gxp: The GXP device to set context for.
 */
void gxp_soc_set_iremap_context(struct gxp_dev *gxp);

/**
 * gxp_soc_lpm_init() - Initialize and prepare TOP PSM
 * @gxp: The GXP device to set TOP PSM for.
 */
void gxp_soc_lpm_init(struct gxp_dev *gxp);

/**
 * gxp_soc_lpm_destroy() - Deinit TOP PSM
 * @gxp: The GXP device to set TOP PSM for.
 */
void gxp_soc_lpm_destroy(struct gxp_dev *gxp);

#endif /* __MOBILE_SOC_H__ */
