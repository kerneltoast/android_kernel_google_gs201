// SPDX-License-Identifier: GPL-2.0
/*
 * GXP debug dump handler
 *
 * Copyright (C) 2020 Google LLC
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
#include <linux/platform_data/sscoredump.h>
#endif

#include "gxp-debug-dump.h"
#include "gxp-dma.h"
#include "gxp-doorbell.h"
#include "gxp-firmware.h"
#include "gxp-host-device-structs.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-mapping.h"
#include "gxp-pm.h"
#include "gxp-vd.h"
#include "gxp-wakelock.h"

#define SSCD_MSG_LENGTH 64

#define SYNC_BARRIER_BLOCK	0x00100000
#define SYNC_BARRIER_BASE(_x_)	((_x_) << 12)

#define DEBUG_DUMP_MEMORY_SIZE 0x400000 /* size in bytes */

/* Enum indicating the debug dump request reason. */
enum gxp_debug_dump_init_type {
	DEBUG_DUMP_FW_INIT,
	DEBUG_DUMP_KERNEL_INIT
};

enum gxp_common_segments_idx {
	GXP_COMMON_REGISTERS_IDX,
	GXP_LPM_REGISTERS_IDX
};

/* Whether or not the debug dump subsystem should be enabled. */
static int gxp_debug_dump_enable;
module_param_named(debug_dump_enable, gxp_debug_dump_enable, int, 0660);

static void gxp_debug_dump_cache_invalidate(struct gxp_dev *gxp)
{
	/* Debug dump carveout is currently coherent. NO-OP. */
	return;
}

static void gxp_debug_dump_cache_flush(struct gxp_dev *gxp)
{
	/* Debug dump carveout is currently coherent. NO-OP. */
	return;
}

static u32 gxp_read_sync_barrier_shadow(struct gxp_dev *gxp, uint index)
{
	uint barrier_reg_offset;

	if (index >= SYNC_BARRIER_COUNT) {
		dev_err(gxp->dev,
			"Attempt to read non-existent sync barrier: %0u\n",
			index);
		return 0;
	}

	barrier_reg_offset = SYNC_BARRIER_BLOCK + SYNC_BARRIER_BASE(index) +
			     SYNC_BARRIER_SHADOW_OFFSET;

	return gxp_read_32(gxp, barrier_reg_offset);
}

static void
gxp_get_common_registers(struct gxp_dev *gxp, struct gxp_seg_header *seg_header,
			 struct gxp_common_registers *common_regs)
{
	int i;
	u32 addr;

	dev_dbg(gxp->dev, "Getting common registers\n");

	strscpy(seg_header->name, "Common Registers", sizeof(seg_header->name));
	seg_header->valid = 1;
	seg_header->size = sizeof(*common_regs);

	/* Get Aurora Top registers */
	common_regs->aurora_revision =
		gxp_read_32(gxp, GXP_REG_AURORA_REVISION);
	common_regs->common_int_pol_0 =
		gxp_read_32(gxp, GXP_REG_COMMON_INT_POL_0);
	common_regs->common_int_pol_1 =
		gxp_read_32(gxp, GXP_REG_COMMON_INT_POL_1);
	common_regs->dedicated_int_pol =
		gxp_read_32(gxp, GXP_REG_DEDICATED_INT_POL);
	common_regs->raw_ext_int = gxp_read_32(gxp, GXP_REG_RAW_EXT_INT);

	for (i = 0; i < CORE_PD_COUNT; i++) {
		common_regs->core_pd[i] =
			gxp_read_32(gxp, GXP_REG_CORE_PD + CORE_PD_BASE(i));
	}

	common_regs->global_counter_low =
		gxp_read_32(gxp, GXP_REG_GLOBAL_COUNTER_LOW);
	common_regs->global_counter_high =
		gxp_read_32(gxp, GXP_REG_GLOBAL_COUNTER_HIGH);
	common_regs->wdog_control = gxp_read_32(gxp, GXP_REG_WDOG_CONTROL);
	common_regs->wdog_value = gxp_read_32(gxp, GXP_REG_WDOG_VALUE);

	for (i = 0; i < TIMER_COUNT; i++) {
		addr = GXP_REG_TIMER_COMPARATOR + TIMER_BASE(i);
		common_regs->timer[i].comparator =
			gxp_read_32(gxp, addr + TIMER_COMPARATOR_OFFSET);
		common_regs->timer[i].control =
			gxp_read_32(gxp, addr + TIMER_CONTROL_OFFSET);
		common_regs->timer[i].value =
			gxp_read_32(gxp, addr + TIMER_VALUE_OFFSET);
	}

	/* Get Doorbell registers */
	for (i = 0; i < DOORBELL_COUNT; i++)
		common_regs->doorbell[i] = gxp_doorbell_status(gxp, i);

	/* Get Sync Barrier registers */
	for (i = 0; i < SYNC_BARRIER_COUNT; i++)
		common_regs->sync_barrier[i] =
			gxp_read_sync_barrier_shadow(gxp, i);

	dev_dbg(gxp->dev, "Done getting common registers\n");
}

static void gxp_get_lpm_psm_registers(struct gxp_dev *gxp,
				      struct gxp_lpm_psm_registers *psm_regs,
				      int psm)
{
	struct gxp_lpm_state_table_registers *state_table_regs;
	int i, j;
	uint offset;

	/* Get State Table registers */
	for (i = 0; i < PSM_STATE_TABLE_COUNT; i++) {
		state_table_regs = &psm_regs->state_table[i];

		/* Get Trans registers */
		for (j = 0; j < PSM_TRANS_COUNT; j++) {
			offset = PSM_STATE_TABLE_BASE(i) + PSM_TRANS_BASE(j);
			state_table_regs->trans[j].next_state =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_NEXT_STATE_OFFSET);
			state_table_regs->trans[j].seq_addr =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_SEQ_ADDR_OFFSET);
			state_table_regs->trans[j].timer_val =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_TIMER_VAL_OFFSET);
			state_table_regs->trans[j].timer_en =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_TIMER_EN_OFFSET);
			state_table_regs->trans[j].trigger_num =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_TRIGGER_NUM_OFFSET);
			state_table_regs->trans[j].trigger_en =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_TRIGGER_EN_OFFSET);
		}

		state_table_regs->enable_state =
			lpm_read_32_psm(gxp, psm, PSM_STATE_TABLE_BASE(i) +
					PSM_ENABLE_STATE_OFFSET);
	}

	/* Get DMEM registers */
	for (i = 0; i < PSM_DATA_COUNT; i++) {
		offset = PSM_DMEM_BASE(i) + PSM_DATA_OFFSET;
		psm_regs->data[i] = lpm_read_32_psm(gxp, psm, offset);
	}

	psm_regs->cfg = lpm_read_32_psm(gxp, psm, PSM_CFG_OFFSET);
	psm_regs->status = lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET);

	/* Get Debug CSR registers */
	psm_regs->debug_cfg = lpm_read_32_psm(gxp, psm, PSM_DEBUG_CFG_OFFSET);
	psm_regs->break_addr = lpm_read_32_psm(gxp, psm, PSM_BREAK_ADDR_OFFSET);
	psm_regs->gpin_lo_rd = lpm_read_32_psm(gxp, psm, PSM_GPIN_LO_RD_OFFSET);
	psm_regs->gpin_hi_rd = lpm_read_32_psm(gxp, psm, PSM_GPIN_HI_RD_OFFSET);
	psm_regs->gpout_lo_rd =
		lpm_read_32_psm(gxp, psm, PSM_GPOUT_LO_RD_OFFSET);
	psm_regs->gpout_hi_rd =
		lpm_read_32_psm(gxp, psm, PSM_GPOUT_HI_RD_OFFSET);
	psm_regs->debug_status =
		lpm_read_32_psm(gxp, psm, PSM_DEBUG_STATUS_OFFSET);
}

static void
gxp_get_lpm_registers(struct gxp_dev *gxp, struct gxp_seg_header *seg_header,
		      struct gxp_lpm_registers *lpm_regs)
{
	int i;
	uint offset;

	dev_dbg(gxp->dev, "Getting LPM registers\n");

	strscpy(seg_header->name, "LPM Registers", sizeof(seg_header->name));
	seg_header->valid = 1;
	seg_header->size = sizeof(*lpm_regs);

	/* Get LPM Descriptor registers */
	lpm_regs->lpm_version = lpm_read_32(gxp, LPM_VERSION_OFFSET);
	lpm_regs->trigger_csr_start =
		lpm_read_32(gxp, TRIGGER_CSR_START_OFFSET);
	lpm_regs->imem_start = lpm_read_32(gxp, IMEM_START_OFFSET);
	lpm_regs->lpm_config = lpm_read_32(gxp, LPM_CONFIG_OFFSET);

	for (i = 0; i < PSM_DESCRIPTOR_COUNT; i++) {
		offset = PSM_DESCRIPTOR_OFFSET + PSM_DESCRIPTOR_BASE(i);
		lpm_regs->psm_descriptor[i] = lpm_read_32(gxp, offset);
	}

	/* Get Trigger CSR registers */
	for (i = 0; i < EVENTS_EN_COUNT; i++) {
		offset = EVENTS_EN_OFFSET + EVENTS_EN_BASE(i);
		lpm_regs->events_en[i] = lpm_read_32(gxp, offset);
	}

	for (i = 0; i < EVENTS_INV_COUNT; i++) {
		offset = EVENTS_INV_OFFSET + EVENTS_INV_BASE(i);
		lpm_regs->events_inv[i] = lpm_read_32(gxp, offset);
	}

	lpm_regs->function_select = lpm_read_32(gxp, FUNCTION_SELECT_OFFSET);
	lpm_regs->trigger_status = lpm_read_32(gxp, TRIGGER_STATUS_OFFSET);
	lpm_regs->event_status = lpm_read_32(gxp, EVENT_STATUS_OFFSET);

	/* Get IMEM registers */
	for (i = 0; i < OPS_COUNT; i++) {
		offset = OPS_OFFSET + OPS_BASE(i);
		lpm_regs->ops[i] = lpm_read_32(gxp, offset);
	}

	/* Get PSM registers */
	for (i = 0; i < PSM_COUNT; i++)
		gxp_get_lpm_psm_registers(gxp, &lpm_regs->psm_regs[i], i);

	dev_dbg(gxp->dev, "Done getting LPM registers\n");
}

/*
 * Caller must make sure that gxp->debug_dump_mgr->common_dump is not NULL.
 */
static int gxp_get_common_dump(struct gxp_dev *gxp)
{
	struct gxp_common_dump *common_dump = gxp->debug_dump_mgr->common_dump;
	struct gxp_seg_header *common_seg_header = common_dump->seg_header;
	struct gxp_common_dump_data *common_dump_data =
		&common_dump->common_dump_data;
	int ret;

	/* Power on BLK_AUR to read the common registers */
	ret = gxp_wakelock_acquire(gxp);
	if (ret) {
		dev_err(gxp->dev,
			"Failed to acquire wakelock for getting common dump\n");
		return ret;
	}
	gxp_pm_update_requested_power_states(gxp, AUR_OFF, true, AUR_UUD, false,
					     AUR_MEM_UNDEFINED,
					     AUR_MEM_UNDEFINED);

	gxp_get_common_registers(gxp,
				 &common_seg_header[GXP_COMMON_REGISTERS_IDX],
				 &common_dump_data->common_regs);
	gxp_get_lpm_registers(gxp, &common_seg_header[GXP_LPM_REGISTERS_IDX],
			      &common_dump_data->lpm_regs);

	gxp_wakelock_release(gxp);
	gxp_pm_update_requested_power_states(gxp, AUR_UUD, false, AUR_OFF, true,
					     AUR_MEM_UNDEFINED,
					     AUR_MEM_UNDEFINED);

	dev_dbg(gxp->dev, "Segment Header for Common Segment\n");
	dev_dbg(gxp->dev, "Name: %s, Size: 0x%0x bytes, Valid :%0x\n",
		common_seg_header->name, common_seg_header->size,
		common_seg_header->valid);
	dev_dbg(gxp->dev, "Register aurora_revision: 0x%0x\n",
		common_dump_data->common_regs.aurora_revision);

	return ret;
}

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
static void gxp_send_to_sscd(struct gxp_dev *gxp, void *segs, int seg_cnt,
			     const char *info)
{
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;
	struct sscd_platform_data *pdata =
		(struct sscd_platform_data *)mgr->sscd_pdata;

	if (!pdata->sscd_report) {
		dev_err(gxp->dev, "Failed to generate coredump\n");
		return;
	}

	if (pdata->sscd_report(gxp->debug_dump_mgr->sscd_dev, segs, seg_cnt,
			       SSCD_FLAGS_ELFARM64HDR, info)) {
		dev_err(gxp->dev, "Unable to send the report to SSCD daemon\n");
		return;
	}
}

/*
 * `user_bufs` is an input buffer containing up to GXP_NUM_BUFFER_MAPPINGS
 * virtual addresses
 */
static int gxp_add_user_buffer_to_segments(struct gxp_dev *gxp,
					   struct gxp_core_header *core_header,
					   int core_id, int seg_idx,
					   void *user_bufs[])
{
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;
	struct gxp_user_buffer user_buf;
	int i;

	for (i = 0; i < GXP_NUM_BUFFER_MAPPINGS; i++) {
		user_buf = core_header->user_bufs[i];
		if (user_buf.size == 0)
			continue;
		if (seg_idx >= GXP_NUM_SEGMENTS_PER_CORE)
			return -EFAULT;

		mgr->segs[core_id][seg_idx].addr = user_bufs[i];
		mgr->segs[core_id][seg_idx].size = user_buf.size;
		seg_idx++;
	}

	return 0;
}

/*
 * Caller must have locked `gxp->vd_semaphore` for reading.
 */
static void gxp_user_buffers_vunmap(struct gxp_dev *gxp,
				    struct gxp_core_header *core_header)
{
	struct gxp_virtual_device *vd;
	struct gxp_user_buffer user_buf;
	int i;
	struct gxp_mapping *mapping;

	lockdep_assert_held(&gxp->vd_semaphore);

	/*
	 * TODO (b/234172464): When implementing per-core debug dump locks,
	 * down_read(&gxp->vd_semaphore) must be re-added before accessing
	 * gxp->core_to_vd[], and up_read(&gxp->vd_semaphore) must be re-added
	 * after.
	 */
	vd = gxp->core_to_vd[core_header->core_id];
	if (!vd) {
		dev_err(gxp->dev, "Virtual device is not available for vunmap\n");
		return;
	}

	for (i = 0; i < GXP_NUM_BUFFER_MAPPINGS; i++) {
		user_buf = core_header->user_bufs[i];
		if (user_buf.size == 0)
			continue;

		mapping = gxp_vd_mapping_search_in_range(
			vd, (dma_addr_t)user_buf.device_addr);
		if (!mapping) {
			dev_err(gxp->dev,
				"No mapping found for user buffer at device address %#llX\n",
				user_buf.device_addr);
			continue;
		}

		gxp_mapping_vunmap(mapping);
		gxp_mapping_put(mapping);
	}
}

/*
 * Caller must have locked `gxp->vd_semaphore` for reading.
 */
static int gxp_user_buffers_vmap(struct gxp_dev *gxp,
				 struct gxp_core_header *core_header,
				 void *user_buf_vaddrs[])
{
	struct gxp_virtual_device *vd;
	struct gxp_user_buffer *user_buf;
	int i, cnt = 0;
	dma_addr_t daddr;
	struct gxp_mapping *mapping;
	void *vaddr;

	lockdep_assert_held(&gxp->vd_semaphore);

	/*
	 * TODO (b/234172464): When implementing per-core debug dump locks,
	 * down_read(&gxp->vd_semaphore) must be re-added before accessing
	 * gxp->core_to_vd[], and up_read(&gxp->vd_semaphore) must be re-added
	 * after.
	 */
	vd = gxp->core_to_vd[core_header->core_id];
	if (!vd) {
		dev_err(gxp->dev, "Virtual device is not available for vmap\n");
		goto out;
	}

	for (i = 0; i < GXP_NUM_BUFFER_MAPPINGS; i++) {
		user_buf = &core_header->user_bufs[i];
		if (user_buf->size == 0)
			continue;

		/* Get mapping */
		daddr = (dma_addr_t)user_buf->device_addr;
		mapping = gxp_vd_mapping_search_in_range(vd, daddr);
		if (!mapping) {
			user_buf->size = 0;
			continue;
		}

		/* Map the mapping into kernel space */
		vaddr = gxp_mapping_vmap(mapping);

		/*
		 * Release the reference from searching for the mapping.
		 * Either vmapping was successful and obtained a new reference
		 * or vmapping failed, and the gxp_mapping is no longer needed.
		 */
		gxp_mapping_put(mapping);

		if (IS_ERR(vaddr)) {
			gxp_user_buffers_vunmap(gxp, core_header);
			return 0;
		}

		/* Get kernel address of the user buffer inside the mapping */
		user_buf_vaddrs[i] =
			vaddr + daddr -
			(mapping->device_address & ~(PAGE_SIZE - 1));

		/* Check that the entire user buffer is mapped */
		if ((user_buf_vaddrs[i] + user_buf->size) >
		    (vaddr + mapping->size)) {
			gxp_user_buffers_vunmap(gxp, core_header);
			return 0;
		}

		cnt++;
	}

out:
	return cnt;
}
#endif

static void gxp_invalidate_segments(struct gxp_dev *gxp, uint32_t core_id)
{
	int i;
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;
	struct gxp_core_dump *core_dump;
	struct gxp_common_dump *common_dump;
	struct gxp_core_dump_header *core_dump_header;

	core_dump = mgr->core_dump;
	common_dump = mgr->common_dump;
	if (!core_dump || !common_dump) {
		dev_dbg(gxp->dev,
			"Failed to get core_dump or common_dump for invalidating segments\n");
		return;
	}

	core_dump_header = &core_dump->core_dump_header[core_id];
	if (!core_dump_header) {
		dev_dbg(gxp->dev,
			"Failed to get core_dump_header for invalidating segments\n");
		return;
	}

	for (i = 0; i < GXP_NUM_COMMON_SEGMENTS; i++)
		common_dump->seg_header[i].valid = 0;

	for (i = 0; i < GXP_NUM_CORE_SEGMENTS; i++)
		core_dump_header->seg_header[i].valid = 0;

	for (i = 0; i < GXP_NUM_BUFFER_MAPPINGS; i++)
		core_dump_header->core_header.user_bufs[i].size = 0;

	core_dump_header->core_header.dump_available = 0;
}

/*
 * Caller must make sure that gxp->debug_dump_mgr->common_dump and
 * gxp->debug_dump_mgr->core_dump are not NULL.
 */
static int gxp_handle_debug_dump(struct gxp_dev *gxp, uint32_t core_id)
{
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;
	struct gxp_core_dump *core_dump = mgr->core_dump;
	struct gxp_core_dump_header *core_dump_header =
		&core_dump->core_dump_header[core_id];
	struct gxp_core_header *core_header = &core_dump_header->core_header;
	int ret = 0;
#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	struct gxp_common_dump *common_dump = mgr->common_dump;
	int i;
	int seg_idx = 0;
	void *data_addr;
	char sscd_msg[SSCD_MSG_LENGTH];
	void *user_buf_vaddrs[GXP_NUM_BUFFER_MAPPINGS];
	int user_buf_cnt;
#endif

	/* Core */
	if (!core_header->dump_available) {
		dev_err(gxp->dev, "Core dump should have been available\n");
		ret = -EINVAL;
		goto out;
	}

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	/* Common */
	data_addr = &common_dump->common_dump_data.common_regs;
	for (i = 0; i < GXP_NUM_COMMON_SEGMENTS; i++) {
		if (seg_idx >= GXP_NUM_SEGMENTS_PER_CORE) {
			ret = -EFAULT;
			goto out_efault;
		}
		mgr->segs[core_id][seg_idx].addr = data_addr;
		mgr->segs[core_id][seg_idx].size =
			common_dump->seg_header[i].size;
		data_addr += mgr->segs[core_id][seg_idx].size;
		seg_idx++;
	}

	/* Core Header */
	if (seg_idx >= GXP_NUM_SEGMENTS_PER_CORE) {
		ret = -EFAULT;
		goto out_efault;
	}
	mgr->segs[core_id][seg_idx].addr = core_header;
	mgr->segs[core_id][seg_idx].size = sizeof(struct gxp_core_header);
	seg_idx++;

	data_addr = &core_dump->dump_data[core_id *
					  core_header->core_dump_size /
					  sizeof(u32)];

	for (i = 0; i < GXP_NUM_CORE_SEGMENTS - 1; i++) {
		if (seg_idx >= GXP_NUM_SEGMENTS_PER_CORE) {
			ret = -EFAULT;
			goto out_efault;
		}
		mgr->segs[core_id][seg_idx].addr = data_addr;
		mgr->segs[core_id][seg_idx].size = 0;
		if (core_dump_header->seg_header[i].valid) {
			mgr->segs[core_id][seg_idx].size =
				core_dump_header->seg_header[i].size;
		}

		data_addr += core_dump_header->seg_header[i].size;
		seg_idx++;
	}

	/* DRAM */
	if (seg_idx >= GXP_NUM_SEGMENTS_PER_CORE) {
		ret = -EFAULT;
		goto out_efault;
	}
	mgr->segs[core_id][seg_idx].addr = gxp->fwbufs[core_id].vaddr;
	mgr->segs[core_id][seg_idx].size = gxp->fwbufs[core_id].size;
	seg_idx++;

	/* User Buffers */
	user_buf_cnt = gxp_user_buffers_vmap(gxp, core_header, user_buf_vaddrs);
	if (user_buf_cnt > 0) {
		if (gxp_add_user_buffer_to_segments(gxp, core_header, core_id,
						    seg_idx, user_buf_vaddrs)) {
			gxp_user_buffers_vunmap(gxp, core_header);
			ret = -EFAULT;
			goto out_efault;
		}
	}

out_efault:
	if (ret) {
		dev_err(gxp->dev,
			"seg_idx %x is larger than the size of the array\n",
			seg_idx);
	} else {
		dev_dbg(gxp->dev, "Passing dump data to SSCD daemon\n");
		snprintf(sscd_msg, SSCD_MSG_LENGTH - 1,
			 "gxp debug dump (core %0x)", core_id);
		gxp_send_to_sscd(gxp, mgr->segs[core_id],
				 seg_idx + user_buf_cnt, sscd_msg);

		gxp_user_buffers_vunmap(gxp, core_header);
	}
#endif

out:
	gxp_invalidate_segments(gxp, core_id);

	return ret;
}

static int gxp_init_segments(struct gxp_dev *gxp)
{
#if !IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	return 0;
#else
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;

	mgr->common_dump = kzalloc(sizeof(*mgr->common_dump), GFP_KERNEL);
	if (!mgr->common_dump)
		return -ENOMEM;

	return 0;
#endif
}

/*
 * Caller must have locked `gxp->debug_dump_mgr->debug_dump_lock` before calling
 * `gxp_generate_coredump`.
 */
static int gxp_generate_coredump(struct gxp_dev *gxp, uint32_t core_id)
{
	int ret = 0;

	if (!gxp->debug_dump_mgr->core_dump ||
	    !gxp->debug_dump_mgr->common_dump) {
		dev_err(gxp->dev, "Memory is not allocated for debug dump\n");
		return -EINVAL;
	}

	gxp_debug_dump_cache_invalidate(gxp);

	ret = gxp_get_common_dump(gxp);
	if (ret)
		goto out;

	ret = gxp_handle_debug_dump(gxp, core_id);
	if (ret)
		goto out;

out:
	gxp_debug_dump_cache_flush(gxp);

	return ret;
}

static void gxp_debug_dump_process_dump(struct work_struct *work)
{
	struct gxp_debug_dump_work *debug_dump_work =
		container_of(work, struct gxp_debug_dump_work, work);

	uint core_id = debug_dump_work->core_id;
	struct gxp_dev *gxp = debug_dump_work->gxp;
	u32 boot_mode;
	bool gxp_generate_coredump_called = false;

	mutex_lock(&gxp->debug_dump_mgr->debug_dump_lock);

	/*
	 * Lock the VD semaphore to ensure no suspend/resume/start/stop requests
	 * can be made on core `core_id` while generating debug dump.
	 * However, since VD semaphore is used by other VDs as well, it can
	 * potentially block device creation and destruction for other cores.
	 * TODO (b/234172464): Implement per-core debug dump locks and
	 * lock/unlock vd_semaphore before/after accessing gxp->core_to_vd[].
	 */
	down_read(&gxp->vd_semaphore);

	boot_mode = gxp_firmware_get_boot_mode(gxp, core_id);

	if (gxp_is_fw_running(gxp, core_id) &&
	    (boot_mode == GXP_BOOT_MODE_STATUS_COLD_BOOT_COMPLETED ||
	     boot_mode == GXP_BOOT_MODE_STATUS_RESUME_COMPLETED)) {
		gxp_generate_coredump_called = true;
		if (gxp_generate_coredump(gxp, core_id))
			dev_err(gxp->dev, "Failed to generate coredump\n");
	}

	/* Invalidate segments to prepare for the next debug dump trigger */
	gxp_invalidate_segments(gxp, core_id);

	up_read(&gxp->vd_semaphore);

	/*
	 * This delay is needed to ensure there's sufficient time
	 * in between sscd_report() being called, as the file name of
	 * the core dump files generated by the SSCD daemon includes a
	 * time format with a seconds precision.
	 */
	if (gxp_generate_coredump_called)
		msleep(1000);

	mutex_unlock(&gxp->debug_dump_mgr->debug_dump_lock);
}

struct work_struct *gxp_debug_dump_get_notification_handler(struct gxp_dev *gxp,
							    uint core)
{
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;

	if (!gxp_debug_dump_is_enabled())
		return NULL;

	if (!mgr->buf.vaddr) {
		dev_err(gxp->dev,
			"Debug dump must be initialized before %s is called\n",
			__func__);
		return NULL;
	}

	return &mgr->debug_dump_works[core].work;
}

int gxp_debug_dump_init(struct gxp_dev *gxp, void *sscd_dev, void *sscd_pdata)
{
	struct gxp_debug_dump_manager *mgr;
	int core;

	/* Don't initialize the debug dump subsystem unless it's enabled. */
	if (!gxp_debug_dump_enable)
		return 0;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;
	gxp->debug_dump_mgr = mgr;
	mgr->gxp = gxp;

	mgr->buf.vaddr =
		gxp_dma_alloc_coherent(gxp, NULL, 0, DEBUG_DUMP_MEMORY_SIZE,
				       &mgr->buf.daddr, GFP_KERNEL, 0);
	if (!mgr->buf.vaddr) {
		dev_err(gxp->dev, "Failed to allocate memory for debug dump\n");
		return -ENODEV;
	}
	mgr->buf.size = DEBUG_DUMP_MEMORY_SIZE;

	mgr->core_dump = (struct gxp_core_dump *)mgr->buf.vaddr;

	gxp_init_segments(gxp);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		gxp_invalidate_segments(gxp, core);
		mgr->debug_dump_works[core].gxp = gxp;
		mgr->debug_dump_works[core].core_id = core;
		INIT_WORK(&mgr->debug_dump_works[core].work,
			  gxp_debug_dump_process_dump);
	}

	/* No need for a DMA handle since the carveout is coherent */
	mgr->debug_dump_dma_handle = 0;
	mgr->sscd_dev = sscd_dev;
	mgr->sscd_pdata = sscd_pdata;
	mutex_init(&mgr->debug_dump_lock);

	return 0;
}

void gxp_debug_dump_exit(struct gxp_dev *gxp)
{
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;

	if (!mgr) {
		dev_dbg(gxp->dev, "Debug dump manager was not allocated\n");
		return;
	}

	kfree(gxp->debug_dump_mgr->common_dump);
	gxp_dma_free_coherent(gxp, NULL, 0, DEBUG_DUMP_MEMORY_SIZE,
			      mgr->buf.vaddr, mgr->buf.daddr);

	mutex_destroy(&mgr->debug_dump_lock);
	devm_kfree(mgr->gxp->dev, mgr);
	gxp->debug_dump_mgr = NULL;
}

bool gxp_debug_dump_is_enabled(void)
{
	return gxp_debug_dump_enable;
}
