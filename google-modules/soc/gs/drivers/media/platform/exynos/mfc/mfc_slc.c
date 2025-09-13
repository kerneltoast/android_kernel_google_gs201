// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 */

#include "mfc_slc.h"
#include "mfc_rm.h"

#include "mfc_core_reg_api.h"

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
void mfc_slc_enable(struct mfc_core *core)
{
	int i;

	mfc_core_debug_enter();

	if (slc_disable)
		goto done;

	/*
	 * SSMT ALLOCATE_OVERRIDE set to BYPASS
	 * Cache hint is applied on its own by 3 step below.
	 * 1) set AxCACHE(0x404) in SYSREG
	 * 2) set AXI_xxx_SLC in SFRs
	 * 3) Firmware control
	 */
	MFC_SYSREG_WRITEL(MFC_SLC_CMD_SYSREG_AX_CACHE, 0x404);
	/* Stream ID used from 0 to 15, and reserved up to max 63 */
	for (i = 0; i < 16; i++) {
		MFC_SSMT0_WRITEL(MFC_SLC_CMD_SSMT_AXI_XXX_SLC, 0x600 + 0x4 * i);
		MFC_SSMT0_WRITEL(MFC_SLC_CMD_SSMT_AXI_XXX_SLC, 0x800 + 0x4 * i);
		MFC_SSMT1_WRITEL(MFC_SLC_CMD_SSMT_AXI_XXX_SLC, 0x600 + 0x4 * i);
		MFC_SSMT1_WRITEL(MFC_SLC_CMD_SSMT_AXI_XXX_SLC, 0x800 + 0x4 * i);
	}

	/* default use 512KB for internal buffers */
	core->curr_slc_pt_idx[MFC_SLC_INTERNAL] = MFC_SLC_PARTITION_512KB;
	core->ptid[MFC_SLC_INTERNAL] = pt_client_enable(core->pt_handle,
							core->curr_slc_pt_idx[MFC_SLC_INTERNAL]);
	core->curr_slc_option = MFC_SLC_OPTION_INTERNAL;

	/*
	 * SSMT PID settings for internal buffers
	 * stream AXI ID: 4, 6, 7, 8, 9, 13
	 * READ : base + 0x000 + (0x4 * ID)
	 * WRITE: base + 0x200 + (0x4 * ID)
	 */
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x4 * 4);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x4 * 6);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x4 * 7);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x4 * 8);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x4 * 9);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x4 * 13);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x200 + 0x4 * 4);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x200 + 0x4 * 6);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x200 + 0x4 * 7);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x200 + 0x4 * 8);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x200 + 0x4 * 9);
	MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_INTERNAL], 0x200 + 0x4 * 13);

	core->slc_on_status = 1;
	mfc_core_info("[SLC] enabled ptid: %d for internal buffers \n",
			core->ptid[MFC_SLC_INTERNAL]);
	MFC_TRACE_CORE("[SLC] enabled\n");

done:
	mfc_core_debug_leave();
}

void mfc_slc_disable(struct mfc_core *core)
{
	int i;
	mfc_core_debug_enter();

	for (i = 0; i < MFC_MAX_SLC_PARTITIONS; i++) {
		if (core->ptid[i] != PT_PTID_INVALID) {
			pt_client_disable(core->pt_handle, core->curr_slc_pt_idx[i]);
			core->ptid[i] = PT_PTID_INVALID;
			core->curr_slc_pt_idx[i] = MFC_SLC_PARTITION_INVALID;
		}
	}
	core->slc_on_status = 0;

	mfc_core_info("[SLC] disabled\n");
	MFC_TRACE_CORE("[SLC] disabled\n");

	mfc_core_debug_leave();
}

void mfc_slc_flush(struct mfc_core *core, struct mfc_ctx *ctx)
{
	mfc_core_debug_enter();

	if (slc_disable)
		goto done;

	atomic_inc(&core->during_idle_resume);
	/* Trigger idle resume if core is in the idle mode */
	mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

	mfc_slc_disable(core);
	mfc_slc_enable(core);

	mfc_slc_update_partition(core, ctx);

	atomic_dec(&core->during_idle_resume);
	mfc_core_debug(2, "[SLC] flushed\n");
	MFC_TRACE_CORE("[SLC] flushed\n");

done:
	mfc_core_debug_leave();
}

void mfc_pt_resize_callback(void *data, int id, size_t resize_allocated)
{
	struct mfc_core *core = (struct mfc_core *)data;

	if (resize_allocated < 512 * 1024)
		mfc_core_info("[SLC] available SLC size(%ld) is too small\n",
				resize_allocated);
}

void mfc_client_pt_register(struct mfc_core *core)
{
	int i;
	mfc_core_debug_enter();

	core->pt_handle = pt_client_register(core->device->of_node, (void *)core,
		mfc_pt_resize_callback);
	if (!IS_ERR(core->pt_handle)) {
		core->has_slc = 1;
		core->num_slc_pt = of_property_count_strings(core->device->of_node, "pt_id");
		for (i = 0; i < MFC_MAX_SLC_PARTITIONS; i++) {
			core->ptid[i] = PT_PTID_INVALID;
			core->curr_slc_pt_idx[i] = MFC_SLC_PARTITION_INVALID;
		}
		mfc_core_debug(2, "[SLC] PT Client Register success\n");
	} else {
		core->pt_handle = NULL;
		core->has_slc = 0;
		core->num_slc_pt = 0;
		mfc_core_info("[SLC] PT Client Register fail\n");
	}

	mfc_core_debug_leave();
}

void mfc_client_pt_unregister(struct mfc_core *core)
{
	mfc_core_debug_enter();

	if (core->pt_handle) {
		core->has_slc = 0;
		pt_client_unregister(core->pt_handle);

		mfc_core_info("[SLC] PT Client Unregister.\n");
	}

	mfc_core_debug_leave();
}

void mfc_slc_update_partition(struct mfc_core *core, struct mfc_ctx *ctx)
{
	mfc_core_debug_enter();

	if (slc_disable)
		goto done;

	mfc_slc_check_options(core, ctx);
	mfc_slc_enable_more_partitions(core, ctx);

	if (core->num_slc_pt > 1) {
		/* When codec resolution >= 4k, resizing SLC partition to 1MB */
		if (OVER_UHD_RES(ctx) &&
			(core->curr_slc_pt_idx[MFC_SLC_INTERNAL] == MFC_SLC_PARTITION_512KB)) {
			core->ptid[MFC_SLC_INTERNAL] = pt_client_mutate(core->pt_handle,
				core->curr_slc_pt_idx[MFC_SLC_INTERNAL], MFC_SLC_PARTITION_1MB);

			if (core->ptid[MFC_SLC_INTERNAL] == PT_PTID_INVALID) {
				mfc_core_err("[SLC] Resizing SLC partition fail");
				mfc_slc_disable(core);
			} else {
				mfc_core_debug(2, "[SLC] Resizing SLC partition success\n");
				core->curr_slc_pt_idx[MFC_SLC_INTERNAL] = MFC_SLC_PARTITION_1MB;
			}
		}
	}

done:
	mfc_core_debug_leave();
}

void mfc_slc_check_options(struct mfc_core *core, struct mfc_ctx *ctx) {
	mfc_core_debug_enter();
	if (core->num_slc_pt >= MFC_MAX_SLC_PARTITIONS) {
		/*
		 * Default SLC option assumption:
		 * Enable full reference frame cache when decoder resolution is less
		 * than or equal to 1080p and the number of instances is 1.
		 * Enable partial reference frame cache when decoder resolution is greater
		 * than 1080p and the number of instances is 1.
		 * Otherwise use internal buffer cache only.
		 */
		if (slc_option) {
			core->curr_slc_option = slc_option;
		} else if ((ctx->type == MFCINST_DECODER) && (core->num_inst == 1)) {
			if (UNDER_FHD_RES(ctx)) {
				core->curr_slc_option = (MFC_SLC_OPTION_INTERNAL |
							MFC_SLC_OPTION_DPB_FULL_W |
							MFC_SLC_OPTION_DPB_LUMA_W |
							MFC_SLC_OPTION_DPB_CHROMA_W |
							MFC_SLC_OPTION_REF_PXL_R);
			} else {
				core->curr_slc_option = (MFC_SLC_OPTION_INTERNAL |
							MFC_SLC_OPTION_DPB_PARTIAL_W |
							MFC_SLC_OPTION_DPB_LUMA_W |
							MFC_SLC_OPTION_DPB_CHROMA_W |
							MFC_SLC_OPTION_REF_PXL_R);
			}
		} else {
			core->curr_slc_option = MFC_SLC_OPTION_INTERNAL;
		}
	}
	mfc_core_info("[SLC] Current SLC Option: %d\n", core->curr_slc_option);
	mfc_core_debug_leave();
}

void mfc_slc_disable_particular_partition(struct mfc_core *core, int partition) {
	mfc_core_debug_enter();

	if (core->ptid[partition] == PT_PTID_INVALID)
		goto done;

	pt_client_disable(core->pt_handle, core->curr_slc_pt_idx[partition]);
	core->ptid[partition] = PT_PTID_INVALID;
	core->curr_slc_pt_idx[partition] = MFC_SLC_PARTITION_INVALID;
done:
	mfc_core_debug_leave();
}

void mfc_slc_enable_more_partitions(struct mfc_core *core, struct mfc_ctx *ctx) {
	mfc_core_debug_enter();

	if (!IS_SLC_PARTITION_INTERNAL_NEED(core->curr_slc_option))
		mfc_slc_disable_particular_partition(core, MFC_SLC_INTERNAL);

	if (IS_SLC_PARTITION_DPB_W_NEED(core->curr_slc_option)) {
		if (core->ptid[MFC_SLC_DPB_W] == PT_PTID_INVALID) {
		        if (UNDER_HD_RES(ctx)) /* limited 3MB for resolution under 720p */
				core->curr_slc_pt_idx[MFC_SLC_DPB_W] = MFC_SLC_PARTITION_REF_W_3MB;
		        else if (UNDER_FHD_RES(ctx)) /* limited 6MB for resolution under 1080p */
				core->curr_slc_pt_idx[MFC_SLC_DPB_W] = MFC_SLC_PARTITION_REF_W_6MB;
		        else
				core->curr_slc_pt_idx[MFC_SLC_DPB_W] = MFC_SLC_PARTITION_REF_W_12MB;

			core->ptid[MFC_SLC_DPB_W] = pt_client_enable(core->pt_handle,
							core->curr_slc_pt_idx[MFC_SLC_DPB_W]);
			/*
			 * SSMT PID settings for reference write
			 * stream AXI ID: 10(D0) / 3(D1)
			 * WRITE: base + 0x200 + (0x4 * ID)
			 */
			MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_DPB_W], 0x200 + 0x4 * 10);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_DPB_W], 0x200 + 0x4 * 3);
			mfc_core_info("[SLC] enabled ptid: %d for DPB reference write \n",
					core->ptid[MFC_SLC_DPB_W]);
		}
	} else {
		mfc_slc_disable_particular_partition(core, MFC_SLC_DPB_W);
	}

	if (IS_SLC_PARTITION_REF_R_NEED(core->curr_slc_option)) {
		if(core->ptid[MFC_SLC_REF_R] == PT_PTID_INVALID) {
			/* use 512KB for Reference frame read */
			core->curr_slc_pt_idx[MFC_SLC_REF_R] = MFC_SLC_PARTITION_REF_R_512KB;
			core->ptid[MFC_SLC_REF_R] = pt_client_enable(core->pt_handle,
							core->curr_slc_pt_idx[MFC_SLC_REF_R]);
			/*
			 * SSMT PID settings for DPB reference read
			 * stream AXI ID: 0, 1, 2, 3 (D0) / 0, 1, 2, 3, 4, 5, 6, 7 (D1)
			 * READ: base + 0x000 + (0x4 * ID)
			 */
			MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 0);
			MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 1);
			MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 2);
			MFC_SSMT0_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 3);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 0);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 1);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 2);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 3);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 4);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 5);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 6);
			MFC_SSMT1_WRITEL(core->ptid[MFC_SLC_REF_R], 0x4 * 7);
			mfc_core_info("[SLC] enabled ptid: %d for DPB reference read \n",
				core->ptid[MFC_SLC_REF_R]);
		}
	} else {
		mfc_slc_disable_particular_partition(core, MFC_SLC_REF_R);
	}

	mfc_core_debug_leave();
}
#endif
