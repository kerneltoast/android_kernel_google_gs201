/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 */

#ifndef __MFC_SLC_H
#define __MFC_SLC_H __FILE__

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
#include <soc/google/pt.h>
#endif

#include "mfc_common.h"

#define MFC_SLC_ALLOC_FULL(reg, id)		\
	mfc_set_bits(reg, MFC_REG_AXI_ATTR_MASK, (id) * 2, MFC_REG_AXI_ATTR_FULL_ALLOC)
#define MFC_SLC_ALLOC_PARTIAL(reg, id)		\
	mfc_set_bits(reg, MFC_REG_AXI_ATTR_MASK, (id) * 2, MFC_REG_AXI_ATTR_CONDITIONAL_ALLOC)

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)

#define MFC_SLC_CMD_SYSREG_AX_CACHE		0xeeee
#define MFC_SLC_CMD_SSMT_AXI_XXX_SLC	0x80000000

#define IS_SLC_PARTITION_INTERNAL_NEED(option)		((option) & MFC_SLC_OPTION_INTERNAL)
#define IS_SLC_PARTITION_DPB_W_NEED(option)		((option) & MFC_SLC_OPTION_DPB_FULL_W) ||	\
							((option) & MFC_SLC_OPTION_DPB_PARTIAL_W)
#define IS_SLC_PARTITION_REF_R_NEED(option)		((option) & MFC_SLC_OPTION_REF_PXL_R)

/**
 * enum mfc_slc_partition_index - The index of MFC SLC partition
 * @MFC_SLC_PARTITION_INVALID: no valid slc partition for mfc
 * @MFC_SLC_PARTITION_512KB: 512KB of slc partition for mfc internal buffers
 * @MFC_SLC_PARTITION_1MB: 1MB of slc partition for mfc internal buffers
 * @MFC_SLC_PARTITION_3MB: 3MB of slc partition for mfc reference buffers read
 * @MFC_SLC_PARTITION_12MB: 12MB of slc partition for mfc reference buffers write
 * The index and size of slc partition need to match from the device tree.
 */
enum mfc_slc_partition_index {
	MFC_SLC_PARTITION_INVALID	= PT_PTID_INVALID,
	MFC_SLC_PARTITION_512KB		= 0,
	MFC_SLC_PARTITION_1MB		= 1,
	MFC_SLC_PARTITION_REF_R_512KB	= 2,
	MFC_SLC_PARTITION_REF_W_3MB	= 3,
	MFC_SLC_PARTITION_REF_W_6MB	= 4,
	MFC_SLC_PARTITION_REF_W_12MB	= 5,
};

/**
 * enum mfc_slc_partition_type - The type of MFC SLC partition
 */
enum mfc_slc_partition_type {
	MFC_SLC_INTERNAL	= 0,
	MFC_SLC_DPB_W		= 1,
	MFC_SLC_REF_R		= 2,
};

/**
 * enum mfc_slc_option - The option of slc setting
 * 1: Fully RW allocated to SLC for internal buffers.
 * 2: Fully W allocated to SLC for DPB reference frames.
 * 4: Partial W allocated to SLC for DPB reference frames.
 * 8: Only W allocated for Luma channel to SLC for DPB reference frames.
 * 16: Only W allocated for Chroma channel to SLC for DPB reference frames.
 * 32: Fully R allocated to SLC for DPB reference frames.
 */
enum mfc_slc_option {
	MFC_SLC_OPTION_INTERNAL			= BIT(0),
	MFC_SLC_OPTION_DPB_FULL_W		= BIT(1),
	MFC_SLC_OPTION_DPB_PARTIAL_W		= BIT(2),
	MFC_SLC_OPTION_DPB_LUMA_W		= BIT(3),
	MFC_SLC_OPTION_DPB_CHROMA_W		= BIT(4),
	MFC_SLC_OPTION_REF_PXL_R		= BIT(5),
};

void mfc_slc_enable(struct mfc_core *core);
void mfc_slc_disable(struct mfc_core *core);
void mfc_slc_flush(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_pt_resize_callback(void *data, int id, size_t resize_allocated);
void mfc_slc_update_partition(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_slc_check_options(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_slc_disable_particular_partition(struct mfc_core *core, int partition);
void mfc_slc_enable_more_partitions(struct mfc_core *core, struct mfc_ctx *ctx);

void mfc_client_pt_register(struct mfc_core *core);
void mfc_client_pt_unregister(struct mfc_core *core);
#else
#define mfc_slc_enable(core)	do {} while (0)
#define mfc_slc_disable(core)	do {} while (0)
#define mfc_slc_flush(core)	do {} while (0)

#define mfc_client_pt_register(core) do {} while (0)
#define mfc_client_pt_unregister(core) do {} while (0)
#endif

#endif /* __MFC_SLC_H */
