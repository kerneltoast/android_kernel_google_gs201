/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#ifndef __EXYNOS_IPC_H__
#define __EXYNOS_IPC_H__

#include <linux/types.h>
#include "modem_v1.h"

#define SHM_BOOT_MAGIC		0xBDBD
#define SHM_DUMP_MAGIC		0xBDBD
#define SHM_IPC_MAGIC		0xAA

#define EXYNOS_SINGLE_MASK		(0x3<<6)
#define EXYNOS_MULTI_START_MASK		(0x1<<7)
#define EXYNOS_MULTI_LAST_MASK		(0x1<<6)

#define EXYNOS_START_MASK			0xABCD
#define EXYNOS_START_OFFSET		0
#define EXYNOS_START_SIZE			2

#define EXYNOS_FRAME_SEQ_OFFSET	2
#define EXYNOS_FRAME_SIZE			2

#define EXYNOS_FRAG_CONFIG_OFFSET	4
#define EXYNOS_FRAG_CONFIG_SIZE	2

#define EXYNOS_LEN_OFFSET			6
#define EXYNOS_LEN_SIZE			2

#define EXYNOS_CH_ID_OFFSET		8
#define EXYNOS_CH_SIZE				1

#define EXYNOS_CH_SEQ_OFFSET		9
#define EXYNOS_CH_SEQ_SIZE			1

#define EXYNOS_HEADER_SIZE		12

#define EXYNOS_FMT_NUM		1
#define EXYNOS_RFS_NUM		10

struct __packed frag_config {
	u8 frame_first:1,
	frame_last:1,
	packet_index:6;
	u8 frame_index;
};

/* EXYNOS link-layer header */
struct __packed exynos_link_header {
	u16 sync;
	u16 seq;
	u16 cfg;
	u16 len;
	u8 ch_id;
	u8 ch_seq;
};

struct __packed exynos_seq_num {
	u16 frame_cnt;
	u8 ch_cnt[255];
};

struct exynos_frame_data {
	/* Frame length calculated from the length fields */
	unsigned int len;

	/* The length of link layer header */
	unsigned int hdr_len;

	/* The length of received header */
	unsigned int hdr_rcvd;

	/* The length of link layer payload */
	unsigned int pay_len;

	/* The length of received data */
	unsigned int pay_rcvd;

	/* The length of link layer padding */
	unsigned int pad_len;

	/* The length of received padding */
	unsigned int pad_rcvd;

	/* Header buffer */
	u8 hdr[EXYNOS_HEADER_SIZE];
};

static inline bool exynos_start_valid(u8 *frm)
{
	u16 cfg = *(u16 *)(frm + EXYNOS_START_OFFSET);

	return cfg == EXYNOS_START_MASK ? true : false;
}

static inline bool exynos_multi_start_valid(u8 *frm)
{
	u16 cfg = *(u16 *)(frm + EXYNOS_FRAG_CONFIG_OFFSET);

	return ((cfg >> 8) & EXYNOS_MULTI_START_MASK) == EXYNOS_MULTI_START_MASK;
}

static inline bool exynos_multi_last_valid(u8 *frm)
{
	u16 cfg = *(u16 *)(frm + EXYNOS_FRAG_CONFIG_OFFSET);

	return ((cfg >> 8) & EXYNOS_MULTI_LAST_MASK) == EXYNOS_MULTI_LAST_MASK;
}

static inline bool exynos_single_frame(u8 *frm)
{
	u16 cfg = *(u16 *)(frm + EXYNOS_FRAG_CONFIG_OFFSET);

	return ((cfg >> 8) & EXYNOS_SINGLE_MASK) == EXYNOS_SINGLE_MASK;
}

static inline bool exynos_multi_frame(u8 *frm)
{
	u16 cfg = *(u16 *)(frm + EXYNOS_FRAG_CONFIG_OFFSET);

	return ((cfg >> 8) & EXYNOS_SINGLE_MASK) != EXYNOS_SINGLE_MASK;
}

static inline bool exynos_rcs_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_RCS_0 || ch == EXYNOS_CH_ID_RCS_1) ? true : false;
}

static inline u8 exynos_get_ch(u8 *frm)
{
	return frm[EXYNOS_CH_ID_OFFSET];
}

static inline unsigned int exynos_get_frame_seq(u8 *frm)
{
	u16 cfg = *(u16 *)(frm + EXYNOS_FRAME_SEQ_OFFSET);

	return cfg;
}

static inline unsigned int exynos_get_ch_seq(u8 *frm)
{
	return frm[EXYNOS_CH_SEQ_OFFSET];
}

static inline unsigned int exynos_calc_padding_size(unsigned int len)
{
	unsigned int residue = len & 0x7;

	return residue ? (8 - residue) : 0;
}

static inline unsigned int exynos_get_frame_len(u8 *frm)
{
	return (unsigned int)*(u16 *)(frm + EXYNOS_LEN_OFFSET);
}

static inline bool exynos_fmt_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_FMT_0 || ch == EXYNOS_CH_ID_FMT_1) ? true : false;
}

static inline bool exynos_rfs_ch(u8 ch)
{
	return (ch >= EXYNOS_CH_ID_RFS_0 && ch <= EXYNOS_CH_ID_RFS_9) ?
		true : false;
}

static inline bool exynos_boot_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_BOOT) ? true : false;
}

static inline bool exynos_dump_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_LOOPBACK) ? true : false;
}

static inline bool exynos_bootdump_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_BOOT || ch == EXYNOS_CH_ID_DUMP) ?
		true : false;
}

static inline bool exynos_ipc_ch(u8 ch)
{
	return (ch > 0 && (ch != EXYNOS_CH_ID_BOOT && ch != EXYNOS_CH_ID_DUMP)) ?
		true : false;
}

static inline bool exynos_ps_ch(u8 ch)
{
	return (ch >= EXYNOS_CH_ID_PDP_0 && ch <= EXYNOS_CH_ID_PDP_9) ?
		true : false;
}

static inline bool exynos_log_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_CPLOG) ? true : false;
}

static inline bool exynos_router_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_BT_DUN) ? true : false;
}

static inline bool exynos_embms_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_EMBMS_0 || ch == EXYNOS_CH_ID_EMBMS_1) ? true : false;
}

static inline bool exynos_uts_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_UTS) ? true : false;
}

static inline bool exynos_wfs0_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_WFS_0) ? true : false;
}

static inline bool exynos_wfs1_ch(u8 ch)
{
	return (ch == EXYNOS_CH_ID_WFS_1) ? true : false;
}

static inline bool exynos_oem_ch(u8 ch)
{
	return (ch >= EXYNOS_CH_ID_OEM_0 && ch <= EXYNOS_CH_ID_OEM_7) ?
		true : false;
}

static inline unsigned int exynos_get_total_len(u8 *frm)
{
	unsigned int len;
	unsigned int pad;

	len = exynos_get_frame_len(frm);
	pad = exynos_calc_padding_size(len) ? exynos_calc_padding_size(len) : 0;
	return len + pad;
}

static inline u16 modify_next_frame(u16 fr_cfg)
{
	u8 frame_index;

	frame_index = fr_cfg & 0x3fff;

	if (!(--frame_index)) {
		fr_cfg &= 0x3fff;
		fr_cfg |= 0x4000;
	}

	return (fr_cfg & 0xff00) | frame_index;
}

static inline bool exynos_padding_exist(u8 *frm)
{
	return exynos_calc_padding_size(exynos_get_frame_len(frm)) ? true : false;
}

static inline u32 exynos_multi_packet_index(u16 ctrl)
{
	return ((ctrl >> 8) & 0x3f);
}

static inline u32 exynos_multi_frame_index(u16 ctrl)
{
	return ctrl & 0xff;
}

static inline bool exynos_multi_last(u16 ctrl)
{
	return (ctrl >> 8) & EXYNOS_MULTI_LAST_MASK ? true : false;
}

static inline unsigned int exynos_get_hdr_len(u8 *frm)
{
	return EXYNOS_HEADER_SIZE;
}
static inline bool exynos_ext_len(u8 *frm)
{
	return 0;
}
static inline u8 exynos_get_ctrl(u8 *frm)
{
	return 0;
}
#endif
