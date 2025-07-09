/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#ifndef __SIPC5_H__
#define __SIPC5_H__

#include <linux/types.h>
#include "../modem_v1.h"

/* SIPC5 link-layer header */
struct __packed sipc5_link_header {
	u8 cfg;
	u8 ch;
	u16 len;
	union {
		struct multi_frame_control ctrl;
		u16 ext_len;
	};
};

#define SIPC5_START_MASK	(0xF8)
#define SIPC5_CONFIG_MASK	(0x07)
#define SIPC5_EXT_FIELD_MASK	(0x03)

#define SIPC5_PADDING_EXIST	(0x04)
#define SIPC5_EXT_FIELD_EXIST	(0x02)
#define SIPC5_CTL_FIELD_EXIST	(0x01)

#define SIPC5_MULTI_FRAME_CFG	(0x03)
#define SIPC5_EXT_LENGTH_CFG	(0x02)

#define SIPC5_CONFIG_OFFSET	0
#define SIPC5_CONFIG_SIZE	1

#define SIPC5_CH_ID_OFFSET	1
#define SIPC5_CH_ID_SIZE	1

#define SIPC5_LEN_OFFSET	2
#define SIPC5_LEN_SIZE		2

#define SIPC5_CTRL_OFFSET	4
#define SIPC5_CTRL_SIZE		1

#define SIPC5_EXT_LEN_OFFSET	4
#define SIPC5_EXT_LEN_SIZE	2

#define SIPC5_MIN_HEADER_SIZE		4
#define SIPC5_HEADER_SIZE_WITH_CTL_FLD	5
#define SIPC5_HEADER_SIZE_WITH_EXT_LEN	6
#define SIPC5_MAX_HEADER_SIZE		SIPC5_HEADER_SIZE_WITH_EXT_LEN

static inline bool sipc5_start_valid(u8 *frm)
{
	return (*frm & SIPC5_START_MASK) == SIPC5_START_MASK;
}

static inline bool sipc5_padding_exist(u8 *frm)
{
	return (*frm & SIPC5_PADDING_EXIST) ? true : false;
}

static inline bool sipc5_multi_frame(u8 *frm)
{
	return (*frm & SIPC5_EXT_FIELD_MASK) == SIPC5_MULTI_FRAME_CFG;
}

static inline bool sipc5_ext_len(u8 *frm)
{
	return (*frm & SIPC5_EXT_FIELD_MASK) == SIPC5_EXT_LENGTH_CFG;
}

static inline u8 sipc5_get_ch(u8 *frm)
{
	return frm[SIPC5_CH_ID_OFFSET];
}

static inline u8 sipc5_get_ctrl(u8 *frm)
{
	return frm[SIPC5_CTRL_OFFSET];
}

static inline unsigned int sipc5_calc_padding_size(unsigned int len)
{
	unsigned int residue = len & (BITS_PER_LONG / 8 - 1);

	return residue ? (BITS_PER_LONG / 8 - residue) : 0;
}

/*
 * @brief	get the length of the header of an SIPC5 link frame
 * @param frm	the pointer to an SIPC5 link frame
 * @return	the size of the header of an SIPC5 link frame
 */
static inline unsigned int sipc5_get_hdr_len(u8 *frm)
{
	if (unlikely(frm[0] & SIPC5_EXT_FIELD_EXIST)) {
		if (sipc5_multi_frame(frm))
			return SIPC5_HEADER_SIZE_WITH_CTL_FLD;
		else
			return SIPC5_HEADER_SIZE_WITH_EXT_LEN;
	} else {
		return SIPC5_MIN_HEADER_SIZE;
	}
}

/*
 * @brief	get the real length of an SIPC5 link frame WITHOUT padding
 * @param frm	the pointer to an SIPC5 link frame
 * @return	the real length of an SIPC5 link frame WITHOUT padding
 */
static inline unsigned int sipc5_get_frame_len(u8 *frm)
{
	u16 *sz16 = (u16 *)(frm + SIPC5_LEN_OFFSET);
	u32 *sz32 = (u32 *)(frm + SIPC5_LEN_OFFSET);

	if (unlikely(frm[0] & SIPC5_EXT_FIELD_EXIST)) {
		if (sipc5_multi_frame(frm))
			return *sz16;
		else
			return *sz32;
	} else {
		return *sz16;
	}
}

/*
 * @brief	get the total length of an SIPC5 link frame with padding
 * @param frm	the pointer to an SIPC5 link frame
 * @return	the total length of an SIPC5 link frame with padding
 */
static inline unsigned int sipc5_get_total_len(u8 *frm)
{
	unsigned int len;
	unsigned int pad;

	len = sipc5_get_frame_len(frm);
	pad = sipc5_padding_exist(frm) ? sipc5_calc_padding_size(len) : 0;
	return len + pad;
}

/*
 * @param ch	the channel ID
 * @return	true if the channel ID is for FMT channel
 */
static inline bool sipc5_fmt_ch(u8 ch)
{
	return (ch >= SIPC5_CH_ID_FMT_0 && ch <= SIPC5_CH_ID_FMT_9) ?
		true : false;
}

/*
 * @param ch	the channel ID
 * @return	true if the channel ID is for RFS channel
 */
static inline bool sipc5_rfs_ch(u8 ch)
{
	return (ch >= SIPC5_CH_ID_RFS_0 && ch <= SIPC5_CH_ID_RFS_9) ?
		true : false;
}

/*
 * @param ch	the channel ID
 * @return	true if the channel ID is for BOOT channel
 */
static inline bool sipc5_boot_ch(u8 ch)
{
	return (ch >= SIPC5_CH_ID_BOOT_0 && ch <= SIPC5_CH_ID_BOOT_9) ?
		true : false;
}

/*
 * @param ch	the channel ID
 * @return	true if the channel ID is for DUMP channel
 */
static inline bool sipc5_dump_ch(u8 ch)
{
	return (ch >= SIPC5_CH_ID_DUMP_0 && ch <= SIPC5_CH_ID_DUMP_9) ?
		true : false;
}

/*
 * @param ch	the channel ID
 * @return	true if the channel ID is for BOOT/DUMP channel
 */
static inline bool sipc5_bootdump_ch(u8 ch)
{
	return (ch >= SIPC5_CH_ID_BOOT_0 && ch <= SIPC5_CH_ID_DUMP_9) ?
		true : false;
}

/*
 * @param ch	the channel ID
 * @return	true if the channel ID is for IPC channel
 */
static inline bool sipc5_ipc_ch(u8 ch)
{
	return (ch > 0 && (ch < SIPC5_CH_ID_BOOT_0 || ch > SIPC5_CH_ID_DUMP_9))
		? true : false;
}

static inline bool sipc_ps_ch(u8 ch)
{
#if IS_ENABLED(CONFIG_CH_EXTENSION)
	return (ch >= SIPC_CH_EX_ID_PDP_0 && ch <= SIPC_CH_EX_ID_PDP_MAX) ?
#else
	return (ch >= SIPC_CH_ID_PDP_0 && ch <= SIPC_CH_ID_PDP_14) ?
#endif
		true : false;
}

static inline bool sipc_csd_ch(u8 ch)
{
	return (ch >= SIPC_CH_ID_CS_VT_DATA && ch <= SIPC_CH_ID_CS_VT_VIDEO) ?
		true : false;
}

static inline bool sipc_log_ch(u8 ch)
{
	return (ch >= SIPC_CH_ID_CPLOG1 && ch <= SIPC_CH_ID_CPLOG2) ?
		true : false;
}

static inline bool sipc_router_ch(u8 ch)
{
	return (ch == SIPC_CH_ID_BT_DUN) ?
		true : false;
}

static inline bool sipc_misc_ch(u8 ch)
{
	return (ch == SIPC_CH_ID_CASS) ? true : false;
}

#endif
