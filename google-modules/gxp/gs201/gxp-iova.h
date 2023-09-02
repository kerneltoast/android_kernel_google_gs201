/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP IOVAs. The list of addresses for fixed device-side IOVAs
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_IOVAS_H__
#define __GXP_IOVAS_H__

#define GXP_IOVA_SYNC_BARRIERS          (0x100000)
#define GXP_IOVA_MAILBOX(_x_)           (0x18390000 + (_x_) * 0x00020000)
#define GXP_IOVA_EXT_TPU_MBX            (0x1CEC0000)
#define GXP_IOVA_AURORA_TOP             (0x25C00000)
#define GXP_IOVA_FIRMWARE(_x_)          (0xFA000000 + (_x_) * 0x0100000)
#define GXP_IOVA_FW_DATA                (0xFA400000)
#define GXP_IOVA_TPU_MBX_BUFFER(_x_)    (0xFE100000 + (_x_) * 0x00040000)

#endif /* __GXP_IOVAS_H__ */
