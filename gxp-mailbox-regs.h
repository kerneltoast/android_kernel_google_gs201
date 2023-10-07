/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP mailbox registers.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_MAILBOX_REGS_H__
#define __GXP_MAILBOX_REGS_H__

/*
 * Macros for separating out the command queue tail and response queue head in
 * the `MBOX_DATA_CMD_TAIL_RESP_HEAD_OFFSET` register.
 */
#define CMD_TAIL_SHIFT  16
#define RESP_HEAD_SHIFT 0
#define CMD_TAIL_MASK   (0xFFFF << CMD_TAIL_SHIFT)
#define RESP_HEAD_MASK  (0xFFFF << RESP_HEAD_SHIFT)

/*
 * Macros for separating out the command queue head and response queue tail in
 * the `MBOX_DATA_CMD_HEAD_RESP_TAIL_OFFSET` register.
 */
#define CMD_HEAD_SHIFT  16
#define RESP_TAIL_SHIFT 0
#define CMD_HEAD_MASK   (0xFFFF << CMD_HEAD_SHIFT)
#define RESP_TAIL_MASK  (0xFFFF << RESP_TAIL_SHIFT)

#endif /* __GXP_MAILBOX_REGS_H__ */
