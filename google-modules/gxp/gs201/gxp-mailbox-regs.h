/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP mailbox registers.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_MAILBOX_REGS_H__
#define __GXP_MAILBOX_REGS_H__

/* Mailbox CSRs */
#define MBOX_MCUCTLR_OFFSET	0x0000

#define MBOX_INTGR0_OFFSET	0x0020
#define MBOX_INTCR0_OFFSET	0x0024
#define MBOX_INTMR0_OFFSET	0x0028
#define MBOX_INTSR0_OFFSET	0x002C
#define MBOX_INTMSR0_OFFSET	0x0030

#define MBOX_INTGR1_OFFSET	0x0040
#define MBOX_INTCR1_OFFSET	0x0044
#define MBOX_INTMR1_OFFSET	0x0048
#define MBOX_INTSR1_OFFSET	0x004C
#define MBOX_INTMSR1_OFFSET	0x0050

/* Mailbox Shared Data Registers  */
#define MBOX_DATA_REG_BASE	0x0080

#define MBOX_STATUS_OFFSET		0x00
#define MBOX_DESCRIPTOR_ADDR_OFFSET	0x04
#define MBOX_CMD_TAIL_RESP_HEAD_OFFSET	0x08
#define MBOX_CMD_HEAD_RESP_TAIL_OFFSET	0x0C

#define MBOX_REGS_SIZE 0x180

/*
 * Macros for separating out the command queue tail and response queue head in
 * the `MBOX_CMD_TAIL_RESP_HEAD_OFFSET` register.
 */
#define CMD_TAIL_SHIFT  16
#define RESP_HEAD_SHIFT 0
#define CMD_TAIL_MASK   (0xFFFF << CMD_TAIL_SHIFT)
#define RESP_HEAD_MASK  (0xFFFF << RESP_HEAD_SHIFT)

/*
 * Macros for separating out the command queue head and response queue tail in
 * the `MBOX_CMD_HEAD_RESP_TAIL_OFFSET` register.
 */
#define CMD_HEAD_SHIFT  16
#define RESP_TAIL_SHIFT 0
#define CMD_HEAD_MASK   (0xFFFF << CMD_HEAD_SHIFT)
#define RESP_TAIL_MASK  (0xFFFF << RESP_TAIL_SHIFT)

#endif /* __GXP_MAILBOX_REGS_H__ */
