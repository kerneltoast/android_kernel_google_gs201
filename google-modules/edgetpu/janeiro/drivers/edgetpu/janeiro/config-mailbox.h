/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Chip-dependent configuration for mailbox.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#ifndef __JANEIRO_CONFIG_MAILBOX_H__
#define __JANEIRO_CONFIG_MAILBOX_H__

#include <linux/types.h> /* u32 */

#define EDGETPU_NUM_VII_MAILBOXES 7
#define EDGETPU_NUM_P2P_MAILBOXES 0
#define EDGETPU_NUM_EXT_DSP_MAILBOXES 4
#define EDGETPU_NUM_EXT_AOC_MAILBOXES 1
#define EDGETPU_NUM_EXT_MAILBOXES (EDGETPU_NUM_EXT_DSP_MAILBOXES + EDGETPU_NUM_EXT_AOC_MAILBOXES)
#define EDGETPU_NUM_MAILBOXES (EDGETPU_NUM_VII_MAILBOXES + EDGETPU_NUM_EXT_MAILBOXES + 1)
/*
 * Mailbox index layout in mailbox manager is like:
 * ---------------------------------------------------------------
 * | KCI X 1 |   VII(s) X 7  | EXT_DSP(s) X 4  | EXT_AOC(s) X 1  |
 * ---------------------------------------------------------------
 * The TZ mailbox is not managed by the kernel, but we still need to tell firmware to enable it,
 * so its index is placed after the kernel managed mailboxes.
 */
#define EDGETPU_TZ_MAILBOX_ID	13

#define JANEIRO_EXT_DSP_MAILBOX_START (EDGETPU_NUM_VII_MAILBOXES + 1)
#define JANEIRO_EXT_DSP_MAILBOX_END \
			(EDGETPU_NUM_EXT_DSP_MAILBOXES + JANEIRO_EXT_DSP_MAILBOX_START - 1)

#define JANEIRO_EXT_AOC_MAILBOX_START (JANEIRO_EXT_DSP_MAILBOX_END + 1)
#define JANEIRO_EXT_AOC_MAILBOX_END \
			(EDGETPU_NUM_EXT_AOC_MAILBOXES + JANEIRO_EXT_AOC_MAILBOX_START - 1)


#define JANEIRO_CSR_MBOX2_CONTEXT_ENABLE 0xa0000 /* starting kernel mb*/
#define JANEIRO_CSR_AOC_MBOX_CONTEXT_ENABLE 0xb0000 /* AOC mailbox */
#define JANEIRO_CSR_DSP_MBOX_CONTEXT_ENABLE 0xc0000 /* DSP mailbox */
#define EDGETPU_MBOX_CSRS_SIZE 0x2000 /* CSR size of each mailbox */

#define JANEIRO_CSR_MBOX_CMD_QUEUE_DOORBELL_SET_OFFSET 0x1000
#define JANEIRO_CSR_MBOX_RESP_QUEUE_DOORBELL_SET_OFFSET 0x1800
#define EDGETPU_MBOX_BASE JANEIRO_CSR_MBOX2_CONTEXT_ENABLE

static inline u32 edgetpu_mailbox_get_context_csr_base(u32 index)
{
	u32 base;

	if (index <= EDGETPU_NUM_VII_MAILBOXES) {
		base = JANEIRO_CSR_MBOX2_CONTEXT_ENABLE;
		return base + index * EDGETPU_MBOX_CSRS_SIZE;
	} else if (index <= JANEIRO_EXT_DSP_MAILBOX_END) {
		base = JANEIRO_CSR_DSP_MBOX_CONTEXT_ENABLE;
		return base + (index - JANEIRO_EXT_DSP_MAILBOX_START) * EDGETPU_MBOX_CSRS_SIZE;
	}

	return JANEIRO_CSR_AOC_MBOX_CONTEXT_ENABLE;
}

static inline u32 edgetpu_mailbox_get_cmd_queue_csr_base(u32 index)
{
	return edgetpu_mailbox_get_context_csr_base(index) +
	       JANEIRO_CSR_MBOX_CMD_QUEUE_DOORBELL_SET_OFFSET;
}

static inline u32 edgetpu_mailbox_get_resp_queue_csr_base(u32 index)
{
	return edgetpu_mailbox_get_context_csr_base(index) +
	       JANEIRO_CSR_MBOX_RESP_QUEUE_DOORBELL_SET_OFFSET;
}

#endif /* __JANEIRO_CONFIG_MAILBOX_H__ */
