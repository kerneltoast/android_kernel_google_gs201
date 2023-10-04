/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP mailbox driver.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_MAILBOX_DRIVER_H__
#define __GXP_MAILBOX_DRIVER_H__

#include "gxp-mailbox.h"

void gxp_mailbox_driver_init(struct gxp_mailbox *mailbox);
void gxp_mailbox_driver_exit(struct gxp_mailbox *mailbox);

void gxp_mailbox_driver_enable_interrupts(struct gxp_mailbox *mailbox);
void gxp_mailbox_driver_disable_interrupts(struct gxp_mailbox *mailbox);

void __iomem *gxp_mailbox_get_csr_base(struct gxp_dev *gxp, uint index);
void __iomem *gxp_mailbox_get_data_base(struct gxp_dev *gxp, uint index);

void gxp_mailbox_reset_hw(struct gxp_mailbox *mailbox);

void gxp_mailbox_generate_device_interrupt(struct gxp_mailbox *mailbox,
					   u32 int_mask);
u32 gxp_mailbox_get_device_mask_status(struct gxp_mailbox *mailbox);

void gxp_mailbox_clear_host_interrupt(struct gxp_mailbox *mailbox,
				      u32 int_mask);
void gxp_mailbox_mask_host_interrupt(struct gxp_mailbox *mailbox, u32 int_mask);
u32 gxp_mailbox_get_host_mask_status(struct gxp_mailbox *mailbox);

void gxp_mailbox_write_status(struct gxp_mailbox *mailbox, u32 status);
void gxp_mailbox_write_descriptor(struct gxp_mailbox *mailbox,
				  dma_addr_t descriptor_addr);

void gxp_mailbox_write_cmd_queue_tail(struct gxp_mailbox *mailbox, u16 val);
void gxp_mailbox_write_resp_queue_head(struct gxp_mailbox *mailbox, u16 val);
u16 gxp_mailbox_read_cmd_queue_head(struct gxp_mailbox *mailbox);
u16 gxp_mailbox_read_resp_queue_tail(struct gxp_mailbox *mailbox);

/*
 * These functions are only used to initialize these values on mailbox startup.
 * During normal use, the host must not write the command queue head or response
 * queue tail, as the device sets those values.
 */
void gxp_mailbox_write_cmd_queue_head(struct gxp_mailbox *mailbox, u16 val);
void gxp_mailbox_write_resp_queue_tail(struct gxp_mailbox *mailbox, u16 val);
u16 gxp_mailbox_read_cmd_queue_tail(struct gxp_mailbox *mailbox);
u16 gxp_mailbox_read_resp_queue_head(struct gxp_mailbox *mailbox);

#endif /* __GXP_MAILBOX_DRIVER_H__ */
