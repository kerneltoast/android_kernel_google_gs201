// SPDX-License-Identifier: GPL-2.0
/*
 * Janeiro Edge TPU ML accelerator device host support.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/irqreturn.h>
#include <linux/uaccess.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-mobile-platform.h"
#include "edgetpu-telemetry.h"
#include "mobile-pm.h"

static irqreturn_t janeiro_mailbox_handle_irq(struct edgetpu_dev *etdev,
					      int irq)
{
	struct edgetpu_mailbox *mailbox;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mailbox_manager *mgr = etdev->mailbox_manager;
	uint i;

	if (!mgr)
		return IRQ_NONE;
	for (i = 0; i < etmdev->n_irq; i++)
		if (etmdev->irq[i] == irq)
			break;
	if (i == etmdev->n_irq)
		return IRQ_NONE;
	read_lock(&mgr->mailboxes_lock);
	mailbox = mgr->mailboxes[i];
	if (!mailbox)
		goto out;
	if (!EDGETPU_MAILBOX_RESP_QUEUE_READ(mailbox, doorbell_status))
		goto out;
	EDGETPU_MAILBOX_RESP_QUEUE_WRITE(mailbox, doorbell_clear, 1);
	etdev_dbg(mgr->etdev, "mbox %u resp doorbell irq tail=%u\n", i,
		  EDGETPU_MAILBOX_RESP_QUEUE_READ(mailbox, tail));
	if (mailbox->handle_irq)
		mailbox->handle_irq(mailbox);
out:
	read_unlock(&mgr->mailboxes_lock);
	return IRQ_HANDLED;
}

irqreturn_t edgetpu_chip_irq_handler(int irq, void *arg)
{
	struct edgetpu_dev *etdev = arg;

	edgetpu_telemetry_irq_handler(etdev);
	return janeiro_mailbox_handle_irq(etdev, irq);
}

u64 edgetpu_chip_tpu_timestamp(struct edgetpu_dev *etdev)
{
	return edgetpu_dev_read_64(etdev, EDGETPU_REG_CPUNS_TIMESTAMP);
}

void edgetpu_chip_init(struct edgetpu_dev *etdev)
{
}

void edgetpu_chip_exit(struct edgetpu_dev *etdev)
{
}

void edgetpu_mark_probe_fail(struct edgetpu_dev *etdev)
{
}

void edgetpu_chip_handle_reverse_kci(struct edgetpu_dev *etdev,
				     struct edgetpu_kci_response_element *resp)
{
	switch (resp->code) {
	case RKCI_CODE_PM_QOS_BTS:
		/* FW indicates to ignore the request by setting them to undefined values. */
		if (resp->retval != (typeof(resp->retval))~0ull)
			edgetpu_mobile_pm_set_pm_qos(etdev, resp->retval);
		if (resp->status != (typeof(resp->status))~0ull)
			edgetpu_mobile_pm_set_bts(etdev, resp->status);
		break;
	default:
		etdev_warn(etdev, "%s: Unrecognized KCI request: %u\n",
			   __func__, resp->code);
		break;
	}
}

int edgetpu_chip_get_ext_mailbox_index(u32 mbox_type, u32 *start, u32 *end)
{
	switch (mbox_type) {
	case EDGETPU_EXTERNAL_MAILBOX_TYPE_DSP:
		*start = JANEIRO_EXT_DSP_MAILBOX_START;
		*end = JANEIRO_EXT_DSP_MAILBOX_END;
		return 0;
	case EDGETPU_EXTERNAL_MAILBOX_TYPE_AOC:
		*start = JANEIRO_EXT_AOC_MAILBOX_START;
		*end = JANEIRO_EXT_AOC_MAILBOX_END;
		return 0;
	default:
		return -ENOENT;
	}
}
