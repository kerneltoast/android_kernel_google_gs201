// SPDX-License-Identifier: GPL-2.0
/*
 * Mailbox manager abstracts the mailbox interfaces of user commands.
 *
 * Copyright (C) 2022 Google LLC
 */

#include "gxp-mailbox-driver.h"
#include "gxp-mailbox-manager.h"
#include "gxp-mailbox.h"

struct gxp_mailbox_manager *gxp_mailbox_create_manager(struct gxp_dev *gxp,
						       uint num_cores)
{
	struct gxp_mailbox_manager *mgr;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return ERR_PTR(-ENOMEM);

	mgr->gxp = gxp;
	mgr->num_cores = num_cores;
	mgr->get_mailbox_csr_base = gxp_mailbox_get_csr_base;
	mgr->get_mailbox_data_base = gxp_mailbox_get_data_base;

	mgr->mailboxes = devm_kcalloc(gxp->dev, mgr->num_cores,
				      sizeof(*mgr->mailboxes), GFP_KERNEL);
	if (!mgr->mailboxes)
		return ERR_PTR(-ENOMEM);

	return mgr;
}
