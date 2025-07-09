// SPDX-License-Identifier: GPL-2.0-only
/*
 * Mailbox manager abstracts the mailbox interfaces of user commands.
 *
 * Copyright (C) 2022 Google LLC
 */

#include "gxp-mailbox-driver.h"
#include "gxp-mailbox-manager.h"
#include "gxp-mailbox.h"
#include "gxp.h"
#if GXP_HAS_MCU
#include "gxp-mcu-platform.h"
#endif

#include <gcip/gcip-pm.h>

#define DEBUGFS_MAILBOX "mailbox"

static int debugfs_mailbox_execute_cmd(void *data, u64 val)
{
	int core = 0, retval;
	u16 status;
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	struct gxp_mailbox *mbx;
	struct gxp_client *client;
	struct gxp_power_states power_states = {
		.power = GXP_POWER_STATE_NOM,
		.memory = MEMORY_POWER_STATE_UNDEFINED,
	};
	u16 cmd_code;
	int ret;

	ret = gcip_pm_get(gxp->power_mgr->pm);
	if (ret) {
		dev_err(gxp->dev, "Failed to power up block %d", ret);
		return ret;
	}
	mutex_lock(&gxp->debugfs_client_lock);
	client = gxp->debugfs_client;

	if (gxp_is_direct_mode(gxp)) {
		core = val / 1000;
		if (core >= GXP_NUM_CORES) {
			dev_notice(gxp->dev,
				   "Mailbox for core %d doesn't exist.\n",
				   core);
			ret = -EINVAL;
			goto out;
		}

		if (gxp->mailbox_mgr->mailboxes[core] == NULL) {
			dev_notice(
				gxp->dev,
				"Unable to send mailbox command -- mailbox %d not ready\n",
				core);
			ret = -EINVAL;
			goto out;
		}

		/* Create a dummy client to access @client->gxp from the `execute_cmd` callback. */
		if (!client)
			client = gxp_client_create(gxp);
		mbx = gxp->mailbox_mgr->mailboxes[core];
		cmd_code = GXP_MBOX_CODE_DISPATCH;
	} else {
#if GXP_HAS_MCU
		if (!client) {
			dev_err(gxp->dev,
				"You should load firmwares via gxp/firmware_run first\n");
			ret = -EIO;
			goto out;
		}

		down_read(&gxp->debugfs_client->semaphore);
		if (!gxp_client_has_available_vd(gxp->debugfs_client,
						 "GXP_MAILBOX_COMMAND")) {
			ret = -ENODEV;
			up_read(&gxp->debugfs_client->semaphore);
			goto out;
		}
		up_read(&gxp->debugfs_client->semaphore);

		mbx = to_mcu_dev(gxp)->mcu.uci.mbx;
		if (!mbx) {
			dev_err(gxp->dev, "UCI is not initialized.\n");
			ret = -EIO;
			goto out;
		}

		cmd_code = CORE_COMMAND;
#else
		dev_err(gxp->dev, "This platform only supports direct-mode.\n");
		ret = -ENODEV;
		goto out;
#endif /* GXP_HAS_MCU */
	}

	retval = gxp->mailbox_mgr->execute_cmd(client, mbx, core, cmd_code, 0,
					       0, 0, 0, 1, power_states, NULL,
					       &status);

	dev_info(
		gxp->dev,
		"Mailbox Command Sent: core=%d, resp.status=%d, resp.retval=%d\n",
		core, status, retval);
	ret = 0;
out:
	if (client && client != gxp->debugfs_client)
		gxp_client_destroy(client);
	mutex_unlock(&gxp->debugfs_client_lock);
	gcip_pm_put(gxp->power_mgr->pm);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(debugfs_mailbox_fops, NULL,
			 debugfs_mailbox_execute_cmd, "%llu\n");

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

	debugfs_create_file(DEBUGFS_MAILBOX, 0200, gxp->d_entry, gxp,
			    &debugfs_mailbox_fops);

	return mgr;
}

void gxp_mailbox_destroy_manager(struct gxp_dev *gxp,
				 struct gxp_mailbox_manager *mgr)
{
	debugfs_remove(debugfs_lookup(DEBUGFS_MAILBOX, gxp->d_entry));
	devm_kfree(gxp->dev, mgr->mailboxes);
	devm_kfree(gxp->dev, mgr);
}
