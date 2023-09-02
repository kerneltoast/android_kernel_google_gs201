// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2020 Google LLC. All Rights Reserved.
 *
 * Interface to the AoC USB control service
 */

#define pr_fmt(fmt) "aoc_usb_control: " fmt

#include <linux/device.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "aoc.h"
#include "aoc-interface.h"

#include "aoc_usb.h"

#define AOC_USB_NAME "aoc_usb"

enum { NONBLOCKING = 0, BLOCKING = 1 };

enum {
	TYPE_SCRATCH_PAD = 0,
	TYPE_DEVICE_CONTEXT,
	TYPE_END_OF_SETUP,
	TYPE_DCBAA
};

static ssize_t aoc_usb_send_command(struct aoc_usb_drvdata *drvdata,
				    void *in_cmd, size_t in_size, void *out_cmd,
				    size_t out_size)
{
	struct aoc_service_dev *adev = drvdata->adev;
	ssize_t ret;

	ret = mutex_lock_interruptible(&drvdata->lock);
	if (ret != 0)
		return ret;

	__pm_stay_awake(drvdata->ws);

	if (aoc_service_flush_read_data(adev))
		dev_err(&drvdata->adev->dev ,"Previous response left in channel\n");

	dev_dbg(&drvdata->adev->dev, "send cmd id [%u]\n", ((struct CMD_CORE_GENERIC *)in_cmd)->parent.id);

	ret = aoc_service_write_timeout(adev, in_cmd, in_size, drvdata->service_timeout);
	if (ret != in_size) {
		ret = -EIO;
		goto out;
	}

	ret = aoc_service_read_timeout(adev, out_cmd, out_size, drvdata->service_timeout);
	if (ret != out_size)
		ret = -EIO;

out:
	__pm_relax(drvdata->ws);
	mutex_unlock(&drvdata->lock);
	return ret;
}

static int aoc_usb_get_dev_ctx(struct aoc_usb_drvdata *drvdata,
			       unsigned int slot_id, size_t length, u8 *dev_ctx)
{
	int ret = 0;
	struct CMD_USB_CONTROL_GET_DEVICE_CONTEXT *cmd;

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_GET_DEVICE_CONTEXT), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_GET_DEVICE_CONTEXT_ID,
		     sizeof(*cmd));

	cmd->device_id = slot_id;
	cmd->length = length;

	dev_dbg(&drvdata->adev->dev, "cmd=(%u, %u)\n", cmd->device_id, cmd->length);
	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));
	if (ret < 0) {
		kfree(cmd);
		return ret;
	}

	memcpy(dev_ctx, cmd->payload, length);

	kfree(cmd);

	return 0;
}

static int aoc_usb_get_dcbaa_ptr(struct aoc_usb_drvdata *drvdata,
				 u64 *aoc_dcbaa_ptr)
{
	int ret = 0;
	struct CMD_USB_CONTROL_GET_DCBAA_PTR *cmd;

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_GET_DCBAA_PTR), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_GET_DCBAA_PTR_ID,
		     sizeof(*cmd));

	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));
	if (ret < 0) {
		kfree(cmd);
		return ret;
	}

	*aoc_dcbaa_ptr = cmd->aoc_dcbaa_ptr;

	kfree(cmd);

	return 0;
}

static int aoc_usb_set_dcbaa_ptr(struct aoc_usb_drvdata *drvdata,
				 u64 *aoc_dcbaa_ptr)
{
	int ret = 0;
	struct CMD_USB_CONTROL_SET_DCBAA_PTR *cmd;

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_SET_DCBAA_PTR), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_SET_DCBAA_PTR_ID,
		     sizeof(*cmd));

	cmd->aoc_dcbaa_ptr = *aoc_dcbaa_ptr;
	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));
	if (ret < 0) {
		kfree(cmd);
		return ret;
	}

	kfree(cmd);

	return 0;
}

static int aoc_usb_setup_done(struct aoc_usb_drvdata *drvdata)
{
	int ret;
	struct CMD_USB_CONTROL_SETUP *cmd;

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_SETUP), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_SETUP_ID,
		     sizeof(*cmd));

	cmd->type = TYPE_END_OF_SETUP;
	cmd->ctx_idx = 0;
	cmd->spbuf_idx = 0;
	cmd->length = 0;
	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));
	if (ret < 0) {
		kfree(cmd);
		return ret;
	}

	kfree(cmd);

	return 0;
}

static int aoc_usb_notify_conn_stat(struct aoc_usb_drvdata *drvdata, void *data)
{
	int ret = 0;
	struct CMD_USB_CONTROL_NOTIFY_CONN_STAT_V2 *cmd;
	struct conn_stat_args *args = data;

	// Don't update usb audio device counter if the notification is for xhci driver state.
	if (args->bus_id != 0 && args->dev_num != 0 && args->slot_id != 0) {
		if (args->conn_stat)
			drvdata->usb_conn_state++;
		else
			drvdata->usb_conn_state--;

		dev_dbg(&drvdata->adev->dev, "currently connected usb audio device count = %u\n",
			drvdata->usb_conn_state);
	}

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_NOTIFY_CONN_STAT_V2), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_NOTIFY_CONN_STAT_V2_ID,
		     sizeof(*cmd));

	cmd->bus_id = args->bus_id;
	cmd->dev_num = args->dev_num;
	cmd->slot_id = args->slot_id;
	cmd->conn_state = args->conn_stat;

	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));
	if (ret < 0) {
		kfree(cmd);
		return ret;
	}

	kfree(cmd);

	return 0;
}

static int aoc_usb_get_isoc_tr_info(struct aoc_usb_drvdata *drvdata, void *args)
{
	int ret;
	struct get_isoc_tr_info_args *tr_info_args =
		(struct get_isoc_tr_info_args *)args;
	struct CMD_USB_CONTROL_GET_ISOC_TR_INFO *cmd;

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_GET_ISOC_TR_INFO), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_GET_ISOC_TR_INFO_ID,
		     sizeof(*cmd));

	cmd->ep_id = tr_info_args->ep_id;
	cmd->dir = tr_info_args->dir;

	dev_dbg(&drvdata->adev->dev, "ep_id=%u, dir=%u\n", cmd->ep_id, cmd->dir);
	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));
	if (ret < 0) {
		kfree(cmd);
		return ret;
	}

	tr_info_args->type = cmd->type;
	tr_info_args->num_segs = cmd->num_segs;
	tr_info_args->seg_ptr = cmd->seg_ptr;
	tr_info_args->max_packet = cmd->max_packet;
	tr_info_args->cycle_state = cmd->cycle_state;
	tr_info_args->num_trbs_free = cmd->num_trbs_free;

	kfree(cmd);

	return 0;
}

static int aoc_usb_set_isoc_tr_info(struct aoc_usb_drvdata *drvdata, void *args)
{
	int ret;
	struct get_isoc_tr_info_args *tr_info_args =
		(struct get_isoc_tr_info_args *)args;
	struct CMD_USB_CONTROL_SET_ISOC_TR_INFO *cmd;

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_SET_ISOC_TR_INFO), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_SET_ISOC_TR_INFO_ID,
		     sizeof(*cmd));

	cmd->ep_id = tr_info_args->ep_id;
	cmd->dir = tr_info_args->dir;
	cmd->type = tr_info_args->type;
	cmd->num_segs = tr_info_args->num_segs;
	cmd->seg_ptr = tr_info_args->seg_ptr;
	cmd->max_packet = tr_info_args->max_packet;
	cmd->cycle_state = tr_info_args->cycle_state;
	cmd->num_trbs_free = tr_info_args->num_trbs_free;

	dev_dbg(&drvdata->adev->dev, "%s: ep_id=%u, dir=%u\n", __func__, cmd->ep_id, cmd->dir);
	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));
	if (ret < 0) {
		kfree(cmd);
		return ret;
	}

	kfree(cmd);

	return 0;
}

static int aoc_usb_set_offload_state(struct aoc_usb_drvdata *drvdata, bool *enabled)
{
	int ret = 0;
	struct CMD_USB_CONTROL_SET_OFFLOAD_STATE *cmd;

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_SET_OFFLOAD_STATE), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_SET_OFFLOAD_STATE_ID,
		     sizeof(*cmd));

	cmd->offloading = *enabled;
	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));
	if (ret < 0) {
		kfree(cmd);
		return ret;
	}

	kfree(cmd);

	return 0;
}

static int aoc_usb_send_feedback_ep_info(struct aoc_usb_drvdata *drvdata, void *args)
{
	int ret = 0;
	struct feedback_ep_info_args *fb_ep_info_args =
		(struct feedback_ep_info_args *)args;
	struct CMD_USB_CONTROL_SEND_FEEDBACK_EP_INFO *cmd;

	cmd = kzalloc(sizeof(struct CMD_USB_CONTROL_SEND_FEEDBACK_EP_INFO), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	AocCmdHdrSet(&cmd->parent,
		     CMD_USB_CONTROL_SEND_FEEDBACK_EP_INFO_ID,
		     sizeof(*cmd));

	cmd->enabled = fb_ep_info_args->enabled;
	cmd->bus_id = fb_ep_info_args->bus_id;
	cmd->dev_num = fb_ep_info_args->dev_num;
	cmd->slot_id = fb_ep_info_args->slot_id;
	cmd->ep_num = fb_ep_info_args->ep_num;
	cmd->max_packet = fb_ep_info_args->max_packet;
	cmd->binterval = fb_ep_info_args->binterval;
	cmd->brefresh = fb_ep_info_args->brefresh;
	ret = aoc_usb_send_command(drvdata, cmd, sizeof(*cmd), cmd, sizeof(*cmd));

	kfree(cmd);

	return ret;
}

static int aoc_usb_notify(struct notifier_block *this,
			  unsigned long code, void *data)
{
	struct aoc_usb_drvdata *drvdata =
		container_of(this, struct aoc_usb_drvdata, nb);
	int ret;
	struct get_dev_ctx_args *dev_ctx_args;

	switch (code) {
	case SYNC_DEVICE_CONTEXT:
		dev_ctx_args = data;
		ret = aoc_usb_get_dev_ctx(drvdata, dev_ctx_args->slot_id,
					  dev_ctx_args->length,
					  dev_ctx_args->dev_ctx);
		break;
	case GET_DCBAA_PTR:
		ret = aoc_usb_get_dcbaa_ptr(drvdata, data);
		break;
	case SET_DCBAA_PTR:
		ret = aoc_usb_set_dcbaa_ptr(drvdata, data);
		break;
	case SETUP_DONE:
		ret = aoc_usb_setup_done(drvdata);
		break;
	case GET_ISOC_TR_INFO:
		ret = aoc_usb_get_isoc_tr_info(drvdata, data);
		break;
	case SET_ISOC_TR_INFO:
		ret = aoc_usb_set_isoc_tr_info(drvdata, data);
		break;
	case SYNC_CONN_STAT:
		ret = aoc_usb_notify_conn_stat(drvdata, data);
		break;
	case SET_OFFLOAD_STATE:
		ret = aoc_usb_set_offload_state(drvdata, data);
		break;
	case SEND_FB_EP_INFO:
		ret = aoc_usb_send_feedback_ep_info(drvdata, data);
		break;
	default:
		dev_warn(&drvdata->adev->dev, "Code %lu is not supported\n", code);
		ret = -EINVAL;
		break;
	}

	if (ret < 0)
		dev_err(&drvdata->adev->dev, "Fail to handle code %lu, ret = %d", code, ret);

	return ret;
}

static enum usb_recover_state recover_state;
static struct work_struct usb_recovery_ws;
static void usb_recovery_work(struct work_struct *ws)
{
	pr_debug("%s: recover_state: %d\n", __func__, recover_state);

	switch(recover_state) {
	case RECOVER_HOST_OFF:
		dwc3_otg_host_enable(false);
		break;
	case RECOVER_HOST_ON:
		dwc3_otg_host_enable(true);
		recover_state = RECOVERED;
		break;
	default:
		pr_err("%s: unhandled recover_state: %d\n", __func__, recover_state);
		break;
	}

	return;
}

static int aoc_usb_match(struct device *dev, void *data)
{
	if (sysfs_streq(dev_driver_string(dev), "xhci-hcd-exynos"))
		return 1;

	return 0;
}

static bool aoc_usb_is_hcd_working()
{
	struct device_node *np;
	struct platform_device *pdev;
	struct device *udev;
	int ret;

	np = of_find_node_by_name(NULL, "dwc3");
	if (!np || !of_device_is_available(np)) {
		pr_err("Cannot find dwc3 device node\n");
		return false;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return false;

	udev = device_find_child(&pdev->dev, NULL, aoc_usb_match);
	if (!udev)
		return false;

	ret = usb_host_mode_state_notify(USB_CONNECTED);
	if (ret)
		dev_err(udev, "Notifying AoC for xhci driver status is failed.\n");

	return true;
}

static struct work_struct usb_host_mode_checking_ws;
static void usb_host_mode_checking_work(struct work_struct *ws)
{
	if (aoc_usb_is_hcd_working())
		pr_info("USB HCD is working, send notification to AoC\n");

	return;
}

bool aoc_usb_probe_done;
static int aoc_usb_probe(struct aoc_service_dev *adev)
{
	struct device *dev = &adev->dev;
	struct aoc_usb_drvdata *drvdata;

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->adev = adev;

	mutex_init(&drvdata->lock);

	drvdata->ws = wakeup_source_register(dev, dev_name(dev));
	if (!drvdata->ws)
		return -ENOMEM;

	drvdata->usb_conn_state = 0;
	drvdata->service_timeout = msecs_to_jiffies(100);
	drvdata->nb.notifier_call = aoc_usb_notify;
	register_aoc_usb_notifier(&drvdata->nb);

	dev_set_drvdata(dev, drvdata);

	aoc_usb_probe_done = true;

	/* Restart host if recover_state was triggered */
	if (recover_state == RECOVER_HOST_OFF) {
		dev_dbg(&drvdata->adev->dev, "restart usb device\n");
		recover_state = RECOVER_HOST_ON;
		schedule_work(&usb_recovery_ws);
	} else {
		recover_state = NONE;
	}

	schedule_work(&usb_host_mode_checking_ws);

	return 0;
}

static int aoc_usb_remove(struct aoc_service_dev *adev)
{
	struct aoc_usb_drvdata *drvdata = dev_get_drvdata(&adev->dev);

	/*
	 * Trigger recovery if usb accessory is connected.
	 * We only disable host at this moment, it will restart host
	 * after aoc usb probe again.
	 */
	if (drvdata->usb_conn_state) {
		dev_dbg(&drvdata->adev->dev, "need to recover usb device\n");
		recover_state = RECOVER_HOST_OFF;
		schedule_work(&usb_recovery_ws);
	}

	unregister_aoc_usb_notifier(&drvdata->nb);
	wakeup_source_unregister(drvdata->ws);
	mutex_destroy(&drvdata->lock);

	kfree(drvdata);

	aoc_usb_probe_done = false;

	return 0;
}

static const char *const aoc_usb_service_names[] = {
	"usb_control",
	NULL,
};

static struct aoc_driver aoc_usb_driver = {
	.drv = {
		.name = AOC_USB_NAME,
	},
	.service_names = aoc_usb_service_names,
	.probe = aoc_usb_probe,
	.remove = aoc_usb_remove,
};

static int __init aoc_usb_init(void)
{
	xhci_vendor_helper_init();
	usb_vendor_helper_init();
	snd_usb_audio_vendor_helper_init();

	INIT_WORK(&usb_recovery_ws, usb_recovery_work);
	INIT_WORK(&usb_host_mode_checking_ws, usb_host_mode_checking_work);

	return aoc_driver_register(&aoc_usb_driver);
}

static void __exit aoc_usb_exit(void)
{
	aoc_driver_unregister(&aoc_usb_driver);
}

module_init(aoc_usb_init);
module_exit(aoc_usb_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Howard Yen (Google)");
MODULE_DESCRIPTION("USB driver for AoC");
