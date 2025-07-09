// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Google LLC
 *
 * Samsung DisplayPort driver placeholder for gs101 and gs201.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/of_address.h>
#include <linux/irq.h>
#include <linux/hdmi.h>
#include <video/videomode.h>

#include <drm/display/drm_hdcp_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_edid.h>

#include "exynos_drm_dp.h"

struct dp_device *dp_drvdata;
EXPORT_SYMBOL_GPL(dp_drvdata);

struct blocking_notifier_head dp_ado_notifier_head =
		BLOCKING_NOTIFIER_INIT(dp_ado_notifier_head);
EXPORT_SYMBOL_GPL(dp_ado_notifier_head);

#define DP_SUPPORT_TPS(_v) BIT((_v)-1)

/* Audio(ALSA) Handshaking Functions */
int dp_audio_config(struct dp_audio_config *audio_config)
{
	pr_err("%s: not supported by this device\n", __func__);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(dp_audio_config);

/* HDCP Driver Handshaking Functions */
void dp_hdcp_update_cp(u32 drm_cp_status)
{
	pr_err("%s: not supported by this device\n", __func__);
}
EXPORT_SYMBOL_GPL(dp_hdcp_update_cp);

int dp_dpcd_read_for_hdcp22(u32 address, u32 length, u8 *data)
{
	pr_err("%s: not supported by this device\n", __func__);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(dp_dpcd_read_for_hdcp22);

int dp_dpcd_write_for_hdcp22(u32 address, u32 length, u8 *data)
{
	pr_err("%s: not supported by this device\n", __func__);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(dp_dpcd_write_for_hdcp22);

static const struct of_device_id dp_of_match[] = {
	{ .compatible = "samsung,exynos-dp" },
	{},
};

static int dp_probe(struct platform_device *pdev)
{
	pr_err("%s: not supported by this device\n", __func__);
	return -EFAULT;
}

static int dp_remove(struct platform_device *pdev)
{
	pr_err("%s: not supported by this device\n", __func__);
	return -EFAULT;
}


MODULE_DEVICE_TABLE(of, dp_of_match);

struct platform_driver dp_driver
	__refdata = { .probe = dp_probe,
		      .remove = dp_remove,
		      .driver = {
			      .name = "exynos-drmdp",
			      .owner = THIS_MODULE,
			      .of_match_table = of_match_ptr(dp_of_match),
		      } };

MODULE_AUTHOR("Qian-Hao Huang <qhhuang@google.com>");
MODULE_DESCRIPTION("Placeholder driver for Samusung DisplayPort");
MODULE_LICENSE("GPL");
