// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2021, The Linux Foundation. All rights reserved. */

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_reserved_mem.h>
#include <linux/exynos-pci-ctrl.h>
#include <linux/platform_data/sscoredump.h>
#include "pci_platform.h"
#include "debug.h"
#include "bus.h"

#define PLT_PATH "/chosen/plat"
#define CDB_PATH "/chosen/config"
#define HW_SKU    "sku"
#define HW_STAGE  "stage"
#define HW_MAJOR  "major"
#define HW_MINOR  "minor"
#define MAX_HW_INFO_LEN   10u
#define MAX_HW_EXT_LEN    (MAX_HW_INFO_LEN * 2)
#define MAX_FILE_COUNT    4u
#define MAX_HW_STAGE      6u
#ifndef ARRAYSIZE
#define ARRAYSIZE(a)		(u32)(sizeof(a) / sizeof(a[0]))
#endif

enum {
	REV_SKU = 0,
	REV_ONLY = 1,
	SKU_ONLY = 2,
	NO_EXT_NAME = 3
};

typedef struct {
    char hw_id[MAX_HW_INFO_LEN];
    char sku[MAX_HW_INFO_LEN];
} sku_info_t;

char hw_stage_name[MAX_HW_STAGE][MAX_HW_INFO_LEN] = {
	"DEV",
	"PROTO",
	"EVT",
	"DVT",
	"PVT",
	"MP",
};

sku_info_t sku_table[] = {
	{ {"G0DZQ"}, {"MMW"} },
	{ {"GWKK3"}, {"NA"} },
	{ {"G82U8"}, {"JPN"} },
	{ {"GHL1X"}, {"ROW"} }
};

typedef struct platform_hw_info {
	unsigned long avail_bmap;
	char ext_name[MAX_FILE_COUNT][MAX_HW_EXT_LEN];
} platform_hw_info_t;

platform_hw_info_t platform_hw_info;
char val_revision[MAX_HW_INFO_LEN] = "NULL";
char val_sku[MAX_HW_INFO_LEN] = "NULL";

extern int exynos_pcie_pm_resume(int ch_num);
extern void exynos_pcie_pm_suspend(int ch_num);
extern void exynos_pcie_set_perst(int ch_num, bool on);
extern void exynos_pcie_set_perst_gpio(int ch_num, bool on);
extern int exynos_pcie_register_event(struct exynos_pcie_register_event *reg);
extern int exynos_pcie_deregister_event(struct exynos_pcie_register_event *reg);
extern void pm_system_wakeup(void);
extern int exynos_pcie_rc_l1ss_ctrl(int enable, int id, int ch_num);

static DEFINE_SPINLOCK(pci_link_down_lock);

static struct cnss_msi_config msi_config = {
	.total_vectors = 16,
	.total_users = MSI_USERS,
	.users = (struct cnss_msi_user[]) {
		{ .name = "MHI", .num_vectors = 3, .base_vector = 0 },
		{ .name = "CE", .num_vectors = 5, .base_vector = 3 },
		{ .name = "WAKE", .num_vectors = 1, .base_vector = 8 },
		{ .name = "DP", .num_vectors = 7, .base_vector = 9 },
	},
};

int _cnss_pci_enumerate(struct cnss_plat_data *plat_priv, u32 rc_num)
{
	int ret = 0;
	ret = exynos_pcie_pm_resume(rc_num);
	return ret;
}

int cnss_pci_assert_perst(struct cnss_pci_data *pci_priv)
{
	return -EOPNOTSUPP;
}

int cnss_pci_disable_pc(struct cnss_pci_data *pci_priv, bool vote)
{
	return 0;
}

int cnss_pci_set_link_bandwidth(struct cnss_pci_data *pci_priv,
				       u16 link_speed, u16 link_width)
{
	return 0;
}

int cnss_pci_set_max_link_speed(struct cnss_pci_data *pci_priv,
				       u32 rc_num, u16 link_speed)
{
	return 0;
}

static void cnss_pci_event_cb(struct exynos_pcie_notify *notify)
{
	unsigned long flags;
	struct pci_dev *pci_dev;
	struct cnss_pci_data *pci_priv;
	struct cnss_plat_data *plat_priv;

	if (!notify)
		return;

	pci_dev = notify->user;
	if (!pci_dev)
		return;

	pci_priv = cnss_get_pci_priv(pci_dev);
	if (!pci_priv)
		return;

	plat_priv = pci_priv->plat_priv;
	switch (notify->event) {
//	case EXYNOS_PCIE_EVENT_CPL_TIMEOUT:
//               cnss_pr_err("Received PCI CPL timeout event, link possibly down\n");
               /* Fall through, handle it as link down */
	case EXYNOS_PCIE_EVENT_LINKDOWN:
		//exynos_pcie_set_perst(GOOGLE_RC_ID, false);
		exynos_pcie_set_perst_gpio(plat_priv->rc_num, false);
		if (test_bit(ENABLE_PCI_LINK_DOWN_PANIC,
			     &plat_priv->ctrl_params.quirks))
			panic("cnss: PCI link is down\n");

		spin_lock_irqsave(&pci_link_down_lock, flags);
		if (pci_priv->pci_link_down_ind) {
			cnss_pr_dbg("PCI link down recovery is in progress, ignore\n");
			spin_unlock_irqrestore(&pci_link_down_lock, flags);
			return;
		}
		pci_priv->pci_link_down_ind = true;
		spin_unlock_irqrestore(&pci_link_down_lock, flags);

		cnss_fatal_err("PCI link down, schedule recovery\n");
		cnss_schedule_recovery(&pci_dev->dev, CNSS_REASON_LINK_DOWN);
		break;
	default:
		cnss_pr_err("Received invalid PCI event: %d\n", notify->event);
	}
}

int cnss_reg_pci_event(struct cnss_pci_data *pci_priv)
{
	int ret = 0;
	struct exynos_pcie_register_event *pci_event;

	pci_event = &pci_priv->exynos_pci_event;
	pci_event->events = EXYNOS_PCIE_EVENT_LINKDOWN;
//		EXYNOS_PCIE_EVENT_CPL_TIMEOUT;
	pci_event->user = pci_priv->pci_dev;
	pci_event->mode = EXYNOS_PCIE_TRIGGER_CALLBACK;
	pci_event->callback = cnss_pci_event_cb;

	ret = exynos_pcie_register_event(pci_event);
	if (ret)
		cnss_pr_err("Failed to register exynos PCI event, err = %d\n",
			    ret);
	return ret;
}

void cnss_dereg_pci_event(struct cnss_pci_data *pci_priv)
{
	exynos_pcie_deregister_event(&pci_priv->exynos_pci_event);
}

int cnss_wlan_adsp_pc_enable(struct cnss_pci_data *pci_priv, bool control)
{
	return 0;
}

int cnss_set_pci_link(struct cnss_pci_data *pci_priv, bool link_up)
{
	cnss_pr_vdbg("%s PCI link\n", link_up ? "Resuming" : "Suspending");

	if (link_up) {
		return exynos_pcie_pm_resume(pci_priv->plat_priv->rc_num);
	} else {
		exynos_pcie_pm_suspend(pci_priv->plat_priv->rc_num);
		return 0;
	}
}

int cnss_pci_prevent_l1(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct cnss_pci_data *pci_priv = cnss_get_pci_priv(pci_dev);
	int ret;

	if (!pci_priv) {
		cnss_pr_err("pci_priv is NULL\n");
		return -ENODEV;
	}

	if (pci_priv->pci_link_state == PCI_LINK_DOWN) {
		cnss_pr_err("PCIe link is in suspend state\n");
		return -EIO;
	}

	if (pci_priv->pci_link_down_ind) {
		cnss_pr_err("PCIe link is down\n");
		return -EIO;
	}

	ret = exynos_pcie_rc_l1ss_ctrl(0, PCIE_L1SS_CTRL_WIFI, pci_priv->plat_priv->rc_num);
	if (ret) {
		cnss_pr_err("Disable PCIe L1ss failed\n");
	}

	return ret;
}
EXPORT_SYMBOL(cnss_pci_prevent_l1);

void cnss_pci_allow_l1(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct cnss_pci_data *pci_priv = cnss_get_pci_priv(pci_dev);
	int ret;

	if (!pci_priv) {
		cnss_pr_err("pci_priv is NULL\n");
		return;
	}

	if (pci_priv->pci_link_state == PCI_LINK_DOWN) {
		cnss_pr_dbg("PCIe link is in suspend state\n");
		return;
	}

	if (pci_priv->pci_link_down_ind) {
		cnss_pr_err("PCIe link is down\n");
		return;
	}

	ret = exynos_pcie_rc_l1ss_ctrl(1, PCIE_L1SS_CTRL_WIFI, pci_priv->plat_priv->rc_num);
	if (ret) {
		cnss_pr_err("Enable PCIe L1ss failed\n");
	}

}
EXPORT_SYMBOL(cnss_pci_allow_l1);

int cnss_pci_get_msi_assignment(struct cnss_pci_data *pci_priv)
{
	pci_priv->msi_config = &msi_config;

	return 0;
}

int cnss_pci_init_smmu(struct cnss_pci_data *pci_priv)
{
	return 0;
}

int _cnss_pci_get_reg_dump(struct cnss_pci_data *pci_priv,
			   u8 *buf, u32 len)
{
	return 0;
}

int cnss_pci_of_reserved_mem_device_init(struct cnss_pci_data *pci_priv)
{
	int ret = 0;
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(NULL);
	struct device *dev = &pci_priv->pci_dev->dev;
	ret = of_reserved_mem_device_init_by_idx(dev, (&plat_priv->plat_dev->dev)->of_node, 0);
	if (ret)
		cnss_pr_err("Failed to init reserved mem device, err = %d\n", ret);
	if (dev->cma_area)
		cnss_pr_dbg("CMA area\n");

	return ret;
}

/*
 * The following functions are for ssrdump.
 */

#define DEVICE_NAME "wlan"

static struct sscd_platform_data sscd_pdata;

static struct platform_device sscd_dev = {
	.name            = DEVICE_NAME,
	.driver_override = SSCD_NAME,
	.id              = -1,
	.dev             = {
		.platform_data = &sscd_pdata,
		.release       = sscd_release,
    },
};

void cnss_register_sscd(void)
{
	memset(&sscd_pdata, 0, sizeof(struct sscd_platform_data));
	memset(&sscd_dev, 0, sizeof(struct platform_device));
	sscd_dev.name = DEVICE_NAME;
	sscd_dev.driver_override = SSCD_NAME;
	sscd_dev.id = -1;
	sscd_dev.dev.platform_data = &sscd_pdata;
	sscd_dev.dev.release = sscd_release;
	platform_device_register(&sscd_dev);
}

void cnss_unregister_sscd(void)
{
	platform_device_unregister(&sscd_dev);
}

void sscd_release(struct device *dev)
{
	cnss_pr_info("%s: enter\n", __FUNCTION__);
}

u8 *crash_info = 0;
void sscd_set_coredump(void *buf, int buf_len)
{
	struct sscd_platform_data *pdata = dev_get_platdata(&sscd_dev.dev);
	struct sscd_segment seg;

	if (pdata->sscd_report) {
		memset(&seg, 0, sizeof(seg));
		seg.addr = buf;
		seg.size = buf_len;
		if(crash_info) {
			pdata->sscd_report(&sscd_dev, &seg, 1, 0, crash_info);
			kfree(crash_info);
			crash_info = 0;
		} else {
			pdata->sscd_report(&sscd_dev, &seg, 1, 0, "Unknown");
		}
	}
}

void crash_info_handler(u8 *info)
{
	u32 string_len = 0;

	if (crash_info) {
		kfree(crash_info);
		crash_info = 0;
	}

	string_len = strlen(info);
	crash_info = kzalloc(string_len + 1, GFP_KERNEL);
	if (!crash_info)
		return;
	strncpy(crash_info, info, string_len);
	crash_info[string_len] = '\0';
}

static void
cnss_set_platform_ext_name(char *hw_rev, char* val_sku)
{
	memset(&platform_hw_info, 0, sizeof(platform_hw_info_t));

	if (strncmp(hw_rev, "NULL", MAX_HW_INFO_LEN) != 0) {
		if (strncmp(val_sku, "NULL", MAX_HW_INFO_LEN) != 0) {
			snprintf(platform_hw_info.ext_name[REV_SKU], MAX_HW_EXT_LEN, "_%s_%s",
				hw_rev, val_sku);
			set_bit(REV_SKU, &platform_hw_info.avail_bmap);
		}
		snprintf(platform_hw_info.ext_name[REV_ONLY], MAX_HW_EXT_LEN, "_%s", hw_rev);
		set_bit(REV_ONLY, &platform_hw_info.avail_bmap);
	}

	if (strncmp(val_sku, "NULL", MAX_HW_INFO_LEN) != 0) {
		snprintf(platform_hw_info.ext_name[SKU_ONLY], MAX_HW_EXT_LEN, "_%s", val_sku);
		set_bit(SKU_ONLY, &platform_hw_info.avail_bmap);
	}

	memset(platform_hw_info.ext_name[NO_EXT_NAME], 0, MAX_HW_EXT_LEN);
	set_bit(NO_EXT_NAME, &platform_hw_info.avail_bmap);

	return;
}

int cnss_wlan_init_hardware_info(void)
{
	struct device_node *node = NULL;
	const char *hw_sku = NULL;
	int hw_stage = -1;
	int hw_major = -1;
	int hw_minor = -1;
	int i;

	node = of_find_node_by_path(PLT_PATH);
	if (!node) {
		cnss_pr_err("Node not created under %s\n", PLT_PATH);
		goto exit;
	} else {

		if (of_property_read_u32(node, HW_STAGE, &hw_stage)) {
			cnss_pr_err("%s: Failed to get hw stage\n", __FUNCTION__);
			goto exit;
		}

		if (of_property_read_u32(node, HW_MAJOR, &hw_major)) {
			cnss_pr_err("%s: Failed to get hw major\n", __FUNCTION__);
			goto exit;
		}

		if (of_property_read_u32(node, HW_MINOR, &hw_minor)) {
			cnss_pr_err("%s: Failed to get hw minor\n", __FUNCTION__);
			goto exit;
		}

		if (hw_stage > 0 && hw_stage <= MAX_HW_STAGE) {
			snprintf(val_revision, MAX_HW_INFO_LEN, "%s%d.%d",
					hw_stage_name[hw_stage-1], hw_major, hw_minor);

		} else {
			snprintf(val_revision, MAX_HW_INFO_LEN, "NULL");
		}
	}

	node = of_find_node_by_path(CDB_PATH);
	if (!node) {
		cnss_pr_err("Node not created under %s\n", CDB_PATH);
		goto exit;
	} else {
		if (of_property_read_string(node, HW_SKU, &hw_sku)) {
			cnss_pr_err("%s: Failed to get hw sku\n", __FUNCTION__);
			goto exit;
		}

		for (i = 0; i < ARRAYSIZE(sku_table); i ++) {
			if (strcmp(hw_sku, sku_table[i].hw_id) == 0) {
				strcpy(val_sku, sku_table[i].sku);
				break;
			}
		}
	}

	cnss_pr_info("%s: val_revision is %s, hw_sku is %s, val_sku is %s\n",
		__FUNCTION__, val_revision, hw_sku, val_sku);

exit:
	cnss_set_platform_ext_name(val_revision, val_sku);
	return 0;
}

int cnss_request_multiple_bdf_files(const struct firmware **fw,
				const char *name, struct device *device)
{
	int i, ret;
	char tmp_name[MAX_FIRMWARE_NAME_LEN];

	for (i = 0; i <= NO_EXT_NAME; i++) {
		if (!test_bit(i, &platform_hw_info.avail_bmap)) {
			continue;
		}
		memset(tmp_name, 0, MAX_FIRMWARE_NAME_LEN);
		snprintf(tmp_name, MAX_FIRMWARE_NAME_LEN, "%s%s", name,
				platform_hw_info.ext_name[i]);
		ret = firmware_request_nowarn(fw, tmp_name, device);

		if (ret) {
			cnss_pr_info("Failed to load BDF: %s, ret: %d\n", tmp_name, ret);
			continue;
		} else {
			cnss_pr_info("Load BDF successfully: %s, size: %u\n",
							tmp_name, (*fw)->size);
			break;
		}
	}
	return ret;
}
