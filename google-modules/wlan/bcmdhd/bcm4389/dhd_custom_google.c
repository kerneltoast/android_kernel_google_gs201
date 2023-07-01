/*
 * Platform Dependent file for Hikey
 *
 * Copyright (C) 2022, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <bcmutils.h>
#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/wlan_plat.h>
#else
#include <dhd_plat.h>
#endif /* CONFIG_WIFI_CONTROL_FUNC */
#include <dhd_dbg.h>
#include <dhd.h>

#if defined(CONFIG_SOC_GOOGLE)
#include <linux/exynos-pci-ctrl.h>
#endif /* CONFIG_SOC_GOOGLE */

#ifdef DHD_COREDUMP
#include <linux/platform_data/sscoredump.h>
#endif /* DHD_COREDUMP */

#define EXYNOS_PCIE_VENDOR_ID 0x144d
#if defined(CONFIG_SOC_GOOGLE)
#define EXYNOS_PCIE_DEVICE_ID 0xecec
#define EXYNOS_PCIE_CH_NUM 0
#else
#error "Not supported platform"
#endif /* CONFIG_SOC_GOOGLE */

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
extern int dhd_init_wlan_mem(void);
extern void *dhd_wlan_mem_prealloc(int section, unsigned long size);
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#define WLAN_REG_ON_GPIO		491
#define WLAN_HOST_WAKE_GPIO		493

static int wlan_reg_on = -1;
#define DHD_DT_COMPAT_ENTRY		"android,bcmdhd_wlan"
#define WIFI_WL_REG_ON_PROPNAME		"wl_reg_on"

static int wlan_host_wake_up = -1;
#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
static int wlan_host_wake_irq = 0;
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */
#define WIFI_WLAN_HOST_WAKE_PROPNAME    "wl_host_wake"

static int resched_streak = 0;
static int resched_streak_max = 0;
static uint64 last_resched_cnt_check_time_ns = 0;
static uint64 last_affinity_update_time_ns = 0;
/* force to switch to small core at beginning */
static bool is_irq_on_big_core = TRUE;

static int pcie_ch_num = EXYNOS_PCIE_CH_NUM;
#if defined(CONFIG_SOC_GOOGLE)
#define EXYNOS_PCIE_RC_ONOFF
extern int exynos_pcie_pm_resume(int);
extern void exynos_pcie_pm_suspend(int);
extern int exynos_pcie_l1_exit(int ch_num);
#endif /* CONFIG_SOC_GOOGLE */

#ifdef EXYNOS_PCIE_DEBUG
extern void exynos_pcie_register_dump(int ch_num);
#endif /* EXYNOS_PCIE_DEBUG */
#ifdef PRINT_WAKEUP_GPIO_STATUS
extern void exynos_pin_dbg_show(unsigned int pin, const char* str);
#endif /* PRINT_WAKEUP_GPIO_STATUS */

#ifdef DHD_COREDUMP
#define DEVICE_NAME "wlan"

static void sscd_release(struct device *dev);
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

static void sscd_release(struct device *dev)
{
	DHD_INFO(("%s: enter\n", __FUNCTION__));
}

/* trigger coredump */
static int
dhd_set_coredump(const char *buf, int buf_len, const char *info)
{
	struct sscd_platform_data *pdata = dev_get_platdata(&sscd_dev.dev);
	struct sscd_segment seg;

	if (pdata->sscd_report) {
		memset(&seg, 0, sizeof(seg));
		seg.addr = (void *) buf;
		seg.size = buf_len;
		pdata->sscd_report(&sscd_dev, &seg, 1, 0, info);
	}
	return 0;
}
#endif /* DHD_COREDUMP */

#ifdef GET_CUSTOM_MAC_ENABLE

#define CDB_PATH "/chosen/config"
#define WIFI_MAC "wlan_mac1"
static u8 wlan_mac[6] = {0};

typedef struct {
    char hw_id[MAX_HW_INFO_LEN];
    char sku[MAX_HW_INFO_LEN];
} sku_info_t;

static sku_info_t sku_table[] = {
	{ {"G9S9B"}, {"MMW"} },
	{ {"G8V0U"}, {"MMW"} },
	{ {"GFQM1"}, {"MMW"} },
	{ {"GB62Z"}, {"MMW"} },
	{ {"GE2AE"}, {"MMW"} },
	{ {"GQML3"}, {"MMW"} },
	{ {"GB7N6"}, {"ROW"} },
	{ {"GLU0G"}, {"ROW"} },
	{ {"GNA8F"}, {"ROW"} },
	{ {"GX7AS"}, {"ROW"} },
	{ {"GP4BC"}, {"ROW"} },
	{ {"GVU6C"}, {"ROW"} },
	{ {"GR1YH"}, {"JPN"} },
	{ {"GF5KQ"}, {"JPN"} },
	{ {"GPQ72"}, {"JPN"} },
	{ {"GB17L"}, {"JPN"} },
	{ {"GFE4J"}, {"JPN"} },
	{ {"G03Z5"}, {"JPN"} },
	{ {"G1AZG"}, {"EU"} }
};

static int
dhd_wlan_get_mac_addr(unsigned char *buf)
{
	if (memcmp(wlan_mac, "\0\0\0\0\0\0", 6)) {
		memcpy(buf, wlan_mac, sizeof(wlan_mac));
		return 0;
	}
	return -EIO;
}

int
dhd_wlan_init_mac_addr(void)
{
	u8 mac[6] = {0};
	unsigned int size;
	unsigned char *mac_addr = NULL;
	struct device_node *node;
	unsigned int mac_found = 0;

	node = of_find_node_by_path(CDB_PATH);
	if (!node) {
		DHD_ERROR(("CDB Node not created under %s\n", CDB_PATH));
		return -ENODEV;
	} else {
		mac_addr = (unsigned char *)
				of_get_property(node, WIFI_MAC, &size);
	}

	/* In case Missing Provisioned MAC Address, exit with error */
	if (!mac_addr) {
		DHD_ERROR(("Missing Provisioned MAC address\n"));
		return -EINVAL;
	}

	/* Start decoding MAC Address
	 * Note that 2 formats are supported for now
	 * AA:BB:CC:DD:EE:FF (with separating colons) and
	 * AABBCCDDEEFF (without separating colons)
	 */
	if (sscanf(mac_addr,
			"%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
			&mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
			&mac[5]) == 6) {
		mac_found = 1;
	} else if (sscanf(mac_addr,
			"%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
			&mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
			&mac[5]) == 6) {
		mac_found = 1;
	}

	/* Make sure Address decoding succeeds */
	if (!mac_found) {
		DHD_ERROR(("Invalid format for Provisioned MAC Address\n"));
		return -EINVAL;
	}

	/* Make sure Provisioned MAC Address is globally Administered */
	if (mac[0] & 2) {
		DHD_ERROR(("Invalid Provisioned MAC Address\n"));
		return -EINVAL;
	}

	memcpy(wlan_mac, mac, sizeof(mac));
	return 0;
}
#endif /* GET_CUSTOM_MAC_ENABLE */

#if defined(SUPPORT_MULTIPLE_NVRAM) || defined(SUPPORT_MULTIPLE_CLMBLOB)
enum {
	CHIP_REV_SKU = 0,
	CHIP_REV = 1,
	CHIP_SKU = 2,
	CHIP = 3,
	REV_SKU = 4,
	REV_ONLY = 5,
	SKU_ONLY = 6,
	NO_EXT_NAME = 7
};

#define PLT_PATH "/chosen/plat"

#ifndef CDB_PATH
#define CDB_PATH "/chosen/config"
#endif /* CDB_PATH */

#define HW_SKU    "sku"
#define HW_STAGE  "stage"
#define HW_MAJOR  "major"
#define HW_MINOR  "minor"

static char val_revision[MAX_HW_INFO_LEN] = "NA";
static char val_sku[MAX_HW_INFO_LEN] = "NA";

enum hw_stage_attr {
	DEV = 1,
	PROTO = 2,
	EVT = 3,
	DVT = 4,
	PVT = 5,
	MP = 6,
	HW_STAGE_MAX
};
typedef struct platform_hw_info {
	uint8 avail_bmap;
	char ext_name[MAX_FILE_COUNT][MAX_HW_EXT_LEN];
} platform_hw_info_t;
static platform_hw_info_t platform_hw_info;

static void
dhd_set_platform_ext_name(char *hw_rev, char* hw_sku)
{
	bzero(&platform_hw_info, sizeof(platform_hw_info_t));

	if (strncmp(hw_rev, "NA", MAX_HW_INFO_LEN) != 0) {
		if (strncmp(hw_sku, "NA", MAX_HW_INFO_LEN) != 0) {
			snprintf(platform_hw_info.ext_name[REV_SKU], MAX_HW_EXT_LEN, "_%s_%s",
				hw_rev, hw_sku);
			setbit(&platform_hw_info.avail_bmap, REV_SKU);
		}
		snprintf(platform_hw_info.ext_name[REV_ONLY], MAX_HW_EXT_LEN, "_%s", hw_rev);
		setbit(&platform_hw_info.avail_bmap, REV_ONLY);
	}

	if (strncmp(hw_sku, "NA", MAX_HW_INFO_LEN) != 0) {
		snprintf(platform_hw_info.ext_name[SKU_ONLY], MAX_HW_EXT_LEN, "_%s", hw_sku);
		setbit(&platform_hw_info.avail_bmap, SKU_ONLY);
	}

#ifdef USE_CID_CHECK
	setbit(&platform_hw_info.avail_bmap, NO_EXT_NAME);
#endif /* USE_CID_CHECK */

	return;
}

void
dhd_set_platform_ext_name_for_chip_version(char* chip_version)
{
	if (strncmp(val_revision, "NA", MAX_HW_INFO_LEN) != 0) {
		if (strncmp(val_sku, "NA", MAX_HW_INFO_LEN) != 0) {
			snprintf(platform_hw_info.ext_name[CHIP_REV_SKU], MAX_HW_EXT_LEN,
				"%s_%s_%s", chip_version, val_revision, val_sku);
			setbit(&platform_hw_info.avail_bmap, CHIP_REV_SKU);
		}

		snprintf(platform_hw_info.ext_name[CHIP_REV], MAX_HW_EXT_LEN, "%s_%s",
			chip_version, val_revision);
		setbit(&platform_hw_info.avail_bmap, CHIP_REV);
	}
	if (strncmp(val_sku, "NA", MAX_HW_INFO_LEN) != 0) {
		snprintf(platform_hw_info.ext_name[CHIP_SKU], MAX_HW_EXT_LEN, "%s_%s",
			chip_version, val_sku);
		setbit(&platform_hw_info.avail_bmap, CHIP_SKU);
	}

	snprintf(platform_hw_info.ext_name[CHIP], MAX_HW_EXT_LEN, "%s", chip_version);
	setbit(&platform_hw_info.avail_bmap, CHIP);

	return;
}

static int
dhd_check_file_exist(char* fname)
{
	int err = BCME_OK;
#ifdef DHD_LINUX_STD_FW_API
	const struct firmware *fw = NULL;
#else
	struct file *filep = NULL;
	mm_segment_t fs;
#endif /* DHD_LINUX_STD_FW_API */

	if (fname == NULL) {
		DHD_ERROR(("%s: ERROR fname is NULL \n", __FUNCTION__));
		return BCME_ERROR;
	}

#ifdef DHD_LINUX_STD_FW_API
	err = dhd_os_get_img_fwreq(&fw, fname);
	if (err < 0) {
		DHD_LOG_MEM(("dhd_os_get_img(Request Firmware API) error : %d\n",
			err));
		goto fail;
	}
#else
	fs = get_fs();
	set_fs(KERNEL_DS);

	filep = dhd_filp_open(fname, O_RDONLY, 0);
	if (IS_ERR(filep) || (filep == NULL)) {
		DHD_LOG_MEM(("%s: Failed to open %s \n",  __FUNCTION__, fname));
		err = BCME_NOTFOUND;
		goto fail;
	}
#endif /* DHD_LINUX_STD_FW_API */

fail:
#ifdef DHD_LINUX_STD_FW_API
	if (fw) {
		dhd_os_close_img_fwreq(fw);
	}
#else
	if (!IS_ERR(filep))
		dhd_filp_close(filep, NULL);

	set_fs(fs);
#endif /* DHD_LINUX_STD_FW_API */
	return err;
}

int
dhd_get_platform_naming_for_nvram_clmblob_file(download_type_t component, char *file_name)
{
	int i, error = BCME_OK;
	char *nvram_clmblob_file;
	char tmp_fname[MAX_FILE_LEN] = {0, };

	if (!platform_hw_info.avail_bmap) {
		DHD_ERROR(("ext_name is not composed.\n"));
		return BCME_ERROR;
	}

	if (component == NVRAM) {
#ifdef DHD_LINUX_STD_FW_API
		nvram_clmblob_file = DHD_NVRAM_NAME;
#else
		nvram_clmblob_file = CONFIG_BCMDHD_NVRAM_PATH;
#endif /* DHD_LINUX_STD_FW_API */
	}
	else if (component == CLM_BLOB) {
#ifdef DHD_LINUX_STD_FW_API
		nvram_clmblob_file = DHD_CLM_NAME;
#else
		nvram_clmblob_file = VENDOR_PATH CONFIG_BCMDHD_CLM_PATH;
#endif /* DHD_LINUX_STD_FW_API */
	}

	for (i = 0; i < MAX_FILE_COUNT; i++) {
		if (!isset(&platform_hw_info.avail_bmap, i)) {
			continue;
		}
		memset_s(tmp_fname, MAX_FILE_LEN, 0, MAX_FILE_LEN);
		snprintf(tmp_fname, MAX_FILE_LEN,
			"%s%s", nvram_clmblob_file, platform_hw_info.ext_name[i]);
		error = dhd_check_file_exist(tmp_fname);
		if (error == BCME_OK) {
			DHD_LOG_MEM(("%02d path[%s]\n", i, tmp_fname));
			strlcpy(file_name, tmp_fname, MAX_FILE_LEN);
			break;
		}
	}
	return error;
}

int
dhd_wlan_init_hardware_info(void)
{

	struct device_node *node = NULL;
	const char *hw_sku = NULL;
	int hw_stage = -1;
	int hw_major = -1;
	int hw_minor = -1;
	int i;

	node = of_find_node_by_path(PLT_PATH);
	if (!node) {
		DHD_ERROR(("Node not created under %s\n", PLT_PATH));
		goto exit;
	} else {

		if (of_property_read_u32(node, HW_STAGE, &hw_stage)) {
			DHD_ERROR(("%s: Failed to get hw stage\n", __FUNCTION__));
			goto exit;
		}

		if (of_property_read_u32(node, HW_MAJOR, &hw_major)) {
			DHD_ERROR(("%s: Failed to get hw major\n", __FUNCTION__));
			goto exit;
		}

		if (of_property_read_u32(node, HW_MINOR, &hw_minor)) {
			DHD_ERROR(("%s: Failed to get hw minor\n", __FUNCTION__));
			goto exit;
		}

		switch (hw_stage) {
			case DEV:
				snprintf(val_revision, MAX_HW_INFO_LEN, "DEV%d.%d",
					hw_major, hw_minor);
				break;
			case PROTO:
				snprintf(val_revision, MAX_HW_INFO_LEN, "PROTO%d.%d",
					hw_major, hw_minor);
				break;
			case EVT:
				snprintf(val_revision, MAX_HW_INFO_LEN, "EVT%d.%d",
					hw_major, hw_minor);
				break;
			case DVT:
				snprintf(val_revision, MAX_HW_INFO_LEN, "DVT%d.%d",
					hw_major, hw_minor);
				break;
			case PVT:
				snprintf(val_revision, MAX_HW_INFO_LEN, "PVT%d.%d",
					hw_major, hw_minor);
				break;
			case MP:
				snprintf(val_revision, MAX_HW_INFO_LEN, "MP%d.%d",
					hw_major, hw_minor);
				break;
			default:
				strcpy(val_revision, "NA");
				break;
		}
	}

	node = of_find_node_by_path(CDB_PATH);
	if (!node) {
		DHD_ERROR(("Node not created under %s\n", CDB_PATH));
		goto exit;
	} else {

		if (of_property_read_string(node, HW_SKU, &hw_sku)) {
			DHD_ERROR(("%s: Failed to get hw sku\n", __FUNCTION__));
			goto exit;
		}

		for (i = 0; i < ARRAYSIZE(sku_table); i ++) {
			if (strcmp(hw_sku, sku_table[i].hw_id) == 0) {
				strcpy(val_sku, sku_table[i].sku);
				break;
			}
		}
		DHD_ERROR(("%s: hw_sku is %s, val_sku is %s\n", __FUNCTION__, hw_sku, val_sku));
	}

exit:
	dhd_set_platform_ext_name(val_revision, val_sku);

	return 0;
}
#endif /* SUPPORT_MULTIPLE_NVRAM || SUPPORT_MULTIPLE_CLMBLOB */

int
dhd_wifi_init_gpio(void)
{
	int gpio_reg_on_val;
	/* ========== WLAN_PWR_EN ============ */
	char *wlan_node = DHD_DT_COMPAT_ENTRY;
	struct device_node *root_node = NULL;

	root_node = of_find_compatible_node(NULL, NULL, wlan_node);
	if (!root_node) {
		DHD_ERROR(("failed to get device node of BRCM WLAN\n"));
		return -ENODEV;
	}

	wlan_reg_on = of_get_named_gpio(root_node, WIFI_WL_REG_ON_PROPNAME, 0);
	if (!gpio_is_valid(wlan_reg_on)) {
		DHD_ERROR(("Invalid gpio pin : %d\n", wlan_reg_on));
		return -ENODEV;
	}

	/* ========== WLAN_PWR_EN ============ */
	DHD_INFO(("%s: gpio_wlan_power : %d\n", __FUNCTION__, wlan_reg_on));

	/*
	 * For reg_on, gpio_request will fail if the gpio is configured to output-high
	 * in the dts using gpio-hog, so do not return error for failure.
	 */
	if (gpio_request_one(wlan_reg_on, GPIOF_OUT_INIT_HIGH, "WL_REG_ON")) {
		DHD_ERROR(("%s: Failed to request gpio %d for WL_REG_ON, "
			"might have configured in the dts\n",
			__FUNCTION__, wlan_reg_on));
	} else {
		DHD_ERROR(("%s: gpio_request WL_REG_ON done - WLAN_EN: GPIO %d\n",
			__FUNCTION__, wlan_reg_on));
	}

	gpio_reg_on_val = gpio_get_value(wlan_reg_on);
	DHD_INFO(("%s: Initial WL_REG_ON: [%d]\n",
		__FUNCTION__, gpio_get_value(wlan_reg_on)));

	if (gpio_reg_on_val == 0) {
		DHD_INFO(("%s: WL_REG_ON is LOW, drive it HIGH\n", __FUNCTION__));
		if (gpio_direction_output(wlan_reg_on, 1)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
			return -EIO;
		}
	}

	DHD_ERROR(("%s: WL_REG_ON is pulled up\n", __FUNCTION__));

	/* Wait for WIFI_TURNON_DELAY due to power stability */
	msleep(WIFI_TURNON_DELAY);

#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
	/* ========== WLAN_HOST_WAKE ============ */
	wlan_host_wake_up = of_get_named_gpio(root_node,
		WIFI_WLAN_HOST_WAKE_PROPNAME, 0);
	DHD_INFO(("%s: gpio_wlan_host_wake : %d\n", __FUNCTION__, wlan_host_wake_up));

	if (gpio_request_one(wlan_host_wake_up, GPIOF_IN, "WLAN_HOST_WAKE")) {
		DHD_ERROR(("%s: Failed to request gpio %d for WLAN_HOST_WAKE\n",
			__FUNCTION__, wlan_host_wake_up));
			return -ENODEV;
	} else {
		DHD_ERROR(("%s: gpio_request WLAN_HOST_WAKE done"
			" - WLAN_HOST_WAKE: GPIO %d\n",
			__FUNCTION__, wlan_host_wake_up));
	}

	if (gpio_direction_input(wlan_host_wake_up)) {
		DHD_ERROR(("%s: Failed to set WL_HOST_WAKE gpio direction\n", __FUNCTION__));
	}

	wlan_host_wake_irq = gpio_to_irq(wlan_host_wake_up);
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */
	return 0;
}

int
dhd_wlan_power(int onoff)
{
	DHD_INFO(("------------------------------------------------\n"));
	DHD_INFO(("------------------------------------------------\n"));
	DHD_INFO(("%s Enter: power %s\n", __func__, onoff ? "on" : "off"));

	if (onoff) {
		if (gpio_direction_output(wlan_reg_on, 1)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
			return -EIO;
		}
		if (gpio_get_value(wlan_reg_on)) {
			DHD_INFO(("WL_REG_ON on-step-2 : [%d]\n",
				gpio_get_value(wlan_reg_on)));
		} else {
			DHD_ERROR(("[%s] gpio value is 0. We need reinit.\n", __func__));
			if (gpio_direction_output(wlan_reg_on, 1)) {
				DHD_ERROR(("%s: WL_REG_ON is "
					"failed to pull up\n", __func__));
			}
		}
	} else {
		if (gpio_direction_output(wlan_reg_on, 0)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
			return -EIO;
		}
		if (gpio_get_value(wlan_reg_on)) {
			DHD_INFO(("WL_REG_ON on-step-2 : [%d]\n",
				gpio_get_value(wlan_reg_on)));
		}
	}
	return 0;
}
EXPORT_SYMBOL(dhd_wlan_power);

static int
dhd_wlan_reset(int onoff)
{
	return 0;
}

static int
dhd_wlan_set_carddetect(int val)
{
#ifdef EXYNOS_PCIE_RC_ONOFF
	struct device_node *root_node = NULL;
	char *wlan_node = DHD_DT_COMPAT_ENTRY;

	root_node = of_find_compatible_node(NULL, NULL, wlan_node);
	if (!root_node) {
		DHD_ERROR(("failed to get device node of BRCM WLAN\n"));
		return -ENODEV;
	}

	if (of_property_read_u32(root_node, "ch-num", &pcie_ch_num)) {
		DHD_INFO(("%s: Failed to parse the channel number\n", __FUNCTION__));
		return -EINVAL;
	}
	/* ========== WLAN_PCIE_NUM ============ */
	DHD_INFO(("%s: pcie_ch_num : %d\n", __FUNCTION__, pcie_ch_num));
#endif /* EXYNOS_PCIE_RC_ONOFF */

	if (val) {
		exynos_pcie_pm_resume(pcie_ch_num);
	} else {
		printk(KERN_INFO "%s Ignore carddetect: %d\n", __FUNCTION__, val);
	}
	return 0;
}

#include <linux/exynos-pci-noti.h>
extern int exynos_pcie_register_event(struct exynos_pcie_register_event *reg);
extern int exynos_pcie_deregister_event(struct exynos_pcie_register_event *reg);

#include <dhd_plat.h>

typedef struct dhd_plat_info {
	struct exynos_pcie_register_event pcie_event;
	struct exynos_pcie_notify pcie_notify;
	struct pci_dev *pdev;
} dhd_plat_info_t;

static dhd_pcie_event_cb_t g_pfn = NULL;

uint32 dhd_plat_get_info_size(void)
{
	return sizeof(dhd_plat_info_t);
}

void plat_pcie_notify_cb(struct exynos_pcie_notify *pcie_notify)
{
	struct pci_dev *pdev;

	if (pcie_notify == NULL) {
		pr_err("%s(): Invalid argument to Platform layer call back \r\n", __func__);
		return;
	}

	if (g_pfn) {
		pdev = (struct pci_dev *)pcie_notify->user;
		pr_err("%s(): Invoking DHD call back with pdev %p \r\n",
				__func__, pdev);
		(*(g_pfn))(pdev);
	} else {
		pr_err("%s(): Driver Call back pointer is NULL \r\n", __func__);
	}
	return;
}

int dhd_plat_pcie_register_event(void *plat_info, struct pci_dev *pdev, dhd_pcie_event_cb_t pfn)
{
		dhd_plat_info_t *p = plat_info;

		if ((p == NULL) || (pdev == NULL) || (pfn == NULL)) {
			pr_err("%s(): Invalid argument p %p, pdev %p, pfn %p\r\n",
				__func__, p, pdev, pfn);
			return -1;
		}
		g_pfn = pfn;
		p->pdev = pdev;
#ifdef CPL_TIMEOUT_RECOVERY
		p->pcie_event.events = EXYNOS_PCIE_EVENT_LINKDOWN | EXYNOS_PCIE_EVENT_CPL_TIMEOUT;
#else
		p->pcie_event.events = EXYNOS_PCIE_EVENT_LINKDOWN;
#endif /* CPL_TIMEOUT_RECOVERY */
		p->pcie_event.user = pdev;
		p->pcie_event.mode = EXYNOS_PCIE_TRIGGER_CALLBACK;
		p->pcie_event.callback = plat_pcie_notify_cb;
		exynos_pcie_register_event(&p->pcie_event);
		pr_err("%s(): Registered Event PCIe event pdev %p \r\n", __func__, pdev);
		return 0;
}

void dhd_plat_pcie_deregister_event(void *plat_info)
{
	dhd_plat_info_t *p = plat_info;
	if (p) {
		exynos_pcie_deregister_event(&p->pcie_event);
	}
	return;
}

static int
set_affinity(unsigned int irq, const struct cpumask *cpumask)
{
#ifdef BCMDHD_MODULAR
	return irq_set_affinity_hint(irq, cpumask);
#else
	return irq_set_affinity(irq, cpumask);
#endif
}

static void
irq_affinity_hysteresis_control(struct pci_dev *pdev, int resched_streak_max,
	uint64 curr_time_ns)
{
	int err = 0;
	bool has_recent_affinity_update = (curr_time_ns - last_affinity_update_time_ns)
		< (AFFINITY_UPDATE_MIN_PERIOD_SEC * NSEC_PER_SEC);
	if (!pdev) {
		DHD_ERROR(("%s : pdev is NULL\n", __FUNCTION__));
		return;
	}

	if (!is_irq_on_big_core && (resched_streak_max >= RESCHED_STREAK_MAX_HIGH) &&
		!has_recent_affinity_update) {
		err = set_affinity(pdev->irq, cpumask_of(IRQ_AFFINITY_BIG_CORE));
		if (!err) {
			is_irq_on_big_core = TRUE;
			last_affinity_update_time_ns = curr_time_ns;
			DHD_INFO(("%s switches to big core successfully\n", __FUNCTION__));
		} else {
			DHD_ERROR(("%s switches to big core unsuccessfully!\n", __FUNCTION__));
		}
	}
	if (is_irq_on_big_core && (resched_streak_max <= RESCHED_STREAK_MAX_LOW) &&
		!has_recent_affinity_update) {
		err = set_affinity(pdev->irq, cpumask_of(IRQ_AFFINITY_SMALL_CORE));
		if (!err) {
			is_irq_on_big_core = FALSE;
			last_affinity_update_time_ns = curr_time_ns;
			DHD_INFO(("%s switches to all cores successfully\n", __FUNCTION__));
		} else {
			DHD_ERROR(("%s switches to all cores unsuccessfully\n", __FUNCTION__));
		}
	}
}

/*
 * DHD Core layer reports whether the bottom half is getting rescheduled or not
 * resched = 1, BH is getting rescheduled.
 * resched = 0, BH is NOT getting rescheduled.
 * resched is used to detect bottom half load and configure IRQ affinity dynamically
 */
void dhd_plat_report_bh_sched(void *plat_info, int resched)
{
	dhd_plat_info_t *p = plat_info;
	uint64 curr_time_ns;
	uint64 time_delta_ns;

	if (resched > 0) {
		resched_streak++;
		return;
	}

	if (resched_streak > resched_streak_max) {
		resched_streak_max = resched_streak;
	}
	resched_streak = 0;

	curr_time_ns = OSL_LOCALTIME_NS();
	time_delta_ns = curr_time_ns - last_resched_cnt_check_time_ns;
	if (time_delta_ns < (RESCHED_CNT_CHECK_PERIOD_SEC * NSEC_PER_SEC)) {
		return;
	}
	last_resched_cnt_check_time_ns = curr_time_ns;

	DHD_INFO(("%s resched_streak_max=%d\n",
		__FUNCTION__, resched_streak_max));

	irq_affinity_hysteresis_control(p->pdev, resched_streak_max, curr_time_ns);

	resched_streak_max = 0;
	return;
}

#ifdef BCMSDIO
static int dhd_wlan_get_wake_irq(void)
{
	return gpio_to_irq(wlan_host_wake_up);
}
#endif /* BCMSDIO */

#if defined(CONFIG_BCMDHD_OOB_HOST_WAKE) && defined(CONFIG_BCMDHD_GET_OOB_STATE)
int
dhd_get_wlan_oob_gpio(void)
{
	return gpio_is_valid(wlan_host_wake_up) ?
		gpio_get_value(wlan_host_wake_up) : -1;
}
EXPORT_SYMBOL(dhd_get_wlan_oob_gpio);
int
dhd_get_wlan_oob_gpio_number(void)
{
	return gpio_is_valid(wlan_host_wake_up) ?
		wlan_host_wake_up : -1;
}
EXPORT_SYMBOL(dhd_get_wlan_oob_gpio_number);
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE && CONFIG_BCMDHD_GET_OOB_STATE */

struct resource dhd_wlan_resources = {
	.name	= "bcmdhd_wlan_irq",
	.start	= 0, /* Dummy */
	.end	= 0, /* Dummy */
	.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE |
	IORESOURCE_IRQ_HIGHLEVEL,
};
EXPORT_SYMBOL(dhd_wlan_resources);

struct wifi_platform_data dhd_wlan_control = {
	.set_power	= dhd_wlan_power,
	.set_reset	= dhd_wlan_reset,
	.set_carddetect	= dhd_wlan_set_carddetect,
#ifdef DHD_COREDUMP
	.set_coredump = dhd_set_coredump,
#endif /* DHD_COREDUMP */
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
#ifdef GET_CUSTOM_MAC_ENABLE
	.get_mac_addr = dhd_wlan_get_mac_addr,
#endif /* GET_CUSTOM_MAC_ENABLE */
#ifdef BCMSDIO
	.get_wake_irq	= dhd_wlan_get_wake_irq,
#endif // endif
};
EXPORT_SYMBOL(dhd_wlan_control);


#ifdef WLAN_TRACKER
#include <wlan_ptracker_client.h>
#include <wldev_common.h>
#include <wl_cfg80211.h>
#include <dhd_linux_priv.h>

static int
dhd_twt_setup(void *priv, struct dytwt_setup_param *setup)
{
	wl_twt_config_t val;
	struct net_device *dev = (struct net_device *)priv;
	s32 bw;
	u8 mybuf[WLC_IOCTL_SMLEN] = {0};
	u8 resp_buf[WLC_IOCTL_SMLEN] = {0};
	uint8 *rem = mybuf;
	uint16 rem_len = sizeof(mybuf);

	bzero(&val, sizeof(val));
	val.version = WL_TWT_SETUP_VER;
	val.length = sizeof(val.version) + sizeof(val.length);

	/* Default values, Override Below */
	val.desc.flow_flags = 0;
	val.desc.wake_time_h = 0xFFFFFFFF;
	val.desc.wake_time_l = 0xFFFFFFFF;
	val.desc.wake_int_min = 0xFFFFFFFF;
	val.desc.wake_int_max = 0xFFFFFFFF;
	val.desc.wake_dur_min = 0xFFFFFFFF;
	val.desc.wake_dur_max = 0xFFFFFFFF;
	val.desc.avg_pkt_num  = 0xFFFFFFFF;
	val.desc.avg_pkt_size = 0xFFFFFFFF;

	/* Flow Flag for setup cmd: request, flow type: unannounced  */
	val.desc.flow_flags |= WL_TWT_FLOW_FLAG_REQUEST;
	val.desc.flow_flags |= WL_TWT_FLOW_FLAG_UNANNOUNCED;
	/* Config ID */
	val.desc.configID = setup->config_id;
	/* negotiation_type */
	val.desc.negotiation_type  = setup->nego_type;
	/* Wake Duration */
	val.desc.wake_dur = setup->wake_duration;
	/* Wake interval */
	val.desc.wake_int = setup->wake_interval;

	bw = bcm_pack_xtlv_entry(&rem, &rem_len, WL_TWT_CMD_CONFIG,
			sizeof(val), (uint8 *)&val, BCM_XTLV_OPTION_ALIGN32);
	if (bw != BCME_OK) {
		goto exit;
	}

	bw = wldev_iovar_setbuf(dev, "twt",
		mybuf, sizeof(mybuf) - rem_len, resp_buf, WLC_IOCTL_SMLEN, NULL);
	if (bw < 0) {
		pr_err("twt config set failed. ret:%d\n", bw);
	} else {
		pr_err("twt config setup succeeded, config ID %d "
			"Negotiation type %d flow flags %d\n", val.desc.configID,
			val.desc.negotiation_type, val.desc.flow_flags);
	}
exit:
	return bw;
}

static int
dhd_twt_teardown(void *priv, struct dytwt_setup_param *setup)
{
	wl_twt_teardown_t val;
	struct net_device *dev = (struct net_device *)priv;
	s32 bw;
	u8 mybuf[WLC_IOCTL_SMLEN] = {0};
	u8 res_buf[WLC_IOCTL_SMLEN] = {0};
	uint8 *rem = mybuf;
	uint16 rem_len = sizeof(mybuf);

	bzero(&val, sizeof(val));
	val.version = WL_TWT_TEARDOWN_VER;
	val.length = sizeof(val.version) + sizeof(val.length);

	/* Default values, Override Below */
	val.teardesc.flow_id = 0xFF;
	val.teardesc.bid = 0xFF;
	/* Config ID */
	val.configID = setup->config_id;
	/* negotiation_type */
	val.teardesc.negotiation_type  = setup->nego_type;

	bw = bcm_pack_xtlv_entry(&rem, &rem_len, WL_TWT_CMD_TEARDOWN,
		sizeof(val), (uint8 *)&val, BCM_XTLV_OPTION_ALIGN32);
	if (bw != BCME_OK) {
		goto exit;
	}

	bw = wldev_iovar_setbuf(dev, "twt",
		mybuf, sizeof(mybuf) - rem_len, res_buf, WLC_IOCTL_SMLEN, NULL);
	if (bw < 0) {
		pr_err("twt teardown failed. ret:%d\n", bw);
	} else {
		pr_err("twt teardown succeeded, config ID %d "
			"Negotiation type %d alltwt %d\n", val.configID,
			val.teardesc.negotiation_type, val.teardesc.alltwt);
	}
exit:
	return bw;
}

static void dhd_twt_stats2_dytwt(wl_twt_stats_v2_t *src, struct dytwt_stats *dest)
{
	wl_twt_peer_stats_v2_t *peer_stats = &src->peer_stats_list[0];

	dest->sp_seq = peer_stats->sp_seq;
	dest->tx_ucast_pkts = peer_stats->tx_ucast_pkts;
	dest->tx_pkts_min = peer_stats->tx_pkts_min;
	dest->tx_pkts_max = peer_stats->tx_pkts_max;
	dest->tx_pkts_avg = peer_stats->tx_pkts_avg;
	dest->tx_failures = peer_stats->tx_failures;
	dest->rx_ucast_pkts = peer_stats->rx_ucast_pkts;
	dest->rx_pkts_min = peer_stats->rx_pkts_min;
	dest->rx_pkts_max = peer_stats->rx_pkts_max;
	dest->rx_pkts_avg = peer_stats->rx_pkts_avg;
	dest->rx_pkts_retried = peer_stats->rx_pkts_retried;
	dest->tx_pkt_sz_avg = peer_stats->tx_pkt_sz_avg;
	dest->rx_pkt_sz_avg = peer_stats->rx_pkt_sz_avg;
	dest->eosp_dur_avg = peer_stats->eosp_dur_avg;
	dest->eosp_count = peer_stats->eosp_count;
}

static int
dhd_twt_get_stats(void *priv, struct dytwt_stats *stats)
{
	struct net_device *dev = (struct net_device *)priv;
	wl_twt_stats_cmd_v1_t query;
	wl_twt_stats_v2_t stats_v2;
	int ret = BCME_OK;
	char iovbuf[WLC_IOCTL_SMLEN] = {0, };
	uint8 *pxtlv = NULL;
	uint8 *iovresp = NULL;
	uint16 buflen = 0, bufstart = 0;
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);

	bzero(&query, sizeof(query));
	query.version = WL_TWT_STATS_CMD_VERSION_1;
	query.length = sizeof(query) - OFFSETOF(wl_twt_stats_cmd_v1_t, peer);

	/* Default values, Override Below */
	query.num_bid = 0xFF;
	query.num_fid = 0xFF;
	query.configID = stats->config_id;

	iovresp = (uint8 *)MALLOCZ(cfg->osh, WLC_IOCTL_MEDLEN);
	if (iovresp == NULL) {
		WL_ERR(("%s: iov resp memory alloc exited\n", __FUNCTION__));
		goto exit;
	}
	//query.flags |= WL_TWT_STATS_CMD_FLAGS_RESET;
	buflen = bufstart = WLC_IOCTL_SMLEN;
	pxtlv = (uint8 *)iovbuf;
	ret = bcm_pack_xtlv_entry(&pxtlv, &buflen, WL_TWT_CMD_STATS,
			sizeof(query), (uint8 *)&query, BCM_XTLV_OPTION_ALIGN32);
	if (ret != BCME_OK) {
		WL_ERR(("%s : Error return during pack xtlv :%d\n", __FUNCTION__, ret));
		goto exit;
	}

	if ((ret = wldev_iovar_getbuf(dev, "twt", iovbuf, bufstart-buflen,
		iovresp, WLC_IOCTL_MEDLEN, NULL))) {
		WL_ERR(("twt status failed with err=%d \n", ret));
		goto exit;
	}

	(void)memcpy_s(&stats_v2, sizeof(stats_v2), iovresp, sizeof(stats_v2));
	if (dtoh16(stats_v2.version) == WL_TWT_STATS_VERSION_2 &&
		stats_v2.num_stats) {
			dhd_twt_stats2_dytwt((wl_twt_stats_v2_t*)iovresp, stats);
	} else {
		ret = BCME_UNSUPPORTED;
		WL_ERR(("Version 1 unsupported. ver %d, \n", dtoh16(stats_v2.version)));
		goto exit;
	}
exit:
	if (iovresp) {
		MFREE(cfg->osh, iovresp, WLC_IOCTL_MEDLEN);
	}
	return ret;
}

static void dhd_twt_status2_dytwt(
	wl_twt_status_v1_t *result,
	struct dytwt_status *status)
{
	int i;
	wl_twt_sdesc_v0_t *sdesc = NULL;

	for (i = 0 ; i < WL_TWT_MAX_ITWT; i++) {
		if (result->itwt_status[i].configID != status->config_id)
			continue;
		sdesc = &result->itwt_status[i].desc;
		status->config_id = result->itwt_status[i].configID;
		status->flow_id = sdesc->flow_id;
		status->flow_flags = sdesc->flow_flags;
		status->setup_cmd = sdesc->setup_cmd;
		status->channel = sdesc->channel;
		status->nego_type = sdesc->negotiation_type;
		status->wake_dur = sdesc->wake_dur;
		status->wake_int = sdesc->wake_int;
	}
}

static int
dhd_twt_get_status(void *priv, struct dytwt_status *status)
{
	struct net_device *dev = (struct net_device *)priv;
	wl_twt_status_cmd_v1_t query;
	wl_twt_status_v1_t result;
	int ret = BCME_OK;
	char iovbuf[WLC_IOCTL_SMLEN] = {0, };
	uint8 *pxtlv = NULL;
	uint8 *iovresp = NULL;
	uint16 buflen = 0, bufstart = 0;
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);

	bzero(&query, sizeof(query));
	bzero(&result, sizeof(result));
	query.version = WL_TWT_CMD_STATUS_VERSION_1;
	query.length = sizeof(query) - OFFSETOF(wl_twt_status_cmd_v1_t, peer);

	query.configID = status->config_id;

	iovresp = (uint8 *)MALLOCZ(cfg->osh, WLC_IOCTL_MEDLEN);
	if (iovresp == NULL) {
		WL_ERR(("%s: iov resp memory alloc exited\n", __FUNCTION__));
		goto exit;
	}
	buflen = bufstart = WLC_IOCTL_SMLEN;
	pxtlv = (uint8 *)iovbuf;
	ret = bcm_pack_xtlv_entry(&pxtlv, &buflen, WL_TWT_CMD_STATUS,
			sizeof(query), (uint8 *)&query, BCM_XTLV_OPTION_ALIGN32);
	if (ret != BCME_OK) {
		WL_ERR(("%s : Error return during pack xtlv :%d\n", __FUNCTION__, ret));
		goto exit;
	}

	if ((ret = wldev_iovar_getbuf(dev, "twt", iovbuf, bufstart-buflen,
		iovresp, WLC_IOCTL_MEDLEN, NULL))) {
		WL_ERR(("twt status failed with err=%d \n", ret));
		goto exit;
	}

	(void)memcpy_s(&result, sizeof(result), iovresp, sizeof(result));
	printk("wlan_ptracker: %s(): version: %d, length: %d\n",
		__func__, result.version, result.length);
	if (dtoh16(result.version) == WL_TWT_CMD_STATUS_VERSION_1) {
			dhd_twt_status2_dytwt(&result, status);
	} else {
		ret = BCME_UNSUPPORTED;
		WL_ERR(("Version 1 unsupported. ver %d, \n", dtoh16(result.version)));
		goto exit;
	}
exit:
	if (iovresp) {
		MFREE(cfg->osh, iovresp, WLC_IOCTL_MEDLEN);
	}
	return ret;
}

static int
dhd_twt_cap(void *priv, struct dytwt_cap *cap)
{
	int ret = BCME_OK;
	struct net_device *dev = (struct net_device *)priv;
	char iovbuf[WLC_IOCTL_SMLEN] = {0, };
	uint8 *pxtlv = NULL;
	uint8 *iovresp = NULL;
	wl_twt_cap_cmd_t cmd_cap;
	wl_twt_cap_t result;
	scb_val_t scbval;
	uint16 buflen = 0, bufstart = 0;
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);

	/* Query link speed */
	ret = wldev_get_link_speed(dev, &cap->link_speed);
	if (unlikely(ret)) {
		WL_ERR(("get_link speed error (%d)\n", ret));
		goto exit;
	}

	/* Query RSSI */
	bzero(&scbval, sizeof(scb_val_t));
	ret = wldev_get_rssi(dev, &scbval);
	if (unlikely(ret)) {
		WL_ERR(("get_rssi error (%d)\n", ret));
		goto exit;
	}
	cap->rssi = scbval.val;
	bzero(&cmd_cap, sizeof(cmd_cap));

	cmd_cap.version = WL_TWT_CAP_CMD_VERSION_1;
	cmd_cap.length = sizeof(cmd_cap) - OFFSETOF(wl_twt_cap_cmd_t, peer);

	iovresp = (uint8 *)MALLOCZ(cfg->osh, WLC_IOCTL_MEDLEN);
	if (iovresp == NULL) {
		pr_err("%s: iov resp memory alloc exited\n", __FUNCTION__);
		goto exit;
	}

	buflen = bufstart = WLC_IOCTL_SMLEN;
	pxtlv = (uint8 *)iovbuf;

	ret = bcm_pack_xtlv_entry(&pxtlv, &buflen, WL_TWT_CMD_CAP,
			sizeof(cmd_cap), (uint8 *)&cmd_cap, BCM_XTLV_OPTION_ALIGN32);
	if (ret != BCME_OK) {
		pr_err("%s : Error return during pack xtlv :%d\n", __FUNCTION__, ret);
		goto exit;
	}

	if ((ret = wldev_iovar_getbuf(dev, "twt", iovbuf, bufstart-buflen,
		iovresp, WLC_IOCTL_MEDLEN, NULL))) {
		pr_err("Getting twt status failed with err=%d \n", ret);
		goto exit;
	}

	(void)memcpy_s(&result, sizeof(result), iovresp, sizeof(result));

	if (dtoh16(result.version) == WL_TWT_CAP_CMD_VERSION_1) {
		pr_err("capability ver %d, \n", dtoh16(result.version));
		cap->device_cap = dtoh16(result.device_cap);
		cap->peer_cap = dtoh16(result.peer_cap);
		return ret;
	} else {
		ret = BCME_UNSUPPORTED;
		pr_err("Version 1 unsupported. ver %d, \n", dtoh16(result.version));
		goto exit;
	}
exit:
	if (iovresp) {
		MFREE(cfg->osh, iovresp, WLC_IOCTL_MEDLEN);
	}
	return ret;
}

extern 	ssize_t show_pwrstats_path(struct dhd_info *dev, char *buf);

static u64 twt_pwrstate_hex_get(char *buf)
{
	char *ptr = buf;
	char *token = strsep(&ptr, "\x0a");

	/* remove prefix space */
	token++;
	return simple_strtoull(token, NULL, 16);
}

#define DHD_PWR_STAT_STR_SIZE 512
static int dhd_twt_pwstate(void *priv, struct dytwt_pwr_state *state)
{
	char buf[DHD_PWR_STAT_STR_SIZE];
	struct net_device *dev = (struct net_device *)priv;
	struct dhd_info *dhd = DHD_DEV_INFO(dev);
	char *ptr = &buf[0];
	char *token;
	int cnt = 0;
	int ret;

	ret = show_pwrstats_path(dhd, buf);
	if (!ret)
		goto out;

	token = strsep(&ptr, ":");
	while (token) {
		if (cnt == 3)
			state->awake = twt_pwrstate_hex_get(token);
		if (cnt == 6)
			state->count = twt_pwrstate_hex_get(token);
		if (cnt == 7)
			state->asleep = twt_pwrstate_hex_get(token);
		token = strsep(&ptr, ":");
		cnt++;
	}
out:
	return 0;
}

struct dytwt_client_ops twt_ops = {
	.setup = dhd_twt_setup,
	.teardown = dhd_twt_teardown,
	.get_cap = dhd_twt_cap,
	.get_pwrstates = dhd_twt_pwstate,
	.get_stats = dhd_twt_get_stats,
	.get_status = dhd_twt_get_status,
};

struct wlan_ptracker_client client = {
	.ifname = "wlan0",
	.dytwt_ops = &twt_ops,
	.cb = NULL,
};

static int dhd_plat_ptracker_register(void)
{
	return wlan_ptracker_register_client(&client);
}

static void dhd_plat_ptracker_unregister(void)
{
	wlan_ptracker_unregister_client(&client);
}

static u32 custom_notify_table[] = {
	WLAN_PTRACKER_NOTIFY_SUSPEN, //CUSTOM_NOTIFY_BUS_SUSPEND,
	WLAN_PTRACKER_NOTIFY_STA_CONNECT, //CUSTOM_NOTIFY_STA_CONNECT,
	WLAN_PTRACKER_NOTIFY_STA_DISCONNECT, //CUSTOM_NOTIFY_STA_DISCONNECT,
	WLAN_PTRACKER_NOTIFY_DYTWT_DISABLE, //CUSTOM_NOTIFY_TWT_SETUP,
	WLAN_PTRACKER_NOTIFY_DYTWT_ENABLE, //CUSTOM_NOTIFY_TWT_TEARDOWN,
};

int
dhd_custom_notify(u32 id)
{
	u32 _id = custom_notify_table[id];

	if (!client.cb)
		return 0;
	return client.cb(&client, _id);
}
#endif /* WLAN_TRACKER */

int
dhd_wlan_init(void)
{
	int ret;

	DHD_INFO(("%s: START.......\n", __FUNCTION__));

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	ret = dhd_init_wlan_mem();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to alloc reserved memory,"
					" ret=%d\n", __FUNCTION__, ret));
		goto fail;
	}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#ifdef DHD_COREDUMP
	platform_device_register(&sscd_dev);
#endif /* DHD_COREDUMP */

	ret = dhd_wifi_init_gpio();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to initiate GPIO, ret=%d\n",
			__FUNCTION__, ret));
		goto fail;
	}
#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
	dhd_wlan_resources.start = wlan_host_wake_irq;
	dhd_wlan_resources.end = wlan_host_wake_irq;
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */

#ifdef GET_CUSTOM_MAC_ENABLE
	dhd_wlan_init_mac_addr();
#endif /* GET_CUSTOM_MAC_ENABLE */

#if defined(SUPPORT_MULTIPLE_NVRAM) || defined(SUPPORT_MULTIPLE_CLMBLOB)
	dhd_wlan_init_hardware_info();
#endif /* SUPPORT_MULTIPLE_NVRAM || SUPPORT_MULTIPLE_CLMBLOB */

#ifdef WLAN_TRACKER
	dhd_plat_ptracker_register();
#endif /* WLAN_TRACKER */

fail:
	DHD_ERROR(("%s: FINISH.......\n", __FUNCTION__));
	return ret;
}

int
dhd_wlan_deinit(void)
{
	if (gpio_is_valid(wlan_host_wake_up)) {
		gpio_free(wlan_host_wake_up);
	}
	if (gpio_is_valid(wlan_reg_on)) {
		gpio_free(wlan_reg_on);
	}

#ifdef WLAN_TRACKER
	dhd_plat_ptracker_unregister();
#endif /* WLAN_TRACKER */

#ifdef DHD_COREDUMP
	platform_device_unregister(&sscd_dev);
#endif /* DHD_COREDUMP */

	return 0;
}

void dhd_plat_l1ss_ctrl(bool ctrl)
{
#if defined(CONFIG_SOC_GOOGLE)
	DHD_CONS_ONLY(("%s: Control L1ss RC side %d \n", __FUNCTION__, ctrl));
	exynos_pcie_rc_l1ss_ctrl(ctrl, PCIE_L1SS_CTRL_WIFI, 1);
#endif /* CONFIG_SOC_GOOGLE */
	return;
}

void dhd_plat_l1_exit_io(void)
{
#if defined(DHD_PCIE_L1_EXIT_DURING_IO)
	exynos_pcie_l1_exit(pcie_ch_num);
#endif /* DHD_PCIE_L1_EXIT_DURING_IO */
	return;
}

void dhd_plat_l1_exit(void)
{
	exynos_pcie_l1_exit(pcie_ch_num);
	return;
}

int dhd_plat_pcie_suspend(void *plat_info)
{
	exynos_pcie_pm_suspend(pcie_ch_num);
	return 0;
}

int dhd_plat_pcie_resume(void *plat_info)
{
	int ret = 0;
	ret = exynos_pcie_pm_resume(pcie_ch_num);
	is_irq_on_big_core = true;
	return ret;
}

void dhd_plat_pin_dbg_show(void *plat_info)
{
#ifdef PRINT_WAKEUP_GPIO_STATUS
	exynos_pin_dbg_show(dhd_get_wlan_oob_gpio_number(), "gpa0");
#endif /* PRINT_WAKEUP_GPIO_STATUS */
}

void dhd_plat_pcie_register_dump(void *plat_info)
{
#ifdef EXYNOS_PCIE_DEBUG
	exynos_pcie_register_dump(1);
#endif /* EXYNOS_PCIE_DEBUG */
}

uint32 dhd_plat_get_rc_vendor_id(void)
{
	return EXYNOS_PCIE_VENDOR_ID;
}

uint32 dhd_plat_get_rc_device_id(void)
{
	return EXYNOS_PCIE_DEVICE_ID;
}

#define RXBUF_ALLOC_PAGE_SIZE
uint16 dhd_plat_align_rxbuf_size(uint16 rxbufpost_sz)
{
#ifdef RXBUF_ALLOC_PAGE_SIZE
	/* The minimum number of pages is 1 */
	uint16 num_pages = rxbufpost_sz / PAGE_SIZE + 1;
	/*
	 * Align sk buffer + skb overhead + NET_SKB_PAD with the page size boundary
	 * Refer to __netdev_alloc_skb() in skbuff.c for details.
	 */
	if (rxbufpost_sz > (SKB_WITH_OVERHEAD(num_pages * PAGE_SIZE) - NET_SKB_PAD)) {
		num_pages++;
	}
	return SKB_WITH_OVERHEAD(num_pages * PAGE_SIZE) - NET_SKB_PAD;
#else
	return rxbufpost_sz;
#endif
}


#ifndef BCMDHD_MODULAR
/* Required only for Built-in DHD */
device_initcall(dhd_wlan_init);
#endif /* BCMDHD_MODULAR */
