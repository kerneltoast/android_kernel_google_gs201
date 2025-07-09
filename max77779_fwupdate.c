/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023, Google Inc
 *
 * MAX77779 firmware updater
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>

#include "google_bms.h"
#include "max77779_regs.h"
#include "max77779.h"
#include "maxfg_common.h"
#include "max77779_charger.h"

#define MAX77779_FIRMWARE_BINARY_PREFIX "batt_fw_adi_79"
#define MAX77779_REASON_FIRMWARE        "FW_UPDATE"

#define FW_UPDATE_RETRY_CPU_RESET             100
#define FW_UPDATE_RETRY_FW_UPDATE             1000
#define FW_UPDATE_RETRY_RISCV_REBOOT          20
#define FW_UPDATE_RETRY_ONCE                  1
#define FW_UPDATE_WAIT_INTERVAL_MS            50
#define FW_UPDATE_WAIT_LOAD_BIN_MS            50
#define FW_UPDATE_TIMER_CHECK_INTERVAL_MS     1000
#define FW_UPDATE_CONDITION_CHECK_INTERVAL_MS (60 * 1000)

#define FW_UPDATE_MAXIMUM_PAGE_SIZE            (PAGE_SIZE*10)

/* b/308445917: adding device tree for voltage threshold in micro volts */
#define MAX77779_FW_UPDATE_MIN_VOLTAGE 4000000

#define MAX77779_FW_IMG_SZ_HEADER 8
#define MAX77779_FW_IMG_SZ_PACKET 42
#define MAX77779_FW_IMG_SZ_FRAME (MAX77779_FW_IMG_SZ_PACKET * 20)


#define MAX77779_FG_SECUPDATE_STATUS_REG 0x6F
#define MAX77779_FG_SECUPDATE_STATUS_SUCCESS 0x03

/* PMIC.0x62 can be set 0xFF if previous firmware update fails */
#define MAX77779_FW_INVALID_FW_VER 0xFF

#define MAX77779_REV_PASS_1_5 0x1
#define MAX77779_REV_PASS_2_0 0x2

#define MAX77779_REV_PASS_1_5_FIRMWARE 2
#define MAX77779_REV_PASS_2_0_FIRMWARE 3

/* vimon's memory mapped to 0x80 */
#define MAX77779_VIMON_MEM_BASE_ADDR 0x80
#define MAX77779_VIMON_PG_SIZE 256
#define MAX77779_VIMON_PG3_SIZE (MAX77779_VIMON_PG_SIZE - 32)

#define MAX77779_OFFSET_VER_MAJOR 7
#define MAX77779_OFFSET_VER_MINOR 6

#define MAX77779_FW_UPDATE_STRING_MAX 32

#define MAX77779_FW_UPDATE_RETRY_MAX 10

#define MAX77779_GET_DATA_FRAME_SIZE(filelen) \
	(filelen - MAX77779_FW_IMG_SZ_HEADER - 3*MAX77779_FW_IMG_SZ_PACKET)

#define MAX77779_ABORT_ON_ERROR(result, name, err_op) { \
	if (result) { \
		dev_err(fwu->dev, "[%s] failed: %s (%d)\n", name, err_op, result); \
		return result; \
	} \
}

#define MARK_IN_PROGRESS() do {} while (0);

#define MAX77779_FW_HIST_OFFSET_TAG 16
#define MAX77779_FW_HIST_VER_MASK   0xFFFF


enum max77779_fwupdate_fg_lock {
	FG_ST_LOCK_ALL_SECTION = 0x0e,
	FG_ST_UNLOCK_ALL_SECTION = 0x00,
};

enum max77779_fg_operation_status {
	FGST_NOT_CACHED = 0x01,
	FGST_FWUPDATE = 0x02,
	FGST_BASEFW = 0x03,
	FGST_ERR_READTAG = 0x10,
	FGST_NORMAL = 0xff,
};

enum max77779_fwupdate_intr {
	MAX77779_INTR_CLEAR = 0x00,
	MAX77779_INTR_SESSION_START = 0x70,
	MAX77779_INTR_TRANSFER_FRAMES = 0x72,
	MAX77779_INTR_APP_VALID = 0x77,
	MAX77779_INTR_SESSION_END = 0x74,
};

enum max77779_fwupdate_rsp_code {
	MAX77779_RSP_CODE_OK = 0x00,
	MAX77779_RSP_CODE_UNEXPECTED = 0xF0,
	MAX77779_RSP_CODE_CMD_SEC_FAIL = 0xF1,
	MAX77779_RSP_CODE_INVALID_PARAM = 0xF4,
	MAX77779_RSP_CODE_NOT_READY = 0xFF,
};

enum max77779_fwupdate_cmd {
	MAX77779_CMD_CLEAR_ALL = 0x00,
        MAX77779_CMD_REBOOT_FG = 0x0F,
	MAX77779_CMD_REBOOT_RISCV = 0x080F,
};

enum max77779_fwupdate_status {
	MAX77779_FWU_OK = 0x0,
	MAX77779_FWU_RUNNING_UPDATE,
	MAX77779_FWU_REG_ACCESS_ERR,
	MAX77779_FWU_UPDATE_FAIL,
	MAX77779_FWU_BOOT_ERR,
	MAX77779_FWU_TIMER_ERR,
};

enum max77779_fwupdate_err_code {
	MAX77779_FWU_ERR_NONE = 0,
	MAX77779_FWU_ERR_PREPARE,
	MAX77779_FWU_ERR_DATA_TRANSFER,
	MAX77779_FWU_ERR_POST_STATUS_CHECK,
};

struct max77779_version_info {
	u8 major;
	u8 minor;
};

/* saved as GBMS_TAG_FWSF */
struct max77779_fwupdate_stats {
	s16 count;
	s8  success;
	s8  fail;
}__attribute__((packed));

struct max77779_fwupdate_custom_data {
	ssize_t size;
	char*  data;
};

struct max77779_fwupdate_info {
	int new_tag;
	int retry_cnt;
	bool force_update;
	bool reboot_on_failure;
};

struct max77779_fwupdate {
	struct device *dev;
	struct dentry *de;

	struct delayed_work update_work;

	struct device *pmic;
	struct device *fg;
	struct device *vimon;
	struct device *chg;

	struct platform_device *batt;

	bool can_update;
	bool running_update;
	struct max77779_fwupdate_info update_info;

	struct max77779_version_info v_cur;
	struct max77779_version_info v_new;

	char fw_name[MAX77779_FW_UPDATE_STRING_MAX];

	size_t data_frame_size;
	u32 crc_val;

	u8 op_st;

	u8* scratch_buffer;
	u8* zero_filled_buffer;

	int minimum_voltage;

	struct max77779_version_info minimum;

	struct max77779_fwupdate_custom_data debug_image;

	struct wakeup_source *fwupdate_wake_lock;
	struct mutex status_lock;

	struct max77779_fwupdate_stats stats;

	struct logbuffer *lb;

	struct gvotable_election *mode_votable;
};

/* Defined at max77779_fg.c */
int max77779_external_fg_reg_write_nolock(struct device *, uint16_t, uint16_t);

static int get_firmware_update_tag(const struct max77779_fwupdate *fwu)
{
	int ret, ver_tag = 0;
	uint32_t fw_tag = 0, cur_ver;

	ret = gbms_storage_read(GBMS_TAG_FWHI, &fw_tag, sizeof(fw_tag));
	if (ret < 0)
		gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_WARNING, 0, LOGLEVEL_INFO,
				     "failed to read GBMS_TAG_FWHI (%d)\n", ret);

	cur_ver = (fwu->v_cur.major << 8) | fwu->v_cur.minor;

	if ((fw_tag & MAX77779_FW_HIST_VER_MASK) == cur_ver)
		ver_tag = (fw_tag >> MAX77779_FW_HIST_OFFSET_TAG);

	return ver_tag;
}

static void set_firmware_update_tag(const struct max77779_fwupdate *fwu, int tag)
{
	int ret;
	uint32_t fw_tag;

	fw_tag = (tag << MAX77779_FW_HIST_OFFSET_TAG);
	fw_tag |= (fwu->v_cur.major << 8) | fwu->v_cur.minor;

	ret= gbms_storage_write(GBMS_TAG_FWHI, &fw_tag, sizeof(fw_tag));
	if (ret < 0)
		gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_WARNING, 0, LOGLEVEL_INFO,
				     "failed to write GBMS_TAG_FWHI (%d)\n", ret);
}

static inline void read_fwupdate_stats(struct max77779_fwupdate *fwu)
{
	int ret;

	ret = gbms_storage_read(GBMS_TAG_FWSF, &fwu->stats, sizeof(fwu->stats));
	if (ret < 0)
		gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_WARNING, 0, LOGLEVEL_INFO,
				     "failed to read GBMS_TAG_FWSF (%d)\n", ret);

	if (ret < 0 || fwu->stats.count < 0 || fwu->stats.success < 0 || fwu->stats.fail < 0
	    || fwu->stats.count != fwu->stats.success + fwu->stats.fail) {
		/* invalid stats values. reset all counter to 0 */
		fwu->stats.count = 0;
		fwu->stats.success = 0;
		fwu->stats.fail = 0;
	}
}

static inline void update_fwupdate_stats(struct max77779_fwupdate *fwu,
					 struct max77779_fwupdate_stats* stats)
{
	int ret;

	ret = gbms_storage_write(GBMS_TAG_FWSF, stats, sizeof(*stats));
	if (ret < 0)
		gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_WARNING, 0, LOGLEVEL_INFO,
				     "failed to write GBMS_TAG_FWSF (%d)\n", ret);
}

/* TODO: b/326472325 it needs to handle counter reaches MAX77779_FW_UPDATE_RETRY_MAX */
static inline int max77779_schedule_update(struct max77779_fwupdate* fwu)
{
	int ret = -1;

	mutex_lock(&fwu->status_lock);

	if (fwu->update_info.retry_cnt < MAX77779_FW_UPDATE_RETRY_MAX) {
		fwu->update_info.retry_cnt++;
		ret = 0;
	}

	mutex_unlock(&fwu->status_lock);

	if (ret == 0) {
		dev_info(fwu->dev, "will schedule firmware update for [%s]\n", fwu->fw_name);
		schedule_delayed_work(&fwu->update_work,
				      msecs_to_jiffies(FW_UPDATE_CONDITION_CHECK_INTERVAL_MS));
	}

	return ret;
}

static int max77779_fwupdate_init(struct max77779_fwupdate *fwu)
{
	struct device* dev = fwu->dev;
	struct device_node *dn;
	int val = 0;

	if (!dev)
		return -EINVAL;

	fwu->update_info.force_update = false;
	fwu->running_update = false;
	fwu->minimum_voltage = MAX77779_FW_UPDATE_MIN_VOLTAGE;

	fwu->debug_image.data = NULL;
	fwu->debug_image.size = 0;

	mutex_init(&fwu->status_lock);

	fwu->pmic = max77779_get_dev(fwu->dev, MAX77779_PMIC_OF_NAME);
	if (!fwu->pmic) {
		dev_err(dev, "Error finding pmic\n");
		return -EPROBE_DEFER;
	}

	fwu->fg = max77779_get_dev(fwu->dev, "max77779,fg");
	if (!fwu->fg) {
		dev_err(dev, "Error finding fg\n");
		return -EPROBE_DEFER;
	}

	fwu->vimon = max77779_get_dev(fwu->dev, "max77779,vimon");
	if (!fwu->vimon) {
		dev_err(dev, "Error finding vimon\n");
		return -EPROBE_DEFER;
	}

	fwu->chg = max77779_get_dev(fwu->dev, "max77779,chg");
	if (!fwu->chg) {
		dev_err(dev, "Error finding chg\n");
		return -EPROBE_DEFER;
	}

	if (!fwu->batt) {
		dn = of_parse_phandle(dev->of_node, "google,battery", 0);
		if (!dn)
			return -ENXIO;

		fwu->batt = of_find_device_by_node(dn);
		if (!fwu->batt)
			return -EPROBE_DEFER;
	}

	val = gbms_storage_read(GBMS_TAG_FGST, &fwu->op_st, sizeof(fwu->op_st));
	if (val < 0)
		gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_WARNING, 0, LOGLEVEL_INFO,
				     "failed to read FGST tag (%d)\n", val);

	if (of_property_read_u32(dev->of_node, "fwu,enabled", &val) == 0)
		fwu->can_update = val;

	if (of_property_read_u32(dev->of_node, "minimum-voltage", &val) == 0)
		fwu->minimum_voltage = val;

	if (of_property_read_u32(dev->of_node, "version-major", &val) == 0)
		fwu->minimum.major = (u8)val;

	if (of_property_read_u32(dev->of_node, "version-minor", &val) == 0)
		fwu->minimum.minor = (u8)val;

	fwu->lb = logbuffer_register("max77779_fwupdate");
	if (IS_ERR(fwu->lb)) {
		val = PTR_ERR(fwu->lb);
		dev_err(dev, "failed to obtain logbuffer, ret=%d\n", val);
		fwu->lb = NULL;
		return val;
	}

	read_fwupdate_stats(fwu);

	return 0;
}

static int max77779_wait_cpu_reset(struct max77779_fwupdate *fwu)
{
	int ret;
	int cnt = 0;
	uint8_t val;

	dev_info(fwu->dev, "waiting for cpu reset\n");

	while (cnt < FW_UPDATE_RETRY_CPU_RESET) {
		msleep(FW_UPDATE_WAIT_INTERVAL_MS);
		ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_PMIC_RISCV_AP_DATAIN0, &val);
		if (ret == 0) {
			if (val != MAX77779_RSP_CODE_UNEXPECTED) {
				MARK_IN_PROGRESS();
			} else {
				dev_info(fwu->dev, "cpu reset completed\n");
				return 0;
			}
		}
		cnt++;
	}

	dev_err(fwu->dev, "timeout for max77779_wait_cpu_reset\n");
	return -ETIMEDOUT;
}

static int max77779_wait_fw_update(struct max77779_fwupdate *fwu)
{
	int ret;
	int cnt = 0;
	uint8_t val;

	dev_info(fwu->dev, "waiting for firmware update\n");

	do {
		msleep(FW_UPDATE_WAIT_INTERVAL_MS);
		cnt++;

		ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_PMIC_RISCV_AP_DATAIN0, &val);
		if (ret != 0 || val == MAX77779_RSP_CODE_NOT_READY) {
			MARK_IN_PROGRESS();
			continue;
		} else if (val == MAX77779_RSP_CODE_UNEXPECTED) {
			dev_err(fwu->dev, "failed to firmware update rsp %02x\n", val);
			return -EBADFD;
		}

		dev_info(fwu->dev, "firmware update completed: rsp %02x\n", val);
		return 0;

	} while (cnt < FW_UPDATE_RETRY_FW_UPDATE);

	dev_err(fwu->dev, "timeout for max77779_wait_fw_update\n");
	return -ETIMEDOUT;
}

static int max77779_wait_riscv_reboot(struct max77779_fwupdate *fwu)
{
	int ret;
	int cnt = 0;
	uint16_t val;

	dev_info(fwu->dev, "waiting for riscv reboot\n");

	while (cnt < FW_UPDATE_RETRY_RISCV_REBOOT) {
		msleep(FW_UPDATE_WAIT_INTERVAL_MS);
		ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_FG_INT_STS, &val);
		if (!ret && (val & MAX77779_FG_FG_INT_MASK_POR_m_MASK)) {
			dev_info(fwu->dev, "wait_risc_reboot POR interrupt received\n");
			return 0;
		}

		cnt++;
	}

	dev_err(fwu->dev, "timeout for POR interrupt\n");
	return -ETIMEDOUT;
}

/* b/328083603: Even POR triggered, RISC-V may not be ready */
static int check_boot_completed(struct max77779_fwupdate *fwu, int max_retry)
{
	int ret, retry;
	uint16_t val;

	for (retry = 0; retry < max_retry; retry++) {
		ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_BOOT_CHECK_REG, &val);
		if (ret) {
			dev_err(fwu->dev, "failed to read %02x (%d) in check boot completed\n",
				MAX77779_FG_BOOT_CHECK_REG, ret);
			return ret;
		}

		/* b/323382370 */
		if ((val & MAX77779_FG_BOOT_CHECK_SUCCESS) == MAX77779_FG_BOOT_CHECK_SUCCESS)
			break;

		msleep(FW_UPDATE_WAIT_INTERVAL_MS);
	}

	if (retry == max_retry) {
		dev_err(fwu->dev, "Boot NOT completed successfully: %04x\n", val);
		return -EIO;
	}

	dev_info(fwu->dev, "Boot completed successfully\n");
	return 0;
}

static int max77779_check_timer_refresh(struct max77779_fwupdate *fwu)
{
	int ret;
	uint16_t val0, val1;

	dev_info(fwu->dev, "check for timer refresh\n");

	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_Timer, &val0);
	if (ret)
		goto check_timer_error;

	msleep(FW_UPDATE_TIMER_CHECK_INTERVAL_MS);

	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_Timer, &val1);
	if (ret)
		goto check_timer_error;

	if (val1 <= val0) {
		dev_err(fwu->dev, "Timer NOT updating correctly\n");
		return -EIO;
	}

	dev_info(fwu->dev, "Timer updating correctly\n");
	return 0;

check_timer_error:
	dev_err(fwu->dev, "failed to read %02x (%d) in max77779_check_timer_refresh\n",
		MAX77779_FG_Timer, ret);
	return ret;
}

static int max77779_send_command(struct max77779_fwupdate *fwu,
				 enum max77779_fwupdate_cmd cmd)
{
	int ret;

	ret = max77779_external_fg_reg_write_nolock(fwu->fg, MAX77779_FG_Command_fw, cmd);
	if (ret)
		dev_err(fwu->dev, "failed to write fg reg %02x (%d) in max77779_send_command\n",
			MAX77779_FG_Command_fw, ret);

	return ret;
}

static int max77779_trigger_interrupt(struct max77779_fwupdate *fwu,
				      enum max77779_fwupdate_intr intr)
{
	int ret;

	ret = max77779_external_pmic_reg_write(fwu->pmic, MAX77779_PMIC_RISCV_AP_DATAOUT_OPCODE,
					       intr);
	if (ret)
		dev_err(fwu->dev, "failed to write pmic reg %02x (%d) in trigger_interrupt\n",
			MAX77779_PMIC_RISCV_AP_DATAOUT_OPCODE, ret);

	return ret;
}

static int max77779_get_firmware_version(struct max77779_fwupdate *fwu,
					 struct max77779_version_info *ver)
{
	int ret;
	uint8_t major, minor;

	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_PMIC_RISCV_FW_REV, &major);
	if (ret) {
		dev_err(fwu->dev, "failed to read pmic reg %02x (%d) in read firmware version\n",
			MAX77779_PMIC_RISCV_FW_REV, ret);
		goto max77779_get_firmware_version_done;
	}

	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_PMIC_RISCV_FW_SUB_REV, &minor);
	if (ret)
		dev_err(fwu->dev, "failed to read pmic reg %02x (%d) in read firmware version\n",
			MAX77779_PMIC_RISCV_FW_SUB_REV, ret);

	ver->major = major;
	ver->minor = minor;

max77779_get_firmware_version_done:
	return ret;
}


static int max77779_change_fg_lock(struct max77779_fwupdate *fwu,
				   const enum max77779_fwupdate_fg_lock st)
{
	int ret;

	/*
	 * change FG's lock status
	 * - write status_value 2 times to MAX77779_FG_USR
	 */
	ret = max77779_external_fg_reg_write_nolock(fwu->fg, MAX77779_FG_USR, st);
	if (ret == 0)
		ret = max77779_external_fg_reg_write_nolock(fwu->fg, MAX77779_FG_USR, st);

	if (ret)
		dev_err(fwu->dev, "failed to write fg reg %02x (%d) in change lock status\n",
			MAX77779_FG_USR, ret);

	return ret;
}

static int max77779_copy_to_vimon_mem(struct max77779_fwupdate *fwu,
				      const u16 page, const u8* data,
				      const size_t data_len)
{
	int ret;

	ret = max77779_external_vimon_reg_write(fwu->vimon, MAX77779_BVIM_PAGE_CTRL,
						(u8*)&page, 2);
	if (ret) {
		dev_err(fwu->dev, "failed to set page %x (%d)\n", page, ret);
		goto max77779_copy_to_vimon_mem_done;
	}

	ret = max77779_external_vimon_reg_write(fwu->vimon, MAX77779_VIMON_MEM_BASE_ADDR,
						data, data_len);
	if (ret) {
		dev_err(fwu->dev,
			"failed to write data to vimon's memory page %x (%d)\n",
			page, ret);
		goto max77779_copy_to_vimon_mem_done;
	}

max77779_copy_to_vimon_mem_done:
	return ret;
}

static int max77779_load_fw_binary(struct max77779_fwupdate *fwu,
				   const u8* data, const size_t data_len)
{
	u16 page;
	int ret;
	size_t cp_len;
	ssize_t remains = (ssize_t)data_len;

	/* copy firmware binary to vimon's memory */
	for (page = 0; (page < 4) && remains > 0; page++) {
		if (remains > MAX77779_VIMON_PG_SIZE)
			cp_len = MAX77779_VIMON_PG_SIZE;
		else
			cp_len = remains;

		ret = max77779_copy_to_vimon_mem(fwu, page, data, cp_len);
		if (ret) {
			dev_err(fwu->dev, "failed load binary in copy data in page %d\n",
				(int)page);
			goto max77779_load_fw_binary_done;
		}

		data += MAX77779_VIMON_PG_SIZE;
		remains -= MAX77779_VIMON_PG_SIZE;
	}

max77779_load_fw_binary_done:
	return ret;
}

static inline int max77779_clear_state_for_update(struct max77779_fwupdate *fwu)
{
	int ret;
	uint16_t val;

	/* clear POR bits*/
	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_FG_INT_STS, &val);
	if (ret) {
		dev_err(fwu->dev,
			"failed to read reg %02x (%d) in max77779_clear_state_for_update\n",
			MAX77779_FG_FG_INT_STS, ret);
		return ret;
	}

	ret = max77779_external_fg_reg_write_nolock(fwu->fg, MAX77779_FG_FG_INT_STS, val);
	if (ret) {
		dev_err(fwu->dev,
			"failed to write reg %02x (%d) in max77779_clear_state_for_update\n",
			MAX77779_FG_FG_INT_STS, ret);
		return ret;
	}

	/* clear commands */
	max77779_send_command(fwu, MAX77779_CMD_CLEAR_ALL);

	/* corner case, handles commands still present in AP_REQUEST_OPCODE */
	ret = max77779_trigger_interrupt(fwu, MAX77779_INTR_CLEAR);
	return ret;
}

static inline int max77779_session_start(struct max77779_fwupdate *fwu,
					 const u8* fw_binary_data,
					 const char* name)
{
	int ret;

	dev_info(fwu->dev, "[%s] begins\n", name);

	ret = max77779_trigger_interrupt(fwu, MAX77779_INTR_SESSION_START);
	MAX77779_ABORT_ON_ERROR(ret, name, "interrupt trigger");

	max77779_wait_cpu_reset(fwu);

	ret = max77779_load_fw_binary(fwu, fw_binary_data, MAX77779_FW_IMG_SZ_PACKET);
	MAX77779_ABORT_ON_ERROR(ret, name, "load_binary");

	ret = max77779_wait_fw_update(fwu);
	MAX77779_ABORT_ON_ERROR(ret, name, "max77779_wait_fw_update");

	dev_info(fwu->dev, "[%s] ends\n", name);

	return ret;
}

static int max77779_transfer_binary_data(struct max77779_fwupdate *fwu,
					 const u8* fw_binary_data,
					 const size_t data_size,
					 enum max77779_fwupdate_intr intr,
					 const char* name)
{
	int ret;
	ssize_t frame_len;
	ssize_t remains = (ssize_t)data_size;

	dev_info(fwu->dev, "[%s] begins\n", name);

	while (remains > 0) {
		frame_len = (remains > MAX77779_FW_IMG_SZ_FRAME)?
			    MAX77779_FW_IMG_SZ_FRAME : remains;

		ret = max77779_load_fw_binary(fwu, fw_binary_data, frame_len);
		MAX77779_ABORT_ON_ERROR(ret, name, "load_binary");

		msleep(FW_UPDATE_WAIT_LOAD_BIN_MS);

		ret = max77779_trigger_interrupt(fwu, intr);
		MAX77779_ABORT_ON_ERROR(ret, name, "max77779_trigger_interrupt");

		ret = max77779_wait_fw_update(fwu);
		MAX77779_ABORT_ON_ERROR(ret, name, "max77779_wait_fw_update");

		fw_binary_data += frame_len;
		remains -= frame_len;

		dev_info(fwu->dev, "transferred data (%zu/%zu)\n", (data_size - remains),
			 data_size);
	}

	dev_info(fwu->dev, "[%s] ends\n", name);

	return ret;
}

/* TODO: b/303731272 condtion check */
static int max77779_can_update(struct max77779_fwupdate *fwu, struct max77779_version_info* target)
{
	/* compatibility check: major version should match */
	if (target->major != fwu->v_cur.major)
		return -EINVAL;

	/* Is this device ellgabie to update firmware? */
	if (!fwu->can_update)
		return -EACCES;

	/* check version */
	if (target->minor <= fwu->v_cur.minor && !fwu->update_info.force_update)
		return -EINVAL;

	return 0;
}

static inline int max77779_set_firmwarename(struct max77779_fwupdate *fwu)
{
	uint8_t val;
	int ret, fw_ver;

	fw_ver = fwu->v_cur.major;

	/* b/322967969 version value can be 0xFF */
	if (fw_ver == MAX77779_FW_INVALID_FW_VER) {
		ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_PMIC_REVISION, &val);
		if (ret) {
			dev_err(fwu->dev, "faild to read pmic reg %02x (%d)\n",
				MAX77779_PMIC_REVISION, ret);
			return ret;
		}

		if (val == MAX77779_REV_PASS_1_5)
			fw_ver = MAX77779_REV_PASS_1_5_FIRMWARE;
		else if (val == MAX77779_REV_PASS_2_0)
			fw_ver = MAX77779_REV_PASS_2_0_FIRMWARE;
		else
			return -EINVAL;
	}

	scnprintf(fwu->fw_name, MAX77779_FW_UPDATE_STRING_MAX, "%s_%d.bin",
		  MAX77779_FIRMWARE_BINARY_PREFIX, fw_ver);

	return 0;
}

static inline int max77779_fwupdate_chip_reset(struct max77779_fwupdate *fwu)
{
	int ret;

	/* non zero opcode may distrub chip reset */
	ret = max77779_external_pmic_reg_write(fwu->pmic, MAX77779_PMIC_RISCV_AP_DATAOUT_OPCODE,
					       0x0);
	if (ret)
		dev_err(fwu->dev, "failed to clear opcode (%d)\n", ret);

	ret = max77779_external_pmic_reg_write(fwu->pmic, MAX77779_PMIC_RISCV_COMMAND_HW,
					       MAX77779_CMD_REBOOT_FG);
	if (ret)
		dev_err(fwu->dev, "failed to reset chip (%d)\n", ret);

	return ret;
}

static int max77779_fwl_prepare(struct max77779_fwupdate *fwu,
				const u8 *data, u32 size)
{
	int ret;
	size_t data_frame_size;
	struct gvotable_election *mode_votable;

	fwu->zero_filled_buffer = kzalloc(MAX77779_VIMON_PG_SIZE, GFP_KERNEL);
	fwu->scratch_buffer = kmalloc(MAX77779_VIMON_PG_SIZE, GFP_KERNEL);
	if (!fwu->zero_filled_buffer || !fwu->scratch_buffer) {
		dev_err(fwu->dev, "failed to allocate temporay work buffer\n");
		return -ENOMEM;
	}

	dev_info(fwu->dev, "prepare firmware update (image size: %d)\n", size);

	data_frame_size = MAX77779_GET_DATA_FRAME_SIZE(size);
	if (data_frame_size % MAX77779_FW_IMG_SZ_PACKET) {
		dev_err(fwu->dev, "incorrect image size (data section size: %zu)\n",
			data_frame_size);
		return -EINVAL;
	}

	ret = max77779_get_firmware_version(fwu, &fwu->v_cur);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to read version information\n");

	fwu->data_frame_size = data_frame_size;
	fwu->op_st = (u8)FGST_FWUPDATE;

	ret = gbms_storage_write(GBMS_TAG_FGST, &fwu->op_st, sizeof(fwu->op_st));
	if (ret != sizeof(fwu->op_st)) {
		fwu->op_st = (u8)FGST_ERR_READTAG;
		gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_WARNING, 0, LOGLEVEL_INFO,
				     "failed to update eeprom:GBMS_TAG_FGST (%d)\n", ret);
	}

	if (!fwu->mode_votable) {
		mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (!mode_votable) {
			dev_err(fwu->dev, "failed to get %s(%ld)\n", GBMS_MODE_VOTABLE,
				PTR_ERR(mode_votable));
			return -ENODEV;
		}

		fwu->mode_votable = mode_votable;
	}

	ret = gvotable_cast_long_vote(fwu->mode_votable, MAX77779_REASON_FIRMWARE,
				      GBMS_CHGR_MODE_FWUPDATE_BOOST_ON, true);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to set mode BOOST_ON");

	ret = max77779_fg_enable_firmware_update(fwu->fg, true);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to set fg_enable_firmware_update");

	dev_info(fwu->dev, "the current installed firmware version %u.%u\n",
		 (unsigned int)fwu->v_cur.major,
		 (unsigned int)fwu->v_cur.minor);

	ret = max77779_change_fg_lock(fwu, FG_ST_UNLOCK_ALL_SECTION);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed unlock FG");

	ret = max77779_clear_state_for_update(fwu);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed clear command / POR  interrupt");

	ret = max77779_send_command(fwu, MAX77779_CMD_REBOOT_RISCV);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed send command CMD_REBOOT_RISCV");

	/* wait_riscv_reboot might timeout but subsequent updates will be ok */
	max77779_wait_riscv_reboot(fwu);

	return ret;
}

/* TODO: b/303132973 - consider: "offset" */
static int max77779_fwl_write(struct max77779_fwupdate *fwu, const u8 *fw_binary_data, u32 offset,
			      u32 size, u32 *written)
{
	int ret;
	uint8_t val;

	dev_info(fwu->dev, "perform firmware update\n");

	/* skip header*/
	fw_binary_data += MAX77779_FW_IMG_SZ_HEADER;
	*written += MAX77779_FW_IMG_SZ_HEADER;

	/* Session Start */
	ret = max77779_session_start(fwu, fw_binary_data, "Session Start");
	MAX77779_ABORT_ON_ERROR(ret, __func__, "Session Start");

	fw_binary_data += MAX77779_FW_IMG_SZ_PACKET;
	*written += MAX77779_FW_IMG_SZ_PACKET;

	/* Transfer Frame */
	ret = max77779_transfer_binary_data(fwu, fw_binary_data, fwu->data_frame_size,
					    MAX77779_INTR_TRANSFER_FRAMES, "Transfer Frame");
	MAX77779_ABORT_ON_ERROR(ret, __func__, "Transfer Frame");

	fw_binary_data += fwu->data_frame_size;
	*written += fwu->data_frame_size;

	/* App Valid: CRC check */
	ret = max77779_transfer_binary_data(fwu, fw_binary_data, MAX77779_FW_IMG_SZ_PACKET,
					    MAX77779_INTR_APP_VALID, "App Valid");
	MAX77779_ABORT_ON_ERROR(ret, __func__, "App Valid");

	fw_binary_data += MAX77779_FW_IMG_SZ_PACKET;
	*written += MAX77779_FW_IMG_SZ_PACKET;

	fwu->crc_val = 0;
	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_PMIC_RISCV_AP_DATAIN0, &val);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to read crc information");
	dev_info(fwu->dev, "RISCV lock status: %x\n", val);

	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_PMIC_RISCV_AP_DATAIN2, &val);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to read crc information");
	fwu->crc_val = val;

	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_PMIC_RISCV_AP_DATAIN3, &val);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to read crc information");
	fwu->crc_val |= (val << 8);


	/* Session End */
	ret = max77779_transfer_binary_data(fwu, fw_binary_data, MAX77779_FW_IMG_SZ_PACKET,
					    MAX77779_INTR_SESSION_END, "Session End");
	MAX77779_ABORT_ON_ERROR(ret, __func__, "Session End");

	*written += MAX77779_FW_IMG_SZ_PACKET;

	return ret;
}

static int max77779_fwl_poll_complete(struct max77779_fwupdate *fwu)
{
	int ret;
	uint16_t val;

	dev_info(fwu->dev, "max77779_fwl_poll_complete\n");

	/* check firmware update status */
	dev_info(fwu->dev, "firmware update CRC: %x\n", fwu->crc_val);
	if (fwu->crc_val == 0) {
		dev_info(fwu->dev, "bad CRC value returns");
		return -EIO;
	}

	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_SECUPDATE_STATUS_REG, &val);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to read MAX77779_FG_SECUPDATE_STATUS_REG");
	if (val != MAX77779_FG_SECUPDATE_STATUS_SUCCESS) {
		dev_err(fwu->dev, "firmware update fail: MAX77779_FG_SECUPDATE_STATUS_REG:%02x\n",
			val);
		return -EAGAIN;
	}

	/* b/310710147: risc-v is not operational state. requires reboot */
	max77779_fwupdate_chip_reset(fwu);
	max77779_wait_riscv_reboot(fwu);

	ret = check_boot_completed(fwu, FW_UPDATE_RETRY_CPU_RESET);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed on check_boot_completed\n");

	ret = max77779_get_firmware_version(fwu, &fwu->v_new);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to get firmware version\n");
	dev_info(fwu->dev, "updated firmware version: %u.%u\n",
		 fwu->v_new.major, fwu->v_new.minor);

	ret = max77779_check_timer_refresh(fwu);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed on max77779_check_timer_refresh\n");

	mutex_lock(&fwu->status_lock);

	fwu->op_st = FGST_NORMAL;
	fwu->stats.count++;
	fwu->stats.success++;

	update_fwupdate_stats(fwu, &fwu->stats);

	mutex_unlock(&fwu->status_lock);

	if (fwu->v_cur.major != fwu->v_new.major || fwu->v_cur.minor != fwu->v_new.minor)
		fwu->v_cur = fwu->v_new;

	set_firmware_update_tag(fwu, fwu->update_info.new_tag);

	fwu->update_info.force_update = false;
	fwu->update_info.retry_cnt = 0;

	return ret;
}

static void max77779_fwl_cleanup(struct max77779_fwupdate *fwu)
{
	int ret;

	dev_info(fwu->dev, "max77779_fwl_cleanup\n");

	if (fwu->zero_filled_buffer)
		kfree(fwu->zero_filled_buffer);
	if (fwu->scratch_buffer)
		kfree(fwu->scratch_buffer);

	ret = max77779_fg_enable_firmware_update(fwu->fg, false);
	if (ret)
		dev_err(fwu->dev, "failed to restore FG from update mode (%d)\n", ret);

	if (!fwu->mode_votable)
		return;

	ret = gvotable_cast_long_vote(fwu->mode_votable, MAX77779_REASON_FIRMWARE,
				      GBMS_CHGR_MODE_FWUPDATE_BOOST_ON, false);
	if (ret)
		dev_err(fwu->dev, "failed to restore CHG from update mode (%d)\n", ret);
}

static inline int update_running_state(struct max77779_fwupdate *fwu, bool running)
{
	bool ret = false;

	mutex_lock(&fwu->status_lock);

	if (fwu->running_update != running) {
		fwu->running_update = running;
		ret = true;
	}

	mutex_unlock(&fwu->status_lock);

	return ret;
}

static inline int perform_firmware_update(struct max77779_fwupdate *fwu, const char* data,
					  const size_t count)
{
	u32 written = 0;
	enum gbms_fwupdate_max77779_err_code err_code = FWU_MAX77779_ERR_NONE;
	int ret, ret_st;
	struct max77779_fwupdate_stats stats_backup;

	/* if previous update is not completed yet, stop at here */
	if (!update_running_state(fwu, true))
		return -EBUSY;

	__pm_stay_awake(fwu->fwupdate_wake_lock);

	logbuffer_log(fwu->lb, "perform_firmware_update: %s", fwu->fw_name);

	/*
	 * increase failure count upfront
	 *  - update can be distrubed without cleanup
	 *  - store with new value inside of max77779_fwl_poll_complete when everything is OK
	 */
	mutex_lock(&fwu->status_lock);

	stats_backup.count = fwu->stats.count + 1;
	stats_backup.success = fwu->stats.success;
	stats_backup.fail = fwu->stats.fail + 1;

	update_fwupdate_stats(fwu, &stats_backup);

	mutex_unlock(&fwu->status_lock);

	ret = max77779_fwl_prepare(fwu, data, count);
	if (ret) {
		err_code = FWU_MAX77779_ERR_PREPARE;
		goto perform_firmware_update_cleanup;
	}

	ret = max77779_fwl_write(fwu, data, 0, count, &written);
	if (ret || written != count) {
		err_code = FWU_MAX77779_ERR_DATA_TRANSFER;
		goto perform_firmware_update_cleanup;
	}

	if (max77779_fwl_poll_complete(fwu) != 0)
		err_code = FWU_MAX77779_ERR_POST_STATUS_CHECK;

perform_firmware_update_cleanup:
	max77779_fwl_cleanup(fwu);

	/* force reboot RISC-V for the case of update failure*/
	if (ret || written != count) {
		max77779_fwupdate_chip_reset(fwu);

		mutex_lock(&fwu->status_lock);

		fwu->op_st = FGST_BASEFW;
		fwu->stats.count++;
		fwu->stats.fail++;

		mutex_unlock(&fwu->status_lock);
	}

	ret_st = gbms_storage_write(GBMS_TAG_FGST, &fwu->op_st, sizeof(fwu->op_st));
	if (ret_st != sizeof(fwu->op_st))
		gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_WARNING, 0, LOGLEVEL_INFO,
				     "failed to update eeprom:GBMS_TAG_FGST (%d)\n", ret_st);

	gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			     "complete_firmware_update: %d %d %d (%d)", fwu->stats.count,
			     fwu->stats.success, fwu->stats.fail, (int)err_code);

	gbms_logbuffer_prlog(fwu->lb, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			     "0x%04X %X %X %X %X %X %X %X %X %llX %X %X",
			     MONITOR_TAG_FU, FWU_MSG_TYPE_UPDATE_END, FWU_MSG_CATEGORY_MAX77779,
			     fwu->v_cur.major, fwu->v_cur.minor,
			     data[MAX77779_OFFSET_VER_MAJOR], data[MAX77779_OFFSET_VER_MINOR],
			     err_code, fwu->stats.count, ktime_get_real_seconds(),
			     fwu->stats.success, fwu->stats.fail);

	__pm_relax(fwu->fwupdate_wake_lock);
	update_running_state(fwu, false);

	kobject_uevent(&fwu->fg->kobj, KOBJ_CHANGE);

	return ret;
}

static void firmware_update_work(struct work_struct* work)
{
	int ret;
	struct max77779_version_info target_version;
	const struct firmware* fw_data;
	struct max77779_fwupdate *fwu = NULL;

	fwu = container_of(work, struct max77779_fwupdate, update_work.work);
	if (!fwu)
		return;

	ret = request_firmware(&fw_data, fwu->fw_name, fwu->dev);
	if (ret) {
		dev_warn(fwu->dev, "fails on request_firmware %d\n", ret);
		goto firmware_update_work_cleanup;
	}

	target_version.major = fw_data->data[MAX77779_OFFSET_VER_MAJOR];
	target_version.minor = fw_data->data[MAX77779_OFFSET_VER_MINOR];

	ret = max77779_can_update(fwu, &target_version);
	if (ret) {
		dev_info(fwu->dev, "can not update firmware %d\n", ret);
		goto firmware_update_work_cleanup;
	}

	ret = perform_firmware_update(fwu, fw_data->data, fw_data->size);
	if (ret)
		dev_err(fwu->dev, "firmware update failed (retry:%d) %d\n",
			fwu->update_info.retry_cnt, ret);

firmware_update_work_cleanup:
	release_firmware(fw_data);

	if (ret)
		max77779_schedule_update(fwu);
}

static inline bool max77779_can_charge(struct device* chg)
{
	struct max77779_chgr_data *data = dev_get_drvdata(chg);
	int ret;
	uint8_t chg_detail;

	ret = max77779_external_chg_reg_read(chg, MAX77779_CHG_DETAILS_00, &chg_detail);
	if (ret)
		return false;

	/* check usb: 0x0 or 0x1 means VBUS is invalid */
	if (_max77779_chg_details_00_chgin_dtls_get(chg_detail) >= 2 && !data->chgin_input_suspend)
		return true;

	/* check wireless: 0x0 or 0x1 means VWCIN is invalid */
	if ((_max77779_chg_details_00_wcin_dtls_get(chg_detail) >= 2 && !data->wcin_input_suspend)
	    || data->wlc_spoof)
		return true;

	return false;
}

/*
 * trigger firmware update with override version tag
 *  - echo xxx > update_firmware
 */
static ssize_t trigger_update_firmware(struct device *dev,
				       struct device_attribute *attr,
				       const char *options, size_t count)
{
	int ret, current_ver, read_cnt, target_ver;
	int bypass_check = 0, override_ver = 0;
	uint16_t voltage;
	struct max77779_fwupdate *fwu = dev_get_drvdata(dev);

	if (!fwu)
		return -EAGAIN;

	if (!fwu->can_update) {
		dev_err(fwu->dev, "not allowed to update firmware\n");
		return -EACCES;
	}

	read_cnt = sscanf(options, "%d %d %d", &target_ver, &override_ver, &bypass_check);
	if (read_cnt < 1) {
		dev_err(fwu->dev, "incorrect input: expects override_tag(number) and reset_tag"
			"(optional)\n");
		return -EINVAL;
	}

	if (!bypass_check) {
		/* check chgin/wcin */
		if (!max77779_can_charge(fwu->chg)) {
			dev_err(fwu->dev, "charger is not plugged. connect charger required\n");
			return -EBUSY;
		}

		/* check current voltage */
		ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_AvgVCell, &voltage);
		if (ret || reg_to_micro_volt(voltage) < fwu->minimum_voltage) {
			dev_err(fwu->dev, "low voltage for update\n");
			return -ERANGE;
		}
	}

	current_ver = get_firmware_update_tag(fwu);
	if (!override_ver && (target_ver <= current_ver)) {
		dev_info(fwu->dev, "ver %d already installed: update request will be skipped",
			target_ver);
		return count;
	}

	if (max77779_set_firmwarename(fwu) < 0) {
		dev_err(fwu->dev, "can't set proper firmware file\n");
		return -EINVAL;
	}

	mutex_lock(&fwu->status_lock);

	fwu->update_info.new_tag = target_ver;
	fwu->update_info.force_update = true;
	fwu->update_info.reboot_on_failure = true;
	fwu->update_info.retry_cnt = 0;

	mutex_unlock(&fwu->status_lock);

	schedule_delayed_work(&fwu->update_work,
			      msecs_to_jiffies(FW_UPDATE_TIMER_CHECK_INTERVAL_MS));

	return count;
}

static DEVICE_ATTR(update_firmware, 0220, NULL, trigger_update_firmware);


static ssize_t enable_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77779_fwupdate *fwu = dev_get_drvdata(dev);
	if (!fwu)
		return -EAGAIN;

	return scnprintf(buf, PAGE_SIZE, "%d\n", fwu->can_update);
}

static ssize_t enable_update_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct max77779_fwupdate *fwu = dev_get_drvdata(dev);
	if (!fwu)
		return -EAGAIN;

	if (kstrtobool(buf, &fwu->can_update))
		return -EINVAL;

	return count;
}

static DEVICE_ATTR_RW(enable_update);

static ssize_t update_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	uint16_t val;
	enum max77779_fwupdate_status st = MAX77779_FWU_OK;
	struct max77779_fwupdate *fwu = dev_get_drvdata(dev);

	if (!fwu)
		return -EAGAIN;

	mutex_lock(&fwu->status_lock);
	if (fwu->running_update) {
		st = MAX77779_FWU_RUNNING_UPDATE;
		goto update_status_show_exit;
	}

	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_SECUPDATE_STATUS_REG, &val);
	if (ret < 0){
		st = MAX77779_FWU_REG_ACCESS_ERR;
		goto update_status_show_exit;
	}

	if (val != MAX77779_FG_SECUPDATE_STATUS_SUCCESS) {
		dev_err(fwu->dev, "firmware update fail: %X:%02x\n",
			MAX77779_FG_SECUPDATE_STATUS_REG, val);
		st = MAX77779_FWU_UPDATE_FAIL;
		goto update_status_show_exit;
	}

	ret = check_boot_completed(fwu, FW_UPDATE_RETRY_ONCE);
	if (ret < 0) {
		st = MAX77779_FWU_BOOT_ERR;
		goto update_status_show_exit;
	}

	ret = max77779_check_timer_refresh(fwu);
	if (ret < 0)
		st = MAX77779_FWU_TIMER_ERR;

update_status_show_exit:
	mutex_unlock(&fwu->status_lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", st);
}

static const DEVICE_ATTR_RO(update_status);

static ssize_t chip_reset_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	bool trigger;
	int rt = -EBUSY;
	struct max77779_fwupdate *fwu = dev_get_drvdata(dev);

	if (!fwu)
		return -EAGAIN;

	if (kstrtobool(buf, &trigger) && !trigger)
		return -EINVAL;

	mutex_lock(&fwu->status_lock);
	/* if there is no on-going fwupdate, trigger reset */
	if (!fwu->running_update && max77779_fwupdate_chip_reset(fwu) == 0)
		rt = count;

	mutex_unlock(&fwu->status_lock);

	return rt;
}

static DEVICE_ATTR_WO(chip_reset);

static ssize_t update_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct max77779_fwupdate *fwu = dev_get_drvdata(dev);
	if (!fwu)
		return -EAGAIN;

	mutex_lock(&fwu->status_lock);

	ret = scnprintf(buf, PAGE_SIZE, "%d %d %d\n", fwu->stats.count, fwu->stats.success,
			fwu->stats.fail);

	mutex_unlock(&fwu->status_lock);

	return ret;
}

static ssize_t update_stats_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct max77779_fwupdate *fwu = dev_get_drvdata(dev);
	int value, ret;

	if (!fwu)
		return -EAGAIN;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&fwu->status_lock);

	if (value == 0 && !fwu->running_update) {
		fwu->stats.count = 0;
		fwu->stats.success = 0;
		fwu->stats.fail = 0;

		update_fwupdate_stats(fwu, &fwu->stats);

		ret = count;
	} else {
		ret = -EBUSY;
	}

	mutex_unlock(&fwu->status_lock);

	return ret;
}

static DEVICE_ATTR_RW(update_stats);

/*
 * Using the same pattern as FW_LOADER
 *  echo 1 > loading
 *  cat FW_IMG > data
 *  echo 0 > loading
 */
static int debug_update_firmware_loading(void *data, u64 val)
{
	struct max77779_fwupdate *fwu = data;
	int ret = 0;

	if (!fwu)
		return -EAGAIN;

	if (val) {
		if (!fwu->debug_image.data)
			fwu->debug_image.data = kzalloc(FW_UPDATE_MAXIMUM_PAGE_SIZE, GFP_KERNEL);

		if (!fwu->debug_image.data)
			return -ENOMEM;

		fwu->debug_image.size = 0;
	} else {
		if (fwu->debug_image.size > 0)
			ret = perform_firmware_update(fwu, fwu->debug_image.data, fwu->debug_image.size);

		if (fwu->debug_image.data) {
			kfree(fwu->debug_image.data);
			fwu->debug_image.data = NULL;
			fwu->debug_image.size = 0;
		}
	}

	fwu->can_update = (val != 0);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_update_firmware_loading_fops, NULL, debug_update_firmware_loading,
			"%llu\n");


static ssize_t debug_update_firmware_data(struct file *filp, const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct max77779_fwupdate *fwu;
	ssize_t ret;

	fwu = (struct max77779_fwupdate *)filp->private_data;
	if (!fwu)
		return -EAGAIN;

	if (!fwu->debug_image.data)
		return -EINVAL;

	if ((FW_UPDATE_MAXIMUM_PAGE_SIZE - fwu->debug_image.size) < count)
		return -EFBIG;

	ret = simple_write_to_buffer(fwu->debug_image.data, FW_UPDATE_MAXIMUM_PAGE_SIZE, ppos,
				     user_buf, count);

	if (ret >= 0)
		fwu->debug_image.size += ret;
	else
		fwu->debug_image.size = -EINVAL;

	return ret;
}

BATTERY_DEBUG_ATTRIBUTE(debug_update_firmware_data_fops, NULL, debug_update_firmware_data);


static int max77779_fwupdate_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct dentry *de;
	struct max77779_fwupdate *fwu;

	fwu = devm_kzalloc(&pdev->dev, sizeof(*fwu), GFP_KERNEL);
	if (!fwu)
		return -ENOMEM;

	fwu->dev = &pdev->dev;
	platform_set_drvdata(pdev, fwu);

	ret = max77779_fwupdate_init(fwu);
	if (ret) {
		dev_err(fwu->dev, "error to set max77779_fwupdate\n");
		return ret;
	}

	ret = max77779_get_firmware_version(fwu, &fwu->v_cur);
	if (ret)
		dev_err(fwu->dev, "failed to read version information\n");

	ret = max77779_set_firmwarename(fwu);
	if (ret)
		dev_err(fwu->dev, "failed to set proper firmware file\n");

	INIT_DELAYED_WORK(&fwu->update_work, firmware_update_work);

	ret = device_create_file(fwu->dev, &dev_attr_update_firmware);
	if (ret != 0) {
		pr_err("Failed to create update_firmware files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(fwu->dev, &dev_attr_enable_update);
	if (ret != 0) {
		pr_err("Failed to create enable_update files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(fwu->dev, &dev_attr_update_status);
	if (ret != 0) {
		pr_err("Failed to create update_status files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(fwu->dev, &dev_attr_chip_reset);
	if (ret != 0) {
		pr_err("Failed to create chip_reset files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(fwu->dev, &dev_attr_update_stats);
	if (ret != 0) {
		pr_err("Failed to create update_stats files, ret=%d\n", ret);
		return ret;
	}

	fwu->fwupdate_wake_lock = wakeup_source_register(NULL, "max77779-fwupdate");
	if (!fwu->fwupdate_wake_lock) {
		dev_err(fwu->dev, "failed to register wakeup source\n");
		return -ENODEV;
	}

	de = debugfs_create_dir("max77779_fwupdate", 0);
	if (!de)
		return 0;

	debugfs_create_file("loading", 0400, de, fwu, &debug_update_firmware_loading_fops);
	debugfs_create_file("data", 0444, de, fwu, &debug_update_firmware_data_fops);

	fwu->de = de;

	/* the chip is running with base firmware: need to be updated */
	if (fwu->op_st == FGST_BASEFW) {
		fwu->update_info.retry_cnt = 0;
		fwu->update_info.force_update = true;
		fwu->update_info.reboot_on_failure = false;
		max77779_schedule_update(fwu);
	}

	return ret;
}

static int max77779_fwupdate_remove(struct platform_device *pdev)
{
	struct max77779_fwupdate *fwu = platform_get_drvdata(pdev);
	if (!fwu)
		return 0;

	if (fwu->lb) {
		logbuffer_unregister(fwu->lb);
		fwu->lb = NULL;
	}

	mutex_destroy(&fwu->status_lock);

	if (fwu->debug_image.data)
		kfree(fwu->debug_image.data);

	if (fwu->fwupdate_wake_lock)
		wakeup_source_unregister(fwu->fwupdate_wake_lock);

	if (fwu->de)
		debugfs_remove(fwu->de);

	return 0;
}

static const struct of_device_id max77779_fwupdate_of_match[] = {
	{.compatible = "maxim,max77779fwu"},
	{},
};
MODULE_DEVICE_TABLE(of, max77779_fwupdate_of_match);


static const struct platform_device_id max77779_fwupdate_id[] = {
	{"max77779_fwupdate", 0},
	{}
};
MODULE_DEVICE_TABLE(platform, max77779_fwupdate_id);

static struct platform_driver max77779_fwupdate_driver = {
	.driver = {
		.name = "max77779_fwupdate",
		.owner = THIS_MODULE,
		.of_match_table = max77779_fwupdate_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		},
	.id_table = max77779_fwupdate_id,
	.probe = max77779_fwupdate_probe,
	.remove = max77779_fwupdate_remove,
};

module_platform_driver(max77779_fwupdate_driver);

MODULE_DESCRIPTION("MAX77779 Firmware Update Driver");
MODULE_AUTHOR("Chungro Lee <chungro@google.com>");
MODULE_LICENSE("GPL");
