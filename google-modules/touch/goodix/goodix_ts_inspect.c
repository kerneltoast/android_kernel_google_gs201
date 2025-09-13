/*
 * Goodix Touchscreen Driver
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include "goodix_ts_core.h"
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/version.h>

/* test config */
#define TOTAL_FRAME_NUM 1      /* rawdata test frames */
#define NOISEDATA_TEST_TIMES 1 /* noise test frames */

#define GOODIX_RESULT_SAVE_PATH "/vendor/etc/Test_Data.csv"
#define GOODIX_TEST_FILE_NAME "goodix"
#define MAX_DATA_BUFFER 28000
#define MAX_SHORT_NUM 15
#define MAX_LINE_LEN (1024 * 3 * 7)
#define MAX_DRV_NUM 52
#define MAX_SEN_NUM 75

#define STATISTICS_DATA_LEN 32
#define MAX_STR_LEN 32
#define MAX_TEST_ITEMS 10 /* 0P-1P-2P-3P-5P total test items */
#define GTP_CAP_TEST 1
#define GTP_DELTA_TEST 2
#define GTP_NOISE_TEST 3
#define GTP_OPEN_TEST 4
#define GTP_SHORT_TEST 5
#define GTP_SELFCAP_TEST 6
#define GTP_SELFNOISE_TEST 7

#define GTP_TEST_PASS 1
#define GTP_PANEL_REASON 2
#define SYS_SOFTWARE_REASON 3

#define CHN_VDD 0xFF
#define CHN_GND 0x7F
#define DRV_CHANNEL_FLAG 0x80

#define CSV_TP_SPECIAL_RAW_MIN "special_raw_min"
#define CSV_TP_SPECIAL_RAW_MAX "special_raw_max"
#define CSV_TP_SPECIAL_RAW_DELTA "special_raw_delta"
#define CSV_TP_SHORT_THRESHOLD "shortciurt_threshold"
#define CSV_TP_SPECIAL_SELFRAW_MAX "special_selfraw_max"
#define CSV_TP_SPECIAL_SELFRAW_MIN "special_selfraw_min"
#define CSV_TP_NOISE_LIMIT "noise_data_limit"
#define CSV_TP_SELFNOISE_LIMIT "noise_selfdata_limit"
#define CSV_TP_TEST_CONFIG "test_config"

#define MAX_TEST_TIME_MS 15000
#define DEFAULT_TEST_TIME_MS 7000

/* berlin A */
#define MAX_DRV_NUM_BRA 21
#define MAX_SEN_NUM_BRA 42
#define SHORT_TEST_TIME_REG_BRA 0x11FF2
#define DFT_ADC_DUMP_NUM_BRA 1396
#define DFT_SHORT_THRESHOLD_BRA 16
#define DFT_DIFFCODE_SHORT_THRESHOLD_BRA 16
#define SHORT_TEST_STATUS_REG_BRA 0x10400
#define SHORT_TEST_RESULT_REG_BRA 0x10410
#define DRV_DRV_SELFCODE_REG_BRA 0x1045E
#define SEN_SEN_SELFCODE_REG_BRA 0x1084E
#define DRV_SEN_SELFCODE_REG_BRA 0x11712
#define DIFF_CODE_DATA_REG_BRA 0x11F72

/* berlin B */
#define MAX_DRV_NUM_BRB 52
#define MAX_SEN_NUM_BRB 75
#define SHORT_TEST_TIME_REG_BRB 0x26AE0
#define DFT_ADC_DUMP_NUM_BRB 762
#define DFT_SHORT_THRESHOLD_BRB 100
#define DFT_DIFFCODE_SHORT_THRESHOLD_BRB 32
#define SHORT_TEST_STATUS_REG_BRB 0x20400
#define SHORT_TEST_RESULT_REG_BRB 0x20410
#define DRV_DRV_SELFCODE_REG_BRB 0x2049A
#define SEN_SEN_SELFCODE_REG_BRB 0x21AF2
#define DRV_SEN_SELFCODE_REG_BRB 0x248A6
#define DIFF_CODE_DATA_REG_BRB 0x269E0

/* berlinD */
#define MAX_DRV_NUM_BRD 20
#define MAX_SEN_NUM_BRD 40
#define SHORT_TEST_TIME_REG_BRD 0x14D7A
#define DFT_ADC_DUMP_NUM_BRD 762
#define DFT_SHORT_THRESHOLD_BRD 100
#define DFT_DIFFCODE_SHORT_THRESHOLD_BRD 32
#define SHORT_TEST_STATUS_REG_BRD 0x13400
#define SHORT_TEST_RESULT_REG_BRD 0x13408
#define DRV_DRV_SELFCODE_REG_BRD 0x1344E
#define SEN_SEN_SELFCODE_REG_BRD 0x137E6
#define DRV_SEN_SELFCODE_REG_BRD 0x14556
#define DIFF_CODE_DATA_REG_BRD 0x14D00

/* nottingham */
#define MAX_DRV_NUM_NOT 17
#define MAX_SEN_NUM_NOT 35
#define SHORT_TEST_TIME_REG_NOT 0x1479E
#define SHORT_TEST_STATUS_REG_NOT 0x13400
#define SHORT_TEST_RESULT_REG_NOT 0x13408
#define DRV_DRV_SELFCODE_REG_NOT 0x13446
#define SEN_SEN_SELFCODE_REG_NOT 0x136EE
#define DRV_SEN_SELFCODE_REG_NOT 0x14152
#define DIFF_CODE_DATA_REG_NOT 0x14734

#define ABS(val) ((val < 0) ? -(val) : val)
#define MAX(a, b) ((a > b) ? a : b)

/* short threshold, drv-drv, drv-sen, sen-sen, drv-gnd, sen-gnd, avdd */
static u8 short_circuit_threshold[] = { 10, 200, 200, 200, 200, 200, 30 };

/* berlin A drv-sen map */
static u8 brl_a_drv_map[] = { 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53,
	54, 55, 56, 57, 58, 59, 60, 61, 62 };

static u8 brl_a_sen_map[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
	33, 34, 35, 36, 37, 38, 39, 40, 41 };

/* berlin B drv-sen map */
static u8 brl_b_drv_map[] = { 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86,
	87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103,
	104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117,
	118, 119, 120, 121, 122, 123, 124, 125, 126 };

static u8 brl_b_sen_map[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
	33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
	51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68,
	69, 70, 71, 72, 73, 74 };

/* berlin D drv-sen map */
static u8 brl_d_drv_map[] = {
	40,
	41,
	42,
	43,
	44,
	45,
	46,
	47,
	48,
	49,
	50,
	51,
	52,
	53,
	54,
	55,
	56,
	57,
	58,
	59,
};

static u8 brl_d_sen_map[] = {
	0,
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	12,
	13,
	14,
	15,
	16,
	17,
	18,
	19,
	20,
	21,
	22,
	23,
	24,
	25,
	26,
	27,
	28,
	29,
	30,
	31,
	32,
	33,
	34,
	35,
	36,
	37,
	38,
	39,
};

/* nottingham drv-sen map */
static u8 not_drv_map[] = { 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51 };

static u8 not_sen_map[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
	33, 34 };

typedef struct __attribute__((packed)) {
	u8 result;
	u8 drv_drv_num;
	u8 sen_sen_num;
	u8 drv_sen_num;
	u8 drv_gnd_avdd_num;
	u8 sen_gnd_avdd_num;
	u16 checksum;
} test_result_t;

struct params_info_t {
	u32 max_drv_num;
	u32 max_sen_num;
	u8 *drv_map;
	u8 *sen_map;
	u32 short_test_time_reg;
	u32 short_test_status_reg;
	u32 short_test_result_reg;
	u32 drv_drv_selfcode_reg;
	u32 sen_sen_selfcode_reg;
	u32 drv_sen_selfcode_reg;
	u32 diffcode_data_reg;
	u16 short_test_dump_num;
	u16 dft_short_threshold;
	u16 short_diffcode_threshold;
};

struct params_info_t params_bra = {
	MAX_DRV_NUM_BRA,
	MAX_SEN_NUM_BRA,
	brl_a_drv_map,
	brl_a_sen_map,
	SHORT_TEST_TIME_REG_BRA,
	SHORT_TEST_STATUS_REG_BRA,
	SHORT_TEST_RESULT_REG_BRA,
	DRV_DRV_SELFCODE_REG_BRA,
	SEN_SEN_SELFCODE_REG_BRA,
	DRV_SEN_SELFCODE_REG_BRA,
	DIFF_CODE_DATA_REG_BRA,
	DFT_ADC_DUMP_NUM_BRA,
	DFT_SHORT_THRESHOLD_BRA,
	DFT_DIFFCODE_SHORT_THRESHOLD_BRA,
};

struct params_info_t params_brb = {
	MAX_DRV_NUM_BRB,
	MAX_SEN_NUM_BRB,
	brl_b_drv_map,
	brl_b_sen_map,
	SHORT_TEST_TIME_REG_BRB,
	SHORT_TEST_STATUS_REG_BRB,
	SHORT_TEST_RESULT_REG_BRB,
	DRV_DRV_SELFCODE_REG_BRB,
	SEN_SEN_SELFCODE_REG_BRB,
	DRV_SEN_SELFCODE_REG_BRB,
	DIFF_CODE_DATA_REG_BRB,
	DFT_ADC_DUMP_NUM_BRB,
	DFT_SHORT_THRESHOLD_BRB,
	DFT_DIFFCODE_SHORT_THRESHOLD_BRB,
};

struct params_info_t params_brd = {
	MAX_DRV_NUM_BRD,
	MAX_SEN_NUM_BRD,
	brl_d_drv_map,
	brl_d_sen_map,
	SHORT_TEST_TIME_REG_BRD,
	SHORT_TEST_STATUS_REG_BRD,
	SHORT_TEST_RESULT_REG_BRD,
	DRV_DRV_SELFCODE_REG_BRD,
	SEN_SEN_SELFCODE_REG_BRD,
	DRV_SEN_SELFCODE_REG_BRD,
	DIFF_CODE_DATA_REG_BRD,
	DFT_ADC_DUMP_NUM_BRD,
	DFT_SHORT_THRESHOLD_BRD,
	DFT_DIFFCODE_SHORT_THRESHOLD_BRD,
};

struct params_info_t params_not = {
	MAX_DRV_NUM_NOT,
	MAX_SEN_NUM_NOT,
	not_drv_map,
	not_sen_map,
	SHORT_TEST_TIME_REG_NOT,
	SHORT_TEST_STATUS_REG_NOT,
	SHORT_TEST_RESULT_REG_NOT,
	DRV_DRV_SELFCODE_REG_NOT,
	SEN_SEN_SELFCODE_REG_NOT,
	DRV_SEN_SELFCODE_REG_NOT,
	DIFF_CODE_DATA_REG_NOT,
	0,
	0,
	0,
};

struct ts_test_params {
	bool test_items[MAX_TEST_ITEMS];

	u32 rawdata_addr;
	u32 noisedata_addr;
	u32 self_rawdata_addr;
	u32 self_noisedata_addr;

	u32 drv_num;
	u32 sen_num;

	struct params_info_t *params_info;

	s32 cfg_buf[GOODIX_CFG_MAX_SIZE];
	s32 max_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s32 min_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s32 deviation_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s32 self_max_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s32 self_min_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s32 noise_threshold;
	s32 self_noise_threshold;

	u32 short_threshold;
	u32 r_drv_drv_threshold;
	u32 r_drv_sen_threshold;
	u32 r_sen_sen_threshold;
	u32 r_drv_gnd_threshold;
	u32 r_sen_gnd_threshold;
	u32 avdd_value;
};

struct ts_test_rawdata {
	s16 data[MAX_DRV_NUM * MAX_SEN_NUM];
	u32 size;
};

struct ts_test_self_rawdata {
	s16 data[MAX_DRV_NUM + MAX_SEN_NUM];
	u32 size;
};

struct ts_short_res {
	u8 short_num;
	s16 short_msg[4 * MAX_SHORT_NUM];
};

struct ts_open_res {
	u8 beyond_max_limit_cnt[MAX_DRV_NUM * MAX_SEN_NUM];
	u8 beyond_min_limit_cnt[MAX_DRV_NUM * MAX_SEN_NUM];
	u8 beyond_accord_limit_cnt[MAX_DRV_NUM * MAX_SEN_NUM];
};

struct goodix_ts_test {
	struct goodix_ts_core *ts;
	struct ts_test_params test_params;
	struct ts_test_rawdata rawdata[TOTAL_FRAME_NUM];
	struct ts_test_rawdata accord_arr[TOTAL_FRAME_NUM];
	struct ts_test_rawdata noisedata[NOISEDATA_TEST_TIMES];
	struct goodix_ic_config test_config;
	struct ts_test_self_rawdata self_rawdata;
	struct ts_test_self_rawdata self_noisedata;
	struct ts_short_res short_res;
	struct ts_open_res open_res;

	/*[0][0][0][0][0]..  0 without test; 1 pass, 2 panel failed; 3 software
	 * failed */
	char test_result[MAX_TEST_ITEMS];
	char test_info[TS_RAWDATA_RESULT_MAX];
};

static int cal_cha_to_cha_res(struct goodix_ts_test *ts_test, int v1, int v2)
{
	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A)
		return (v1 - v2) * 63 / v2;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_B)
		return (v1 - v2) * 74 / v2 + 20;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D)
		return (v1 / v2 - 1) * 70 + 59;
	else
		return (v1 / v2 - 1) * 55 + 45;
}

static int cal_cha_to_avdd_res(struct goodix_ts_test *ts_test, int v1, int v2)
{
	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A)
		return 64 * (2 * v2 - 25) * 40 / v1 - 40;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_B)
		return 64 * (2 * v2 - 25) * 99 / v1 - 60;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D)
		return 64 * (2 * v2 - 25) * 93 / v1 - 20;
	else
		return 64 * (2 * v2 - 25) * 76 / v1 - 15;
}

static int cal_cha_to_gnd_res(struct goodix_ts_test *ts_test, int v)
{
	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A)
		return 64148 / v - 40;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_B)
		return 150500 / v - 60;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D)
		return 145000 / v - 15;
	else
		return 120000 / v - 16;
}

static int ts_test_reset(struct goodix_ts_test *ts_test, u32 delay_ms)
{
	return ts_test->ts->hw_ops->reset(ts_test->ts, delay_ms);
}

static int ts_test_read(
	struct goodix_ts_test *ts_test, u32 addr, u8 *data, u32 len)
{
	return ts_test->ts->hw_ops->read(ts_test->ts, addr, data, len);
}

static int ts_test_write(
	struct goodix_ts_test *ts_test, u32 addr, u8 *data, u32 len)
{
	return ts_test->ts->hw_ops->write(ts_test->ts, addr, data, len);
}

static int ts_test_send_cmd(
	struct goodix_ts_test *ts_test, struct goodix_ts_cmd *cmd)
{
	return ts_test->ts->hw_ops->send_cmd(ts_test->ts, cmd);
}

static int ts_test_irq_enable(struct goodix_ts_test *ts_test, bool flag)
{
	return ts_test->ts->hw_ops->irq_enable(ts_test->ts, flag);
}

/*
static int ts_test_send_config(struct goodix_ts_test *ts_test, int type)
{
	struct goodix_ic_config *cfg;

	if (type >= GOODIX_MAX_CONFIG_GROUP) {
		ts_err("unsupported config type %d", type);
		return -EINVAL;
	}
	cfg = ts_test->ts->ic_configs[type];
	if (!cfg || cfg->len <= 0) {
		ts_err("no valid normal config found");
		return -EINVAL;
	}

	return ts_test->ts->hw_ops->send_config(
		ts_test->ts, cfg->data, cfg->len);
}
*/

static int ts_test_read_version(
	struct goodix_ts_test *ts_test, struct goodix_fw_version *version)
{
	return ts_test->ts->hw_ops->read_version(ts_test->ts, version);
}

static void goodix_init_params(struct goodix_ts_test *ts_test)
{
	struct goodix_ts_core *ts = ts_test->ts;
	struct ts_test_params *test_params = &ts_test->test_params;

	test_params->rawdata_addr = ts->ic_info.misc.mutual_rawdata_addr;
	test_params->noisedata_addr = ts->ic_info.misc.mutual_diffdata_addr;
	test_params->self_rawdata_addr = ts->ic_info.misc.self_rawdata_addr;
	test_params->self_noisedata_addr = ts->ic_info.misc.self_diffdata_addr;

	test_params->short_threshold = short_circuit_threshold[0];
	test_params->r_drv_drv_threshold = short_circuit_threshold[1];
	test_params->r_drv_sen_threshold = short_circuit_threshold[2];
	test_params->r_sen_sen_threshold = short_circuit_threshold[3];
	test_params->r_drv_gnd_threshold = short_circuit_threshold[4];
	test_params->r_sen_gnd_threshold = short_circuit_threshold[5];
	test_params->avdd_value = short_circuit_threshold[6];

	test_params->drv_num = ts->ic_info.parm.drv_num;
	test_params->sen_num = ts->ic_info.parm.sen_num;

	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A)
		test_params->params_info = &params_bra;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_B)
		test_params->params_info = &params_brb;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D)
		test_params->params_info = &params_brd;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_NOTTINGHAM)
		test_params->params_info = &params_not;
}

static int goodix_tptest_prepare(struct goodix_ts_test *ts_test)
{
	ts_info("TP test prepare IN");

	goodix_init_params(ts_test);

	/* disable irq */
	ts_test_irq_enable(ts_test, false);
	/* close esd */
	goodix_ts_esd_off(ts_test->ts);

	return 0;
}

static void goodix_tptest_finish(struct goodix_ts_test *ts_test)
{
	ts_info("TP test finish IN");
	/* reset chip */
	ts_test_reset(ts_test, 100);

	/* open esd */
	goodix_ts_esd_on(ts_test->ts);
	/* enable irq */
	ts_test_irq_enable(ts_test, true);
}

#define PRODUCT_TEST_ADDR 0x15D4C
#define DRV_CHAN_BYTES 7
#define SEN_CHAN_BYTES 10
static void goodix_opencircuit_test(struct goodix_ts_test *ts_test)
{
	struct goodix_ts_cmd temp_cmd;
	u8 buf[24];
	int retry = 20;
	int i;
	int ret;

	ts_info("---------------------- open_test begin ----------------------");
	ts_test->test_result[GTP_OPEN_TEST] = SYS_SOFTWARE_REASON;

	temp_cmd.cmd = 0x63;
	temp_cmd.len = 4;
	ret = ts_test_send_cmd(ts_test, &temp_cmd);
	if (ret < 0) {
		ts_err("send open test cmd failed");
		return;
	}

	msleep(200);

	while (retry--) {
		ret = ts_test_read(
			ts_test, PRODUCT_TEST_ADDR, buf, sizeof(buf));
		if (ret < 0) {
			ts_err("read open test result failed");
			return;
		}
		if (buf[0] == 0xCC && buf[1] == 0xCC)
			break;
		msleep(50);
	}
	if (retry < 0) {
		ts_err("open test not ready, status = %x%x", buf[0], buf[1]);
		return;
	}
	ret = checksum_cmp(buf, sizeof(buf), CHECKSUM_MODE_U8_LE);
	if (ret) {
		ts_err("open test result checksum error");
		return;
	}

	if (buf[2] == 0) {
		ts_info("open test pass");
		ts_test->test_result[GTP_OPEN_TEST] = GTP_TEST_PASS;
	} else {
		ts_err("open test failed");
		ts_test->test_result[GTP_OPEN_TEST] = GTP_PANEL_REASON;

		/* DRV[0~55] total 7 bytes */
		for (i = 0; i < DRV_CHAN_BYTES; i++) {
			if (buf[i + 3])
				ts_info("DRV[%d~%d] open circuit, ret=0x%X",
					i * 8, i * 8 + 7, buf[i + 3]);
		}
		/* SEN[0~79] total 10 bytes */
		for (i = 0; i < SEN_CHAN_BYTES; i++) {
			if (buf[i + 10])
				ts_info("SEN[%d~%d] open circuit, ret=0x%X",
					i * 8, i * 8 + 7, buf[i + 10]);
		}
	}
}

#define SHORT_TEST_RUN_REG 0x10400
#define SHORT_TEST_RUN_FLAG 0xAA
#define INSPECT_FW_SWITCH_CMD 0x85
#define TEST_FW_PID "OST"
static int goodix_short_test_prepare(struct goodix_ts_test *ts_test)
{
	struct goodix_ts_cmd tmp_cmd;
	struct goodix_fw_version fw_ver;
	int ret;
	int retry;
	int resend = 3;
	u8 status;

	ts_info("short test prepare IN");
	ts_test->test_result[GTP_SHORT_TEST] = SYS_SOFTWARE_REASON;
	tmp_cmd.len = 4;
	tmp_cmd.cmd = INSPECT_FW_SWITCH_CMD;

resend_cmd:
	ret = ts_test_send_cmd(ts_test, &tmp_cmd);
	if (ret < 0) {
		ts_err("send test mode failed");
		return ret;
	}

	retry = 3;
	while (retry--) {
		msleep(40);
		if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A) {
			ret = ts_test_read_version(ts_test, &fw_ver);
			if (ret < 0) {
				ts_err("read test version failed");
				return ret;
			}
			ret = memcmp(&(fw_ver.patch_pid[3]), TEST_FW_PID,
				strlen(TEST_FW_PID));
			if (ret == 0)
				return 0;
			else
				ts_info("patch ID dismatch %s != %s",
					fw_ver.patch_pid, TEST_FW_PID);
		} else {
			ret = ts_test_read(
				ts_test, SHORT_TEST_RUN_REG, &status, 1);
			if (!ret && status == SHORT_TEST_RUN_FLAG)
				return 0;
			ts_info("short_mode_status=0x%02x ret=%d", status, ret);
		}
	}

	if (resend--) {
		ts_test_reset(ts_test, 100);
		goto resend_cmd;
	}

	return -EINVAL;
}

static u32 map_die2pin(struct ts_test_params *test_params, u32 chn_num)
{
	int i = 0;
	u32 res = 255;

	if (chn_num & DRV_CHANNEL_FLAG)
		chn_num = (chn_num & ~DRV_CHANNEL_FLAG) +
			  test_params->params_info->max_sen_num;

	for (i = 0; i < test_params->params_info->max_sen_num; i++) {
		if (test_params->params_info->sen_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	/* res != 255 mean found the corresponding channel num */
	if (res != 255)
		return res;
	/* if cannot find in SenMap try find in DrvMap */
	for (i = 0; i < test_params->params_info->max_drv_num; i++) {
		if (test_params->params_info->drv_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	if (i >= test_params->params_info->max_drv_num)
		ts_err("Faild found corrresponding channel num:%d", chn_num);
	else
		res |= DRV_CHANNEL_FLAG;

	return res;
}

static void goodix_save_short_res(
	struct ts_test_params *params, u16 chn1, u16 chn2, int r)
{
	int i;
	u8 repeat_cnt = 0;
	u8 repeat = 0;
	struct goodix_ts_test *ts_test =
		container_of(params, struct goodix_ts_test, test_params);
	struct ts_short_res *short_res = &ts_test->short_res;

	if (chn1 == chn2 || short_res->short_num >= MAX_SHORT_NUM)
		return;

	for (i = 0; i < short_res->short_num; i++) {
		repeat_cnt = 0;
		if (short_res->short_msg[4 * i] == chn1)
			repeat_cnt++;
		if (short_res->short_msg[4 * i] == chn2)
			repeat_cnt++;
		if (short_res->short_msg[4 * i + 1] == chn1)
			repeat_cnt++;
		if (short_res->short_msg[4 * i + 1] == chn2)
			repeat_cnt++;
		if (repeat_cnt >= 2) {
			repeat = 1;
			break;
		}
	}
	if (repeat == 0) {
		short_res->short_msg[4 * short_res->short_num + 0] = chn1;
		short_res->short_msg[4 * short_res->short_num + 1] = chn2;
		short_res->short_msg[4 * short_res->short_num + 2] =
			(r >> 8) & 0xFF;
		short_res->short_msg[4 * short_res->short_num + 3] = r & 0xFF;
		if (short_res->short_num < MAX_SHORT_NUM)
			short_res->short_num++;
	}
}

static int gdix_check_tx_tx_shortcircut(
	struct goodix_ts_test *ts_test, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_drv_num = test_params->params_info->max_drv_num;
	int max_sen_num = test_params->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	data_reg = test_params->params_info->drv_drv_selfcode_reg;
	for (i = 0; i < short_ch_num; i++) {
		ret = ts_test_read(ts_test, data_reg, data_buf, size);
		if (ret < 0) {
			ts_err("Failed read Drv-to-Drv short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Drv-to-Drv adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_drv_drv_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		short_die_num -= max_sen_num;
		if (short_die_num >= max_drv_num) {
			ts_info("invalid short pad num:%d",
				short_die_num + max_sen_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < max_drv_num; j++) {
			adc_signal =
				le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(
				ts_test, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(test_params,
					short_die_num + max_sen_num);
				slave_pin_num = map_die2pin(
					test_params, j + max_sen_num);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(test_params,
					master_pin_num, slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
					short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_rx_rx_shortcircut(
	struct goodix_ts_test *ts_test, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_sen_num = test_params->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_sen_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	data_reg = test_params->params_info->sen_sen_selfcode_reg;
	for (i = 0; i < short_ch_num; i++) {
		ret = ts_test_read(ts_test, data_reg, data_buf, size);
		if (ret) {
			ts_err("Failed read Sen-to-Sen short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Sen-to-Sen adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_sen_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= max_sen_num) {
			ts_info("invalid short pad num:%d", short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < max_sen_num; j++) {
			adc_signal =
				le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(
				ts_test, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num =
					map_die2pin(test_params, short_die_num);
				slave_pin_num = map_die2pin(test_params, j);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(test_params,
					master_pin_num, slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
					short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_tx_rx_shortcircut(
	struct goodix_ts_test *ts_test, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf = NULL;
	u32 data_reg;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_drv_num = test_params->params_info->max_drv_num;
	int max_sen_num = test_params->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&sen shortcircut check */
	data_reg = test_params->params_info->drv_sen_selfcode_reg;
	for (i = 0; i < short_ch_num; i++) {
		ret = ts_test_read(ts_test, data_reg, data_buf, size);
		if (ret) {
			ts_err("Failed read Drv-to-Sen short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Drv-to-Sen adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_drv_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= max_sen_num) {
			ts_info("invalid short pad num:%d", short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = 0; j < max_drv_num; j++) {
			adc_signal =
				le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(
				ts_test, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num =
					map_die2pin(test_params, short_die_num);
				slave_pin_num = map_die2pin(
					test_params, j + max_sen_num);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(test_params,
					master_pin_num, slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
					short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_resistance_to_gnd(
	struct ts_test_params *test_params, u16 adc_signal, u32 pos)
{
	long r = 0;
	u16 r_th = 0, avdd_value = 0;
	u16 chn_id_tmp = 0;
	u8 pin_num = 0;
	unsigned short short_type;
	struct goodix_ts_test *ts_test =
		container_of(test_params, struct goodix_ts_test, test_params);
	int max_drv_num = test_params->params_info->max_drv_num;
	int max_sen_num = test_params->params_info->max_sen_num;

	avdd_value = test_params->avdd_value;
	short_type = adc_signal & 0x8000;
	adc_signal &= ~0x8000;
	if (adc_signal == 0)
		adc_signal = 1;

	if (short_type == 0) {
		/* short to GND */
		r = cal_cha_to_gnd_res(ts_test, adc_signal);
	} else {
		/* short to VDD */
		r = cal_cha_to_avdd_res(ts_test, adc_signal, avdd_value);
	}

	if (pos < max_drv_num)
		r_th = test_params->r_drv_gnd_threshold;
	else
		r_th = test_params->r_sen_gnd_threshold;

	chn_id_tmp = pos;
	if (chn_id_tmp < max_drv_num)
		chn_id_tmp += max_sen_num;
	else
		chn_id_tmp -= max_drv_num;

	if (r < r_th) {
		pin_num = map_die2pin(test_params, chn_id_tmp);
		goodix_save_short_res(test_params, pin_num,
			short_type ? CHN_VDD : CHN_GND, r);
		ts_err("%s%d shortcircut to %s,R=%ldK,R_Threshold=%dK",
			(pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
			(pin_num & ~DRV_CHANNEL_FLAG),
			short_type ? "VDD" : "GND", r, r_th);

		return -EINVAL;
	}

	return 0;
}

static int gdix_check_gndvdd_shortcircut(struct goodix_ts_test *ts_test)
{
	int ret = 0, err = 0;
	int size = 0, i = 0;
	u16 adc_signal = 0;
	u32 data_reg;
	u8 *data_buf = NULL;
	int max_drv_num = ts_test->test_params.params_info->max_drv_num;
	int max_sen_num = ts_test->test_params.params_info->max_sen_num;

	size = (max_drv_num + max_sen_num) * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* read diff code, diff code will be used to calculate
	 * resistance between channel and GND */
	data_reg = ts_test->test_params.params_info->diffcode_data_reg;
	ret = ts_test_read(ts_test, data_reg, data_buf, size);
	if (ret < 0) {
		ts_err("Failed read to-gnd rawdata");
		err = -EINVAL;
		goto err_out;
	}

	if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
		ts_err("diff code checksum error");
		err = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < max_drv_num + max_sen_num; i++) {
		adc_signal = le16_to_cpup((__le16 *)&data_buf[i * 2]);
		ret = gdix_check_resistance_to_gnd(
			&ts_test->test_params, adc_signal, i);
		if (ret != 0) {
			ts_err("Resistance to-gnd/vdd short");
			err = ret;
		}
	}

err_out:
	kfree(data_buf);
	return err;
}

static int goodix_shortcircut_analysis(struct goodix_ts_test *ts_test)
{
	int ret;
	int err = 0;
	test_result_t test_result;

	ret = ts_test_read(ts_test,
		ts_test->test_params.params_info->short_test_result_reg,
		(u8 *)&test_result, sizeof(test_result));
	if (ret < 0) {
		ts_err("Read TEST_RESULT_REG failed");
		return ret;
	}

	if (checksum_cmp((u8 *)&test_result, sizeof(test_result),
		    CHECKSUM_MODE_U8_LE)) {
		ts_err("shrot result checksum err");
		return -EINVAL;
	}

	if (!(test_result.result & 0x0F)) {
		ts_info(">>>>> No shortcircut");
		return 0;
	}
	ts_info("short flag 0x%02x, drv&drv:%d, sen&sen:%d, drv&sen:%d, drv/GNDVDD:%d, sen/GNDVDD:%d",
		test_result.result, test_result.drv_drv_num,
		test_result.sen_sen_num, test_result.drv_sen_num,
		test_result.drv_gnd_avdd_num, test_result.sen_gnd_avdd_num);

	if (test_result.drv_drv_num)
		err |= gdix_check_tx_tx_shortcircut(
			ts_test, test_result.drv_drv_num);
	if (test_result.sen_sen_num)
		err |= gdix_check_rx_rx_shortcircut(
			ts_test, test_result.sen_sen_num);
	if (test_result.drv_sen_num)
		err |= gdix_check_tx_rx_shortcircut(
			ts_test, test_result.drv_sen_num);
	if (test_result.drv_gnd_avdd_num || test_result.sen_gnd_avdd_num)
		err |= gdix_check_gndvdd_shortcircut(ts_test);

	ts_info(">>>>> short check return 0x%x", err);

	return err;
}

#define SHORT_FW_CMD_REG 0x10400
static int send_test_cmd(
	struct goodix_ts_test *ts_test, struct goodix_ts_cmd *cmd)
{
	int ret;
	u32 reg = SHORT_FW_CMD_REG;
	cmd->state = 0;
	cmd->ack = 0;
	goodix_append_checksum(
		&(cmd->buf[2]), cmd->len - 2, CHECKSUM_MODE_U8_LE);
	ret = ts_test_write(ts_test, reg, cmd->buf, cmd->len + 2);
	if (ret < 0)
		return ret;
	usleep_range(10000, 11000);
	return ret;
}

#define INSPECT_PARAM_CMD 0xAA
#define SHORT_TEST_FINISH_FLAG 0x88
#define SHORT_TEST_THRESHOLD_REG 0x20402
static void goodix_shortcircuit_test(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	int retry;
	u16 test_time;
	u8 status;
	int ic_type = ts_test->ts->bus->ic_type;
	struct goodix_ts_cmd test_parm_cmd;
	// u8 test_param[6];

	ts_info("---------------------- short_test begin ----------------------");
	ret = goodix_short_test_prepare(ts_test);
	if (ret < 0) {
		ts_err("Failed enter short test mode");
		return;
	}

	/* get short test time */
	ret = ts_test_read(ts_test,
		ts_test->test_params.params_info->short_test_time_reg,
		(u8 *)&test_time, 2);
	if (ret < 0) {
		ts_err("Failed to get test_time, default %dms",
			DEFAULT_TEST_TIME_MS);
		test_time = DEFAULT_TEST_TIME_MS;
	} else {
		if (ic_type == IC_TYPE_BERLIN_A)
			test_time /= 10;
		if (test_time > MAX_TEST_TIME_MS) {
			ts_info("test time too long %d > %d", test_time,
				MAX_TEST_TIME_MS);
			test_time = MAX_TEST_TIME_MS;
		}
		ts_info("get test time %dms", test_time);
	}

	/* start short test */
	if (ic_type == IC_TYPE_BERLIN_A) {
		test_parm_cmd.len = 0x0A;
		test_parm_cmd.cmd = INSPECT_PARAM_CMD;
		test_parm_cmd.data[0] =
			ts_test->test_params.params_info->dft_short_threshold &
			0xFF;
		test_parm_cmd.data[1] = (ts_test->test_params.params_info
							->dft_short_threshold >>
						8) &
					0xFF;
		test_parm_cmd.data[2] = ts_test->test_params.params_info
						->short_diffcode_threshold &
					0xFF;
		test_parm_cmd.data[3] =
			(ts_test->test_params.params_info
					->short_diffcode_threshold >>
				8) &
			0xFF;
		test_parm_cmd.data[4] =
			ts_test->test_params.params_info->short_test_dump_num &
			0xFF;
		test_parm_cmd.data[5] = (ts_test->test_params.params_info
							->short_test_dump_num >>
						8) &
					0xFF;
		ret = send_test_cmd(ts_test, &test_parm_cmd);
		if (ret < 0) {
			ts_err("send INSPECT_PARAM_CMD failed");
			return;
		}
	} else {
		// test_param[0] =
		// ts_test->test_params.params_info->dft_short_threshold & 0xFF;
		// test_param[1] =
		// (ts_test->test_params.params_info->dft_short_threshold >> 8)
		// & 0xFF; test_param[2] =
		// ts_test->test_params.params_info->short_diffcode_threshold &
		// 0xFF; test_param[3] =
		// (ts_test->test_params.params_info->short_diffcode_threshold
		// >> 8) & 0xFF; test_param[4] =
		// ts_test->test_params.params_info->short_test_dump_num & 0xFF;
		// test_param[5] =
		// (ts_test->test_params.params_info->short_test_dump_num >> 8)
		// & 0xFF; ts_test_write(ts_test, SHORT_TEST_THRESHOLD_REG,
		// test_param, sizeof(test_param));
		status = 0;
		ts_test_write(ts_test, SHORT_TEST_RUN_REG, &status, 1);
	}

	/* wait short test finish */
	msleep(test_time);
	retry = 50;
	while (retry--) {
		ret = ts_test_read(ts_test,
			ts_test->test_params.params_info->short_test_status_reg,
			&status, 1);
		if (!ret && status == SHORT_TEST_FINISH_FLAG)
			break;
		msleep(50);
	}
	if (retry < 0) {
		ts_err("short test failed, status:0x%02x", status);
		return;
	}

	/* start analysis short result */
	ts_info("short_test finished, start analysis");
	ret = goodix_shortcircut_analysis(ts_test);
	if (ret < 0)
		ts_test->test_result[GTP_SHORT_TEST] = GTP_PANEL_REASON;
	else
		ts_test->test_result[GTP_SHORT_TEST] = GTP_TEST_PASS;
}

int goodix_do_inspect(struct goodix_ts_core *cd, struct ts_rawdata_info *info)
{
	int ret;
	int cnt;
	struct goodix_ts_test *ts_test = NULL;

	if (!cd || !info) {
		ts_err("core_data or info is NULL");
		return -ENODEV;
	}

	ts_test = kzalloc(sizeof(*ts_test), GFP_KERNEL);
	if (!ts_test)
		return -ENOMEM;

	ts_test->ts = cd;
	ret = goodix_tptest_prepare(ts_test);
	if (ret < 0) {
		ts_err("Failed to prepare TP test, exit");
		strncpy(info->result, "[FAIL]\n", TS_RAWDATA_RESULT_MAX - 1);
		goto exit_finish;
	}
	ts_info("TP test prepare OK");

	goodix_opencircuit_test(ts_test);
	goodix_shortcircuit_test(ts_test);

	cnt = snprintf(info->result, TS_RAWDATA_RESULT_MAX - 1,
		"open_test-[%s] ",
		(ts_test->test_result[GTP_OPEN_TEST] == GTP_TEST_PASS)
			? "PASS"
			: "FAIL");
	snprintf(&info->result[cnt], TS_RAWDATA_RESULT_MAX - 1,
		"short_test-[%s]\n",
		(ts_test->test_result[GTP_SHORT_TEST] == GTP_TEST_PASS)
			? "PASS"
			: "FAIL");

	goodix_tptest_finish(ts_test);
exit_finish:
	kfree(ts_test);
	return ret;
}

/* show rawdata */
static ssize_t get_rawdata_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct ts_rawdata_info *info = NULL;
	struct goodix_ts_core *cd = dev_get_drvdata(dev);

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	goodix_do_inspect(cd, info);

	ret = snprintf(buf, TS_RAWDATA_RESULT_MAX - 1, "%s", info->result);

	kfree(info);
	return ret;
}

static DEVICE_ATTR(get_rawdata, 0444, get_rawdata_show, NULL);

int inspect_module_init(struct goodix_ts_core *core_data)
{
	int ret;
	struct kobject *def_kobj = &core_data->pdev->dev.kobj;

	/* create sysfs */
	ret = sysfs_create_file(def_kobj, &dev_attr_get_rawdata.attr);
	if (ret < 0) {
		ts_err("create sysfs of get_rawdata failed");
		goto err_out;
	}
	ts_info("inspect module init success");
	return 0;

err_out:
	ts_err("inspect module init failed!");
	return ret;
}

void inspect_module_exit(struct goodix_ts_core *core_data)
{
	struct kobject *def_kobj = &core_data->pdev->dev.kobj;

	ts_info("inspect module exit");
	sysfs_remove_file(def_kobj, &dev_attr_get_rawdata.attr);
}
