#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/version.h>

#include "goodix_ts_core.h"

#define CMD_FW_UPDATE "fw_update"
#define CMD_AUTO_TEST "auto_test"
#define CMD_STYLUS_RAW_TEST "stylus_raw_test"
#define CMD_STYLUS_OSC_TEST "stylus_osc_test"
#define CMD_OPEN_TEST "open_test"
#define CMD_SELF_OPEN_TEST "self_open_test"
#define CMD_NOISE_TEST "noise_test"
#define CMD_AUTO_NOISE_TEST "auto_noise_test"
#define CMD_SHORT_TEST "short_test"
#define CMD_GET_PACKAGE_ID "get_package_id"
#define CMD_GET_MCU_ID "get_mcu_id"
#define CMD_GET_VERSION "get_version"
#define CMD_GET_RAWDATA "get_raw"
#define CMD_GET_DIFFDATA "get_diff"
#define CMD_GET_BASEDATA "get_base"
#define CMD_GET_SELF_RAWDATA "get_self_raw"
#define CMD_GET_SELF_DIFFDATA "get_self_diff"
#define CMD_GET_SELF_BASEDATA "get_self_base"
#define CMD_SET_DOUBLE_TAP "set_double_tap"
#define CMD_SET_SINGLE_TAP "set_single_tap"
#define CMD_SET_LONG_PRESS "set_long_press"
#define CMD_SET_ST_PARAM "set_st_param"
#define CMD_SET_LP_PARAM "set_lp_param"
#define CMD_SET_CHARGE_MODE "set_charge_mode"
#define CMD_SET_IRQ_ENABLE "set_irq_enable"
#define CMD_SET_ESD_ENABLE "set_esd_enable"
#define CMD_SET_DEBUG_LOG "set_debug_log"
#define CMD_SET_SCAN_MODE "set_scan_mode"
#define CMD_GET_SCAN_MODE "get_scan_mode"
#define CMD_SET_CONTINUE_MODE "set_continue_mode"
#define CMD_GET_CHANNEL_NUM "get_channel_num"
#define CMD_GET_TX_FREQ "get_tx_freq"
#define CMD_RESET "reset"
#define CMD_SET_SENSE_MODE "set_sense_mode"
#define CMD_GET_CONFIG "get_config"
#define CMD_GET_FW_STATUS "get_fw_status"
#define CMD_SET_HIGHSENSE_MODE "set_highsense_mode"
#define CMD_SET_GRIP_DATA "set_grip_data"
#define CMD_SET_GRIP_MODE "set_grip_mode"
#define CMD_SET_PALM_MODE "set_palm_mode"
#define CMD_SET_NOISE_MODE "set_noise_mode"
#define CMD_SET_WATER_MODE "set_water_mode"
#define CMD_SET_HEATMAP "set_heatmap"
#define CMD_GET_SELF_COMPEN "get_self_compensation"
#define CMD_SET_REPORT_RATE "set_report_rate"
#define CMD_GET_DUMP_LOG "get_dump_log"
#define CMD_GET_STYLUS_DATA "get_stylus_data"
#define CMD_SET_FREQ_INDEX "set_freq_index"
#define CMD_DISABLE_FILTER "disable_filter"
#define CMD_GET_IM_DATA "get_im_data"
#define CMD_SET_HSYNC_SPEED "set_hsync_speed"
#define CMD_SET_WIRELESS_CHARGE "set_wireless_charge"
#define CMD_SET_CONTINUE_HEATMAP "set_continue_heatmap"

struct cmd_handler {
	const char *name;
	void (*handler)(struct goodix_ts_core *cd, int *buf, int bufsz);
};

/* test limits keyword */
#define CSV_TP_SPECIAL_RAW_MIN "special_raw_min"
#define CSV_TP_SPECIAL_RAW_MAX "special_raw_max"
#define CSV_TP_SPECIAL_FREQ_RAW_MIN "special_freq_raw_min"
#define CSV_IP_SPECIAL_FREQ_RAW_MAX "special_freq_raw_max"
#define CSV_TP_SPECIAL_RAW_DELTA "special_raw_delta"
#define CSV_TP_SHORT_THRESHOLD "shortciurt_threshold"
#define CSV_TP_SPECIAL_SELFRAW_MAX "special_selfraw_max"
#define CSV_TP_SPECIAL_SELFRAW_MIN "special_selfraw_min"
#define CSV_TP_SPECIAL_STYLUSRAW_MAX "special_stylusraw_max"
#define CSV_TP_SPECIAL_STYLUSRAW_MIN "special_stylusraw_min"
#define CSV_TP_SPECIAL_FREQ_STYLUSRAW_MAX "special_freq_stylusraw_max"
#define CSV_TP_SPECIAL_FREQ_STYLUSRAW_MIN "special_freq_stylusraw_min"
#define CSV_TP_NOISE_LIMIT "noise_data_limit"
#define CSV_TP_SELFNOISE_LIMIT "noise_selfdata_limit"
#define CSV_TP_ICAL_RAW_MIN "ical_raw_min"
#define CSV_TP_ICAL_RAW_MAX "ical_raw_max"
#define CSV_TP_ICAL_SHORT_THRESHOLD "ical_shortciurt_threshold"
#define CSV_TP_TEST_CONFIG "test_config"

#define PALM_FUNC 0
#define NOISE_FUNC 1
#define WATER_FUNC 2
#define GRIP_FUNC 3

#define SHORT_SIZE 512
#define LARGE_SIZE (15 * 1024)
#define HUGE_SIZE  (100 * 1024)
static char wbuf[SHORT_SIZE];
static char *rbuf;
static uint32_t index;

/* factory test */
#define DISCARD_FRAME_NUMS 6
#define ABS(x) (((x) >= 0) ? (x) : -(x))
#define MAX(a, b) ((a > b) ? a : b)

#define GTP_CAP_TEST 1
#define GTP_DELTA_TEST 2
#define GTP_NOISE_TEST 3
#define GTP_SHORT_TEST 5
#define GTP_SELFCAP_TEST 6
#define GTP_SELFNOISE_TEST 7
#define GTP_STYLUS_RAW_TEST 8
#define MAX_TEST_ITEMS 10 /* 0P-1P-2P-3P-5P total test items */

#define TEST_OK 1
#define TEST_NG 0

#define MAX_LINE_LEN (1024 * 10)
#define MAX_DRV_NUM 52
#define MAX_SEN_NUM 75
#define MAX_SHORT_NUM 15

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

#define GESTURE_STTW 0
#define GESTURE_LPTW 1
typedef union __attribute__((packed)) {
	struct {
		u8 length;
		u8 type;
		u16 st_min_x;
		u16 st_max_x;
		u16 st_min_y;
		u16 st_max_y;
		u16 st_min_count;
		u16 st_max_count;
		u16 st_motion_tolerance;
		u16 st_max_size;
		u16 checksum;
	};
	u8 buf[20];
} gesture_param_st_t;
static gesture_param_st_t gesture_param_st;

typedef union __attribute__((packed)) {
	struct {
		u8 length;
		u8 type;
		u16 lp_min_x;
		u16 lp_max_x;
		u16 lp_min_y;
		u16 lp_max_y;
		u16 lp_min_count;
		u16 lp_max_size;
		u16 lp_marginal_min_x;
		u16 lp_marginal_max_x;
		u16 lp_marginal_min_y;
		u16 lp_marginal_max_y;
		u8 lp_monitor_chan_min_tx;
		u8 lp_monitor_chan_max_tx;
		u8 lp_monitor_chan_min_rx;
		u8 lp_monitor_chan_max_rx;
		u16 lp_min_node_count;
		u16 lp_motion_tolerance_inner;
		u16 lp_motion_tolerance_outer;
		u16 checksum;
	};
	u8 buf[34];
} gesture_param_lp_t;
static gesture_param_lp_t gesture_param_lp;

typedef struct __attribute__((packed)) {
	u8 result;
	u8 drv_drv_num;
	u8 sen_sen_num;
	u8 drv_sen_num;
	u8 drv_gnd_avdd_num;
	u8 sen_gnd_avdd_num;
	u16 checksum;
} test_result_t;

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

struct ts_short_test_info {
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

static struct ts_short_test_info params_brb = {
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

static struct ts_short_test_info params_brd = {
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

static struct ts_short_test_info params_not = {
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

struct ts_short_res {
	u8 short_num;
	s16 short_msg[4 * MAX_SHORT_NUM];
};

struct ts_test_rawdata {
	s16 data[MAX_DRV_NUM * MAX_SEN_NUM];
	u32 size;
};

struct ts_test_self_rawdata {
	s16 data[MAX_DRV_NUM + MAX_SEN_NUM];
	u32 size;
};

struct ts_test_stylus_rawdata {
	s16 data[MAX_DRV_NUM + MAX_SEN_NUM];
	u32 size;
};

static int raw_data_cnt;
static int noise_data_cnt;

struct goodix_ts_test {
	bool item[MAX_TEST_ITEMS];
	char result[MAX_TEST_ITEMS];
	s16 min_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s16 max_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s16 deviation_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s16 self_max_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s16 self_min_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s16 stylus_min_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s16 stylus_max_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s16 noise_threshold;
	s16 short_threshold;
	s16 r_drv_drv_threshold;
	s16 r_drv_sen_threshold;
	s16 r_sen_sen_threshold;
	s16 r_drv_gnd_threshold;
	s16 r_sen_gnd_threshold;
	s16 avdd_value;

	int freq;
	bool is_ical;
	int stylus_test_freq;
	struct ts_test_rawdata *rawdata;
	struct ts_test_rawdata *deltadata;
	struct ts_test_rawdata *noisedata;
	struct ts_test_self_rawdata selfrawdata;
	struct ts_test_stylus_rawdata *stylusraw;
	struct ts_short_test_info *params_info;
	struct ts_short_res short_res;
};
static struct goodix_ts_test *ts_test;

static int cal_cha_to_cha_res(struct goodix_ts_core *cd, int v1, int v2)
{
	if (cd->bus->ic_type == IC_TYPE_BERLIN_A)
		return (v1 - v2) * 63 / v2;
	else if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
		return (v1 - v2) * 74 / v2 + 20;
	else if (cd->bus->ic_type == IC_TYPE_BERLIN_D)
		return (v1 / v2 - 1) * 70 + 59;
	else
		return (v1 / v2 - 1) * 55 + 45;
}

static int cal_cha_to_avdd_res(struct goodix_ts_core *cd, int v1, int v2)
{
	if (cd->bus->ic_type == IC_TYPE_BERLIN_A)
		return 64 * (2 * v2 - 25) * 40 / v1 - 40;
	else if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
		return 64 * (2 * v2 - 25) * 99 / v1 - 60;
	else if (cd->bus->ic_type == IC_TYPE_BERLIN_D)
		return 64 * (2 * v2 - 25) * 93 / v1 - 20;
	else
		return 64 * (2 * v2 - 25) * 76 / v1 - 15;
}

static int cal_cha_to_gnd_res(struct goodix_ts_core *cd, int v)
{
	if (cd->bus->ic_type == IC_TYPE_BERLIN_A)
		return 64148 / v - 40;
	else if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
		return 150500 / v - 60;
	else if (cd->bus->ic_type == IC_TYPE_BERLIN_D)
		return 145000 / v - 15;
	else
		return 120000 / v - 16;
}

/*
 * [GOOG]
 * Use kvzalloc/kzalloc to alloc instead of vmalloc/kmalloc(vendor).
 * This will help to use kvfree for all cases w/o additional `malloc_type`.
 */
//static u8 malloc_type;
static inline void *malloc_proc_buffer(size_t size)
{
	if (size > HUGE_SIZE)
		return kvzalloc(size, GFP_KERNEL);

	return kzalloc(size, GFP_KERNEL);
}

static inline void free_proc_buffer(char **ptr)
{
	if (*ptr == NULL)
		return;

	kvfree(*ptr);
	*ptr = NULL;
}
/*~[GOOG] */

static int malloc_test_resource(void)
{
	ts_test = vzalloc(sizeof(*ts_test));
	if (!ts_test)
		return -ENOMEM;
	if (raw_data_cnt > 0) {
		ts_test->rawdata =
			vzalloc(raw_data_cnt * sizeof(struct ts_test_rawdata));
		if (ts_test->rawdata == NULL)
			return -ENOMEM;

		ts_test->deltadata =
			vzalloc(raw_data_cnt * sizeof(struct ts_test_rawdata));
		if (ts_test->deltadata == NULL)
			return -ENOMEM;

		ts_test->stylusraw =
			vzalloc(raw_data_cnt * sizeof(struct ts_test_stylus_rawdata));
		if (ts_test->stylusraw == NULL)
			return -ENOMEM;
	}
	if (noise_data_cnt > 0) {
		ts_test->noisedata = vzalloc(
			noise_data_cnt * sizeof(struct ts_test_rawdata));
		if (ts_test->noisedata == NULL)
			return -ENOMEM;
	}

	return 0;
}

static void release_test_resource(void)
{
	if (ts_test) {
		vfree(ts_test->rawdata);
		vfree(ts_test->deltadata);
		vfree(ts_test->noisedata);
		vfree(ts_test->stylusraw);
		vfree(ts_test);
		ts_test = NULL;
	}
	raw_data_cnt = 0;
	noise_data_cnt = 0;
}

#define CHN_VDD 0xFF
#define CHN_GND 0x7F
#define DRV_CHANNEL_FLAG 0x80
static u32 map_die2pin(u32 chn_num)
{
	int i = 0;
	u32 res = 255;

	if (chn_num & DRV_CHANNEL_FLAG)
		chn_num = (chn_num & ~DRV_CHANNEL_FLAG) +
			  ts_test->params_info->max_sen_num;

	for (i = 0; i < ts_test->params_info->max_sen_num; i++) {
		if (ts_test->params_info->sen_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	/* res != 255 mean found the corresponding channel num */
	if (res != 255)
		return res;
	/* if cannot find in SenMap try find in DrvMap */
	for (i = 0; i < ts_test->params_info->max_drv_num; i++) {
		if (ts_test->params_info->drv_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	if (i >= ts_test->params_info->max_drv_num)
		ts_err("Faild found corrresponding channel num:%d", chn_num);
	else
		res |= DRV_CHANNEL_FLAG;

	return res;
}

static void goodix_save_short_res(u16 chn1, u16 chn2, int r)
{
	int i;
	u8 repeat_cnt = 0;
	u8 repeat = 0;
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

static int gdix_check_tx_tx_shortcircut(struct goodix_ts_core *cd, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg = ts_test->params_info->drv_drv_selfcode_reg;
	int max_drv_num = ts_test->params_info->max_drv_num;
	int max_sen_num = ts_test->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = cd->hw_ops->read(cd, data_reg, data_buf, size);
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

		r_threshold = ts_test->r_drv_drv_threshold;
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

			if (adc_signal < ts_test->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(cd, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(short_die_num + max_sen_num);
				slave_pin_num = map_die2pin(j + max_sen_num);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(
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

static int gdix_check_rx_rx_shortcircut(struct goodix_ts_core *cd, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg = ts_test->params_info->sen_sen_selfcode_reg;
	int max_sen_num = ts_test->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_sen_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = cd->hw_ops->read(cd, data_reg, data_buf, size);
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

		r_threshold = ts_test->r_sen_sen_threshold;
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

			if (adc_signal < ts_test->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(cd, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(short_die_num);
				slave_pin_num = map_die2pin(j);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(
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

static int gdix_check_tx_rx_shortcircut(struct goodix_ts_core *cd, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf = NULL;
	u32 data_reg = ts_test->params_info->drv_sen_selfcode_reg;
	int max_drv_num = ts_test->params_info->max_drv_num;
	int max_sen_num = ts_test->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&sen shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = cd->hw_ops->read(cd, data_reg, data_buf, size);
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

		r_threshold = ts_test->r_drv_sen_threshold;
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

			if (adc_signal < ts_test->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(cd, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(short_die_num);
				slave_pin_num = map_die2pin(j + max_sen_num);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(
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

static int gdix_check_resistance_to_gnd(struct goodix_ts_core *cd, u16 adc_signal, u32 pos)
{
	long r = 0;
	u16 r_th = 0, avdd_value = 0;
	u16 chn_id_tmp = 0;
	u8 pin_num = 0;
	unsigned short short_type;
	int max_drv_num = ts_test->params_info->max_drv_num;
	int max_sen_num = ts_test->params_info->max_sen_num;

	avdd_value = ts_test->avdd_value;
	short_type = adc_signal & 0x8000;
	adc_signal &= ~0x8000;
	if (adc_signal == 0)
		adc_signal = 1;

	if (short_type == 0) {
		/* short to GND */
		r = cal_cha_to_gnd_res(cd, adc_signal);
	} else {
		/* short to VDD */
		r = cal_cha_to_avdd_res(cd, adc_signal, avdd_value);
	}

	if (pos < max_drv_num)
		r_th = ts_test->r_drv_gnd_threshold;
	else
		r_th = ts_test->r_sen_gnd_threshold;

	chn_id_tmp = pos;
	if (chn_id_tmp < max_drv_num)
		chn_id_tmp += max_sen_num;
	else
		chn_id_tmp -= max_drv_num;

	if (r < r_th) {
		pin_num = map_die2pin(chn_id_tmp);
		goodix_save_short_res(
			pin_num, short_type ? CHN_VDD : CHN_GND, r);
		ts_err("%s%d shortcircut to %s,R=%ldK,R_Threshold=%dK",
			(pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
			(pin_num & ~DRV_CHANNEL_FLAG),
			short_type ? "VDD" : "GND", r, r_th);

		return -EINVAL;
	}

	return 0;
}

static int gdix_check_gndvdd_shortcircut(struct goodix_ts_core *cd)
{
	int ret = 0, err = 0;
	int size = 0, i = 0;
	u16 adc_signal = 0;
	u32 data_reg = ts_test->params_info->diffcode_data_reg;
	u8 *data_buf = NULL;
	int max_drv_num = ts_test->params_info->max_drv_num;
	int max_sen_num = ts_test->params_info->max_sen_num;

	size = (max_drv_num + max_sen_num) * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* read diff code, diff code will be used to calculate
	 * resistance between channel and GND */
	ret = cd->hw_ops->read(cd, data_reg, data_buf, size);
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
		ret = gdix_check_resistance_to_gnd(cd, adc_signal, i);
		if (ret != 0) {
			ts_err("Resistance to-gnd/vdd short");
			err = ret;
		}
	}

err_out:
	kfree(data_buf);
	return err;
}

#define NOTT_CHECKSUM_LEN 54
#define BRLD_CHECKSUM_LEN 62
#define BRLB_CHECKSUM_LEN 129
static int goodix_shortcircut_analysis(struct goodix_ts_core *cd)
{
	int ret;
	int err = 0;
	u8 temp_buf[140];
	test_result_t test_result;
	int checksum_len = 0;

	ret = cd->hw_ops->read(cd, ts_test->params_info->short_test_result_reg,
		temp_buf, sizeof(temp_buf));
	if (ret < 0) {
		ts_err("Read TEST_RESULT_REG failed");
		return ret;
	}

	if (cd->bus->ic_type == IC_TYPE_BERLIN_B) {
		checksum_len = BRLB_CHECKSUM_LEN;
	} else if (cd->bus->ic_type == IC_TYPE_BERLIN_D) {
		checksum_len = BRLD_CHECKSUM_LEN;
	} else if (cd->bus->ic_type == IC_TYPE_NOTTINGHAM) {
		checksum_len = NOTT_CHECKSUM_LEN;
	}

	if (checksum_cmp(&temp_buf[sizeof(test_result)], checksum_len,
		    CHECKSUM_MODE_U8_LE)) {
		ts_err("short result checksum err");
		return -EINVAL;
	}

	memcpy((u8 *)&test_result, temp_buf, sizeof(test_result));

	if (!(test_result.result & 0x0F)) {
		ts_info(">>>>> No shortcircut");
		return 0;
	}
	ts_info("short flag 0x%02x, drv&drv:%d, sen&sen:%d, drv&sen:%d, drv/GNDVDD:%d, sen/GNDVDD:%d",
		test_result.result, test_result.drv_drv_num,
		test_result.sen_sen_num, test_result.drv_sen_num,
		test_result.drv_gnd_avdd_num, test_result.sen_gnd_avdd_num);

	if (test_result.drv_drv_num)
		err |= gdix_check_tx_tx_shortcircut(cd, test_result.drv_drv_num);
	if (test_result.sen_sen_num)
		err |= gdix_check_rx_rx_shortcircut(cd, test_result.sen_sen_num);
	if (test_result.drv_sen_num)
		err |= gdix_check_tx_rx_shortcircut(cd, test_result.drv_sen_num);
	if (test_result.drv_gnd_avdd_num || test_result.sen_gnd_avdd_num)
		err |= gdix_check_gndvdd_shortcircut(cd);

	ts_info(">>>>> short check return 0x%x", err);

	return err;
}

#define INSPECT_FW_SWITCH_CMD 0x85
#define SHORT_TEST_RUN_FLAG 0xAA
#define SHORT_TEST_RUN_REG 0x10400
static int goodix_short_test_prepare(struct goodix_ts_core *cd)
{
	struct goodix_ts_cmd tmp_cmd;
	int ret;
	int retry;
	int resend = 3;
	u8 status;

	ts_info("short test prepare IN");
	tmp_cmd.len = 4;
	tmp_cmd.cmd = INSPECT_FW_SWITCH_CMD;

resend_cmd:
	ret = cd->hw_ops->send_cmd(cd, &tmp_cmd);
	if (ret < 0) {
		ts_err("send test mode failed");
		return ret;
	}

	retry = 3;
	while (retry--) {
		msleep(40);
		ret = cd->hw_ops->read(cd, SHORT_TEST_RUN_REG, &status, 1);
		if (!ret && status == SHORT_TEST_RUN_FLAG)
			return 0;
		ts_info("short_mode_status=0x%02x ret=%d", status, ret);
	}

	if (resend--) {
		cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
		goto resend_cmd;
	}

	return -EINVAL;
}

#define MAX_TEST_TIME_MS 15000
#define DEFAULT_TEST_TIME_MS 7000
#define SHORT_TEST_FINISH_FLAG 0x88
static int goodix_shortcircut_test(struct goodix_ts_core *cd)
{
	int ret = 0;
	int res;
	int retry;
	u16 test_time;
	u8 status;

	ts_info("---------------------- short_test begin ----------------------");
	ret = goodix_short_test_prepare(cd);
	if (ret < 0) {
		ts_err("Failed enter short test mode");
		return ret;
	}

	if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
		msleep(500);

	/* get short test time */
	ret = cd->hw_ops->read(cd, ts_test->params_info->short_test_time_reg,
		(u8 *)&test_time, 2);
	if (ret < 0) {
		ts_err("Failed to get test_time, default %dms",
			DEFAULT_TEST_TIME_MS);
		test_time = DEFAULT_TEST_TIME_MS;
	} else {
		if (test_time > MAX_TEST_TIME_MS) {
			ts_info("test time too long %d > %d", test_time,
				MAX_TEST_TIME_MS);
			test_time = MAX_TEST_TIME_MS;
		}
		ts_info("get test time %dms", test_time);
	}

	/* start short test */
	status = 0;
	cd->hw_ops->write(cd, SHORT_TEST_RUN_REG, &status, 1);

	/* wait short test finish */
	if (test_time > 0)
		msleep(test_time);

	retry = 50;
	while (retry--) {
		ret = cd->hw_ops->read(cd,
			ts_test->params_info->short_test_status_reg, &status,
			1);
		if (!ret && status == SHORT_TEST_FINISH_FLAG)
			break;
		msleep(50);
	}
	if (retry < 0) {
		ts_err("short test failed, status:0x%02x", status);
		return -EINVAL;
	}

	/* start analysis short result */
	ts_info("short_test finished, start analysis");
	res = goodix_shortcircut_analysis(cd);
	if (res == 0) {
		ts_test->result[GTP_SHORT_TEST] = TEST_OK;
		ret = 0;
	}

	return ret;
}

static void *seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= index)
		return NULL;

	return rbuf + *pos;
}

static int seq_show(struct seq_file *s, void *v)
{
	seq_printf(s, (u8 *)v);
	return 0;
}

static void *seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	*pos += index;
	return NULL;
}

static void seq_stop(struct seq_file *s, void *v)
{
	if (s->read_pos >= index) {
		// ts_info("read_pos:%d", (int)s->read_pos);
		free_proc_buffer(&rbuf);
		index = 0;
		release_test_resource();
	}
}

static const struct seq_operations seq_ops = {
	.start = seq_start, .next = seq_next, .stop = seq_stop, .show = seq_show
};

static int driver_test_open(struct inode *inode, struct file *file)
{
	struct seq_file *seq_file;
	int ret;

	ret = seq_open(file, &seq_ops);
	if (ret)
		return ret;
	seq_file = (struct seq_file *)file->private_data;
	if (seq_file) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		seq_file->private = pde_data(inode);
#else
		seq_file->private = PDE_DATA(inode);
#endif
	}

	mutex_lock(&goodix_devices.mutex);
	return 0;
}

static int driver_test_release(struct inode *inode, struct file *file)
{
	mutex_unlock(&goodix_devices.mutex);
	return seq_release(inode, file);
}

static void goodix_save_header(struct goodix_ts_core *cd)
{
	int i;
	bool total_result = true;

	index = sprintf(
		&rbuf[index], "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
	index += sprintf(&rbuf[index], "<TESTLOG>\n");
	index += sprintf(&rbuf[index], "<Header>\n");
	/* sava test result */
	for (i = 0; i < MAX_TEST_ITEMS; i++) {
		if (ts_test->item[i] && ts_test->result[i] == TEST_NG)
			total_result = false;
	}
	if (total_result == false) {
		index += sprintf(&rbuf[index], "<Result>NG</Result>\n");
	} else {
		index += sprintf(&rbuf[index], "<Result>OK</Result>\n");
	}

	index += sprintf(&rbuf[index], "<DeviceType>GT%s</DeviceType>\n",
		cd->fw_version.patch_pid);
	index += sprintf(&rbuf[index], "<SensorId>%d</SensorId>\n",
		cd->fw_version.sensor_id);
	index += sprintf(&rbuf[index], "</Header>\n");

	index += sprintf(&rbuf[index], "<ItemList>\n");
	if (ts_test->item[GTP_CAP_TEST]) {
		if (ts_test->result[GTP_CAP_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Rawdata MAX/MIN Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Rawdata MAX/MIN Test\" result=\"OK\"/>\n");
		}
		if (ts_test->result[GTP_DELTA_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Rawdata Adjcent Deviation Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Rawdata Adjcent Deviation Test\" result=\"OK\"/>\n");
		}
	}

	if (ts_test->item[GTP_NOISE_TEST]) {
		if (ts_test->result[GTP_NOISE_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Diffdata Jitter Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Diffdata Jitter Test\" result=\"OK\"/>\n");
		}
	}

	if (ts_test->item[GTP_SELFCAP_TEST]) {
		if (ts_test->result[GTP_SELFCAP_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Self Rawdata Upper Limit Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Self Rawdata Upper Limit Test\" result=\"OK\"/>\n");
		}
	}

	if (ts_test->item[GTP_SHORT_TEST]) {
		if (ts_test->result[GTP_SHORT_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Short Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Short Test\" result=\"OK\"/>\n");
		}
	}

	if (ts_test->item[GTP_STYLUS_RAW_TEST]) {
		if (ts_test->result[GTP_STYLUS_RAW_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Stylus Rawdata MAX/MIN Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Stylus Rawdata MAX/MIN Test\" result=\"OK\"/>\n");
		}
	}

	index += sprintf(&rbuf[index], "</ItemList>\n");
}

//static void goodix_save_limits(void)
//{
//	int tx = cd->ic_info.parm.drv_num;
//	int rx = cd->ic_info.parm.sen_num;
//	int i;
//	int chn1;
//	int chn2;
//	int r;
//
//	index += sprintf(&rbuf[index], "<TestItems>\n");
//
//	/* save short result */
//	if (ts_test->item[GTP_SHORT_TEST]) {
//		index += sprintf(&rbuf[index], "<Item name=\"Short Test\">\n");
//		index += sprintf(&rbuf[index], "<ShortNum>%d</ShortNum>\n",
//			ts_test->short_res.short_num);
//		for (i = 0; i < ts_test->short_res.short_num; i++) {
//			chn1 = ts_test->short_res.short_msg[4 * i];
//			chn2 = ts_test->short_res.short_msg[4 * i + 1];
//			r = (ts_test->short_res.short_msg[4 * i + 2] << 8) +
//			    ts_test->short_res.short_msg[4 * i + 3];
//			if (chn1 == CHN_VDD)
//				index += sprintf(&rbuf[index],
//					"<ShortMess Chn1=\"VDD\" ");
//			else if (chn1 == CHN_GND)
//				index += sprintf(&rbuf[index],
//					"<ShortMess Chn1=\"GND\" ");
//			else if (chn1 & DRV_CHANNEL_FLAG)
//				index += sprintf(&rbuf[index],
//					"<ShortMess Chn1=\"Tx%d\" ",
//					chn1 & 0x7f);
//			else
//				index += sprintf(&rbuf[index],
//					"<ShortMess Chn1=\"Rx%d\" ",
//					chn1 & 0x7f);
//			if (chn2 == CHN_VDD)
//				index += sprintf(&rbuf[index],
//					"Chn2=\"VDD\" ShortResistor= \"%dKom\"/>\n",
//					r);
//			else if (chn2 == CHN_GND)
//				index += sprintf(&rbuf[index],
//					"Chn2=\"GND\" ShortResistor= \"%dKom\"/>\n",
//					r);
//			else if (chn2 & DRV_CHANNEL_FLAG)
//				index += sprintf(&rbuf[index],
//					"Chn2=\"Tx%d\" ShortResistor= \"%dKom\"/>\n",
//					chn2 & 0x7f, r);
//			else
//				index += sprintf(&rbuf[index],
//					"Chn2=\"Rx%d\" ShortResistor= \"%dKom\"/>\n",
//					chn2 & 0x7f, r);
//		}
//		index += sprintf(&rbuf[index], "</Item>\n");
//	}
//
//	/* save open limits */
//	if (ts_test->item[GTP_CAP_TEST]) {
//		index += sprintf(
//			&rbuf[index], "<Item name=\"Rawdata Test Sets\">\n");
//		index += sprintf(&rbuf[index],
//			"<TotalFrameCnt>%d</TotalFrameCnt>\n", raw_data_cnt);
//		/* rawdata max limit */
//		index += sprintf(&rbuf[index], "<MaxRawLimit>\n");
//		for (i = 0; i < tx * rx; i++) {
//			index += sprintf(
//				&rbuf[index], "%d,", ts_test->max_limits[i]);
//			if ((i + 1) % tx == 0)
//				index += sprintf(&rbuf[index], "\n");
//		}
//		index += sprintf(&rbuf[index], "</MaxRawLimit>\n");
//		/* rawdata min limit */
//		index += sprintf(&rbuf[index], "<MinRawLimit>\n");
//		for (i = 0; i < tx * rx; i++) {
//			index += sprintf(
//				&rbuf[index], "%d,", ts_test->min_limits[i]);
//			if ((i + 1) % tx == 0)
//				index += sprintf(&rbuf[index], "\n");
//		}
//		index += sprintf(&rbuf[index], "</MinRawLimit>\n");
//		/* Max Accord limit */
//		index += sprintf(&rbuf[index], "<MaxAccordLimit>\n");
//		for (i = 0; i < tx * rx; i++) {
//			index += sprintf(&rbuf[index], "%d,",
//				ts_test->deviation_limits[i]);
//			if ((i + 1) % tx == 0)
//				index += sprintf(&rbuf[index], "\n");
//		}
//		index += sprintf(&rbuf[index], "</MaxAccordLimit>\n");
//		index += sprintf(&rbuf[index], "</Item>\n");
//	}
//
//	/* save noise limit */
//	if (ts_test->item[GTP_NOISE_TEST]) {
//		index += sprintf(
//			&rbuf[index], "<Item name=\"Diffdata Test Sets\">\n");
//		index += sprintf(&rbuf[index],
//			"<TotalFrameCnt>%d</TotalFrameCnt>\n", noise_data_cnt);
//		index += sprintf(&rbuf[index],
//			"<MaxJitterLimit>%d</MaxJitterLimit>\n",
//			ts_test->noise_threshold);
//		index += sprintf(&rbuf[index], "</Item>\n");
//	}
//
//	/* save self rawdata limit */
//	if (ts_test->item[GTP_SELFCAP_TEST]) {
//		index += sprintf(&rbuf[index],
//			"<Item name=\"Self Rawdata Test Sets\">\n");
//		index += sprintf(
//			&rbuf[index], "<TotalFrameCnt>1</TotalFrameCnt>\n");
//		index += sprintf(&rbuf[index], "<MaxRawLimit>\n");
//		for (i = 0; i < tx + rx; i++) {
//			index += sprintf(&rbuf[index], "%d,",
//				ts_test->self_max_limits[i]);
//			if ((i + 1) % tx == 0)
//				index += sprintf(&rbuf[index], "\n");
//		}
//		if ((tx + rx) % tx != 0)
//			index += sprintf(&rbuf[index], "\n");
//		index += sprintf(&rbuf[index], "</MaxRawLimit>\n");
//		index += sprintf(&rbuf[index], "<MinRawLimit>\n");
//		for (i = 0; i < tx + rx; i++) {
//			index += sprintf(&rbuf[index], "%d,",
//				ts_test->self_min_limits[i]);
//			if ((i + 1) % tx == 0)
//				index += sprintf(&rbuf[index], "\n");
//		}
//		if ((tx + rx) % tx != 0)
//			index += sprintf(&rbuf[index], "\n");
//		index += sprintf(&rbuf[index], "</MinRawLimit>\n");
//		index += sprintf(&rbuf[index], "</Item>\n");
//	}
//
//	index += sprintf(&rbuf[index], "</TestItems>\n");
//}

static void goodix_data_cal(s16 *data, size_t data_size, s16 *stat_result)
{
	int i = 0;
	s16 avg = 0;
	s16 min = 0;
	s16 max = 0;
	long long sum = 0;

	min = data[0];
	max = data[0];
	for (i = 0; i < data_size; i++) {
		sum += data[i];
		if (max < data[i])
			max = data[i];
		if (min > data[i])
			min = data[i];
	}
	avg = div_s64(sum, data_size);
	stat_result[0] = avg;
	stat_result[1] = max;
	stat_result[2] = min;
}

static void goodix_save_data(struct goodix_ts_core *cd)
{
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	s16 stat_result[3];
	int i, j;

	index += sprintf(&rbuf[index], "<DataRecord>\n");

	/* save rawdata */
	if (ts_test->item[GTP_CAP_TEST]) {
		index += sprintf(&rbuf[index], "<RawDataRecord>\n");
		for (i = 0; i < raw_data_cnt; i++) {
			goodix_data_cal(
				ts_test->rawdata[i].data, tx * rx, stat_result);
			index += sprintf(&rbuf[index],
				"<DataContent No.=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
				i, tx * rx, stat_result[1], stat_result[2],
				stat_result[0]);
			for (j = 0; j < tx * rx; j++) {
				index += sprintf(&rbuf[index], "%5d,",
					ts_test->rawdata[i].data[j]);
				if ((j + 1) % tx == 0)
					index += sprintf(&rbuf[index], "\n");
			}
			index += sprintf(&rbuf[index], "</DataContent>\n");
			goodix_data_cal(ts_test->deltadata[i].data, tx * rx,
				stat_result);
			index += sprintf(&rbuf[index],
				"<RawAccord No.=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
				i, tx * rx, stat_result[1], stat_result[2],
				stat_result[0]);
			for (j = 0; j < tx * rx; j++) {
				index += sprintf(&rbuf[index], "%5d,",
					ts_test->deltadata[i].data[j]);
				if ((j + 1) % tx == 0)
					index += sprintf(&rbuf[index], "\n");
			}
			index += sprintf(&rbuf[index], "</RawAccord>\n");
		}
		index += sprintf(&rbuf[index], "</RawDataRecord>\n");
	}

	/* save noisedata */
	if (ts_test->item[GTP_NOISE_TEST]) {
		index += sprintf(&rbuf[index], "<DiffDataRecord>\n");
		for (i = 0; i < noise_data_cnt; i++) {
			goodix_data_cal(ts_test->noisedata[i].data, tx * rx,
				stat_result);
			index += sprintf(&rbuf[index],
				"<DataContent No.=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
				i, tx * rx, stat_result[1], stat_result[2],
				stat_result[0]);
			for (j = 0; j < tx * rx; j++) {
				index += sprintf(&rbuf[index], "%5d,",
					ts_test->noisedata[i].data[j]);
				if ((j + 1) % tx == 0)
					index += sprintf(&rbuf[index], "\n");
			}
			index += sprintf(&rbuf[index], "</DataContent>\n");
		}
		index += sprintf(&rbuf[index], "</DiffDataRecord>\n");
	}

	/* save self rawdata */
	if (ts_test->item[GTP_SELFCAP_TEST]) {
		index += sprintf(&rbuf[index], "<selfDataRecord>\n");
		goodix_data_cal(
			ts_test->selfrawdata.data, tx + rx, stat_result);
		index += sprintf(&rbuf[index],
			"<DataContent No.=\"0\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
			tx + rx, stat_result[1], stat_result[2],
			stat_result[0]);
		for (i = 0; i < tx + rx; i++) {
			index += sprintf(&rbuf[index], "%5d,",
				ts_test->selfrawdata.data[i]);
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
		if ((tx + rx) % tx != 0)
			index += sprintf(&rbuf[index], "\n");
		index += sprintf(&rbuf[index], "</DataContent>\n");
		index += sprintf(&rbuf[index], "</selfDataRecord>\n");
	}

	/* save stylus rawdata */
	if (ts_test->item[GTP_STYLUS_RAW_TEST]) {
		index += sprintf(&rbuf[index], "<StylusRawDataRecord>\n");
		for (i = 0; i < raw_data_cnt; i++) {
			goodix_data_cal(ts_test->stylusraw[i].data, tx + rx, stat_result);
			index += sprintf(&rbuf[index],
			"<DataContent No.=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
			i, tx + rx, stat_result[1], stat_result[2], stat_result[0]);
			for (j = 0; j < tx + rx; j++) {
				index += sprintf(&rbuf[index], "%5d,",
					ts_test->stylusraw[i].data[j]);
				if ((j + 1) % tx == 0)
					index += sprintf(&rbuf[index], "\n");
			}
			if ((tx + rx) % tx != 0)
				index += sprintf(&rbuf[index], "\n");
			index += sprintf(&rbuf[index], "</DataContent>\n");
		}
		index += sprintf(&rbuf[index], "</StylusRawDataRecord>\n");
	}

	index += sprintf(&rbuf[index], "</DataRecord>\n");
}

static void goodix_save_tail(struct goodix_ts_core *cd)
{
	index += sprintf(&rbuf[index], "</TESTLOG>\n");
}

static void goodix_save_test_result(struct goodix_ts_core *cd, bool is_brief)
{
	if (is_brief) {
		if (ts_test->item[GTP_CAP_TEST]) {
			index += sprintf(&rbuf[index], "Open test:\n");
			index += sprintf(&rbuf[index], "%s\n",
				ts_test->result[GTP_CAP_TEST] ? "PASS"
							      : "FAIL");
		}
		if (ts_test->item[GTP_DELTA_TEST]) {
			index += sprintf(&rbuf[index], "Delta test:\n");
			index += sprintf(&rbuf[index], "%s\n",
				ts_test->result[GTP_DELTA_TEST] ? "PASS"
								: "FAIL");
		}
		if (ts_test->item[GTP_SHORT_TEST]) {
			index += sprintf(&rbuf[index], "Short test:\n");
			index += sprintf(&rbuf[index], "%s\n",
				ts_test->result[GTP_SHORT_TEST] ? "PASS"
								: "FAIL");
		}
		if (ts_test->item[GTP_NOISE_TEST]) {
			index += sprintf(&rbuf[index], "Noise test:\n");
			index += sprintf(&rbuf[index], "%s\n",
				ts_test->result[GTP_NOISE_TEST] ? "PASS"
								: "FAIL");
		}
		if (ts_test->item[GTP_SELFCAP_TEST]) {
			index += sprintf(&rbuf[index], "Self test:\n");
			index += sprintf(&rbuf[index], "%s\n",
				ts_test->result[GTP_SELFCAP_TEST] ? "PASS"
								  : "FAIL");
		}
	} else {
		goodix_save_header(cd);
		goodix_save_data(cd);
		goodix_save_tail(cd);
	}
}

static void goto_next_line(char **ptr)
{
	do {
		*ptr = *ptr + 1;
	} while (**ptr != '\n' && **ptr != '\0');
	if (**ptr == '\0') {
		return;
	}
	*ptr = *ptr + 1;
}

static void copy_this_line(char *dest, char *src)
{
	char *copy_from;
	char *copy_to;

	copy_from = src;
	copy_to = dest;
	do {
		*copy_to = *copy_from;
		copy_from++;
		copy_to++;
	} while ((*copy_from != '\n') && (*copy_from != '\r') &&
		 (*copy_from != '\0'));
	*copy_to = '\0';
}

static int getrid_space(s8 *data, s32 len)
{
	u8 *buf = NULL;
	s32 i;
	u32 count = 0;

	buf = (char *)kzalloc(len + 5, GFP_KERNEL);
	if (buf == NULL) {
		ts_err("get space kzalloc error");
		return -ESRCH;
	}

	for (i = 0; i < len; i++) {
		if (data[i] == ' ' || data[i] == '\r' || data[i] == '\n') {
			continue;
		}
		buf[count++] = data[i];
	}

	buf[count++] = '\0';

	memcpy(data, buf, count);
	kfree(buf);

	return count;
}

static int parse_valid_data(
	char *buf_start, loff_t buf_size, char *ptr, s16 *data, s32 rows)
{
	int i = 0;
	int j = 0;
	char *token = NULL;
	char *tok_ptr = NULL;
	char *row_data = NULL;
	long temp_val;

	if (!ptr || !data)
		return -EINVAL;

	row_data = (char *)kzalloc(MAX_LINE_LEN, GFP_KERNEL);
	if (!row_data) {
		ts_err("alloc index %d failed.", MAX_LINE_LEN);
		return -ENOMEM;
	}

	for (i = 0; i < rows; i++) {
		memset(row_data, 0, MAX_LINE_LEN);
		copy_this_line(row_data, ptr);
		getrid_space(row_data, strlen(row_data));
		tok_ptr = row_data;
		while ((token = strsep(&tok_ptr, ","))) {
			if (strlen(token) == 0)
				continue;
			if (kstrtol(token, 0, &temp_val)) {
				kfree(row_data);
				return -EINVAL;
			}
			data[j++] = (s16)temp_val;
		}
		if (i == rows - 1)
			break;
		goto_next_line(&ptr); // next row
		if (!ptr || (0 == strlen(ptr)) ||
			(ptr >= (buf_start + buf_size))) {
			ts_info("invalid ptr, return");
			kfree(row_data);
			row_data = NULL;
			return -EPERM;
		}
	}
	kfree(row_data);
	return j;
}

static int parse_csvfile(
	char *buf, size_t size, char *target_name, s16 *data, s32 rows, s32 col)
{
	char *ptr = NULL;

	if (!data || !buf)
		return -EIO;

	ptr = buf;
	ptr = strstr(ptr, target_name);
	if (!ptr) {
		ts_info("load %s failed 1, maybe not this item", target_name);
		return -EINTR;
	}

	goto_next_line(&ptr);
	if (!ptr || (0 == strlen(ptr))) {
		ts_err("load %s failed 2!", target_name);
		return -EIO;
	}

	return parse_valid_data(buf, size, ptr, data, rows);
}

static int goodix_obtain_testlimits(struct goodix_ts_core *cd)
{
	const struct firmware *firmware = NULL;
	struct device *dev = &cd->pdev->dev;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	char *limit_file = cd->board_data.test_limits_name;
	char *temp_buf = NULL;
	char *raw_limit_min = CSV_TP_SPECIAL_RAW_MIN;
	char *raw_limit_max = CSV_TP_SPECIAL_RAW_MAX;
	char *short_threshold = CSV_TP_SHORT_THRESHOLD;
	s16 data_buf[7];
	int ret;

	ts_info("limit_file_name:%s", limit_file);

	ret = request_firmware(&firmware, limit_file, dev);
	if (ret < 0) {
		ts_err("limits file [%s] not available", limit_file);
		return -EINVAL;
	}
	if (firmware->size <= 0) {
		ts_err("request_firmware, limits param length error,len:%zu",
			firmware->size);
		ret = -EINVAL;
		goto exit_free;
	}
	temp_buf = kzalloc(firmware->size + 1, GFP_KERNEL);
	if (!temp_buf) {
		ret = -ENOMEM;
		goto exit_free;
	}
	memcpy(temp_buf, firmware->data, firmware->size);

	if (ts_test->item[GTP_CAP_TEST]) {
		if (ts_test->freq > 0) {
			raw_limit_min = CSV_TP_SPECIAL_FREQ_RAW_MIN;
			raw_limit_max = CSV_IP_SPECIAL_FREQ_RAW_MAX;
		} else if (ts_test->is_ical) {
			raw_limit_min = CSV_TP_ICAL_RAW_MIN;
			raw_limit_max = CSV_TP_ICAL_RAW_MAX;
		}

		/* obtain mutual_raw min */
		ret = parse_csvfile(temp_buf, firmware->size, raw_limit_min,
			ts_test->min_limits, rx, tx);
		if (ret < 0) {
			ts_err("Failed get min_limits");
			goto exit_free;
		}
		/* obtain mutual_raw max */
		ret = parse_csvfile(temp_buf, firmware->size, raw_limit_max,
			ts_test->max_limits, rx, tx);
		if (ret < 0) {
			ts_err("Failed get max_limits");
			goto exit_free;
		}
		/* obtain delta limit */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SPECIAL_RAW_DELTA, ts_test->deviation_limits, rx,
			tx);
		if (ret < 0) {
			ts_err("Failed get delta limit");
			goto exit_free;
		}
	}

	if (ts_test->item[GTP_SELFCAP_TEST]) {
		/* obtain self_raw min */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SPECIAL_SELFRAW_MIN, ts_test->self_min_limits, 1,
			tx + rx);
		if (ret < 0) {
			ts_err("Failed get self_min_limits");
			goto exit_free;
		}
		/* obtain self_raw max */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SPECIAL_SELFRAW_MAX, ts_test->self_max_limits, 1,
			tx + rx);
		if (ret < 0) {
			ts_err("Failed get self_min_limits");
			goto exit_free;
		}
	}

	if (ts_test->item[GTP_NOISE_TEST]) {
		/* obtain noise_threshold */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_NOISE_LIMIT, &ts_test->noise_threshold, 1, 1);
		if (ret < 0) {
			ts_err("Failed get noise limits");
			goto exit_free;
		}
	}

	if (ts_test->item[GTP_SHORT_TEST]) {
		if (ts_test->is_ical)
			short_threshold = CSV_TP_ICAL_SHORT_THRESHOLD;

		/* obtain short_params */
		ret = parse_csvfile(temp_buf, firmware->size,
			short_threshold, data_buf, 1, 7);
		if (ret < 0) {
			ts_err("Failed get short circuit limits");
			goto exit_free;
		}
		ts_test->short_threshold = data_buf[0];
		ts_test->r_drv_drv_threshold = data_buf[1];
		ts_test->r_drv_sen_threshold = data_buf[2];
		ts_test->r_sen_sen_threshold = data_buf[3];
		ts_test->r_drv_gnd_threshold = data_buf[4];
		ts_test->r_sen_gnd_threshold = data_buf[5];
		ts_test->avdd_value = data_buf[6];

		if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
			ts_test->params_info = &params_brb;
		else if (cd->bus->ic_type == IC_TYPE_BERLIN_D)
			ts_test->params_info = &params_brd;
		else if (cd->bus->ic_type == IC_TYPE_NOTTINGHAM)
			ts_test->params_info = &params_not;
	}

	if (ts_test->item[GTP_STYLUS_RAW_TEST]) {
		if (ts_test->stylus_test_freq > 0) {
			raw_limit_min = CSV_TP_SPECIAL_FREQ_STYLUSRAW_MIN;
			raw_limit_max = CSV_TP_SPECIAL_FREQ_STYLUSRAW_MAX;
		} else {
			raw_limit_min = CSV_TP_SPECIAL_STYLUSRAW_MIN;
			raw_limit_max = CSV_TP_SPECIAL_STYLUSRAW_MAX;
		}
		ret = parse_csvfile(temp_buf, firmware->size, raw_limit_min,
			ts_test->stylus_min_limits, 1, tx + rx);
		if (ret < 0) {
			ts_err("Failed get %s", raw_limit_min);
			goto exit_free;
		}
		ret = parse_csvfile(temp_buf, firmware->size, raw_limit_max,
			ts_test->stylus_max_limits, 1, tx + rx);
		if (ret < 0) {
			ts_err("Failed get %s", raw_limit_max);
			goto exit_free;
		}
	}

exit_free:
	kfree(temp_buf);
	if (firmware)
		release_firmware(firmware);
	return ret;
}

static int goodix_delta_test(struct goodix_ts_core *cd)
{
	int i, j;
	int max_val;
	int raw;
	int temp;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 data_size = tx * rx;
	int ret = 0;

	for (i = 0; i < raw_data_cnt; i++) {
		for (j = 0; j < data_size; j++) {
			raw = ts_test->rawdata[i].data[j];
			max_val = 0;
			/* calcu delta with above node */
			if (j - tx >= 0) {
				temp = ts_test->rawdata[i].data[j - tx];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with bellow node */
			if (j + tx < data_size) {
				temp = ts_test->rawdata[i].data[j + tx];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with left node */
			if (j % tx) {
				temp = ts_test->rawdata[i].data[j - 1];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with right node */
			if ((j + 1) % tx) {
				temp = ts_test->rawdata[i].data[j + 1];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			temp = max_val * 1000 / raw;
			ts_test->deltadata[i].data[j] = temp;
			if (temp > ts_test->deviation_limits[j]) {
				// ts_err("delta_data[%d] > limits[%d]",
				//         temp, ts_test->deviation_limits[j]);
				ret = -EINVAL;
			}
		}
	}

	return ret;
}

static void print_open_test_ng_data(
	struct goodix_ts_core *cd, int index, s16 *data)
{
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	char *output_buf;
	int buf_size = (MAX(tx, rx) + 1) * 6;
	u32 self_raw_addr;
	s16 self_raw_data[MAX_DRV_NUM + MAX_SEN_NUM];
	int i, j;
	int cnt = 0;

	self_raw_addr = cd->ic_info.misc.frame_data_addr +
			cd->ic_info.misc.frame_data_head_len +
			cd->ic_info.misc.fw_attr_len +
			cd->ic_info.misc.fw_log_len +
			cd->ic_info.misc.mutual_struct_len + 10;

	output_buf = malloc_proc_buffer(buf_size);
	if (output_buf == NULL) {
		ts_err("failed to alloc output buffer");
		return;
	}

	ts_info("NG Rawdata[%d]:", index);
	for (i = 0; i < rx; i++) {
		cnt = 0;
		memset(output_buf, 0, buf_size);
		for (j = 0; j < tx; j++)
			cnt += scnprintf(
				&output_buf[cnt], buf_size - cnt, "%5d,", data[i * tx + j]);
		ts_info("%s", output_buf);
	}

	cd->hw_ops->read(cd, self_raw_addr, (unsigned char *)self_raw_data,
		(tx + rx) * 2);
	ts_info("self Tx Raw:");
	cnt = 0;
	memset(output_buf, 0, buf_size);
	for (i = 0; i < tx; i++)
		cnt += scnprintf(&output_buf[cnt], buf_size - cnt, "%d,",
			self_raw_data[i]);
	ts_info("%s", output_buf);
	ts_info("self Rx Raw:");
	cnt = 0;
	memset(output_buf, 0, buf_size);
	for (i = tx; i < tx + rx; i++)
		cnt += scnprintf(&output_buf[cnt], buf_size - cnt, "%d,",
			self_raw_data[i]);
	ts_info("%s", output_buf);

	free_proc_buffer(&output_buf);
}

static void print_self_compensation(struct goodix_ts_core *cd)
{
	u8 *cfg;
	u8 *cfg_buf;
	int len;
	int cfg_num;
	int sub_cfg_index;
	int sub_cfg_len;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	char *output_buf;
	int buf_size = (MAX(tx, rx) + 1) * 6;
	int cnt;
	s16 val;
	int i, j;

	cfg_buf = kzalloc(GOODIX_CFG_MAX_SIZE, GFP_KERNEL);
	if (cfg_buf == NULL) {
		ts_err("failed to alloc cfg buffer");
		return;
	}

	output_buf = malloc_proc_buffer(buf_size);
	if (output_buf == NULL) {
		ts_err("failed to alloc output buffer");
		goto exit_free_cfg_buf;
	}

	if (cd->bus->ic_type == IC_TYPE_BERLIN_B) {
		cd->hw_ops->read(cd, cd->ic_info.misc.auto_scan_cmd_addr,
			cfg_buf, (tx + rx) * 2);
		ts_info("self Tx compensation:");
		cnt = 0;
		memset(output_buf, 0, buf_size);
		for (i = 0; i < tx; i++) {
			val = le16_to_cpup((__le16 *)&cfg_buf[i * 2]);
			cnt += scnprintf(&output_buf[cnt], buf_size - cnt, "%d,", val);
		}
		ts_info("%s", output_buf);
		cnt = 0;
		memset(output_buf, 0, buf_size);
		ts_info("self Tx compensation:");
		for (i = 0; i < rx; i++) {
			val = le16_to_cpup((__le16 *)&cfg_buf[tx * 2 + i * 2]);
			cnt += scnprintf(&output_buf[cnt], buf_size - cnt, "%d,", val);
		}
		ts_info("%s", output_buf);
		goto exit_free;
	}

	len = cd->hw_ops->read_config(cd, cfg_buf, GOODIX_CFG_MAX_SIZE);
	if (len < 0) {
		ts_err("read config failed");
		goto exit_free;
	}

	cfg = cfg_buf;
	cfg_num = cfg[61];
	cfg += 64;
	for (i = 0; i < cfg_num; i++) {
		sub_cfg_len = cfg[0] - 2;
		sub_cfg_index = cfg[1];
		if (sub_cfg_index == 13) { // TX self cancel
			ts_info("self Tx compensation:");
			cnt = 0;
			memset(output_buf, 0, buf_size);
			for (j = 0; j < tx; j++) {
				val = le16_to_cpup((__le16 *)&cfg[2 + j * 4]) +
				      le16_to_cpup((__le16 *)&cfg[4 + j * 4]);
				cnt += scnprintf(&output_buf[cnt], buf_size - cnt, "%d,", val);
			}
			ts_info("%s", output_buf);
		} else if (sub_cfg_index == 14) { // RX self cancel
			cnt = 0;
			memset(output_buf, 0, buf_size);
			ts_info("self Tx compensation:");
			for (j = 0; j < rx; j++) {
				val = le16_to_cpup((__le16 *)&cfg[2 + j * 2]);
				cnt += scnprintf(&output_buf[cnt], buf_size - cnt, "%d,", val);
			}
			ts_info("%s", output_buf);
		}
		cfg += (sub_cfg_len + 2);
	}

exit_free:
	free_proc_buffer(&output_buf);
exit_free_cfg_buf:
	kfree(cfg_buf);
}

static int goodix_open_test(struct goodix_ts_core *cd)
{
	u8 *tmp_buf;
	struct goodix_ts_cmd temp_cmd = {0};
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 raw_addr;
	int ret;
	int err_cnt = 0;
	int i, j;
	s16 tmp_val;
	u16 tmp_freq;
	u8 val;
	int retry;
	bool is_ng;

	raw_addr = cd->ic_info.misc.frame_data_addr +
		   cd->ic_info.misc.frame_data_head_len +
		   cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		   8;

	tmp_buf = kzalloc(GOODIX_MAX_FRAMEDATA_LEN, GFP_KERNEL);
	if (tmp_buf == NULL)
		return -ENOMEM;

	/* open test prepare */
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x84;
	temp_cmd.len = 5;
	if (cd->bus->sub_ic_type == IC_TYPE_SUB_GT7986) {
		raw_addr = cd->ic_info.misc.mutual_rawdata_addr;
		sync_addr = cd->ic_info.misc.touch_data_addr;
		temp_cmd.cmd = 0x01;
		temp_cmd.len = 4;
	}

	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("send rawdata cmd failed");
		goto exit;
	}

	/* switch freq */
	if (ts_test->freq > 0) {
		ts_info("set freq %d", ts_test->freq);
		tmp_freq = ts_test->freq / 61;
		temp_cmd.len = 6;
		temp_cmd.cmd = 0xB1;
		temp_cmd.data[0] = tmp_freq & 0xFF;
		temp_cmd.data[1] = (tmp_freq >> 8) & 0xFF;
		ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
		if (ret < 0) {
			ts_err("set freq %d failed", ts_test->freq);
			goto exit;
		}
	}

	/* discard the first few frames */
	for (i = 0; i < DISCARD_FRAME_NUMS; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		usleep_range(20000, 21000);
	}

	/* read rawdata */
	for (i = 0; i < raw_data_cnt; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		retry = 20;
		while (retry--) {
			usleep_range(5000, 5100);
			cd->hw_ops->read(cd, sync_addr, &val, 1);
			if (val & 0x80)
				break;
		}
		if (retry < 0) {
			ts_err("rawdata is not ready val:0x%02x i:%d, exit",
				val, i);
			ret = -EAGAIN;
			goto exit;
		}

		cd->hw_ops->read(cd, raw_addr, tmp_buf, tx * rx * 2);
		goodix_rotate_abcd2cbad(tx, rx, (s16 *)tmp_buf, NULL);
		memcpy((u8 *)ts_test->rawdata[i].data, tmp_buf, tx * rx * 2);

		is_ng = false;
		for (j = 0; j < tx * rx; j++) {
			tmp_val = ts_test->rawdata[i].data[j];
			if (tmp_val > ts_test->max_limits[j] ||
				tmp_val < ts_test->min_limits[j]) {
				err_cnt++;
				is_ng = true;
			}
		}
		if (is_ng)
			print_open_test_ng_data(
				cd, i, ts_test->rawdata[i].data);
	}

	if (err_cnt == 0) {
		ts_test->result[GTP_CAP_TEST] = TEST_OK;
	} else {
		ret = -EINVAL;
		ts_test->result[GTP_CAP_TEST] = TEST_NG;
		print_self_compensation(cd);
	}

	if (goodix_delta_test(cd) == 0)
		ts_test->result[GTP_DELTA_TEST] = TEST_OK;

exit:
	kfree(tmp_buf);
	return ret;
}

#define SELF_RAWDATA_NG_RETRY 3
static int goodix_self_open_test(struct goodix_ts_core *cd)
{
	u8 *tmp_buf;
	struct goodix_ts_cmd temp_cmd = {0};
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 raw_addr;
	int ret;
	int j;
	s16 tmp_val;
	u8 val;
	int retry;
	int ng_rty = SELF_RAWDATA_NG_RETRY;

	raw_addr = cd->ic_info.misc.frame_data_addr +
		   cd->ic_info.misc.frame_data_head_len +
		   cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		   cd->ic_info.misc.mutual_struct_len + 10;

	tmp_buf = kzalloc(GOODIX_MAX_FRAMEDATA_LEN, GFP_KERNEL);
	if (tmp_buf == NULL)
		return -ENOMEM;

restart:
	/* test prepare */
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x84;
	temp_cmd.len = 5;
	if (cd->bus->sub_ic_type == IC_TYPE_SUB_GT7986) {
		raw_addr = cd->ic_info.misc.self_rawdata_addr;
		sync_addr = cd->ic_info.misc.touch_data_addr;
		temp_cmd.cmd = 0x01;
		temp_cmd.len = 4;
	}

	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("send rawdata cmd failed");
		goto exit;
	}

	/* discard the first few frames */
	for (j = 0; j < DISCARD_FRAME_NUMS; j++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		usleep_range(20000, 21000);
	}

	/* read self rawdata */
	val = 0;
	cd->hw_ops->write(cd, sync_addr, &val, 1);
	retry = 20;
	while (retry--) {
		usleep_range(5000, 5100);
		cd->hw_ops->read(cd, sync_addr, &val, 1);
		if (val & 0x80)
			break;
	}
	if (retry < 0) {
		ts_err("self rawdata is not ready val:0x%02x, exit", val);
		ret = -EAGAIN;
		goto exit;
	}

	cd->hw_ops->read(cd, raw_addr, tmp_buf, (tx + rx) * 2);
	memcpy((u8 *)ts_test->selfrawdata.data, tmp_buf, (tx + rx) * 2);

	/* analysis results */
	ts_test->result[GTP_SELFCAP_TEST] = TEST_OK;
	for (j = 0; j < tx + rx; j++) {
		tmp_val = ts_test->selfrawdata.data[j];
		if (tmp_val > ts_test->self_max_limits[j] ||
			tmp_val < ts_test->self_min_limits[j]) {
			// ts_err("self_rawdata[%d] out of range[%d %d]",
			//         tmp_val, ts_test->self_min_limits[j],
			//         ts_test->self_max_limits[j]);
			ts_test->result[GTP_SELFCAP_TEST] = TEST_NG;
			ret = -EINVAL;
		}
	}

	if (ts_test->result[GTP_SELFCAP_TEST] == TEST_NG && ng_rty > 0) {
		ng_rty--;
		cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
		ts_info("self rawdata out of limit, retry:%d",
			SELF_RAWDATA_NG_RETRY - ng_rty);
		goto restart;
	}

exit:
	kfree(tmp_buf);
	return ret;
}

static int goodix_noise_test(struct goodix_ts_core *cd)
{
	u8 *tmp_buf = NULL;
	struct goodix_ts_cmd temp_cmd = {0};
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 raw_addr;
	int ret = 0;
	int i, j;
	s16 tmp_val;
	u8 val;
	int retry;

	raw_addr = cd->ic_info.misc.frame_data_addr +
		   cd->ic_info.misc.frame_data_head_len +
		   cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		   8;

	tmp_buf = kzalloc(GOODIX_MAX_FRAMEDATA_LEN, GFP_KERNEL);
	if (tmp_buf == NULL)
		return -ENOMEM;

	/* open test prepare */
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x82;
	temp_cmd.len = 5;
	if (cd->bus->sub_ic_type == IC_TYPE_SUB_GT7986) {
		raw_addr = cd->ic_info.misc.mutual_diffdata_addr;
		sync_addr = cd->ic_info.misc.touch_data_addr;
		temp_cmd.cmd = 0x01;
		temp_cmd.len = 4;
	}
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);

	if (ret < 0) {
		ts_err("send rawdata cmd failed");
		goto exit;
	}

	/* discard the first few frames */
	for (i = 0; i < DISCARD_FRAME_NUMS; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		usleep_range(20000, 21000);
	}

	/* read noisedata */
	for (i = 0; i < noise_data_cnt; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		retry = 20;
		while (retry--) {
			usleep_range(5000, 5100);
			cd->hw_ops->read(cd, sync_addr, &val, 1);
			if (val & 0x80)
				break;
		}
		if (retry < 0) {
			ts_err("noisedata is not ready val:0x%02x i:%d, exit",
				val, i);
			ret = -EAGAIN;
			goto exit;
		}

		cd->hw_ops->read(cd, raw_addr, tmp_buf, tx * rx * 2);
		goodix_rotate_abcd2cbad(tx, rx, (s16 *)tmp_buf, NULL);
		memcpy((u8 *)ts_test->noisedata[i].data, tmp_buf, tx * rx * 2);
	}

	/* analysis results */
	ts_test->result[GTP_NOISE_TEST] = TEST_OK;
	for (i = 0; i < noise_data_cnt; i++) {
		for (j = 0; j < tx * rx; j++) {
			tmp_val = ts_test->noisedata[i].data[j];
			tmp_val = ABS(tmp_val);
			if (tmp_val > ts_test->noise_threshold) {
				// ts_err("noise data[%d] > noise
				// threshold[%d]",
				//         tmp_val, ts_test->noise_threshold);
				ts_test->result[GTP_NOISE_TEST] = TEST_NG;
				ret = -EINVAL;
			}
		}
	}

exit:
	kfree(tmp_buf);
	return ret;
}

static int goodix_stylus_rawdata_test(struct goodix_ts_core *cd)
{
	struct goodix_ts_cmd temp_cmd = {0};
	int tmp_freq = ts_test->stylus_test_freq / 61;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	struct goodix_frame_head frame_head;
	u32 raw_addr;
	int retry;
	int ret;
	int i, j;
	int tmp_val;
	int err_cnt = 0;
	u8 val;

	raw_data_cnt = 16;
	rbuf = malloc_proc_buffer(HUGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		ret = -ENOMEM;
		goto exit;
	}

	raw_addr = cd->ic_info.misc.frame_data_addr +
		cd->ic_info.misc.frame_data_head_len +
		cd->ic_info.misc.fw_attr_len +
		cd->ic_info.misc.fw_log_len + 28;

	temp_cmd.len = 7;
	temp_cmd.cmd = 0x65;
	temp_cmd.data[0] = 1;
	temp_cmd.data[1] = tmp_freq & 0xFF;
	temp_cmd.data[2] = (tmp_freq >> 8) & 0xFF;
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("send stylus test cmd failed");
		goto exit;
	}
	temp_cmd.len = 5;
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x81;
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("send rawdata cmd failed");
		goto exit;
	}

	/* discard the first few frames */
	for (i = 0; i < DISCARD_FRAME_NUMS; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		usleep_range(20000, 21000);
	}

	for (i = 0; i < raw_data_cnt; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		retry = 20;
		while (retry--) {
			usleep_range(5000, 5100);
			cd->hw_ops->read(cd, sync_addr, (u8 *)&frame_head, sizeof(frame_head));
			if (frame_head.sync & 0x80)
				break;
		}
		if (retry < 0) {
			ts_err("rawdata is not ready val:0x%02x i:%d, exit", frame_head.sync, i);
			ret = -EINVAL;
			goto exit;
		}
		if (!(frame_head.data_en & STYLUS_PACK_EN)) {
			ts_err("frame has no stylus pack data");
			ret = -EINVAL;
			goto exit;
		}

		cd->hw_ops->read(cd, raw_addr, (u8 *)ts_test->stylusraw[i].data, (tx + rx) * 2);
	}

	/* analysis results */
	for (i = 0; i < raw_data_cnt; i++) {
		for (j = 0; j < tx + rx; j++) {
			tmp_val = ts_test->stylusraw[i].data[j];
			if (tmp_val > ts_test->stylus_max_limits[j] ||
				tmp_val < ts_test->stylus_min_limits[j]) {
				ts_err("stylusraw[%d] out of range[%d %d]",
					tmp_val, ts_test->stylus_min_limits[j],
					ts_test->stylus_max_limits[j]);
				err_cnt++;
			}
		}
	}

	if (err_cnt > 0) {
		ret = -EINVAL;
	} else {
		ret = 0;
		ts_test->result[GTP_STYLUS_RAW_TEST] = TEST_OK;
	}

exit:
	return ret;
}

#pragma pack(1)
struct clk_test_parm {
	union {
		struct {
			u8 gio;
			u8 div:2;           /* 0:no div 1:8 div */
			u8 is_64m:1;       /* 0: not 64M osc 1:64M osc */
			u8 en:1;
			u8 pll_prediv:2;
			u8 pll_fbdiv:2;
			u8 trigger_mode;    /* 0:rising 1:high 2:falling 3:low */
			u16 clk_in_num;     /* collect clk num (1-1022) */
			u16 checksum;
		};
		u8 buf[7];
	};
};
#pragma pack()

static int goodix_auto_test(struct goodix_ts_core *cd, bool is_brief)
{
	struct goodix_ts_cmd temp_cmd = {0};
	int ret;
	int i;

	ret = goodix_obtain_testlimits(cd);
	if (ret < 0) {
		ts_err("obtain open test limits failed");
		return ret;
	}

	cd->hw_ops->irq_enable(cd, false);
	goodix_ts_esd_off(cd);

	temp_cmd.len = 5;
	temp_cmd.cmd = 0x64;
	temp_cmd.data[0] = 1;

	if (ts_test->item[GTP_CAP_TEST]) {
		for (i = 0; i < 3; i++) {
			cd->hw_ops->send_cmd(cd, &temp_cmd);
			ret = goodix_open_test(cd);
			cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
			if (ret != -EAGAIN)
				break;
		}
	}

	if (ts_test->item[GTP_NOISE_TEST]) {
		for (i = 0; i < 3; i++) {
			if (!cd->board_data.noise_test_disable_cmd)
				cd->hw_ops->send_cmd(cd, &temp_cmd);
			ret = goodix_noise_test(cd);
			cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
			if (ret != -EAGAIN)
				break;
		}
	}

	if (ts_test->item[GTP_SELFCAP_TEST]) {
		for (i = 0; i < 3; i++) {
			ret = goodix_self_open_test(cd);
			cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
			if (ret != -EAGAIN)
				break;
		}
	}

	if (ts_test->item[GTP_SHORT_TEST]) {
		goodix_shortcircut_test(cd);
		cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
	}

	if (ts_test->item[GTP_STYLUS_RAW_TEST]) {
		ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
		if (ret < 0)
			ts_err("enter test mode failed");
		goodix_stylus_rawdata_test(cd);
		cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
	}

	cd->hw_ops->irq_enable(cd, true);
	goodix_ts_esd_on(cd);
	goodix_save_test_result(cd, is_brief);
	return 0;
}

static void goodix_auto_noise_test(struct goodix_ts_core *cd, u16 cnt, int threshold)
{
	struct goodix_ts_cmd temp_cmd = {0};
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 raw_addr;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	s16 *tmp_buf;
	int tmp_val;
	u8 status;
	int test_try = 2;
	int ret = 0;
	int retry = 20;
	int err_cnt = 0;
	int i;

	tmp_buf = kcalloc(MAX_DRV_NUM * MAX_SEN_NUM, 2, GFP_KERNEL);
	if (tmp_buf == NULL)
		return;

	raw_addr = cd->ic_info.misc.frame_data_addr +
		   cd->ic_info.misc.frame_data_head_len +
		   cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		   8;

	cd->hw_ops->irq_enable(cd, false);
	goodix_ts_esd_off(cd);

restart:
	temp_cmd.len = 0x07;
	temp_cmd.cmd = 0x90;
	if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
		temp_cmd.data[0] = 0x87;
	else
		temp_cmd.data[0] = 0x86;
	temp_cmd.data[1] = cnt & 0xFF;
	temp_cmd.data[2] = (cnt >> 8) & 0xFF;
	cd->hw_ops->send_cmd(cd, &temp_cmd);

	msleep(cnt * 8);

	while (retry--) {
		cd->hw_ops->read(cd, sync_addr, &status, 1);
		if (status & 0x80)
			break;
		usleep_range(5000, 5100);
	}
	if (retry < 0) {
		ts_err("noise data not ready, status[%x]", status);
		ret = -EINVAL;
		goto exit;
	}

	cd->hw_ops->read(cd, raw_addr, (u8 *)tmp_buf, tx * rx * 2);
	goodix_rotate_abcd2cbad(tx, rx, tmp_buf, NULL);
	index += sprintf(&rbuf[index], "max:\n");
	for (i = 0; i < tx * rx; i++) {
		tmp_val = tmp_buf[i];
		index += sprintf(&rbuf[index], "%3d,", tmp_val);
		if ((i + 1) % tx == 0)
			index += sprintf(&rbuf[index], "\n");
		if (ABS(tmp_val) > threshold)
			err_cnt++;
	}

	status = 0;
	cd->hw_ops->write(cd, sync_addr, &status, 1);
	retry = 10;
	while (retry--) {
		cd->hw_ops->read(cd, sync_addr, &status, 1);
		if (status & 0x80)
			break;
		usleep_range(5000, 5100);
	}
	if (retry < 0) {
		ts_err("noise data not ready, status[%x]", status);
		ret = -EINVAL;
		goto exit;
	}
	cd->hw_ops->read(cd, raw_addr, (u8 *)tmp_buf, tx * rx * 2);
	goodix_rotate_abcd2cbad(tx, rx, tmp_buf, NULL);
	index += sprintf(&rbuf[index], "min:\n");
	for (i = 0; i < tx * rx; i++) {
		tmp_val = tmp_buf[i];
		index += sprintf(&rbuf[index], "%3d,", tmp_val);
		if ((i + 1) % tx == 0)
			index += sprintf(&rbuf[index], "\n");
		if (ABS(tmp_val) > threshold)
			err_cnt++;
	}

	if (err_cnt > 0)
		index += sprintf(&rbuf[index], "Result: FAIL\n");
	else
		index += sprintf(&rbuf[index], "Result: PASS\n");

exit:
	cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
	if (ret < 0 && --test_try > 0) {
		ts_err("auto noise running failed, retry:%d", test_try);
		ret = 0;
		index = 0;
		goto restart;
	}
	kfree(tmp_buf);
	cd->hw_ops->irq_enable(cd, true);
	goodix_ts_esd_on(cd);
}

static int get_cap_data(struct goodix_ts_core *cd, uint8_t *type)
{
	struct goodix_ts_cmd temp_cmd = {0};
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u8 val;
	int retry = 40;
	u8 *frame_buf;
	u32 flag_addr = cd->ic_info.misc.frame_data_addr;
	u32 mutual_addr;
	u32 self_addr;
	u32 tx_freq_addr;
	int i;
	int ret;

	mutual_addr = cd->ic_info.misc.frame_data_addr +
		      cd->ic_info.misc.frame_data_head_len +
		      cd->ic_info.misc.fw_attr_len +
		      cd->ic_info.misc.fw_log_len + 8;
	self_addr = cd->ic_info.misc.frame_data_addr +
		    cd->ic_info.misc.frame_data_head_len +
		    cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		    cd->ic_info.misc.mutual_struct_len + 10;
	tx_freq_addr = cd->ic_info.misc.frame_data_addr +
		       cd->ic_info.misc.frame_data_head_len +
		       cd->ic_info.misc.fw_attr_len +
		       cd->ic_info.misc.fw_log_len + 2;

	frame_buf = kzalloc(GOODIX_MAX_FRAMEDATA_LEN, GFP_KERNEL);
	if (frame_buf == NULL)
		return -ENOMEM;

	/* disable irq & close esd */
	cd->hw_ops->irq_enable(cd, false);
	goodix_ts_esd_off(cd);

	if (strstr(type, CMD_GET_BASEDATA) ||
		strstr(type, CMD_GET_SELF_BASEDATA)) {
		temp_cmd.data[0] = 0x83;
	} else if (strstr(type, CMD_GET_RAWDATA) ||
		   strstr(type, CMD_GET_SELF_RAWDATA) ||
		   strstr(type, CMD_GET_TX_FREQ)) {
		temp_cmd.data[0] = 0x81;
	} else {
		temp_cmd.data[0] = 0x82;
	}

	temp_cmd.cmd = 0x90;
	temp_cmd.len = 5;

	if (cd->bus->sub_ic_type == IC_TYPE_SUB_GT7986) {
		if (strstr(type, CMD_GET_BASEDATA))
			mutual_addr = cd->ic_info.misc.mutual_refdata_addr;
		else if (strstr(type, CMD_GET_RAWDATA))
			mutual_addr = cd->ic_info.misc.mutual_rawdata_addr;
		else if (strstr(type, CMD_GET_DIFFDATA))
			mutual_addr = cd->ic_info.misc.mutual_diffdata_addr;
		else if (strstr(type, CMD_GET_SELF_BASEDATA))
			self_addr = cd->ic_info.misc.self_refdata_addr;
		else if (strstr(type, CMD_GET_SELF_RAWDATA))
			self_addr = cd->ic_info.misc.self_rawdata_addr;
		else if (strstr(type, CMD_GET_SELF_DIFFDATA))
			self_addr = cd->ic_info.misc.self_diffdata_addr;

		flag_addr = cd->ic_info.misc.touch_data_addr;
		temp_cmd.cmd = 0x01;
		temp_cmd.len = 4;
	}

	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("report rawdata failed, exit!");
		goto exit;
	}

	/* clean touch event flag */
	val = 0;
	ret = cd->hw_ops->write(cd, flag_addr, &val, 1);
	if (ret < 0) {
		ts_err("clean touch event failed, exit!");
		goto exit;
	}

	while (retry--) {
		usleep_range(2000, 2100);
		ret = cd->hw_ops->read(cd, flag_addr, &val, 1);
		if (!ret && (val & 0x80))
			break;
	}
	if (retry < 0) {
		ts_err("framedata is not ready val:0x%02x, exit!", val);
		ret = -EINVAL;
		goto exit;
	}

	if (strstr(type, CMD_GET_TX_FREQ)) {
		ret = cd->hw_ops->read(cd, tx_freq_addr, frame_buf, 2);
		if (ret < 0) {
			ts_err("read frame data failed");
			goto exit;
		}

		index = sprintf(rbuf, "%s: %dHz\n", CMD_GET_TX_FREQ,
			le16_to_cpup((__le16 *)frame_buf) * 61);
		goto exit;
	}

	if (strstr(type, CMD_GET_RAWDATA) || strstr(type, CMD_GET_DIFFDATA) ||
		strstr(type, CMD_GET_BASEDATA)) {
		ret = cd->hw_ops->read(cd, mutual_addr, frame_buf, tx * rx * 2);
		if (ret < 0) {
			ts_err("read frame data failed");
			goto exit;
		}
		goodix_rotate_abcd2cbad(tx, rx, (s16 *)frame_buf, NULL);
		for (i = 0; i < tx * rx; i++) {
			index += sprintf(
				&rbuf[index], "%5d,", *((s16 *)frame_buf + i));
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
	} else {
		ret = cd->hw_ops->read(cd, self_addr, frame_buf, (tx + rx) * 2);
		if (ret < 0) {
			ts_err("read frame data failed");
			goto exit;
		}
		index += sprintf(&rbuf[index], "TX:");
		for (i = 0; i < tx + rx; i++) {
			index += sprintf(
				&rbuf[index], "%5d,", *((s16 *)frame_buf + i));
			if ((i + 1) == tx)
				index += sprintf(&rbuf[index], "\nRX:");
		}
		index += sprintf(&rbuf[index], "\n");
	}

exit:
	kfree(frame_buf);
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0;
	temp_cmd.len = 5;
	if (cd->bus->sub_ic_type == IC_TYPE_SUB_GT7986) {
		temp_cmd.cmd = 0;
		temp_cmd.len = 4;
	}
	cd->hw_ops->send_cmd(cd, &temp_cmd);
	/* clean touch event flag */
	val = 0;
	cd->hw_ops->write(cd, flag_addr, &val, 1);
	/* enable irq & esd */
	cd->hw_ops->irq_enable(cd, true);
	goodix_ts_esd_on(cd);
	return ret;
}

static void goodix_read_config(struct goodix_ts_core *cd)
{
	int ret;
	u8 *cfg_buf;
	u32 cfg_id;
	u8 cfg_ver;

	cfg_buf = kzalloc(GOODIX_CFG_MAX_SIZE, GFP_KERNEL);
	if (cfg_buf == NULL) {
		ts_err("failed to alloc cfg buffer");
		return;
	}

	ret = cd->hw_ops->read_config(cd, cfg_buf, GOODIX_CFG_MAX_SIZE);
	if (ret < 0) {
		ts_err("read config failed");
		goto exit;
	}

	cfg_id = le32_to_cpup((__le32 *)&cfg_buf[30]);
	cfg_ver = cfg_buf[34];
	index = sprintf(
		rbuf, "config_id:0x%X config_ver:0x%02X\n", cfg_id, cfg_ver);

exit:
	kfree(cfg_buf);
}

static void goodix_set_custom_mode(struct goodix_ts_core *cd, u8 type, u8 val)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (type == PALM_FUNC) {
		index = sprintf(&rbuf[index], "set palm %s\n",
			val ? "enabled" : "disabled");
	} else if (type == NOISE_FUNC) {
		if (val == 0) {
			index = sprintf(&rbuf[index], "set noise disabled\n");
		} else if (val == 1) {
			index = sprintf(&rbuf[index], "set noise enabled\n");
		} else if (val == 2) {
			index = sprintf(&rbuf[index], "set noise lv0\n");
		} else {
			index = sprintf(&rbuf[index], "set noise lv1\n");
		}
	} else if (type == WATER_FUNC) {
		index = sprintf(&rbuf[index], "set water %s\n",
			val ? "enabled" : "disabled");
	} else if (type == GRIP_FUNC) {
		index = sprintf(&rbuf[index], "set grip %s\n",
			val ? "enabled" : "disabled");
	} else {
		ts_err("invalid type, %d", type);
		return;
	}

	temp_cmd.len = 6;
	temp_cmd.cmd = 0xC7;
	temp_cmd.data[0] = type;
	temp_cmd.data[1] = val;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_set_gesture_param(struct goodix_ts_core *cd, u8 type)
{
	struct goodix_ts_cmd temp_cmd = {0};
	u32 cmd_reg = cd->ic_info.misc.cmd_addr;
	u32 fw_buffer_addr = cd->ic_info.misc.fw_buffer_addr;
	int retry;
	u8 status;

	temp_cmd.len = 4;
	temp_cmd.cmd = 0xC8;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
	retry = 20;
	while (retry--) {
		usleep_range(5000, 5100);
		cd->hw_ops->read(cd, cmd_reg, &status, 1);
		if (status == 0x80)
			break;
	}
	if (retry < 0) {
		ts_err("failed to start write gesture param, status[%x]",
			status);
		goto exit;
	}
	if (type == GESTURE_STTW) {
		ts_info("STTW param:%*ph", gesture_param_st.length,
			gesture_param_st.buf);
		cd->hw_ops->write(cd, fw_buffer_addr, gesture_param_st.buf,
			sizeof(gesture_param_st));
	} else {
		ts_info("LPTW param:%*ph", gesture_param_lp.length,
			gesture_param_lp.buf);
		cd->hw_ops->write(cd, fw_buffer_addr, gesture_param_lp.buf,
			sizeof(gesture_param_lp));
	}

	temp_cmd.len = 4;
	temp_cmd.cmd = 0x05;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
	retry = 20;
	while (retry--) {
		usleep_range(5000, 5100);
		cd->hw_ops->read(cd, cmd_reg, &status, 1);
		if (status == 0x80 || status == 0x03)
			break;
	}
	if (retry < 0) {
		ts_err("failed to update gesture param, status[%x]", status);
		goto exit;
	}

	if (status == 0x80) {
		ts_info("update gesture param OK");
		index = sprintf(rbuf, "update gesture param OK\n");
	} else {
		ts_info("update gesture param FAIL");
		index = sprintf(rbuf, "update gesture param FAIL\n");
	}

exit:
	temp_cmd.len = 4;
	temp_cmd.cmd = 0x06;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_force_update(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	ret = goodix_get_config_proc(cd);
	if (ret < 0)
		ts_err("not found valid config");

	ret = goodix_do_fw_update(cd, UPDATE_MODE_BLOCK | UPDATE_MODE_FORCE |
					      UPDATE_MODE_SRC_REQUEST);
	if (ret < 0)
		index = sprintf(rbuf, "%s: NG\n", CMD_FW_UPDATE);
	else
		index = sprintf(rbuf, "%s: OK\n", CMD_FW_UPDATE);
}

static void goodix_run_auto_test(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;

	raw_data_cnt = 16;
	noise_data_cnt = 1;
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	ret = malloc_test_resource();
	if (ret < 0) {
		ts_err("malloc test resource failed");
		return;
	}
	ts_test->item[GTP_CAP_TEST] = true;
	ts_test->item[GTP_NOISE_TEST] = true;
	ts_test->item[GTP_DELTA_TEST] = true;
	ts_test->item[GTP_SELFCAP_TEST] = true;
	ts_test->item[GTP_SHORT_TEST] = true;
	goodix_auto_test(cd, true);
}

static void goodix_run_open_test(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;

	if (bufsz < 1 || bufsz > 2) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}
	if (buf[0] <= 0) {
		ts_err("invalid input param:%d", buf[0]);
		return;
	}

	raw_data_cnt = buf[0];
	/*
	 * [GOOG]
	 * Change the size based on the channel number(tx * rx) for
	 * output (raw + data) and markup text with format "%5d,".
	 */
	//rbuf = malloc_proc_buffer(raw_data_cnt * 30000);
	rbuf = malloc_proc_buffer(raw_data_cnt * tx * rx * 6 * 3);
	/*~[GOOG]*/
	if (rbuf == NULL) {
		ts_err("failed to malloc rbuf");
		return;
	}
	ret = malloc_test_resource();
	if (ret < 0) {
		ts_err("malloc test resource failed");
		return;
	}
	ts_test->item[GTP_CAP_TEST] = true;
	ts_test->freq = buf[1];
	goodix_auto_test(cd, false);
}

static void goodix_run_self_open_test(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;

	rbuf = malloc_proc_buffer(HUGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	ret = malloc_test_resource();
	if (ret < 0) {
		ts_err("malloc test resource failed");
		return;
	}
	ts_test->item[GTP_SELFCAP_TEST] = true;
	goodix_auto_test(cd, false);
}

static void goodix_run_noise_test(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}
	if (buf[0] <= 0) {
		ts_err("invalid input param:%d", buf[0]);
		return;
	}
	noise_data_cnt = buf[0];
	/*
	 * [GOOG]
	 * Change the size based on the channel number(tx * rx) for
	 * output (raw + data) and markup text with format "%5d,".
	 */
	//rbuf = malloc_proc_buffer(noise_data_cnt * 30000);
	rbuf = malloc_proc_buffer(noise_data_cnt * tx * rx * 6 * 3);
	/*~[GOOG]*/
	if (rbuf == NULL) {
		ts_err("failed to malloc rbuf");
		return;
	}
	ret = malloc_test_resource();
	if (ret < 0) {
		ts_err("malloc test resource failed");
		return;
	}
	ts_test->item[GTP_NOISE_TEST] = true;
	goodix_auto_test(cd, false);
}

static void goodix_run_auto_noise_test(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 2) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}
	if (buf[0] <= 0 || buf[1] <= 0) {
		ts_err("invalid input param:%d,%d", buf[0], buf[1]);
		return;
	}

	rbuf = malloc_proc_buffer(HUGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	goodix_auto_noise_test(cd, buf[0], buf[1]);
}

static void goodix_run_short_test(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;

	rbuf = malloc_proc_buffer(HUGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	ret = malloc_test_resource();
	if (ret < 0) {
		ts_err("malloc test resource failed");
		return;
	}
	ts_test->item[GTP_SHORT_TEST] = true;
	goodix_auto_test(cd, false);
}

static void goodix_get_package_id(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;
	u8 id;

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	mutex_lock(&cd->cmd_lock);
	usleep_range(6000, 6100);
	if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
		ret = cd->hw_ops->read_flash(cd, 0x3F301, &id, 1);
	else
		ret = cd->hw_ops->read_flash(cd, 0x1F301, &id, 1);
	mutex_unlock(&cd->cmd_lock);
	if (ret < 0)
		index = sprintf(rbuf, "%s: NG\n", CMD_GET_PACKAGE_ID);
	else
		index = sprintf(rbuf, "%s: 0x%x\n", CMD_GET_PACKAGE_ID, id);
}

static void goodix_get_mcu_id(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;
	u8 id;

	if (cd->bus->ic_type == IC_TYPE_BERLIN_B) {
		ts_info("berlinB is always TSMC");
		return;
	}
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	mutex_lock(&cd->cmd_lock);
	ret = cd->hw_ops->read_flash(cd, 0x1F314, &id, 1);
	mutex_unlock(&cd->cmd_lock);
	if (ret < 0)
		index = sprintf(rbuf, "%s: NG\n", CMD_GET_MCU_ID);
	else
		index = sprintf(rbuf, "%s: 0x%x\n", CMD_GET_MCU_ID, id);
}

static void goodix_get_version(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_fw_version fw_ver;
	struct goodix_ic_info ic_info;

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	cd->hw_ops->read_version(cd, &fw_ver);
	cd->hw_ops->get_ic_info(cd, &ic_info);
	index = sprintf(rbuf, "%s: 0x%02x%02x%02x%02x 0x%x\n", CMD_GET_VERSION,
		fw_ver.patch_vid[0], fw_ver.patch_vid[1], fw_ver.patch_vid[2],
		fw_ver.patch_vid[3], ic_info.version.config_id);
}

static void goodix_get_rawdata(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	get_cap_data(cd, CMD_GET_RAWDATA);
}

static void goodix_get_diffdata(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	get_cap_data(cd, CMD_GET_DIFFDATA);
}

static void goodix_get_basedata(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	get_cap_data(cd, CMD_GET_BASEDATA);
}

static void goodix_get_self_rawdata(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	get_cap_data(cd, CMD_GET_SELF_RAWDATA);
}

static void goodix_get_self_diffdata(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	get_cap_data(cd, CMD_GET_SELF_DIFFDATA);
}

static void goodix_get_self_basedata(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	get_cap_data(cd, CMD_GET_SELF_BASEDATA);
}

static void goodix_set_double_tap_gesture(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		cd->gesture_type &= ~GESTURE_DOUBLE_TAP;
		index = sprintf(rbuf, "%s: disable OK\n", CMD_SET_DOUBLE_TAP);
		ts_info("disable double tap");
	} else {
		cd->gesture_type |= GESTURE_DOUBLE_TAP;
		index = sprintf(rbuf, "%s: enable OK\n", CMD_SET_DOUBLE_TAP);
		ts_info("enable single tap");
	}
}

static void goodix_set_single_tap_gesture(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		cd->gesture_type &= ~GESTURE_SINGLE_TAP;
		index = sprintf(rbuf, "%s: disable OK\n", CMD_SET_SINGLE_TAP);
		ts_info("disable single tap");
	} else {
		cd->gesture_type |= GESTURE_SINGLE_TAP;
		index = sprintf(rbuf, "%s: enable OK\n", CMD_SET_SINGLE_TAP);
		ts_info("enable single tap");
	}
}

static void goodix_set_long_press_gesture(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		cd->gesture_type &= ~GESTURE_FOD_PRESS;
		index = sprintf(rbuf, "%s: disable OK\n", CMD_SET_LONG_PRESS);
		ts_info("disable long press");
	} else {
		cd->gesture_type |= GESTURE_FOD_PRESS;
		index = sprintf(rbuf, "%s: enable OK\n", CMD_SET_LONG_PRESS);
		ts_info("enable long press");
	}
}

static void goodix_set_st_param(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 8) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	gesture_param_st.length = sizeof(gesture_param_st);
	gesture_param_st.type = GESTURE_STTW;
	gesture_param_st.st_min_x = buf[0];
	gesture_param_st.st_max_x = buf[1];
	gesture_param_st.st_min_y = buf[2];
	gesture_param_st.st_max_y = buf[3];
	gesture_param_st.st_min_count = buf[4];
	gesture_param_st.st_max_count = buf[5];
	gesture_param_st.st_motion_tolerance = buf[6];
	gesture_param_st.st_max_size = buf[7];
	goodix_append_checksum(gesture_param_st.buf,
		sizeof(gesture_param_st) - 2, CHECKSUM_MODE_U8_LE);
	ts_info("st_min_x:                  %d", gesture_param_st.st_min_x);
	ts_info("st_max_x:                  %d", gesture_param_st.st_max_x);
	ts_info("st_min_y:                  %d", gesture_param_st.st_min_y);
	ts_info("st_max_y:                  %d", gesture_param_st.st_max_y);
	ts_info("st_min_count:              %d", gesture_param_st.st_min_count);
	ts_info("st_max_count:              %d", gesture_param_st.st_max_count);
	ts_info("st_motion_tolerance:       %d",
		gesture_param_st.st_motion_tolerance);
	ts_info("st_max_size:               %d", gesture_param_st.st_max_size);
	goodix_set_gesture_param(cd, GESTURE_STTW);
}

static void goodix_set_lp_param(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 17) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	gesture_param_lp.length = sizeof(gesture_param_lp);
	gesture_param_lp.type = GESTURE_LPTW;
	gesture_param_lp.lp_min_x = buf[0];
	gesture_param_lp.lp_max_x = buf[1];
	gesture_param_lp.lp_min_y = buf[2];
	gesture_param_lp.lp_max_y = buf[3];
	gesture_param_lp.lp_min_count = buf[4];
	gesture_param_lp.lp_max_size = buf[5];
	gesture_param_lp.lp_marginal_min_x = buf[6];
	gesture_param_lp.lp_marginal_max_x = buf[7];
	gesture_param_lp.lp_marginal_min_y = buf[8];
	gesture_param_lp.lp_marginal_max_y = buf[9];
	gesture_param_lp.lp_monitor_chan_min_tx = buf[10];
	gesture_param_lp.lp_monitor_chan_max_tx = buf[11];
	gesture_param_lp.lp_monitor_chan_min_rx = buf[12];
	gesture_param_lp.lp_monitor_chan_max_rx = buf[13];
	gesture_param_lp.lp_min_node_count = buf[14];
	gesture_param_lp.lp_motion_tolerance_inner = buf[15];
	gesture_param_lp.lp_motion_tolerance_outer = buf[16];
	goodix_append_checksum(gesture_param_lp.buf,
		sizeof(gesture_param_lp) - 2, CHECKSUM_MODE_U8_LE);
	ts_info("lp_min_x:                  %d", gesture_param_lp.lp_min_x);
	ts_info("lp_max_x:                  %d", gesture_param_lp.lp_max_x);
	ts_info("lp_min_y:                  %d", gesture_param_lp.lp_min_y);
	ts_info("lp_max_y:                  %d", gesture_param_lp.lp_max_y);
	ts_info("lp_min_count:              %d", gesture_param_lp.lp_min_count);
	ts_info("lp_max_size:               %d", gesture_param_lp.lp_max_size);
	ts_info("lp_marginal_min_x:         %d",
		gesture_param_lp.lp_marginal_min_x);
	ts_info("lp_marginal_max_x:         %d",
		gesture_param_lp.lp_marginal_max_x);
	ts_info("lp_marginal_min_y:         %d",
		gesture_param_lp.lp_marginal_min_y);
	ts_info("lp_marginal_max_y:         %d",
		gesture_param_lp.lp_marginal_max_y);
	ts_info("lp_monitor_chan_min_tx:    %d",
		gesture_param_lp.lp_monitor_chan_min_tx);
	ts_info("lp_monitor_chan_max_tx:    %d",
		gesture_param_lp.lp_monitor_chan_max_tx);
	ts_info("lp_monitor_chan_min_rx:    %d",
		gesture_param_lp.lp_monitor_chan_min_rx);
	ts_info("lp_monitor_chan_max_rx:    %d",
		gesture_param_lp.lp_monitor_chan_max_rx);
	ts_info("lp_min_node_count:         %d",
		gesture_param_lp.lp_min_node_count);
	ts_info("lp_motion_tolerance_inner: %d",
		gesture_param_lp.lp_motion_tolerance_inner);
	ts_info("lp_motion_tolerance_outer: %d",
		gesture_param_lp.lp_motion_tolerance_outer);
	goodix_set_gesture_param(cd, GESTURE_LPTW);
}

static void goodix_set_irq_enable(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		cd->hw_ops->irq_enable(cd, false);
		index = sprintf(rbuf, "%s: disable OK\n", CMD_SET_IRQ_ENABLE);
	} else {
		cd->hw_ops->irq_enable(cd, true);
		index = sprintf(rbuf, "%s: enable OK\n", CMD_SET_IRQ_ENABLE);
	}
}

static void goodix_set_esd_enable(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		goodix_ts_esd_off(cd);
		index = sprintf(rbuf, "%s: disable OK\n", CMD_SET_ESD_ENABLE);
	} else {
		goodix_ts_esd_on(cd);
		index = sprintf(rbuf, "%s: enable OK\n", CMD_SET_ESD_ENABLE);
	}
}

static void goodix_set_debug_log(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		debug_log_flag = false;
		index = sprintf(rbuf, "%s: disable OK\n", CMD_SET_DEBUG_LOG);
	} else {
		debug_log_flag = true;
		index = sprintf(rbuf, "%s: enable OK\n", CMD_SET_DEBUG_LOG);
	}
}

static void goodix_set_scan_mode(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	temp_cmd.len = 5;
	temp_cmd.cmd = 0x9F;

	if (buf[0] == 0) {
		temp_cmd.data[0] = 0;
		ts_info("set scan mode to default");
		index = sprintf(rbuf, "set scan mode to default\n");
	} else if (buf[0] == 1) {
		temp_cmd.data[0] = 3;
		ts_info("set scan mode to idle");
		index = sprintf(rbuf, "set scan mode to idle\n");
	} else {
		temp_cmd.data[0] = 2;
		ts_info("set scan mode to active");
		index = sprintf(rbuf, "set scan mode to active\n");
	}

	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_get_scan_mode(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	u8 status;

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	cd->hw_ops->read(cd, 0x10219, &status, 1);
	ts_info("ic status:%d", status);

	if (status == 1) {
		index = sprintf(rbuf, "normal active\n");
	} else if (status == 2) {
		index = sprintf(rbuf, "normal idle\n");
	} else if (status == 3) {
		index = sprintf(rbuf, "lowpower active\n");
	} else if (status == 4) {
		index = sprintf(rbuf, "lowpower idle\n");
	} else if (status == 5) {
		index = sprintf(rbuf, "sleep\n");
	} else {
		index = sprintf(rbuf, "invalid mode %d\n", status);
	}
}

static void goodix_set_continue_mode(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		ts_info("enable continue report");
		index = sprintf(rbuf, "enable continue report\n");
	} else {
		ts_info("disable continue report");
		index = sprintf(rbuf, "disable continue report\n");
	}

	temp_cmd.len = 5;
	temp_cmd.cmd = 0xC6;
	temp_cmd.data[0] = buf[0];
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_get_channel_num(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	index = sprintf(rbuf, "TX:%d RX:%d\n", cd->ic_info.parm.drv_num,
		cd->ic_info.parm.sen_num);
}

static void goodix_get_tx_freq(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	int ret;

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	ret = get_cap_data(cd, CMD_GET_TX_FREQ);
	if (ret < 0)
		index = sprintf(rbuf, "%s: NG\n", CMD_GET_TX_FREQ);
}

static void goodix_hw_reset(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	cd->hw_ops->irq_enable(cd, false);
	cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
	cd->hw_ops->irq_enable(cd, true);
}

static void goodix_set_sense_mode(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	temp_cmd.len = 4;
	temp_cmd.cmd = 0xA7;

	if (buf[0] == 1) {
		/* normal mode */
		index = sprintf(rbuf, "switch to coordinate mode\n");
		cd->hw_ops->send_cmd(cd, &temp_cmd);
		goodix_ts_esd_on(cd);
		cd->hw_ops->irq_enable(cd, true);
		atomic_set(&cd->suspended, 0);
	} else if (buf[0] == 2) {
		/* gesture mode */
		index = sprintf(rbuf, "switch to gesture mode\n");
		goodix_ts_esd_off(cd);
		cd->hw_ops->gesture(cd, 0);
		cd->hw_ops->irq_enable(cd, true);
		atomic_set(&cd->suspended, 1);
	} else {
		/* sleep mode */
		index = sprintf(rbuf, "switch to sleep mode\n");
		goodix_ts_esd_off(cd);
		cd->hw_ops->irq_enable(cd, false);
		cd->hw_ops->suspend(cd);
	}
}

static void goodix_get_config(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	goodix_read_config(cd);
}

static void goodix_get_fw_status(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_status_data status_data;
	u8 val[2];

	index = 0;
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	cd->hw_ops->read(cd, 0x1021A, val, sizeof(val));
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"coordfilter_status[%d] ", (val[0] >> 7) & 0x01);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"set_highsense_mode[%d] ", (val[0] >> 6) & 0x01);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"set_noise_mode[%d] ", (val[0] >> 4) & 0x03);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"set_water_mode[%d] ", (val[0] >> 3) & 0x01);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"set_grip_mode[%d] ", (val[0] >> 2) & 0x01);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"set_palm_mode[%d] ", (val[0] >> 1) & 0x01);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"set_heatmap_mode[%d]\n", (val[0] >> 0) & 0x01);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"fw_ns/hs[%d]", (val[1] >> 0) & 0x01);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"wireless mode[%d]", (val[1] >> 1) & 0x01);

	cd->hw_ops->read(cd, 0x1021C, (u8 *)&status_data, sizeof(status_data));
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"water[%d] ", status_data.water_sta);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"palm[%d] ", status_data.palm_sta);
	index += scnprintf(&rbuf[index], SHORT_SIZE - index,
		"noise_lv[%d]\n", status_data.noise_lv);
}

static void goodix_set_highsense_mode(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		ts_info("exit highsense mode");
		index = sprintf(rbuf, "exit highsense mode\n");
	} else {
		ts_info("enter highsense mode");
		index = sprintf(rbuf, "enter highsense mode\n");
	}

	temp_cmd.len = 5;
	temp_cmd.cmd = 0x72;
	temp_cmd.data[0] = buf[0];
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_set_grip_data(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	if (buf[0] == 0) {
		ts_info("portrait mode");
		index = sprintf(rbuf, "portrait mode\n");
		temp_cmd.len = 4;
		temp_cmd.cmd = 0x18;
	} else if (buf[0] == 1) {
		ts_info("landscape left");
		index = sprintf(rbuf, "landscape left\n");
		temp_cmd.len = 5;
		temp_cmd.cmd = 0x17;
		temp_cmd.data[0] = 0;
	} else if (buf[0] == 2) {
		ts_info("landscape right");
		index = sprintf(rbuf, "landscape right\n");
		temp_cmd.len = 5;
		temp_cmd.cmd = 0x17;
		temp_cmd.data[0] = 1;
	} else {
		ts_err("invalid grip data, %d", buf[0]);
		return;
	}

	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_set_grip_mode(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	goodix_set_custom_mode(cd, GRIP_FUNC, buf[0]);
}

static void goodix_set_palm_mode(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	goodix_set_custom_mode(cd, PALM_FUNC, buf[0]);
}

static void goodix_set_noise_mode(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	goodix_set_custom_mode(cd, NOISE_FUNC, buf[0]);
}

static void goodix_set_water_mode(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	goodix_set_custom_mode(cd, WATER_FUNC, buf[0]);
}

static void goodix_set_heatmap(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	cd->hw_ops->irq_enable(cd, false);
	if (buf[0] == 0) {
		index = sprintf(rbuf, "disable heatmap\n");
/*
 * [GOOG]
 * Use goodix_update_heatmap() to do the heatmap process.
		kfree(cd->heatmap_buffer);
		cd->heatmap_buffer = NULL;
 */
		temp_cmd.len = 5;
		temp_cmd.cmd = 0xC9;
		temp_cmd.data[0] = 0;
	} else {
		index = sprintf(rbuf, "enable heatmap\n");
/*
 * [GOOG]
 * Use goodix_update_heatmap() to do the heatmap process.
		cd->heatmap_buffer = kzalloc(GOODIX_MAX_FRAMEDATA_LEN, GFP_KERNEL);
 */
		temp_cmd.len = 5;
		temp_cmd.cmd = 0xC9;
		temp_cmd.data[0] = 1;
	}
	// no need to do anything for GT7986
	if (cd->bus->sub_ic_type != IC_TYPE_SUB_GT7986)
		cd->hw_ops->send_cmd(cd, &temp_cmd);
	cd->hw_ops->irq_enable(cd, true);
}

static void goodix_get_self_compensation(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	u8 *cfg;
	u8 *cfg_buf;
	int len;
	int cfg_num;
	int sub_cfg_index;
	int sub_cfg_len;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	struct goodix_ts_cmd temp_cmd = {0};
	s16 val;
	int i, j;

	cfg_buf = kzalloc(GOODIX_CFG_MAX_SIZE, GFP_KERNEL);
	if (cfg_buf == NULL) {
		ts_err("failed to alloc cfg buffer");
		return;
	}

	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	temp_cmd.len = 5;
	temp_cmd.cmd = 0x9F;

	if (cd->bus->ic_type == IC_TYPE_BERLIN_B) {
		/* exit idle firstly */
		temp_cmd.data[0] = 2;
		cd->hw_ops->send_cmd(cd, &temp_cmd);
		msleep(20);
		cd->hw_ops->read(cd, cd->ic_info.misc.auto_scan_cmd_addr, cfg_buf,
			(tx + rx) * 2);
		/* restore default */
		temp_cmd.data[0] = 0;
		cd->hw_ops->send_cmd(cd, &temp_cmd);
		index += sprintf(&rbuf[index], "Tx:");
		for (i = 0; i < tx; i++) {
			val = le16_to_cpup((__le16 *)&cfg_buf[i * 2]);
			index += sprintf(&rbuf[index], "%d,", val);
		}
		index += sprintf(&rbuf[index], "\nRx:");
		for (i = 0; i < rx; i++) {
			val = le16_to_cpup((__le16 *)&cfg_buf[tx * 2 + i * 2]);
			index += sprintf(&rbuf[index], "%d,", val);
		}
		index += sprintf(&rbuf[index], "\n");
		goto exit_free;
	}

	len = cd->hw_ops->read_config(cd, cfg_buf, GOODIX_CFG_MAX_SIZE);
	if (len < 0) {
		ts_err("read config failed");
		goto exit_free;
	}

	cfg = cfg_buf;
	cfg_num = cfg[61];
	cfg += 64;
	for (i = 0; i < cfg_num; i++) {
		sub_cfg_len = cfg[0] - 2;
		sub_cfg_index = cfg[1];
		if (sub_cfg_index == 13) { // TX self cancel
			ts_info("sub_cfg_len:%d", sub_cfg_len);
			index += sprintf(&rbuf[index], "Tx:");
			for (j = 0; j < tx; j++) {
				val = le16_to_cpup((__le16 *)&cfg[2 + j * 4]) +
				      le16_to_cpup((__le16 *)&cfg[4 + j * 4]);
				index += sprintf(&rbuf[index], "%d,", val);
			}
			index += sprintf(&rbuf[index], "\n");
		} else if (sub_cfg_index == 14) { // RX self cancel
			ts_info("sub_cfg_len:%d", sub_cfg_len);
			index += sprintf(&rbuf[index], "Rx:");
			for (j = 0; j < rx; j++) {
				val = le16_to_cpup((__le16 *)&cfg[2 + j * 2]);
				index += sprintf(&rbuf[index], "%d,", val);
			}
			index += sprintf(&rbuf[index], "\n");
		}
		cfg += (sub_cfg_len + 2);
	}
exit_free:
	kfree(cfg_buf);
}

static void goodix_set_report_rate(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	index = sprintf(rbuf, "set report rate %d\n", buf[0]);
	ts_info("set report rate %d", buf[0]);
	temp_cmd.len = 5;
	temp_cmd.cmd = 0x9D;
	temp_cmd.data[0] = buf[0];
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

#define DUMP_AREA1_ADDR 0x10194
#define DUMP_AREA1_LEN  132
#define DUMP_AREA2_ADDR 0x10400
#define DUMP_AREA2_LEN  596
#define DUMP_AREA3_ADDR 0x10308
#define DUMP_AREA3_LEN 64
static void goodix_get_dump_log(struct goodix_ts_core *cd, int *buf, int bufsz)
{
	u8 debug_buf[DUMP_AREA2_LEN];
	int i;

	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	cd->hw_ops->read(cd, DUMP_AREA1_ADDR, debug_buf, DUMP_AREA1_LEN);
	index += scnprintf(&rbuf[index], DUMP_AREA2_LEN - index, "0x%04x:\n", DUMP_AREA1_ADDR);
	for (i = 0; i < DUMP_AREA1_LEN; i++) {
		index += scnprintf(&rbuf[index], sizeof(rbuf) - index,
			"%02x,", debug_buf[i]);
		if ((i + 1) % 32 == 0)
			index += sprintf(&rbuf[index], "\n");
	}

	cd->hw_ops->read(cd, DUMP_AREA2_ADDR, debug_buf, DUMP_AREA2_LEN);
	index += scnprintf(&rbuf[index], DUMP_AREA2_LEN - index,
		"\n0x%04x:\n", DUMP_AREA2_ADDR);
	for (i = 0; i < DUMP_AREA2_LEN; i++) {
		index += scnprintf(&rbuf[index], sizeof(rbuf) - index, "%02x,",
			debug_buf[i]);
		if ((i + 1) % 32 == 0)
			index += sprintf(&rbuf[index], "\n");
	}

	cd->hw_ops->read(cd, DUMP_AREA3_ADDR, debug_buf, DUMP_AREA3_LEN);
	index += scnprintf(&rbuf[index], DUMP_AREA2_LEN - index,
		"\n0x%04x:\n", DUMP_AREA3_ADDR);
	for (i = 0; i < DUMP_AREA3_LEN; i++) {
		index += scnprintf(&rbuf[index], sizeof(rbuf) - index, "%02x,",
			debug_buf[i]);
		if ((i + 1) % 32 == 0)
			index += sprintf(&rbuf[index], "\n");
	}
}

static void goodix_set_freq_index(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	index = sprintf(rbuf, "set frequency index %d\n", buf[0]);
	ts_info("set frequency index %d", buf[0]);
	temp_cmd.len = 5;
	temp_cmd.cmd = 0x9C;
	temp_cmd.data[0] = buf[0];
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_disable_coor_filter(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	index = sprintf(rbuf, "disable coordinate filter %d\n", buf[0]);
	temp_cmd.len = 5;
	temp_cmd.cmd = 0xCA;
	temp_cmd.data[0] = buf[0] ? 1 : 0;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_get_im_rawdata(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	u32 im_addr = cd->ic_info.misc.frame_data_addr +
		      cd->ic_info.misc.frame_data_head_len +
		      cd->ic_info.misc.fw_attr_len +
		      cd->ic_info.misc.fw_log_len +
		      cd->ic_info.misc.mutual_struct_len +
		      cd->ic_info.misc.self_struct_len + 32;
	int freq_num = cd->ic_info.parm.mutual_freq_num;
	int rx = cd->ic_info.parm.sen_num;
	s16 data[MAX_SCAN_FREQ_NUM * MAX_SEN_NUM_BRB];
	int i, j;
	int cnt = 0;
	struct goodix_ts_cmd temp_cmd = {0};
	u32 flag_addr = cd->ic_info.misc.frame_data_addr;
	u8 val;
	int retry = 20;

	/* disable irq & close esd */
	cd->hw_ops->irq_enable(cd, false);
	goodix_ts_esd_off(cd);

	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x81;
	temp_cmd.len = 5;
	cd->hw_ops->send_cmd(cd, &temp_cmd);

	/* clean touch event flag */
	val = 0;
	cd->hw_ops->write(cd, flag_addr, &val, 1);
	while (retry--) {
		usleep_range(2000, 2100);
		cd->hw_ops->read(cd, flag_addr, &val, 1);
		if (val & 0x80)
			break;
	}
	if (retry < 0) {
		ts_err("framedata is not ready val:0x%02x, exit!", val);
		goto exit;
	}

	cd->hw_ops->read(cd, im_addr, (u8 *)data, freq_num * rx * 2);
	rbuf = malloc_proc_buffer(LARGE_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	for (i = 0; i < freq_num; i++) {
		index += sprintf(&rbuf[index], "freq%d: ", i);
		for (j = 0; j < rx; j++)
			index += sprintf(&rbuf[index], "%d,", data[cnt++]);
		index += sprintf(&rbuf[index], "\n");
	}

exit:
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0;
	temp_cmd.len = 5;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
	val = 0;
	cd->hw_ops->write(cd, flag_addr, &val, 1);
	/* enable irq & esd */
	cd->hw_ops->irq_enable(cd, true);
	goodix_ts_esd_on(cd);
}

static void goodix_set_hsync_speed(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd = {0};

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	index = sprintf(rbuf, "hsync mode: %s\n",
		(buf[0] == 0) ? "high speed" : "normal speed");
	temp_cmd.len = 5;
	temp_cmd.cmd = 0xF2;
	temp_cmd.data[0] = (buf[0] == 0) ? 0 : 1;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_set_wireless_charge(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd;

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		return;
	}

	index = sprintf(
		rbuf, "%s wireless mode\n", (buf[0] == 0) ? "exit" : "enter");
	temp_cmd.len = 5;
	temp_cmd.cmd = 0xAF;
	temp_cmd.data[0] = (buf[0] == 0) ? 0 : 1;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_set_continue_heatmap(
	struct goodix_ts_core *cd, int *buf, int bufsz)
{
	struct goodix_ts_cmd temp_cmd;

	if (bufsz != 1) {
		ts_err("invalid cmd size:%d", bufsz);
		return;
	}

	rbuf = malloc_proc_buffer(SHORT_SIZE);
	index = sprintf(rbuf, "%s continue heatmap\n",
		(buf[0] == 0) ? "disable" : "enable");
	temp_cmd.len = 5;
	temp_cmd.cmd = 0xCC;
	temp_cmd.data[0] = (buf[0] == 0) ? 0 : 1;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static struct cmd_handler cmd_handler_list[] = {
	{ CMD_FW_UPDATE, goodix_force_update },
	{ CMD_AUTO_TEST, goodix_run_auto_test }, { CMD_STYLUS_RAW_TEST, NULL },
	{ CMD_STYLUS_OSC_TEST, NULL }, { CMD_GET_STYLUS_DATA, NULL },
	{ CMD_OPEN_TEST, goodix_run_open_test },
	{ CMD_SELF_OPEN_TEST, goodix_run_self_open_test },
	{ CMD_NOISE_TEST, goodix_run_noise_test },
	{ CMD_AUTO_NOISE_TEST, goodix_run_auto_noise_test },
	{ CMD_SHORT_TEST, goodix_run_short_test },
	{ CMD_GET_PACKAGE_ID, goodix_get_package_id },
	{ CMD_GET_MCU_ID, goodix_get_mcu_id },
	{ CMD_GET_VERSION, goodix_get_version },
	{ CMD_GET_RAWDATA, goodix_get_rawdata },
	{ CMD_GET_DIFFDATA, goodix_get_diffdata },
	{ CMD_GET_BASEDATA, goodix_get_basedata },
	{ CMD_GET_SELF_RAWDATA, goodix_get_self_rawdata },
	{ CMD_GET_SELF_DIFFDATA, goodix_get_self_diffdata },
	{ CMD_GET_SELF_BASEDATA, goodix_get_self_basedata },
	{ CMD_SET_DOUBLE_TAP, goodix_set_double_tap_gesture },
	{ CMD_SET_SINGLE_TAP, goodix_set_single_tap_gesture },
	{ CMD_SET_LONG_PRESS, goodix_set_long_press_gesture },
	{ CMD_SET_ST_PARAM, goodix_set_st_param },
	{ CMD_SET_LP_PARAM, goodix_set_lp_param },
	{ CMD_SET_CHARGE_MODE, NULL },
	{ CMD_SET_IRQ_ENABLE, goodix_set_irq_enable },
	{ CMD_SET_ESD_ENABLE, goodix_set_esd_enable },
	{ CMD_SET_DEBUG_LOG, goodix_set_debug_log },
	{ CMD_SET_SCAN_MODE, goodix_set_scan_mode },
	{ CMD_GET_SCAN_MODE, goodix_get_scan_mode },
	{ CMD_SET_CONTINUE_MODE, goodix_set_continue_mode },
	{ CMD_GET_CHANNEL_NUM, goodix_get_channel_num },
	{ CMD_GET_TX_FREQ, goodix_get_tx_freq }, { CMD_RESET, goodix_hw_reset },
	{ CMD_SET_SENSE_MODE, goodix_set_sense_mode },
	{ CMD_GET_CONFIG, goodix_get_config },
	{ CMD_GET_FW_STATUS, goodix_get_fw_status },
	{ CMD_SET_HIGHSENSE_MODE, goodix_set_highsense_mode },
	{ CMD_SET_GRIP_DATA, goodix_set_grip_data },
	{ CMD_SET_GRIP_MODE, goodix_set_grip_mode },
	{ CMD_SET_PALM_MODE, goodix_set_palm_mode },
	{ CMD_SET_NOISE_MODE, goodix_set_noise_mode },
	{ CMD_SET_WATER_MODE, goodix_set_water_mode },
	{ CMD_SET_HEATMAP, goodix_set_heatmap },
	{ CMD_GET_SELF_COMPEN, goodix_get_self_compensation },
	{ CMD_SET_REPORT_RATE, goodix_set_report_rate },
	{ CMD_GET_DUMP_LOG, goodix_get_dump_log },
	{ CMD_SET_FREQ_INDEX, goodix_set_freq_index },
	{ CMD_DISABLE_FILTER, goodix_disable_coor_filter },
	{ CMD_GET_IM_DATA, goodix_get_im_rawdata },
	{ CMD_SET_HSYNC_SPEED, goodix_set_hsync_speed },
	{ CMD_SET_WIRELESS_CHARGE, goodix_set_wireless_charge },
	{ CMD_SET_CONTINUE_HEATMAP, goodix_set_continue_heatmap },
	{ NULL, NULL } };

static int split_string(char *str, char *cmd, int *param, int *param_len)
{
	char buf[150] = { 0 };
	char *token = NULL;
	char *rest = buf;
	int i = 0;
	s32 val;

	strcpy(buf, str);
	if (buf[strlen(buf) - 1] == '\n')
		buf[strlen(buf) - 1] = 0;
	token = strsep(&rest, ",");
	if (token == NULL)
		return -1;
	strcpy(cmd, token);

	while ((token = strsep(&rest, ",")) != NULL) {
		if (kstrtos32(token, 10, &val))
			return -1;
		if (i == *param_len)
			break;
		param[i++] = val;
	}

	*param_len = i;
	return 0;
}

static ssize_t driver_test_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
	struct seq_file *seq_file = (struct seq_file *)file->private_data;
	struct goodix_ts_core *cd = (struct goodix_ts_core *)seq_file->private;
	struct cmd_handler *cmder = &cmd_handler_list[0];
	int ret;
	char *p = wbuf;
	char cmd[64] = { 0 };
	int tx_buf[32] = { 0 };
	int tx_len = 0;

	if (count >= SHORT_SIZE) { /* [GOOG] */
		ts_err("invalid cmd size[%ld]", count);
		return count;
	}

	memset(wbuf, 0, sizeof(wbuf));
	if (copy_from_user(p, buf, count) != 0) {
		ts_err("copy from user failed");
		return count;
	}

	free_proc_buffer(&rbuf);
	index = 0;
	release_test_resource();

	tx_len = 32;
	ret = split_string(p, cmd, tx_buf, &tx_len);
	if (ret < 0) {
		ts_err("invalid cmd: %s", p);
		return count;
	}

	ts_info("input cmd[%s]", p);

	while (cmder->name) {
		if (!strcmp(cmd, cmder->name))
			break;
		cmder++;
	}

	if (cmder->name == NULL) {
		rbuf = malloc_proc_buffer(SHORT_SIZE);
		if (rbuf == NULL) {
			ts_err("failed to alloc rbuf");
			return count;
		}

		index = sprintf(rbuf, "not support cmd %s\n", p);
		ts_err("not support cmd[%s]", p);
		return count;
	}
	if (cmder->handler)
		cmder->handler(cd, tx_buf, tx_len);
	return count;
}

static int cmd_list_show(struct seq_file *m, void *v)
{
	struct cmd_handler *cmder = &cmd_handler_list[0];

	if (!m || !v)
		return -EIO;

	while (cmder->name != NULL) {
		seq_printf(m, "%s\n", cmder->name);
		cmder++;
	}

	return 0;
}

static int cmd_list_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmd_list_show, NULL);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops driver_test_ops = {
	.proc_open = driver_test_open,
	.proc_read = seq_read,
	.proc_write = driver_test_write,
	.proc_lseek = seq_lseek,
	.proc_release = driver_test_release,
};

static const struct proc_ops cmd_list_ops = {
	.proc_open = cmd_list_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations driver_test_ops = {
	.open = driver_test_open,
	.read = seq_read,
	.write = driver_test_write,
	.llseek = seq_lseek,
	.release = driver_test_release,
};

static const struct file_operations cmd_list_ops = {
	.open = cmd_list_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

/* [GOOG] */
int driver_test_selftest(struct goodix_ts_core *cd, char *buf, bool *result,
	bool is_ical)
{
	int ret = 0;

	release_test_resource();
	index = 0;
	free_proc_buffer(&rbuf);

	raw_data_cnt = 16;
	noise_data_cnt = 1;
	rbuf = malloc_proc_buffer(SHORT_SIZE);
	if (rbuf == NULL) {
		ts_err("failed to alloc rbuf");
		ret = -ENOMEM;
		goto exit;
	}
	ret = malloc_test_resource();
	if (ret < 0) {
		ts_err("malloc test resource failed");
		goto exit;
	}

	ts_test->is_ical = is_ical;
	if (is_ical) {
		ts_test->item[GTP_CAP_TEST] = true;
		ts_test->item[GTP_NOISE_TEST] = false;
		ts_test->item[GTP_DELTA_TEST] = false;
		ts_test->item[GTP_SELFCAP_TEST] = false;
		ts_test->item[GTP_SHORT_TEST] = true;
		goodix_auto_test(cd, true);

		*result = ts_test->result[GTP_CAP_TEST] &&
			ts_test->result[GTP_SHORT_TEST];
	} else {
		ts_test->item[GTP_CAP_TEST] = true;
		ts_test->item[GTP_NOISE_TEST] = true;
		ts_test->item[GTP_DELTA_TEST] = true;
		ts_test->item[GTP_SELFCAP_TEST] = true;
		ts_test->item[GTP_SHORT_TEST] = true;

		goodix_auto_test(cd, true);

		*result = ts_test->result[GTP_CAP_TEST] &&
			ts_test->result[GTP_DELTA_TEST] &&
			ts_test->result[GTP_SHORT_TEST] &&
			ts_test->result[GTP_NOISE_TEST] &&
			ts_test->result[GTP_SELFCAP_TEST];
	}

	strlcpy(buf, rbuf, PAGE_SIZE);

exit:
	return ret;
}

int driver_test_proc_init(struct goodix_ts_core *core_data)
{
	struct proc_dir_entry *proc_entry;

	proc_entry = proc_create_data(
		"driver_test", 0660, core_data->proc_dir_entry, &driver_test_ops, core_data);
	if (!proc_entry) {
		ts_err("failed to create proc driver_test");
		return -ENODEV;
	}

	proc_entry =
		proc_create("cmd_list", 0440, core_data->proc_dir_entry, &cmd_list_ops);
	if (!proc_entry) {
		ts_err("failed to create proc cmd_list");
		return -ENODEV;
	}
	return 0;
}

void driver_test_proc_remove(struct goodix_ts_core *core_data)
{
	remove_proc_entry("cmd_list", core_data->proc_dir_entry);
	remove_proc_entry("driver_test", core_data->proc_dir_entry);
}
