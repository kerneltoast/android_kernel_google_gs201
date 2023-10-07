/*
 * Goodix Gesture Module
 *
 * Copyright (C) 2019 - 2020 Goodix, Inc.
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
 */
#ifndef _GOODIX_TS_CORE_H_
#define _GOODIX_TS_CORE_H_
#include <asm/unaligned.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <drm/drm_panel.h>
#if IS_ENABLED(CONFIG_OF)
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
#if IS_ENABLED(CONFIG_FB)
#include <linux/fb.h>
#include <linux/notifier.h>
#endif
#include "touch_apis.h"
#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
#include "touch_mf_mode.h"
#endif
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
#include <goog_touch_interface.h>
#endif
#if IS_ENABLED(CONFIG_VH_SYSTRACE)
#include "../../../gs-google/drivers/soc/google/vh/kernel/systrace.h"
#else
#define ATRACE_BEGIN(f)
#define ATRACE_END()
#endif

#define GOODIX_CORE_DRIVER_NAME "goodix_ts"
#define GOODIX_PEN_DRIVER_NAME "goodix_ts,pen"
#define GOODIX_DRIVER_VERSION "v1.2.4"
#define GOODIX_MAX_TOUCH 10
#define GOODIX_PEN_MAX_PRESSURE 4096
#define GOODIX_MAX_PEN_KEY 2
#define GOODIX_PEN_MAX_TILT 90
#define GOODIX_CFG_MAX_SIZE 4096
#define GOODIX_FW_MAX_SIEZE (300 * 1024)
#define GOODIX_MAX_STR_LABEL_LEN 32
#define GOODIX_MAX_FRAMEDATA_LEN (3 * 1024)
#define GOODIX_GESTURE_DATA_LEN 16
#define GOODIX_REQUEST_DATA_LEN 16
#define GOODIX_NORMAL_RESET_DELAY_MS 100
#define GOODIX_HOLD_CPU_RESET_DELAY_MS 5

#define GOODIX_RETRY_3 3
#define GOODIX_RETRY_5 5
#define GOODIX_RETRY_10 10

#define GOODIX_GESTURE_UNKNOWN 0x00
#define GOODIX_GESTURE_DOUBLE_TAP 0xCC
#define GOODIX_GESTURE_SINGLE_TAP 0x4C
#define GOODIX_GESTURE_FOD_DOWN 0x46
#define GOODIX_GESTURE_FOD_UP 0x55

#define TS_DEFAULT_FIRMWARE "goodix_firmware.bin"
#define TS_DEFAULT_CFG_BIN "goodix_cfg_group.bin"
#define TS_DEFAULT_TEST_LIMITS "goodix_test_limits_255.csv"

enum GOODIX_GESTURE_TYP {
	GESTURE_SINGLE_TAP = (1 << 0),
	GESTURE_DOUBLE_TAP = (1 << 1),
	GESTURE_FOD_PRESS = (1 << 2)
};

enum CORD_PROB_STA {
	CORE_MODULE_UNPROBED = 0,
	CORE_MODULE_PROB_SUCCESS = 1,
	CORE_MODULE_PROB_FAILED = -1,
	CORE_MODULE_REMOVED = -2,
};

enum GOODIX_ERR_CODE {
	GOODIX_EBUS = (1 << 0),
	GOODIX_ECHECKSUM = (1 << 1),
	GOODIX_EVERSION = (1 << 2),
	GOODIX_ETIMEOUT = (1 << 3),
	GOODIX_EMEMCMP = (1 << 4),

	GOODIX_EOTHER = (1 << 7)
};

/* MAIN-ID */
enum IC_TYPE_ID {
	IC_TYPE_NONE,
	IC_TYPE_NORMANDY,
	IC_TYPE_NANJING,
	IC_TYPE_YELLOWSTONE,
	IC_TYPE_BERLIN_A,
	IC_TYPE_BERLIN_B,
	IC_TYPE_BERLIN_D,
	IC_TYPE_NOTTINGHAM
};

/* SUB-ID
 * sub type of berlinB serial IC.
 * for convenience we put the MAIN-ID on the hith bits,
 * hith 8 bits is MAIN-ID, low 8 bits is MIN-ID
 */
enum BERLIN_B_SUB_ID {
	IC_TYPE_SUB_B2 = (IC_TYPE_BERLIN_B << 8) | 0x2,
};

enum GOODIX_IC_CONFIG_TYPE {
	CONFIG_TYPE_TEST = 0,
	CONFIG_TYPE_NORMAL = 1,
	CONFIG_TYPE_HIGHSENSE = 2,
	CONFIG_TYPE_CHARGER = 3,
	CONFIG_TYPE_CHARGER_HS = 4,
	CONFIG_TYPE_HOLSTER = 5,
	CONFIG_TYPE_HOSTER_CH = 6,
	CONFIG_TYPE_OTHER = 7,
	/* keep this at the last */
	GOODIX_MAX_CONFIG_GROUP = 8,
};

enum CHECKSUM_MODE {
	CHECKSUM_MODE_U8_LE,
	CHECKSUM_MODE_U16_LE,
};

enum PINCTRL_MODE {
	PINCTRL_MODE_ACTIVE,
	PINCTRL_MODE_SUSPEND,
};

enum raw_scan_mode : u8 {
	RAW_SCAN_MODE_AUTO = 0,
	RAW_SCAN_MODE_NORMAL_ACTIVE,
	RAW_SCAN_MODE_NORMAL_IDLE,
	RAW_SCAN_MODE_LOW_POWER_ACTIVE,
	RAW_SCAN_MODE_LOW_POWER_IDLE,
	RAW_SCAN_MODE_SLEEP,
};

enum frame_data_type : u8 {
	FRAME_DATA_TYPE_RAW = 0x81,
	FRAME_DATA_TYPE_DIFF = 0x82,
	FRAME_DATA_TYPE_BASE = 0x83,
};

#define MAX_SCAN_FREQ_NUM 8
#define MAX_SCAN_RATE_NUM 8
#define MAX_FREQ_NUM_STYLUS 8
#define MAX_STYLUS_SCAN_FREQ_NUM 6
#pragma pack(1)
struct frame_head {
	uint8_t sync;
	uint16_t frame_index;
	uint16_t cur_frame_len;
	uint16_t next_frame_len;
	uint32_t data_en; /* 0- 7 for pack_en; 8 - 31 for type en */
	uint8_t touch_pack_index;
	uint8_t stylus_pack_index;
	uint8_t res;
	uint16_t checksum;
};

struct goodix_fw_version {
	u8 rom_pid[6]; /* rom PID */
	u8 rom_vid[3]; /* Mask VID */
	u8 rom_vid_reserved;
	u8 patch_pid[8]; /* Patch PID */
	u8 patch_vid[4]; /* Patch VID */
	u8 patch_vid_reserved;
	u8 sensor_id;
	u8 reserved[2];
	u16 checksum;
};

struct goodix_ic_info_version {
	u8 info_customer_id;
	u8 info_version_id;
	u8 ic_die_id;
	u8 ic_version_id;
	u32 config_id;
	u8 config_version;
	u8 frame_data_customer_id;
	u8 frame_data_version_id;
	u8 touch_data_customer_id;
	u8 touch_data_version_id;
	u8 reserved[3];
};

struct goodix_ic_info_feature { /* feature info*/
	u16 freqhop_feature;
	u16 calibration_feature;
	u16 gesture_feature;
	u16 side_touch_feature;
	u16 stylus_feature;
};

struct goodix_ic_info_param { /* param */
	u8 drv_num;
	u8 sen_num;
	u8 button_num;
	u8 force_num;
	u8 active_scan_rate_num;
	u16 active_scan_rate[MAX_SCAN_RATE_NUM];
	u8 mutual_freq_num;
	u16 mutual_freq[MAX_SCAN_FREQ_NUM];
	u8 self_tx_freq_num;
	u16 self_tx_freq[MAX_SCAN_FREQ_NUM];
	u8 self_rx_freq_num;
	u16 self_rx_freq[MAX_SCAN_FREQ_NUM];
	u8 stylus_freq_num;
	u16 stylus_freq[MAX_FREQ_NUM_STYLUS];
};

struct goodix_ic_info_misc { /* other data */
	u32 cmd_addr;
	u16 cmd_max_len;
	u32 cmd_reply_addr;
	u16 cmd_reply_len;
	u32 fw_state_addr;
	u16 fw_state_len;
	u32 fw_buffer_addr;
	u16 fw_buffer_max_len;
	u32 frame_data_addr;
	u16 frame_data_head_len;
	u16 fw_attr_len;
	u16 fw_log_len;
	u8 pack_max_num;
	u8 pack_compress_version;
	u16 stylus_struct_len;
	u16 mutual_struct_len;
	u16 self_struct_len;
	u16 noise_struct_len;
	u32 touch_data_addr;
	u16 touch_data_head_len;
	u16 point_struct_len;
	u16 reserved1;
	u16 reserved2;
	u32 mutual_rawdata_addr;
	u32 mutual_diffdata_addr;
	u32 mutual_refdata_addr;
	u32 self_rawdata_addr;
	u32 self_diffdata_addr;
	u32 self_refdata_addr;
	u32 iq_rawdata_addr;
	u32 iq_refdata_addr;
	u32 im_rawdata_addr;
	u16 im_readata_len;
	u32 noise_rawdata_addr;
	u16 noise_rawdata_len;
	u32 stylus_rawdata_addr;
	u16 stylus_rawdata_len;
	u32 noise_data_addr;
	u32 esd_addr;
	u32 auto_scan_cmd_addr;
	u32 auto_scan_info_addr;
};

struct goodix_ic_info {
	u16 length;
	struct goodix_ic_info_version version;
	struct goodix_ic_info_feature feature;
	struct goodix_ic_info_param parm;
	struct goodix_ic_info_misc misc;
};
#pragma pack()

/*
 * struct ts_rawdata_info
 *
 */
#define TS_RAWDATA_BUFF_MAX 7000
#define TS_RAWDATA_RESULT_MAX 100
struct ts_rawdata_info {
	int used_size; // fill in rawdata size
	s16 buff[TS_RAWDATA_BUFF_MAX];
	char result[TS_RAWDATA_RESULT_MAX];
};

/*
 * struct goodix_module - external modules container
 * @head: external modules list
 * @initialized: whether this struct is initialized
 * @mutex: mutex lock
 * @wq: workqueue to do register work
 * @core_data: core_data pointer
 */
struct goodix_module {
	struct list_head head;
	bool initialized;
	struct mutex mutex;
	struct workqueue_struct *wq;
	struct goodix_ts_core *core_data;
};

/*
 * struct goodix_ts_board_data -  board data
 * @avdd_name: name of analoy regulator
 * @iovdd_name: name of analoy regulator
 * @reset_gpio: reset gpio number
 * @irq_gpio: interrupt gpio number
 * @irq_flag: irq trigger type
 * @swap_axis: whether swaw x y axis
 * @panel_max_x/y/w/p: resolution and size
 * @panel_height_mm: the height of display in mm
 * @pannel_key_map: key map
 * @fw_name: name of the firmware image
 */
struct goodix_ts_board_data {
	char avdd_name[GOODIX_MAX_STR_LABEL_LEN];
	char iovdd_name[GOODIX_MAX_STR_LABEL_LEN];
	int reset_gpio;
	int irq_gpio;
	int avdd_gpio;
	int iovdd_gpio;
	unsigned int irq_flags;

	struct pinctrl *pinctrl;
	struct pinctrl_state *state_active;
	struct pinctrl_state *state_suspend;

	unsigned int swap_axis;
	unsigned int panel_max_x;
	unsigned int panel_max_y;
	unsigned int panel_max_w; /*major and minor*/
	unsigned int panel_max_p; /*pressure*/
	unsigned int panel_height_mm;
	unsigned int udfps_x;
	unsigned int udfps_y;

	bool pen_enable;
	bool sleep_enable;
	bool use_one_binary;
	char fw_name[GOODIX_MAX_STR_LABEL_LEN];
	char cfg_bin_name[GOODIX_MAX_STR_LABEL_LEN];
	char test_limits_name[GOODIX_MAX_STR_LABEL_LEN];
};

enum goodix_fw_update_mode {
	UPDATE_MODE_DEFAULT = 0,
	UPDATE_MODE_FORCE = (1 << 0),	    /* force update mode */
	UPDATE_MODE_BLOCK = (1 << 1),	    /* update in block mode */
	UPDATE_MODE_FLASH_CFG = (1 << 2),   /* reflash config */
	UPDATE_MODE_SRC_SYSFS = (1 << 4),   /* firmware file from sysfs */
	UPDATE_MODE_SRC_HEAD = (1 << 5),    /* firmware file from head file */
	UPDATE_MODE_SRC_REQUEST = (1 << 6), /* request firmware */
	UPDATE_MODE_SRC_ARGS = (1 << 7), /* firmware data from function args */
};

#define MAX_CMD_DATA_LEN 10
#define MAX_CMD_BUF_LEN 16
#pragma pack(1)
struct goodix_ts_cmd {
	union {
		struct {
			u8 state;
			u8 ack;
			u8 len;
			u8 cmd;
			u8 data[MAX_CMD_DATA_LEN];
		};
		u8 buf[MAX_CMD_BUF_LEN];
	};
};

struct goodix_status_data {
	u8 water_change : 1;
	u8 hop_change : 1;
	u8 base_update : 1;
	u8 soft_reset : 1;
	u8 palm_change : 1;
	u8 noise_lv_change : 1;
	u8 grip_change : 1;
	u8 water_sta;
	u8 before_factorA;
	u8 after_factorA;
	u8 base_update_type;
	u8 soft_reset_type;
	u8 palm_sta;
	u8 noise_lv;
	u8 grip_type;
	u8 res[9];
	u8 event_id;
	u8 checksum;
};

struct goodix_stylus_data {
	u8 stylus_protocol;
	u8 sample_mode;
	u8 stylus_key;
	u16 stylus_pressure;
	u16 stylus_freqA;
	u16 stylus_freqB;
	u8 res;
	u16 stylus_next_freqA;
	u16 stylus_next_freqB;
	u16 stylus_noise_value[4];
	s16 angle_coord_x;
	s16 angle_coord_y;
	s16 delta_x;
	s16 delta_y;
	u8 freq_indexA;
	u8 freq_indexB;
	u16 tx1[32];
	u16 rx1[39];
	u16 tx2[32];
	u16 rx2[39];
};
#pragma pack()

/* interrupt event type */
enum ts_event_type {
	EVENT_INVALID = 0,
	EVENT_TOUCH = (1 << 0), /* finger touch event */
	EVENT_PEN = (1 << 1),	/* pen event */
	EVENT_REQUEST = (1 << 2),
	EVENT_GESTURE = (1 << 3),
	EVENT_STATUS = (1 << 4),
};

enum ts_request_type {
	REQUEST_TYPE_CONFIG = 1,
	REQUEST_TYPE_RESET = 3,
	REQUEST_TYPE_UPDATE = 5,
	REQUEST_PEN_FREQ_HOP = 0x10
};

/* notifier event */
enum ts_notify_event {
	NOTIFY_FWUPDATE_START,
	NOTIFY_FWUPDATE_FAILED,
	NOTIFY_FWUPDATE_SUCCESS,
	NOTIFY_SUSPEND,
	NOTIFY_RESUME,
	NOTIFY_ESD_OFF,
	NOTIFY_ESD_ON,
	NOTIFY_CFG_BIN_FAILED,
	NOTIFY_CFG_BIN_SUCCESS,
};

enum touch_point_status {
	TS_NONE,
	TS_RELEASE,
	TS_TOUCH,
};
/* coordinate package */
struct goodix_ts_coords {
	int status; /* NONE, RELEASE, TOUCH */
	unsigned int x, y, major, minor, p, w;
	signed char angle;
};

struct goodix_pen_coords {
	int status;    /* NONE, RELEASE, TOUCH */
	int tool_type; /* BTN_TOOL_RUBBER BTN_TOOL_PEN */
	unsigned int x, y, p;
	signed char tilt_x;
	signed char tilt_y;
};

/* touch event data */
struct goodix_touch_data {
	int touch_num;
	struct goodix_ts_coords coords[GOODIX_MAX_TOUCH];
};

/* gesture event data */
struct goodix_gesture_data {
	u8 gesture_type;
	int touches;
	u8 data[GOODIX_GESTURE_DATA_LEN];
};

struct goodix_ts_key {
	int status;
	int code;
};

struct goodix_pen_data {
	struct goodix_pen_coords coords;
	struct goodix_ts_key keys[GOODIX_MAX_PEN_KEY];
};

/*
 * struct goodix_ts_event - touch event struct
 * @clear_count1: clear count for old firmware
 * @clear_count2: clear count for latest firmware
 * @event_type: touch event type, touch data or
 *	request event
 * @event_data: event data
 */
struct goodix_ts_event {
	enum ts_event_type event_type;
	u8 clear_count1;
	u8 clear_count2;
	u8 fp_flag;	 /* finger print DOWN flag */
	u8 request_code; /* represent the request type */
	u8 request_data[GOODIX_REQUEST_DATA_LEN];
	struct goodix_gesture_data gesture_data;
	struct goodix_touch_data touch_data;
	struct goodix_pen_data pen_data;
	struct goodix_status_data status_data;
};

struct goodix_ts_event_data {
	u8 reserved1 : 1;
	u8 status_changed : 1;
	u8 reserved2 : 1;
	u8 fp_flag : 1;
	u8 type : 4;
	u8 int_count;
	u8 reserved3;
	u8 reserved4 : 4;
	u8 clear_count1 : 4;
	u8 reserved5;
	u8 reserved6 : 4;
	u8 clear_count2 : 4;
};

struct goodix_ts_request_event_data {
	u8 reserved1 : 1;
	u8 status_changed : 1;
	u8 reserved2 : 1;
	u8 fp_flag : 1;
	u8 type : 4;
	u8 int_count;
	u8 request_type;
	u8 reserved3;
	u8 reserved4;
	u8 data_len;
	u16 checksum;
	u8 data[GOODIX_REQUEST_DATA_LEN];
};

struct goodix_ts_touch_event_data {
	u8 reserved1 : 1;
	u8 status_changed : 1;
	u8 reserved2 : 1;
	u8 fp_flag : 1;
	u8 type : 4;
	u8 int_count;
	u8 touches : 4;
	u8 large_touch : 1;
	u8 hover_approach_flag : 1;
	u8 edge_flag : 1;
	u8 reset_int : 1;
	u8 custom_coor_info_flag : 1;
	u8 reserved3 : 3;
	u8 clear_count1 : 4;
	u8 reserved4;
	u8 reserved5 : 4;
	u8 clear_count2 : 4;
	u16 checksum;
	u8 data[0];
};

struct goodix_ts_gesture_event_data {
	u8 reserved1 : 1;
	u8 status_changed : 1;
	u8 reserved2 : 1;
	u8 fp_flag : 1;
	u8 type : 4;
	u8 int_count;
	u8 reserved3 : 4;
	u8 large_touch : 1;
	u8 hover_approach_flag : 1;
	u8 edge_flag : 1;
	u8 reset_int : 1;
	u8 touches;
	u8 gesture_type;
	u8 reserved4;
	u16 checksum;
	u8 data[GOODIX_GESTURE_DATA_LEN];
};

struct goodix_mutual_data {
	uint16_t duration;
	uint16_t tx1_freq;
	uint16_t tx2_freq;
	uint16_t res;
	uint16_t data[0];
};

struct goodix_self_sensing_data {
	uint16_t tx_duration;
	uint16_t rx_duration;
	uint16_t tx_freq;
	uint16_t rx_freq;
	uint16_t res;
	uint16_t data[0];
};

struct goodix_rx_package {
	uint8_t header[8];
	uint16_t data[0];
};

enum goodix_ic_bus_type {
	GOODIX_BUS_TYPE_I2C,
	GOODIX_BUS_TYPE_SPI,
	GOODIX_BUS_TYPE_I3C,
};

struct goodix_bus_interface {
	int bus_type;
	int ic_type;
	int sub_ic_type;
	struct device *dev;
	u8 *rx_buf;
	u8 *tx_buf;
	struct mutex mutex;
	bool dma_mode_enabled;
	int (*read)(struct device *dev, unsigned int addr, unsigned char *data,
		unsigned int len);
	int (*read_fast)(struct device *dev, unsigned int addr,
		struct goodix_rx_package *package, unsigned int len);
	int (*write)(struct device *dev, unsigned int addr, unsigned char *data,
		unsigned int len);
};

struct goodix_ts_hw_ops {
	int (*power_on)(struct goodix_ts_core *cd, bool on);
	int (*resume)(struct goodix_ts_core *cd);
	int (*suspend)(struct goodix_ts_core *cd);
	int (*gesture)(struct goodix_ts_core *cd, int gesture_type);
	int (*reset)(struct goodix_ts_core *cd, int delay_ms);
	int (*irq_enable)(struct goodix_ts_core *cd, bool enable);
	int (*disable_irq_nosync)(struct goodix_ts_core *cd);
	int (*read)(struct goodix_ts_core *cd, unsigned int addr,
		unsigned char *data, unsigned int len);
	int (*read_fast)(struct goodix_ts_core *cd, unsigned int addr,
		struct goodix_rx_package *package, unsigned int len);
	int (*write)(struct goodix_ts_core *cd, unsigned int addr,
		unsigned char *data, unsigned int len);
	int (*send_cmd)(struct goodix_ts_core *cd, struct goodix_ts_cmd *cmd);
	int (*send_config)(struct goodix_ts_core *cd, u8 *config, int len);
	int (*read_config)(
		struct goodix_ts_core *cd, u8 *config_data, int size);
	int (*read_version)(
		struct goodix_ts_core *cd, struct goodix_fw_version *version);
	int (*get_ic_info)(
		struct goodix_ts_core *cd, struct goodix_ic_info *ic_info);
	int (*esd_check)(struct goodix_ts_core *cd);
	int (*event_handler)(
		struct goodix_ts_core *cd, struct goodix_ts_event *ts_event);
	int (*after_event_handler)(struct goodix_ts_core *cd);
	int (*get_capacitance_data)(
		struct goodix_ts_core *cd, struct ts_rawdata_info *info);
	int (*ping)(struct goodix_ts_core *cd);
	int (*get_scan_mode)(struct goodix_ts_core *cd, enum raw_scan_mode* mode);
	int (*set_scan_mode)(struct goodix_ts_core *cd, enum raw_scan_mode mode);
	int (*set_continuously_report_enabled)(
		struct goodix_ts_core *cd, bool enabled);
	int (*set_heatmap_enabled)(struct goodix_ts_core *cd, bool enabled);
	int (*set_palm_enabled)(struct goodix_ts_core *cd, bool enabled);
	int (*get_palm_enabled)(struct goodix_ts_core *cd, bool* enabled);
	int (*set_grip_enabled)(struct goodix_ts_core *cd, bool enabled);
	int (*get_grip_enabled)(struct goodix_ts_core *cd, bool* enabled);
	int (*set_screen_protector_mode_enabled)(
		struct goodix_ts_core *cd, bool enabled);
	int (*get_screen_protector_mode_enabled)(
		struct goodix_ts_core *cd, bool* enabled);
	int (*get_mutual_data)(
		struct goodix_ts_core *cd, enum frame_data_type type);
	int (*get_self_sensing_data)(
		struct goodix_ts_core *cd, enum frame_data_type type);
	int (*set_coord_filter_enabled)(
		struct goodix_ts_core *cd, bool enabled);
	int (*get_coord_filter_enabled)(
		struct goodix_ts_core *cd, bool* enabled);
	int (*set_report_rate)(struct goodix_ts_core *cd, u32 rate);
};

/*
 * struct goodix_ts_esd - esd protector structure
 * @esd_work: esd delayed work
 * @esd_on: 1 - turn on esd protection, 0 - turn
 *  off esd protection
 * @skip_once: skip once if the check is no need this time.
 */
struct goodix_ts_esd {
	bool skip_once;
	atomic_t esd_on;
	struct delayed_work esd_work;
	struct notifier_block esd_notifier;
	struct goodix_ts_core *ts_core;
};

enum goodix_core_init_stage {
	CORE_UNINIT,
	CORE_INIT_FAIL,
	CORE_INIT_STAGE1,
	CORE_INIT_STAGE2
};

struct goodix_ic_config {
	int len;
	u8 data[GOODIX_CFG_MAX_SIZE];
};

struct goodix_ts_core {
	int init_stage;
	struct platform_device *pdev;
	struct goodix_fw_version fw_version;
	struct goodix_ic_info ic_info;
	struct goodix_bus_interface *bus;
	struct goodix_ts_board_data board_data;
	struct touch_apis_data apis_data;
	struct goodix_ts_hw_ops *hw_ops;
	struct input_dev *input_dev;
	struct input_dev *pen_dev;
	struct mutex cmd_lock;
	/* TODO counld we remove this from core data? */
	struct goodix_ts_event ts_event;
	struct workqueue_struct *event_wq;
	struct delayed_work monitor_gesture_work;
	ktime_t gesture_down_timeout;
	ktime_t gesture_up_timeout;

	/* every pointer of this array represent a kind of config */
	struct goodix_ic_config *ic_configs[GOODIX_MAX_CONFIG_GROUP];
	struct regulator *avdd;
	struct regulator *iovdd;
	unsigned char gesture_type;
	struct goodix_rx_package *touch_frame_package;
	size_t touch_frame_size;
	uint16_t *mutual_data;
	uint16_t *self_sensing_data;
	uint16_t *mutual_data_manual;
	uint16_t *self_sensing_data_manual;

	int power_on;
	int irq;
	size_t irq_trig_cnt;

	atomic_t irq_enabled;
	atomic_t suspended;
	bool screen_protector_mode_enabled;
	/* when this flag is true, driver should not clean the sync flag */
	bool tools_ctrl_sync;

	struct notifier_block ts_notifier;
	struct goodix_ts_esd ts_esd;

#if IS_ENABLED(CONFIG_FB)
	struct notifier_block fb_notifier;
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
	struct touch_mf tmf;
#endif
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	struct goog_touch_interface *gti;
#endif

	/* Time that the event was first received from the touch IC,
	 * acquired during hard interrupt, in CLOCK_MONOTONIC.
	 */
	ktime_t isr_timestamp;
	ktime_t coords_timestamp;
};

/* external module structures */
enum goodix_ext_priority {
	EXTMOD_PRIO_RESERVED = 0,
	EXTMOD_PRIO_FWUPDATE,
	EXTMOD_PRIO_GESTURE,
	EXTMOD_PRIO_HOTKNOT,
	EXTMOD_PRIO_DBGTOOL,
	EXTMOD_PRIO_DEFAULT,
};

#define EVT_HANDLED 0
#define EVT_CONTINUE 0
#define EVT_CANCEL 1
#define EVT_CANCEL_IRQEVT 1
#define EVT_CANCEL_SUSPEND 1
#define EVT_CANCEL_RESUME 1
#define EVT_CANCEL_RESET 1

struct goodix_ext_module;
/* external module's operations callback */
struct goodix_ext_module_funcs {
	int (*init)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
	int (*exit)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
	int (*before_reset)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
	int (*after_reset)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
	int (*before_suspend)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
	int (*after_suspend)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
	int (*before_resume)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
	int (*after_resume)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
	int (*irq_event)(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module);
};

/*
 * struct goodix_ext_module - external module struct
 * @list: list used to link into modules manager
 * @name: name of external module
 * @priority: module priority value, zero is invalid
 * @funcs: operations callback
 * @priv_data: private data region
 * @kobj: kobject
 * @work: used to queue one work to do registration
 */
struct goodix_ext_module {
	struct list_head list;
	char *name;
	enum goodix_ext_priority priority;
	const struct goodix_ext_module_funcs *funcs;
	void *priv_data;
	struct kobject kobj;
	struct work_struct work;
};

/*
 * struct goodix_ext_attribute - exteranl attribute struct
 * @attr: attribute
 * @show: show interface of external attribute
 * @store: store interface of external attribute
 */
struct goodix_ext_attribute {
	struct attribute attr;
	ssize_t (*show)(struct goodix_ext_module *module, char *buf);
	ssize_t (*store)(
		struct goodix_ext_module *module, const char *buf, size_t len);
};

/* external attrs helper macro */
#define __EXTMOD_ATTR(_name, _mode, _show, _store)                             \
	{                                                                      \
		.attr = { .name = __stringify(_name), .mode = _mode },         \
		.show = _show, .store = _store,                                \
	}

/* external attrs helper macro, used to define external attrs */
#define DEFINE_EXTMOD_ATTR(_name, _mode, _show, _store)                        \
	static struct goodix_ext_attribute ext_attr_##_name =                  \
		__EXTMOD_ATTR(_name, _mode, _show, _store)

/* log macro */
extern bool debug_log_flag;
#define ts_info(fmt, arg...)                                                   \
	pr_info("[GTP-INF][%s:%d] " fmt "\n", __func__, __LINE__, ##arg)
#define ts_err(fmt, arg...)                                                    \
	pr_err("[GTP-ERR][%s:%d] " fmt "\n", __func__, __LINE__, ##arg)
#define ts_debug(fmt, arg...)                                                  \
	{                                                                      \
		if (debug_log_flag)                                            \
			pr_info("[GTP-DBG][%s:%d] " fmt "\n", __func__,        \
				__LINE__, ##arg);                              \
	}

/*
 * get board data pointer
 */
static inline struct goodix_ts_board_data *board_data(
	struct goodix_ts_core *core)
{
	if (!core)
		return NULL;
	return &(core->board_data);
}

/**
 * goodix_register_ext_module - interface for external module
 * to register into touch core modules structure
 *
 * @module: pointer to external module to be register
 * return: 0 ok, <0 failed
 */
int goodix_register_ext_module(struct goodix_ext_module *module);
/* register module no wait */
int goodix_register_ext_module_no_wait(struct goodix_ext_module *module);
/**
 * goodix_unregister_ext_module - interface for external module
 * to unregister external modules
 *
 * @module: pointer to external module
 * return: 0 ok, <0 failed
 */
int goodix_unregister_ext_module(struct goodix_ext_module *module);
/* remove all registered ext module
 * return 0 on success, otherwise return < 0
 */
int goodix_ts_blocking_notify(enum ts_notify_event evt, void *v);
struct kobj_type *goodix_get_default_ktype(void);
struct kobject *goodix_get_default_kobj(void);

struct goodix_ts_hw_ops *goodix_get_hw_ops(void);
int goodix_get_config_proc(struct goodix_ts_core *cd);

int goodix_spi_bus_init(void);
void goodix_spi_bus_exit(void);
int goodix_i2c_bus_init(void);
void goodix_i2c_bus_exit(void);

u32 goodix_append_checksum(u8 *data, int len, int mode);
int checksum_cmp(const u8 *data, int size, int mode);
int is_risk_data(const u8 *data, int size);
u32 goodix_get_file_config_id(u8 *ic_config);
void goodix_rotate_abcd2cbad(int tx, int rx, s16 *src, s16 *dest);

int goodix_fw_update_init(struct goodix_ts_core *core_data);
void goodix_fw_update_uninit(void);
int goodix_do_fw_update(struct goodix_ic_config *ic_config, int mode);

int goodix_get_ic_type(
	struct device_node *node, struct goodix_bus_interface *bus_inf);
int gesture_module_init(void);
void gesture_module_exit(void);
int inspect_module_init(void);
void inspect_module_exit(void);
int goodix_tools_init(void);
void goodix_tools_exit(void);

int driver_test_selftest(char* buf);
int driver_test_proc_init(struct goodix_ts_core *core_data);
void driver_test_proc_remove(void);
int goodix_do_inspect(struct goodix_ts_core *cd, struct ts_rawdata_info *info);
void goodix_ts_report_status(struct goodix_ts_core *core_data,
	struct goodix_ts_event *ts_event);
int goodix_update_pen_freq(struct goodix_ts_core *cd, u8 *data, int len);

#endif
