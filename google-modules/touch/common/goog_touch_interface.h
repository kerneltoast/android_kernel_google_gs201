/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Touch Interface for Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#ifndef _GOOG_TOUCH_INTERFACE_
#define _GOOG_TOUCH_INTERFACE_

#include <drm/drm_panel.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <linux/kfifo.h>
#include <linux/pm_qos.h>

#include "heatmap.h"
#include "touch_offload.h"
#include "uapi/input/touch_offload.h"

#define GTI_NAME "goog_touch_interface"
#define GOOG_LOG_NAME(gti) ((gti && gti->dev) ? dev_name(gti->dev) : "gti")
#define GOOG_DBG(gti, fmt, args...)  pr_debug("%s: " fmt, GOOG_LOG_NAME(gti), ##args)
#define GOOG_INFO(gti, fmt, args...) pr_info("%s: " fmt, GOOG_LOG_NAME(gti), ##args)
#define GOOG_WARN(gti, fmt, args...) pr_warn("%s: " fmt, GOOG_LOG_NAME(gti), ##args)
#define GOOG_ERR(gti, fmt, args...)  pr_err("%s: " fmt, GOOG_LOG_NAME(gti), ##args)
#define GOOG_LOGD(gti, fmt, args...) GOOG_DBG(gti, "%s: "fmt, __func__, ##args)
#define GOOG_LOGI(gti, fmt, args...) GOOG_INFO(gti, "%s: "fmt, __func__, ##args)
#define GOOG_LOGW(gti, fmt, args...) GOOG_WARN(gti, "%s: "fmt, __func__, ##args)
#define GOOG_LOGE(gti, fmt, args...) GOOG_ERR(gti, "%s: "fmt, __func__, ##args)
#define MAX_SLOTS 10
/*
 * GTI_DEBUG_HEALTHCHECK_KFIFO_LEN
 * Define the array length of struct gti_debug_healthcheck to track recent
 * touch interrupts information for debug.
 */
#define GTI_DEBUG_HEALTHCHECK_KFIFO_LEN 32	/* must be power of 2. */
#define GTI_DEBUG_HEALTHCHECK_LOGS_LEN 4
/*
 * GTI_DEBUG_INPUT_KFIFO_LEN
 * Define the array length of struct gti_debug_input to track recent
 * touch input report for debug.
 */
#define GTI_DEBUG_INPUT_KFIFO_LEN 16	/* must be power of 2. */
#define GTI_DEBUG_INPUT_LOGS_LEN 4

/*-----------------------------------------------------------------------------
 * Interactive calibration minimum and maximum state times.
 */

#define S_IN_NS 1000000000ULL  /* 1s = 1e9ns */

/* Must wait at least 5s before beginning any operation */
#define MIN_DELAY_IDLE		(5 * S_IN_NS)

/* Must wait at least 1s between screen-turning-off and calibration start, and
 * cannot stay in this state longer than 15s.
 */
#define MIN_DELAY_INIT_CAL	(1 * S_IN_NS)
#define MAX_DELAY_INIT_CAL	(15 * S_IN_NS)

/* Must wait at least 3s for calibration to complete and cannot stay in this
 * state longer than 15s.
 */
#define MIN_DELAY_RUN_CAL	(3 * S_IN_NS)
#define MAX_DELAY_RUN_CAL	(15 * S_IN_NS)

/* Must wait at least 1s before final return to idle and cannot stay in this
 * state longer than 15s.
 */
#define MIN_DELAY_END_CAL	(1 * S_IN_NS)
#define MAX_DELAY_END_CAL	(15 * S_IN_NS)

/* Must wait at least 1s between screen-turning-off and self-test start, and
 * cannot state in this state longer than 15s.
 */
#define MIN_DELAY_INIT_TEST	(1 * S_IN_NS)
#define MAX_DELAY_INIT_TEST	(15 * S_IN_NS)

/* Must wait at least 3s for self-test to complete and cannot stay in this state
 * longer than 15s.
 */
#define MIN_DELAY_RUN_TEST	(3 * S_IN_NS)
#define MAX_DELAY_RUN_TEST	(15 * S_IN_NS)

/* Must wait at least 1s before final return to idle and cannot stay in this
 * state longer than 15s.
 */
#define MIN_DELAY_END_TEST	(1 * S_IN_NS)
#define MAX_DELAY_END_TEST	(15 * S_IN_NS)

/* Must wait at least 1s before beginning reset operation and cannot stay in this state
 * longer than 15s.
 */
#define MIN_DELAY_INIT_RESET	(1 * S_IN_NS)
#define MAX_DELAY_INIT_RESET	(15 * S_IN_NS)
#define MIN_DELAY_RUN_RESET	(1 * S_IN_NS)
#define MAX_DELAY_RUN_RESET	(15 * S_IN_NS)
#define MIN_DELAY_END_RESET	(1 * S_IN_NS)
#define MAX_DELAY_END_RESET	(15 * S_IN_NS)

/*-----------------------------------------------------------------------------
 * enums.
 */

enum gti_cmd_type : u32 {
	/* GTI_CMD operations. */
	GTI_CMD_OPS_START = 0x100,
	GTI_CMD_CALIBRATE,
	GTI_CMD_PING,
	GTI_CMD_RESET,
	GTI_CMD_SELFTEST,

	/* GTI_CMD_GET operations. */
	GTI_CMD_GET_OPS_START = 0x200,
	GTI_CMD_GET_CONTEXT_DRIVER,
	GTI_CMD_GET_CONTEXT_STYLUS,
	GTI_CMD_GET_COORD_FILTER_ENABLED,
	GTI_CMD_GET_FW_VERSION,
	GTI_CMD_GET_GRIP_MODE,
	GTI_CMD_GET_IRQ_MODE,
	GTI_CMD_GET_PALM_MODE,
	GTI_CMD_GET_SCAN_MODE,
	GTI_CMD_GET_SCREEN_PROTECTOR_MODE,
	GTI_CMD_GET_SENSING_MODE,
	GTI_CMD_GET_SENSOR_DATA,
	GTI_CMD_GET_SENSOR_DATA_MANUAL,

	/* GTI_CMD_NOTIFY operations. */
	GTI_CMD_NOTIFY_OPS_START = 0x300,
	GTI_CMD_NOTIFY_DISPLAY_STATE,
	GTI_CMD_NOTIFY_DISPLAY_VREFRESH,

	/* GTI_CMD_SET operations. */
	GTI_CMD_SET_OPS_START = 0x400,
	GTI_CMD_SET_CONTINUOUS_REPORT,
	GTI_CMD_SET_COORD_FILTER_ENABLED,
	GTI_CMD_SET_GESTURE_CONFIG,
	GTI_CMD_SET_GRIP_MODE,
	GTI_CMD_SET_HEATMAP_ENABLED,
	GTI_CMD_SET_IRQ_MODE,
	GTI_CMD_SET_PALM_MODE,
	GTI_CMD_SET_PANEL_SPEED_MODE,
	GTI_CMD_SET_REPORT_RATE,
	GTI_CMD_SET_SCAN_MODE,
	GTI_CMD_SET_SCREEN_PROTECTOR_MODE,
	GTI_CMD_SET_SENSING_MODE,
};

enum gti_calibrate_result : u32 {
	GTI_CALIBRATE_RESULT_DONE = 0,
	GTI_CALIBRATE_RESULT_SHELL_CMDS_REDIRECT,
	GTI_CALIBRATE_RESULT_FAIL,
	GTI_CALIBRATE_RESULT_NA = 0xFFFFFFFF,
};

enum gti_continuous_report_setting : u32 {
	GTI_CONTINUOUS_REPORT_DISABLE = 0,
	GTI_CONTINUOUS_REPORT_ENABLE,
	GTI_CONTINUOUS_REPORT_DRIVER_DEFAULT,
};

enum gti_coord_filter_setting : u32 {
	GTI_COORD_FILTER_DISABLE = 0,
	GTI_COORD_FILTER_ENABLE,
};

enum gti_display_state_setting : u32 {
	GTI_DISPLAY_STATE_OFF = 0,
	GTI_DISPLAY_STATE_ON,
};

enum gti_grip_setting : u32 {
	GTI_GRIP_DISABLE = 0,
	GTI_GRIP_ENABLE,
};

enum gti_heatmap_setting : u32 {
	GTI_HEATMAP_DISABLE = 0,
	GTI_HEATMAP_ENABLE,
};

enum gti_irq_mode : u32 {
	GTI_IRQ_MODE_DISABLE = 0,
	GTI_IRQ_MODE_ENABLE,
	GTI_IRQ_MODE_NA = 0xFFFFFFFF,
};

/**
 * Motion filter mode.
 *   GTI_MF_MODE_UNFILTER: enable unfilter by continuous reporting.
 *   GTI_MF_MODE_DYNAMIC: dynamic control for motion filter.
 *   GTI_MF_MODE_FILTER: only report touch if coord report changed.
 *   GTI_MF_MODE_AUTO: for development case.
 */
enum gti_mf_mode : u32 {
	GTI_MF_MODE_UNFILTER = 0,
	GTI_MF_MODE_DEFAULT = 1,
	GTI_MF_MODE_DYNAMIC = GTI_MF_MODE_DEFAULT,
	GTI_MF_MODE_FILTER = 2,
	GTI_MF_MODE_AUTO_REPORT = 3,
};

/**
 * Motion filter finite state machine (FSM) state.
 *   GTI_MF_STATE_FILTERED: default coordinate filtering
 *   GTI_MF_STATE_UNFILTERED: coordinate unfiltering for single-touch.
 *   GTI_MF_STATE_FILTERED_LOCKED: filtered coordinates. Locked until
 *                                 touch is lifted or timeout.
 */
enum gti_mf_state : u32 {
	GTI_MF_STATE_FILTERED = 0,
	GTI_MF_STATE_UNFILTERED,
	GTI_MF_STATE_FILTERED_LOCKED,
};

enum gti_palm_setting : u32 {
	GTI_PALM_DISABLE = 0,
	GTI_PALM_ENABLE,
};

enum gti_panel_speed_mode_setting : u32 {
	GTI_PANEL_SPEED_MODE_NS = 0,
	GTI_PANEL_SPEED_MODE_HS,
};

enum gti_ping_mode : u32 {
	GTI_PING_NOP = 0,
	GTI_PING_ENABLE,
	GTI_PING_NA = 0xFFFFFFFF,
};

enum gti_pm_state : u32 {
	GTI_PM_SUSPEND = 0,
	GTI_PM_RESUME,
};

#define GTI_PM_WAKELOCK_TYPE_LOCK_MASK 0xFFFF
/**
 * @brief: wakelock type.
 */
enum gti_pm_wakelock_type : u32 {
	GTI_PM_WAKELOCK_TYPE_SCREEN_ON = (1 << 0),
	GTI_PM_WAKELOCK_TYPE_IRQ = (1 << 1),
	GTI_PM_WAKELOCK_TYPE_FW_UPDATE = (1 << 2),
	GTI_PM_WAKELOCK_TYPE_SYSFS = (1 << 3),
	GTI_PM_WAKELOCK_TYPE_FORCE_ACTIVE = (1 << 4),
	GTI_PM_WAKELOCK_TYPE_BUGREPORT = (1 << 5),
	GTI_PM_WAKELOCK_TYPE_OFFLOAD_REPORT = (1 << 6),
	GTI_PM_WAKELOCK_TYPE_SENSOR_DATA = (1 << 7),
	GTI_PM_WAKELOCK_TYPE_FW_SETTINGS = (1 << 8),
	GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST = (1 << 9),
};

enum gti_proc_type : u32 {
	GTI_PROC_DUMP,
	GTI_PROC_MS_BASE,
	GTI_PROC_MS_DIFF,
	GTI_PROC_MS_RAW,
	GTI_PROC_SS_BASE,
	GTI_PROC_SS_DIFF,
	GTI_PROC_SS_RAW,
	GTI_PROC_NUM,
};

enum gti_reset_mode : u32 {
	GTI_RESET_MODE_NOP = 0,
	GTI_RESET_MODE_SW = (1 << 0),
	GTI_RESET_MODE_HW = (1 << 1),
	GTI_RESET_MODE_AUTO = GTI_RESET_MODE_HW | GTI_RESET_MODE_SW,
	GTI_RESET_MODE_NA = 0xFFFFFFFF,
};

enum gti_scan_mode : u32 {
	GTI_SCAN_MODE_AUTO = 0,
	GTI_SCAN_MODE_NORMAL_ACTIVE,
	GTI_SCAN_MODE_NORMAL_IDLE,
	GTI_SCAN_MODE_LP_ACTIVE,
	GTI_SCAN_MODE_LP_IDLE,
	GTI_SCAN_MODE_NA = 0xFFFFFFFF,
};

enum gti_screen_protector_mode : u32 {
	GTI_SCREEN_PROTECTOR_MODE_DISABLE = 0,
	GTI_SCREEN_PROTECTOR_MODE_ENABLE,
	GTI_SCREEN_PROTECTOR_MODE_NA = 0xFFFFFFFF,
};

enum gti_selftest_result : u32 {
	GTI_SELFTEST_RESULT_DONE = 0,
	GTI_SELFTEST_RESULT_PASS = GTI_SELFTEST_RESULT_DONE,
	GTI_SELFTEST_RESULT_SHELL_CMDS_REDIRECT,
	GTI_SELFTEST_RESULT_FAIL = 0x80000000,
	GTI_SELFTEST_RESULT_NA = 0xFFFFFFFF,
};

enum gti_sensing_mode : u32 {
	GTI_SENSING_MODE_DISABLE = 0,
	GTI_SENSING_MODE_ENABLE,
	GTI_SENSING_MODE_NA = 0xFFFFFFFF,
};

/* Touch read method for automatically reading data from interrupt */
#define TOUCH_SENSOR_DATA_READ_METHOD_INT 0x10000
/* Touch read method for manually reading data from command */
#define TOUCH_SENSOR_DATA_READ_METHOD_COMMAND 0x20000

enum gti_sensor_data_type : u32 {
	GTI_SENSOR_DATA_TYPE_COORD = TOUCH_DATA_TYPE_COORD,
	GTI_SENSOR_DATA_TYPE_MS = TOUCH_SENSOR_DATA_READ_METHOD_INT |
			TOUCH_SCAN_TYPE_MUTUAL | TOUCH_DATA_TYPE_STRENGTH,
	GTI_SENSOR_DATA_TYPE_MS_DIFF = TOUCH_SENSOR_DATA_READ_METHOD_COMMAND |
			TOUCH_SCAN_TYPE_MUTUAL | TOUCH_DATA_TYPE_STRENGTH,
	GTI_SENSOR_DATA_TYPE_MS_RAW = TOUCH_SENSOR_DATA_READ_METHOD_COMMAND |
			TOUCH_SCAN_TYPE_MUTUAL | TOUCH_DATA_TYPE_RAW,
	GTI_SENSOR_DATA_TYPE_MS_BASELINE = TOUCH_SENSOR_DATA_READ_METHOD_COMMAND |
			TOUCH_SCAN_TYPE_MUTUAL | TOUCH_DATA_TYPE_BASELINE,
	GTI_SENSOR_DATA_TYPE_SS = TOUCH_SENSOR_DATA_READ_METHOD_INT |
			TOUCH_SCAN_TYPE_SELF | TOUCH_DATA_TYPE_STRENGTH,
	GTI_SENSOR_DATA_TYPE_SS_DIFF = TOUCH_SENSOR_DATA_READ_METHOD_COMMAND |
			TOUCH_SCAN_TYPE_SELF | TOUCH_DATA_TYPE_STRENGTH,
	GTI_SENSOR_DATA_TYPE_SS_RAW = TOUCH_SENSOR_DATA_READ_METHOD_COMMAND |
			TOUCH_SCAN_TYPE_SELF | TOUCH_DATA_TYPE_RAW,
	GTI_SENSOR_DATA_TYPE_SS_BASELINE = TOUCH_SENSOR_DATA_READ_METHOD_COMMAND |
			TOUCH_SCAN_TYPE_SELF | TOUCH_DATA_TYPE_BASELINE,
};

enum gti_fw_status : u32 {
	GTI_FW_STATUS_RESET = 0,
	GTI_FW_STATUS_PALM_ENTER,
	GTI_FW_STATUS_PALM_EXIT,
	GTI_FW_STATUS_GRIP_ENTER,
	GTI_FW_STATUS_GRIP_EXIT,
	GTI_FW_STATUS_WATER_ENTER,
	GTI_FW_STATUS_WATER_EXIT,
	GTI_FW_STATUS_NOISE_MODE,
	GTI_FW_STATUS_GESTURE_EVENT,
};

enum gti_gesture_params : u8 {
	GTI_STTW_MIN_X = 0,
	GTI_STTW_MAX_X,
	GTI_STTW_MIN_Y,
	GTI_STTW_MAX_Y,
	GTI_STTW_MIN_FRAME,
	GTI_STTW_MAX_FRAME,
	GTI_STTW_JITTER,
	GTI_STTW_MAX_TOUCH_SIZE,

	GTI_LPTW_MIN_X,
	GTI_LPTW_MAX_X,
	GTI_LPTW_MIN_Y,
	GTI_LPTW_MAX_Y,
	GTI_LPTW_MIN_FRAME,
	GTI_LPTW_JITTER,
	GTI_LPTW_MAX_TOUCH_SIZE,
	GTI_LPTW_MARGINAL_MIN_X,
	GTI_LPTW_MARGINAL_MAX_X,
	GTI_LPTW_MARGINAL_MIN_Y,
	GTI_LPTW_MARGINAL_MAX_Y,
	GTI_LPTW_MONITOR_CH_MIN_TX,
	GTI_LPTW_MONITOR_CH_MAX_TX,
	GTI_LPTW_MONITOR_CH_MIN_RX,
	GTI_LPTW_MONITOR_CH_MAX_RX,
	GTI_LPTW_NODE_COUNT_MIN,
	GTI_LPTW_MOTION_BOUNDARY,

	GTI_GESTURE_TYPE,

	GTI_GESTURE_PARAMS_MAX,
};

enum gti_gesture_type : u8 {
	GTI_GESTURE_DISABLE = 0,
	GTI_GESTURE_STTW,
	GTI_GESTURE_LPTW,
	GTI_GESTURE_STTW_AND_LPTW,
	GTI_GESTURE_TYPE_MAX,
};

enum gti_noise_mode_level : u8 {
	GTI_NOISE_MODE_EXIT = 0,
	GTI_NOISE_MODE_LEVEL1,
	GTI_NOISE_MODE_LEVEL2,
	GTI_NOISE_MODE_LEVEL3,
};

enum gti_ical_res : u32 {
	ICAL_RES_SUCCESS = 0,
	ICAL_RES_FAIL = 0x80000000,
	ICAL_RES_FAIL_INVALID_BUS_ACCESS = 0x80000001,
	ICAL_RES_NA = 0xFFFFFFFF,
};

enum gti_ical_state : u32 {
	ICAL_STATE_IDLE = 0,
	ICAL_STATE_INIT_CAL = 101,
	ICAL_STATE_RUN_CAL = 102,
	ICAL_STATE_END_CAL = 103,
	ICAL_STATE_INIT_TEST = 201,
	ICAL_STATE_RUN_TEST = 202,
	ICAL_STATE_END_TEST = 203,
	ICAL_STATE_INIT_RESET = 301,
	ICAL_STATE_RUN_RESET = 302,
	ICAL_STATE_END_RESET = 303,
	ICAL_STATE_NA = 0xFFFFFFFF,
};

/*-----------------------------------------------------------------------------
 * const char.
 */

const static char *gesture_params_list[GTI_GESTURE_PARAMS_MAX] = {
	"sttw_min_x",
	"sttw_max_x",
	"sttw_min_y",
	"sttw_max_y",
	"sttw_min_frame",
	"sttw_max_frame",
	"sttw_jitter",
	"sttw_max_touch_size",
	"lptw_min_x",
	"lptw_max_x",
	"lptw_min_y",
	"lptw_max_y",
	"lptw_min_frame",
	"lptw_jitter",
	"lptw_max_touch_size",
	"lptw_marginal_min_x",
	"lptw_marginal_max_x",
	"lptw_marginal_min_y",
	"lptw_marginal_max_y",
	"lptw_monitor_ch_min_tx",
	"lptw_monitor_ch_max_tx",
	"lptw_monitor_ch_min_rx",
	"lptw_monitor_ch_max_rx",
	"lptw_node_count_min",
	"lptw_motion_boundary",
	"gesture_type",
};

/*-----------------------------------------------------------------------------
 * Structures.
 */

struct gti_calibrate_cmd {
	enum gti_calibrate_result result;
	char buffer[PAGE_SIZE];
};

struct gti_context_changed {
	union {
		struct {
		u32 screen_state : 1;
		u32 display_refresh_rate : 1;
		u32 touch_report_rate : 1;
		u32 noise_state : 1;
		u32 water_mode : 1;
		u32 charger_state : 1;
		u32 hinge_angle : 1;
		u32 offload_timestamp : 1;
		};
		u32 value;
	};
};

struct gti_context_driver_cmd {
	struct gti_context_changed context_changed;

	u8 screen_state;
	u8 display_refresh_rate;
	u8 touch_report_rate;
	u8 noise_state;
	u8 water_mode;
	u8 charger_state;
	s16 hinge_angle;

	ktime_t offload_timestamp;
};

struct gti_context_stylus_cmd {
	struct {
		u32 coords : 1;
		u32 coords_timestamp : 1;
		u32 pen_paired : 1;
		u32 pen_active : 1;
	} contents;
	struct TouchOffloadCoord pen_offload_coord;
	ktime_t pen_offload_coord_timestamp;
	u8 pen_paired;
	u8 pen_active;
};

struct gti_continuous_report_cmd {
	enum gti_continuous_report_setting setting;
};

struct gti_debug_coord {
	ktime_t time;
	u64 irq_index;
	struct TouchOffloadCoord coord;
};

struct gti_debug_healthcheck {
	ktime_t irq_time;
	u64 irq_index;
	u64 input_index;
	unsigned long slot_bit_active;
};

struct gti_debug_input {
	int slot;
	struct gti_debug_coord pressed;
	struct gti_debug_coord released;
};

struct gti_coord_filter_cmd {
	enum gti_coord_filter_setting setting;
};

struct gti_display_state_cmd {
	enum gti_display_state_setting setting;
};

struct gti_display_vrefresh_cmd {
	u32 setting;
};

struct gti_fw_version_cmd {
	char buffer[0x200];
};

struct gti_gesture_config_cmd {
	u8 updating_params[GTI_GESTURE_PARAMS_MAX];
	u16 params[GTI_GESTURE_PARAMS_MAX];
};

struct gti_grip_cmd {
	enum gti_grip_setting setting;
};

struct gti_heatmap_cmd {
	enum gti_heatmap_setting setting;
};

struct gti_irq_cmd {
	enum gti_irq_mode setting;
};

struct gti_palm_cmd {
	enum gti_palm_setting setting;
};

struct gti_panel_speed_mode_cmd {
	enum gti_panel_speed_mode_setting setting;
};

struct gti_ping_cmd {
	enum gti_ping_mode setting;
};

struct gti_report_rate_cmd {
	u32 setting;
};

struct gti_reset_cmd {
	enum gti_reset_mode setting;
};

struct gti_scan_cmd {
	enum gti_scan_mode setting;
};

struct gti_screen_protector_mode_cmd {
	enum gti_screen_protector_mode setting;
};

struct gti_selftest_cmd {
	enum gti_selftest_result result;
	char buffer[PAGE_SIZE];
	bool is_ical;
};

struct gti_sensing_cmd {
	enum gti_sensing_mode setting;
};

struct gti_sensor_data_cmd {
	enum gti_sensor_data_type type;
	u8 *buffer;
	u32 size;
	/* Set by vendor driver and the default value is false */
	bool is_unsigned;
};

/**
 * struct gti_union_cmd_data - GTI commands to vendor driver.
 * @calibrate_cmd: command to calibrate the touchscreen
 * @context_driver_cmd: command to update touch offload driver context.
 * @context_stylus_cmd: command to update touch offload stylus context.
 * @continuous_report_cmd: command to set continuous reporting.
 * @coord_filter_cmd: command to set/get coordinate filter enabled.
 * @display_state_cmd: command to notify display state.
 * @display_vrefresh_cmd: command to notify display vertical refresh rate.
 * @fw_version_cmd: command to get fw version.
 * @gesture_config_cmd: command to set gesture parameters and gesture types.
 * @grip_cmd: command to set/get grip mode.
 * @heatmap_cmd: command to set heatmap enabled.
 * @irq_cmd: command to set/get irq mode.
 * @palm_cmd: command to set/get palm mode.
 * @panel_speed_mode_cmd: command to set panel speed mode.
 * @ping_cmd: command to ping T-IC.
 * @report_rate_cmd: command to change touch report rate.
 * @reset_cmd: command to reset T-IC.
 * @scan_cmd: command to set/get scan mode.
 * @screen_protector_mode_cmd: command to set/get screen protector mode.
 * @selftest_cmd: command to do self-test.
 * @sensing_cmd: command to set/set sensing mode.
 * @sensor_data_cmd: command to get sensor data.
 * @manual_sensor_data_cmd: command to get sensor data manually.
 */
struct gti_union_cmd_data {
	struct gti_calibrate_cmd calibrate_cmd;
	struct gti_context_driver_cmd context_driver_cmd;
	struct gti_context_stylus_cmd context_stylus_cmd;
	struct gti_continuous_report_cmd continuous_report_cmd;
	struct gti_coord_filter_cmd coord_filter_cmd;
	struct gti_display_state_cmd display_state_cmd;
	struct gti_display_vrefresh_cmd display_vrefresh_cmd;
	struct gti_fw_version_cmd fw_version_cmd;
	struct gti_gesture_config_cmd gesture_config_cmd;
	struct gti_grip_cmd grip_cmd;
	struct gti_heatmap_cmd heatmap_cmd;
	struct gti_irq_cmd irq_cmd;
	struct gti_palm_cmd palm_cmd;
	struct gti_panel_speed_mode_cmd panel_speed_mode_cmd;
	struct gti_ping_cmd ping_cmd;
	struct gti_report_rate_cmd report_rate_cmd;
	struct gti_reset_cmd reset_cmd;
	struct gti_scan_cmd scan_cmd;
	struct gti_screen_protector_mode_cmd screen_protector_mode_cmd;
	struct gti_selftest_cmd selftest_cmd;
	struct gti_sensing_cmd sensing_cmd;
	struct gti_sensor_data_cmd sensor_data_cmd;
	struct gti_sensor_data_cmd manual_sensor_data_cmd;
};

/**
 * struct gti_gesture_event_data - GTI gesture event data for notifying changed.
 * @gesture_type: triggered gesture type.
 * @x: x coordinate for the finger.
 * @y: x coordinate for the finger.
 * @major: major of the finger in pixels.
 * @minor: minor of the finger in pixels.
 * @angle: finger angle from -4095 ~ 4096.
 */
struct gti_gesture_event_data {
	enum gti_gesture_type type;
	u16 x;
	u16 y;
	u16 major;
	u16 minor;
	s16 angle;
};

/**
 * struct gti_fw_status_data - GTI fw status data for notifying changed.
 * @noise_level: the noise level for noise mode.
 */
struct gti_fw_status_data {
	enum gti_noise_mode_level noise_level;
	u8 water_mode;
	struct gti_gesture_event_data gesture_event;
};

/**
 * struct gti_optional_configuration - optional configuration by vendor driver.
 * @calibrate: vendor driver operation to exec calibration
 * @get_context_driver: vendor driver operation to update touch offload driver context.
 * @get_context_stylus: vendor driver operation to update touch offload stylus context.
 * @get_coord_filter_enabled: vendor driver operation to get the coordinate filter enabled.
 * @get_fw_version: vendor driver operation to get fw version info.
 * @get_grip_mode: vendor driver operation to get the grip mode setting.
 * @get_irq_mode: vendor driver operation to get irq mode setting.
 * @get_mutual_sensor_data: vendor driver operation to get the mutual sensor data.
 * @get_palm_mode: vendor driver operation to get the palm mode setting.
 * @get_scan_mode: vendor driver operation to get scan mode.
 * @get_screen_protector_mode: vendor driver operation to get screen protector mode.
 * @get_self_sensor_data: vendor driver operation to get the self sensor data.
 * @get_sensing_mode: vendor driver operation to get sensing mode.
 * @notify_display_state: vendor driver operation to notify the display state.
 * @notify_display_vrefresh: vendor driver operation to notify the display vertical refresh rate.
 * @ping: vendor driver operation to ping T-IC.
 * @reset: vendor driver operation to exec reset.
 * @selftest: vendor driver operation to exec self-test.
 * @set_continuous_report: vendor driver operation to apply the continuous reporting setting.
 * @set_coord_filter_enabled: vendor driver operation to apply the coordinate filter enabled.
 * @set_gesture_config: vendor driver operation to apply the gesture settings.
 * @set_grip_mode: vendor driver operation to apply the grip setting.
 * @set_heatmap_enabled: vendor driver operation to apply the heatmap setting.
 * @set_irq_mode: vendor driver operation to apply the irq setting.
 * @set_palm_mode: vendor driver operation to apply the palm setting.
 * @set_panel_speed_mode: vendor driver operation to apply the panel speed mode setting.
 * @set_report_rate: driver operation to set touch report rate.
 * @set_scan_mode: vendor driver operation to set scan mode.
 * @set_screen_protector_mode: vendor driver operation to set screen protector mode.
 * @set_sensing_mode: vendor driver operation to set sensing mode.
 * @post_irq_thread_fn: post irq thread function that register by vendor driver.
 */
struct gti_optional_configuration {
	/*
	 * Vendor command with GTI default NOP handler.
	 * No need to check before use.
	 */
	int (*calibrate)(void *private_data, struct gti_calibrate_cmd *cmd);
	int (*get_context_driver)(void *private_data, struct gti_context_driver_cmd *cmd);
	int (*get_context_stylus)(void *private_data, struct gti_context_stylus_cmd *cmd);
	int (*get_coord_filter_enabled)(void *private_data, struct gti_coord_filter_cmd *cmd);
	int (*get_fw_version)(void *private_data, struct gti_fw_version_cmd *cmd);
	int (*get_grip_mode)(void *private_data, struct gti_grip_cmd *cmd);
	int (*get_irq_mode)(void *private_data, struct gti_irq_cmd *cmd);
	int (*get_mutual_sensor_data)(void *private_data, struct gti_sensor_data_cmd *cmd);
	int (*get_palm_mode)(void *private_data, struct gti_palm_cmd *cmd);
	int (*get_scan_mode)(void *private_data, struct gti_scan_cmd *cmd);
	int (*get_screen_protector_mode)(void *private_data,
			struct gti_screen_protector_mode_cmd *cmd);
	int (*get_self_sensor_data)(void *private_data, struct gti_sensor_data_cmd *cmd);
	int (*get_sensing_mode)(void *private_data, struct gti_sensing_cmd *cmd);
	int (*notify_display_state)(void *private_data, struct gti_display_state_cmd *cmd);
	int (*notify_display_vrefresh)(void *private_data, struct gti_display_vrefresh_cmd *cmd);
	int (*ping)(void *private_data, struct gti_ping_cmd *cmd);
	int (*reset)(void *private_data, struct gti_reset_cmd *cmd);
	int (*selftest)(void *private_data, struct gti_selftest_cmd *cmd);
	int (*set_continuous_report)(void *private_data, struct gti_continuous_report_cmd *cmd);
	int (*set_coord_filter_enabled)(void *private_data, struct gti_coord_filter_cmd *cmd);
	int (*set_gesture_config)(void *private_data, struct gti_gesture_config_cmd *cmd);
	int (*set_grip_mode)(void *private_data, struct gti_grip_cmd *cmd);
	int (*set_heatmap_enabled)(void *private_data, struct gti_heatmap_cmd *cmd);
	int (*set_irq_mode)(void *private_data, struct gti_irq_cmd *cmd);
	int (*set_palm_mode)(void *private_data, struct gti_palm_cmd *cmd);
	int (*set_panel_speed_mode)(void *private_data, struct gti_panel_speed_mode_cmd *cmd);
	int (*set_report_rate)(void *private_data, struct gti_report_rate_cmd *cmd);
	int (*set_scan_mode)(void *private_data, struct gti_scan_cmd *cmd);
	int (*set_screen_protector_mode)(void *private_data,
			struct gti_screen_protector_mode_cmd *cmd);
	int (*set_sensing_mode)(void *private_data, struct gti_sensing_cmd *cmd);

	/*
	 * Vendor command without GTI default NOP handler.
	 * Need to check before use.
	 */
	irq_handler_t post_irq_thread_fn;
};

/**
 * struct gti_pm - power manager for GTI.
 * @state_update_work: a work to update pm state.
 * @locks: the lock state.
 * @lock_mutex: protect the lock state.
 * @state: GTI pm state.
 * @new_state: New GTI pm state to be updated to.
 * @enabled: Boolean value to represent if GTI PM is active.
 * @update_state: Boolean value if state needs to be updated.
 * @resume: callback for notifying resume.
 * @suspend: callback for notifying suspend.
 */
struct gti_pm {
	struct work_struct state_update_work;

	u32 locks;
	struct mutex lock_mutex;
	enum gti_pm_state state;
	enum gti_pm_state new_state;
	bool enabled;
	bool update_state;

	int (*resume)(struct device *dev);
	int (*suspend)(struct device *dev);
};

/**
 * struct goog_touch_interface - Google touch interface data for Pixel.
 * @vendor_private_data: the private data pointer that used by touch vendor driver.
 * @vendor_dev: pointer to struct device that used by touch vendor driver.
 * @vendor_input_dev: pointer to struct inpu_dev that used by touch vendor driver.
 * @dev: pointer to struct device that used by google touch interface driver.
 * @options: optional configuration that could apply by vendor driver.
 * @input_lock: protect the input report between non-offload and offload.
 * @input_process_lock: mutex for goog_input_process() function.
 * @input_heatmap_lock: mutex for heatmap reading between vendor driver, GTI or sysfs/procfs.
 * @offload: struct that used by touch offload.
 * @offload_frame: reserved frame that used by touch offload.
 * @v4l2: struct that used by v4l2.
 * @panel_bridge: struct that used to register panel bridge notification.
 * @connector: struct that used to get panel status.
 * @cmd: struct that used by vendor default handler.
 * @proc_dir: struct that used for procfs.
 * @proc_heatmap: struct that used for heatmap procfs.
 * @set_op_hz_work: a work to set op_hz.
 * @event_wq: a work queue to run suspend/resume work.
 * @input_dev_mono_ktime: input timestamp used by input dev and input subsystem.
 * @input_timestamp: input timestamp from touch vendor driver.
 * @mf_downtime: timestamp for motion filter control.
 * @vrr_enabled: variable touch report rate is enabled or not.
 * @report_rate_table_size: report rate table size from device tree.
 * @touch_report_rate_table: touch report rate table parsed from device tree.
 * @display_refresh_rate_table: display refresh rate table parsed from device tree.
 * @report_rate_setting: current touch report rate.
 * @report_rate_setting_next: next touch report rate going be set.
 * @set_report_rate_work: delayed work for setting report rate.
 * @increase_report_rate_delay: delayed work will be start after a delay in seconds.
 * @decrease_report_rate_delay: delayed work will be start after a delay in seconds.
 * @abs_x_min: minimum x of input resolution.
 * @abs_x_max: maximum x of input resolution.
 * @abs_y_min: minimum y of input resolution.
 * @abs_y_max: maximum y of input resolution.
 * @resolution_scale_factor: scale factor of input to display resolution.
 * @display_vrefresh: display vrefresh in Hz.
 * @display_state: current display state.
 * @mf_mode: current motion filter mode.
 * @mf_state: current motion filter state.
 * @screen_protector_mode_setting: the setting of screen protector mode.
 * @tbn_register_mask: the tbn_mask that used to request/release touch bus.
 * @pm: struct that used by gti pm.
 * @pm_qos_req: struct that used by pm qos.
 * @fw_status: firmware status such as water_mode, noise_level, etc.
 * @context_changed: flags that indicate driver status changing.
 * @panel_is_lp_mode: display is in low power mode.
 * @offload_enabled: touch offload is enabled or not.
 * @v4l2_enabled: v4l2 is enabled or not.
 * @tbn_enabled: tbn is enabled or not.
 * @coord_filter_enabled: coordinate filter is enabled or not.
 * @input_timestamp_changed: input timestamp changed from touch vendor driver.
 * @ignore_grip_update: Ignore fw_grip status updates made on offload state change.
 * @default_grip_enabled: the grip default setting.
 * @ignore_palm_update: Ignore fw_palm status updates made on offload state change.
 * @default_palm_enabled: the palm default setting.
 * @ignore_coord_filter_update: Ignore fw_coordinate_filter status updates.
 * @fw_coord_filter_enabled: the current setting of coordinate filter.
 * @default_coord_filter_enabled: the default setting of coordinate filter.
 * @panel_notifier_enabled: enable flag for receiving panel notifications.
 * @reset_after_selftest: reset FW after running self-test.
 * @lptw_triggered: LPTW is triggered or not.
 * @lptw_suppress_coords_enabled: enable flag for suppressing the coords after lptw.
 * @lptw_track_finger: flag for tracking the suppressed fingers.
 * @lptw_track_min_x: minimum x of tracking area.
 * @lptw_track_max_x: maximum x of tracking area.
 * @lptw_track_min_y: minimum y of tracking area.
 * @lptw_track_max_y: maximum y of tracking area.
 * @lptw_cancel_delayed_work: delayed work for canceling finger.
 * @lptw_cancel_time: record the time for lptw cancel timeout.
 * @lptw_down: true if the finger is still on the screen.
 * @lptw_data: x, y, major, minor, angle for the tracking finger.
 * @ignore_force_active: Ignore the force_active sysfs request.
 * @offload_id: id that used by touch offload.
 * @heatmap_buf: heatmap buffer that used by v4l2.
 * @heatmap_buf_size: heatmap buffer size that used by v4l2.
 * @slot: slot id that current used by input report.
 * @slot_bit_in_use: bitmap of slot in use for this input process cycle.
 * @slot_bit_changed: bitmap of slot state changed for this input process cycle.
 * @slot_bit_active: bitmap of active slot during GTI lifecycle.
 * @slot_bit_lptw_track: bitmap of lptw suppressed fingers.
 * @panel_op_hz: the operating rate of display panel.
 * @dev_id: dev_t used for google interface driver.
 * @panel_id: id of the display panel.
 * @charger_state: indicates a USB charger is connected.
 * @charger_notifier: notifier for power_supply updates.
 * @panel_notifier: notifier for display panel updates.
 * @ical_state: state of interactive calibration finite state machine.
 * @ical_timestamp_ns: time of last interactive calibration state transition.
 * @ical_result: interactive calibration FSM result.
 * @ical_func_result: result returned from the requested interactive function.
 * @frame_index: the count that handle by goog_input_process().
 * @irq_index: irq count that handle by GTI.
 * @input_index: the count of slot bit changed during goog_input_process().
 * @vendor_irq_handler: irq handler that register by vendor driver.
 * @vendor_irq_thread_fn: irq thread function that register by vendor driver.
 * @vendor_irq_cookie: irq cookie that register by vendor driver.
 * @vendor_default_handler: touch vendor driver default operation.
 * @released_index: finger up count.
 * @debug_warning_limit: limit number of warning logs.
 * @debug_input: struct that used to debug input.
 * @debug_fifo_input: kfifo struct to track input report.
 * @debug_healthcheck: struct that used for the health check.
 * @debug_fifo_healthcheck: kfifo struct to track touch interrupt information.
 */

struct goog_touch_interface {
	void *vendor_private_data;
	struct device *vendor_dev;
	struct input_dev *vendor_input_dev;
	struct device *dev;
	struct gti_optional_configuration options;
	struct mutex input_lock;
	struct mutex input_process_lock;
	struct mutex input_heatmap_lock;
	struct touch_offload_context offload;
	struct touch_offload_frame *offload_frame;
	struct v4l2_heatmap v4l2;
	struct drm_bridge panel_bridge;
	struct drm_connector *connector;
	struct gti_union_cmd_data cmd;
	struct proc_dir_entry *proc_dir;
	struct proc_dir_entry *proc_show[GTI_PROC_NUM];
	struct work_struct set_op_hz_work;
	struct workqueue_struct *event_wq;
	ktime_t input_dev_mono_ktime;
	ktime_t input_timestamp;
	ktime_t mf_downtime;

	bool vrr_enabled;
	int report_rate_table_size;
	u32 *display_refresh_rate_table;
	u32 *touch_report_rate_table;
	u32 report_rate_setting;
	u32 report_rate_setting_next;
	struct delayed_work set_report_rate_work;
	u32 increase_report_rate_delay;
	u32 decrease_report_rate_delay;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int resolution_scale_factor;

	int display_vrefresh;
	enum gti_display_state_setting display_state;
	enum gti_mf_mode mf_mode;
	enum gti_mf_state mf_state;
	enum gti_screen_protector_mode screen_protector_mode_setting;
	u32 tbn_register_mask;
	struct gti_pm pm;
	struct pm_qos_request pm_qos_req;

	struct gti_fw_status_data fw_status;
	struct gti_context_changed context_changed;

	bool panel_is_lp_mode;
	bool offload_enabled;
	bool v4l2_enabled;
	bool tbn_enabled;
	bool coord_filter_enabled;
	bool input_timestamp_changed;
	bool ignore_grip_update;
	bool default_grip_enabled;
	bool ignore_palm_update;
	bool default_palm_enabled;
	bool ignore_coord_filter_update;
	bool fw_coord_filter_enabled;
	bool default_coord_filter_enabled;
	bool panel_notifier_enabled;
	bool reset_after_selftest;
	bool lptw_triggered;
	bool lptw_suppress_coords_enabled;
	bool lptw_track_finger;
	u32 lptw_track_min_x;
	u32 lptw_track_max_x;
	u32 lptw_track_min_y;
	u32 lptw_track_max_y;
	struct delayed_work lptw_cancel_delayed_work;
	ktime_t lptw_cancel_time;

	bool lptw_down;
	union {
		int lptw_data[5];
		struct {
			int lptw_x;
			int lptw_y;
			int lptw_major;
			int lptw_minor;
			int lptw_angle;
		};
	};

	bool ignore_force_active;
	bool gesture_config_enabled;
	bool manual_heatmap_from_irq;
	union {
		u8 offload_id_byte[4];
		u32 offload_id;
	};
	u8 *heatmap_buf;
	u32 heatmap_buf_size;
	int slot;
	unsigned long slot_bit_in_use;
	unsigned long slot_bit_changed;
	unsigned long slot_bit_active;
	unsigned long slot_bit_lptw_track;
	unsigned int panel_op_hz;
	dev_t dev_id;
	int panel_id;
	char fw_name[64];
	char config_name[64];
	char test_limits_name[64];
	char usb_psy_name[64];

	u8 charger_state;
	struct notifier_block charger_notifier;
	struct notifier_block panel_notifier;

	u32 ical_state;
	u64 ical_timestamp_ns;
	s32 ical_result;
	s32 ical_func_result;

	u64 frame_index;
	u64 irq_index;
	u64 input_index;
	irq_handler_t vendor_irq_handler;
	irq_handler_t vendor_irq_thread_fn;
	void *vendor_irq_cookie;

	int (*vendor_default_handler)(void *private_data,
		enum gti_cmd_type cmd_type, struct gti_union_cmd_data *cmd);

	/* Debug used. */
	u64 released_index;
	int debug_warning_limit;
	struct gti_debug_input debug_input[MAX_SLOTS];
	struct gti_debug_input debug_input_history[GTI_DEBUG_INPUT_KFIFO_LEN];
	DECLARE_KFIFO(debug_fifo_input, struct gti_debug_input, GTI_DEBUG_INPUT_KFIFO_LEN);
	struct gti_debug_healthcheck debug_healthcheck;
	struct gti_debug_healthcheck debug_healthcheck_history[GTI_DEBUG_HEALTHCHECK_KFIFO_LEN];
	DECLARE_KFIFO(debug_fifo_healthcheck, struct gti_debug_healthcheck,
		GTI_DEBUG_HEALTHCHECK_KFIFO_LEN);
};

/*-----------------------------------------------------------------------------
 * Forward declarations.
 */
inline bool goog_check_spi_dma_enabled(struct spi_device *spi_dev);
inline ktime_t *goog_input_get_timestamp(struct goog_touch_interface *gti);
inline void goog_input_lock(struct goog_touch_interface *gti);
inline void goog_input_unlock(struct goog_touch_interface *gti);
inline void goog_input_set_timestamp(
		struct goog_touch_interface *gti,
		struct input_dev *dev, ktime_t timestamp);
inline void goog_input_mt_slot(
		struct goog_touch_interface *gti,
		struct input_dev *dev, int slot);
inline void goog_input_mt_report_slot_state(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int tool_type, bool active);
inline void goog_input_report_abs(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int code, int value);
inline void goog_input_report_key(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int code, int value);
inline void goog_input_sync(struct goog_touch_interface *gti, struct input_dev *dev);
inline int goog_devm_request_threaded_irq(struct goog_touch_interface *gti,
		struct device *dev, unsigned int irq,
		irq_handler_t handler, irq_handler_t thread_fn,
		unsigned long irqflags, const char *devname,
		void *dev_id);
void goog_devm_free_irq(struct goog_touch_interface *gti,
		struct device *dev, unsigned int irq);
inline int goog_request_threaded_irq(struct goog_touch_interface *gti,
		unsigned int irq, irq_handler_t handler, irq_handler_t thread_fn,
		unsigned long irqflags, const char *devname, void *dev_id);

int goog_process_vendor_cmd(struct goog_touch_interface *gti, enum gti_cmd_type cmd_type);
int goog_input_process(struct goog_touch_interface *gti, bool reset_data);
struct goog_touch_interface *goog_touch_interface_probe(
		void *private_data,
		struct device *dev,
		struct input_dev *input_dev,
		int (*default_handler)(void *private_data,
			enum gti_cmd_type cmd_type, struct gti_union_cmd_data *cmd),
		struct gti_optional_configuration *options);
int goog_touch_interface_remove(struct goog_touch_interface *gti);

int goog_pm_wake_lock_nosync(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type, bool skip_pm_resume);
int goog_pm_wake_lock(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type, bool skip_pm_resume);
int goog_pm_wake_unlock_nosync(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type);
int goog_pm_wake_unlock(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type);
bool goog_pm_wake_check_locked(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type);
u32 goog_pm_wake_get_locks(struct goog_touch_interface *gti);
int goog_pm_register_notification(struct goog_touch_interface *gti,
		const struct dev_pm_ops* ops);
int goog_pm_unregister_notification(struct goog_touch_interface *gti);

void goog_notify_fw_status_changed(struct goog_touch_interface *gti,
		enum gti_fw_status status, struct gti_fw_status_data* data);
void gti_debug_healthcheck_dump(struct goog_touch_interface *gti);
void gti_debug_input_dump(struct goog_touch_interface *gti);

int goog_get_lptw_triggered(struct goog_touch_interface *gti);
void goog_lptw_notifier_register(struct notifier_block *nb, bool reg);

int goog_get_max_touch_report_rate(struct goog_touch_interface *gti);
int goog_get_panel_id(struct device_node *node);
int goog_get_firmware_name(struct device_node *node, int id, char *name, size_t size);
int goog_get_config_name(struct device_node *node, int id, char *name, size_t size);
int goog_get_test_limits_name(struct device_node *node, int id, char *name, size_t size);

#endif // _GOOG_TOUCH_INTERFACE_

