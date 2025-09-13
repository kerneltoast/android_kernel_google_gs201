/* SPDX-License-Identifier: GPL-2.0
 *
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file syna_tcm2.h
 *
 * The header file is used for the Synaptics TouchComm reference driver.
 * Platform-specific functions and included headers are implemented in
 * syna_touchcom_platform.h and syna_touchcom_runtime.h.
 */

#ifndef _SYNAPTICS_TCM2_DRIVER_H_
#define _SYNAPTICS_TCM2_DRIVER_H_

#include "syna_tcm2_platform.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_touch.h"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
#include <touch_bus_negotiator.h>
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
#include <touch_offload.h>
#include <linux/hrtimer.h>
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
#include <heatmap.h>
#endif

#include <linux/pm_qos.h>
#include <trace/hooks/systrace.h>

#define PLATFORM_DRIVER_NAME "synaptics_tcm"

#define TOUCH_INPUT_NAME "synaptics_tcm_touch"
#define TOUCH_INPUT_PHYS_PATH "synaptics_tcm/touch_input"

#define CHAR_DEVICE_NAME "tcm"
#define CHAR_DEVICE_MODE (0x0600)

#define SYNAPTICS_TCM_DRIVER_ID (1 << 0)
#define SYNAPTICS_TCM_DRIVER_VERSION 1
#define SYNAPTICS_TCM_DRIVER_SUBVER "2.8"

/**
 * @section: Driver Configurations
 *
 * The macros in the driver files below are used for doing compile time
 * configuration of the driver.
 */

/**
 * @brief: HAS_SYSFS_INTERFACE
 *         Open to enable the sysfs interface
 *
 * @brief: HAS_REFLASH_FEATURE
 *         Open to enable firmware reflash features
 *
 * @brief: HAS_ROMBOOT_REFLASH_FEATURE
 *         Open to enable ROMBOOT reflash features
 *
 * @brief: HAS_TESTING_FEATURE
 *         Open to enable testing features
 */
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_SYSFS)
#define HAS_SYSFS_INTERFACE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_REFLASH)
#define HAS_REFLASH_FEATURE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_ROMBOOT)
#define HAS_ROMBOOT_REFLASH_FEATURE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_TESTING)
#define HAS_TESTING_FEATURE
#endif

/**
 * @brief: TYPE_B_PROTOCOL
 *         Open to enable the multi-touch (MT) protocol
 */
#define TYPE_B_PROTOCOL

/**
 * @brief: RESET_ON_RESUME
 *         Open if willing to issue a reset to the touch controller
 *         from suspend.
 *         Set "disable" in default.
 */
#define RESET_ON_RESUME

/**
 * @brief ENABLE_WAKEUP_GESTURE
 *        Open if having wake-up gesture support.
 */
/* #define ENABLE_WAKEUP_GESTURE */

/**
 * @brief REPORT_SWAP_XY
 *        Open if trying to swap x and y position coordinate reported.
 * @brief REPORT_FLIP_X
 *        Open if trying to flip x position coordinate reported.
 * @brief REPORT_FLIP_Y
 *        Open if trying to flip x position coordinate reported.
 */
/* #define REPORT_SWAP_XY */
/* #define REPORT_FLIP_X */
/* #define REPORT_FLIP_Y */

/**
 * @brief REPORT_TOUCH_WIDTH
 *        Open if willing to add the width data to the input event.
 */
#define REPORT_TOUCH_WIDTH

/**
 * @brief USE_CUSTOM_TOUCH_REPORT_CONFIG
 *        Open if willing to set up the format of touch report.
 *        The custom_touch_format[] array in syna_tcm2.c can be used
 *        to describe the customized report format.
 */
/* #define USE_CUSTOM_TOUCH_REPORT_CONFIG */

/**
 * @brief STARTUP_REFLASH
 *        Open if willing to do fw checking and update at startup.
 *        The firmware image will be obtained by request_firmware() API,
 *        so please ensure the image is built-in or included properly.
 *
 *        This property is available only when SYNA_TCM2_REFLASH
 *        feature is enabled.
 */
#if defined(HAS_REFLASH_FEATURE) || defined(HAS_ROMBOOT_REFLASH_FEATURE)
#define STARTUP_REFLASH
#endif
/**
 * @brief  MULTICHIP_DUT_REFLASH
 *         Open if willing to do fw update and the DUT belongs to multi-chip
 *         product. This property dependent on STARTUP_REFLASH property.
 *
 *         Set "disable" in default.
 */
#if defined(HAS_ROMBOOT_REFLASH_FEATURE) && defined(STARTUP_REFLASH)
/* #define MULTICHIP_DUT_REFLASH */
#endif

/**
 * @section: STARTUP_REFLASH_DELAY_TIME_MS
 *           The delayed time to start fw update during the startup time.
 *           This configuration depends on STARTUP_REFLASH.
 */
#ifdef STARTUP_REFLASH
#define STARTUP_REFLASH_DELAY_TIME_MS (200)

#define FW_IMAGE_NAME "synaptics.img"
#endif

/**
 * @brief  ENABLE_DISP_NOTIFIER
 *         Open if having display notification event and willing to listen
 *         the event from display driver.
 *
 *         Set "disable" in default due to no generic notifier for DRM
 */
#if defined(CONFIG_FB) || defined(CONFIG_DRM_PANEL)
/* #define ENABLE_DISP_NOTIFIER */
#endif
/**
 * @brief RESUME_EARLY_UNBLANK
 *        Open if willing to resume in early un-blanking state.
 *
 *        This property is available only when ENABLE_DISP_NOTIFIER
 *        feature is enabled.
 */
#ifdef ENABLE_DISP_NOTIFIER
/* #define RESUME_EARLY_UNBLANK */
#endif
/**
 * @brief  USE_DRM_PANEL_NOTIFIER
 *         Open if willing to listen the notification event from
 *         DRM_PANEL. Please be noted that 'struct drm_panel_notifier'
 *         must be implemented in the target BSP.
 *
 *        This property is available only when ENABLE_DISP_NOTIFIER
 *        feature is enabled.
 *
 *         Set "disable" in default due to no generic notifier for DRM
 */
#if defined(ENABLE_DISP_NOTIFIER) && defined(CONFIG_DRM_PANEL)
#define USE_DRM_PANEL_NOTIFIER
#endif

/**
 * @brief ENABLE_EXTERNAL_FRAME_PROCESS
 *        Open if having external frame process to the userspace application.
 *
 *        Set "enable" in default
 *
 * @brief REPORT_TYPES
 *        Total types of report being used for external frame process.
 *
 * @brief EFP_ENABLE / EFP_DISABLE
 *        Specific value to label whether the report is required to be
 *        process or not.
 *
 * @brief REPORT_CONCURRENTLY
 *        Open if willing to concurrently handle reports for both kernel
 *        and userspace application.
 *
 *        Set "disable" in default
 */
#define ENABLE_EXTERNAL_FRAME_PROCESS
#define REPORT_TYPES (256)
#define EFP_ENABLE	(1)
#define EFP_DISABLE (0)
/* #define REPORT_CONCURRENTLY */

/**
 * @brief TCM_CONNECT_IN_PROBE
 *        Open if willing to detect and connect to TouchComm device at
 *        probe function; otherwise, please invoke connect() manually.
 *
 *        Set "enable" in default
 */
#define TCM_CONNECT_IN_PROBE

/**
 * @brief FORCE_CONNECTION
 *        Open if willing to connect to TouchComm device w/o error outs.
 *
 *        Set "disable" in default
 */
/* #define FORCE_CONNECTION */

/**
 * @brief ENABLE_CUSTOM_TOUCH_ENTITY
 *        Open if having custom requirements to parse the custom code
 *        entity in the touch report.
 *
 *        Set "disable" in default
 */
#define ENABLE_CUSTOM_TOUCH_ENTITY

/**
 * @brief ENABLE_HELPER
 *        Open if willing to do additional handling upon helper workqueue
 *
 *        Set "disable" in default
 */
#define ENABLE_HELPER

/**
 * @brief: Power States
 *
 * Enumerate the power states of device
 */
enum power_state {
	PWR_OFF = 0,
	PWR_ON,
	LOW_PWR,
};

/**
  * @brief: Bits masking for bus reference.
  */
enum {
	SYNA_BUS_REF_SCREEN_ON		= 0x0001,
	SYNA_BUS_REF_IRQ		= 0x0002,
	SYNA_BUS_REF_FW_UPDATE		= 0x0004,
	SYNA_BUS_REF_SYSFS		= 0x0008,
	SYNA_BUS_REF_FORCE_ACTIVE	= 0x0010,
	SYNA_BUS_REF_BUGREPORT		= 0x0020,
};

/* Motion filter finite state machine (FSM) states
 * MF_FILTERED        - default coordinate filtering
 * MF_UNFILTERED      - unfiltered single-touch coordinates
 * MF_FILTERED_LOCKED - filtered coordinates. Locked until touch is lifted.
 */
typedef enum {
	MF_FILTERED		= 0,
	MF_UNFILTERED		= 1,
	MF_FILTERED_LOCKED	= 2
} motion_filter_state_t;

/* Motion filter mode.
 *  MF_OFF    : 0 = Always unfilter.
 *  MF_DYNAMIC: 1 = Dynamic change motion filter.
 *  MF_ON     : 2 = Always filter by touch FW.
 */
enum MF_MODE {
    MF_OFF,
    MF_DYNAMIC,
    MF_ON,
};

#if defined(ENABLE_HELPER)
/**
 * @brief: Tasks for helper
 *
 * Tasks being supported in the helper thread and the structure
 */
enum helper_task {
	HELP_NONE = 0,
	HELP_RESET_DETECTED,
};

struct syna_tcm_helper {
	syna_pal_atomic_t task;
	struct work_struct work;
};
#endif

/**
 * @brief: Structure for $C2 report
 *
 * Enumerate the power states of device
 */
struct custom_fw_status {
	union {
		struct {
			unsigned char b0_moisture:1;
			unsigned char b1_noise_state:1;
			unsigned char b2_freq_hopping:1;
			unsigned char b3_grip:1;
			unsigned char b4_palm:1;
			unsigned char b5__7_reserved:3;
			unsigned char reserved;
		} __packed;
		unsigned char data[2];
	};
};

/**
 * @brief: Custom Commands, Reports, or Events
 */
enum custom_report_type {
	REPORT_FW_STATUS = 0xc2,
	REPORT_HEAT_MAP = 0xc3,
};

#if defined(ENABLE_WAKEUP_GESTURE)
/**
 * @brief: Custom gesture type
 */
enum custom_gesture_type {
	GESTURE_SINGLE_TAP = 6,
	GESTURE_LONG_PRESS = 11,
};
#endif

#if defined(ENABLE_CUSTOM_TOUCH_ENTITY)
/**
 * @brief: Custom touch entity code
 */
enum custom_gesture_type {
	TOUCH_ENTITY_CUSTOM_ANGLE = 0xD1,
	TOUCH_ENTITY_CUSTOM_MAJOR = 0xD2,
	TOUCH_ENTITY_CUSTOM_MINOR = 0xD3,
};

enum custom_data {
	CUSTOM_DATA_ANGLE = 0x0,
	CUSTOM_DATA_MAJOR = 0x1,
	CUSTOM_DATA_MINOR = 0x2,
};
#endif

struct syna_health_check_fifo {
	ktime_t int_ktime;
	u64 int_idx;
	u64 coord_idx;
	u64 status_idx;
	/* Slot active bit from FW. */
	unsigned long active_bit;
	/* Check whether have coord, status or unknown event. */
	bool coord_updated;
	bool status_updated;
};

struct syna_touch_info_fifo {
	u8 idx;
	u16 x_pressed;	/* x coord on first down timing. */
	u16 y_pressed;	/* y coord on first down timing. */
	u16 x;
	u16 y;
	ktime_t ktime_pressed;
	ktime_t ktime_released;
};

struct syna_health_check {
	struct syna_health_check_fifo hc_fifo;
	u64 int_cnt;
	u64 coord_event_cnt;
	u64 status_event_cnt;
	unsigned long touch_idx_state;

	struct syna_touch_info_fifo touch_info_fifo[MAX_NUM_OBJECTS];
	u32 reset_cnt;
	u32 wet_cnt;
	u32 palm_cnt;
	u32 pressed_cnt;
	s64 longest_duration; /* ms unit */
};

/**
 * @brief: context of the synaptics linux-based driver
 *
 * The structure defines the kernel specific data in linux-based driver
 */
struct syna_tcm {

	/* TouchComm device core context */
	struct tcm_dev *tcm_dev;

	/* PLatform device driver */
	struct platform_device *pdev;

	/* Generic touched data generated by tcm core lib */
	struct tcm_touch_data_blob tp_data;

	syna_pal_mutex_t tp_event_mutex;

	unsigned char prev_obj_status[MAX_NUM_OBJECTS];

	/* Buffer stored the irq event data */
	struct tcm_buffer event_data;

	/* Hardware interface layer */
	struct syna_hw_interface *hw_if;

	/* ISR-related variables */
	pid_t isr_pid;
	bool irq_wake;

	/* cdev and sysfs nodes creation */
	struct cdev char_dev;
	dev_t char_dev_num;
	int char_dev_ref_count;

	struct class *device_class;
	struct device *device;

	struct kobject *sysfs_dir;

	/* Input device registration */
	struct input_dev *input_dev;
	struct input_params {
		unsigned int max_x;
		unsigned int max_y;
		unsigned int max_objects;
	} input_dev_params;

	/* Workqueue used for fw update */
	struct delayed_work reflash_work;
	struct workqueue_struct *reflash_workqueue;
	u8 reflash_count;
	bool force_reflash;

	struct work_struct suspend_work;
	struct work_struct resume_work;
	struct workqueue_struct *event_wq;
	struct completion bus_resumed;
	struct pinctrl *pinctrl;

	u32 bus_refmask;
	struct mutex bus_mutex;
	ktime_t bugreport_ktime_start;
	ktime_t isr_timestamp; /* Time that the event was first received from the
				* touch IC, acquired during hard interrupt, in
				* CLOCK_MONOTONIC */
	ktime_t coords_timestamp;

	struct syna_health_check syna_hc;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	struct touch_offload_context offload;
	u16 *heatmap_buff;
	struct touch_offload_frame *reserved_frame;
	bool offload_reserved_coords;
	u8 touch_offload_active_coords;
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	bool heatmap_decoded;
	struct v4l2_heatmap v4l2;
#endif

	/* Motion filter mode.
	 *  0 = Always unfilter.
	 *  1 = Dynamic change motion filter.
	 *  2 = Always filter by touch FW.
	 */
	u8 mf_mode;
	/* Payload for continuously report. */
	u16 set_continuously_report;
	/* Motion filter finite state machine (FSM) state */
	motion_filter_state_t mf_state;
	/* Time of initial single-finger touch down. This timestamp is used to
	 * compute the duration a single finger is touched before it is lifted.
	 */
	ktime_t mf_downtime;
	/* Work for motion filter commands. */
	struct work_struct motion_filter_work;

	/* Work for setting firmware grip mode. */
	struct work_struct set_grip_mode_work;
	/* Work for setting firmware palm mode. */
	struct work_struct set_palm_mode_work;

	/* IOCTL-related variables */
	pid_t proc_pid;
	struct task_struct *proc_task;

	int touch_count;
	bool touch_report_rate_config;
	bool next_report_rate_config;
	int last_vrefresh_rate;
	struct delayed_work set_report_rate_work;

	/* flags */
	int pwr_state;
	bool slept_in_early_suspend;
	bool lpwg_enabled;
	bool is_attn_redirecting;
	unsigned char fb_ready;
	bool is_connected;

	/* framebuffer callbacks notifier */
#if defined(ENABLE_DISP_NOTIFIER)
	struct notifier_block fb_notifier;
#endif
	u8 raw_data_report_code;
	s16 *raw_data_buffer;
	struct completion raw_data_completion;
	bool high_sensitivity_mode;
	u8 enable_fw_grip;
	u8 enable_fw_palm;
	u8 next_enable_fw_grip;
	u8 next_enable_fw_palm;

#if defined(USE_DRM_BRIDGE)
	struct drm_bridge panel_bridge;
	struct drm_connector *connector;
	bool is_panel_lp_mode;
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	u32 tbn_register_mask;
#endif

	struct pm_qos_request pm_qos_req;

	/* fifo to pass the data to userspace */
	unsigned int fifo_remaining_frame;
	struct list_head frame_fifo_queue;
	wait_queue_head_t wait_frame;
	unsigned char report_to_queue[REPORT_TYPES];

#if defined(ENABLE_HELPER)
	/* helper workqueue */
	struct syna_tcm_helper helper;
#endif

	/* Specific function pointer to do device connection.
	 *
	 * This function will power on and identify the connected device.
	 * At the end of function, the ISR will be registered as well.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_connect)(struct syna_tcm *tcm);

	/* Specific function pointer to disconnect the device
	 *
	 * This function will power off the connected device.
	 * Then, all the allocated resource will be released.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_disconnect)(struct syna_tcm *tcm);

	/* Specific function pointer to set up app fw firmware
	 *
	 * This function should be called whenever the device initially
	 * powers up, resets, or firmware update.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_set_up_app_fw)(struct syna_tcm *tcm);

	/* Specific function pointer to resume the device from suspend state.
	 *
	 * @param
	 *    [ in] dev: an instance of device
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_resume)(struct device *dev);

	/* Specific function pointer to put device into suspend state.
	 *
	 * @param
	 *    [ in] dev: an instance of device
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_suspend)(struct device *dev);
};

/**
 * @brief: Helpers for cdevice nodes and sysfs nodes creation
 *
 * These functions are implemented in syna_touchcom_sysfs.c
 * and available only when HAS_SYSFS_INTERFACE is enabled.
 */
#ifdef HAS_SYSFS_INTERFACE

int syna_cdev_create_sysfs(struct syna_tcm *ptcm,
		struct platform_device *pdev);

void syna_cdev_remove_sysfs(struct syna_tcm *ptcm);

void syna_cdev_redirect_attn(struct syna_tcm *ptcm);

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
void syna_cdev_update_report_queue(struct syna_tcm *tcm,
		unsigned char code, struct tcm_buffer *pevent_data);
#endif

#endif
int syna_set_bus_ref(struct syna_tcm *tcm, u32 ref, bool enable);
void syna_hc_dump(struct syna_tcm *tcm);
void syna_debug_dump(struct syna_tcm *tcm);

#endif /* end of _SYNAPTICS_TCM2_DRIVER_H_ */

