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

/*
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

#define PLATFORM_DRIVER_NAME "synaptics_tcm"

#define TOUCH_INPUT_NAME "synaptics_tcm_touch"
#define TOUCH_INPUT_PHYS_PATH "synaptics_tcm/touch_input"

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
#include <goog_touch_interface.h>
#endif

#define CHAR_DEVICE_NAME "tcm"
#define CHAR_DEVICE_MODE (0x0600)

#define SYNAPTICS_TCM_DRIVER_ID (1 << 0)
#define SYNAPTICS_TCM_DRIVER_VERSION 1
#define SYNAPTICS_TCM_DRIVER_SUBVER "5.6"

/*
 * @section: Driver Configurations
 *
 * The macros in the driver files below are used for doing compile time
 * configuration of the driver.
 */

/*
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

/*
 * @brief: TYPE_B_PROTOCOL
 *         Open to enable the multi-touch (MT) protocol
 */
#define TYPE_B_PROTOCOL

/*
 * @brief: POWER_SEQUENCE_ON_CONNECT
 *         Open if willing to issue the power sequence when connecting to the
 *         touch controller.
 *         Set "enable" in default.
 */
#define POWER_SEQUENCE_ON_CONNECT

/*
 * @brief: RESET_ON_CONNECT
 *         Open if willing to issue a reset when connecting to the
 *         touch controller.
 *         Set "enable" in default.
 */
#define RESET_ON_CONNECT

/*
 * @brief: RESET_ON_RESUME
 *         Open if willing to issue a reset to the touch controller
 *         from suspend.
 *         Set "disable" in default.
 */
/* #define RESET_ON_RESUME */

/*
 * @brief: GOOG_INT2_FEATURE
 *         Open if willing to issue a reset to the touch controller
 *         from suspend.
 *         Set "disable" in default.
 */
#define GOOG_INT2_FEATURE

/*
 * @brief ENABLE_WAKEUP_GESTURE
 *        Open if having wake-up gesture support.
 */
#define ENABLE_WAKEUP_GESTURE

/*
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

/*
 * @brief REPORT_TOUCH_WIDTH
 *        Open if willing to add the width data to the input event.
 */
#define REPORT_TOUCH_WIDTH

/*
 * @brief USE_CUSTOM_TOUCH_REPORT_CONFIG
 *        Open if willing to set up the format of touch report.
 *        The custom_touch_format[] array in syna_tcm2.c can be used
 *        to describe the customized report format.
 */
/* #define USE_CUSTOM_TOUCH_REPORT_CONFIG */

/*
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
/*
 * @brief  MULTICHIP_DUT_REFLASH
 *         Open if willing to do fw update and the DUT belongs to multi-chip
 *         product. This property dependent on STARTUP_REFLASH property.
 *
 *         Set "disable" in default.
 */
#if defined(HAS_ROMBOOT_REFLASH_FEATURE) && defined(STARTUP_REFLASH)
/* #define MULTICHIP_DUT_REFLASH */
#endif

/*
 * @section: STARTUP_REFLASH_DELAY_TIME_MS
 *           The delayed time to start fw update during the startup time.
 *           This configuration depends on STARTUP_REFLASH.
 */
#ifdef STARTUP_REFLASH
#define STARTUP_REFLASH_DELAY_TIME_MS (200)

#define FW_IMAGE_NAME "synaptics.img"
#endif

/*
 * @brief  ENABLE_DISP_NOTIFIER
 *         Open if having display notification event and willing to listen
 *         the event from display driver.
 *
 *         Set "disable" in default due to no generic notifier for DRM
 */
#if defined(CONFIG_FB) || defined(CONFIG_DRM_PANEL)
/* #define ENABLE_DISP_NOTIFIER */
#endif
/*
 * @brief RESUME_EARLY_UNBLANK
 *        Open if willing to resume in early un-blanking state.
 *
 *        This property is available only when ENABLE_DISP_NOTIFIER
 *        feature is enabled.
 */
#ifdef ENABLE_DISP_NOTIFIER
/* #define RESUME_EARLY_UNBLANK */
#endif
/*
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

/*
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

/*
 * @brief TCM_CONNECT_IN_PROBE
 *        Open if willing to detect and connect to TouchComm device at
 *        probe function; otherwise, please invoke connect() manually.
 *
 *        Set "enable" in default
 */
#define TCM_CONNECT_IN_PROBE

/*
 * @brief FORCE_CONNECTION
 *        Open if still connect to TouchComm device even error occurs.
 *
 *        Set "disable" in default
 */
/* #define FORCE_CONNECTION */

/*
 * @brief ENABLE_CUSTOM_TOUCH_ENTITY
 *        Open if having custom requirements to parse the custom code
 *        entity in the touch report.
 *
 *        Set "disable" in default
 */
#define ENABLE_CUSTOM_TOUCH_ENTITY

/*
 * @brief ENABLE_HELPER
 *        Open if willing to do additional handling upon helper workqueue
 *
 *        Set "disable" in default
 */
#define ENABLE_HELPER

/*
 * @brief: Power States
 *
 * Enumerate the power states of device
 */
enum power_state {
	PWR_OFF = 0,
	PWR_ON,
	LOW_PWR,
	BARE_MODE,
};

#if defined(ENABLE_HELPER)
/*
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

/*
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
			unsigned char b5_fast_relaxation:1;
			unsigned char b6__7_reserved:2;
			unsigned char reserved;
		} __packed;
		unsigned char data[2];
	};
};

/*
 * @brief: Custom Commands, Reports, or Events
 */
enum custom_report_type {
	REPORT_FW_STATUS = 0xc2,
	REPORT_HEAT_MAP = 0xc3,
	REPORT_TOUCH_AND_HEATMAP = 0xc5,
};

#if defined(ENABLE_WAKEUP_GESTURE)
/*
 * @brief: Custom gesture type
 */
enum custom_gesture_type {
	GESTURE_NONE = 0,
	GESTURE_SINGLE_TAP = 6,
	GESTURE_LONG_PRESS = 11,
};
#endif

#if defined(ENABLE_CUSTOM_TOUCH_ENTITY)
/*
 * @brief: Custom touch entity code
 */
enum custom_shape_data {
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

/*
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

	struct workqueue_struct *event_wq;
	struct pinctrl *pinctrl;

	ktime_t timestamp; /* Time that the event was first received from the
				* touch IC, acquired during hard interrupt, in
				* CLOCK_MONOTONIC */

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	/* Stored the last status data */
	struct custom_fw_status fw_status;

	unsigned short heatmap_mode;
	bool set_continuously_report;
	uint16_t *mutual_data;
	uint16_t *self_data;
	uint16_t *mutual_data_manual;
	uint16_t *self_data_manual;
	struct goog_touch_interface *gti;
	/* Work for setting coordinate filter. */
	struct work_struct set_coord_filter_work;
	/* Work for setting firmware grip mode. */
	struct work_struct set_grip_mode_work;
	/* Work for setting firmware palm mode. */
	struct work_struct set_palm_mode_work;
	/* Work for setting heatmap mode. */
	struct work_struct set_heatmap_enabled_work;
	/* Work for setting screen protector mode. */
	struct work_struct set_screen_protector_mode_work;
	/* Work for continuous report commands. */
	struct work_struct set_continuous_report_work;
#else
	syna_pal_mutex_t tp_event_mutex;
#endif
	syna_pal_mutex_t raw_data_mutex;

	/* IOCTL-related variables */
	pid_t proc_pid;
	struct task_struct *proc_task;

	int touch_count;

	/* flags */
	int pwr_state;
	bool slept_in_early_suspend;
	bool lpwg_enabled;
	bool is_attn_asserted;
	unsigned char fb_ready;
	bool is_connected;
	bool has_custom_tp_config;
	bool helper_enabled;
	bool startup_reflash_enabled;
	bool rst_on_resume_enabled;

	/* frame-buffer callbacks notifier */
#if defined(ENABLE_DISP_NOTIFIER)
	struct notifier_block fb_notifier;
#endif
	u8 raw_data_report_code;
	s16 *raw_data_buffer;
	struct completion raw_data_completion;
	bool coord_filter_enable;
	bool high_sensitivity_mode;
	u8 enable_fw_grip;
	u8 enable_fw_palm;

	/* fifo to pass the data to userspace */
	unsigned int fifo_remaining_frame;
	struct list_head frame_fifo_queue;
	wait_queue_head_t wait_frame;
	unsigned char report_to_queue[REPORT_TYPES];

#if defined(ENABLE_HELPER)
	/* helper workqueue */
	struct syna_tcm_helper helper;
#endif

	/* the pointer of userspace application info data */
	void *userspace_app_info;

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

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	/* Self test function pointer.
	 *
	 * @param
	 *    [ in] private_data: driver data
	 *    [out] cmd: self test result
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*selftest)(void *private_data, struct gti_selftest_cmd *cmd);
#endif
};

/*
 * @brief: Helpers for chardev nodes and sysfs nodes creation
  *
  * These functions are implemented in syna_touchcom_sysfs.c
  * and available only when HAS_SYSFS_INTERFACE is enabled.
  */
int syna_cdev_create(struct syna_tcm *ptcm,
		struct platform_device *pdev);

void syna_cdev_remove(struct syna_tcm *ptcm);

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
void syna_cdev_update_report_queue(struct syna_tcm *tcm,
		unsigned char code, struct tcm_buffer *pevent_data);
#endif

#ifdef HAS_SYSFS_INTERFACE

int syna_sysfs_create_dir(struct syna_tcm *ptcm,
		struct platform_device *pdev);

void syna_sysfs_remove_dir(struct syna_tcm *tcm);

#endif

ssize_t syna_get_fw_info(struct syna_tcm *tcm, char *buf, size_t buf_size);

bool syna_testing_compare_byte_vector(unsigned char *data,
		unsigned int data_size, const unsigned char *limit,
		unsigned int limit_size);

#endif /* end of _SYNAPTICS_TCM2_DRIVER_H_ */

