// SPDX-License-Identifier: GPL-2.0
/*
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
 * @file syna_tcm2.c
 *
 * This file implements the Synaptics device driver running under Linux kernel
 * input device subsystem, and also communicate with Synaptics touch controller
 * through TouchComm command-response protocol.
 */

#include "syna_tcm2.h"
#include "syna_tcm2_cdev.h"
#include "syna_tcm2_platform.h"
#include "syna_tcm2_testing_limits.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_touch.h"
#ifdef STARTUP_REFLASH
#ifdef HAS_ROMBOOT_REFLASH_FEATURE
#include "synaptics_touchcom_func_romboot.h"
#else
#include "synaptics_touchcom_func_reflash.h"
#endif
#endif

static irqreturn_t syna_dev_interrupt_thread(int irq, void *data);
static irqreturn_t syna_dev_isr(int irq, void *handle);
static void syna_dev_release_irq(struct syna_tcm *tcm);
static void syna_dev_restore_feature_setting(struct syna_tcm *tcm, unsigned int delay_ms_resp);

/*
 * @section: USE_CUSTOM_TOUCH_REPORT_CONFIG
 *           Open if willing to set up the format of touch report.
 *           The custom_touch_format[] array can be used to describe the
 *           customized report format.
 */
#ifdef USE_CUSTOM_TOUCH_REPORT_CONFIG
static unsigned char custom_touch_format[] = {
	/* entity code */                    /* bits */
#ifdef ENABLE_WAKEUP_GESTURE
	TOUCH_REPORT_GESTURE_ID,                8,
#endif
	TOUCH_REPORT_NUM_OF_ACTIVE_OBJECTS,     8,
	TOUCH_REPORT_FOREACH_ACTIVE_OBJECT,
	TOUCH_REPORT_OBJECT_N_INDEX,            8,
	TOUCH_REPORT_OBJECT_N_CLASSIFICATION,   8,
	TOUCH_REPORT_OBJECT_N_X_POSITION,       16,
	TOUCH_REPORT_OBJECT_N_Y_POSITION,       16,
	TOUCH_REPORT_FOREACH_END,
	TOUCH_REPORT_END
};
#endif

/*
 * @section: RESET_ON_RESUME_DELAY_MS
 *           The delayed time to issue a reset on resume state.
 *           This configuration depends on RESET_ON_RESUME.
 */
#ifdef RESET_ON_RESUME
#define RESET_ON_RESUME_DELAY_MS (100)
#endif

#define FW_UPDATE_DELAY_MS(erase, write) ((erase << 16) | write)

/*
 * @section: POWER_ALIVE_AT_SUSPEND
 *           indicate that the power is still alive even at
 *           system suspend.
 *           otherwise, there is no power supplied when system
 *           is going to suspend stage.
 */
#define POWER_ALIVE_AT_SUSPEND

/*
 * @section: global variables for an active drm panel
 *           in order to register display notifier
 */
#ifdef USE_DRM_PANEL_NOTIFIER
	struct drm_panel *active_panel;
#endif

#if IS_ENABLED(CONFIG_PM) || IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
static const struct dev_pm_ops syna_dev_pm_ops;
#endif

/*
 * syna_dev_enable_lowpwr_gesture()
 *
 * Enable or disable the low power gesture mode.
 * Furthermore, set up the wake-up irq.
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *    [ in] en:  '1' to enable low power gesture mode; '0' to disable
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_enable_lowpwr_gesture(struct syna_tcm *tcm, bool en)
{
	int retval = 0;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;

	if (!tcm->lpwg_enabled)
		return 0;

	if (attn->irq_id == 0)
		return 0;

	if (en) {
		if (!tcm->irq_wake) {
			enable_irq_wake(attn->irq_id);
			tcm->irq_wake = true;
		}

		/* enable wakeup gesture mode
		 *
		 * the wakeup gesture control may result from by finger event;
		 * therefore, it's better to use ATTN driven mode here
		 */
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE,
				1,
				RESP_IN_ATTN);
		if (retval < 0) {
			LOGE("Fail to enable wakeup gesture via DC command\n");
			return retval;
		}
	} else {
		if (tcm->irq_wake) {
			disable_irq_wake(attn->irq_id);
			tcm->irq_wake = false;
		}

		/* disable wakeup gesture mode
		 *
		 * the wakeup gesture control may result from by finger event;
		 * therefore, it's better to use ATTN driven mode here
		 */
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE,
				0,
				RESP_IN_ATTN);
		if (retval < 0) {
			LOGE("Fail to disable wakeup gesture via DC command\n");
			return retval;
		}
	}

	return retval;
}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
static int gti_default_handler(void *private_data, enum gti_cmd_type cmd_type,
	struct gti_union_cmd_data *cmd)
{
	LOGD("Not supported cmd_type %#x!", cmd_type);

	return -EOPNOTSUPP;
}

static int get_fw_version(void *private_data, struct gti_fw_version_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	int cmd_buffer_size = sizeof(cmd->buffer);
	int retval = 0;

	if (!tcm || !tcm->tcm_dev)
		return -ENODEV;

	retval = syna_tcm_identify(tcm->tcm_dev, &tcm->tcm_dev->id_info);
	if (retval < 0) {
		LOGE("Fail to get identification\n");
		return retval;
	}

	retval = syna_tcm_get_app_info(tcm->tcm_dev, &tcm->tcm_dev->app_info);
	if (retval < 0) {
		LOGE("Fail to get application info\n");
		return retval;
	}

	syna_get_fw_info(tcm, cmd->buffer, cmd_buffer_size);
	return 0;
}

static int get_irq_mode(void *private_data, struct gti_irq_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	cmd->setting = tcm->hw_if->bdata_attn.irq_enabled ?
			GTI_IRQ_MODE_ENABLE : GTI_IRQ_MODE_DISABLE;

	return 0;
}

static int set_irq_mode(void *private_data, struct gti_irq_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	return tcm->hw_if->ops_enable_irq(tcm->hw_if, cmd->setting == GTI_IRQ_MODE_ENABLE);
}

static int set_reset(void *private_data, struct gti_reset_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot trigger reset because touch is off");
		return -EPERM;
	}

	if (cmd->setting == GTI_RESET_MODE_HW || cmd->setting == GTI_RESET_MODE_AUTO) {
		tcm->hw_if->ops_hw_reset(tcm->hw_if);
	} else if (cmd->setting == GTI_RESET_MODE_SW) {
		syna_tcm_reset(tcm->tcm_dev);
		syna_dev_restore_feature_setting(tcm, RESP_IN_ATTN);
	} else {
		return -EOPNOTSUPP;
	}

	return 0;
}

static int syna_calibrate(void *private_data, struct gti_calibrate_cmd *cmd)
{
	(void)private_data;

	/* Return successful calibration since there is nothing to do. */
	cmd->result = GTI_CALIBRATE_RESULT_DONE;
	return 0;
}

static int syna_set_coord_filter_enabled(void *private_data, struct gti_coord_filter_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot set coordinate filter because touch is off");
		return -EPERM;
	}

	tcm->coord_filter_enable = cmd->setting == GTI_COORD_FILTER_ENABLE ? 1 : 0;

	if (tcm->hw_if->bdata_attn.irq_enabled) {
		queue_work(tcm->event_wq, &tcm->set_coord_filter_work);
	} else {
		LOGI("%s firmware coordinate filter.\n",
			tcm->coord_filter_enable ? "Enable" : "Disable");
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_COORD_FILTER,
			tcm->coord_filter_enable,
			RESP_IN_POLLING);
	}

	return 0;
}

static int syna_get_coord_filter_enabled(void *private_data, struct gti_coord_filter_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	unsigned short coord_filter_enabled;
	int retval;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot get coordinte filter because touch is off");
		return -EPERM;
	}

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev, DC_COORD_FILTER,
			&coord_filter_enabled, RESP_IN_POLLING);
	if (retval < 0) {
		LOGE("Fail to read coordinate filter, retval:%d.", retval);
		return -EIO;
	}

	cmd->setting = coord_filter_enabled ? GTI_COORD_FILTER_ENABLE : GTI_COORD_FILTER_DISABLE;

	return retval;
}

static void syna_set_coord_filter_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, set_coord_filter_work);
	int retval = 0;

	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval) {
		LOGE("Failed to obtain wake lock, ret = %d", retval);
		return;
	}

	LOGI("%s firmware coordinate filter.\n",
		tcm->coord_filter_enable ? "Enable" : "Disable");
	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_COORD_FILTER,
			tcm->coord_filter_enable,
			RESP_IN_ATTN);

	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
}

static int syna_set_palm_mode(void *private_data, struct gti_palm_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot set palm mode because touch is off");
		return -EPERM;
	}

	tcm->enable_fw_palm = cmd->setting == GTI_PALM_ENABLE ? 1 : 0;

	if (tcm->hw_if->bdata_attn.irq_enabled) {
		queue_work(tcm->event_wq, &tcm->set_palm_mode_work);
	} else {
		LOGI("%s firmware palm rejection.\n",
			(tcm->enable_fw_palm & 0x01) ? "Enable" : "Disable");
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_PALM_REJECTION,
			tcm->enable_fw_palm & 0x01,
			RESP_IN_POLLING);
	}

	return 0;
}

static int syna_get_palm_mode(void *private_data, struct gti_palm_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	unsigned short palm_mode;
	int retval;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot get palm mode because touch is off");
		return -EPERM;
	}

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev, DC_ENABLE_PALM_REJECTION,
			&palm_mode, RESP_IN_POLLING);
	if (retval < 0) {
		LOGE("Fail to read palm mode.");
		return retval;
	}

	cmd->setting = palm_mode ? GTI_PALM_ENABLE : GTI_PALM_DISABLE;

	return retval;
}

static void syna_set_palm_mode_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, set_palm_mode_work);
	int retval = 0;

	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval) {
		LOGE("Failed to obtain wake lock, ret = %d", retval);
		return;
	}

	LOGI("%s firmware palm rejection.\n",
		(tcm->enable_fw_palm & 0x01) ? "Enable" : "Disable");
	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_PALM_REJECTION,
			tcm->enable_fw_palm & 0x01,
			RESP_IN_ATTN);

	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
}

static int syna_set_grip_mode(void *private_data, struct gti_grip_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot set grip mode because touch is off");
		return -EPERM;
	}

	tcm->enable_fw_grip = cmd->setting == GTI_GRIP_ENABLE ? 1 : 0;

	if (tcm->hw_if->bdata_attn.irq_enabled) {
		queue_work(tcm->event_wq, &tcm->set_grip_mode_work);
	} else {
		LOGI("%s firmware grip suppression.\n",
			(tcm->enable_fw_grip & 0x01) ? "Enable" : "Disable");
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_GRIP_SUPPRESSION,
			tcm->enable_fw_grip & 0x01,
			RESP_IN_POLLING);
	}

	return 0;
}

static int syna_get_grip_mode(void *private_data, struct gti_grip_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	unsigned short grip_mode;
	int retval;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot get grip mode because touch is off");
		return -EPERM;
	}

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev, DC_ENABLE_GRIP_SUPPRESSION,
			&grip_mode, RESP_IN_POLLING);
	if (retval < 0) {
		LOGE("Fail to read grip mode.");
		return retval;
	}

	cmd->setting = grip_mode ? GTI_GRIP_ENABLE : GTI_GRIP_DISABLE;

	return retval;
}

static void syna_set_grip_mode_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, set_grip_mode_work);
	int retval = 0;

	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval) {
		LOGE("Failed to obtain wake lock, ret = %d", retval);
		return;
	}

	LOGI("%s firmware grip suppression.\n",
		(tcm->enable_fw_grip & 0x01) ? "Enable" : "Disable");
	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_GRIP_SUPPRESSION,
			tcm->enable_fw_grip & 0x01,
			RESP_IN_ATTN);

	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
}

static int syna_set_heatmap_enabled(void *private_data, struct gti_heatmap_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot set heatmap mode because touch is off");
		return -EPERM;
	}

	tcm->heatmap_mode = cmd->setting == GTI_HEATMAP_ENABLE ?
			HEATMAP_MODE_COMBINED : HEATMAP_MODE_COORD;

	if (tcm->hw_if->bdata_attn.irq_enabled) {
		queue_work(tcm->event_wq, &tcm->set_heatmap_enabled_work);
	} else {
		LOGI("Set heatmap mode %d.\n", tcm->heatmap_mode);
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_HEATMAP_MODE,
			tcm->heatmap_mode,
			RESP_IN_POLLING);
	}

	return 0;
}

static void syna_set_heatmap_enabled_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, set_heatmap_enabled_work);
	int retval = 0;

	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval) {
		LOGE("Failed to obtain wake lock, ret = %d", retval);
		return;
	}

	LOGI("Set heatmap mode %d.\n", tcm->heatmap_mode);

	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_HEATMAP_MODE,
			tcm->heatmap_mode,
			RESP_IN_ATTN);

	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
}

static int syna_set_scan_mode(void *private_data, struct gti_scan_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	int retval = 0;
	unsigned short gesture_mode;
	bool doze_enable;

	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval < 0) {
		LOGE("Failed to obtain wake lock, ret = %d", retval);
		return retval;
	}

	switch (cmd->setting) {
	case GTI_SCAN_MODE_NORMAL_ACTIVE:
		gesture_mode = 0;
		doze_enable = false;
		break;
	case GTI_SCAN_MODE_NORMAL_IDLE:
		gesture_mode = 0;
		doze_enable = true;
		break;
	case GTI_SCAN_MODE_LP_ACTIVE:
		gesture_mode = 1;
		doze_enable = false;
		break;
	case GTI_SCAN_MODE_LP_IDLE:
		gesture_mode = 1;
		doze_enable = true;
		break;
	default:
		LOGE("Invalid scan mode %d.", cmd->setting);
		retval = -EINVAL;
		goto exit;
	}

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_WAKEUP_GESTURE_MODE,
			gesture_mode,
			RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Fail to set wakeup gesture mode via DC command, retval:%d\n", retval);
		retval = -EIO;
		goto exit;
	}

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_DISABLE_DOZE,
			doze_enable ? 0 : 1,
			RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Fail to set DC_DISABLE_DOZE, retval:%d\n", retval);
		retval = -EIO;
		goto exit;
	}

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_FORCE_DOZE_MODE,
			doze_enable ? 1 : 0,
			RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Fail to set DC_FORCE_DOZE_MODE, retval:%d\n", retval);
		retval = -EIO;
	}

exit:
	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
	return retval;
}

static int syna_get_scan_mode(void *private_data, struct gti_scan_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	unsigned short scan_mode;
	int retval;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot get scan mode because touch is off");
		return -EPERM;
	}

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev, DC_TOUCH_SCAN_MODE,
			&scan_mode, RESP_IN_POLLING);
	if (retval < 0) {
		LOGE("Fail to read scan mode, retval:%d", retval);
		return -EIO;
	}

	switch (scan_mode) {
	case SCAN_NORMAL_IDLE:
		cmd->setting = GTI_SCAN_MODE_NORMAL_IDLE;
		break;
	case SCAN_NORMAL_ACTIVE:
		cmd->setting = GTI_SCAN_MODE_NORMAL_ACTIVE;
		break;
	case SCAN_LPWG_IDLE:
		cmd->setting = GTI_SCAN_MODE_LP_IDLE;
		break;
	case SCAN_LPWG_ACTIVE:
		cmd->setting = GTI_SCAN_MODE_LP_ACTIVE;
		break;
	case SCAN_SLEEP:
		LOGI("Touch is in sleep mode.");
		retval = -EINVAL;
		break;
	default:
		LOGE("Invalid scan mode %u", scan_mode);
		retval = -EINVAL;
		break;
	}

	return retval;
}

static int syna_set_sensing_mode(void *private_data, struct gti_sensing_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	int retval = 0;

	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval < 0) {
		LOGE("Failed to obtain wake lock, ret = %d", retval);
		return retval;
	}

	if (cmd->setting == GTI_SENSING_MODE_DISABLE) {
		retval = syna_tcm_sleep(tcm->tcm_dev, true);
		if (retval < 0) {
			LOGE("Failed enter deep sleep mode, ret:%d", retval);
			retval = -EIO;
		}
	} else if (cmd->setting == GTI_SENSING_MODE_ENABLE) {
		retval = syna_tcm_sleep(tcm->tcm_dev, false);
		if (retval < 0) {
			LOGE("Failed exit deep sleep mode, ret:%d", retval);
			retval = -EIO;
		}
	} else {
		LOGE("Invalid sensing mode %d", cmd->setting);
		retval = -EINVAL;
	}

	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
	return retval;
}

static int syna_get_sensing_mode(void *private_data, struct gti_sensing_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	int retval = 0;
	unsigned short scan_mode;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot get sensing mode because touch is off");
		return -EPERM;
	}

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev, DC_TOUCH_SCAN_MODE,
			&scan_mode, RESP_IN_POLLING);
	if (retval < 0) {
		LOGE("Fail to read sensing mode, retval:%d", retval);
		return -EIO;
	}

	switch (scan_mode) {
	case SCAN_SLEEP:
		cmd->setting = GTI_SENSING_MODE_DISABLE;
		break;
	case SCAN_NORMAL_IDLE:
	case SCAN_NORMAL_ACTIVE:
	case SCAN_LPWG_IDLE:
	case SCAN_LPWG_ACTIVE:
		cmd->setting = GTI_SENSING_MODE_ENABLE;
		break;
	default:
		LOGE("Invalid scan mode %d", scan_mode);
		retval = -EINVAL;
		break;
	}

	return retval;
}

static int syna_set_screen_protector_mode(void *private_data,
		struct gti_screen_protector_mode_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot set screen protector mode because touch is off");
		return -EPERM;
	}

	tcm->high_sensitivity_mode = cmd->setting == GTI_SCREEN_PROTECTOR_MODE_ENABLE ? 1 : 0;

	if (tcm->hw_if->bdata_attn.irq_enabled) {
		queue_work(tcm->event_wq, &tcm->set_screen_protector_mode_work);
	} else {
		LOGI("%s screen protector mode.\n",
				tcm->high_sensitivity_mode ? "Enable" : "Disable");
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_HIGH_SENSITIVITY_MODE,
				tcm->high_sensitivity_mode,
				RESP_IN_POLLING);
	}

	return 0;
}

static int syna_get_screen_protector_mode(void *private_data,
		struct gti_screen_protector_mode_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	unsigned short screen_protector_mode;
	int retval;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot get screen protector mode because touch is off");
		return -EPERM;
	}

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev, DC_HIGH_SENSITIVITY_MODE,
			&screen_protector_mode, RESP_IN_POLLING);
	if (retval < 0) {
		LOGE("Fail to read screen protector mode.");
		return retval;
	}

	cmd->setting = screen_protector_mode ?
			GTI_SCREEN_PROTECTOR_MODE_ENABLE : GTI_SCREEN_PROTECTOR_MODE_DISABLE;

	return retval;
}

static void syna_set_screen_protector_mode_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, set_screen_protector_mode_work);
	int retval = 0;

	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval) {
		LOGE("Failed to obtain wake lock, ret = %d", retval);
		return;
	}

	LOGI("%s screen protector mode.\n",
		tcm->high_sensitivity_mode ? "Enable" : "Disable");
	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_HIGH_SENSITIVITY_MODE,
			tcm->high_sensitivity_mode,
			RESP_IN_ATTN);

	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
}

static u8 syna_gesture_dc_list[GTI_GESTURE_PARAMS_MAX] = {
	[GTI_STTW_MIN_X] = DC_STTW_MIN_X,
	[GTI_STTW_MAX_X] = DC_STTW_MAX_X,
	[GTI_STTW_MIN_Y] = DC_STTW_MIN_Y,
	[GTI_STTW_MAX_Y] = DC_STTW_MAX_Y,
	[GTI_STTW_MIN_FRAME] = DC_STTW_MIN_FRAME,
	[GTI_STTW_MAX_FRAME] = DC_STTW_MAX_FRAME,
	[GTI_STTW_JITTER] = DC_STTW_JITTER,
	[GTI_STTW_MAX_TOUCH_SIZE] = DC_STTW_MAX_TOUCH_SIZE,
	[GTI_LPTW_MIN_X] = DC_LPTW_MIN_X,
	[GTI_LPTW_MAX_X] = DC_LPTW_MAX_X,
	[GTI_LPTW_MIN_Y] = DC_LPTW_MIN_Y,
	[GTI_LPTW_MAX_Y] = DC_LPTW_MAX_Y,
	[GTI_LPTW_MIN_FRAME] = DC_LPTW_MIN_FRAME,
	[GTI_LPTW_JITTER] = DC_LPTW_JITTER,
	[GTI_LPTW_MAX_TOUCH_SIZE] = DC_LPTW_MAX_TOUCH_SIZE,
	[GTI_LPTW_MARGINAL_MIN_X] = DC_LPTW_MARGINAL_MIN_X,
	[GTI_LPTW_MARGINAL_MAX_X] = DC_LPTW_MARGINAL_MAX_X,
	[GTI_LPTW_MARGINAL_MIN_Y] = DC_LPTW_MARGINAL_MIN_Y,
	[GTI_LPTW_MARGINAL_MAX_Y] = DC_LPTW_MARGINAL_MAX_Y,
	[GTI_LPTW_MONITOR_CH_MIN_TX] = DC_LPTW_MONITOR_CH_MIN_TX,
	[GTI_LPTW_MONITOR_CH_MAX_TX] = DC_LPTW_MONITOR_CH_MAX_TX,
	[GTI_LPTW_MONITOR_CH_MIN_RX] = DC_LPTW_MONITOR_CH_MIN_RX,
	[GTI_LPTW_MONITOR_CH_MAX_RX] = DC_LPTW_MONITOR_CH_MAX_RX,
	[GTI_LPTW_NODE_COUNT_MIN] = DC_LPTW_NODE_COUNT_MIN,
	[GTI_LPTW_MOTION_BOUNDARY] = DC_LPTW_MOTION_BOUNDARY,
};

static int syna_set_gesture_type(struct syna_tcm *tcm, u8 gesture_type)
{
	int retval = 0;
	unsigned short set_gesture_type = 0;

	if (gesture_type == GTI_GESTURE_DISABLE) {
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE,
				0,
				RESP_IN_POLLING);
		if (retval)
			goto exit;

		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_HEATMAP_MODE,
				HEATMAP_MODE_COMBINED,
				RESP_IN_POLLING);
	} else {
		if (gesture_type == GTI_GESTURE_STTW) {
			set_gesture_type = GESTURE_TYPE_STTW;
		} else if (gesture_type == GTI_GESTURE_LPTW) {
			set_gesture_type = GESTURE_TYPE_LPTW;
		} else if (gesture_type == GTI_GESTURE_STTW_AND_LPTW) {
			set_gesture_type = GESTURE_TYPE_STTW_AND_LPTW;
		} else {
			LOGE("Unsuppoted gesture type %d", gesture_type);
			retval = -EINVAL;
			goto exit;
		}

		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE,
				1,
				RESP_IN_POLLING);
		if (retval)
			goto exit;

		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_GESTURE_TYPE,
				set_gesture_type,
				RESP_IN_POLLING);
		if (retval)
			goto exit;

		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_HEATMAP_MODE,
				HEATMAP_MODE_COORD,
				RESP_IN_POLLING);
	}

exit:
	return retval;
}

static int syna_set_gesture_config(void *private_data, struct gti_gesture_config_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	int retval = 0;
	int i = 0;

	LOGI("Set gesture config");

	for (i = 0; i < GTI_GESTURE_PARAMS_MAX; i++) {
		if (cmd->updating_params[i]) {
			if (i == GTI_GESTURE_TYPE) {
				syna_set_gesture_type(tcm, cmd->params[i]);
			} else {
				retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
						syna_gesture_dc_list[i],
						cmd->params[i],
						RESP_IN_POLLING);
			}

			if (retval)
				goto exit;
		}
	}

exit:
	return retval;
}

static int syna_set_continuous_report(void *private_data, struct gti_continuous_report_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;

	if (goog_pm_wake_get_locks(tcm->gti) == 0 || tcm->pwr_state != PWR_ON) {
		LOGI("Connot set continuous report because touch is off");
		return -EPERM;
	}

	tcm->set_continuously_report = cmd->setting == GTI_CONTINUOUS_REPORT_ENABLE ? 1 : 0;

	if (tcm->hw_if->bdata_attn.irq_enabled) {
		queue_work(tcm->event_wq, &tcm->set_continuous_report_work);
	} else {
		LOGI("%s continuous report.\n",
				tcm->set_continuously_report ? "Enable" : "Disable");
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_CONTINUOUSLY_REPORT,
				tcm->set_continuously_report,
				RESP_IN_POLLING);
	}

	return 0;
}

static void syna_set_continuous_report_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, set_continuous_report_work);
	int retval = 0;

	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval) {
		LOGE("Failed to obtain wake lock, ret = %d", retval);
		return;
	}

	/* Send command to update continuous report state */
	LOGD("%s continuous report.\n", tcm->set_continuously_report ? "Enable" : "Disable");
	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_CONTINUOUSLY_REPORT,
			tcm->set_continuously_report,
			RESP_IN_ATTN);

	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
}

static int syna_get_mutual_sensor_data(void *private_data, struct gti_sensor_data_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	unsigned char report_code = 0;
	int ret = 0;
	int i, j;
	unsigned int rows = tcm->tcm_dev->rows;
	unsigned int cols = tcm->tcm_dev->cols;

	if (cmd->type == GTI_SENSOR_DATA_TYPE_MS) {
		cmd->buffer = (u8 *)tcm->mutual_data;
		cmd->size = rows * cols * sizeof(uint16_t);
		return 0;
	}

	switch (cmd->type) {
	case GTI_SENSOR_DATA_TYPE_MS_DIFF:
		report_code = REPORT_DELTA;
		cmd->is_unsigned = false;
		break;
	case GTI_SENSOR_DATA_TYPE_MS_RAW:
		report_code = REPORT_RAW;
		cmd->is_unsigned = true;
		break;
	case GTI_SENSOR_DATA_TYPE_MS_BASELINE:
		report_code = REPORT_BASELINE;
		cmd->is_unsigned = true;
		break;
	default:
		LOGE("Unsupported report type %u", cmd->type);
		return -EINVAL;
	}

	reinit_completion(&tcm->raw_data_completion);

	syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_DISABLE_DOZE, 1, RESP_IN_ATTN);

	tcm->raw_data_report_code = report_code;
	syna_tcm_enable_report(tcm->tcm_dev, tcm->raw_data_report_code, true);

	if (wait_for_completion_timeout(&tcm->raw_data_completion, msecs_to_jiffies(500)) == 0) {
		LOGE("Wait for sensor data %#x timeout.", cmd->type);
		ret = -ETIMEDOUT;
		goto exit;
	}

	syna_pal_mutex_lock(&tcm->raw_data_mutex);
	for (i = 0; i < cols; i++) {
		for (j = 0; j < rows; j++) {
			tcm->mutual_data_manual[i * rows + j] =
					(u16) (tcm->raw_data_buffer[j * cols + i]);
		}
	}
	syna_pal_mutex_unlock(&tcm->raw_data_mutex);

	cmd->buffer = (u8 *)tcm->mutual_data_manual;
	cmd->size = rows * cols * sizeof(u16);
exit:
	syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_DISABLE_DOZE, 0, RESP_IN_ATTN);
	syna_tcm_enable_report(tcm->tcm_dev, tcm->raw_data_report_code, false);

	return ret;
}

static int syna_get_self_sensor_data(void *private_data, struct gti_sensor_data_cmd *cmd)
{
	struct syna_tcm *tcm = private_data;
	unsigned char report_code = 0;
	int ret = 0;
	int i;
	unsigned int rows = tcm->tcm_dev->rows;
	unsigned int cols = tcm->tcm_dev->cols;

	if (cmd->type == GTI_SENSOR_DATA_TYPE_SS) {
		cmd->buffer = (u8 *)tcm->self_data;
		cmd->size = (rows + cols) * sizeof(uint16_t);
		return 0;
	}

	switch (cmd->type) {
	case GTI_SENSOR_DATA_TYPE_SS_DIFF:
		report_code = REPORT_DELTA;
		cmd->is_unsigned = false;
		break;
	case GTI_SENSOR_DATA_TYPE_SS_RAW:
		report_code = REPORT_RAW;
		cmd->is_unsigned = true;
		break;
	case GTI_SENSOR_DATA_TYPE_SS_BASELINE:
		report_code = REPORT_BASELINE;
		cmd->is_unsigned = true;
		break;
	default:
		LOGE("Unsupported report type %u", cmd->type);
		return -EINVAL;
	}

	reinit_completion(&tcm->raw_data_completion);

	syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_DISABLE_DOZE, 1, RESP_IN_ATTN);

	tcm->raw_data_report_code = report_code;
	syna_tcm_enable_report(tcm->tcm_dev, tcm->raw_data_report_code, true);

	if (wait_for_completion_timeout(&tcm->raw_data_completion, msecs_to_jiffies(500)) == 0) {
		LOGE("Wait for sensor data %#x timeout.", cmd->type);
		ret = -ETIMEDOUT;
		goto exit;
	}

	syna_pal_mutex_lock(&tcm->raw_data_mutex);
	for (i = 0; i < rows; i++)
		tcm->self_data_manual[i] = (u16) (tcm->raw_data_buffer[rows * cols + cols + i]);

	for (i = 0; i < cols; i++)
		tcm->self_data_manual[rows + i] = (u16) (tcm->raw_data_buffer[rows * cols + i]);
	syna_pal_mutex_unlock(&tcm->raw_data_mutex);

	cmd->buffer = (u8 *)tcm->self_data_manual;
	cmd->size = (rows + cols) * sizeof(u16);
exit:
	syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_DISABLE_DOZE, 0, RESP_IN_ATTN);
	syna_tcm_enable_report(tcm->tcm_dev, tcm->raw_data_report_code, false);

	return ret;
}

static int syna_dev_ptflib_decoder(struct syna_tcm *tcm, const u16 *in_array,
				   const int in_array_size, u16 *out_array,
				   const int out_array_max_size)
{
	const u16 ESCAPE_MASK = 0xF000;
	const u16 ESCAPE_BIT = 0x8000;

	int i;
	int j;
	int out_array_size = 0;
	u16 prev_word = 0;
	u16 repetition = 0;
	u16 *temp_out_array = out_array;

	for (i = 0; i < in_array_size; i++) {
		u16 curr_word = in_array[i];
		if ((curr_word & ESCAPE_MASK) == ESCAPE_BIT) {
			repetition = (curr_word & ~ESCAPE_MASK);
			if (out_array_size + repetition > out_array_max_size)
				break;
			for (j = 0; j < repetition; j++) {
				*temp_out_array++ = prev_word;
				out_array_size++;
			}
		} else {
			if (out_array_size >= out_array_max_size)
				break;
			*temp_out_array++ = curr_word;
			out_array_size++;
			prev_word = curr_word;
		}
	}

	if (i != in_array_size || out_array_size != out_array_max_size) {
		LOGE("%d (in=%d, out=%d, rep=%d, out_max=%d).\n",
			i, in_array_size, out_array_size,
			repetition, out_array_max_size);
		memset(out_array, 0, out_array_max_size * sizeof(u16));
		return -1;
	}

	return out_array_size;
}

static void syna_parse_heatmap(struct syna_tcm *tcm, unsigned char *heatmap_data,
		unsigned short heatmap_data_size)
{
	int i, j;
	uint16_t *temp_buffer;
	unsigned int rows = tcm->tcm_dev->rows;
	unsigned int cols = tcm->tcm_dev->cols;

	temp_buffer = kcalloc(cols * rows, sizeof(u_int16_t), GFP_KERNEL);
	if (!temp_buffer) {
		LOGE("Failed to allocate temp_buffer");
		return;
	}

	if (!tcm->self_data || !tcm->mutual_data) {
		LOGE("There is no self_data or mutual_data");
		return;
	}

	/* Parse self data. */
	for (i = 0; i < rows; i++)
		tcm->self_data[i] = ((uint16_t *) heatmap_data)[cols + i];

	for (i = 0; i < cols; i++)
		tcm->self_data[rows + i] = ((uint16_t *) heatmap_data)[i];

	/* Parse mutual data. */
	syna_dev_ptflib_decoder(tcm, &((u16 *) heatmap_data)[cols + rows],
			heatmap_data_size / 2 - cols - rows, temp_buffer, cols * rows);

	for (i = 0; i < cols; i++) {
		for (j = 0; j < rows; j++)
			tcm->mutual_data[rows * i + j] = temp_buffer[cols * j + i];
	}

	kfree(temp_buffer);
}

static void syna_gti_init(struct syna_tcm *tcm)
{
	int retval = 0;
	struct gti_optional_configuration *options;
	struct platform_device *pdev = tcm->pdev;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
	unsigned int mutual_data_size =
			sizeof(u_int16_t) * (tcm->tcm_dev->rows * tcm->tcm_dev->cols);
	unsigned int self_data_size =
			sizeof(u_int16_t) * (tcm->tcm_dev->rows + tcm->tcm_dev->cols);

	tcm->mutual_data = devm_kzalloc(&pdev->dev, mutual_data_size, GFP_KERNEL);
	if (!tcm->mutual_data) {
		LOGE("Failed to allocate mutual_sensing_data");
		return;
	}

	tcm->self_data = devm_kzalloc(&pdev->dev, self_data_size, GFP_KERNEL);
	if (!tcm->self_data) {
		LOGE("Failed to allocate self_sensing_data");
		return;
	}

	tcm->mutual_data_manual = devm_kzalloc(&pdev->dev, mutual_data_size, GFP_KERNEL);
	if (!tcm->mutual_data) {
		LOGE("Failed to allocate mutual_sensing_data");
		return;
	}

	tcm->self_data_manual = devm_kzalloc(&pdev->dev, self_data_size, GFP_KERNEL);
	if (!tcm->self_data) {
		LOGE("Failed to allocate self_sensing_data");
		return;
	}

	/* release the interrupt and register the gti irq later. */
	syna_dev_release_irq(tcm);

	INIT_WORK(&tcm->set_coord_filter_work, syna_set_coord_filter_work);
	INIT_WORK(&tcm->set_grip_mode_work, syna_set_grip_mode_work);
	INIT_WORK(&tcm->set_palm_mode_work, syna_set_palm_mode_work);
	INIT_WORK(&tcm->set_heatmap_enabled_work, syna_set_heatmap_enabled_work);
	INIT_WORK(&tcm->set_screen_protector_mode_work, syna_set_screen_protector_mode_work);
	INIT_WORK(&tcm->set_continuous_report_work, syna_set_continuous_report_work);

	pdev->dev.of_node = pdev->dev.parent->of_node;
	options = devm_kzalloc(&pdev->dev, sizeof(struct gti_optional_configuration), GFP_KERNEL);

	options->get_fw_version = get_fw_version;
	options->get_irq_mode = get_irq_mode;
	options->set_irq_mode = set_irq_mode;
	options->reset = set_reset;
	options->calibrate = syna_calibrate;
	options->selftest = tcm->selftest;
	options->get_coord_filter_enabled = syna_get_coord_filter_enabled;
	options->set_coord_filter_enabled = syna_set_coord_filter_enabled;
	options->set_grip_mode = syna_set_grip_mode;
	options->get_grip_mode = syna_get_grip_mode;
	options->set_palm_mode = syna_set_palm_mode;
	options->get_palm_mode = syna_get_palm_mode;
	options->set_heatmap_enabled = syna_set_heatmap_enabled;
	options->set_scan_mode = syna_set_scan_mode;
	options->get_scan_mode = syna_get_scan_mode;
	options->set_sensing_mode = syna_set_sensing_mode;
	options->get_sensing_mode = syna_get_sensing_mode;
	options->set_screen_protector_mode = syna_set_screen_protector_mode;
	options->get_screen_protector_mode = syna_get_screen_protector_mode;
	options->set_gesture_config = syna_set_gesture_config;
	options->set_continuous_report = syna_set_continuous_report;
	options->get_mutual_sensor_data = syna_get_mutual_sensor_data;
	options->get_self_sensor_data = syna_get_self_sensor_data;

	tcm->gti = goog_touch_interface_probe(
		tcm, &pdev->dev, tcm->input_dev, gti_default_handler, options);
	if (!tcm->gti) {
		LOGE("Failed to initialize GTI");
		return;
	}

	retval = goog_pm_register_notification(tcm->gti, &syna_dev_pm_ops);
	if (retval < 0)
		LOGE("Failed to register GTI pm");

	LOGI("Register IRQ by GTI.");
	attn->irq_id = gpio_to_irq(attn->irq_gpio);
	retval = goog_devm_request_threaded_irq(tcm->gti, &tcm->pdev->dev,
			attn->irq_id, syna_dev_isr, syna_dev_interrupt_thread,
			attn->irq_flags, PLATFORM_DRIVER_NAME, tcm);
	if (retval < 0)
		LOGE("Failed to request GTI IRQ");
	else
		attn->irq_enabled = true;

	syna_dev_restore_feature_setting(tcm, RESP_IN_ATTN);
}

static void syna_notify_fw_status(struct syna_tcm *tcm, struct custom_fw_status *status)
{
	struct gti_fw_status_data gti_status_data = { 0 };

	if (!tcm->gti)
		return;

	if (status->b0_moisture != tcm->fw_status.b0_moisture) {
		goog_notify_fw_status_changed(tcm->gti,
			status->b0_moisture ? GTI_FW_STATUS_WATER_ENTER : GTI_FW_STATUS_WATER_EXIT,
			&gti_status_data);
	}
	if (status->b1_noise_state != tcm->fw_status.b1_noise_state) {
		gti_status_data.noise_level = status->b1_noise_state;
		goog_notify_fw_status_changed(tcm->gti,
			GTI_FW_STATUS_NOISE_MODE, &gti_status_data);
	}
	if (status->b3_grip != tcm->fw_status.b3_grip) {
		goog_notify_fw_status_changed(tcm->gti,
			status->b3_grip ? GTI_FW_STATUS_GRIP_ENTER : GTI_FW_STATUS_GRIP_EXIT,
			&gti_status_data);
	}
	if (status->b4_palm != tcm->fw_status.b4_palm) {
		goog_notify_fw_status_changed(tcm->gti,
			status->b4_palm ? GTI_FW_STATUS_PALM_ENTER : GTI_FW_STATUS_PALM_EXIT,
			&gti_status_data);
	}
	memcpy(&tcm->fw_status, status, sizeof(tcm->fw_status));
}
#endif

/*
 * syna_dev_restore_feature_setting()
 *
 * Restore the feature settings after the device resume.
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *    [ in] delay_ms_resp: delay time for response reading.
 *                         a positive value presents the time for polling;
 *                         or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static void syna_dev_restore_feature_setting(struct syna_tcm *tcm, unsigned int delay_ms_resp)
{
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	struct gti_fw_status_data gti_status_data = { 0 };
	struct custom_fw_status status = { 0 };
#endif

	LOGI("Restore touch feature settings.");

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	syna_notify_fw_status(tcm, &status);
	goog_notify_fw_status_changed(tcm->gti, GTI_FW_STATUS_RESET, &gti_status_data);
#endif

	if (tcm->hw_if->compression_threshold != 0) {
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_COMPRESSION_THRESHOLD,
				tcm->hw_if->compression_threshold,
				delay_ms_resp);
	}

	if (tcm->hw_if->grip_delta_threshold != 0) {
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_GRIP_DELTA_THRESHOLD,
				tcm->hw_if->grip_delta_threshold,
				delay_ms_resp);
	}

	if (tcm->hw_if->grip_border_threshold != 0) {
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_GRIP_BORDER_THRESHOLD,
				tcm->hw_if->grip_border_threshold,
				delay_ms_resp);
	}
}

#if defined(ENABLE_HELPER)
static void syna_dev_get_reset_reason(struct syna_tcm *tcm)
{
	int retval;
	struct tcm_boot_info boot_info;

	retval = syna_tcm_switch_fw_mode(tcm->tcm_dev,
			MODE_BOOTLOADER,
			FW_MODE_SWITCH_DELAY_MS);
	if (retval < 0) {
		LOGE("Fail to enter bootloader mode\n");
		goto exit;
	}

	retval = syna_tcm_get_boot_info(tcm->tcm_dev, &boot_info);
	if (retval < 0) {
		LOGE("Fail to get boot info");
		goto exit;
	}

	LOGI("Boot info: %*ph", (int) sizeof(struct tcm_boot_info), (unsigned char*) &boot_info);

exit:
	retval = syna_tcm_switch_fw_mode(tcm->tcm_dev,
			MODE_APPLICATION_FIRMWARE,
			FW_MODE_SWITCH_DELAY_MS);
	if (retval < 0)
		LOGE("Fail to go back to application firmware\n");
}

/*
 * syna_dev_reset_detected_cb()
 *
 * Callback to assign a task to event workqueue.
 *
 * Please be noted that this function will be invoked in ISR so don't
 * issue another touchcomm command here.
 *
 * @param
 *    [ in] callback_data: pointer to caller data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static void syna_dev_reset_detected_cb(void *callback_data)
{
	struct syna_tcm *tcm = (struct syna_tcm *)callback_data;

#ifdef RESET_ON_RESUME
	if (tcm->pwr_state != PWR_ON)
		return;
#endif

	if (ATOMIC_GET(tcm->helper.task) == HELP_NONE) {
		ATOMIC_SET(tcm->helper.task, HELP_RESET_DETECTED);

		queue_work(tcm->event_wq, &tcm->helper.work);
	}
}
/*
 * syna_dev_helper_work()
 *
 * According to the given task, perform the delayed work
 *
 * @param
 *    [ in] work: data for work used
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static void syna_dev_helper_work(struct work_struct *work)
{
	unsigned char task;
	int retval;
	struct syna_tcm_helper *helper =
			container_of(work, struct syna_tcm_helper, work);
	struct syna_tcm *tcm =
			container_of(helper, struct syna_tcm, helper);

	if (tcm->pwr_state != PWR_ON) {
		LOGI("Touch is already off.");
		goto exit;
	}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	retval = goog_pm_wake_lock(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST, true);
	if (retval) {
		LOGI("%s: Failed to obtain wake lock, ret = %d", __func__, retval);
		goto exit;
	}
	syna_dev_get_reset_reason(tcm);
#endif

	task = ATOMIC_GET(helper->task);

	switch (task) {
	case HELP_RESET_DETECTED:
		LOGI("Reset caught (device mode:0x%x)\n", tcm->tcm_dev->dev_mode);
		syna_dev_restore_feature_setting(tcm, RESP_IN_ATTN);
		break;
	default:
		break;
	}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	goog_pm_wake_unlock_nosync(tcm->gti, GTI_PM_WAKELOCK_TYPE_VENDOR_REQUEST);
#endif

exit:
	ATOMIC_SET(helper->task, HELP_NONE);
}
#endif

#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
/*
 * syna_dev_parse_custom_touch_data_cb()
 *
 * Callback to parse the custom or non-standard touch entity from the
 * touch report.
 *
 * Please be noted that this function will be invoked in ISR so don't
 * issue another touchcomm command here.
 * If really needed, please assign a task to helper thread.
 *
 * @param
 *    [ in]    code:          the code of current touch entity
 *    [ in]    config:        the report configuration stored
 *    [in/out] config_offset: offset of current position in report config,
 *                            and then return the updated position.
 *    [ in]    report:        touch report given
 *    [in/out] report_offset: offset of current position in touch report,
 *                            the updated position should be returned.
 *    [ in]    report_size:   size of given touch report
 *    [ in]    callback_data: pointer to caller data passed to callback function
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_dev_parse_custom_touch_data_cb(const unsigned char code,
		const unsigned char *config, unsigned int *config_offset,
		const unsigned char *report, unsigned int *report_offset,
		unsigned int report_size, void *callback_data)
{
	struct syna_tcm *tcm = (struct syna_tcm *)callback_data;
	struct tcm_touch_data_blob *touch_data;
	struct tcm_objects_data_blob *object_data;
	unsigned int data;
	unsigned int bits;

	touch_data = &tcm->tp_data;
	object_data = touch_data->object_data;

	switch (code) {
	case TOUCH_ENTITY_CUSTOM_ANGLE:
		bits = config[(*config_offset)++];
		syna_tcm_get_touch_data(report, report_size,
				*report_offset, bits, &data);

		object_data[touch_data->obji].custom_data[CUSTOM_DATA_ANGLE] = data;

		*report_offset += bits;
		return bits;
	case TOUCH_ENTITY_CUSTOM_MAJOR:
		bits = config[(*config_offset)++];
		syna_tcm_get_touch_data(report, report_size,
				*report_offset, bits, &data);

		object_data[touch_data->obji].custom_data[CUSTOM_DATA_MAJOR] = data;

		*report_offset += bits;
		return bits;
	case TOUCH_ENTITY_CUSTOM_MINOR:
		bits = config[(*config_offset)++];
		syna_tcm_get_touch_data(report, report_size,
				*report_offset, bits, &data);

		object_data[touch_data->obji].custom_data[CUSTOM_DATA_MINOR] = data;
		*report_offset += bits;
		return bits;
	default:
		LOGW("Unknown touch config code (idx:%d 0x%02x)\n",
			*config_offset, code);
		return (-EINVAL);
	}

	return (-EINVAL);
}
#endif

#if defined(ENABLE_WAKEUP_GESTURE)
/*
 * syna_dev_parse_custom_gesture_cb()
 *
 * Callback to parse the custom or non-standard gesture data from the
 * touch report.
 *
 * Please be noted that this function will be invoked in ISR so don't
 * issue another touchcomm command here.
 * If really needed, please assign a task to helper thread.
 *
 * @param
 *    [ in]    code:          the code of current touch entity
 *    [ in]    config:        the report configuration stored
 *    [in/out] config_offset: offset of current position in report config,
 *                            the updated position should be returned.
 *    [ in]    report:        touch report given
 *    [in/out] report_offset: offset of current position in touch report,
 *                            the updated position should be returned.
 *    [ in]    report_size:   size of given touch report
 *    [ in]    callback_data: pointer to caller data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_dev_parse_custom_gesture_cb(const unsigned char code,
		const unsigned char *config, unsigned int *config_offset,
		const unsigned char *report, unsigned int *report_offset,
		unsigned int report_size, void *callback_data)
{
	struct syna_tcm *tcm = (struct syna_tcm *)callback_data;
	unsigned int data;
	unsigned int bits;
	unsigned int offset = *report_offset;
	struct custom_gesture_data {
		unsigned short x;
		unsigned short y;
		unsigned char major;
		unsigned char minor;
		unsigned char angle;
	} __packed;
	struct custom_gesture_data g_pos = {0};
	int major = 0;
	int minor = 0;
	int angle = 0;

	bits = config[(*config_offset)++];

	if (code == TOUCH_REPORT_GESTURE_ID) {
		syna_tcm_get_touch_data(report, report_size, offset, bits, &data);

		switch (data) {
		case GESTURE_NONE:
			break;
		case GESTURE_SINGLE_TAP:
			LOGD("Gesture single tap detected\n");
			break;
		case GESTURE_LONG_PRESS:
			LOGD("Gesture long press detected\n");
			break;
		default:
			LOGW("Unknown gesture id %d\n", data);
			break;
		}
		tcm->tp_data.gesture_id = data;

		*report_offset += bits;

	} else if (code == TOUCH_REPORT_GESTURE_DATA) {
		if (bits != (sizeof(struct custom_gesture_data) * 8)) {
			LOGE("Invalid size of gesture data %d (expected:%d)\n",
				bits, (int)(sizeof(struct custom_gesture_data) * 8));
			return -EINVAL;
		}

		syna_tcm_get_touch_data(report, report_size, offset, 16, &data);
		g_pos.x = (unsigned short)data;
		offset += 16;

		syna_tcm_get_touch_data(report, report_size, offset, 16, &data);
		g_pos.y = (unsigned short)data;
		offset += 16;

		syna_tcm_get_touch_data(report, report_size, offset, 8, &data);
		g_pos.minor = (unsigned char)data;
		minor = g_pos.minor * tcm->hw_if->pixels_per_mm;
		offset += 8;

		syna_tcm_get_touch_data(report, report_size, offset, 8, &data);
		g_pos.major = (unsigned char)data;
		major = g_pos.major * tcm->hw_if->pixels_per_mm;
		offset += 8;

		syna_tcm_get_touch_data(report, report_size, offset, 8, &data);
		g_pos.angle = (unsigned char) data;
		angle = (int) (((s8) g_pos.angle) * 2048 / 45);
		offset += 8;

		*report_offset += bits;

		if (tcm->tp_data.gesture_id != GESTURE_NONE) {
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
			struct gti_fw_status_data data;
			if (tcm->tp_data.gesture_id == GESTURE_SINGLE_TAP)
				data.gesture_event.type = GTI_GESTURE_STTW;
			else if (tcm->tp_data.gesture_id == GESTURE_LONG_PRESS)
				data.gesture_event.type = GTI_GESTURE_LPTW;
			else
				return bits;

			data.gesture_event.x = g_pos.x;
			data.gesture_event.y = g_pos.y;
			data.gesture_event.major = major;
			data.gesture_event.minor = minor;
			data.gesture_event.angle = angle;
			goog_notify_fw_status_changed(tcm->gti, GTI_FW_STATUS_GESTURE_EVENT, &data);
#endif
			LOGD("Gesture data x:%u y:%u major:%d minor:%d  angle:%d\n",
				g_pos.x, g_pos.y, major, minor, angle);
		}
	} else {
		return -EINVAL;
	}

	return bits;
}
#endif

/*
 * syna_tcm_free_input_events()
 *
 * Clear all relevant touched events.
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    none.
 */
#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
static void syna_dev_free_input_events(struct syna_tcm *tcm)
{
	struct input_dev *input_dev = tcm->input_dev;
#ifdef TYPE_B_PROTOCOL
	unsigned int idx;
#endif

	if (input_dev == NULL)
		return;

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

#ifdef TYPE_B_PROTOCOL
	for (idx = 0; idx < MAX_NUM_OBJECTS; idx++) {
		input_mt_slot(input_dev, idx);
		input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_report_key(input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(input_dev);
#endif
	input_sync(input_dev);

	syna_pal_mutex_unlock(&tcm->tp_event_mutex);
}
#endif

/*
 * syna_dev_report_input_events()
 *
 * Report touched events to the input subsystem.
 *
 * After syna_tcm_get_event_data() function and the touched data is ready,
 * this function can be called to report an input event.
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    none.
 */
static void syna_dev_report_input_events(struct syna_tcm *tcm)
{
	unsigned int idx;
	unsigned int x;
	unsigned int y;
	unsigned int z;
#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
	int major;
	int minor;
	int angle;
#else
	int wx;
	int wy;
#endif

	unsigned int status;
	unsigned int touch_count;
	struct input_dev *input_dev = tcm->input_dev;
	unsigned int max_objects = tcm->tcm_dev->max_objects;
	struct tcm_touch_data_blob __maybe_unused *touch_data;
	struct tcm_objects_data_blob *object_data;

	if (input_dev == NULL)
		return;

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	goog_input_lock(tcm->gti);
#else
	syna_pal_mutex_lock(&tcm->tp_event_mutex);
#endif

	object_data = &tcm->tp_data.object_data[0];

#ifdef ENABLE_WAKEUP_GESTURE
	touch_data = &tcm->tp_data;
	if ((tcm->pwr_state == LOW_PWR) && tcm->irq_wake) {
		if (touch_data->gesture_id) {
			LOGD("Gesture detected, id:%d\n",
				touch_data->gesture_id);
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
			goog_input_report_key(tcm->gti, input_dev, KEY_WAKEUP, 1);
			goog_input_sync(tcm->gti, input_dev);
			goog_input_report_key(tcm->gti, input_dev, KEY_WAKEUP, 0);
			goog_input_sync(tcm->gti, input_dev);
#else
			input_report_key(input_dev, KEY_WAKEUP, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_WAKEUP, 0);
			input_sync(input_dev);
#endif
		}
	}
#endif

	if (tcm->pwr_state == LOW_PWR)
		goto exit;

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	goog_input_set_timestamp(tcm->gti, input_dev, tcm->timestamp);
#endif

	touch_count = 0;

	for (idx = 0; idx < max_objects; idx++) {
		if (tcm->prev_obj_status[idx] == LIFT &&
				object_data[idx].status == LIFT)
			status = NOP;
		else
			status = object_data[idx].status;

		switch (status) {
		case LIFT:
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
			goog_input_mt_slot(tcm->gti, input_dev, idx);
			goog_input_report_abs(tcm->gti, input_dev, ABS_MT_PRESSURE, 0);
			goog_input_mt_report_slot_state(tcm->gti, input_dev,
					MT_TOOL_FINGER, 0);
#else
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(input_dev, idx);
			input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, 0);
#endif
#endif
			break;
		case FINGER:
		case GLOVED_OBJECT:
		case PALM:
			x = object_data[idx].x_pos;
			y = object_data[idx].y_pos;
#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
			major = object_data[idx].custom_data[CUSTOM_DATA_MAJOR];
			minor = object_data[idx].custom_data[CUSTOM_DATA_MINOR];
			angle = object_data[idx].custom_data[CUSTOM_DATA_ANGLE];
			LOGD("Finger %d: major = %d, minor = %d, angle = %d.\n",
				idx, major, minor, (s8) angle);
			/* Report major and minor in display pixels. */
			major = major * tcm->hw_if->pixels_per_mm;
			minor = minor * tcm->hw_if->pixels_per_mm;
#else
			wx = object_data[idx].x_width;
			wy = object_data[idx].y_width;
			/* Report major and minor in display pixels. */
			wx = wx * tcm->hw_if->pixels_per_mm;
			wy = wy * tcm->hw_if->pixels_per_mm;
#endif

			if (object_data[idx].z == 0) {
				z = 1;
				LOGW("Get a touch coordinate with pressure = 0");
			} else {
				z = object_data[idx].z;
			}
#ifdef REPORT_SWAP_XY
			x = x ^ y;
			y = x ^ y;
			x = x ^ y;
#endif
#ifdef REPORT_FLIP_X
			x = tcm->input_dev_params.max_x - x;
#endif
#ifdef REPORT_FLIP_Y
			y = tcm->input_dev_params.max_y - y;
#endif

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
			goog_input_mt_slot(tcm->gti, input_dev, idx);
			goog_input_mt_report_slot_state(tcm->gti, input_dev, MT_TOOL_FINGER, 1);
			goog_input_report_key(tcm->gti, input_dev, BTN_TOUCH, 1);
			goog_input_report_key(tcm->gti, input_dev, BTN_TOOL_FINGER, 1);
			goog_input_report_abs(tcm->gti, input_dev, ABS_MT_POSITION_X, x);
			goog_input_report_abs(tcm->gti, input_dev, ABS_MT_POSITION_Y, y);
			goog_input_report_abs(tcm->gti, input_dev, ABS_MT_PRESSURE, z);
#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
			goog_input_report_abs(tcm->gti, input_dev, ABS_MT_TOUCH_MAJOR, major);
			goog_input_report_abs(tcm->gti, input_dev, ABS_MT_TOUCH_MINOR, minor);
			goog_input_report_abs(tcm->gti, input_dev,
					ABS_MT_ORIENTATION, (s16) (((s8) angle) * 2048 / 45));
#else
			goog_input_report_abs(tcm->gti, input_dev,
					ABS_MT_TOUCH_MAJOR, MAX(wx, wy));
			goog_input_report_abs(tcm->gti, input_dev,
					ABS_MT_TOUCH_MINOR, MIN(wx, wy));
#endif
#else
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(input_dev, idx);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, 1);
#endif
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_report_key(input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(input_dev, ABS_MT_PRESSURE, z);
#ifdef REPORT_TOUCH_WIDTH
#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
			input_report_abs(input_dev,
					ABS_MT_TOUCH_MAJOR, major);
			input_report_abs(input_dev,
					ABS_MT_TOUCH_MINOR, minor);
			input_report_abs(input_dev,
					ABS_MT_ORIENTATION, (s16) (((s8) angle) * 2048 / 45));
#else
			input_report_abs(input_dev,
					ABS_MT_TOUCH_MAJOR, MAX(wx, wy));
			input_report_abs(input_dev,
					ABS_MT_TOUCH_MINOR, MIN(wx, wy));
#endif
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(input_dev);
#endif
#endif

			LOGD("Finger %d: x = %d, y = %d, z = %d\n", idx, x, y, z);
			touch_count++;
			break;
		default:
			break;
		}

		tcm->prev_obj_status[idx] = object_data[idx].status;
	}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	if (touch_count == 0) {
		goog_input_report_key(tcm->gti, input_dev, BTN_TOUCH, 0);
		goog_input_report_key(tcm->gti, input_dev, BTN_TOOL_FINGER, 0);
	}

	goog_input_sync(tcm->gti, input_dev);
#else
	if (touch_count == 0) {
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_key(input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(input_dev);
#endif
	}

	input_set_timestamp(input_dev, tcm->timestamp);
	input_sync(input_dev);
#endif
	tcm->touch_count = touch_count;

exit:
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	goog_input_unlock(tcm->gti);
#else
	syna_pal_mutex_unlock(&tcm->tp_event_mutex);
#endif
}

/*
 * syna_dev_create_input_device()
 *
 * Allocate an input device and set up relevant parameters to the
 * input subsystem.
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_create_input_device(struct syna_tcm *tcm)
{
	int retval = 0;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;
	struct input_dev *input_dev = NULL;
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		return -EINVAL;
	}

	input_dev = devm_input_allocate_device(dev);
#else /* Legacy API */
	input_dev = input_allocate_device();
#endif
	if (input_dev == NULL) {
		LOGE("Fail to allocate input device\n");
		return -ENODEV;
	}

	input_dev->name = TOUCH_INPUT_NAME;
	input_dev->phys = TOUCH_INPUT_PHYS_PATH;
	input_dev->uniq = "google_touchscreen";
	input_dev->id.product = SYNAPTICS_TCM_DRIVER_ID;
	input_dev->id.version = SYNAPTICS_TCM_DRIVER_VERSION;
	input_dev->dev.parent = tcm->pdev->dev.parent;
	input_set_drvdata(input_dev, tcm);

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

#ifdef ENABLE_WAKEUP_GESTURE
	set_bit(KEY_WAKEUP, input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
#endif

	input_set_abs_params(input_dev,
			ABS_MT_POSITION_X, 0, tcm_dev->max_x, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_Y, 0, tcm_dev->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	input_mt_init_slots(input_dev, tcm_dev->max_objects,
			INPUT_MT_DIRECT);

#ifdef REPORT_TOUCH_WIDTH
	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MAJOR, 0, tcm_dev->max_x, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MINOR, 0, tcm_dev->max_y, 0, 0);
#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
	input_set_abs_params(input_dev,
			ABS_MT_ORIENTATION, -4096, 4096, 0, 0);
#endif
#endif

	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, MT_TOOL_FINGER,
			MT_TOOL_PALM, 0, 0);

	tcm->input_dev_params.max_x = tcm_dev->max_x;
	tcm->input_dev_params.max_y = tcm_dev->max_y;
	tcm->input_dev_params.max_objects = tcm_dev->max_objects;

	retval = input_register_device(input_dev);
	if (retval < 0) {
		LOGE("Fail to register input device\n");
		input_free_device(input_dev);
		input_dev = NULL;
		return retval;
	}

	tcm->input_dev = input_dev;

	return 0;
}

/*
 * syna_dev_release_input_device()
 *
 * Release an input device allocated previously.
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    none.
 */
static void syna_dev_release_input_device(struct syna_tcm *tcm)
{
	if (!tcm->input_dev)
		return;

	input_unregister_device(tcm->input_dev);

	tcm->input_dev = NULL;
}

/*
 * syna_dev_check_input_params()
 *
 * Check if any of the input parameters registered to the input subsystem
 * has changed.
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    positive value to indicate mismatching parameters; otherwise, return 0.
 */
static int syna_dev_check_input_params(struct syna_tcm *tcm)
{
	struct tcm_dev *tcm_dev = tcm->tcm_dev;

	if (tcm_dev->max_x == 0 && tcm_dev->max_y == 0)
		return 0;

	if (tcm->input_dev_params.max_x != tcm_dev->max_x)
		return 1;

	if (tcm->input_dev_params.max_y != tcm_dev->max_y)
		return 1;

	if (tcm->input_dev_params.max_objects != tcm_dev->max_objects)
		return 1;

	if (tcm_dev->max_objects > MAX_NUM_OBJECTS) {
		LOGW("Out of max num objects defined, in app_info: %d\n",
			tcm_dev->max_objects);
		return 0;
	}

	LOGN("Input parameters unchanged\n");

	return 0;
}

/*
 * syna_dev_set_up_input_device()
 *
 * Set up input device to the input subsystem by confirming the supported
 * parameters and creating the device.
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_set_up_input_device(struct syna_tcm *tcm)
{
	int retval = 0;

	if (IS_NOT_APP_FW_MODE(tcm->tcm_dev->dev_mode)) {
		LOGN("Application firmware not running, current mode: %02x\n",
			tcm->tcm_dev->dev_mode);
		return 0;
	}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	if (tcm->gti)
		goog_input_lock(tcm->gti);
#else
	syna_dev_free_input_events(tcm);

	syna_pal_mutex_lock(&tcm->tp_event_mutex);
#endif

	retval = syna_dev_check_input_params(tcm);
	if (retval == 0)
		goto exit;

	if (tcm->input_dev != NULL)
		syna_dev_release_input_device(tcm);

	retval = syna_dev_create_input_device(tcm);
	if (retval < 0) {
		LOGE("Fail to create input device\n");
		goto exit;
	}

exit:
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	if (tcm->gti)
		goog_input_unlock(tcm->gti);
#else
	syna_pal_mutex_unlock(&tcm->tp_event_mutex);
#endif

	return retval;
}

static irqreturn_t syna_dev_isr(int irq, void *handle)
{
	struct syna_tcm *tcm = handle;

	tcm->timestamp = ktime_get();

	return IRQ_WAKE_THREAD;
}

/*
 * syna_dev_interrupt_thread()
 *
 * This is the function to be called when the interrupt is asserted.
 * The purposes of this handler is to read events generated by device and
 * retrieve all enqueued messages until ATTN is no longer asserted.
 *
 * @param
 *    [ in] irq:  interrupt line
 *    [ in] data: private data being passed to the handler function
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static irqreturn_t syna_dev_interrupt_thread(int irq, void *data)
{
	int retval;
	unsigned char code = 0;
	struct syna_tcm *tcm = data;
	struct custom_fw_status *status;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;
	unsigned char *touch_data = 0;
	unsigned short touch_data_size = 0;
	unsigned char *heatmap_data_start = 0;
	unsigned char *heatmap_data = 0;
	unsigned short heatmap_data_size = 0;

	if (unlikely(gpio_get_value(attn->irq_gpio) != attn->irq_on_state))
		goto exit;

	tcm->isr_pid = current->pid;

	/* retrieve the original report date generated by firmware */
	retval = syna_tcm_get_event_data(tcm->tcm_dev,
			&code,
			&tcm->event_data);
	if (retval < 0) {
		LOGE("Fail to get event data\n");
		goto exit;
	}

	tcm->is_attn_asserted = true;

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	if (tcm->report_to_queue[code] == EFP_ENABLE) {
		syna_tcm_buf_lock(&tcm->tcm_dev->external_buf);
		syna_cdev_update_report_queue(tcm, code,
		    &tcm->tcm_dev->external_buf);
		syna_tcm_buf_unlock(&tcm->tcm_dev->external_buf);
#ifndef REPORT_CONCURRENTLY
		goto exit;
#endif
	}
#endif
	/* report input event only when receiving a touch report */
	if (code == REPORT_TOUCH) {
		/* parse touch report once received */
		retval = syna_tcm_parse_touch_report(tcm->tcm_dev,
				tcm->event_data.buf,
				tcm->event_data.data_length,
				&tcm->tp_data);
		if (retval < 0) {
			LOGE("Fail to parse touch report\n");
			goto exit;
		}

		/* forward the touch event to system */
		syna_dev_report_input_events(tcm);
	} else if (code == tcm->raw_data_report_code) {
		if (!tcm->raw_data_buffer) {
			tcm->raw_data_buffer = kmalloc(
					       sizeof(u16) * (tcm_dev->rows * tcm_dev->cols +
							      tcm_dev->rows + tcm_dev->cols),
					       GFP_KERNEL);
			if (!tcm->raw_data_buffer) {
				LOGE("Allocate raw_data_buffer failed\n");
				goto exit;
			}
		}
		if (tcm->event_data.data_length == sizeof(u16) * (tcm_dev->rows * tcm_dev->cols +
								  tcm_dev->rows + tcm_dev->cols)) {
			syna_pal_mutex_lock(&tcm->raw_data_mutex);
			memcpy(tcm->raw_data_buffer, tcm->event_data.buf,
			       tcm->event_data.data_length);
			syna_pal_mutex_unlock(&tcm->raw_data_mutex);
			complete_all(&tcm->raw_data_completion);
		} else {
			LOGE("Raw data length: %d is incorrect.\n", tcm->event_data.data_length);
		}
	}

	/* handling the particular report data */
	switch (code) {
	case REPORT_HEAT_MAP:
		/* for 'heat map' ($c3) report,
		 * report data has been stored at tcm->event_data.buf;
		 * while, tcm->event_data.data_length is the size of data
		 */
		LOGD("Heat map data received, size:%d\n",
			tcm->event_data.data_length);
		break;
	case REPORT_TOUCH_AND_HEATMAP:
		/* parse $c5 report containing touch and heatmap data */

		/* touch data */
		touch_data_size = (tcm->event_data.buf[1] << 8) | tcm->event_data.buf[0];
		touch_data = tcm->event_data.buf + 2;

		retval = syna_tcm_parse_touch_report(tcm->tcm_dev,
				touch_data,
				touch_data_size,
				&tcm->tp_data);
		if (retval < 0) {
			LOGE("Fail to parse touch report\n");
			goto exit;
		}
		/* forward the touch event to system */
		syna_dev_report_input_events(tcm);

		/* heatmap data */
		heatmap_data_start = touch_data + touch_data_size;
		heatmap_data_size = (heatmap_data_start[1] << 8) | heatmap_data_start[0];
		heatmap_data = touch_data + touch_data_size + 2;

		syna_parse_heatmap(tcm, heatmap_data, heatmap_data_size);

		LOGD("$c5 Heat map data received, size:%d\n", heatmap_data_size);

		break;
	case REPORT_FW_STATUS:
		/* for 'fw status' ($c2) report,
		 * report size shall be 2-byte only; the
		 */
		status = (struct custom_fw_status *)&tcm->event_data.buf[0];
		LOGI("Status: moisture:%d noise:%d freq-change:%d, grip:%d, palm:%d, "
			"fast relax:%d\n",
			status->b0_moisture, status->b1_noise_state,
			status->b2_freq_hopping, status->b3_grip, status->b4_palm,
			status->b5_fast_relaxation);
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
		syna_notify_fw_status(tcm, status);
#endif

		break;
	default:
		break;
	}

exit:
	return IRQ_HANDLED;
}

/*
 * syna_dev_request_irq()
 *
 * Allocate an interrupt line and register the ISR handler
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_request_irq(struct syna_tcm *tcm)
{
	int retval;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		retval = -EINVAL;
		goto exit;
	}
#endif

	if (attn->irq_gpio < 0) {
		LOGE("Invalid IRQ GPIO\n");
		retval = -EINVAL;
		goto exit;
	}

	attn->irq_id = gpio_to_irq(attn->irq_gpio);

#ifdef DEV_MANAGED_API
	retval = devm_request_threaded_irq(dev,
			attn->irq_id,
			syna_dev_isr,
			syna_dev_interrupt_thread,
			attn->irq_flags,
			PLATFORM_DRIVER_NAME,
			tcm);
#else /* Legacy API */
	retval = request_threaded_irq(attn->irq_id,
			syna_dev_isr,
			syna_dev_interrupt_thread,
			attn->irq_flags,
			PLATFORM_DRIVER_NAME,
			tcm);
#endif
	if (retval < 0) {
		LOGE("Fail to request threaded irq\n");
		goto exit;
	}

	attn->irq_enabled = true;

	LOGI("Interrupt handler registered\n");

exit:
	return retval;
}

/*
 * syna_dev_release_irq()
 *
 * Release an interrupt line allocated previously
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    none.
 */
static void syna_dev_release_irq(struct syna_tcm *tcm)
{
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		return;
	}
#endif

	if (attn->irq_id <= 0)
		return;

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	syna_pal_mutex_lock(&attn->irq_en_mutex);
	disable_irq(attn->irq_id);
	syna_pal_mutex_unlock(&attn->irq_en_mutex);
#else
	if (tcm->hw_if->ops_enable_irq)
		tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
#endif

#ifdef DEV_MANAGED_API
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	if (tcm->gti)
		goog_devm_free_irq(tcm->gti, &tcm->pdev->dev, attn->irq_id);
	else
		devm_free_irq(dev, attn->irq_id, tcm);
#else
	devm_free_irq(dev, attn->irq_id, tcm);
#endif
#else
	free_irq(attn->irq_id, tcm);
#endif

	attn->irq_id = 0;
	attn->irq_enabled = false;

	LOGI("Interrupt handler released\n");
}

/*
 * syna_dev_set_up_app_fw()
 *
 * Implement the essential steps for the initialization including the
 * preparation of app info and the configuration of touch report.
 *
 * This function should be called whenever the device initially powers
 * up, resets, or firmware update.
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_set_up_app_fw(struct syna_tcm *tcm)
{
	int retval = 0;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGN("Application firmware not running, current mode: %02x\n",
			tcm_dev->dev_mode);
		return -EINVAL;
	}

	/* collect app info containing most of sensor information */
	retval = syna_tcm_get_app_info(tcm_dev, &tcm_dev->app_info);
	if (retval < 0) {
		LOGE("Fail to get application info\n");
		return retval;
	}

	/* set up the format of touch report */
#ifdef USE_CUSTOM_TOUCH_REPORT_CONFIG
	retval = syna_tcm_set_touch_report_config(tcm_dev,
			custom_touch_format,
			(unsigned int)sizeof(custom_touch_format));
	if (retval < 0) {
		LOGE("Fail to setup the custom touch report format\n");
		return retval;
	}
#endif
	/* preserve the format of touch report */
	retval = syna_tcm_preserve_touch_report_config(tcm_dev);
	if (retval < 0) {
		LOGE("Fail to preserve touch report config\n");
		return retval;
	}

#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
	/* set up custom touch data parsing method */
	retval = syna_tcm_set_custom_touch_entity_callback(tcm_dev,
			syna_dev_parse_custom_touch_data_cb,
			(void *)tcm);
	if (retval < 0) {
		LOGE("Fail to set up custom touch data parsing method\n");
		return retval;
	}
#endif
#ifdef ENABLE_WAKEUP_GESTURE
	/* set up custom gesture parsing method */
	retval = syna_tcm_set_custom_gesture_callback(tcm_dev,
			syna_dev_parse_custom_gesture_cb,
			(void *)tcm);
	if (retval < 0) {
		LOGE("Fail to set up custom gesture parsing method\n");
		return retval;
	}
#endif
	return 0;
}

#ifdef STARTUP_REFLASH
/*
 * syna_dev_reflash_startup_work()
 *
 * Perform firmware update during system startup.
 * Function is available when the 'STARTUP_REFLASH' configuration
 * is enabled.
 *
 * @param
 *    [ in] work: handle of work structure
 *
 * @return
 *    none.
 */
static void syna_dev_reflash_startup_work(struct work_struct *work)
{
	int retval, i;
	int suffix_fw_name_count = 0;
	struct delayed_work *delayed_work;
	struct syna_tcm *tcm;
	struct tcm_dev *tcm_dev;
	struct device_node *np;
	struct property *prop;
	const struct firmware *fw_entry = NULL;
	const unsigned char *fw_image = NULL;
	const char *suffix_fw_name = NULL;
	unsigned int fw_image_size;

	delayed_work = container_of(work, struct delayed_work, work);
	tcm = container_of(delayed_work, struct syna_tcm, reflash_work);

	tcm_dev = tcm->tcm_dev;
	np = tcm->pdev->dev.parent->of_node;

	pm_stay_awake(&tcm->pdev->dev);

	/* Use CPU mode for the firmware update because it cannot fit the 4 bytes alignment.*/
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_SPI_S3C64XX_GS)
	if (goog_check_spi_dma_enabled(tcm->hw_if->pdev) && tcm->hw_if->s3c64xx_sci) {
		tcm->hw_if->ops_disable_irq_sync(tcm->hw_if);
		tcm->hw_if->dma_mode = 0;
		tcm->hw_if->s3c64xx_sci->dma_mode = CPU_MODE;
		tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
	}
#endif

	prop = of_find_property(np, "synaptics,suffix-fw-name", NULL);
	if (prop && prop->length) {
		suffix_fw_name_count = of_property_count_strings(np, "synaptics,suffix-fw-name");
		for (i = 0; i < suffix_fw_name_count; i++) {
			of_property_read_string_index(np, "synaptics,suffix-fw-name", i,
					&suffix_fw_name);
			if (strncmp(tcm_dev->id_info.part_number, suffix_fw_name,
					strlen(suffix_fw_name)) == 0) {
				strcat(tcm->hw_if->fw_name, "_");
				strcat(tcm->hw_if->fw_name, suffix_fw_name);
				break;
			}
		}
	}
	LOGI("Firmware name %s for %s", tcm->hw_if->fw_name, tcm_dev->id_info.part_number);

	/* get firmware image */
	retval = request_firmware(&fw_entry,
			tcm->hw_if->fw_name,
			tcm->pdev->dev.parent);
	if (retval < 0) {
		LOGE("Fail to request %s\n", tcm->hw_if->fw_name);
		if (tcm->input_dev)
			goto skip_fw_update;
		else
			goto exit;
	}

	fw_image = fw_entry->data;
	fw_image_size = fw_entry->size;

	LOGD("Firmware image size = %d\n", fw_image_size);

	/* perform fw update */
#ifdef MULTICHIP_DUT_REFLASH
	/* do firmware update for the multichip-based device */
	retval = syna_tcm_romboot_do_multichip_reflash(tcm_dev,
			fw_image,
			fw_image_size,
			RESP_IN_ATTN,
			tcm->force_reflash);
#else
	/* do firmware update for the common device */
	retval = syna_tcm_do_fw_update(tcm_dev,
			fw_image,
			fw_image_size,
			FW_UPDATE_DELAY_MS(200, 20),
			tcm->force_reflash);
#endif
	/* Restore DMA mode */
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_SPI_S3C64XX_GS)
	tcm->hw_if->ops_disable_irq_sync(tcm->hw_if);
	if (goog_check_spi_dma_enabled(tcm->hw_if->pdev) && tcm->hw_if->s3c64xx_sci) {
		tcm->hw_if->dma_mode = 1;
		tcm->hw_if->s3c64xx_sci->dma_mode = DMA_MODE;
	}
	/* Wait 300ms to make sure the SPI driver suspends so that it will
	 * acquire the DMA channel when it resumes next time because DMA_MODE
	 * was enabled.
	 */
	msleep(300);
	tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
#endif
	if (retval < 0) {
		LOGE("Fail to do reflash, reflash_count = %d\n", tcm->reflash_count);
		tcm->force_reflash = true;
		if (tcm->reflash_count++ < 3) {
			queue_delayed_work(tcm->reflash_workqueue, &tcm->reflash_work,
				msecs_to_jiffies(STARTUP_REFLASH_DELAY_TIME_MS));
		}
		goto exit;
	}

	/* re-initialize the app fw */
	retval = syna_dev_set_up_app_fw(tcm);
	if (retval < 0) {
		LOGE("Fail to set up app fw after fw update\n");
		goto exit;
	}

	/* ensure the settings of input device
	 * if needed, re-create a new input device
	 */
	retval = syna_dev_set_up_input_device(tcm);
	if (retval < 0) {
		LOGE("Fail to register input device\n");
		goto exit;
	}

skip_fw_update:
	syna_gti_init(tcm);
exit:
	fw_image = NULL;

	if (fw_entry) {
		release_firmware(fw_entry);
		fw_entry = NULL;
	}

	pm_relax(&tcm->pdev->dev);
}
#endif
#if defined(POWER_ALIVE_AT_SUSPEND) && !defined(RESET_ON_RESUME)
/*
 * syna_dev_enter_normal_sensing()
 *
 * Helper to enter normal sensing mode
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_enter_normal_sensing(struct syna_tcm *tcm)
{
	int retval = 0;
#ifdef GOOG_INT2_FEATURE
	unsigned short enable;
#endif

	if (!tcm)
		return -EINVAL;

	/* bring out of sleep mode. */
	retval = syna_tcm_sleep(tcm->tcm_dev, false);
	if (retval < 0) {
		LOGE("Fail to exit deep sleep\n");
		return retval;
	}

	/* disable low power gesture mode, if needed */
	if (tcm->lpwg_enabled) {
		retval = syna_dev_enable_lowpwr_gesture(tcm, false);
		if (retval < 0) {
			LOGE("Fail to disable low power gesture mode\n");
			return retval;
		}
	}

#ifdef GOOG_INT2_FEATURE
	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE,
				&enable,
				RESP_IN_POLLING);
	if (retval < 0) {
		LOGE("Fail to get low power gesture mode\n");
		return retval;
	}

	if (enable) {
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE,
				0,
				RESP_IN_POLLING);
		if (retval < 0) {
			LOGE("Fail to exit low power gesture mode\n");
			return retval;
		}
		LOGI("Exit gesture mode.");
	}

#endif
	return 0;
}
#endif
#ifdef POWER_ALIVE_AT_SUSPEND
/*
 * syna_dev_enter_lowpwr_sensing()
 *
 * Helper to enter power-saved sensing mode, that
 * may be the lower power gesture mode or deep sleep mode.
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_enter_lowpwr_sensing(struct syna_tcm *tcm)
{
	int retval = 0;

	if (!tcm)
		return -EINVAL;

	/* enable low power gesture mode, if needed */
	if (tcm->lpwg_enabled) {
		retval = syna_dev_enable_lowpwr_gesture(tcm, true);
		if (retval < 0) {
			LOGE("Fail to disable low power gesture mode\n");
			return retval;
		}
	} else {
	/* enter sleep mode for non-LPWG cases */
		if (!tcm->slept_in_early_suspend) {
			retval = syna_tcm_sleep(tcm->tcm_dev, true);
			if (retval < 0) {
				LOGE("Fail to enter deep sleep\n");
				return retval;
			}
		}
	}

	return 0;
}
#endif

static int syna_pinctrl_configure(struct syna_tcm *tcm, bool enable)
{
	struct pinctrl_state *state;

	if (IS_ERR_OR_NULL(tcm->pinctrl)) {
		LOGE("Invalid pinctrl!\n");
		return -EINVAL;
	}

	LOGD("%s\n", enable ? "ACTIVE" : "SUSPEND");

	if (enable) {
		state = pinctrl_lookup_state(tcm->pinctrl, "ts_active");
		if (IS_ERR(state))
			LOGE("Could not get ts_active pinstate!\n");
	} else {
		state = pinctrl_lookup_state(tcm->pinctrl, "ts_suspend");
		if (IS_ERR(state))
			LOGE("Could not get ts_suspend pinstate!\n");
	}

	if (!IS_ERR_OR_NULL(state))
		return pinctrl_select_state(tcm->pinctrl, state);

	return 0;
}

/*
 * syna_dev_resume()
 *
 * Resume from the suspend state.
 * If RESET_ON_RESUME is defined, a reset is issued to the touch controller.
 * Otherwise, the touch controller is brought out of sleep mode.
 *
 * @param
 *    [ in] dev: an instance of device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_resume(struct device *dev)
{
	int retval = 0;
	struct syna_tcm *tcm = dev_get_drvdata(dev);
	struct syna_hw_interface *hw_if = tcm->hw_if;
	bool irq_enabled = true;
#if defined(RESET_ON_RESUME) || defined(POWER_ALIVE_AT_SUSPEND)
	unsigned char status;
#endif

	/* exit directly if device isn't in suspend state */
	if (tcm->pwr_state == PWR_ON)
		return 0;

	LOGI("Prepare to resume device\n");

	syna_pinctrl_configure(tcm, true);

#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	/* clear all input events  */
	syna_dev_free_input_events(tcm);
#endif

#ifdef RESET_ON_RESUME
	LOGI("Do reset on resume\n");

	for (retry = 0; retry < 3; retry++) {
		if (hw_if->ops_hw_reset) {
			hw_if->ops_hw_reset(hw_if);
			retval = syna_tcm_get_event_data(tcm->tcm_dev,
				&status, NULL);
			if ((retval < 0) || (status != REPORT_IDENTIFY)) {
				LOGE("Fail to complete hw reset, ret = %d, status = %d\n",
				     retval, status);
				continue;
			}
			break;
		} else {
			retval = syna_tcm_reset(tcm->tcm_dev);
			if (retval < 0) {
				LOGE("Fail to do sw reset, ret = %d\n", retval);
				continue;
			}
			break;
		}
	}
	if (retval < 0 || (hw_if->ops_hw_reset && (status != REPORT_IDENTIFY))) {
		goto exit;
	}
#else
#ifdef POWER_ALIVE_AT_SUSPEND
	/* enter normal power mode */
	retval = syna_dev_enter_normal_sensing(tcm);
	if (retval < 0) {
		LOGE("Fail to enter normal power mode, trigger reset to recover\n");
		tcm->pwr_state = PWR_ON;
		if (hw_if->ops_hw_reset) {
			hw_if->ops_hw_reset(hw_if);
			retval = syna_tcm_get_event_data(tcm->tcm_dev, &status, NULL);
			if ((retval < 0) || (status != REPORT_IDENTIFY)) {
				LOGE("Fail to complete hw reset, ret = %d, status = %d\n",
						retval, status);
			}
		} else {
			retval = syna_tcm_reset(tcm->tcm_dev);
			if (retval < 0)
				LOGE("Fail to do sw reset, ret = %d\n", retval);
		}
		/* The settings will be done by syna_dev_helper_work if reset is triggered. */
		goto exit;
	}
#endif
#ifndef GOOG_INT2_FEATURE
	retval = syna_tcm_rezero(tcm->tcm_dev);
	if (retval < 0) {
		LOGE("Fail to rezero\n");
		goto exit;
	}
#endif
#endif
	tcm->pwr_state = PWR_ON;

	LOGI("Prepare to set up application firmware\n");

	/* set up app firmware */
	retval = syna_dev_set_up_app_fw(tcm);
	if (retval < 0) {
		LOGE("Fail to set up app firmware on resume\n");
		goto exit;
	}

	syna_dev_restore_feature_setting(tcm, RESP_IN_POLLING);

	retval = 0;

	LOGI("Device resumed (pwr_state:%d)\n", tcm->pwr_state);
exit:
	/* set irq back to active mode if not enabled yet */
	irq_enabled = (!hw_if->bdata_attn.irq_enabled);

	/* enable irq */
	if (irq_enabled && (hw_if->ops_enable_irq))
		hw_if->ops_enable_irq(hw_if, true);

	tcm->slept_in_early_suspend = false;

	return retval;
}

/*
 * syna_dev_suspend()
 *
 * Put device into suspend state.
 * Enter either the lower power gesture mode or sleep mode.
 *
 * @param
 *    [ in] dev: an instance of device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_suspend(struct device *dev)
{
#ifdef POWER_ALIVE_AT_SUSPEND
	int retval, retry;
#endif
	struct syna_tcm *tcm = dev_get_drvdata(dev);
	struct syna_hw_interface *hw_if = tcm->hw_if;
	bool irq_disabled = true;
	unsigned char status;

	/* exit directly if device is already in suspend state */
	if (tcm->pwr_state != PWR_ON)
		return 0;

#ifdef POWER_ALIVE_AT_SUSPEND
	tcm->pwr_state = LOW_PWR;
#endif

	LOGI("Prepare to suspend device\n");

#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	/* clear all input events  */
	syna_dev_free_input_events(tcm);
#endif
	/* once lpwg is enabled, irq should be alive.
	 * otherwise, disable irq in suspend.
	 */
	irq_disabled = (!tcm->lpwg_enabled);

	/* disable irq */
	if (irq_disabled && (hw_if->ops_enable_irq))
		hw_if->ops_enable_irq(hw_if, false);

#ifdef POWER_ALIVE_AT_SUSPEND
#ifdef GOOG_INT2_FEATURE
	LOGI("Do reset on suspend\n");

	for (retry = 0; retry < 3; retry++) {
		if (hw_if->ops_hw_reset) {
			hw_if->ops_hw_reset(hw_if);
			retval = syna_tcm_get_event_data(tcm->tcm_dev,
				&status, NULL);
			if ((retval < 0) || (status != REPORT_IDENTIFY)) {
				LOGE("Fail to complete hw reset, ret = %d, status = %d\n",
				     retval, status);
				continue;
			}
			break;
		} else {
			retval = syna_tcm_reset(tcm->tcm_dev);
			if (retval < 0) {
				LOGE("Fail to do sw reset, ret = %d\n", retval);
				continue;
			}
			break;
		}
	}
#endif

	/* enter power saved mode if power is not off */
	retval = syna_dev_enter_lowpwr_sensing(tcm);
	if (retval < 0) {
		LOGE("Fail to enter suspended power mode, reset and retry.\n");
		if (hw_if->ops_hw_reset) {
			hw_if->ops_hw_reset(hw_if);
			retval = syna_tcm_get_event_data(tcm->tcm_dev,
				&status, NULL);
			if ((retval < 0) || (status != REPORT_IDENTIFY)) {
				LOGE("Fail to complete hw reset, ret = %d, status = %d\n",
				     retval, status);
			}
		}
		retval = syna_dev_enter_lowpwr_sensing(tcm);
		if (retval < 0)
			LOGE("Fail to enter suspended power mode after reset.\n");
	}
#else
	tcm->pwr_state = PWR_OFF;
#endif

	syna_pinctrl_configure(tcm, false);

	LOGI("Device suspended (pwr_state:%d)\n", tcm->pwr_state);

	return 0;
}

#if defined(ENABLE_DISP_NOTIFIER)
/*
 * syna_dev_early_suspend()
 *
 * If having early suspend support, enter the sleep mode for
 * non-lpwg cases.
 *
 * @param
 *    [ in] dev: an instance of device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_early_suspend(struct device *dev)
{
	int retval;
	struct syna_tcm *tcm = dev_get_drvdata(dev);

	/* exit directly if device is already in suspend state */
	if (tcm->pwr_state != PWR_ON)
		return 0;

	if (!tcm->lpwg_enabled) {
		retval = syna_tcm_sleep(tcm->tcm_dev, true);
		if (retval < 0) {
			LOGE("Fail to enter deep sleep\n");
			return retval;
		}
	}

	tcm->slept_in_early_suspend = true;

	return 0;
}
/*
 * syna_dev_fb_notifier_cb()
 *
 * Listen the display screen on/off event and perform the corresponding
 * actions.
 *
 * @param
 *    [ in] nb:     instance of notifier_block
 *    [ in] action: fb action
 *    [ in] data:   fb event data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_fb_notifier_cb(struct notifier_block *nb,
		unsigned long action, void *data)
{
	int retval;
	int transition;
#if defined(USE_DRM_PANEL_NOTIFIER)
	struct drm_panel_notifier *evdata = data;
#else
	struct fb_event *evdata = data;
#endif
	struct syna_tcm *tcm = container_of(nb, struct syna_tcm, fb_notifier);
	int time = 0;
	int disp_blank_powerdown;
	int disp_early_event_blank;
	int disp_blank;
	int disp_blank_unblank;

	if (!evdata || !evdata->data || !tcm)
		return 0;

	retval = 0;

#if defined(USE_DRM_PANEL_NOTIFIER)
	disp_blank_powerdown = DRM_PANEL_BLANK_POWERDOWN;
	disp_early_event_blank = DRM_PANEL_EARLY_EVENT_BLANK;
	disp_blank = DRM_PANEL_EVENT_BLANK;
	disp_blank_unblank = DRM_PANEL_BLANK_UNBLANK;
#else
	disp_blank_powerdown = FB_BLANK_POWERDOWN;
	disp_early_event_blank = FB_EARLY_EVENT_BLANK;
	disp_blank = FB_EVENT_BLANK;
	disp_blank_unblank = FB_BLANK_UNBLANK;
#endif

	transition = *(int *)evdata->data;

	/* confirm the firmware flashing is completed before screen off */
	if (transition == disp_blank_powerdown) {
		while (ATOMIC_GET(tcm->tcm_dev->firmware_flashing)) {
			syna_pal_sleep_ms(500);

			time += 500;
			if (time >= 5000) {
				LOGE("Timed out waiting for re-flashing\n");
				ATOMIC_SET(tcm->tcm_dev->firmware_flashing, 0);
				return -ETIMEDOUT;
			}
		}
	}

	if (action == disp_early_event_blank &&
		transition == disp_blank_powerdown) {
		retval = syna_dev_early_suspend(&tcm->pdev->dev);
	} else if (action == disp_blank) {
		if (transition == disp_blank_powerdown) {
			retval = syna_dev_suspend(&tcm->pdev->dev);
			tcm->fb_ready = 0;
		} else if (transition == disp_blank_unblank) {
#ifndef RESUME_EARLY_UNBLANK
			retval = syna_dev_resume(&tcm->pdev->dev);
			tcm->fb_ready++;
#endif
		} else if (action == disp_early_event_blank &&
			transition == disp_blank_unblank) {
#ifdef RESUME_EARLY_UNBLANK
			retval = syna_dev_resume(&tcm->pdev->dev);
			tcm->fb_ready++;
#endif
		}
	}

	return 0;
}
#endif

/*
 * syna_dev_disconnect()
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
static int syna_dev_disconnect(struct syna_tcm *tcm)
{
	struct syna_hw_interface *hw_if = tcm->hw_if;

	if (tcm->is_connected == false) {
		LOGI("%s already disconnected\n", PLATFORM_DRIVER_NAME);
		return 0;
	}

	if (tcm->pwr_state == BARE_MODE) {
		LOGI("Disconnect from bare mode\n");
		goto exit;
	}

#ifdef STARTUP_REFLASH
	if (tcm->reflash_workqueue) {
		cancel_delayed_work_sync(&tcm->reflash_work);
		flush_workqueue(tcm->reflash_workqueue);
		destroy_workqueue(tcm->reflash_workqueue);
		tcm->reflash_workqueue = NULL;
	}
#endif

	/* free interrupt line */
	if (hw_if->bdata_attn.irq_id)
		syna_dev_release_irq(tcm);

	/* unregister input device */
	syna_dev_release_input_device(tcm);

	tcm->input_dev_params.max_x = 0;
	tcm->input_dev_params.max_y = 0;
	tcm->input_dev_params.max_objects = 0;

exit:
#ifdef POWER_SEQUENCE_ON_CONNECT
	/* power off */
	if (hw_if->ops_power_on)
		hw_if->ops_power_on(hw_if, false);
#endif
	tcm->pwr_state = PWR_OFF;
	tcm->is_connected = false;

	LOGI("Device %s disconnected\n", PLATFORM_DRIVER_NAME);

	return 0;
}

/*
 * syna_dev_connect()
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
static int syna_dev_connect(struct syna_tcm *tcm)
{
	int retval;
	struct syna_hw_interface *hw_if = tcm->hw_if;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;

	if (!tcm_dev) {
		LOGE("Invalid tcm_dev\n");
		return -EINVAL;
	}

	if (tcm->is_connected) {
		LOGI("Device %s already connected\n", PLATFORM_DRIVER_NAME);
		return 0;
	}
#ifdef POWER_SEQUENCE_ON_CONNECT
	/* power on the connected device */
	if (hw_if->ops_power_on) {
		retval = hw_if->ops_power_on(hw_if, true);
		if (retval < 0)
			return -ENODEV;
	}
#endif
#ifdef RESET_ON_CONNECT
	/* perform a hardware reset */
	if (hw_if->ops_hw_reset)
		hw_if->ops_hw_reset(hw_if);
#endif
	/* detect which modes of touch controller is running
	 *
	 * this function will handle the startup packet once
	 * powering on the ASIC
	 */
	retval = syna_tcm_detect_device(tcm->tcm_dev, 0, true);
	if (retval < 0) {
		LOGE("Fail to detect the device\n");
		goto err_detect_dev;
	}
	/* 'Bare' mode is a special software mode to bypass all
	 * driver control for the specific user scenario
	 */
	if (tcm->pwr_state == BARE_MODE) {
		LOGI("Device %s config into bare mode\n", PLATFORM_DRIVER_NAME);
		tcm->is_connected = true;
		return 0;
	}

#ifdef FORCE_CONNECTION
	goto request_irq;
#endif

	switch (retval) {
	case MODE_APPLICATION_FIRMWARE:
		retval = syna_dev_set_up_app_fw(tcm);
		if (retval < 0) {
			LOGE("Fail to set up application firmware\n");

			/* switch to bootloader mode when failed */
			LOGI("Switch device to bootloader mode instead\n");
			syna_tcm_switch_fw_mode(tcm_dev,
					MODE_BOOTLOADER,
					FW_MODE_SWITCH_DELAY_MS);
		} else {
			/* allocate and register to input device subsystem */
			retval = syna_dev_set_up_input_device(tcm);
			if (retval < 0) {
				LOGE("Fail to set up input device\n");
				goto err_setup_input_dev;
			}
		}

		break;
	default:
		LOGN("Application firmware not running, current mode: %02x\n",
			retval);
		break;
	}

#ifdef FORCE_CONNECTION
request_irq:
#endif

	LOGI("TCM packrat: %d\n", tcm->tcm_dev->packrat_number);
	LOGI("Config: lpwg mode(%s), custom tp config(%s) helper work(%s)\n",
		(tcm->lpwg_enabled) ? "yes" : "no",
		(tcm->has_custom_tp_config) ? "yes" : "no",
		(tcm->helper_enabled) ? "yes" : "no");
	LOGI("Config: startup reflash(%s), hw reset(%s), rst on resume(%s)\n",
		(tcm->startup_reflash_enabled) ? "yes" : "no",
		(hw_if->ops_hw_reset) ? "yes" : "no",
		(tcm->rst_on_resume_enabled) ? "yes" : "no");
	LOGI("Config: max. write size(%d), max. read size(%d), irq ctrl(%s)\n",
		tcm_dev->max_wr_size, tcm_dev->max_rd_size,
		(hw_if->ops_enable_irq) ? "yes" : "no");

	LOGI("Device %s connected\n", PLATFORM_DRIVER_NAME);

	tcm->pwr_state = PWR_ON;
	tcm->is_connected = true;

	return 0;

err_setup_input_dev:
err_detect_dev:
#ifdef POWER_SEQUENCE_ON_CONNECT
	if (hw_if->ops_power_on)
		hw_if->ops_power_on(hw_if, false);
#endif
	return retval;
}

#ifdef USE_DRM_PANEL_NOTIFIER
static struct drm_panel *syna_dev_get_panel(struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return NULL;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			LOGI("Find available panel\n");
			return panel;
		}
	}

	return NULL;
}
#endif

/*
 * syna_dev_probe()
 *
 * Install the TouchComm device driver
 *
 * @param
 *    [ in] pdev: an instance of platform device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_probe(struct platform_device *pdev)
{
	int retval;
	struct syna_tcm *tcm = NULL;
	struct tcm_dev *tcm_dev = NULL;
	struct syna_hw_interface *hw_if = NULL;
#if defined(USE_DRM_PANEL_NOTIFIER)
	struct device *dev;
#endif

	hw_if = pdev->dev.platform_data;
	if (!hw_if) {
		LOGE("Fail to find hardware configuration\n");
		return -EINVAL;
	}

	tcm = syna_pal_mem_alloc(1, sizeof(struct syna_tcm));
	if (!tcm) {
		LOGE("Fail to create the instance of syna_tcm\n");
		return -ENOMEM;
	}

	tcm->pinctrl = devm_pinctrl_get(pdev->dev.parent);
	if (IS_ERR_OR_NULL(tcm->pinctrl)) {
		LOGE("Could not get pinctrl!\n");
	} else {
		syna_pinctrl_configure(tcm, true);
	}

	/* allocate the TouchCom device handle
	 * recommend to set polling mode here because isr is not registered yet
	 */
	retval = syna_tcm_allocate_device(&tcm_dev, hw_if, RESP_IN_POLLING);
	if ((retval < 0) || (!tcm_dev)) {
		LOGE("Fail to allocate TouchCom device handle\n");
		goto err_allocate_cdev;
	}

	tcm->tcm_dev = tcm_dev;
	tcm->pdev = pdev;
	tcm->hw_if = hw_if;

	syna_tcm_buf_init(&tcm->event_data);

#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	syna_pal_mutex_alloc(&tcm->tp_event_mutex);
#endif
	syna_pal_mutex_alloc(&tcm->raw_data_mutex);

#ifdef USE_CUSTOM_TOUCH_REPORT_CONFIG
	tcm->has_custom_tp_config = true;
#else
	tcm->has_custom_tp_config = false;
#endif
#ifdef STARTUP_REFLASH
	tcm->startup_reflash_enabled = true;
#else
	tcm->startup_reflash_enabled = false;
#endif
#ifdef RESET_ON_RESUME
	tcm->rst_on_resume_enabled = true;
#else
	tcm->rst_on_resume_enabled = false;
#endif
#ifdef ENABLE_HELPER
	tcm->helper_enabled = true;
#else
	tcm->helper_enabled = false;
#endif
#ifdef ENABLE_WAKEUP_GESTURE
	tcm->lpwg_enabled = false;
#else
	tcm->lpwg_enabled = false;
#endif
	tcm->irq_wake = false;

	tcm->is_connected = false;
	tcm->pwr_state = PWR_OFF;

	tcm->dev_connect = syna_dev_connect;
	tcm->dev_disconnect = syna_dev_disconnect;
	tcm->dev_set_up_app_fw = syna_dev_set_up_app_fw;
	tcm->dev_resume = syna_dev_resume;
	tcm->dev_suspend = syna_dev_suspend;

	tcm->userspace_app_info = NULL;

	platform_set_drvdata(pdev, tcm);

	device_init_wakeup(&pdev->dev, 1);

	tcm->event_wq = alloc_workqueue("syna_wq", WQ_UNBOUND |
					WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);

	if (!tcm->event_wq) {
		LOGE("Cannot create work thread\n");
		retval = -ENOMEM;
		goto err_alloc_workqueue;
	}

#if defined(TCM_CONNECT_IN_PROBE)
	/* connect to target device */
	retval = tcm->dev_connect(tcm);
	if (retval < 0) {
#ifdef FORCE_CONNECTION
		LOGW("Device detection is failed somehow\n");
		LOGW("Install driver anyway due to force connect\n");
#else
		LOGE("Fail to connect to the device\n");
		retval = -EPROBE_DEFER;
#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
		syna_pal_mutex_free(&tcm->tp_event_mutex);
#endif
		goto err_connect;
#endif
	}
#endif

	tcm->raw_data_report_code = 0;
	init_completion(&tcm->raw_data_completion);
	complete_all(&tcm->raw_data_completion);

	tcm->enable_fw_grip = 0x02;
	tcm->enable_fw_palm = 0x02;

#ifdef HAS_SYSFS_INTERFACE
	/* create the device file and register to char device classes */
	retval = syna_cdev_create(tcm, pdev);
	if (retval < 0) {
		LOGE("Fail to create the device sysfs\n");
#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
		syna_pal_mutex_free(&tcm->tp_event_mutex);
#endif
		goto err_create_cdev;
	}
#endif

#if defined(ENABLE_DISP_NOTIFIER)
#if defined(USE_DRM_PANEL_NOTIFIER)
	dev = syna_request_managed_device();
	active_panel = syna_dev_get_panel(dev->of_node);
	if (active_panel) {
		tcm->fb_notifier.notifier_call = syna_dev_fb_notifier_cb;
		retval = drm_panel_notifier_register(active_panel,
				&tcm->fb_notifier);
		if (retval < 0) {
			LOGE("Fail to register FB notifier client\n");
			goto err_create_cdev;
		}
	} else {
		LOGE("No available drm panel\n");
	}
#else
	tcm->fb_notifier.notifier_call = syna_dev_fb_notifier_cb;
	retval = fb_register_client(&tcm->fb_notifier);
	if (retval < 0) {
		LOGE("Fail to register FB notifier client\n");
		goto err_create_cdev;
	}
#endif
#endif

#if defined(ENABLE_HELPER)
	ATOMIC_SET(tcm->helper.task, HELP_NONE);
	INIT_WORK(&tcm->helper.work, syna_dev_helper_work);
	/* set up custom touch data parsing method */
	syna_tcm_set_reset_occurrence_callback(tcm_dev,
			syna_dev_reset_detected_cb,
			(void *)tcm);
#endif

	retval = syna_dev_request_irq(tcm);
	if (retval < 0) {
		LOGE("Fail to request the interrupt line\n");
		goto err_request_irq;
	}

	/* for the reference,
	 * create a delayed work to perform fw update during the startup time
	 */
#ifdef STARTUP_REFLASH
	tcm->force_reflash = false;
	tcm->reflash_count = 0;
	tcm->reflash_workqueue =
			create_singlethread_workqueue("syna_reflash");
	INIT_DELAYED_WORK(&tcm->reflash_work, syna_dev_reflash_startup_work);
	queue_delayed_work(tcm->reflash_workqueue, &tcm->reflash_work,
			msecs_to_jiffies(STARTUP_REFLASH_DELAY_TIME_MS));
#endif

	LOGI("%s: TouchComm driver, %s ver.: %d.%s, installed\n",
		__func__,
		PLATFORM_DRIVER_NAME,
		SYNAPTICS_TCM_DRIVER_VERSION,
		SYNAPTICS_TCM_DRIVER_SUBVER);

	return 0;

err_request_irq:
#ifdef HAS_SYSFS_INTERFACE
err_create_cdev:
	syna_tcm_remove_device(tcm->tcm_dev);
#endif

#if defined(TCM_CONNECT_IN_PROBE)
	tcm->dev_disconnect(tcm);
err_connect:
#endif

	if (tcm->event_wq)
		destroy_workqueue(tcm->event_wq);
err_alloc_workqueue:
	syna_tcm_buf_release(&tcm->event_data);
#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	syna_pal_mutex_free(&tcm->tp_event_mutex);
#endif
err_allocate_cdev:
	syna_pal_mem_free((void *)tcm);

	return retval;
}

/*
 * syna_dev_remove()
 *
 * Release all allocated resources and remove the TouchCom device handle
 *
 * @param
 *    [ in] pdev: an instance of platform device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_dev_remove(struct platform_device *pdev)
{
	struct syna_tcm *tcm = platform_get_drvdata(pdev);

	if (!tcm) {
		LOGW("Invalid handle to remove\n");
		return 0;
	}
#if defined(ENABLE_HELPER)
	cancel_work_sync(&tcm->helper.work);
#endif

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	goog_pm_unregister_notification(tcm->gti);

	cancel_work_sync(&tcm->set_grip_mode_work);
	cancel_work_sync(&tcm->set_palm_mode_work);
	cancel_work_sync(&tcm->set_heatmap_enabled_work);
	cancel_work_sync(&tcm->set_screen_protector_mode_work);
	cancel_work_sync(&tcm->set_continuous_report_work);
#endif

#if defined(ENABLE_DISP_NOTIFIER)
#if defined(USE_DRM_PANEL_NOTIFIER)
	if (active_panel)
		drm_panel_notifier_unregister(active_panel,
				&tcm->fb_notifier);
#else
	fb_unregister_client(&tcm->fb_notifier);
#endif
#endif

#ifdef HAS_SYSFS_INTERFACE
	/* remove the cdev and sysfs nodes */
	syna_cdev_remove(tcm);
#endif

	/* check the connection status, and do disconnection */
	if (tcm->dev_disconnect(tcm) < 0)
		LOGE("Fail to do device disconnection\n");

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	goog_touch_interface_remove(tcm->gti);
	tcm->gti = NULL;
#endif

	if (tcm->userspace_app_info != NULL)
		syna_pal_mem_free(tcm->userspace_app_info);

	if (tcm->raw_data_buffer) {
		kfree(tcm->raw_data_buffer);
		tcm->raw_data_buffer = NULL;
	}

	syna_tcm_buf_release(&tcm->event_data);

#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	syna_pal_mutex_free(&tcm->tp_event_mutex);
#endif

	/* remove the allocated tcm device */
	syna_tcm_remove_device(tcm->tcm_dev);

	/* release the device context */
	syna_pal_mem_free((void *)tcm);

	return 0;
}

/*
 * Declare a TouchComm platform device
 */
#if IS_ENABLED(CONFIG_PM) || IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
static const struct dev_pm_ops syna_dev_pm_ops = {
#if !defined(ENABLE_DISP_NOTIFIER) || IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	.suspend = syna_dev_suspend,
	.resume = syna_dev_resume,
#endif
};
#endif

static struct platform_driver syna_dev_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_PM) && !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
		.pm = &syna_dev_pm_ops,
#endif
	},
	.probe = syna_dev_probe,
	.remove = syna_dev_remove,
};


/*
 * syna_dev_module_init()
 *
 * The entry function of the reference driver, which initialize the
 * lower-level bus and register a platform driver.
 *
 * @param
 *    void.
 *
 * @return
 *    0 if the driver registered and bound to a device,
 *    else returns a negative error code and with the driver not registered.
 */
static int __init syna_dev_module_init(void)
{
	int retval;

	retval = syna_hw_interface_init();
	if (retval < 0)
		return retval;

	return platform_driver_register(&syna_dev_driver);
}

/*
 * syna_dev_module_exit()
 *
 * Function is called when un-installing the driver.
 * Remove the registered platform driver and the associated bus driver.
 *
 * @param
 *    void.
 *
 * @return
 *    none.
 */
static void __exit syna_dev_module_exit(void)
{
	platform_driver_unregister(&syna_dev_driver);

	syna_hw_interface_exit();
}

module_init(syna_dev_module_init);
module_exit(syna_dev_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Touch Driver");
MODULE_LICENSE("GPL v2");

