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

/**
 * @file syna_tcm2.c
 *
 * This file implements the Synaptics device driver running under Linux kernel
 * input device subsystem, and also communicate with Synaptics touch controller
 * through TouchComm command-response protocol.
 */

#include "syna_tcm2.h"
#include "syna_tcm2_platform.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_touch.h"
#ifdef STARTUP_REFLASH
#include "synaptics_touchcom_func_reflash.h"
#endif
#ifdef MULTICHIP_DUT_REFLASH
#include "synaptics_touchcom_func_romboot.h"
#endif
#if defined(USE_DRM_BRIDGE)
#include <samsung/exynos_drm_connector.h>
#include <samsung/panel/panel-samsung-drv.h>
#endif

/* Init the kfifo for health check. */
#define SYNA_HC_KFIFO_LEN 4 /* Must be power of 2. */
DEFINE_KFIFO(hc_fifo, struct syna_health_check_fifo, SYNA_HC_KFIFO_LEN);

/* Init the kfifo for touch event information. */
#define SYNA_DEBUG_KFIFO_LEN 4 /* must be power of 2. */
DEFINE_KFIFO(debug_fifo, struct syna_touch_info_fifo, SYNA_DEBUG_KFIFO_LEN);

/**
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

/**
 * @section: RESET_ON_RESUME_DELAY_MS
 *           The delayed time to issue a reset on resume state.
 *           This configuration depends on RESET_ON_RESUME.
 */
#ifdef RESET_ON_RESUME
#define RESET_ON_RESUME_DELAY_MS (100)
#endif


/**
 * @section: POWER_ALIVE_AT_SUSPEND
 *           indicate that the power is still alive even at
 *           system suspend.
 *           otherwise, there is no power supplied when system
 *           is going to suspend stage.
 */
#define POWER_ALIVE_AT_SUSPEND

/**
 * @section: global variables for an active drm panel
 *           in order to register display notifier
 */
#ifdef USE_DRM_PANEL_NOTIFIER
	struct drm_panel *active_panel;
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
static void reserve_then_queue_frame(struct syna_tcm *tcm, bool has_heatmap);
#endif

#ifdef SYNA_HC_KFIFO_LEN
inline void syna_hc_update_and_push(struct syna_tcm *tcm, struct syna_health_check_fifo *hc)
{
	hc->int_ktime = tcm->isr_timestamp;
	hc->int_idx = tcm->syna_hc.int_cnt;
	hc->coord_idx = tcm->syna_hc.coord_event_cnt;
	hc->status_idx = tcm->syna_hc.status_event_cnt;
	hc->active_bit = tcm->syna_hc.touch_idx_state;

	/*
	 * Use kfifo as circular buffer by skipping one element
	 * when fifo is full.
	 */
	if (kfifo_is_full(&hc_fifo))
		kfifo_skip(&hc_fifo);
	kfifo_in(&hc_fifo, hc, 1);
}

inline void syna_hc_dump(struct syna_tcm *tcm)
{
	int i;
	u64 count;
	s64 delta;
	s64 sec_delta;
	u32 ms_delta;
	ktime_t current_time = ktime_get();
	struct syna_health_check_fifo last_hc_fifo[SYNA_HC_KFIFO_LEN] = { 0 };

	count = min(tcm->syna_hc.int_cnt, (u64) ARRAY_SIZE(last_hc_fifo));
	if (kfifo_out_peek(&hc_fifo, last_hc_fifo, count) != count) {
		LOGE("Fail to peak health check data.\n");
		return;
	}

	for (i = 0 ; i < count ; i++) {
		sec_delta = 0;
		ms_delta = 0;
		delta = ktime_ms_delta(current_time, last_hc_fifo[i].int_ktime);
		if (delta > 0)
			sec_delta = div_u64_rem(delta, MSEC_PER_SEC, &ms_delta);

		LOGI("dump-int: #%llu(%llu.%u): S#%llu%s C#%llu(0x%lx)%s.\n",
			last_hc_fifo[i].int_idx,
			sec_delta, ms_delta,
			last_hc_fifo[i].status_idx,
			(last_hc_fifo[i].status_updated) ? "(+)" : "   ",
			last_hc_fifo[i].coord_idx,
			last_hc_fifo[i].active_bit,
			(last_hc_fifo[i].coord_updated) ? "(+)" : ""
			);
	}
}
#else
#define syna_hc_update_and_push(tcm, hc) do {} while (0)
#define syna_hc_dump(tcm) do {} while (0)
#endif /* #ifdef SEC_TS_HC_KFIFO_LEN */

#ifdef SYNA_DEBUG_KFIFO_LEN
inline void syna_kfifo_push_coord(struct syna_tcm *tcm, u8 slot)
{
	if (slot < MAX_NUM_OBJECTS) {
		/*
		 * Use kfifo as circular buffer by skipping one element
		 * when fifo is full.
		 */
		if (kfifo_is_full(&debug_fifo))
			kfifo_skip(&debug_fifo);
		kfifo_in(&debug_fifo, &tcm->syna_hc.touch_info_fifo[slot], 1);
	}
}

inline void syna_debug_dump(struct syna_tcm *tcm)
{
	int i;
	u32 count;
	s64 delta;
	s64 sec_longest_duration;
	u32 ms_longest_duration;
	s64 sec_delta_down;
	u32 ms_delta_down;
	s64 sec_delta_duration;
	u32 ms_delta_duration;
	s32 px_delta_x, px_delta_y;
	ktime_t current_time = ktime_get();
	struct syna_touch_info_fifo last_touch_info_fifo[SYNA_DEBUG_KFIFO_LEN] = { 0 };

	sec_longest_duration = div_u64_rem(tcm->syna_hc.longest_duration,
				MSEC_PER_SEC, &ms_longest_duration);

	count = min(tcm->syna_hc.pressed_cnt, (u32) ARRAY_SIZE(last_touch_info_fifo));
	if (kfifo_out_peek(&debug_fifo, last_touch_info_fifo, count) != count) {
		LOGE("Fail to peak touch event data.\n");
		return;
	}

	for (i = 0 ; i < count; i++) {
		sec_delta_down = -1;
		ms_delta_down = 0;
		/* calculate the delta of finger down from current time. */
		delta = ktime_ms_delta(current_time, last_touch_info_fifo[i].ktime_pressed);
		if (delta > 0)
			sec_delta_down = div_u64_rem(delta, MSEC_PER_SEC, &ms_delta_down);

		/* calculate the delta of finger duration between finger up and down. */
		sec_delta_duration = -1;
		ms_delta_duration = 0;
		px_delta_x = 0;
		px_delta_y = 0;
		delta = ktime_ms_delta(last_touch_info_fifo[i].ktime_released,
				last_touch_info_fifo[i].ktime_pressed);
		if (delta > 0) {
			sec_delta_duration = div_u64_rem(delta, MSEC_PER_SEC, &ms_delta_duration);
			px_delta_x = last_touch_info_fifo[i].x - last_touch_info_fifo[i].x_pressed;
			px_delta_y = last_touch_info_fifo[i].y - last_touch_info_fifo[i].y_pressed;
		}
		LOGI("dump: #%u: %lld.%u(%lld.%u) D(%d, %d).\n",
			last_touch_info_fifo[i].idx,
			sec_delta_down, ms_delta_down,
			sec_delta_duration, ms_delta_duration,
			px_delta_x, px_delta_y);
	}
	LOGI("reset %u, longest %lld.%u.\n",
		tcm->syna_hc.reset_cnt, sec_longest_duration, ms_longest_duration);
	LOGI("dump: cnt %u, wet %u, palm %u.\n",
		tcm->syna_hc.pressed_cnt, tcm->syna_hc.wet_cnt, tcm->syna_hc.palm_cnt);
}
#else
#define syna_kfifo_push_coord(tcm, slot) do {} while (0)
#define syna_debug_dump(tcm) do {} while (0)
#endif /* #ifdef SEC_TS_DEBUG_KFIFO_LEN */

/**
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

/**
 * syna_dev_set_heatmap_mode()
 *
 * Enable or disable the low power gesture mode.
 * Furthermore, set up the wake-up irq.
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *    [ in] en:  '1' to enable heatmap mode; '0' to disable.
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static void syna_dev_set_heatmap_mode(struct syna_tcm *tcm, bool en)
{
	int retval = 0;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
	uint8_t resp_code;
	uint8_t heatmap[1] = {REPORT_HEAT_MAP};
	uint8_t command = en ? CMD_ENABLE_REPORT : CMD_DISABLE_REPORT;
	uint32_t delay = attn->irq_enabled ?
			 RESP_IN_ATTN : tcm->tcm_dev->msg_data.default_resp_reading;

	retval = tcm->tcm_dev->write_message(tcm->tcm_dev,
			command,
			heatmap,
			1,
			&resp_code,
			delay);
	if (retval < 0) {
		LOGE("Fail to %s heatmap\n", en ? "enable" : "disable");
	}
}

/**
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
	syna_dev_set_heatmap_mode(tcm, true);

	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_PALM_REJECTION,
			(tcm->enable_fw_palm & 0x01),
			delay_ms_resp);

	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_GRIP_SUPPRESSION,
			(tcm->enable_fw_grip & 0x01),
			delay_ms_resp);

	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_COMPRESSION_THRESHOLD,
			tcm->hw_if->compression_threhsold,
			delay_ms_resp);

	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_GRIP_DELTA_THRESHOLD,
			tcm->hw_if->grip_delta_threshold,
			delay_ms_resp);

	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_GRIP_BORDER_THRESHOLD,
			tcm->hw_if->grip_border_threshold,
			delay_ms_resp);

	if (tcm->hw_if->dynamic_report_rate) {
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_REPORT_RATE_SWITCH,
				tcm->touch_report_rate_config,
				delay_ms_resp);
	}
}

/* Update a state machine used to toggle control of the touch IC's motion
 * filter.
 */
static int syna_update_motion_filter(struct syna_tcm *tcm, u8 touches)
{
	/* Motion filter timeout, in milliseconds */
	const u32 mf_timeout_ms = 500;
	u8 next_state;

	if (tcm->mf_mode == MF_OFF) {
		if (touches != 0)
			next_state = MF_UNFILTERED;
		else
			next_state = MF_FILTERED;
	} else if (tcm->mf_mode == MF_DYNAMIC) {
		/* Determine the next filter state. The motion filter is enabled by
		 * default and it is disabled while a single finger is touching the
		 * screen. If another finger is touched down or if a timeout expires,
		 * the motion filter is reenabled and remains enabled until all fingers
		 * are lifted.
		 */
		next_state = tcm->mf_state;
		switch (tcm->mf_state) {
		case MF_FILTERED:
			if (touches == 1) {
				next_state = MF_UNFILTERED;
				tcm->mf_downtime = ktime_get();
			}
			break;
		case MF_UNFILTERED:
			if (touches == 0) {
				next_state = MF_FILTERED;
			} else if (touches > 1 ||
				   ktime_after(ktime_get(),
					       ktime_add_ms(tcm->mf_downtime,
							    mf_timeout_ms))) {
				next_state = MF_FILTERED_LOCKED;
			}
			break;
		case MF_FILTERED_LOCKED:
			if (touches == 0) {
				next_state = MF_FILTERED;
			}
			break;
		}
	} else if (tcm->mf_mode == MF_ON) {
		next_state = MF_FILTERED;
	} else {
		/* Set 1(Dynamic) as default when an invalid value is found. */
		tcm->mf_mode = MF_DYNAMIC;
		return 0;
	}

	/* Queue work to update filter state */
	if ((next_state == MF_UNFILTERED) !=
	    (tcm->mf_state == MF_UNFILTERED)) {
		tcm->set_continuously_report = (next_state == MF_UNFILTERED) ? 0x01 : 0x00;
		queue_work(tcm->event_wq, &tcm->motion_filter_work);
	}

	tcm->mf_state = next_state;

	return 0;
}

static void syna_motion_filter_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, motion_filter_work);

	if (tcm->pwr_state != PWR_ON) {
		LOGI("Touch is already off.");
		return;
	}

	/* Send command to update filter state */
	LOGD("setting motion filter = %s.\n",
		 tcm->set_continuously_report ? "false" : "true");
	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_CONTINUOUSLY_REPORT,
			tcm->set_continuously_report,
			RESP_IN_ATTN);
}

static void syna_set_report_rate_work(struct work_struct *work)
{
	struct syna_tcm *tcm;
	struct delayed_work *delayed_work;
	delayed_work = container_of(work, struct delayed_work, work);
	tcm = container_of(delayed_work, struct syna_tcm, set_report_rate_work);

	if (tcm->pwr_state != PWR_ON) {
		LOGI("Touch is already off.");
		return;
	}

	if (tcm->touch_count != 0) {
		queue_delayed_work(tcm->event_wq, &tcm->set_report_rate_work,
				msecs_to_jiffies(10));
		return;
	}

	tcm->touch_report_rate_config = tcm->next_report_rate_config;
	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_REPORT_RATE_SWITCH,
			tcm->touch_report_rate_config,
			RESP_IN_ATTN);
	LOGI("Set touch report rate as %dHz",
		(tcm->touch_report_rate_config == CONFIG_HIGH_REPORT_RATE) ? 240 : 120);
}

static void syna_set_grip_mode_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, set_grip_mode_work);

	if (tcm->pwr_state != PWR_ON) {
		LOGI("Touch is already off.");
		return;
	}

	if (tcm->enable_fw_grip != tcm->next_enable_fw_grip) {
		tcm->enable_fw_grip = tcm->next_enable_fw_grip;
		LOGI("%s firmware grip suppression.\n",
			(tcm->enable_fw_grip == 1) ? "Enable" : "Disable");
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_GRIP_SUPPRESSION,
				tcm->enable_fw_grip,
				RESP_IN_ATTN);
	}
}

static void syna_set_palm_mode_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, set_palm_mode_work);

	if (tcm->pwr_state != PWR_ON) {
		LOGI("Touch is already off.");
		return;
	}

	if (tcm->enable_fw_palm != tcm->next_enable_fw_palm) {
		tcm->enable_fw_palm = tcm->next_enable_fw_palm;
		LOGI("%s firmware palm rejection.\n",
			(tcm->enable_fw_palm == 1) ? "Enable" : "Disable");
		syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_PALM_REJECTION,
				tcm->enable_fw_palm,
				RESP_IN_ATTN);
	}
}

#if defined(ENABLE_HELPER)
/**
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
	tcm->syna_hc.reset_cnt++;
}
/**
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
	struct syna_tcm_helper *helper =
			container_of(work, struct syna_tcm_helper, work);
	struct syna_tcm *tcm =
			container_of(helper, struct syna_tcm, helper);

	if (tcm->pwr_state != PWR_ON) {
		LOGI("Touch is already off.");
		goto exit;
	}

	task = ATOMIC_GET(helper->task);

	switch (task) {
	case HELP_RESET_DETECTED:
		LOGI("Reset caught (device mode:0x%x)\n", tcm->tcm_dev->dev_mode);
		syna_dev_restore_feature_setting(tcm, RESP_IN_ATTN);
		break;
	default:
		break;
	}

exit:
	ATOMIC_SET(helper->task, HELP_NONE);
}
#endif

#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
/**
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
/**
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
	} g_pos;

	bits = config[(*config_offset)++];

	if (code == TOUCH_REPORT_GESTURE_ID) {
		syna_tcm_get_touch_data(report, report_size, offset, bits, &data);

		switch (data) {
		case GESTURE_SINGLE_TAP:
			LOGI("Gesture single tap detected\n");
			break;
		case GESTURE_LONG_PRESS:
			LOGI("Gesture long press detected\n");
			break;
		default:
			LOGW("Unknown gesture id %d\n", data);
			break;
		}

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
		g_pos.major = (unsigned short)data;
		offset += 8;

		syna_tcm_get_touch_data(report, report_size, offset, 8, &data);
		g_pos.minor = (unsigned short)data;
		offset += 8;

		*report_offset += bits;

		LOGI("Gesture data x:%d y:%d major:%d minor:%d\n",
			g_pos.x, g_pos.y, g_pos.major, g_pos.minor);
	} else {
		return -EINVAL;
	}

	return bits;
}
#endif

/**
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
static void syna_dev_free_input_events(struct syna_tcm *tcm)
{
	struct input_dev *input_dev = tcm->input_dev;
#ifdef TYPE_B_PROTOCOL
	unsigned int idx;
#endif
	s64 ms_delta;

	if (input_dev == NULL)
		return;

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	for (idx = 0; idx < MAX_NUM_OBJECTS; idx++) {
		tcm->offload.coords[idx].status = COORD_STATUS_INACTIVE;
		tcm->offload.coords[idx].major = 0;
		tcm->offload.coords[idx].minor = 0;
		tcm->offload.coords[idx].pressure = 0;
	}

	/* If the previous coord_frame that was pushed to touch_offload had
	 * active coords (eg. there are fingers still on the screen), push an
	 * empty coord_frame to touch_offload for clearing coords in twoshay.*/
	if (tcm->touch_offload_active_coords && tcm->offload.offload_running) {
		tcm->offload_reserved_coords = true;
		LOGI("active coords %u", tcm->touch_offload_active_coords);
		reserve_then_queue_frame(tcm, false);
	} else {
#endif
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

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	}
#endif

	for (idx = 0; idx < MAX_NUM_OBJECTS; idx++) {
		if (tcm->prev_obj_status[idx] != LIFT) {
			tcm->syna_hc.touch_info_fifo[idx].ktime_released = ktime_get();

			ms_delta = ktime_ms_delta(tcm->syna_hc.touch_info_fifo[idx].ktime_released,
					tcm->syna_hc.touch_info_fifo[idx].ktime_pressed);
			if (tcm->syna_hc.longest_duration < ms_delta)
				tcm->syna_hc.longest_duration = ms_delta;

			/* Special case to push into kfifo during release all fingers. */
			syna_kfifo_push_coord(tcm, idx);
			tcm->prev_obj_status[idx] = LIFT;
		}
	}

	syna_pal_mutex_unlock(&tcm->tp_event_mutex);

}

/**
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
	s64 ms_delta;

	if (input_dev == NULL)
		return;

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

	object_data = &tcm->tp_data.object_data[0];

#ifdef ENABLE_WAKEUP_GESTURE
	touch_data = &tcm->tp_data;
	if ((tcm->pwr_state == LOW_PWR) && tcm->irq_wake) {
		if (touch_data->gesture_id) {
			LOGD("Gesture detected, id:%d\n",
				touch_data->gesture_id);

			input_report_key(input_dev, KEY_WAKEUP, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_WAKEUP, 0);
			input_sync(input_dev);
		}
	}
#endif

	if (tcm->pwr_state == LOW_PWR)
		goto exit;

	touch_count = 0;

	for (idx = 0; idx < max_objects; idx++) {
		if (tcm->prev_obj_status[idx] == LIFT &&
				object_data[idx].status == LIFT)
			status = NOP;
		else
			status = object_data[idx].status;

		switch (status) {
		case LIFT:
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
			tcm->offload.coords[idx].status = COORD_STATUS_INACTIVE;
			if (!tcm->offload.offload_running) {
#endif
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(input_dev, idx);
			input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, 0);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
			}
#endif

			tcm->syna_hc.touch_info_fifo[idx].ktime_released = tcm->isr_timestamp;
			ms_delta = ktime_ms_delta(tcm->syna_hc.touch_info_fifo[idx].ktime_released,
					tcm->syna_hc.touch_info_fifo[idx].ktime_pressed);
			if (tcm->syna_hc.longest_duration < ms_delta)
				tcm->syna_hc.longest_duration = ms_delta;

			tcm->syna_hc.touch_info_fifo[idx].idx = idx;
			syna_kfifo_push_coord(tcm, idx);

			__clear_bit(idx, &tcm->syna_hc.touch_idx_state);
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
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
			tcm->offload.coords[idx].status = COORD_STATUS_FINGER;
			tcm->offload.coords[idx].x = x;
			tcm->offload.coords[idx].y = y;
			tcm->offload.coords[idx].pressure = z;
#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
			tcm->offload.coords[idx].major = major;
			tcm->offload.coords[idx].minor = minor;
			tcm->offload.coords[idx].rotation = (s16) (((s8) angle) * 2048 / 45);
#else
			tcm->offload.coords[idx].major = MAX(wx, wy);
			tcm->offload.coords[idx].minor = MIN(wx, wy);
#endif
			if (!tcm->offload.offload_running) {
#endif
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
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
			}
#endif
			LOGD("Finger %d: x = %d, y = %d, z = %d\n", idx, x, y, z);

			if (tcm->prev_obj_status[idx] == LIFT) {
				tcm->syna_hc.touch_info_fifo[idx].x_pressed = x;
				tcm->syna_hc.touch_info_fifo[idx].y_pressed = y;
				tcm->syna_hc.touch_info_fifo[idx].ktime_pressed =
					tcm->isr_timestamp;
				tcm->syna_hc.pressed_cnt++;
			}
			tcm->syna_hc.touch_info_fifo[idx].x = x;
			tcm->syna_hc.touch_info_fifo[idx].y = y;
			__set_bit(idx, &tcm->syna_hc.touch_idx_state);
			touch_count++;
			break;
		default:
			break;
		}

		tcm->prev_obj_status[idx] = object_data[idx].status;
	}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	if (!tcm->offload.offload_running) {
#endif
	if (touch_count == 0) {
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_key(input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(input_dev);
#endif
	}

	input_set_timestamp(input_dev, tcm->coords_timestamp);
	input_sync(input_dev);
	tcm->touch_count = touch_count;

	syna_update_motion_filter(tcm, touch_count);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	} else {
		tcm->offload_reserved_coords = true;
	}
#endif

exit:
	syna_pal_mutex_unlock(&tcm->tp_event_mutex);

}

/**
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
#endif

	input_set_abs_params(input_dev,
			ABS_MT_ORIENTATION, -4096, 4096, 0, 0);
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

/**
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
	input_free_device(tcm->input_dev);

	tcm->input_dev = NULL;
}

/**
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

/**
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

	syna_dev_free_input_events(tcm);

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

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
	syna_pal_mutex_unlock(&tcm->tp_event_mutex);

	return retval;
}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
static void syna_offload_set_running(struct syna_tcm *tcm, bool running)
{
	if (tcm->offload.offload_running != running)
		tcm->offload.offload_running = running;

	/*
	 * Use the configurations set by touch service if it's running.
	 * Enable the firmware grip and palm if the touch service isn't running.
	 */
	tcm->next_enable_fw_grip = running ? !tcm->offload.config.filter_grip : 1;
	tcm->next_enable_fw_palm = running ? !tcm->offload.config.filter_palm : 1;

	if (tcm->next_enable_fw_grip != tcm->enable_fw_grip && tcm->enable_fw_grip < 2)
		queue_work(tcm->event_wq, &tcm->set_grip_mode_work);

	if (tcm->next_enable_fw_palm != tcm->enable_fw_palm && tcm->enable_fw_palm < 2)
		queue_work(tcm->event_wq, &tcm->set_palm_mode_work);
}

static void syna_offload_report(void *handle,
				struct TouchOffloadIocReport *report)
{
	struct syna_tcm *tcm = (struct syna_tcm *)handle;
	bool touch_down = 0;
	int i;
	int touch_count = 0;
	int tool_type;

	ATRACE_BEGIN("syna_offload_report");

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

	input_set_timestamp(tcm->input_dev, report->timestamp);

	for (i = 0; i < MAX_COORDS; i++) {
		if (report->coords[i].status != COORD_STATUS_INACTIVE) {
			input_mt_slot(tcm->input_dev, i);
			touch_count++;
			touch_down = 1;
			input_report_key(tcm->input_dev, BTN_TOUCH,
					 touch_down);
			input_report_key(tcm->input_dev, BTN_TOOL_FINGER,
					 touch_down);
			switch (report->coords[i].status) {
			case COORD_STATUS_EDGE:
			case COORD_STATUS_PALM:
			case COORD_STATUS_CANCEL:
				tool_type = MT_TOOL_PALM;
				break;
			case COORD_STATUS_FINGER:
			default:
				tool_type = MT_TOOL_FINGER;
				break;
			}
			input_mt_report_slot_state(tcm->input_dev,
						   tool_type, 1);
			input_report_abs(tcm->input_dev, ABS_MT_POSITION_X,
					 report->coords[i].x);
			input_report_abs(tcm->input_dev, ABS_MT_POSITION_Y,
					 report->coords[i].y);
			input_report_abs(tcm->input_dev, ABS_MT_TOUCH_MAJOR,
					 report->coords[i].major);
			input_report_abs(tcm->input_dev, ABS_MT_TOUCH_MINOR,
					 report->coords[i].minor);
			input_report_abs(tcm->input_dev, ABS_MT_PRESSURE,
					 report->coords[i].pressure);
			input_report_abs(tcm->input_dev, ABS_MT_ORIENTATION,
					 report->coords[i].rotation);
		} else {
			input_mt_slot(tcm->input_dev, i);
			input_report_abs(tcm->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(tcm->input_dev,
						   MT_TOOL_FINGER, 0);
			input_report_abs(tcm->input_dev, ABS_MT_TRACKING_ID,
					 -1);
		}
	}
	input_report_key(tcm->input_dev, BTN_TOUCH, touch_down);
	input_report_key(tcm->input_dev, BTN_TOOL_FINGER, touch_down);

	input_sync(tcm->input_dev);
	tcm->touch_count = touch_count;

	syna_update_motion_filter(tcm, touch_count);

	syna_pal_mutex_unlock(&tcm->tp_event_mutex);

	ATRACE_END();
}

static int syna_heatmap_alloc_check(struct syna_tcm *tcm)
{
	if (tcm->heatmap_buff)
	        return 0;

	tcm->heatmap_buff = devm_kzalloc(&tcm->pdev->dev,
			sizeof(u16) * tcm->tcm_dev->rows * tcm->tcm_dev->cols, GFP_KERNEL);
	if (!tcm->heatmap_buff) {
		LOGE("allocate heatmap_buff failed\n");
		return -ENOMEM;
	}
	LOGI("Allocate heatmap memory, rows: %d, cols: %d\n", tcm->tcm_dev->rows,
	     tcm->tcm_dev->cols);

	return 0;
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

static void syna_populate_coordinate_channel(struct syna_tcm *tcm,
					     struct touch_offload_frame *frame,
					     int channel)
{
	int j;
	u8 active_coords = 0;

	struct TouchOffloadDataCoord *dc =
		(struct TouchOffloadDataCoord *)frame->channel_data[channel];
	memset(dc, 0, frame->channel_data_size[channel]);
	dc->header.channel_type = TOUCH_DATA_TYPE_COORD;
	dc->header.channel_size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;

	for (j = 0; j < MAX_COORDS; j++) {
		dc->coords[j].x = tcm->offload.coords[j].x;
		dc->coords[j].y = tcm->offload.coords[j].y;
		dc->coords[j].major = tcm->offload.coords[j].major;
		dc->coords[j].minor = tcm->offload.coords[j].minor;
		dc->coords[j].pressure = tcm->offload.coords[j].pressure;
		dc->coords[j].status = tcm->offload.coords[j].status;
		if (dc->coords[j].status != COORD_STATUS_INACTIVE)
			active_coords += 1;
	}

	tcm->touch_offload_active_coords = active_coords;
}

static void syna_populate_mutual_channel(struct syna_tcm *tcm,
					 struct touch_offload_frame *frame,
					 int channel, bool has_heatmap)
{
	int i, j;
	struct TouchOffloadData2d *mutual_strength =
		(struct TouchOffloadData2d *)frame->channel_data[channel];

	mutual_strength->tx_size = tcm->tcm_dev->rows;
	mutual_strength->rx_size = tcm->tcm_dev->cols;
	mutual_strength->header.channel_type = frame->channel_type[channel];
	mutual_strength->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_2D(mutual_strength->rx_size,
					    mutual_strength->tx_size);

	if (syna_heatmap_alloc_check(tcm) != 0)
		return;

	if (has_heatmap) {
		/* for 'heat map' ($c3) report,
		 * report data has been stored at tcm->event_data.buf;
		 * while, tcm->event_data.data_length is the size of data
		 */
		syna_dev_ptflib_decoder(tcm,
		    &((u16 *) tcm->event_data.buf)[tcm->tcm_dev->rows + tcm->tcm_dev->cols],
		    (tcm->event_data.data_length) / 2 - tcm->tcm_dev->rows - tcm->tcm_dev->cols,
		    tcm->heatmap_buff,
		    tcm->tcm_dev->rows * tcm->tcm_dev->cols);

		for (i = 0; i < tcm->tcm_dev->cols; i++) {
			for (j = 0; j < tcm->tcm_dev->rows; j++) {
				((u16 *) mutual_strength->data)[tcm->tcm_dev->rows * i + j] =
					tcm->heatmap_buff[tcm->tcm_dev->cols * j + i];
			}
		}
		memcpy(tcm->heatmap_buff, (u16 *) mutual_strength->data,
			tcm->tcm_dev->cols * tcm->tcm_dev->rows * sizeof(u16));
	} else {
		memset(mutual_strength->data, 0,
			tcm->tcm_dev->cols * tcm->tcm_dev->rows * sizeof(u16));
	}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	tcm->heatmap_decoded = true;
#endif
}

static void syna_populate_self_channel(struct syna_tcm *tcm,
				       struct touch_offload_frame *frame,
				       int channel, bool has_heatmap)
{
	int i;
	struct TouchOffloadData1d *self_strength =
		(struct TouchOffloadData1d *)frame->channel_data[channel];

	self_strength->tx_size = tcm->tcm_dev->rows;
	self_strength->rx_size = tcm->tcm_dev->cols;
	self_strength->header.channel_type = frame->channel_type[channel];
	self_strength->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_1D(self_strength->rx_size,
					    self_strength->tx_size);
	if (has_heatmap) {
		for (i = 0; i < tcm->tcm_dev->rows; i++) {
			((u16 *) self_strength->data)[i] =
				((u16 *) tcm->event_data.buf)[tcm->tcm_dev->cols + i];
		}

		for (i = 0; i < tcm->tcm_dev->cols; i++) {
			((u16 *) self_strength->data)[tcm->tcm_dev->rows + i] =
				((u16 *) tcm->event_data.buf)[i];
		}
	} else {
		memset(self_strength->data, 0,
			(tcm->tcm_dev->cols + tcm->tcm_dev->rows) * sizeof(u16));
	}
}

static void syna_populate_frame(struct syna_tcm *tcm, bool has_heatmap)
{
	static u64 index;
	int i;
	struct touch_offload_frame *frame = tcm->reserved_frame;

	frame->header.index = index++;
	frame->header.timestamp = tcm->coords_timestamp;

	ATRACE_BEGIN(__func__);

	/* Populate all channels */
	for (i = 0; i < frame->num_channels; i++) {
		if (frame->channel_type[i] == TOUCH_DATA_TYPE_COORD) {
			ATRACE_BEGIN("populate coord");
			syna_populate_coordinate_channel(tcm, frame, i);
			ATRACE_END();
		} else if ((frame->channel_type[i] & TOUCH_SCAN_TYPE_MUTUAL) != 0) {
			ATRACE_BEGIN("populate mutual data");
			syna_populate_mutual_channel(tcm, frame, i, has_heatmap);
			ATRACE_END();
		} else if ((frame->channel_type[i] & TOUCH_SCAN_TYPE_SELF) != 0) {
			ATRACE_BEGIN("populate self data");
			syna_populate_self_channel(tcm, frame, i, has_heatmap);
			ATRACE_END();
		}
	}
	ATRACE_END();
}

static void reserve_then_queue_frame(struct syna_tcm *tcm, bool has_heatmap) {
	int retval;

	/* Skip if no reserved coordinates. */
	if (!tcm->offload_reserved_coords && tcm->offload.offload_running)
		return;

	retval = touch_offload_reserve_frame(&tcm->offload, &tcm->reserved_frame);
	if (retval != 0) {
		LOGD("Could not reserve a frame: error=%d.\n", retval);

		/* Stop offload when there are no buffers available. */
		syna_offload_set_running(tcm, false);
	} else {
		syna_offload_set_running(tcm, true);

		syna_populate_frame(tcm, has_heatmap);

		retval = touch_offload_queue_frame(&tcm->offload, tcm->reserved_frame);
		if (retval != 0)
			LOGE("Failed to queue reserved frame: error=%d.\n", retval);

		tcm->offload_reserved_coords = false;
	}
}
#endif /* CONFIG_TOUCHSCREEN_OFFLOAD */

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
static bool v4l2_read_frame(struct v4l2_heatmap *v4l2)
{
	bool ret = true;
	int i, j;
	struct syna_tcm *tcm =
		container_of(v4l2, struct syna_tcm, v4l2);

	if (tcm->v4l2.width == tcm->tcm_dev->rows &&
		tcm->v4l2.height == tcm->tcm_dev->cols) {
		if (tcm->heatmap_decoded) {
			memcpy(v4l2->frame, tcm->heatmap_buff,
			       tcm->v4l2.width * tcm->v4l2.height * sizeof(u16));
		} else {
		    if (syna_heatmap_alloc_check(tcm) != 0)
			return false;

		    /* for 'heat map' ($c3) report,
		     * report data has been stored at tcm->event_data.buf;
		     * while, tcm->event_data.data_length is the size of data
		     */
		    syna_dev_ptflib_decoder(tcm,
			&((u16 *) tcm->event_data.buf)[tcm->tcm_dev->rows + tcm->tcm_dev->cols],
			(tcm->event_data.data_length) / 2 -
			    tcm->tcm_dev->rows - tcm->tcm_dev->cols,
			tcm->heatmap_buff, tcm->tcm_dev->rows * tcm->tcm_dev->cols);

		    for (i = 0; i < tcm->tcm_dev->cols; i++) {
			for (j = 0; j < tcm->tcm_dev->rows; j++) {
			    ((u16 *) v4l2->frame)[tcm->tcm_dev->rows * i + j] =
				tcm->heatmap_buff[tcm->tcm_dev->cols * j + i];
			}
		    }
		}
	} else {
		LOGE("size mismatched, (%lu, %lu) vs (%u, %u)!\n",
		tcm->v4l2.width, tcm->v4l2.height,
		tcm->tcm_dev->rows, tcm->tcm_dev->cols);
		ret = false;
	}
	tcm->heatmap_decoded = false;

	return ret;
}
#endif

static irqreturn_t syna_dev_isr(int irq, void *handle)
{
	struct syna_tcm *tcm = handle;

	tcm->isr_timestamp = ktime_get();
	tcm->syna_hc.int_cnt++;

	return IRQ_WAKE_THREAD;
}

/**
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
	bool force_complete_all = false;
	struct syna_tcm *tcm = data;
	struct custom_fw_status *status;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;
	memset(&tcm->syna_hc.hc_fifo, 0, sizeof(tcm->syna_hc.hc_fifo));

	ATRACE_BEGIN(__func__);

	cpu_latency_qos_update_request(&tcm->pm_qos_req, 100 /* usec */);

	/* It is possible that interrupts were disabled while the handler is
	 * executing, before acquiring the mutex. If so, simply return.
	 */
	if (syna_set_bus_ref(tcm, SYNA_BUS_REF_IRQ, true) < 0) {
		if (tcm->pwr_state != PWR_ON || completion_done(&tcm_dev->msg_data.cmd_completion))
			goto exit;

		force_complete_all = true;
		LOGI("There is pending cmd_completion.\n");
	}

	if (unlikely(gpio_get_value(attn->irq_gpio) != attn->irq_on_state))
		goto exit;

	tcm->isr_pid = current->pid;

#ifdef HAS_SYSFS_INTERFACE
	if (tcm->is_attn_redirecting) {
		syna_cdev_redirect_attn(tcm);
		goto exit;
	}
#endif
	ATRACE_BEGIN("get_event_data");
	/* retrieve the original report date generated by firmware */
	retval = syna_tcm_get_event_data(tcm->tcm_dev,
			&code,
			&tcm->event_data);
	if (retval < 0) {
		LOGE("Fail to get event data\n");
		goto exit;
	}
	ATRACE_END();

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
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
		/* Reserve and queue the frame if last reserved coordinates is pending. */
		if ((tcm->offload_reserved_coords) && (tcm->offload.offload_running)) {
			LOGW("Last reserved coordinates was not queued.\n");
			reserve_then_queue_frame(tcm, false);
		}
#endif

		/* parse touch report once received */
		retval = syna_tcm_parse_touch_report(tcm->tcm_dev,
				tcm->event_data.buf,
				tcm->event_data.data_length,
				&tcm->tp_data);
		if (retval < 0) {
			LOGE("Fail to parse touch report\n");
			goto exit;
		}
		tcm->coords_timestamp = tcm->isr_timestamp;
		tcm->syna_hc.coord_event_cnt++;
		tcm->syna_hc.hc_fifo.coord_updated = true;

		/* forward the touch event to system */
		ATRACE_BEGIN("report_input_events");
		syna_dev_report_input_events(tcm);
		ATRACE_END();
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
			memcpy(tcm->raw_data_buffer, tcm->event_data.buf,
			       tcm->event_data.data_length);
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
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
		reserve_then_queue_frame(tcm, true);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
		ATRACE_BEGIN("heatmap_read");
		heatmap_read(&tcm->v4l2, ktime_to_ns(tcm->coords_timestamp));
		ATRACE_END();
#endif
		break;
	case REPORT_FW_STATUS:
		/* for 'fw status' ($c2) report,
		 * report size shall be 2-byte only; the
		 */
		status = (struct custom_fw_status *)&tcm->event_data.buf[0];
		LOGI("Status: moisture:%d noise:%d freq-change:%d, grip:%d, palm:%d\n",
			status->b0_moisture, status->b1_noise_state,
			status->b2_freq_hopping, status->b3_grip, status->b4_palm);

		if (status->b0_moisture)
			tcm->syna_hc.wet_cnt++;

		if (status->b4_palm)
			tcm->syna_hc.palm_cnt++;

		tcm->syna_hc.status_event_cnt++;
		tcm->syna_hc.hc_fifo.status_updated = true;
		break;
	default:
		break;
	}

	/* Update the health check info. */
	syna_hc_update_and_push(tcm, &tcm->syna_hc.hc_fifo);

exit:
	syna_set_bus_ref(tcm, SYNA_BUS_REF_IRQ, false);
	if (force_complete_all)
		complete_all(&tcm_dev->msg_data.cmd_completion);
	cpu_latency_qos_update_request(&tcm->pm_qos_req, PM_QOS_DEFAULT_VALUE);
	ATRACE_END();
	return IRQ_HANDLED;
}


/**
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

/**
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

	if (tcm->hw_if->ops_enable_irq)
		tcm->hw_if->ops_enable_irq(tcm->hw_if, false);

#ifdef DEV_MANAGED_API
	devm_free_irq(dev, attn->irq_id, tcm);
#else
	free_irq(attn->irq_id, tcm);
#endif

	attn->irq_id = 0;
	attn->irq_enabled = false;

	LOGI("Interrupt handler released\n");
}

/**
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
		return -EIO;
	}

	/* set up the format of touch report */
#ifdef USE_CUSTOM_TOUCH_REPORT_CONFIG
	retval = syna_tcm_set_touch_report_config(tcm_dev,
			custom_touch_format,
			(unsigned int)sizeof(custom_touch_format));
	if (retval < 0) {
		LOGE("Fail to setup the custom touch report format\n");
		return -EIO;
	}
#endif
	/* preserve the format of touch report */
	retval = syna_tcm_preserve_touch_report_config(tcm_dev);
	if (retval < 0) {
		LOGE("Fail to preserve touch report config\n");
		return -EIO;
	}

#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
	/* set up custom touch data parsing method */
	retval = syna_tcm_set_custom_touch_entity_callback(tcm_dev,
			syna_dev_parse_custom_touch_data_cb,
			(void *)tcm);
	if (retval < 0) {
		LOGE("Fail to set up custom touch data parsing method\n");
		return -EIO;
	}
#endif
#ifdef ENABLE_WAKEUP_GESTURE
	/* set up custom gesture parsing method */
	retval = syna_tcm_set_custom_gesture_callback(tcm_dev,
			syna_dev_parse_custom_gesture_cb,
			(void *)tcm);
	if (retval < 0) {
		LOGE("Fail to set up custom gesture parsing method\n");
		return -EIO;
	}
#endif
	return retval;
}

/**
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
#ifdef STARTUP_REFLASH
static void syna_dev_reflash_startup_work(struct work_struct *work)
{
	int retval;
	struct delayed_work *delayed_work;
	struct syna_tcm *tcm;
	struct tcm_dev *tcm_dev;
	const struct firmware *fw_entry;
	const unsigned char *fw_image = NULL;
	unsigned int fw_image_size;

	delayed_work = container_of(work, struct delayed_work, work);
	tcm = container_of(delayed_work, struct syna_tcm, reflash_work);

	tcm_dev = tcm->tcm_dev;

	/* get firmware image */
	retval = request_firmware(&fw_entry,
			tcm->hw_if->fw_name,
			tcm->pdev->dev.parent);
	if (retval < 0) {
		LOGE("Fail to request %s\n", tcm->hw_if->fw_name);
		return;
	}

	fw_image = fw_entry->data;
	fw_image_size = fw_entry->size;

	LOGD("Firmware image size = %d\n", fw_image_size);

	pm_stay_awake(&tcm->pdev->dev);

	if (syna_set_bus_ref(tcm, SYNA_BUS_REF_FW_UPDATE, true) < 0)
		goto exit;

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
			RESP_IN_ATTN,
			tcm->force_reflash);
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
exit:
	syna_set_bus_ref(tcm, SYNA_BUS_REF_FW_UPDATE, false);
	pm_relax(&tcm->pdev->dev);
}
#endif
#if defined(POWER_ALIVE_AT_SUSPEND) && !defined(RESET_ON_RESUME)
/**
 * syna_dev_enter_normal_sensing()
 *
 * Helper to enter normal seneing mode
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

	return 0;
}
#endif
#ifdef POWER_ALIVE_AT_SUSPEND
/**
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

/**
 * Report a finger down event on the long press gesture area then immediately
 * report a cancel event(MT_TOOL_PALM).
 */
static void syna_report_cancel_event(struct syna_tcm *tcm)
{
	LOGI("Report cancel event for UDFPS");

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

	/* Finger down on UDFPS area. */
	input_mt_slot(tcm->input_dev, 0);
	input_report_key(tcm->input_dev, BTN_TOUCH, 1);
	input_mt_report_slot_state(tcm->input_dev, MT_TOOL_FINGER, 1);
	input_report_abs(tcm->input_dev, ABS_MT_POSITION_X, tcm->hw_if->udfps_x);
	input_report_abs(tcm->input_dev, ABS_MT_POSITION_Y, tcm->hw_if->udfps_y);
	input_report_abs(tcm->input_dev, ABS_MT_TOUCH_MAJOR, 200);
	input_report_abs(tcm->input_dev, ABS_MT_TOUCH_MINOR, 200);
#ifndef SKIP_PRESSURE
	input_report_abs(tcm->input_dev, ABS_MT_PRESSURE, 1);
#endif
	input_report_abs(tcm->input_dev, ABS_MT_ORIENTATION, 0);
	input_sync(tcm->input_dev);

	/* Report MT_TOOL_PALM for canceling the touch event. */
	input_mt_slot(tcm->input_dev, 0);
	input_report_key(tcm->input_dev, BTN_TOUCH, 1);
	input_mt_report_slot_state(tcm->input_dev, MT_TOOL_PALM, 1);
	input_sync(tcm->input_dev);

	/* Release touches. */
	input_mt_slot(tcm->input_dev, 0);
	input_report_abs(tcm->input_dev, ABS_MT_PRESSURE, 0);
	input_mt_report_slot_state(tcm->input_dev, MT_TOOL_FINGER, 0);
	input_report_abs(tcm->input_dev, ABS_MT_TRACKING_ID, -1);
	input_report_key(tcm->input_dev, BTN_TOUCH, 0);
	input_sync(tcm->input_dev);

	syna_pal_mutex_unlock(&tcm->tp_event_mutex);
}

static void syna_check_finger_status(struct syna_tcm *tcm)
{
	int retval = 0;
	unsigned char code = 0;
	u16 touch_mode, x, y, major, minor;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
	ktime_t timeout = ktime_add_ms(ktime_get(), 500);
	LOGI("Check finger status");

	while (ktime_get() < timeout) {
		/* Clear the FIFO if there is pending data. */
		if (gpio_get_value(attn->irq_gpio) == attn->irq_on_state) {
			retval = tcm->tcm_dev->read_message(tcm->tcm_dev, &code);
			continue;
		}

		retval = syna_tcm_get_dynamic_config(tcm->tcm_dev, DC_TOUCH_SCAN_MODE,
				&touch_mode, RESP_IN_POLLING);
		if (retval < 0)
			continue;

		if (touch_mode != SCAN_LPWG_IDLE && touch_mode != SCAN_LPWG_ACTIVE)
			return;

		LOGI("Poll finger events.");
		while (ktime_get() < timeout) {
			msleep(30);
			syna_tcm_get_event_data(tcm->tcm_dev,
				&code,
				&tcm->event_data);
			if (code == REPORT_TOUCH) {
				x = syna_pal_le2_to_uint(&tcm->event_data.buf[1]);
				y = syna_pal_le2_to_uint(&tcm->event_data.buf[3]);
				major = tcm->event_data.buf[4];
				minor = tcm->event_data.buf[5];
				/* Touch reports coordinates and major/minor 0
				 * when the finger leaves.
				 */
				if (x==0 && y==0 && major==0 && minor==0) {
					syna_report_cancel_event(tcm);
					return;
				}
			} else if (code == STATUS_INVALID) {
				syna_report_cancel_event(tcm);
				return;
			}
		}
	}
}

/**
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
	int retry = 0;
	struct syna_tcm *tcm = dev_get_drvdata(dev);
	struct syna_hw_interface *hw_if = tcm->hw_if;
	bool irq_enabled = true;
#ifdef RESET_ON_RESUME
	unsigned char status;
#endif

	/* exit directly if device isn't in suspend state */
	if (tcm->pwr_state == PWR_ON)
		return 0;

	LOGI("Prepare to resume device\n");

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (tcm->tbn_register_mask)
		tbn_request_bus(tcm->tbn_register_mask);
#endif

	syna_pinctrl_configure(tcm, true);

	if (hw_if->udfps_x != 0 && hw_if->udfps_y != 0)
		syna_check_finger_status(tcm);

	/* clear all input events  */
	syna_dev_free_input_events(tcm);

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
		LOGE("Fail to enter normal power mode\n");
		goto exit;
	}
#endif
	retval = syna_tcm_rezero(tcm->tcm_dev);
	if (retval < 0) {
		LOGE("Fail to rezero\n");
		goto exit;
	}
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

	complete_all(&tcm->bus_resumed);

	return retval;
}

/**
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
	int retval;
#endif
	struct syna_tcm *tcm = dev_get_drvdata(dev);
	struct syna_hw_interface *hw_if = tcm->hw_if;
	bool irq_disabled = true;
	unsigned char status;

	/* exit directly if device is already in suspend state */
	if (tcm->pwr_state != PWR_ON)
		return 0;

	LOGI("Prepare to suspend device\n");

	if (tcm->hw_if->dynamic_report_rate)
		cancel_delayed_work_sync(&tcm->set_report_rate_work);

	reinit_completion(&tcm->bus_resumed);

	/* clear all input events  */
	syna_dev_free_input_events(tcm);

#ifdef POWER_ALIVE_AT_SUSPEND
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
	tcm->pwr_state = LOW_PWR;
#else
	tcm->pwr_state = PWR_OFF;
#endif

	/* once lpwg is enabled, irq should be alive.
	 * otherwise, disable irq in suspend.
	 */
	irq_disabled = (!tcm->lpwg_enabled);

	/* disable irq */
	if (irq_disabled && (hw_if->ops_enable_irq))
		hw_if->ops_enable_irq(hw_if, false);

	syna_dev_set_heatmap_mode(tcm, false);

	syna_pinctrl_configure(tcm, false);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (tcm->tbn_register_mask)
		tbn_release_bus(tcm->tbn_register_mask);
#endif
	LOGI("Device suspended (pwr_state:%d), int_cnt:%llu\n", tcm->pwr_state,
	     tcm->syna_hc.int_cnt);

	return 0;
}

#if defined(ENABLE_DISP_NOTIFIER)
/**
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
/**
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
				LOGE("Timed out waiting for reflashing\n");
				ATOMIC_SET(tcm->tcm_dev->firmware_flashing, 0);
				return -EIO;
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

static void syna_suspend_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, suspend_work);

	syna_dev_suspend(&tcm->pdev->dev);
	syna_hc_dump(tcm);
	syna_debug_dump(tcm);
}

static void syna_resume_work(struct work_struct *work)
{
	struct syna_tcm *tcm = container_of(work, struct syna_tcm, resume_work);

	syna_dev_resume(&tcm->pdev->dev);

	tcm->syna_hc.reset_cnt = 0;
	tcm->syna_hc.longest_duration = 0;
	tcm->syna_hc.pressed_cnt = 0;
	tcm->syna_hc.palm_cnt = 0;
	tcm->syna_hc.wet_cnt = 0;
}

static void syna_aggregate_bus_state(struct syna_tcm *tcm)
{
	/* Complete or cancel any outstanding transitions */
	cancel_work_sync(&tcm->suspend_work);
	cancel_work_sync(&tcm->resume_work);

	if ((tcm->bus_refmask == 0 && tcm->pwr_state != PWR_ON) ||
	    (tcm->bus_refmask && tcm->pwr_state == PWR_ON))
		return;

	if (tcm->bus_refmask == 0)
		queue_work(tcm->event_wq, &tcm->suspend_work);
	else
		queue_work(tcm->event_wq, &tcm->resume_work);
}

int syna_set_bus_ref(struct syna_tcm *tcm, u32 ref, bool enable)
{
	int result = 0;

	mutex_lock(&tcm->bus_mutex);

	if ((enable && (tcm->bus_refmask & ref)) ||
	    (!enable && !(tcm->bus_refmask & ref))) {
		LOGD("reference is unexpectedly set: mask=0x%04X, ref=0x%04X, enable=%d\n",
		tcm->bus_refmask, ref, enable);
		mutex_unlock(&tcm->bus_mutex);
		return -EINVAL;
	}

	if (enable) {
		/*
		 * IRQs can only keep the bus active. IRQs received while the
		 * bus is transferred to AOC should be ignored.
		 */
		if (ref == SYNA_BUS_REF_IRQ && tcm->bus_refmask == 0) {
			mutex_unlock(&tcm->bus_mutex);
			return -EAGAIN;
		} else {
			tcm->bus_refmask |= ref;
		}
	} else
		tcm->bus_refmask &= ~ref;
	syna_aggregate_bus_state(tcm);

	mutex_unlock(&tcm->bus_mutex);

	/*
	 * When triggering a wake, wait up to one second to resume. SCREEN_ON
	 * and IRQ references do not need to wait.
	 */
	if (enable &&
	    ref != SYNA_BUS_REF_SCREEN_ON && ref != SYNA_BUS_REF_IRQ) {
		wait_for_completion_timeout(&tcm->bus_resumed, HZ);
		if (tcm->pwr_state != PWR_ON) {
			LOGE("Failed to wake the touch bus.\n");
			result = -ETIMEDOUT;
		}
	}

	return result;
}

#if defined(USE_DRM_BRIDGE)
static struct drm_connector *syna_get_bridge_connector(struct drm_bridge *bridge)
{
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;

	drm_connector_list_iter_begin(bridge->dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		if (connector->encoder == bridge->encoder)
			break;
	}
	drm_connector_list_iter_end(&conn_iter);
	return connector;
}

static bool syna_bridge_is_lp_mode(struct drm_connector *connector)
{
	if (connector && connector->state) {
		struct exynos_drm_connector_state *s =
			to_exynos_connector_state(connector->state);
		return s->exynos_mode.is_lp_mode;
	}
	return false;
}

static void syna_panel_bridge_enable(struct drm_bridge *bridge)
{
	struct syna_tcm *tcm =
			container_of(bridge, struct syna_tcm, panel_bridge);

	LOGD("%s\n", __func__);
	if (!tcm ->is_panel_lp_mode)
		syna_set_bus_ref(tcm, SYNA_BUS_REF_SCREEN_ON, true);
}

static void syna_panel_bridge_disable(struct drm_bridge *bridge)
{
	struct syna_tcm *tcm =
			container_of(bridge, struct syna_tcm, panel_bridge);

	if (bridge->encoder && bridge->encoder->crtc) {
		const struct drm_crtc_state *crtc_state = bridge->encoder->crtc->state;

		if (drm_atomic_crtc_effectively_active(crtc_state))
			return;
	}

	LOGD("%s\n", __func__);
	syna_set_bus_ref(tcm, SYNA_BUS_REF_SCREEN_ON, false);
}

static void syna_panel_bridge_mode_set(struct drm_bridge *bridge,
				       const struct drm_display_mode *mode,
				       const struct drm_display_mode *adjusted_mode)
{
	struct syna_tcm *tcm =
			container_of(bridge, struct syna_tcm, panel_bridge);

	LOGD("%s\n", __func__);

	if (!tcm->connector || !tcm->connector->state) {
		LOGI("%s: Get bridge connector.\n", __func__);
		tcm->connector = syna_get_bridge_connector(bridge);
	}

	tcm->is_panel_lp_mode = syna_bridge_is_lp_mode(tcm->connector);
	if (tcm->is_panel_lp_mode)
		syna_set_bus_ref(tcm, SYNA_BUS_REF_SCREEN_ON, false);
	else
		syna_set_bus_ref(tcm, SYNA_BUS_REF_SCREEN_ON, true);

	if (mode && tcm->hw_if->dynamic_report_rate) {
		int vrefresh = drm_mode_vrefresh(mode);
		LOGD("Display refresh rate %dHz", vrefresh);
		if (vrefresh == 120 || vrefresh == 90)
			tcm->next_report_rate_config = CONFIG_HIGH_REPORT_RATE;
		else
			tcm->next_report_rate_config = CONFIG_LOW_REPORT_RATE;

		if (tcm->last_vrefresh_rate != vrefresh)
			cancel_delayed_work_sync(&tcm->set_report_rate_work);

		if (tcm->next_report_rate_config != tcm->touch_report_rate_config &&
			tcm->pwr_state == PWR_ON && tcm->bus_refmask != 0) {
			/*
			 * Queue the work immediately for increasing touch report rate
			 * to 240Hz and queue 2 seconds delay work for decreasing
			 * touch report rate.
			 */
			queue_delayed_work(tcm->event_wq, &tcm->set_report_rate_work,
				(tcm->next_report_rate_config == CONFIG_HIGH_REPORT_RATE) ?
				0 : msecs_to_jiffies(2 * MSEC_PER_SEC));
		}
		tcm->last_vrefresh_rate = vrefresh;
	}
}

static const struct drm_bridge_funcs panel_bridge_funcs = {
	.enable = syna_panel_bridge_enable,
	.disable = syna_panel_bridge_disable,
	.mode_set = syna_panel_bridge_mode_set,
};

static int syna_register_panel_bridge(struct syna_tcm *tcm)
{
#ifdef CONFIG_OF
	tcm->panel_bridge.of_node = tcm->pdev->dev.parent->of_node;
#endif
	tcm->panel_bridge.funcs = &panel_bridge_funcs;
	drm_bridge_add(&tcm->panel_bridge);

	return 0;
}

static void syna_unregister_panel_bridge(struct drm_bridge *bridge)
{
	struct drm_bridge *node;

	drm_bridge_remove(bridge);

	if (!bridge->dev) /* not attached */
		return;

	drm_modeset_lock(&bridge->dev->mode_config.connection_mutex, NULL);
	list_for_each_entry(node, &bridge->encoder->bridge_chain, chain_node)
		if (node == bridge) {
			if (bridge->funcs->detach)
				bridge->funcs->detach(bridge);
			list_del(&bridge->chain_node);
			break;
		}
	drm_modeset_unlock(&bridge->dev->mode_config.connection_mutex);
	bridge->dev = NULL;
}
#endif

/**
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

	/* unregister input device */
	syna_dev_release_input_device(tcm);

	tcm->input_dev_params.max_x = 0;
	tcm->input_dev_params.max_y = 0;
	tcm->input_dev_params.max_objects = 0;

	/* power off */
	if (hw_if->ops_power_on)
		hw_if->ops_power_on(hw_if, false);

	tcm->pwr_state = PWR_OFF;
	tcm->is_connected = false;

	LOGI("%s device disconnected\n", PLATFORM_DRIVER_NAME);

	return 0;
}

/**
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
		LOGI("%s already connected\n", PLATFORM_DRIVER_NAME);
		return 0;
	}

	/* power on the connected device */
	if (hw_if->ops_power_on) {
		retval = hw_if->ops_power_on(hw_if, true);
		if (retval < 0)
			return -ENODEV;
	}

	/* perform a hardware reset */
	if (hw_if->ops_hw_reset)
		hw_if->ops_hw_reset(hw_if);

	/* detect which modes of touch controller is running
	 *
	 * the signal of the Touch IC ready is called as "identify"
	 * report and generated by firmware
	 */
	retval = syna_tcm_detect_device(tcm->tcm_dev);
	if (retval < 0) {
		LOGE("Fail to detect the device\n");
		goto err_detect_dev;
	}

	switch (retval) {
	case MODE_APPLICATION_FIRMWARE:
		retval = syna_dev_set_up_app_fw(tcm);
		if (retval < 0) {
#ifdef FORCE_CONNECTION
			LOGW("App firmware is not available yet\n");
			LOGW("However, connect and skip initialization\n");
#else
			LOGE("Fail to set up application firmware\n");
			/* switch to bootloader mode when failed */
			LOGI("Switch device to bootloader mode instead\n");
			syna_tcm_switch_fw_mode(tcm_dev,
					MODE_BOOTLOADER,
					FW_MODE_SWITCH_DELAY_MS);
#endif
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

	tcm->pwr_state = PWR_ON;
	tcm->is_connected = true;
	tcm->bus_refmask = SYNA_BUS_REF_SCREEN_ON;

	LOGI("TCM packrat: %d\n", tcm->tcm_dev->packrat_number);
	LOGI("Configuration: name(%s), lpwg(%s), hw_reset(%s), irq_ctrl(%s)\n",
		PLATFORM_DRIVER_NAME,
		(tcm->lpwg_enabled) ? "yes" : "no",
		(hw_if->ops_hw_reset) ? "yes" : "no",
		(hw_if->ops_enable_irq) ? "yes" : "no");

	return 0;

err_setup_input_dev:
err_detect_dev:
	if (hw_if->ops_power_on)
		hw_if->ops_power_on(hw_if, false);

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

/**
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

	syna_pal_mutex_alloc(&tcm->tp_event_mutex);
	mutex_init(&tcm->bus_mutex);

#ifdef ENABLE_WAKEUP_GESTURE
	tcm->lpwg_enabled = true;
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

	platform_set_drvdata(pdev, tcm);

	device_init_wakeup(&pdev->dev, 1);

	tcm->event_wq = alloc_workqueue("syna_wq", WQ_UNBOUND |
					WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);

	if (!tcm->event_wq) {
		LOGE("Cannot create work thread\n");
		retval = -ENOMEM;
		goto err_alloc_workqueue;
	}

	INIT_WORK(&tcm->suspend_work, syna_suspend_work);
	INIT_WORK(&tcm->resume_work, syna_resume_work);

	init_completion(&tcm->bus_resumed);
	complete_all(&tcm->bus_resumed);

	cpu_latency_qos_add_request(&tcm->pm_qos_req, PM_QOS_DEFAULT_VALUE);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (register_tbn(&tcm->tbn_register_mask)) {
		retval = -ENODEV;
		LOGE("Failed to register tbn context.");
		goto err_connect;
	}
	LOGI("tbn_register_mask = %#x.\n", tcm->tbn_register_mask);
#endif

#if defined(TCM_CONNECT_IN_PROBE)
	/* connect to target device */
	retval = tcm->dev_connect(tcm);
	if (retval < 0) {
		LOGE("Fail to connect to the device\n");
		retval = -EPROBE_DEFER;
		syna_pal_mutex_free(&tcm->tp_event_mutex);
		goto err_connect;
	}
#endif

	tcm->raw_data_report_code = 0;
	init_completion(&tcm->raw_data_completion);
	complete_all(&tcm->raw_data_completion);

	tcm->syna_hc.int_cnt = 0;
	tcm->syna_hc.coord_event_cnt = 0;
	tcm->syna_hc.status_event_cnt = 0;
	tcm->syna_hc.touch_idx_state = 0;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	tcm->offload.caps.touch_offload_major_version = TOUCH_OFFLOAD_INTERFACE_MAJOR_VERSION;
	tcm->offload.caps.touch_offload_minor_version = TOUCH_OFFLOAD_INTERFACE_MINOR_VERSION;
	tcm->offload.caps.device_id = tcm->hw_if->offload_id;
	tcm->offload.caps.display_width = tcm->tcm_dev->max_x;
	tcm->offload.caps.display_height = tcm->tcm_dev->max_y;
	tcm->offload.caps.tx_size = tcm->tcm_dev->rows;
	tcm->offload.caps.rx_size = tcm->tcm_dev->cols;

	tcm->offload.caps.bus_type = BUS_TYPE_SPI;
	tcm->offload.caps.bus_speed_hz = 10000000;
	tcm->offload.caps.heatmap_size = HEATMAP_SIZE_FULL;
	tcm->offload.caps.touch_data_types = TOUCH_DATA_TYPE_COORD |
					     TOUCH_DATA_TYPE_STRENGTH;
	tcm->offload.caps.touch_scan_types = TOUCH_SCAN_TYPE_MUTUAL |
					     TOUCH_SCAN_TYPE_SELF;
	tcm->offload.caps.continuous_reporting = true;
	tcm->offload.caps.noise_reporting = false;
	tcm->offload.caps.cancel_reporting = true;
	tcm->offload.caps.size_reporting = true;
	tcm->offload.caps.rotation_reporting = true;
	tcm->offload.caps.filter_grip = true;
	tcm->offload.caps.filter_palm = true;
	tcm->offload.caps.num_sensitivity_settings = 1;

	tcm->offload.hcallback = (void *)tcm;
	tcm->offload.report_cb = syna_offload_report;
	touch_offload_init(&tcm->offload);

	if (syna_heatmap_alloc_check(tcm) != 0)
		goto err_connect;

	tcm->touch_offload_active_coords = 0;
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	/*
	 * Heatmap_probe must be called before irq routine is registered,
	 * because heatmap_read is called from interrupt context.
	 */
	tcm->v4l2.parent_dev = &tcm->pdev->dev;
	tcm->v4l2.input_dev = tcm->input_dev;
	tcm->v4l2.read_frame = v4l2_read_frame;
	tcm->v4l2.width = tcm->tcm_dev->rows;
	tcm->v4l2.height = tcm->tcm_dev->cols;
	/* 240 Hz operation */
	tcm->v4l2.timeperframe.numerator = 1;
	tcm->v4l2.timeperframe.denominator = 240;
	retval = heatmap_probe(&tcm->v4l2);
	if (retval < 0)
		goto err_heatmap_probe;
#endif

	/* init motion filter mode */
	tcm->mf_mode = MF_DYNAMIC;

	INIT_WORK(&tcm->motion_filter_work, syna_motion_filter_work);
	INIT_WORK(&tcm->set_grip_mode_work, syna_set_grip_mode_work);
	INIT_WORK(&tcm->set_palm_mode_work, syna_set_palm_mode_work);

	tcm->touch_report_rate_config = CONFIG_HIGH_REPORT_RATE;
	INIT_DELAYED_WORK(&tcm->set_report_rate_work, syna_set_report_rate_work);

	retval = syna_dev_request_irq(tcm);
	if (retval < 0) {
		LOGE("Fail to request the interrupt line\n");
		goto err_request_irq;
	}

	tcm->enable_fw_grip = 0x00;
	tcm->enable_fw_palm = 0x01;
	syna_dev_restore_feature_setting(tcm, RESP_IN_POLLING);

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

#ifdef HAS_SYSFS_INTERFACE
	/* create the device file and register to char device classes */
	retval = syna_cdev_create_sysfs(tcm, pdev);
	if (retval < 0) {
		LOGE("Fail to create the device sysfs\n");
		syna_pal_mutex_free(&tcm->tp_event_mutex);
		goto err_create_cdev;
	}
#endif

#if defined(USE_DRM_BRIDGE)
	retval = syna_register_panel_bridge(tcm);
#elif defined(ENABLE_DISP_NOTIFIER)
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

	LOGI("%s TouchComm driver v%d.%s installed\n",
		PLATFORM_DRIVER_NAME,
		SYNAPTICS_TCM_DRIVER_VERSION,
		SYNAPTICS_TCM_DRIVER_SUBVER);

	return 0;

#ifdef HAS_SYSFS_INTERFACE
err_create_cdev:
	syna_tcm_remove_device(tcm->tcm_dev);
#ifdef STARTUP_REFLASH
	cancel_delayed_work_sync(&tcm->reflash_work);
	flush_workqueue(tcm->reflash_workqueue);
	destroy_workqueue(tcm->reflash_workqueue);
#endif
	/* free interrupt line */
	if (tcm->hw_if->bdata_attn.irq_id)
		syna_dev_release_irq(tcm);
#endif
err_request_irq:
#if defined(TCM_CONNECT_IN_PROBE)
	tcm->dev_disconnect(tcm);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	heatmap_remove(&tcm->v4l2);
err_heatmap_probe:
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	touch_offload_cleanup(&tcm->offload);
#endif

err_connect:
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (tcm->tbn_register_mask)
		unregister_tbn(&tcm->tbn_register_mask);
#endif
	cpu_latency_qos_remove_request(&tcm->pm_qos_req);

	if (tcm->event_wq)
		destroy_workqueue(tcm->event_wq);
err_alloc_workqueue:
	syna_tcm_buf_release(&tcm->event_data);
	syna_pal_mutex_free(&tcm->tp_event_mutex);
err_allocate_cdev:
	syna_pal_mem_free((void *)tcm);

	return retval;
}

/**
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

	cancel_work_sync(&tcm->suspend_work);
	cancel_work_sync(&tcm->resume_work);
	cancel_work_sync(&tcm->motion_filter_work);
	cancel_work_sync(&tcm->set_grip_mode_work);
	cancel_work_sync(&tcm->set_palm_mode_work);
	cancel_delayed_work_sync(&tcm->set_report_rate_work);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (tcm->tbn_register_mask)
		unregister_tbn(&tcm->tbn_register_mask);
#endif

#if defined(USE_DRM_BRIDGE)
	syna_unregister_panel_bridge(&tcm->panel_bridge);
#elif defined(ENABLE_DISP_NOTIFIER)
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
	syna_cdev_remove_sysfs(tcm);
#endif

#ifdef STARTUP_REFLASH
	cancel_delayed_work_sync(&tcm->reflash_work);
	flush_workqueue(tcm->reflash_workqueue);
	destroy_workqueue(tcm->reflash_workqueue);
#endif

	/* free interrupt line */
	if (tcm->hw_if->bdata_attn.irq_id)
		syna_dev_release_irq(tcm);

	/* check the connection statusm, and do disconnection */
	if (tcm->dev_disconnect(tcm) < 0)
		LOGE("Fail to do device disconnection\n");

	cpu_latency_qos_remove_request(&tcm->pm_qos_req);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	touch_offload_cleanup(&tcm->offload);
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	heatmap_remove(&tcm->v4l2);
#endif

	if (tcm->raw_data_buffer) {
		kfree(tcm->raw_data_buffer);
		tcm->raw_data_buffer = NULL;
	}

	syna_tcm_buf_release(&tcm->event_data);

	syna_pal_mutex_free(&tcm->tp_event_mutex);

	/* remove the allocated tcm device */
	syna_tcm_remove_device(tcm->tcm_dev);

	/* release the device context */
	syna_pal_mem_free((void *)tcm);

	return 0;
}

/**
 * syna_dev_shutdown()
 *
 * Call syna_dev_remove() to release all resources
 *
 * @param
 *    [in] pdev: an instance of platform device
 *
 * @return
 *    none.
 */
static void syna_dev_shutdown(struct platform_device *pdev)
{
	syna_dev_remove(pdev);
}

/**
 * Declare a TouchComm platform device
 */
#ifdef CONFIG_PM
#if defined(USE_DRM_BRIDGE)
static int syna_pm_suspend(struct device *dev)
{
	struct syna_tcm *tcm = dev_get_drvdata(dev);

	if (tcm->bus_refmask)
		LOGW("bus_refmask 0x%X\n", tcm->bus_refmask);

	if (tcm->pwr_state == PWR_ON) {
		LOGW("can't suspend because touch bus is in use!\n");
		if (tcm->bus_refmask == SYNA_BUS_REF_BUGREPORT &&
		    ktime_ms_delta(ktime_get(), tcm->bugreport_ktime_start) > 30 * MSEC_PER_SEC) {
			syna_set_bus_ref(tcm, SYNA_BUS_REF_BUGREPORT, false);
			pm_relax(&tcm->pdev->dev);
			tcm->bugreport_ktime_start = 0;
			LOGE("Force release SYNA_BUS_REF_BUGREPORT reference bit.");
		}
		return -EBUSY;
	}

	return 0;
}

static int syna_pm_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops syna_dev_pm_ops = {
#if defined(USE_DRM_BRIDGE)
	.suspend = syna_pm_suspend,
	.resume = syna_pm_resume,
#elif !defined(ENABLE_DISP_NOTIFIER)
	.suspend = syna_dev_suspend,
	.resume = syna_dev_resume,
#endif
};
#endif

static struct platform_driver syna_dev_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &syna_dev_pm_ops,
#endif
	},
	.probe = syna_dev_probe,
	.remove = syna_dev_remove,
	.shutdown = syna_dev_shutdown,
};


/**
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

/**
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

