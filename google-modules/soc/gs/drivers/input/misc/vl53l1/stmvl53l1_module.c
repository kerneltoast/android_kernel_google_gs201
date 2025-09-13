// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file /stmvl53l1_module.c  vl53l1_module  ST VL53L1 linux kernel module
 *
 * main file
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/jhash.h>
#include <linux/ctype.h>

/*
 * API includes
 */

#include "stmvl53l1.h"
#include "stmvl53l1-i2c.h"

#include "stmvl53l1_if.h" /* our device interface to user space */
#include "stmvl53l1_internal_if.h"

/*
 * include default tuning file
 */
#include "stmvl53l1_tunings.h"

/** @ingroup vl53l1_config
 * @{
 */
/**
 * default polling period delay in millisecond
 *
 * It can be set at run time via @ref vl53l1_ioctl or @ref sysfs_attrib
 *
 * @note apply only for device operating in polling mode only
 */
#define STMVL53L1_CFG_POLL_DELAY_MS 5

/**
 * default timing budget in microsecond
 *
 * Can be change at run time via @ref vl53l1_ioctl or @ref sysfs_attrib
 */
#define STMVL53L1_CFG_TIMING_BUDGET_US 16000

/** default preset ranging mode */
#define STMVL53L1_CFG_DEFAULT_MODE VL53L1_PRESETMODE_RANGING

/** default distance mode */
#define STMVL53L1_CFG_DEFAULT_DISTANCE_MODE VL53L1_DISTANCEMODE_LONG

/** default crosstalk enable */
#define STMVL53L1_CFG_DEFAULT_CROSSTALK_ENABLE 0

/** default output mode */
#define STMVL53L1_CFG_DEFAULT_OUTPUT_MODE VL53L1_OUTPUTMODE_NEAREST

#define STMVL53L1_CFG_DEFAULT_OFFSET_CORRECTION_MODE \
	VL53L1_OFFSETCORRECTIONMODE_STANDARD

/** default Dmax mode */
#define STMVL53L1_CFG_DEFAULT_DMAX_MODE VL53L1_DMAXMODE_FMT_CAL_DATA

/** default smudge correction enable value */
#define STMVL53L1_CFG_DEFAULT_SMUDGE_CORRECTION_MODE \
	VL53L1_SMUDGE_CORRECTION_NONE

/** @} */ /* ingroup vl53l1_config */

/** @ingroup vl53l1_mod_dbg
 * @{
 */

/**
 * activate dump of roi in roi ctrl operation
 *
 * @note uses @a dev_dbg for output so make sure to enable debug
 * to get roi dump
 */
#define STMVL53L1_CFG_ROI_DEBUG 0

/** @} */ /* ingroup vl53l1_mod_dbg*/

static inline void st_gettimeofday(struct timespec64 *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_nsec = now.tv_nsec;
}

/* #define DEBUG_TIME_LOG */
#ifdef DEBUG_TIME_LOG
struct timespec64 start_tv, stop_tv;
#endif

/* Set default value to 1 to allow to see module insertion debug messages */
int stmvl53l1_enable_debug = 1;

#define VL53L1_INPUT_DEVICE_NAME "STM VL53L1 proximity sensor"

static long stmvl53l1_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg);
static int stmvl53l1_open(struct inode *inode, struct file *file);
static int stmvl53l1_release(struct inode *inode, struct file *file);
static int ctrl_start(struct stmvl53l1_data *data);
static int ctrl_stop(struct stmvl53l1_data *data);

static bool force_device_on_en_default = true;

module_param(force_device_on_en_default, bool, 0444);
MODULE_PARM_DESC(force_device_on_en_default,
	"select whether force_device_on_en is true or false by default");

/* boilerplate for integer parameter */
#define IMPLEMENT_PARAMETER_INTEGER(sysfs_name, info_name) \
static ssize_t sysfs_name##_show(struct device *dev, \
				 struct device_attribute *attr, char *buf) \
{ \
	struct stmvl53l1_data *data = dev_get_drvdata(dev); \
	int param; \
\
	mutex_lock(&data->work_mutex); \
	param = data->sysfs_name; \
	mutex_unlock(&data->work_mutex); \
\
	return sysfs_emit(buf, "%d\n", param); \
} \
\
static ssize_t sysfs_name##_store(struct device *dev, \
				  struct device_attribute *attr, \
				  const char *buf, size_t count) \
{ \
	struct stmvl53l1_data *data = dev_get_drvdata(dev); \
	int rc; \
	int param; \
\
	mutex_lock(&data->work_mutex); \
\
	if (kstrtoint(buf, 0, &param)) \
		rc = -EINVAL; \
	else \
		rc = stmvl53l1_set_##sysfs_name(data, param); \
\
	mutex_unlock(&data->work_mutex); \
\
	return rc ? rc : count; \
} \
\
static int ctrl_param_##sysfs_name(struct stmvl53l1_data *data, \
				   struct stmvl53l1_parameter *param) \
{ \
	int rc; \
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object; \
	struct device *dev = &i2c_data->client->dev; \
\
	if (param->is_read) { \
		param->value = data->sysfs_name; \
		param->status = 0; \
		dev_dbg(dev, "get " info_name " %d", param->value); \
		rc = 0; \
	} else { \
		rc = stmvl53l1_set_##sysfs_name(data, param->value); \
		dev_dbg(dev, "rc %d req %d now %d", rc, \
			param->value, data->sysfs_name); \
	} \
\
	return rc; \
}

/**
 *  module interface struct
 *  interface to platform speficic device handling , concern power/reset ...
 */
struct stmvl53l1_module_fn_t {
	int (*init)(void); /*!< init */
	/**
	 * clean up job
	 * @param data module specific data ptr
	 */
	void (*deinit)(void *data);
	/**
	 *  give device power
	 * @param data  specific module storage ptr
	 * @return 0 on success
	 */
	int (*power_up)(void *data);
	/**
	 *  power down TOFO also stop intr
	 */
	int (*power_down)(void *data);
	/*
	 * release reset so device start.
	 */
	int (*reset_release)(void *data);
	/*
	 * put device under reset.
	 */
	int (*reset_hold)(void *data);

	 /**
	  * enable interrupt
	  *
	  * @param object : interface speficic ptr
	  * @note "module specific ptr is data->client_object
	  * @return 0 on success else error then drievr wan't start ranging!
	  * if no interrupt or it can't be hooked but still to operated in poll
	  * mode then return 0  and force data->poll_mode
	  * might have to clear poll_mode exlplcilty if to operate in real intr
	  * mode as pool mode
	  * is the default
	  */
	int (*start_intr)(void *object, int *poll_mode);

	void (*clean_up)(void); /*!< optional can be void */

	/* increment reference counter */
	void *(*get)(void *object);

	/* decrement reference counter and deallocate memory when zero */
	int (*put)(void *object);
};

/** i2c module interface*/
static struct stmvl53l1_module_fn_t stmvl53l1_module_func_tbl = {
	.init = stmvl53l1_init_i2c,
	.deinit = stmvl53l1_exit_i2c,
	.power_up = stmvl53l1_power_up_i2c,
	.power_down = stmvl53l1_power_down_i2c,
	.reset_release = stmvl53l1_reset_release_i2c,
	.reset_hold = stmvl53l1_reset_hold_i2c,
	.clean_up = stmvl53l1_clean_up_i2c,
	.start_intr = stmvl53l1_start_intr,
	.get = stmvl53l1_get,
	.put = stmvl53l1_put,
};

/*
 * INPUT Subsys interface
 */

static void stmvl53l1_input_push_data(struct stmvl53l1_data *data);

/*
 * Mutex to handle device id add/removal
 */
static DEFINE_MUTEX(dev_table_mutex);

/*
 * Mutex to handle device open/release
 */
static DEFINE_MUTEX(dev_open_mutex);

/**
 * in-used device LUT
 * we need this as the message reception from netlink message can't
 * associate directly to a device instance that is as we look up id
 * to device data structure
 */
struct stmvl53l1_data *stmvl53l1_dev_table[STMVL53L1_CFG_MAX_DEV];

/**
 * Misc device operations
 */
static const struct file_operations stmvl53l1_ranging_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = stmvl53l1_ioctl,
	.open = stmvl53l1_open,
	.release = stmvl53l1_release,
	/* .flush = stmvl53l0_flush, */
};

static int store_last_error(struct stmvl53l1_data *data, int rc)
{
	data->last_error = rc;

	return -EIO;
}

static int allocate_dev_id(void)
{
	int i;

	mutex_lock(&dev_table_mutex);

	for (i = 0; i < STMVL53L1_CFG_MAX_DEV; i++)
		if (!stmvl53l1_dev_table[i])
			break;
	i = i < STMVL53L1_CFG_MAX_DEV ? i : -1;

	mutex_unlock(&dev_table_mutex);

	return i;
}

static void deallocate_dev_id(int id)
{
	mutex_lock(&dev_table_mutex);

	stmvl53l1_dev_table[id] = NULL;

	mutex_unlock(&dev_table_mutex);
}

/* helpers to manage reader list for blockint ioctl */
/* call them with lock */
static void empty_and_free_list(struct list_head *head)
{
	struct stmvl53l1_waiters *waiter;
	struct stmvl53l1_waiters *tmp;

	list_for_each_entry_safe(waiter, tmp, head, list) {
		list_del(&waiter->list);
		kfree(waiter);
	}
}

static int add_reader(pid_t pid, struct list_head *head)
{
	struct stmvl53l1_waiters *new_waiter;

	new_waiter = kmalloc(sizeof(struct stmvl53l1_waiters), GFP_KERNEL);
	if (!new_waiter)
		return -ENOMEM;
	new_waiter->pid = pid;
	list_add(&new_waiter->list, head);

	return 0;
}

static bool is_pid_in_list(pid_t pid, struct list_head *head)
{
	struct stmvl53l1_waiters *waiter;

	list_for_each_entry(waiter, head, list)
		if (waiter->pid == pid)
			return true;

	return false;
}

static void wake_up_data_waiters(struct stmvl53l1_data *data)
{
	empty_and_free_list(&data->simple_data_reader_list);
	empty_and_free_list(&data->mz_data_reader_list);
	wake_up(&data->waiter_for_data);
}

static void stmvl53l1_insert_flush_events_lock(struct stmvl53l1_data *data)
{
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	while (data->flush_todo_counter) {
		data->flushCount++;
		input_report_abs(data->input_dev_ps, ABS_GAS, data->flushCount);
		input_sync(data->input_dev_ps);
		dev_dbg(dev, "Sensor HAL Flush Count = %u\n", data->flushCount);
		data->flush_todo_counter--;
	}
}

static int reset_release(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (!data->reset_state)
		return 0;

	rc = stmvl53l1_module_func_tbl.reset_release(data->client_object);
	if (rc)
		dev_err(dev, "reset release fail rc = %d\n", rc);
	else
		data->reset_state = 0;

	return rc;
}

static int reset_hold(struct stmvl53l1_data *data)
{
	int rc;

	if (data->reset_state)
		return 0;

	if (data->force_device_on_enable)
		return 0;

	rc = stmvl53l1_module_func_tbl.reset_hold(data->client_object);
	if (!rc)
		data->reset_state = 1;

	return rc;
}

#ifdef DEBUG_TIME_LOG
static void stmvl53l0_DebugTimeGet(struct timespec64 *ptv)
{
	st_gettimeofday(ptv);
}
#endif

/**
 *
 * @param pstart_tv time val  starting point
 * @param pstop_tv time val  end point
 * @return time dif in usec
 */
long stmvl53l1_tv_dif(struct timespec64 *pstart_tv, struct timespec64 *pstop_tv)
{
	long total_sec, total_nsec;

	total_sec = pstop_tv->tv_sec - pstart_tv->tv_sec;
	total_nsec = (pstop_tv->tv_nsec - pstart_tv->tv_nsec);

	return total_sec * 1000000 + total_nsec / 1000;
}

#if STMVL53L1_CFG_ROI_DEBUG
static void dump_roi(struct device *dev, struct VL53L1_UserRoi_t *rois,
		     uint32_t n)
{
	uint32_t i;

	dev_dbg(dev, "roi dump %d roi:\n", n);
	for (i = 0; i < n; i++) {
		dev_dbg(dev, "ROI#%02d %2d %2d %2d %2d\n", (int)i,
			(int)rois[i].TopLeftX, (int)rois[i].TopLeftY,
			(int)rois[i].BotRightX, (int)rois[i].BotRightY);
	}
}
#else
#define dump_roi(...) (void)0
#endif

static int setup_tunings(struct stmvl53l1_data *data)
{
	int rc = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(tunings); i++) {
		rc = VL53L1_SetTuningParameter(&data->stdev, tunings[i][0],
			tunings[i][1]);
		if (rc) {
			rc = store_last_error(data, rc);
			break;
		}
	}

	return rc;
}

/**
 *
 * @param data device data
 * @return non 0 if current "preset mode" is a multi zone one
 */
static int is_mz_mode(struct stmvl53l1_data *data)
{
	return data->preset_mode == VL53L1_PRESETMODE_RANGING ||
		data->preset_mode == VL53L1_PRESETMODE_MULTIZONES_SCANNING;
}

static void kill_mz_data(struct VL53L1_MultiRangingData_t *pdata)
{
	int i;

	memset(pdata, 0, sizeof(*pdata));
	for (i = 0; i < VL53L1_MAX_RANGE_RESULTS; i++)
		pdata->RangeData[i].RangeStatus = VL53L1_RANGESTATUS_NONE;
	pdata->RoiStatus = VL53L1_ROISTATUS_NOT_VALID;
}

static void stmvl53l1_setup_auto_config(struct stmvl53l1_data *data)
{
	/* default config is detect object below 300mm with 1s period */
	data->auto_pollingTimeInMs = 1000;
	data->auto_config.DetectionMode = VL53L1_DETECTION_DISTANCE_ONLY;
	data->auto_config.IntrNoTarget = 0;
	data->auto_config.Distance.CrossMode = VL53L1_THRESHOLD_CROSSED_LOW;
	data->auto_config.Distance.High = 1000;
	data->auto_config.Distance.Low = 300;
	data->auto_config.Rate.CrossMode = VL53L1_THRESHOLD_CROSSED_LOW;
	data->auto_config.Rate.High = 0;
	data->auto_config.Rate.Low = 0;
}

static uint32_t stmvl53l1_compute_hash(struct VL53L1_RoiConfig_t *roi_cfg)
{
	return jhash(roi_cfg->UserRois,
		roi_cfg->NumberOfRoi * sizeof(struct VL53L1_UserRoi_t),
		roi_cfg->NumberOfRoi);
}

static int stmvl53l1_check_calibration_id(struct stmvl53l1_data *data)
{
	uint32_t roi_id;
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->offset_correction_mode != VL53L1_OFFSETCORRECTIONMODE_PERZONE)
		return 0;
	if (data->current_roi_id == 0)
		return 0;

	roi_id = stmvl53l1_compute_hash(&data->roi_cfg);
	rc = roi_id == data->current_roi_id ? 0 : -EINVAL;

	if (rc)
		dev_err(dev, "Mismatch in zone cal data 0x%08x != 0x%08x\n",
			roi_id, data->current_roi_id);

	return rc;
}

/**
 * send params to sensor
 *
 * @warning must be used if only stopped
 * @param data device data
 * @return 0 on success
 */
static int stmvl53l1_sendparams(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	/* activated  stored or last request defined mode */
	rc = VL53L1_SetPresetMode(&data->stdev, data->preset_mode);
	if (rc) {
		dev_err(dev, "VL53L1_SetPresetMode %d fail %d\n",
			data->preset_mode, rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	rc = VL53L1_SetXTalkCompensationEnable(&data->stdev,
		data->crosstalk_enable);
	if (rc) {
		dev_err(dev, "VL53L1_SetXTalkCompensationEn %d fail %d\n",
			data->crosstalk_enable, rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	/* apply distance mode only in lite and standard ranging */
	rc = VL53L1_SetDistanceMode(&data->stdev, data->distance_mode);
	if (rc) {
		dev_err(dev, "VL53L1_SetDistanceMode %d fail %d\n",
			data->distance_mode, rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	/* apply timing budget */
	rc = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&data->stdev,
			data->timing_budget);
	if (rc) {
		dev_err(dev, "SetTimingBudget %d fail %d\n",
			data->timing_budget, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	dev_dbg(dev, "timing budget @%d\n", data->timing_budget);

	/* apply offset correction mode */
	rc = VL53L1_SetOffsetCorrectionMode(&data->stdev,
			data->offset_correction_mode);
	if (rc) {
		dev_err(dev, "offset correction mode %d fail %d\n",
			data->offset_correction_mode, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	dev_dbg(dev, "offset correction mode @%d\n",
		data->offset_correction_mode);

	/* check zone calibration vs roi */
	rc = stmvl53l1_check_calibration_id(data);
	if (rc)
		goto done;

	/* apply Dmax reflectance */
	rc = VL53L1_SetDmaxReflectance(&data->stdev, data->dmax_reflectance);
	if (rc) {
		dev_err(dev, "dmax relectance %d fail %d\n",
			data->dmax_reflectance, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	dev_dbg(dev, "dmax reflectance @%d\n", data->dmax_reflectance);

	/* apply Dmax mode */
	rc = VL53L1_SetDmaxMode(&data->stdev, data->dmax_mode);
	if (rc) {
		dev_err(dev, "dmax mode %d fail %d\n",
			data->dmax_mode, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	dev_dbg(dev, "dmax mode @%d\n", data->dmax_mode);

	/* apply smudge correction enable */
	rc = VL53L1_SmudgeCorrectionEnable(&data->stdev,
		data->smudge_correction_mode);
	if (rc) {
		dev_err(dev, "smudge correction mode %d fail %d\n",
			data->smudge_correction_mode, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	dev_dbg(dev, "smudge correction mode @%d\n",
		data->smudge_correction_mode);

	/* apply roi if any set */
	if (data->roi_cfg.NumberOfRoi) {
		rc = VL53L1_SetROI(&data->stdev, &data->roi_cfg);
		if (rc) {
			dev_err(dev, "VL53L1_SetROI fail %d\n", rc);
			rc = store_last_error(data, rc);
			goto done;
		}
		dev_dbg(dev, "#%d custom ROI set status\n",
			data->roi_cfg.NumberOfRoi);
	} else {
		dev_dbg(dev, "using default ROI\n");
	}

	/* set autonomous mode configuration */
	if ((data->preset_mode == VL53L1_PRESETMODE_AUTONOMOUS) ||
		(data->preset_mode == VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS)) {
		rc = VL53L1_SetInterMeasurementPeriodMilliSeconds(&data->stdev,
			data->auto_pollingTimeInMs);
		if (rc) {
			dev_err(dev, "Fail to set auto period %d\n", rc);
			rc = store_last_error(data, rc);
			goto done;
		}
		rc = VL53L1_SetThresholdConfig(&data->stdev,
			&data->auto_config);
		if (rc) {
			dev_err(dev, "Fail to set auto config %d\n", rc);
			rc = store_last_error(data, rc);
			goto done;
		}
	}

done:

	return rc;
}

/**
 * start sensor
 *
 * @warning must be used if only stopped
 * @param data device data
 * @return 0 on success
 */
static int stmvl53l1_start(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	data->is_first_irq = true;
	data->is_data_valid = false;
	data->is_xtalk_value_changed = false;

	rc = reset_release(data);
	if (rc)
		goto done;

	if (data->ulp_enable) {
		rc = VL53L1_ULP_SensorInit(&data->stdev);
		if (rc)
			goto done;
	} else {
		/* full setup when out of reset or power up */
		rc = VL53L1_StaticInit(&data->stdev);
		if (rc) {
			dev_err(dev, "VL53L1_StaticInit @%d fail %d\n",
				__LINE__, rc);
			rc = store_last_error(data, rc);
			goto done;
		}
		rc = stmvl53l1_sendparams(data);
		if (rc)
			goto done;
	}

	/* init the timing  */
	st_gettimeofday(&data->start_tv);
	data->meas.start_tv = data->start_tv;
	/* init the ranging data => kill the previous ranging mz data */
	kill_mz_data(&data->meas.multi_range_data);
	/* kill the single ranging data */
	memset(&data->meas.single_range_data, 0,
			sizeof(struct VL53L1_RangingMeasurementData_t));

	data->allow_hidden_start_stop = false;
	/* kick off ranging */
	if (data->ulp_enable)
		rc = VL53L1_ULP_StartRanging(&data->stdev);
	else
		rc = VL53L1_StartMeasurement(&data->stdev);
	if (rc) {
		dev_err(dev, "VL53L1_StartMeasurement @%d fail %d",
			__LINE__, rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	data->meas.cnt = 0;
	data->meas.err_cnt = 0;
	data->meas.err_tot = 0;
	data->meas.poll_cnt = 0;
	data->meas.intr = 0;
	data->enable_sensor = 1;
	if (data->poll_mode) {
		/* kick off the periodical polling work */
		schedule_delayed_work(&data->dwork,
			msecs_to_jiffies(data->poll_delay_ms));
	}
done:
	data->is_first_start_done = true;

	return rc;
}

/**
 * stop sensor
 *
 * work lock must be held
 * @warning to be used if only started!
 */
static int stmvl53l1_stop(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->ulp_enable)
		rc = VL53L1_ULP_StopRanging(&data->stdev);
	else
		rc = VL53L1_StopMeasurement(&data->stdev);
	if (rc) {
		dev_err(dev, "VL53L1_StopMeasurement @%d fail %d\n",
			__LINE__, rc);
		rc = store_last_error(data, rc);
	}

	data->enable_sensor = 0;
	if (data->poll_mode) {
		/* cancel periodical polling work */
		cancel_delayed_work(&data->dwork);
	}

	/* wake up all waiters */
	/* they will receive -ENODEV error */
	wake_up_data_waiters(data);

	return rc;
}

/*
 * SysFS support
 */
static ssize_t enable_ps_sensor_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", data->enable_sensor);
}

static ssize_t enable_ps_sensor_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int rc = 0;
	unsigned long val;

	rc = kstrtoul(buf, 10, &val);
	if (rc) {
		dev_err(dev, "enable sensor syntax in %s\n", buf);
		return -EINVAL;
	}
	if (val == 1) {
		rc = ctrl_start(data);
	} else if (val == 0) {
		rc = ctrl_stop(data);
	} else {
		/* TODO: Remove this workaround after investigation
		 * see Codex - 479397 for details
		 */
		dev_dbg(dev, "Unclog Input sub-system\n");
		/* Unclog the input device sub-system */
		input_report_abs(data->input_dev_ps, ABS_HAT0X, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT0Y, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT1X, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT1Y, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT2X, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT2Y, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT3X, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT3Y, -1);
		input_report_abs(data->input_dev_ps, ABS_WHEEL, -1);
		input_report_abs(data->input_dev_ps, ABS_BRAKE, -1);
		input_report_abs(data->input_dev_ps, ABS_GAS, -1);
		input_report_abs(data->input_dev_ps, ABS_TILT_X, -1);
		input_report_abs(data->input_dev_ps, ABS_TILT_Y, -1);
		input_report_abs(data->input_dev_ps, ABS_TOOL_WIDTH, -1);
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, -1);
		input_report_abs(data->input_dev_ps, ABS_THROTTLE, -1);
		input_report_abs(data->input_dev_ps, ABS_RUDDER, -1);
		input_report_abs(data->input_dev_ps, ABS_MISC, -1);
		input_report_abs(data->input_dev_ps, ABS_VOLUME, -1);
		input_sync(data->input_dev_ps);
		dev_dbg(dev, "Unclog the input sub-system\n");
		rc = 0;
	}

	return rc ? rc : count;
}

static ssize_t product_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int param;

	mutex_lock(&data->work_mutex);
	param = data->product_type;
	mutex_unlock(&data->work_mutex);

	return sysfs_emit(buf, "%d\n", param);
}

/**
 * sysfs attribute "product_type" [rd]
 *
 * @li read show the device product type
 *
 * @return  0 on success , EINVAL if fail to start
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RO(product_type);

/**
 * sysfs attribute "enable_ps_sensor" [rd/wr]
 *
 * @li read show the current enable state
 * @li write set the new state value "0" put sensor off "1"  put it on
 *
 * @return  0 on success , EINVAL if fail to start
 *
 * @warning their's no check and assume exclusive usage of sysfs and ioctl\n
 * Sensor will be put on/off disregard of any setup done by the ioctl channel.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(enable_ps_sensor);

static int stmvl53l1_set_poll_delay_ms(struct stmvl53l1_data *data, int delay)
{
	int rc = 0;

	if (delay <= 0)
		rc = -EINVAL;
	else
		data->poll_delay_ms = delay;

	return rc;
}

static ssize_t set_delay_ms_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int param;

	mutex_lock(&data->work_mutex);
	param = data->poll_delay_ms;
	mutex_unlock(&data->work_mutex);

	return sysfs_emit(buf, "%d\n", param);
}

static ssize_t set_delay_ms_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int rc;
	int param;

	mutex_lock(&data->work_mutex);

	if (kstrtoint(buf, 0, &param))
		rc = -EINVAL;
	else
		rc = stmvl53l1_set_poll_delay_ms(data, param);

	mutex_unlock(&data->work_mutex);

	return rc ? rc : count;
}

static int ctrl_param_poll_delay_ms(struct stmvl53l1_data *data,
				   struct stmvl53l1_parameter *param)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (param->is_read) {
		param->value = data->poll_delay_ms;
		param->status = 0;
		dev_dbg(dev, "get poll delay ms %d", param->value);
		rc = 0;
	} else {
		rc = stmvl53l1_set_poll_delay_ms(data, param->value);
		dev_dbg(dev, "rc %d req %d now %d", rc,
			param->value, data->poll_delay_ms);
	}

	return rc;
}

/**
 * sysfs attribute "set_delay_ms" [rd/wr]
 *
 * @li read show the current polling delay in millisecond
 * @li write set the new polling delay in millisecond
 *
 * @note apply only if device is in polling mode\n
 * for best performances (minimal delay and cpu load ) set it to the device
 * period operating period +1 millis

 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(set_delay_ms);

static int stmvl53l1_set_timing_budget(struct stmvl53l1_data *data, int timing)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (timing <= 0) {
		dev_err(dev, "invalid timing valid %d\n", timing);
		rc = -EINVAL;
	} else if (data->enable_sensor) {
		rc = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&data->stdev,
			timing);
		if (rc) {
			dev_err(dev, "SetTimingBudget %d fail %d\n",
				timing, rc);
			rc = store_last_error(data, rc);
		} else
			data->timing_budget = timing;
	} else
		data->timing_budget = timing;

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(timing_budget, "timing budget")

/**
 * sysfs "timing_budget"  [rd/wr]
 *
 *  set or get the ranging timing budget in microsecond
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(timing_budget);

static ssize_t roi_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int i;
	int n;


	mutex_lock(&data->work_mutex);
	if (data->roi_cfg.NumberOfRoi == 0) {
		/* none define by user */
		/* we could get what stored but may not even be default */
		n = sysfs_emit(buf, "device default\n");
	} else {
		for (i = 0, n = 0; i < data->roi_cfg.NumberOfRoi; i++) {
			n += sysfs_emit_at(buf, n, "%d %d %d %d%c",
					   data->roi_cfg.UserRois[i].TopLeftX,
					   data->roi_cfg.UserRois[i].TopLeftY,
					   data->roi_cfg.UserRois[i].BotRightX,
					   data->roi_cfg.UserRois[i].BotRightY,
					   i == data->roi_cfg.NumberOfRoi-1 ?
					   '\n' : ',');
		}
	}
	mutex_unlock(&data->work_mutex);
	return n;
}

static const char str_roi_ranging[] = "ERROR can't set roi while ranging\n";

static ssize_t roi_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	struct VL53L1_UserRoi_t rois[VL53L1_MAX_USER_ZONES];
	int rc;

	mutex_lock(&data->work_mutex);
	if (data->enable_sensor) {
		dev_err(dev, "can't set roi now\n");
		rc = -EBUSY;
	} else {
		int n, n_roi = 0;
		const char *pc = buf;
		int tlx, tly, brx, bry;

		while (n_roi < VL53L1_MAX_USER_ZONES && pc != NULL &&
		       *pc != 0 && *pc != '\n') {
			n = sscanf(pc, "%d %d %d %d", &tlx, &tly, &brx, &bry);
			if (n == 4) {
				rois[n_roi].TopLeftX = tlx;
				rois[n_roi].TopLeftY = tly;
				rois[n_roi].BotRightX = brx;
				rois[n_roi].BotRightY = bry;
				n_roi++;
			} else {
				dev_err(dev, "wrong roi #%d syntax\n", n_roi);
				dev_err(dev, "around %s of %s\n", pc, buf);
				n_roi = -1;
				break;
			}
			/* find next roi separator */
			pc = strchr(pc, ',');
			if (pc)
				pc++;
		}
		/*if any set them */
		if (n_roi >= 0) {
			if (n_roi)
				memcpy(data->roi_cfg.UserRois, rois,
						n_roi*sizeof(rois[0]));
			data->roi_cfg.NumberOfRoi = n_roi;
			dump_roi(dev, data->roi_cfg.UserRois,
				 data->roi_cfg.NumberOfRoi);
			rc = count;
		} else {
			rc = -EINVAL;
		}
	}
	mutex_unlock(&data->work_mutex);
	dev_dbg(dev, "ret %d count %d\n", rc, (int)count);

	return rc;
}

/**
 * sysfs attribute "roi" [rd/wr]
 *
 * @li read show the current user customized roi setting
 * @li write set user custom roi, it can only be done while not ranging.
 *
 * syntax for set input roi
 * @li "[tlx tly brx bry,]\n" repeat n time require will set the n roi
 * @li "\n" will reset
 *
 * @warning roi coordinate is not image x,y(down) but euclidian x,y(up)
 *
 * @warning roi validity is only check at next range start
 * @warning user is responsible to set appropriate number an roi before each
 * mode change
 * @note roi can be return to default by setting none ""
 *
 *@code
 * >#to set 2x roi
 * >echo "0 15 15 0, 0 8 8 0" > /sys/class/input6/roi
 * >echo $?
 * 0
 * >cat /sys/class/input6/roi
 * "0 15 15 0,0 8 8 0"
 * #to cancel user define roi"
 * >echo "" > /sys/class/input1/roi
 * >echo $?
 * 0
 * >echo "1" > /sys/class/input6/enable_ps_senspor
 * #try to set roi while ranging
 * >echo "0 15 15 0, 0 8 8 0" > /sys/class/input6/roi
 * [58451.912109] stmvl53l1_store_roi:  can't set roi now
 * >echo $?
 * 1
 *@endcode
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(roi);

static int stmvl53l1_set_preset_mode(struct stmvl53l1_data *data, int mode)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev, "can't change mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (mode) {
		case VL53L1_PRESETMODE_RANGING:
		case VL53L1_PRESETMODE_MULTIZONES_SCANNING:
		case VL53L1_PRESETMODE_LITE_RANGING:
		case VL53L1_PRESETMODE_AUTONOMOUS:
		case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
			data->preset_mode = mode;
			break;
		default:
			dev_err(dev, "invalid mode %d\n", mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

static ssize_t mode_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int param;

	mutex_lock(&data->work_mutex);
	param = data->preset_mode;
	mutex_unlock(&data->work_mutex);

	return sysfs_emit(buf, "%d\n", param);
}

static ssize_t mode_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int rc;
	int param;

	mutex_lock(&data->work_mutex);

	if (kstrtoint(buf, 0, &param))
		rc = -EINVAL;
	else
		rc = stmvl53l1_set_preset_mode(data, param);

	mutex_unlock(&data->work_mutex);

	return rc ? rc : count;
}

static int ctrl_param_preset_mode(struct stmvl53l1_data *data,
				  struct stmvl53l1_parameter *param)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (param->is_read) {
		param->value = data->preset_mode;
		param->status = 0;
		dev_dbg(dev, "get preset_mode %d", param->value);
		rc = 0;
	} else {
		rc = stmvl53l1_set_preset_mode(data, param->value);
		dev_dbg(dev, "rc %d req %d now %d", rc,
			param->value, data->poll_delay_ms);
	}

	return rc;
}

/**
 * sysfs attribute "mode " [rd/wr]
 *
 * set the mode value can only be used while: not ranging
 * @li 1 @a VL53L1_PRESETMODE_RANGING default ranging
 * @li 2 @a VL53L1_PRESETMODE_MULTIZONES_SCANNING multiple zone
 * @li 3 @a VL53L1_PRESETMODE_AUTONOMOUS autonomous mode
 * @li 4 @a VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS low Power autonomous mode
 * @li 5 @a VL53L1_PRESETMODE_LITE_RANGING low mips ranging mode
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(mode);

static int stmvl53l1_set_distance_mode(struct stmvl53l1_data *data,
	int distance_mode)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev, "can't change distance mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (distance_mode) {
		case VL53L1_DISTANCEMODE_SHORT:
		case VL53L1_DISTANCEMODE_MEDIUM:
		case VL53L1_DISTANCEMODE_LONG:
			data->distance_mode = distance_mode;
			break;
		default:
			dev_err(dev, "invalid distance mode %d\n", distance_mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(distance_mode, "distance mode")

/**
 * sysfs attribute " distance mode" [rd/wr]
 *
 * set the distance mode value can only be used while: not ranging
 * @li 1 @a VL53L1_DISTANCEMODE_SHORT
 * @li 2 @a VL53L1_DISTANCEMODE_MEDIUM
 * @li 3 @a VL53L1_DISTANCEMODE_LONG
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(distance_mode);

static int stmvl53l1_set_crosstalk_enable(struct stmvl53l1_data *data,
	int crosstalk_enable)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev,
			"can't change crosstalk enable while ranging\n");
		rc = -EBUSY;
	} else if (crosstalk_enable == 0 || crosstalk_enable == 1) {
		data->crosstalk_enable = crosstalk_enable;
	} else {
		dev_err(dev, "invalid crosstalk enable %d\n", crosstalk_enable);
		rc = -EINVAL;
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(crosstalk_enable, "crosstalk enable")

/**
 * sysfs attribute " crosstalk enable" [rd/wr]
 *
 * control if crosstalk compensation is enable or not
 * @li 0 disable crosstalk compensation
 * @li 1 enable crosstalk compensation
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(crosstalk_enable);

static int stmvl53l1_set_output_mode(struct stmvl53l1_data *data,
	int output_mode)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev, "can't change output mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (output_mode) {
		case VL53L1_OUTPUTMODE_NEAREST:
		case VL53L1_OUTPUTMODE_STRONGEST:
			data->output_mode = output_mode;
			break;
		default:
			dev_err(dev, "invalid output mode %d\n", output_mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(output_mode, "output mode")

/**
 * sysfs attribute " output mode" [rd/wr]
 *
 * set the output mode value can only be used while: not ranging
 * @li 1 @a VL53L1_OUTPUTMODE_NEAREST
 * @li 2 @a VL53L1_OUTPUTMODE_STRONGEST
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(output_mode);

static int stmvl53l1_set_force_device_on_enable(struct stmvl53l1_data *data,
						int force_device_on_en)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (force_device_on_en != 0 && force_device_on_en != 1) {
		dev_err(dev, "invalid force_device_on_en mode %d\n",
			force_device_on_en);
		return -EINVAL;
	}

	data->force_device_on_enable = force_device_on_en;

	/* don't update reset if sensor is enable */
	if (data->enable_sensor)
		return 0;

	/* ok update reset according force_device_on_en value */
	if (force_device_on_en)
		rc = reset_release(data);
	else
		rc = reset_hold(data);

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(force_device_on_enable, "force device on enable")

/**
 * sysfs attribute " force_device_on_enable" [rd/wr]
 *
 * Control if device is put under reset when stopped.
 * @li 0 feature is disable. Device is put under reset when stopped.
 * @li 1 feature is enable. Device is not put under reset when stopped.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(force_device_on_enable);

static int stmvl53l1_set_offset_correction_mode(struct stmvl53l1_data *data,
	int offset_correction_mode)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev,
			"can't change offset correction mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (offset_correction_mode) {
		case VL53L1_OFFSETCORRECTIONMODE_STANDARD:
		case VL53L1_OFFSETCORRECTIONMODE_PERZONE:
		case VL53L1_OFFSETCORRECTIONMODE_PERVCSEL:
			data->offset_correction_mode = offset_correction_mode;
			break;
		default:
			dev_err(dev, "invalid offset correction mode %d\n",
				offset_correction_mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(offset_correction_mode, "offset correction mode")

/**
 * sysfs attribute " offset_correction_mode" [rd/wr]
 *
 * Control which offset correction is apply on result. can only be used
 * while: not ranging
 * @li 1 @a VL53L1_OFFSETCORRECTIONMODE_STANDARD
 * @li 2 @a VL53L1_OFFSETCORRECTIONMODE_PERZONE
 * @li 3 @a VL53L1_OFFSETCORRECTIONMODE_PERVCSEL
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(offset_correction_mode);

static ssize_t do_flush_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->work_mutex);

	data->flush_todo_counter++;
	if (data->enable_sensor == 0)
		stmvl53l1_insert_flush_events_lock(data);

	mutex_unlock(&data->work_mutex);

	return count;
}

static DEVICE_ATTR_WO(do_flush);

static ssize_t enable_debug_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", stmvl53l1_enable_debug);
}

static ssize_t enable_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int enable_debug;
	int rc = 0;

	if (kstrtoint(buf, 0, &enable_debug))
		rc = -EINVAL;
	else
		stmvl53l1_enable_debug = enable_debug;

	return rc ? rc : count;
}

/**
 * sysfs attribute " debug enable" [rd/wr]
 *
 * dynamic control of vl53l1_dbgmsg messages. Note that in any case your code
 * must be enable with DEBUG in stmvl53l1.h at compile time.
 * @li 0 disable vl53l1_dbgmsg messages
 * @li 1 enable vl53l1_dbgmsg messages
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(enable_debug);

static ssize_t display_FixPoint1616(char *buf, size_t size, FixPoint1616_t fix)
{
	uint32_t msb = fix >> 16;
	uint32_t lsb = fix & 0xffff;

	lsb = (lsb * 1000000ULL + 32768) / 65536;

	return scnprintf(buf, size, "%d.%06d", msb, (uint32_t)lsb);
}

static ssize_t autonomous_config_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	ssize_t res = 0;

	res += sysfs_emit(buf, "%d %d %d %d %d %d %d ",
		data->auto_pollingTimeInMs,
		data->auto_config.DetectionMode,
		data->auto_config.IntrNoTarget,
		data->auto_config.Distance.CrossMode,
		data->auto_config.Distance.High,
		data->auto_config.Distance.Low,
		data->auto_config.Rate.CrossMode);

	res += display_FixPoint1616(&buf[res], PAGE_SIZE - res,
		data->auto_config.Rate.High);

	res += sysfs_emit_at(buf, res, " ");

	res += display_FixPoint1616(&buf[res], PAGE_SIZE - res,
		data->auto_config.Rate.Low);

	res += sysfs_emit_at(buf, res, "\n");

	return res;
}

static const char *parse_integer(const char *buf, int *res)
{
	int rc;

	while (*buf == ' ')
		buf++;
	rc = sscanf(buf, "%d ", res);
	if (!rc)
		return NULL;

	return strchr(buf, ' ');
}

static bool is_float_format(const char *buf, bool is_last)
{
	char *dot = strchr(buf, '.');
	char *space_or_eos = strchr(buf, is_last ? '\0' : ' ');

	if (!space_or_eos)
		return !!dot;
	if (!dot)
		return false;

	return dot < space_or_eos ? true : false;
}

static int parse_FixPoint16x16_lsb(const char *lsb_char)
{
	int lsb = 0;
	int digit_nb = 0;

	/* parse at most 6 digits */
	lsb_char++;
	while (isdigit(*lsb_char) && digit_nb < 6) {
		lsb = lsb * 10 + (*lsb_char - '0');
		lsb_char++;
		digit_nb++;
	}
	while (digit_nb++ < 6)
		lsb = lsb * 10;

	return div64_s64(lsb * 65536ULL + 500000, 1000000);
}

/* parse next fix point value and return a pointer to next blank or newline
 * character according to is_last parameter.
 * parse string must have digit for integer part (something like '.125' will
 * return an error) or an error will be return. Only the first 6 digit of the
 * decimal part will be parsed.
 */
static const char *parse_FixPoint16x16(const char *buf, FixPoint1616_t *res,
	bool is_last)
{
	bool is_float;
	int msb;
	int lsb = 0;
	int rc;

	while (*buf == ' ')
		buf++;
	is_float = is_float_format(buf, is_last);

	/* scan msb */
	rc = sscanf(buf, "%d ", &msb);
	if (!rc)
		return NULL;
	/* then lsb if present */
	if (is_float)
		lsb = parse_FixPoint16x16_lsb(strchr(buf, '.'));
	*res = (msb << 16) + lsb;

	return strchr(buf, is_last ? '\0' : ' ');
}

static ssize_t autonomous_config_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int pollingTimeInMs, DetectionMode, IntrNoTarget;
	int d_CrossMode, d_High, d_Low;
	int r_CrossMode;
	FixPoint1616_t r_High, r_Low;
	const char *buf_ori = buf;
	int rc;

	mutex_lock(&data->work_mutex);

	if (data->enable_sensor)
		goto busy;

	buf = parse_integer(buf, &pollingTimeInMs);
	if (!buf)
		goto invalid;
	buf = parse_integer(buf, &DetectionMode);
	if (!buf)
		goto invalid;
	buf = parse_integer(buf, &IntrNoTarget);
	if (!buf)
		goto invalid;
	buf = parse_integer(buf, &d_CrossMode);
	if (!buf)
		goto invalid;
	buf = parse_integer(buf, &d_High);
	if (!buf)
		goto invalid;
	buf = parse_integer(buf, &d_Low);
	if (!buf)
		goto invalid;
	buf = parse_integer(buf, &r_CrossMode);
	if (!buf)
		goto invalid;
	buf = parse_FixPoint16x16(buf, &r_High, false);
	if (!buf)
		goto invalid;
	buf = parse_FixPoint16x16(buf, &r_Low, true);
	if (!buf)
		goto invalid;

	data->auto_pollingTimeInMs = pollingTimeInMs;
	data->auto_config.DetectionMode = DetectionMode;
	data->auto_config.IntrNoTarget = IntrNoTarget;
	data->auto_config.Distance.CrossMode = d_CrossMode;
	data->auto_config.Distance.High = d_High;
	data->auto_config.Distance.Low = d_Low;
	data->auto_config.Rate.CrossMode = r_CrossMode;
	data->auto_config.Rate.High = r_High;
	data->auto_config.Rate.Low = r_Low;

	mutex_unlock(&data->work_mutex);

	return count;

busy:
	dev_err(dev, "can't change config while ranging\n");
	rc = -EBUSY;
	goto error;

invalid:
	dev_err(dev, "%s: invalid syntax in %s\n", __func__, buf_ori);
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

/**
 * sysfs attribute " autonomous_config" [rd/wr]
 *
 * Will set/get autonomous configuration using sysfs.
 *
 * format is the following :
 * <poll_ms> <mode> <no_target_irq> <distance_mode> <distance_low>
 * <distance_high> <rate_mode> <rate_low> <rate_high>
 *
 *@code
 * > echo "1000 1 0 0 1000 300 0 2.2 1.001" > autonomous_config
 *@endcode
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(autonomous_config);

static ssize_t last_error_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", data->last_error);
}

/**
 * sysfs attribute " last_error" [rd]
 *
 * Will get last internal error using sysfs.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RO(last_error);

static ssize_t optical_center_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	ssize_t res = 0;

	res += display_FixPoint1616(&buf[res], PAGE_SIZE - res,
		data->optical_offset_x);
	res += sysfs_emit_at(buf, res, " ");
	res += display_FixPoint1616(&buf[res], PAGE_SIZE - res,
		data->optical_offset_y);

	res += sysfs_emit_at(buf, res, "\n");

	return res;
}

/**
 * sysfs attribute " optical_center" [rd]
 *
 * Will get optical_center using sysfs.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RO(optical_center);

static int stmvl53l1_set_dmax_reflectance(struct stmvl53l1_data *data,
	int dmax_reflectance)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev,
			"can't change dmax reflectance while ranging\n");
		rc = -EBUSY;
	} else
		data->dmax_reflectance = dmax_reflectance;

	return rc;
}

static ssize_t dmax_reflectance_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	ssize_t res = 0;

	res += display_FixPoint1616(&buf[res], PAGE_SIZE - res,
		data->dmax_reflectance);

	res += sysfs_emit_at(buf, res, "\n");

	return res;
}

static ssize_t dmax_reflectance_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	FixPoint1616_t dmax_reflectance;
	const char *buf_ori = buf;
	int rc;

	mutex_lock(&data->work_mutex);

	buf = parse_FixPoint16x16(buf, &dmax_reflectance, true);
	if (!buf)
		goto invalid;
	rc = stmvl53l1_set_dmax_reflectance(data, dmax_reflectance);
	if (rc)
		goto error;

	mutex_unlock(&data->work_mutex);

	return count;

invalid:
	dev_err(dev, "%s: invalid syntax in %s\n", __func__, buf_ori);
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

/**
 * sysfs attribute "dmax_reflectance" [rd/wr]
 *
 * target reflectance use for calculate the ambient DMAX. can only be used
 * while: not ranging
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(dmax_reflectance);

static int stmvl53l1_set_dmax_mode(struct stmvl53l1_data *data,
	int dmax_mode)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev, "can't change dmax mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (dmax_mode) {
		case VL53L1_DMAXMODE_FMT_CAL_DATA:
		case VL53L1_DMAXMODE_CUSTCAL_DATA:
		case VL53L1_DMAXMODE_PER_ZONE_CAL_DATA:
			data->dmax_mode = dmax_mode;
			break;
		default:
			dev_err(dev, "invalid dmax mode %d\n", dmax_mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(dmax_mode, "dmax mode")

/**
 * sysfs attribute " dmax mode" [rd/wr]
 *
 * set the dmax mode value can only be used while: not ranging
 * @li 1 @a VL53L1_DMAXMODE_FMT_CAL_DATA
 * @li 2 @a VL53L1_DMAXMODE_CUSTCAL_DATA
 * @li 3 @a VL53L1_DMAXMODE_PER_ZONE_CAL_DATA
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(dmax_mode);

static int stmvl53l1_set_tuning(struct stmvl53l1_data *data, int key,
	int value)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev, "can't change tuning params while ranging\n");
		return -EBUSY;
	}

	if (data->is_calibrating) {
		dev_err(dev,
			"can't change tuning params while calibrating\n");
		return -EBUSY;
	}

	if (key & ~0xffff)
		return -EINVAL;

	dev_dbg(dev, "trying to set %d with key %d\n", value, key);

	rc = VL53L1_SetTuningParameter(&data->stdev, key, value);
	if (rc)
		rc = store_last_error(data, rc);

	return rc;
}

static ssize_t tuning_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int key;
	int value;
	int n;
	int rc;

	mutex_lock(&data->work_mutex);

	n = sscanf(buf, "%d %d", &key, &value);
	if (n != 2) {
		rc = -EINVAL;
		goto error;
	}
	rc = stmvl53l1_set_tuning(data, key, value);
	if (rc)
		goto error;

	mutex_unlock(&data->work_mutex);

	return count;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

/**
 * sysfs attribute "tuning" [wr]
 *
 * write a tuning parameter. Two integer parameters are given. First one
 * is a key that specify which tuning parameter is update. Other one is the
 * value which is write.
 *
 * writing a tuning parameter is only allowed before the first start.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_WO(tuning);

static int stmvl53l1_display_tuning_key(struct stmvl53l1_data *data, char *buf,
	int *pos, int key)
{
	int rc = 0;
	int value = 0;
	int sz;

	rc = VL53L1_GetTuningParameter(&data->stdev, key, &value);
	if (rc)
		return 0;

	sz = sysfs_emit_at(buf, *pos, "%d %d\n", key, value);

	*pos += sz;

	return 0;
}

static ssize_t tuning_status_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const int max_tuning_key = 65535;
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int rc;
	int i;
	int pos = 0;

	mutex_lock(&data->work_mutex);

	for (i = 0; i < max_tuning_key; ++i) {
		rc = stmvl53l1_display_tuning_key(data, buf, &pos, i);
		if (rc)
			break;
	}

	mutex_unlock(&data->work_mutex);

	return rc ? rc : pos;
}

/**
 * sysfs attribute "tuning_status" [rd]
 *
 * write a tuning parameter. Two integer parameters are given. First one
 * is a key that specify which tuning parameter is update. Other one is the
 * value which is write.
 *
 * writing a tuning parameter is only allowed before the first start.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RO(tuning_status);

static int stmvl53l1_set_smudge_correction_mode(struct stmvl53l1_data *data,
	int smudge_correction_mode)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev,
			"can't change smudge corr mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (smudge_correction_mode) {
		case VL53L1_SMUDGE_CORRECTION_NONE:
		case VL53L1_SMUDGE_CORRECTION_CONTINUOUS:
		case VL53L1_SMUDGE_CORRECTION_SINGLE:
		case VL53L1_SMUDGE_CORRECTION_DEBUG:
			data->smudge_correction_mode = smudge_correction_mode;
			break;
		default:
			dev_err(dev, "invalid smudge correction mode %d\n",
				smudge_correction_mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(smudge_correction_mode, "smudge correction mode")

/**
 * sysfs attribute " smudge_correction_mode" [rd/wr]
 *
 * This parameter will control if smudge correction is enable and how crosstalk
 * values are updated.
 * @li 0 @a VL53L1_SMUDGE_CORRECTION_NONE
 * @li 1 @a VL53L1_SMUDGE_CORRECTION_CONTINUOUS
 * @li 2 @a VL53L1_SMUDGE_CORRECTION_SINGLE
 * @li 3 @a VL53L1_SMUDGE_CORRECTION_DEBUG
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RW(smudge_correction_mode);

static ssize_t is_xtalk_value_changed_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int param;

	mutex_lock(&data->work_mutex);
	param = data->is_xtalk_value_changed;
	mutex_unlock(&data->work_mutex);

	return sysfs_emit(buf, "%d\n", param);
}

/**
 * sysfs attribute " optical_center" [rd]
 *
 * Will get optical_center using sysfs.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR_RO(is_xtalk_value_changed);

static int stmvl53l1_set_ulp_enable(struct stmvl53l1_data *data, int value)
{
	struct i2c_data *i2c_data = data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->enable_sensor) {
		dev_err(dev, "can't change ulp mode while ranging\n");
		return -EBUSY;
	}

	data->ulp_enable = value;

	return 0;
}
IMPLEMENT_PARAMETER_INTEGER(ulp_enable, "ulp enable")
static DEVICE_ATTR_RW(ulp_enable);

static int stmvl53l1_set_macro_timing(struct stmvl53l1_data *data, int value)
{
	int rc = 0;
	struct i2c_data *i2c_data = data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (value <= 0 || value > 255) {
		dev_err(dev, "invalid value %d\n", value);
		return -EINVAL;
	}
	if (!data->enable_sensor) {
		data->macro_timing = value;
		return 0;
	}

	rc = VL53L1_ULP_SetMacroTiming(&data->stdev, value);
	if (rc) {
		dev_err(dev, "Set macro_timing %d fail %d\n", value, rc);
		rc = store_last_error(data, rc);
	} else
		data->macro_timing = value;

	return rc;
}
IMPLEMENT_PARAMETER_INTEGER(macro_timing, "macro timing")
static DEVICE_ATTR_RW(macro_timing);

static int stmvl53l1_set_inter_measurement(struct stmvl53l1_data *data,
					   int value)
{
	int rc = 0;
	struct i2c_data *i2c_data = data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (value < 20 || value > 60000) {
		dev_err(dev, "invalid value %d\n", value);
		return -EINVAL;
	}
	if (!data->enable_sensor) {
		data->inter_measurement = value;
		return 0;
	}

	rc = VL53L1_ULP_SetInterMeasurement(&data->stdev, value);
	if (rc) {
		dev_err(dev, "Set inter_measurement %d fail %d\n", value, rc);
		rc = store_last_error(data, rc);
	} else
		data->inter_measurement = value;

	return rc;
}
IMPLEMENT_PARAMETER_INTEGER(inter_measurement, "inter measurement")
static DEVICE_ATTR_RW(inter_measurement);

static int stmvl53l1_set_sigma_threshold(struct stmvl53l1_data *data, int value)
{
	int rc = 0;
	struct i2c_data *i2c_data = data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (value > (0XFFFF >> 2)) {
		dev_err(dev, "invalid value %d\n", value);
		return-EINVAL;
	}
	if (!data->enable_sensor) {
		data->sigma_threshold = value;
		return 0;
	}

	rc = VL53L1_ULP_SetSigmaThreshold(&data->stdev, value);
	if (rc) {
		dev_err(dev, "Set sigma_threshold %d fail %d\n", value, rc);
		rc = store_last_error(data, rc);
	} else
		data->sigma_threshold = value;

	return rc;
}
IMPLEMENT_PARAMETER_INTEGER(sigma_threshold, "sigma threshold")
static DEVICE_ATTR_RW(sigma_threshold);

static int stmvl53l1_set_signal_threshold(struct stmvl53l1_data *data,
					  int value)
{
	int rc = 0;
	struct i2c_data *i2c_data = data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (value < 1 || value > 16384) {
		dev_err(dev, "invalid value %d\n", value);
		return -EINVAL;
	}
	if (!data->enable_sensor) {
		data->signal_threshold = value;
		return 0;
	}

	rc = VL53L1_ULP_SetSignalThreshold(&data->stdev, value);
	if (rc) {
		dev_err(dev, "Set signal_threshold %d fail %d\n", value, rc);
		rc = store_last_error(data, rc);
	} else
		data->signal_threshold = value;

	return rc;
}
IMPLEMENT_PARAMETER_INTEGER(signal_threshold, "signal threshold")
static DEVICE_ATTR_RW(signal_threshold);

static struct attribute *stmvl53l1_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_set_delay_ms.attr,
	&dev_attr_timing_budget.attr,
	&dev_attr_roi.attr,
	&dev_attr_mode.attr,
	&dev_attr_do_flush.attr,
	&dev_attr_distance_mode.attr,
	&dev_attr_crosstalk_enable.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_output_mode.attr,
	&dev_attr_force_device_on_enable.attr,
	&dev_attr_autonomous_config.attr,
	&dev_attr_last_error.attr,
	&dev_attr_offset_correction_mode.attr,
	&dev_attr_optical_center.attr,
	&dev_attr_dmax_reflectance.attr,
	&dev_attr_dmax_mode.attr,
	&dev_attr_tuning.attr,
	&dev_attr_tuning_status.attr,
	&dev_attr_smudge_correction_mode.attr,
	&dev_attr_is_xtalk_value_changed.attr,
	&dev_attr_product_type.attr,
	&dev_attr_ulp_enable.attr,
	&dev_attr_macro_timing.attr,
	&dev_attr_inter_measurement.attr,
	&dev_attr_sigma_threshold.attr,
	&dev_attr_signal_threshold.attr,
	NULL
};

static ssize_t calibration_data_read(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	struct VL53L1_CalibrationData_t calib;
	int rc;
	void *src = (void *)&calib;

	mutex_lock(&data->work_mutex);

	dev_dbg(dev, "off = %lld / count = %zu\n",  off, count);

	if (off < 0 || off > sizeof(struct VL53L1_CalibrationData_t))
		goto invalid;

	/* got current calibration data */
	memset(&calib, 0, sizeof(calib));
	rc = VL53L1_GetCalibrationData(&data->stdev, &calib);
	if (rc) {
		dev_err(dev, "VL53L1_GetCalibrationData fail %d\n", rc);
		rc = store_last_error(data, rc);
		goto error;
	}

	/* copy to buffer */
	if (off + count > sizeof(struct VL53L1_CalibrationData_t))
		count = sizeof(struct VL53L1_CalibrationData_t) - off;
	memcpy(buf, src + off, count);

	mutex_unlock(&data->work_mutex);

	return count;

invalid:
	dev_err(dev, "%s: invalid syntax\n", __func__);
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static ssize_t calibration_data_write(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&data->work_mutex);

	dev_dbg(dev, "off = %lld / count = %zu\n", off, count);

	if (data->enable_sensor) {
		rc = -EBUSY;
		dev_err(dev, "can't set calib data while ranging\n");
		goto error;
	}

	/* we only support one time write */
	if (off != 0 || count != sizeof(struct VL53L1_CalibrationData_t))
		goto invalid;

	rc = VL53L1_SetCalibrationData(&data->stdev,
		(struct VL53L1_CalibrationData_t *)buf);
	if (rc) {
		dev_err(dev, "VL53L1_SetCalibrationData fail %d\n", rc);
		rc = store_last_error(data, rc);
		goto error;
	}

	mutex_unlock(&data->work_mutex);

	return count;

invalid:
	dev_err(dev, "%s: invalid syntax\n", __func__);
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static BIN_ATTR_RW(calibration_data, sizeof(struct VL53L1_CalibrationData_t));

static ssize_t zone_calibration_data_read(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int rc;
	void *src = (void *)&data->calib.data;

	mutex_lock(&data->work_mutex);

	dev_dbg(dev, "off = %lld / count = %zu\n", off, count);

	if (off < 0 || off > sizeof(stmvl531_zone_calibration_data_t))
		goto invalid;

	/* got current zone calibration data */
	rc = VL53L1_GetZoneCalibrationData(&data->stdev,
		&data->calib.data.data);
	if (rc) {
		dev_err(dev, "VL53L1_GetZoneCalibrationData fail %d\n", rc);
		rc = store_last_error(data, rc);
		goto error;
	}
	data->calib.data.id = data->current_roi_id;

	/* copy to buffer */
	if (off + count > sizeof(stmvl531_zone_calibration_data_t))
		count = sizeof(stmvl531_zone_calibration_data_t) - off;
	memcpy(buf, src + off, count);

	mutex_unlock(&data->work_mutex);

	return count;

invalid:
	dev_err(dev, "%s: invalid syntax\n", __func__);
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static ssize_t zone_calibration_data_write(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int rc;
	void *dst = &data->calib.data;

	mutex_lock(&data->work_mutex);

	dev_dbg(dev, "off = %lld / count = %zu\n", off, count);

	/* implementation if quite fragile. We suppose successive access. We
	 * trigger set on last byte write if amount is exact.
	 */
	if (off < 0 || off > sizeof(stmvl531_zone_calibration_data_t))
		goto invalid;
	if (off + count > sizeof(stmvl531_zone_calibration_data_t))
		goto invalid;

	memcpy(dst + off, buf, count);
	if (off + count == sizeof(stmvl531_zone_calibration_data_t)) {
		dev_dbg(dev, "trigger zone calib setting\n");
		rc = VL53L1_SetZoneCalibrationData(&data->stdev,
			&data->calib.data.data);
		if (rc) {
			dev_err(dev, "VL53L1_SetZoneCalibrationData fail %d\n",
				rc);
		rc = store_last_error(data, rc);
		goto error;
		}
		data->current_roi_id = data->calib.data.id;
	}

	mutex_unlock(&data->work_mutex);

	return count;

invalid:
	dev_err(dev, "%s: invalid syntax\n", __func__);
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static BIN_ATTR_RW(zone_calibration_data, sizeof(stmvl531_zone_calibration_data_t));

static struct bin_attribute *stmvl53l1_bin_attributes[] = {
	&bin_attr_calibration_data,
	&bin_attr_zone_calibration_data,
	NULL
};

static const struct attribute_group stmvl53l1_attr_group = {
	.attrs = stmvl53l1_attributes,
	.bin_attrs = stmvl53l1_bin_attributes,
};

static const struct attribute_group *stmvl53l1_attr_groups[] = {
	&stmvl53l1_attr_group,
	NULL,
};

static int ctrl_reg_access(struct stmvl53l1_data *data, void *p)
{
	struct stmvl53l1_register reg;
	size_t total_byte;
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (data->is_device_remove)
		return -ENODEV;

	total_byte = offsetof(struct stmvl53l1_register, data.b);
	if (copy_from_user(&reg, p, total_byte)) {
		dev_err(dev, "%d, fail\n", __LINE__);
		return -EFAULT;
	}

	if (reg.cnt > STMVL53L1_MAX_CCI_XFER_SZ) {
		dev_err(dev, "reg len %d > size limit\n", reg.cnt);
		return -EINVAL;
	}

	total_byte = offsetof(struct stmvl53l1_register, data.bytes[reg.cnt]);
	/* for write get the effective data part of the structure */
	if (!reg.is_read) {
		if (copy_from_user(&reg, p, total_byte)) {
			dev_err(dev, "data copy fail\n");
			return -EFAULT;
		}
	}

	/* put back to user only needed amount of data */
	if (!reg.is_read) {
		rc = VL53L1_WriteMulti(&data->stdev, (uint16_t)reg.index,
				reg.data.bytes, reg.cnt);
		reg.status = rc;
		/* for write only write back status no data */
		total_byte = offsetof(struct stmvl53l1_register, data.b);
		dev_dbg(dev, "wr %x %d bytes statu %d\n",
			reg.index, reg.cnt, rc);
		if (rc)
			rc = store_last_error(data, rc);
	} else {
		rc = VL53L1_ReadMulti(&data->stdev, (uint16_t)reg.index,
				reg.data.bytes, reg.cnt);
		reg.status = rc;
		dev_dbg(dev, "rd %x %d bytes status %d\n",
			reg.index, reg.cnt, rc);
		/*  if fail do not copy back data only status */
		if (rc) {
			total_byte = offsetof(struct stmvl53l1_register,
					data.b);
			rc = store_last_error(data, rc);
		}
		/* else the total byte is already the full pay-load with data */
	}

	if (copy_to_user(p, &reg, total_byte)) {
		dev_err(dev, "%d, fail\n", __LINE__);
		return -EFAULT;
	}
	return rc;
}

static int ctrl_power_up(struct stmvl53l1_data *data)
{
	int rc;

	rc = stmvl53l1_module_func_tbl.power_up(data->client_object);

	return rc;
}

static int ctrl_power_down(struct stmvl53l1_data *data)
{
	int rc;

	rc = stmvl53l1_module_func_tbl.power_down(data->client_object);

	return rc;
}

static int ctrl_start(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}

	dev_dbg(dev, "state = %d\n", data->enable_sensor);

	/* turn on tof sensor only if it's not already started */
	if (data->enable_sensor == 0 && !data->is_calibrating) {
		/* to start */
		rc = stmvl53l1_start(data);
	} else {
		rc = -EBUSY;
	}
	dev_dbg(dev, "final state = %d\n", data->enable_sensor);

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

/**
 * no lock version of ctrl_stop (mutex shall be held)
 *
 * @warning exist only for use in device exit to ensure "locked and started"
 * may also beuse in soem erro handling when mutex is already locked
 * @return 0 on success and was running >0 if already off <0 on error
 */
static int _ctrl_stop(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	dev_dbg(dev, "enter state = %d\n", data->enable_sensor);
	/* be sure waiters are woken */
	data->is_data_valid = true;
	/* turn on tof sensor only if it's not enabled by other	client */
	if (data->enable_sensor == 1) {
		/* to stop */
		rc = stmvl53l1_stop(data);
	} else {
		dev_dbg(dev, "already off did nothing\n");
		rc = 0;
	}
	stmvl53l1_insert_flush_events_lock(data);
	dev_dbg(dev, "final state = %d\n", data->enable_sensor);

	return rc;
}

/**
 * get work lock and stop sensor
 *
 * see @ref _ctrl_stop
 *
 * @param data device
 * @return 0 on success EBUSY if arleady off
 */
static int ctrl_stop(struct stmvl53l1_data *data)
{
	int rc;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	if (data->enable_sensor)
		rc = _ctrl_stop(data);
	else
		rc = -EBUSY;

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_getdata(struct stmvl53l1_data *data, void __user *p)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	rc = copy_to_user(p,
		&data->meas.single_range_data,
		sizeof(stmvl531_range_data_t));
	if (rc) {
		dev_dbg(dev, "copy to user fail %d\n", rc);
		rc = -EFAULT;
	}

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static bool is_new_data_for_me(struct stmvl53l1_data *data, pid_t pid,
	struct list_head *head)
{
	return data->is_data_valid && !is_pid_in_list(pid, head);
}

static bool sleep_for_data_condition(struct stmvl53l1_data *data, pid_t pid,
	struct list_head *head)
{
	bool res;

	mutex_lock(&data->work_mutex);
	res = is_new_data_for_me(data, pid, head);
	mutex_unlock(&data->work_mutex);

	return res;
}

static int sleep_for_data(struct stmvl53l1_data *data, pid_t pid,
				struct list_head *head)
{
	int rc = 0;

	DEFINE_WAIT_FUNC(wait, woken_wake_function);
	mutex_unlock(&data->work_mutex);

	add_wait_queue(&data->waiter_for_data, &wait);
	while (!sleep_for_data_condition(data, pid, head)) {
		wait_woken(&wait, TASK_KILLABLE, MAX_SCHEDULE_TIMEOUT);
		if (fatal_signal_pending(current)) {
			rc = -ERESTARTSYS;
			break;
		}
	}
	remove_wait_queue(&data->waiter_for_data, &wait);
	mutex_lock(&data->work_mutex);

	return data->enable_sensor ? rc : -ENODEV;
}

static int ctrl_getdata_blocking(struct stmvl53l1_data *data, void __user *p)
{
	int rc = 0;
	pid_t pid = current->pid;

	mutex_lock(&data->work_mutex);
	/* If device not ranging then exit on error */
	if (data->is_device_remove || !data->enable_sensor) {
		rc = -ENODEV;
		goto done;
	}
	/* sleep if data already read */
	if (!is_new_data_for_me(data, pid, &data->simple_data_reader_list))
		rc = sleep_for_data(data, pid, &data->simple_data_reader_list);
	if (rc)
		goto done;

	/* unless we got interrupted we return data to user and note read */
	rc = copy_to_user(p, &data->meas.single_range_data,
		sizeof(stmvl531_range_data_t));
	if (rc)
		goto done;
	rc = add_reader(pid, &data->simple_data_reader_list);

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_mz_data_common(struct stmvl53l1_data *data, void __user *p,
	bool is_additional)
{
	struct stmvl53l1_data_with_additional __user *d = p;
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	rc = copy_to_user(&d->data, &data->meas.multi_range_data,
		sizeof(struct VL53L1_MultiRangingData_t));
	if (rc) {
		dev_dbg(dev, "copy to user fail %d\n", rc);
		rc = -EFAULT;
		goto done;
	}
	if (is_additional) {
		rc = copy_to_user(&d->additional_data,
			&data->meas.additional_data,
			sizeof(VL53L1_AdditionalData_t));
		if (rc) {
			dev_dbg(dev, "copy to user fail %d\n", rc);
			rc = -EFAULT;
			goto done;
		}
	}
	if (!data->enable_sensor)
		rc = -ENODEV;
	else if (!is_mz_mode(data))
		rc = -ENOEXEC;

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_mz_data_blocking_common(struct stmvl53l1_data *data,
	void __user *p, bool is_additional)
{
	int rc = 0;
	struct stmvl53l1_data_with_additional __user *d = p;
	pid_t pid = current->pid;

	mutex_lock(&data->work_mutex);
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	/* mode 2 or 3 */
	if (!is_mz_mode(data)) {
		rc = -ENOEXEC;
		goto done;
	}
	/* If device not ranging then exit on error */
	if (!data->enable_sensor) {
		rc = -ENODEV;
		goto done;
	}
	/* sleep if data already read */
	if (!is_new_data_for_me(data, pid, &data->mz_data_reader_list))
		rc = sleep_for_data(data, pid, &data->mz_data_reader_list);
	if (rc)
		goto done;

	/* unless we got interrupted we return data to user and note read */
	rc = copy_to_user(&d->data, &data->meas.multi_range_data,
		sizeof(struct VL53L1_MultiRangingData_t));
	if (rc)
		goto done;
	if (is_additional) {
		rc = copy_to_user(&d->additional_data,
			&data->meas.additional_data,
			sizeof(VL53L1_AdditionalData_t));
		if (rc)
			goto done;
	}
	rc = add_reader(pid, &data->mz_data_reader_list);

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

/**
 * Get multi zone data
 * @param data
 * @param p [out] user ptr to @ref VL53L1_MultiRangingData_t structure\n
 *  is always set but on EFAULT error
 *
 * @return
 * @li 0 on success
 * @li ENODEV if not ranging
 * @li ENOEXEC not in multi zone mode
 * @li EFAULT  copy to user error
 */
static int ctrl_mz_data(struct stmvl53l1_data *data, void __user *p)
{
	return ctrl_mz_data_common(data, p, false);
}

static int ctrl_mz_data_blocking(struct stmvl53l1_data *data, void __user *p)
{
	return ctrl_mz_data_blocking_common(data, p, false);
}

/**
 * Get multi zone data with histogram debug data
 * @param data
 * @param p [out] user ptr to @ref struct stmvl53l1_data_with_additional
 * structure is always set but on EFAULT error
 *
 * @return
 * @li 0 on success
 * @li ENODEV if not ranging
 * @li ENOEXEC not in multi zone mode
 * @li EFAULT  copy to user error
 */
static int ctrl_mz_data_additional(struct stmvl53l1_data *data, void __user *p)
{
	return ctrl_mz_data_common(data, p, true);
}

static int ctrl_mz_data_blocking_additional(struct stmvl53l1_data *data,
	void __user *p)
{
	return ctrl_mz_data_blocking_common(data, p, true);
}

static int ctrl_param_last_error(struct stmvl53l1_data *data,
		struct stmvl53l1_parameter *param)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (param->is_read) {
		param->value = data->last_error;
		param->status = 0;
		dev_dbg(dev, "get last error %d\n", param->value);
		rc = 0;
	} else {
		rc = -EINVAL;
	}

	return rc;
}

static int ctrl_param_optical_center(struct stmvl53l1_data *data,
		struct stmvl53l1_parameter *param)
{
	if (!param->is_read)
		return -EINVAL;

	param->value = data->optical_offset_x;
	param->value2 = data->optical_offset_y;

	return 0;
}

static int ctrl_param_dmax_reflectance(struct stmvl53l1_data *data,
		struct stmvl53l1_parameter *param)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (param->is_read) {
		param->value = data->dmax_reflectance;
		param->status = 0;
		dev_dbg(dev, "get dmax reflectance %d\n", param->value);
		rc = 0;
	} else {
		rc = stmvl53l1_set_dmax_reflectance(data, param->value);
		dev_dbg(dev, "rc %d req %d now %d\n", rc,
			param->value, data->dmax_reflectance);
	}

	return rc;
}

static int ctrl_param_tuning(struct stmvl53l1_data *data,
		struct stmvl53l1_parameter *param)
{
	if (param->is_read)
		return -EINVAL;

	return stmvl53l1_set_tuning(data, param->value, param->value2);
}

static int ctrl_param_is_xtalk_value_changed(struct stmvl53l1_data *data,
		struct stmvl53l1_parameter *param)
{
	if (!param->is_read)
		return -EINVAL;

	param->value = data->is_xtalk_value_changed;

	return 0;
}

/**
 * handle ioctl set param mode
 *
 * @param data
 * @param p
 * @return 0 on success
 */
static int ctrl_params(struct stmvl53l1_data *data, void __user *p)
{
	int rc, rc2;
	struct stmvl53l1_parameter param;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	rc = copy_from_user(&param, p, sizeof(param));
	param.status = 0;
	if (rc) {
		rc = -EFAULT;
		goto done; /* no need for status in user struct */
	}
	switch (param.name) {
	case VL53L1_POLLDELAY_PAR:
		rc = ctrl_param_poll_delay_ms(data, &param);
		break;
	case VL53L1_TIMINGBUDGET_PAR:
		rc = ctrl_param_timing_budget(data, &param);
		break;
	case VL53L1_DEVICEMODE_PAR:
		rc = ctrl_param_preset_mode(data, &param);
		break;
	case VL53L1_DISTANCEMODE_PAR:
		rc = ctrl_param_distance_mode(data, &param);
		break;
	case VL53L1_XTALKENABLE_PAR:
		rc = ctrl_param_crosstalk_enable(data, &param);
		break;
	case VL53L1_OUTPUTMODE_PAR:
		rc = ctrl_param_output_mode(data, &param);
		break;
	case VL53L1_FORCEDEVICEONEN_PAR:
		rc = ctrl_param_force_device_on_enable(data, &param);
		break;
	case VL53L1_LASTERROR_PAR:
		rc = ctrl_param_last_error(data, &param);
		break;
	case VL53L1_OFFSETCORRECTIONMODE_PAR:
		rc = ctrl_param_offset_correction_mode(data, &param);
		break;
	case VL53L1_OPTICALCENTER_PAR:
		rc = ctrl_param_optical_center(data, &param);
		break;
	case VL53L1_DMAXREFLECTANCE_PAR:
		rc = ctrl_param_dmax_reflectance(data, &param);
		break;
	case VL53L1_DMAXMODE_PAR:
		rc = ctrl_param_dmax_mode(data, &param);
		break;
	case VL53L1_TUNING_PAR:
		rc = ctrl_param_tuning(data, &param);
		break;
	case VL53L1_SMUDGECORRECTIONMODE_PAR:
		rc = ctrl_param_smudge_correction_mode(data, &param);
		break;
	case VL53L1_ISXTALKVALUECHANGED_PAR:
		rc = ctrl_param_is_xtalk_value_changed(data, &param);
		break;
	case VL53L1_ULP_ENABLE_PAR:
		rc = ctrl_param_ulp_enable(data, &param);
		break;
	case VL53L1_MACRO_TIMING_PAR:
		rc = ctrl_param_macro_timing(data, &param);
		break;
	case VL53L1_INTER_MEASUREMENT_PAR:
		rc = ctrl_param_inter_measurement(data, &param);
		break;
	case VL53L1_SIGMA_THRESHOLD_PAR:
		rc = ctrl_param_sigma_threshold(data, &param);
		break;
	case VL53L1_SIGNAL_THRESHOLD_PAR:
		rc = ctrl_param_signal_threshold(data, &param);
		break;
	default:
		dev_err(dev, "unknown or unsupported %d\n", param.name);
		rc = -EINVAL;
	}
	/* copy back (status at least ) to user */
	if (param.is_read && rc == 0) {
		rc2 = copy_to_user(p, &param, sizeof(param));
		if (rc2) {
			rc = -EFAULT; /* kill prev status if that fail */
			dev_err(dev, "copy to user fail %d\n", rc);
		}
	}
done:
	mutex_unlock(&data->work_mutex);
	return rc;
}

/**
 * implement set/get roi ioctl
 * @param data device
 * @param p user space ioctl arg ptr
 * @return 0 on success <0 errno code
 *	@li -EINVAL invalid number of roi
 *	@li -EBUSY when trying to set roi while ranging
 *	@li -EFAULT if cpy to/fm user fail for requested number of roi
 */
static int ctrl_roi(struct stmvl53l1_data *data, void __user *p)
{
	int rc;
	int roi_cnt;
	struct stmvl53l1_roi_full_t rois;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	/* copy fixed part of args at first */
	rc = copy_from_user(&rois, p,
			offsetof(struct stmvl53l1_roi_full_t,
					roi_cfg.UserRois[0]));
	if (rc) {
		rc = -EFAULT;
		goto done;
	}
	/* check no of roi limit is ok */
	roi_cnt = rois.roi_cfg.NumberOfRoi;
	if (roi_cnt > VL53L1_MAX_USER_ZONES) {
		dev_err(dev, "invalid roi spec cnt=%d > %d\n",
			rois.roi_cfg.NumberOfRoi, VL53L1_MAX_USER_ZONES);
		rc = -EINVAL;
		goto done;
	}

	if (rois.is_read) {
		int cpy_size;

		roi_cnt = min(rois.roi_cfg.NumberOfRoi,
				data->roi_cfg.NumberOfRoi);
		cpy_size = offsetof(struct VL53L1_RoiConfig_t,
				    UserRois[roi_cnt]);
		/* copy from local to user only effective part requested */
		rc = copy_to_user(&((struct stmvl53l1_roi_full_t *)p)->roi_cfg,
			&data->roi_cfg, cpy_size);
		dev_err(dev, "return %d of %d\n",
			roi_cnt, data->roi_cfg.NumberOfRoi);
	} else {
		/* SET check cnt roi is ok */
		if (data->enable_sensor) {
			rc = -EBUSY;
			dev_err(dev, "can't set roi while ranging\n");
			goto done;
		}
		/* get full data that  required from user */
		rc = copy_from_user(&rois, p,
					offsetof(struct stmvl53l1_roi_full_t,
						roi_cfg.UserRois[roi_cnt]));
		if (rc) {
			dev_err(dev, "get %d roi fm user fail\n", roi_cnt);
			rc = -EFAULT;
			goto done;
		}
		/* revalidate roi_cnt in case it changed */
		if (roi_cnt != rois.roi_cfg.NumberOfRoi) {
			rc = -EINVAL;
			goto done;
		}
		dump_roi(dev, data->roi_cfg.UserRois,
			 data->roi_cfg.NumberOfRoi);
		/* we may ask ll driver to check but check is mode dependent
		 * and so we could get erroneous error back
		 */
		memcpy(&data->roi_cfg, &rois.roi_cfg, sizeof(data->roi_cfg));
	}
done:
	mutex_unlock(&data->work_mutex);
	return rc;
}

static int ctrl_autonomous_config(struct stmvl53l1_data *data, void __user *p)
{
	int rc = 0;
	struct stmvl53l1_autonomous_config_t full;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	/* first copy all data */
	rc = copy_from_user(&full, p, sizeof(full));
	if (rc) {
		rc = -EFAULT;
		goto done;
	}

	if (full.is_read) {
		full.pollingTimeInMs = data->auto_pollingTimeInMs;
		full.config = data->auto_config;
		rc = copy_to_user(p, &full, sizeof(full));
		if (rc)
			rc = -EFAULT;
	} else {
		if (data->enable_sensor) {
			rc = -EBUSY;
			dev_err(dev, "can't change config while ranging\n");
			goto done;
		}
		data->auto_pollingTimeInMs = full.pollingTimeInMs;
		data->auto_config = full.config;
	}

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_calibration_data(struct stmvl53l1_data *data, void __user *p)
{
	int rc;
	struct stmvl53l1_ioctl_calibration_data_t calib;
	int data_offset = offsetof(struct stmvl53l1_ioctl_calibration_data_t,
					data);
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	rc = copy_from_user(&calib, p, data_offset);
	if (rc) {
		dev_err(dev, "fail to detect read or write %d\n", rc);
		rc = -EFAULT;
		goto done;
	}

	if (calib.is_read) {
		memset(&calib.data, 0, sizeof(calib.data));
		rc = VL53L1_GetCalibrationData(&data->stdev, &calib.data);
		if (rc) {
			dev_err(dev, "VL53L1_GetCalibrationData fail %d\n", rc);
			rc = store_last_error(data, rc);
			goto done;
		}
		rc = copy_to_user(p + data_offset, &calib.data,
			sizeof(calib.data));
	} else {
		if (data->enable_sensor) {
			rc = -EBUSY;
			dev_err(dev, "can't set calib data while ranging\n");
			goto done;
		}
		rc = copy_from_user(&calib.data, p + data_offset,
			sizeof(calib.data));
		if (rc) {
			dev_err(dev, "fail to copy calib data\n");
			rc = -EFAULT;
			goto done;
		}
		rc = VL53L1_SetCalibrationData(&data->stdev, &calib.data);
		if (rc) {
			dev_err(dev, "VL53L1_SetCalibrationData fail %d\n", rc);
			rc = store_last_error(data, rc);
		}
	}

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_zone_calibration_data(struct stmvl53l1_data *data,
	void __user *p)
{
	int rc;
	struct stmvl53l1_ioctl_zone_calibration_data_t *calib;
	int data_offset =
		offsetof(struct stmvl53l1_ioctl_zone_calibration_data_t, data);
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);

	calib = &data->calib;
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	rc = copy_from_user(calib, p, data_offset);
	if (rc) {
		dev_err(dev, "fail to detect read or write %d\n", rc);
		rc = -EFAULT;
		goto done;
	}

	if (calib->is_read) {
		memset(&calib->data, 0, sizeof(calib->data));
		rc = VL53L1_GetZoneCalibrationData(&data->stdev,
			&calib->data.data);
		if (rc) {
			dev_err(dev,
				"VL53L1_GetZoneCalibrationData fail %d\n", rc);
			rc = store_last_error(data, rc);
			goto done;
		}
		calib->data.id = data->current_roi_id;
		rc = copy_to_user(p + data_offset, &calib->data,
			sizeof(calib->data));
	} else {
		if (data->enable_sensor) {
			rc = -EBUSY;
			dev_err(dev, "can't set calib data while ranging\n");
			goto done;
		}
		rc = copy_from_user(&calib->data, p + data_offset,
			sizeof(calib->data));
		if (rc) {
			dev_err(dev, "fail to copy calib data\n");
			rc = -EFAULT;
			goto done;
		}
		rc = VL53L1_SetZoneCalibrationData(&data->stdev,
			&calib->data.data);
		if (rc) {
			dev_err(dev, "VL53L1_SetZoneCalibrationData fail %d\n",
				rc);
			rc = store_last_error(data, rc);
			goto done;
		}
		data->current_roi_id = calib->data.id;
	}

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_perform_calibration_ref_spad_lock(struct stmvl53l1_data *data,
	struct stmvl53l1_ioctl_perform_calibration_t *calib)
{
	int rc = VL53L1_PerformRefSpadManagement(&data->stdev);
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (rc) {
		dev_err(dev, "VL53L1_PerformRefSpadManagement fail => %d\n",
			rc);
		rc = store_last_error(data, rc);
	}

	return rc;
}

static int ctrl_perform_calibration_crosstalk_lock(struct stmvl53l1_data *data,
	struct stmvl53l1_ioctl_perform_calibration_t *calib)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	/* Set the preset mode passed on param2 */
	data->preset_mode = (VL53L1_PresetModes)(calib->param2);

	rc = stmvl53l1_sendparams(data);
	if (rc)
		goto done;

	rc = VL53L1_PerformXTalkCalibration(&data->stdev, calib->param1);
	if (rc) {
		dev_err(dev, "VL53L1_PerformXTalkCalibration fail => %d\n", rc);
		rc = store_last_error(data, rc);
	}

done:
	return rc;
}

static int ctrl_perform_zone_calibration_offset_lock(
	struct stmvl53l1_data *data,
	struct stmvl53l1_ioctl_perform_calibration_t *calib)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (calib->param1 != VL53L1_OFFSETCALIBRATIONMODE_MULTI_ZONE) {
		dev_err(dev, "invalid param1\n");
		rc = -EINVAL;
		goto done;
	}

	/* setup offset calibration mode */
	rc = VL53L1_SetOffsetCalibrationMode(&data->stdev, calib->param1);
	if (rc) {
		dev_err(dev, "VL53L1_SetOffsetCalibrationMode fail => %d\n",
			rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	/* set mode to multi zone to allow roi settings */
	rc = VL53L1_SetPresetMode(&data->stdev,
		VL53L1_PRESETMODE_MULTIZONES_SCANNING);
	if (rc) {
		dev_err(dev, "VL53L1_SetPresetMode fail => %d\n", rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	/* setup roi */
	rc = VL53L1_SetROI(&data->stdev, &data->roi_cfg);
	if (rc) {
		dev_err(dev, "VL53L1_SetROI fail => %d\n", rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	/* finally perform calibration */
	/* allow delay add after stop in VL53L1_run_<offset|zone>_calibration */
	data->is_delay_allowed = 1;
	rc = VL53L1_PerformOffsetCalibration(&data->stdev, calib->param2,
		calib->param3);
	data->is_delay_allowed = 0;
	if (rc) {
		dev_err(dev, "VL53L1_PerformOffsetCalibration fail => %d\n",
			rc);
		rc = store_last_error(data, rc);
	}

	/* save roi hash for later use */
	data->current_roi_id = stmvl53l1_compute_hash(&data->roi_cfg);

done:
	return rc;
}

static int ctrl_perform_calibration_offset_lock(struct stmvl53l1_data *data,
	struct stmvl53l1_ioctl_perform_calibration_t *calib)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	/* for legacy purpose we still support mz */
	if (calib->param1 == VL53L1_OFFSETCALIBRATIONMODE_MULTI_ZONE)
		return ctrl_perform_zone_calibration_offset_lock(data, calib);

	/* setup offset calibration mode */
	rc = VL53L1_SetOffsetCalibrationMode(&data->stdev, calib->param1);
	if (rc) {
		dev_err(dev, "VL53L1_SetOffsetCalibrationMode fail => %d\n",
			rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	/* finally perform calibration */
	/* allow delay add after stop in VL53L1_run_<offset|zone>_calibration */
	data->is_delay_allowed = 1;
	rc = VL53L1_PerformOffsetCalibration(&data->stdev, calib->param2,
		calib->param3);
	data->is_delay_allowed = 0;
	if (rc) {
		dev_err(dev, "VL53L1_PerformOffsetCalibration fail => %d\n",
			rc);
		rc = store_last_error(data, rc);
	}

done:
	return rc;
}

static int ctrl_perform_simple_calibration_offset_lock(
	struct stmvl53l1_data *data,
	struct stmvl53l1_ioctl_perform_calibration_t *calib)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	/* finally perform calibration */
	/* allow delay add after stop in VL53L1_run_<offset|zone>_calibration */
	data->is_delay_allowed = 1;

	rc = stmvl53l1_sendparams(data);
	if (rc)
		goto done;

	rc = VL53L1_PerformOffsetSimpleCalibration(&data->stdev, calib->param1);
	data->is_delay_allowed = 0;
	if (rc) {
		dev_err(dev, "VL53L1_PerformOffsetSimpleCalibration fail %d\n",
			rc);
		rc = store_last_error(data, rc);
	}

done:
	return rc;
}

static int ctrl_perform_per_vcsel_calibration_offset_lock(
	struct stmvl53l1_data *data,
	struct stmvl53l1_ioctl_perform_calibration_t *calib)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	/* finally perform calibration */
	/* allow delay add after stop in VL53L1_run_<offset|zone>_calibration */
	data->is_delay_allowed = 1;

	rc = stmvl53l1_sendparams(data);
	if (rc)
		goto done;

	rc = VL53L1_PerformOffsetPerVcselCalibration(&data->stdev,
		calib->param1);
	data->is_delay_allowed = 0;
	if (rc) {
		dev_err(dev,
			"VL53L1_PerformOffsetPerVcselCalibration fail => %d\n",
			rc);
		rc = store_last_error(data, rc);
	}

done:
	return rc;
}

static int ctrl_perform_zero_distance_calibration_offset_lock(
	struct stmvl53l1_data *data,
	struct stmvl53l1_ioctl_perform_calibration_t *calib)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	/* finally perform calibration */
	/* allow delay add after stop in VL53L1_run_<offset|zone>_calibration */
	data->is_delay_allowed = 1;

	rc = stmvl53l1_sendparams(data);
	if (rc)
		goto done;

	rc = VL53L1_PerformOffsetZeroDistanceCalibration(&data->stdev);
	data->is_delay_allowed = 0;
	if (rc) {
		dev_err(dev, "VL53L1_PerformOffsetZeroDistanceCal fail %d\n",
			rc);
		rc = store_last_error(data, rc);
	}

done:
	return rc;
}

static int ctrl_perform_calibration(struct stmvl53l1_data *data, void __user *p)
{
	int rc;
	struct stmvl53l1_ioctl_perform_calibration_t calib;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	data->is_calibrating = true;
	rc = copy_from_user(&calib, p, sizeof(calib));
	if (rc) {
		rc = -EFAULT;
		goto done;
	}
	if (data->enable_sensor) {
		rc = -EBUSY;
		dev_err(dev, "can't perform calibration while ranging\n");
		goto done;
	}

	rc = reset_release(data);
	if (rc)
		goto done;

	rc = VL53L1_StaticInit(&data->stdev);
	if (rc) {
		dev_err(dev, "VL53L1_StaticInit fail => %d\n", rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	switch (calib.calibration_type) {
	case VL53L1_CALIBRATION_REF_SPAD:
		rc = ctrl_perform_calibration_ref_spad_lock(data,
			&calib);
		break;
	case VL53L1_CALIBRATION_CROSSTALK:
		rc = ctrl_perform_calibration_crosstalk_lock(data,
			&calib);
		break;
	case VL53L1_CALIBRATION_OFFSET:
		rc = ctrl_perform_calibration_offset_lock(data,
			&calib);
		break;
	case VL53L1_CALIBRATION_OFFSET_SIMPLE:
		rc = ctrl_perform_simple_calibration_offset_lock(data,
			&calib);
		break;
	case VL53L1_CALIBRATION_OFFSET_PER_ZONE:
		rc = ctrl_perform_zone_calibration_offset_lock(data,
			&calib);
		break;
	case VL53L1_CALIBRATION_OFFSET_PER_VCSEL:
		rc = ctrl_perform_per_vcsel_calibration_offset_lock(data,
			&calib);
		break;
	case VL53L1_CALIBRATION_OFFSET_ZERO_DISTANCE:
		rc = ctrl_perform_zero_distance_calibration_offset_lock(data,
			&calib);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	reset_hold(data);

done:
	data->is_calibrating = false;
	data->is_first_start_done = true;
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int stmvl53l1_ioctl_handler(
		struct stmvl53l1_data *data,
		unsigned int cmd, unsigned long arg,
		void __user *p)
{
	int rc = 0;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (!data)
		return -EINVAL;

	switch (cmd) {
	case VL53L1_IOCTL_POWER_UP:
		dev_dbg(dev, "VL53L1_IOCTL_POWER_UP\n");
		rc = ctrl_power_up(data);
		break;

	case VL53L1_IOCTL_POWER_DOWN:
		dev_dbg(dev, "VL53L1_IOCTL_POWER_DOWN\n");
		reset_hold(data);
		rc = ctrl_power_down(data);
		break;

	case VL53L1_IOCTL_START:
		dev_dbg(dev, "VL53L1_IOCTL_START\n");
		rc = ctrl_start(data);
		break;

	case VL53L1_IOCTL_STOP:
		dev_dbg(dev, "VL53L1_IOCTL_STOP\n");
		rc = ctrl_stop(data);
		break;

	case VL53L1_IOCTL_GETDATAS:
		rc = ctrl_getdata(data, p);
		break;

	case VL53L1_IOCTL_GETDATAS_BLOCKING:
		rc = ctrl_getdata_blocking(data, p);
		break;

	/* Register tool */
	case VL53L1_IOCTL_REGISTER:
		dev_dbg(dev, "VL53L1_IOCTL_REGISTER\n");
		reset_release(data);
		rc = ctrl_reg_access(data, p);
		break;

	case VL53L1_IOCTL_PARAMETER:
		dev_dbg(dev, "VL53L1_IOCTL_PARAMETER\n");
		rc = ctrl_params(data, p);
		break;

	case VL53L1_IOCTL_ROI:
		dev_dbg(dev, "VL53L1_IOCTL_ROI\n");
		rc = ctrl_roi(data, p);
		break;
	case VL53L1_IOCTL_MZ_DATA:
		rc = ctrl_mz_data(data, p);
		break;
	case VL53L1_IOCTL_MZ_DATA_BLOCKING:
		rc = ctrl_mz_data_blocking(data, p);
		break;
	case VL53L1_IOCTL_CALIBRATION_DATA:
		dev_dbg(dev, "VL53L1_IOCTL_CALIBRATION_DATA\n");
		rc = ctrl_calibration_data(data, p);
		break;
	case VL53L1_IOCTL_PERFORM_CALIBRATION:
		dev_dbg(dev, "VL53L1_IOCTL_PERFORM_CALIBRATION\n");
		rc = ctrl_perform_calibration(data, p);
		break;
	case VL53L1_IOCTL_AUTONOMOUS_CONFIG:
		dev_dbg(dev, "VL53L1_IOCTL_AUTONOMOUS_CONFIG\n");
		rc = ctrl_autonomous_config(data, p);
		break;
	case VL53L1_IOCTL_ZONE_CALIBRATION_DATA:
		dev_dbg(dev, "VL53L1_IOCTL_ZONE_CALIBRATION_DATA\n");
		rc = ctrl_zone_calibration_data(data, p);
		break;
	case VL53L1_IOCTL_MZ_DATA_ADDITIONAL:
		rc = ctrl_mz_data_additional(data, p);
		break;
	case VL53L1_IOCTL_MZ_DATA_ADDITIONAL_BLOCKING:
		rc = ctrl_mz_data_blocking_additional(data, p);
		break;
	default:
		rc = -ENOTTY;
		break;
	}

	return rc;
}

static int stmvl53l1_open(struct inode *inode, struct file *file)
{
	struct stmvl53l1_data *data = container_of(file->private_data,
		struct stmvl53l1_data, miscdev);

	mutex_lock(&dev_open_mutex);
	stmvl53l1_module_func_tbl.get(data->client_object);
	mutex_unlock(&dev_open_mutex);

	return 0;
}

static int stmvl53l1_release(struct inode *inode, struct file *file)
{
	struct stmvl53l1_data *data = container_of(file->private_data,
		struct stmvl53l1_data, miscdev);
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;
	int rc;

	mutex_lock(&dev_open_mutex);
	/* Return 1 if the object was removed, otherwise return 0 after kref_put */
	if (stmvl53l1_module_func_tbl.put(data->client_object)) {
		mutex_unlock(&dev_open_mutex);
		return 0;
	}
	/*
	 * Kref count showing 1 after kref_put means that thers's no more reference
	 * to this object. Add a check here to ensure sensor is stopped and powered
	 * down properly.
	 */
	if (kref_read(&i2c_data->ref) == 1) {
		if (data->enable_sensor) {
			rc = ctrl_stop(data);
			if (rc) {
				dev_err(dev, "fail to stop sensor, rc %d\n", rc);
			}
		}
		if (data->is_power_up) {
			rc = ctrl_power_down(data);
			if (rc) {
				dev_err(dev, "fail to power down, rc %d\n", rc);
			}
		}
	}
	mutex_unlock(&dev_open_mutex);

	return 0;
}


/** max number or error per measure too abort */
#define stvm531_get_max_meas_err(...) 3
/** max number or error per stream too abort */
#define stvm531_get_max_stream_err(...) 6

static void detect_xtalk_value_change(struct stmvl53l1_data *data,
	struct VL53L1_MultiRangingData_t *meas)
{
	data->is_xtalk_value_changed = meas->HasXtalkValueChanged ? true :
		data->is_xtalk_value_changed;
}

/**
 * handle data retrieval and dispatch
 *
 * work lock must be held
 *
 * called  form work or interrupt thread it must be a blocable context !
 * @param data the device
 */
static void stmvl53l1_on_newdata_event(struct stmvl53l1_data *data)
{
	int rc;
	struct VL53L1_RangingMeasurementData_t *pmsinglerange;
	struct VL53L1_MultiRangingData_t *pmrange;
	struct VL53L1_MultiRangingData_t *tmprange;
	struct VL53L1_TargetRangeData_t RangeData[VL53L1_MAX_RANGE_RESULTS];
	struct VL53L1_RangingMeasurementData_t singledata;
	long ts_msec;
	int i;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	st_gettimeofday(&data->meas.comp_tv);
	ts_msec = stmvl53l1_tv_dif(&data->start_tv, &data->meas.comp_tv) / 1000;

	pmrange = &data->meas.multi_range_data;
	tmprange = &data->meas.tmp_range_data;
	pmsinglerange = &data->meas.single_range_data;

	memcpy(&singledata, pmsinglerange,
			sizeof(struct VL53L1_RangingMeasurementData_t));
	for (i = 0; i < VL53L1_MAX_RANGE_RESULTS; i++)
		memcpy(&RangeData[i], &pmrange->RangeData[i],
				sizeof(struct VL53L1_TargetRangeData_t));

	data->meas.intr++;

	/* use get data method based on active mode */
	switch (data->preset_mode) {
	case VL53L1_PRESETMODE_LITE_RANGING:
	case VL53L1_PRESETMODE_AUTONOMOUS:
	case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
		rc = VL53L1_GetRangingMeasurementData(&data->stdev,
			pmsinglerange);
	break;
	case VL53L1_PRESETMODE_RANGING:
	case VL53L1_PRESETMODE_MULTIZONES_SCANNING:
	/* IMPORTANT : during VL53L1_GetMultiRangingData() call
	 * work_mutex is release during ipp. This is why we use
	 * tmp_range_data which is not access somewhere else. When we are
	 * back we then copy tmp_range_data in multi_range_data.
	 */

		rc = VL53L1_GetMultiRangingData(&data->stdev,
			&data->meas.tmp_range_data);
		if (rc)
			dev_err(dev,
				"VL53L1_GetRangingMeasurementData @%d %d\n",
				__LINE__, rc);

		/* be sure we got VL53L1_RANGESTATUS_NONE for object 0 if we got
		 * invalid roi or no object. So if client read data using
		 * VL53L1_IOCTL_GETDATAS we got correct status.
		 */
		if (tmprange->RoiStatus == VL53L1_ROISTATUS_NOT_VALID ||
			tmprange->NumberOfObjectsFound == 0)
			tmprange->RangeData[0].RangeStatus =
						VL53L1_RANGESTATUS_NONE;

		memcpy(pmrange, tmprange,
		       sizeof(struct VL53L1_MultiRangingData_t));

		/* got histogram debug data in case user want it later on */
		if (!rc)
			rc = VL53L1_GetAdditionalData(&data->stdev,
				&data->meas.additional_data);
		detect_xtalk_value_change(data, pmrange);
		break;
	default:
		/* that must be some bug or data corruption stop now */
		rc = -1;
		dev_err(dev, "unsorted mode %d=>stop\n", data->preset_mode);
		_ctrl_stop(data);
	}
	/* check if not stopped yet
	 * as we may have been unlocked we must re-check
	 */
	if (data->enable_sensor == 0) {
		dev_dbg(dev, "at meas #%d we got stopped\n", data->meas.cnt);
		return;
	}
	if (rc) {
		dev_err(dev, "VL53L1_GetRangingMeasurementData @%d %d\n",
			__LINE__, rc);
		data->meas.err_cnt++;
		data->meas.err_tot++;
		if (data->meas.err_cnt > stvm531_get_max_meas_err(data) ||
			data->meas.err_tot > stvm531_get_max_stream_err(data)) {
			dev_err(dev, "on #%d %d err %d tot stop\n",
				data->meas.cnt, data->meas.err_cnt,
				data->meas.err_tot);
			_ctrl_stop(data);
		}
		return;
	}

	/* FIXME: remove when implemented by ll or bare driver */
	pmrange->TimeStamp = ts_msec;
	pmsinglerange->TimeStamp = ts_msec;
	for (i = 1; i < pmrange->NumberOfObjectsFound; i++)
		pmrange->TimeStamp = ts_msec;

	data->meas.cnt++;
	dev_dbg(dev, "#%3d %2d poll ts %5d status=%d obj cnt=%d\n",
		data->meas.cnt,
		data->meas.poll_cnt,
		pmrange->TimeStamp,
		pmrange->RangeData[0].RangeStatus,
		pmrange->NumberOfObjectsFound);

	/* ready that is not always on each new data event */

	/* mark data as valid from now */
	data->is_data_valid = true;

	/* wake up sleeping client */
	wake_up_data_waiters(data);

	/* push data to input subsys and only and make val for ioctl*/
	stmvl53l1_input_push_data(data);
	stmvl53l1_insert_flush_events_lock(data);

	/* roll time now data got used */
	data->meas.start_tv = data->meas.comp_tv;
	data->meas.poll_cnt = 0;
	data->meas.err_cnt = 0;
}


/**
 * * handle interrupt/pusdo irq by polling handling
 *
 * work lock must be held
 *
 * @param data driver
 * @return 0 on success
 */
static int stmvl53l1_intr_process(struct stmvl53l1_data *data)
{
	uint8_t data_rdy;
	int rc = 0;
	struct timespec64 tv_now;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	if (!data->enable_sensor)
		goto done;

	if (data->ulp_enable) {
		rc = VL53L1_ULP_Interrupt(&data->stdev);
		data->is_data_valid = true;
		wake_up_data_waiters(data);
		goto done;
	}

	data->meas.poll_cnt++;
	rc = VL53L1_GetMeasurementDataReady(&data->stdev, &data_rdy);
	if (rc) {
		dev_err(dev, "GetMeasurementDataReady @%d %d, fail\n",
			__LINE__, rc);
		/* too many successive fail => stop but do not try to do any new
		 * i/o
		 */
		goto stop_io;
	}

	if (!data_rdy) {
		/* FIXME this part to completely skip
		 * if using interrupt and sure we have
		 * no false interrupt to handle or no to do any timing check
		 */
		long poll_us;

		st_gettimeofday(&tv_now);
		poll_us = stmvl53l1_tv_dif(&data->meas.start_tv, &tv_now);
		if (poll_us > data->timing_budget * 4) {
			dev_err(dev, "we're polling %ld ms too long\n",
				poll_us / 1000);
			/*  fixme stop or just warn ? */
			goto stop_io;
		}
		/*  keep trying it could be intr with no processing */
		work_dbg("intr with no data rdy");
		goto done;
	}
	/* we have data to handle */
	/* first irq after reset has no data so we skip it */
	if (data->is_first_irq) {
		data->is_first_irq = false;

		if (data->preset_mode ==
			VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS) {
			/*
			 * If VL53L1_GetRangingMeasurementData()
			 * is not called after
			 * for the first ranging measurement,
			 * the thresholds do not seem
			 * to work for ALP mode
			 */
			struct VL53L1_RangingMeasurementData_t
				RangingMeasurementData;

			/* printk("Test Workaround for ALP mode\n"); */
			VL53L1_GetRangingMeasurementData(&data->stdev,
				&RangingMeasurementData);
		}

	} else
		stmvl53l1_on_newdata_event(data);
	/* enable_sensor could change on event handling check again */
	if (data->enable_sensor) {
		/* clear interrupt and continue ranging */
		work_dbg("intr clr");
		/* In autonomous mode, bare driver will trigger stop/start
		 * sequence. In that case it wall call platform delay functions.
		 * So allow delay in VL53L1_ClearInterruptAndStartMeasurement()
		 * call.
		 */
		data->is_delay_allowed = data->allow_hidden_start_stop;
		rc = VL53L1_ClearInterruptAndStartMeasurement(&data->stdev);
		data->is_delay_allowed = 0;
		if (rc) {
			/* go to stop but stop any new i/o for dbg */
			dev_err(dev, "Cltr intr restart fail %d\n", rc);
			goto stop_io;
		}
	}
done:
	return rc;
stop_io:
	/* too many successive fail take action => stop but do not try to do
	 * any new i/o
	 */
	dev_err(dev, "GetDatardy fail stop\n");
	_ctrl_stop(data);
	return rc;

}

static void stmvl53l1_work_handler(struct work_struct *work)
{
	struct stmvl53l1_data *data;

	data = container_of(work, struct stmvl53l1_data, dwork.work);
	work_dbg("enter");
	mutex_lock(&data->work_mutex);
	stmvl53l1_intr_process(data);
	if (data->poll_mode && data->enable_sensor) {
		/* re-sched ourself */
		schedule_delayed_work(&data->dwork,
			msecs_to_jiffies(data->poll_delay_ms));
	}
	mutex_unlock(&data->work_mutex);
}

static void stmvl53l1_input_push_data_singleobject(struct stmvl53l1_data *data)
{
	struct input_dev *input = data->input_dev_ps;
	struct VL53L1_RangingMeasurementData_t *meas =
		&data->meas.single_range_data;
	FixPoint1616_t LimitCheckCurrent;
	VL53L1_Error st = VL53L1_ERROR_NONE;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	dev_dbg(dev, "******* FIXME!!! ************\n");
	dev_dbg(dev, "Sensor HAL in Lite ranging mode not yet updated\n");
	dev_dbg(dev, "******* FIXME!!! ************\n");

	/* Do not send the events till this if fixed properly */
	return;

	input_report_abs(input, ABS_DISTANCE, (meas->RangeMilliMeter + 5) / 10);
	input_report_abs(input, ABS_HAT0X, meas->TimeStamp / 1000);
	input_report_abs(input, ABS_HAT0Y, (meas->TimeStamp % 1000) * 1000);
	input_report_abs(input, ABS_HAT1X, meas->RangeMilliMeter);
	input_report_abs(input, ABS_HAT1Y, meas->RangeStatus);
	input_report_abs(input, ABS_HAT2X, meas->SignalRateRtnMegaCps);
	input_report_abs(input, ABS_HAT2Y, meas->AmbientRateRtnMegaCps);
	input_report_abs(input, ABS_HAT3X, meas->SigmaMilliMeter);
	st = VL53L1_GetLimitCheckCurrent(&data->stdev,
		VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, &LimitCheckCurrent);
	if (st == VL53L1_ERROR_NONE)
		input_report_abs(input, ABS_WHEEL, LimitCheckCurrent);
	input_report_abs(input, ABS_TILT_Y, meas->EffectiveSpadRtnCount);
	input_report_abs(input, ABS_TOOL_WIDTH, meas->RangeQualityLevel);

	input_sync(input);
}

static void stmvl53l1_input_push_data_multiobject(struct stmvl53l1_data *data)
{
	struct VL53L1_MultiRangingData_t *mmeas = &data->meas.multi_range_data;
	int i;
	int rc;
	struct VL53L1_TargetRangeData_t *meas_array[4];
	struct VL53L1_CalibrationData_t calibration_data;
	struct timespec64 tv;
	struct input_dev *input = data->input_dev_ps;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	st_gettimeofday(&tv);

	for (i = 0; i < 4; i++)
		meas_array[i] = &mmeas->RangeData[i];

	/*************************************************************
	 *  INPUT EVENT CODE	L1/L3 Data
	   ABS_HAT0X	Time in Sec(32)
	   ABS_HAT0Y	Time in uSec(32)
	   ABS_HAT1X	Obj0_Distance(16) :  Obj0_Sigma(16)
	   ABS_HAT1Y	Obj0_MinRange(16) : Obj0_MaxRange(16)
	   ABS_HAT2X	Obj1_Distance(16) :  Obj1_Sigma(16)
	   ABS_HAT2Y	Obj1_ MinRange (16) : Obj1_ MaxRange (16)
	   ABS_HAT3X	Obj0_SignalRate_Spad(32)
	   ABS_HAT3Y	Obj1_SignalRate_Spad(32)
	   ABS_WHEEL	AmbientRate(32)
	   ABS_BRAKE	EffectiveSpadRtnCount(16):RangeStatus_1(8):
					Range_status_0(8)
	   ABS_TILT_X	XtalkChange(8) :StreamCount(8) :
					NumberofObjects(2) : RoiNumber(4) :
					RoiStatus(2)
	   ABS_TILT_Y	DMAX
	   ABS_TOOL_WIDTH	XtalkValue
	   ABS_DISTANCE
	   ABS_THROTTLE
	   ABS_RUDDER
	   ABS_MISC
	   ABS_VOLUME
	 ************************************************************/

	rc = VL53L1_GetCalibrationData(&data->stdev, &calibration_data);
	if (rc) {
		/* This should not happen */
		dev_err(dev, "%d error:%d\n", __LINE__, rc);
		return;
	}

	/* ABS_HAT0X  -	Time in Sec(32) */

	input_report_abs(input, ABS_HAT0X, tv.tv_sec);
	dev_dbg(dev, "ABS_HAT0X : %lld, %zu\n", tv.tv_sec, sizeof(tv.tv_sec));
	/* ABS_HAT0Y   - Time in uSec(32) */
	/* REVISIT : The following code may cause loss of data due to */
	/* 8 bytes to 32 bits conversion */
	input_report_abs(input, ABS_HAT0Y, tv.tv_nsec);
	dev_dbg(dev, "ABS_HAT0Y : %ld\n", tv.tv_nsec);

	/* ABS_WHEEL - AmbientRate(32) */
	input_report_abs(input, ABS_WHEEL,
		meas_array[0]->AmbientRateRtnMegaCps);
	dev_dbg(dev, "ABS_WHEEL : AmbRate = %d\n",
		meas_array[0]->AmbientRateRtnMegaCps);

	/* ABS_TILT_X	XtalkChange(8) :StreamCount(8) : */
	/* Number of Objects(2) : RoiNumber(4) : RoiStatus(2) */
	input_report_abs(input, ABS_TILT_X,
			 (mmeas->HasXtalkValueChanged << 16) |
				 (mmeas->StreamCount << 8) |
				 ((mmeas->NumberOfObjectsFound & 0x3) << 6) |
				 ((mmeas->RoiNumber & 0xF) << 2) |
				 (mmeas->RoiStatus & 0x3));
	dev_dbg(dev, "ABS_TILT_X :(%d):(%d):(%d):(%d):(%d)\n\n",
		mmeas->HasXtalkValueChanged,
		mmeas->StreamCount,
		mmeas->NumberOfObjectsFound,
		mmeas->RoiNumber,
		mmeas->RoiStatus);

	/* ABS_TILT_Y	DMAX */
	input_report_abs(input, ABS_TILT_Y, mmeas->DmaxMilliMeter);
	dev_dbg(dev, "ABS_TILT_Y DMAX = %d\n", mmeas->DmaxMilliMeter);

	/* ABS_TOOL_WIDTH */
	input_report_abs(
		input, ABS_TOOL_WIDTH,
		calibration_data.customer
			.algo__crosstalk_compensation_plane_offset_kcps);
	dev_dbg(dev, "ABS_TOOL_WIDTH Xtalk = %d\n",
		calibration_data.customer
			.algo__crosstalk_compensation_plane_offset_kcps);

	/* ABS_BRAKE  -	EffectiveSpadRtnCount(16):RangeStatus_3(1): */
	/* Range_status_2(0) */
	input_report_abs(input, ABS_BRAKE,
			 mmeas->EffectiveSpadRtnCount << 16
			| ((meas_array[1]->RangeStatus) << 8)
			| meas_array[0]->RangeStatus);

	dev_dbg(dev, "ABS_BRAKE : (%d):(%d):(%d)\n",
		mmeas->EffectiveSpadRtnCount,
		meas_array[1]->RangeStatus,
		meas_array[0]->RangeStatus);

	dev_dbg(dev, "ABS_BRAKE : 0x%X\n",
		(mmeas->EffectiveSpadRtnCount & 0xFFFF) << 16 |
			((meas_array[1]->RangeStatus) << 8) |
				meas_array[0]->RangeStatus);

	/*  Remaining of data are meaningless in case of no target */
	if (mmeas->NumberOfObjectsFound == 0) {
		input_sync(input);
		return;
	}

	/* ABS_HAT1X   -	 Obj0_Distance(16) :  Obj0_Sigma(16) */
	input_report_abs(input, ABS_HAT1X, meas_array[0]->RangeMilliMeter << 16
			| (meas_array[0]->SigmaMilliMeter/65536));
	dev_dbg(dev, "ABS_HAT1X : 0x%X(%d:%d)\n",
		meas_array[0]->RangeMilliMeter << 16 |
			(meas_array[0]->SigmaMilliMeter/65536),
		meas_array[0]->RangeMilliMeter,
		(meas_array[0]->SigmaMilliMeter/65536));

	/* ABS_HAT1Y   -	Obj0_MinRange(16) : Obj0_MaxRange(16) */
	input_report_abs(input, ABS_HAT1Y,
			meas_array[0]->RangeMinMilliMeter << 16
			| meas_array[0]->RangeMaxMilliMeter);

	dev_dbg(dev, "ABS_HAT1Y : 0x%X(%d:%d)\n",
		meas_array[0]->RangeMinMilliMeter << 16 |
			meas_array[0]->RangeMaxMilliMeter,
		meas_array[0]->RangeMinMilliMeter,
		meas_array[0]->RangeMaxMilliMeter);

	if (mmeas->NumberOfObjectsFound > 1) {
		/* ABS_HAT2X   -	Obj1_Distance(16) :  Obj1_Sigma(16) */
		input_report_abs(input, ABS_HAT2X,
				meas_array[1]->RangeMilliMeter << 16
				| (meas_array[1]->SigmaMilliMeter/65536));
		dev_dbg(dev, "ABS_HAT2X : 0x%x(%d:%d)\n",
			meas_array[1]->RangeMilliMeter << 16 |
				(meas_array[1]->SigmaMilliMeter/65536),
			meas_array[1]->RangeMilliMeter,
			(meas_array[1]->SigmaMilliMeter/65536));

		/* ABS_HAT2Y - Obj1_ MinRange (16) : Obj1_ MaxRange (16) */
		input_report_abs(input, ABS_HAT2Y,
				meas_array[1]->RangeMinMilliMeter << 16
				| meas_array[1]->RangeMaxMilliMeter);

		dev_dbg(dev, "ABS_HAT1Y : 0x%X(%d:%d)\n",
			meas_array[1]->RangeMinMilliMeter << 16 |
				meas_array[1]->RangeMaxMilliMeter,
			meas_array[1]->RangeMinMilliMeter,
			meas_array[1]->RangeMaxMilliMeter);
	}

	/* ABS_HAT3X - Obj0_SignalRate_Spad(32) */
	input_report_abs(input, ABS_HAT3X,
			meas_array[0]->SignalRateRtnMegaCps);
	dev_dbg(dev, "ABS_HAT3X : SignalRateRtnMegaCps_0(%d)\n",
		meas_array[0]->SignalRateRtnMegaCps);
	if (mmeas->NumberOfObjectsFound > 1) {
		/* ABS_HAT3Y  - Obj1_SignalRate_Spad(32) */
		input_report_abs(input, ABS_HAT3Y,
			meas_array[1]->SignalRateRtnMegaCps);
		dev_dbg(dev, "ABS_HAT3Y : SignalRateRtnMegaCps_1(%d)\n",
			meas_array[1]->SignalRateRtnMegaCps);
	}

	input_sync(input);


}

static void stmvl53l1_input_push_data(struct stmvl53l1_data *data)
{
	/* use get data method based on active mode */
	switch (data->preset_mode) {
	case VL53L1_PRESETMODE_LITE_RANGING:
	case VL53L1_PRESETMODE_AUTONOMOUS:
	case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
		stmvl53l1_input_push_data_singleobject(data);
		break;
	default:
	/* VL53L1_PRESETMODE_RANGING:
	 * VL53L1_PRESETMODE_MULTIZONES_SCANNING:
	 */
		stmvl53l1_input_push_data_multiobject(data);
	}
}

static int stmvl53l1_input_setup(struct stmvl53l1_data *data)
{
	int rc;
	struct input_dev *idev;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	/* Register to Input Device */
	idev = input_allocate_device();
	if (idev == NULL) {
		rc = -ENOMEM;
		dev_err(dev, "%d error:%d\n", __LINE__, rc);
		goto exit_err;
	}
	/*  setup all event */
	set_bit(EV_ABS, idev->evbit);

	input_set_abs_params(idev, ABS_DISTANCE, 0, 0xff, 0, 0);

	input_set_abs_params(idev, ABS_HAT0X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT0Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT1X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT1Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT2X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT2Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT3X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT3Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_WHEEL, 0, 0xffffffff, 0, 0);

	input_set_abs_params(idev, ABS_TILT_Y, 0, 0xffffffff, 0, 0);

	input_set_abs_params(idev, ABS_BRAKE, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_TILT_X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_TOOL_WIDTH, 0, 0xffffffff, 0, 0);

	input_set_abs_params(idev, ABS_THROTTLE, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_RUDDER, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_MISC, 0, 0xffffffff, 0, 0);

	input_set_abs_params(idev, ABS_VOLUME, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_GAS, 0, 0xffffffff, 0, 0);

	idev->name = "STM VL53L1 proximity sensor";
	idev->dev.groups = stmvl53l1_attr_groups;
	rc = input_register_device(idev);
	if (rc) {
		rc = -ENOMEM;
		dev_err(dev, "%d error:%d\n", __LINE__, rc);
		goto exit_free_dev_ps;
	}
	/* setup drv data */
	input_set_drvdata(idev, data);
	data->input_dev_ps = idev;
	return 0;


exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_err:
	return rc;
}

/**
 * handler to be called by interface module on interrupt
 *
 * managed poll/irq filtering in case poll/irq can be soft forced
 * and the module side still fire interrupt
 *
 * @param data
 * @return 0 if all ok else for error
 */
int stmvl53l1_intr_handler(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = data->client_object;
	struct device *dev = &i2c_data->client->dev;

	mutex_lock(&data->work_mutex);

	/* handle it only if we are not stopped */
	if (data->enable_sensor) {
		rc = stmvl53l1_intr_process(data);
	} else {
		/* it's likely race/last unhandled interrupt after
		 * stop.
		 * Such complex irq also occurred during offset and crosstalk
		 * calibration procedures.
		 */
		dev_dbg(dev, "got intr but not on (complex or calibration)\n");
		rc = 0;
	}

	mutex_unlock(&data->work_mutex);
	return rc;
}


/**
 * One time device  setup
 *
 * call by bus (i2c/cci) level probe to finalize non bus related device setup
 *
 * @param	data The device data
 * @return	0 on success
 */
int stmvl53l1_setup(struct stmvl53l1_data *data)
{
	int rc = 0;
	uint8_t device_id, device_type;
	struct VL53L1_DeviceInfo_t dev_info;
	struct i2c_data *i2c_data = data->client_object;
	struct device *dev = &i2c_data->client->dev;

	/* acquire an id */
	data->id = allocate_dev_id();
	if (data->id < 0) {
		dev_err(dev, "too many device already created\n");
		return -1;
	}
	dev_dbg(dev, "Dev id %d is @%p\n", data->id, data);
	stmvl53l1_dev_table[data->id] = data;

	/* init mutex */
	/* mutex_init(&data->update_lock); */
	mutex_init(&data->work_mutex);

	/* init work handler */
	INIT_DELAYED_WORK(&data->dwork, stmvl53l1_work_handler);

	data->force_device_on_enable = false/*force_device_on_en_default*/;
	data->reset_state = 1;
	data->is_calibrating = false;
	data->last_error = VL53L1_ERROR_NONE;
	data->is_device_remove = false;
	data->is_power_up = false;

	rc = stmvl53l1_module_func_tbl.power_up(data->client_object);
	if (rc) {
		dev_err(dev, "%d, error rc %d\n", __LINE__, rc);
		goto exit_ipp_cleanup;
	}
	rc = reset_release(data);
	if (rc)
		goto exit_ipp_cleanup;

	rc = stmvl53l1_input_setup(data);
	if (rc)
		goto exit_ipp_cleanup;

	/* init blocking ioctl stuff */
	INIT_LIST_HEAD(&data->simple_data_reader_list);
	INIT_LIST_HEAD(&data->mz_data_reader_list);
	init_waitqueue_head(&data->waiter_for_data);
	data->is_data_valid = false;

	data->enable_sensor = 0;

	data->poll_delay_ms = STMVL53L1_CFG_POLL_DELAY_MS;
	data->timing_budget = STMVL53L1_CFG_TIMING_BUDGET_US;
	data->preset_mode = STMVL53L1_CFG_DEFAULT_MODE;
	data->distance_mode = STMVL53L1_CFG_DEFAULT_DISTANCE_MODE;
	data->crosstalk_enable = STMVL53L1_CFG_DEFAULT_CROSSTALK_ENABLE;
	data->output_mode = STMVL53L1_CFG_DEFAULT_OUTPUT_MODE;
	data->offset_correction_mode =
		STMVL53L1_CFG_DEFAULT_OFFSET_CORRECTION_MODE;
	stmvl53l1_setup_auto_config(data);
	data->dmax_mode = STMVL53L1_CFG_DEFAULT_DMAX_MODE;
	data->smudge_correction_mode =
		STMVL53L1_CFG_DEFAULT_SMUDGE_CORRECTION_MODE;
	data->current_roi_id = 0;
	data->is_xtalk_value_changed = false;
	data->ulp_enable = false;
	data->macro_timing = 1;
	data->inter_measurement = 1000;
	data->sigma_threshold = 45;
	data->signal_threshold = 1500;

	data->is_delay_allowed = true;
	/* need to be done once */
	rc = VL53L1_DataInit(&data->stdev);
	data->is_delay_allowed = false;
	if (rc) {
		dev_err(dev, "VL53L1_DataInit %d\n", rc);
		goto exit_unregister_dev_ps;
	}

	rc = VL53L1_GetDeviceInfo(&data->stdev, &dev_info);
	if (rc) {
		dev_err(dev, "VL53L1_GetDeviceInfo %d\n", rc);
		goto exit_unregister_dev_ps;
	}
	dev_info(dev, "device name %s type %s\n", dev_info.Name, dev_info.Type);
	data->product_type = dev_info.ProductType;

	rc = VL53L1_RdByte(&data->stdev, VL53L1_IDENTIFICATION__MODEL_ID,
			   &device_id);
	if (rc)
		dev_err(dev, "Failed to read sensor id %d\n", rc);
	else
		dev_info(dev, "Probe Success, device id 0x%x\n", device_id);

	rc = VL53L1_RdByte(&data->stdev, VL53L1_IDENTIFICATION__MODULE_TYPE,
			&device_type);
	if (rc)
		dev_err(dev, "Failed to read device type %d\n", rc);
	else
		dev_info(dev, "Probe Success, device type 0x%x\n", device_type);

	/* get managed data here */
	rc = VL53L1_GetDmaxReflectance(&data->stdev, &data->dmax_reflectance);
	if (rc) {
		dev_err(dev, "VL53L1_GetDmaxReflectance %d\n", rc);
		goto exit_unregister_dev_ps;
	}
	rc = VL53L1_GetOpticalCenter(&data->stdev, &data->optical_offset_x,
				     &data->optical_offset_y);
	if (rc) {
		dev_err(dev, "VL53L1_GetOpticalCenter %d\n", rc);
		goto exit_unregister_dev_ps;
	}

	/* set tuning from stmvl53l1_tunings.h */
	rc = setup_tunings(data);
	if (rc) {
		dev_err(dev, "setup_tunings %d\n", rc);
		goto exit_unregister_dev_ps;
	}

	/* Set special parameters for VL53L3 (ticket 513812) */
	if (dev_info.ProductType == 0xAA) {
		data->timing_budget = 30000;
		data->crosstalk_enable = 1;
		data->dmax_mode = VL53L1_DMAXMODE_CUSTCAL_DATA;
		data->smudge_correction_mode =
			VL53L1_SMUDGE_CORRECTION_CONTINUOUS;
		data->dmax_reflectance = (5 << 16);
	}
	/* End of Set special parameters for VL53L3 (ticket 513812) */

	/* if working in interrupt ask intr to enable and hook the handler */
	data->poll_mode = 0;
	rc = stmvl53l1_module_func_tbl.start_intr(data->client_object,
						  &data->poll_mode);
	if (rc < 0) {
		dev_err(dev, "can't start intr\n");
		goto exit_unregister_dev_ps;
	}

	data->is_first_irq = true;
	data->is_first_start_done = false;
	data->is_delay_allowed = false;

	/* to register as a misc device */
	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	/* multiple dev name use id in name but 1st */
	if (data->id == 0)
		strcpy(data->name, VL53L1_MISC_DEV_NAME);
	else
		sprintf(data->name, "%s%d", VL53L1_MISC_DEV_NAME, data->id);

	data->miscdev.name = data->name;
	data->miscdev.fops = &stmvl53l1_ranging_fops;
	dev_info(dev, "Misc device registration name:%s\n", data->miscdev.name);
	rc = misc_register(&data->miscdev);
	if (rc != 0) {
		dev_err(dev, "misc dev reg fail\n");
		goto exit_unregister_dev_ps;
	}
	/* bring back device under reset */
	reset_hold(data);

	/* power down after probe done */
	stmvl53l1_module_func_tbl.power_down(data->client_object);

	return 0;

exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_ipp_cleanup:
	/* power down if probe failed */
	stmvl53l1_module_func_tbl.power_down(data->client_object);
	return rc;
}


void stmvl53l1_cleanup(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	rc = _ctrl_stop(data);
	if (rc < 0)
		dev_err(dev, "stop failed %d aborting anyway\n", rc);

	if (data->input_dev_ps) {
		dev_dbg(dev, "to unregister input dev\n");
		input_unregister_device(data->input_dev_ps);
	}

	if (!IS_ERR(data->miscdev.this_device) &&
			data->miscdev.this_device != NULL) {
		dev_dbg(dev, "to unregister misc dev\n");
		misc_deregister(&data->miscdev);
	}

	/* be sure device is put under reset */
	data->force_device_on_enable = false;
	reset_hold(data);
	stmvl53l1_module_func_tbl.power_down(data->client_object);

	deallocate_dev_id(data->id);
	data->is_device_remove = true;
}

#ifdef CONFIG_PM_SLEEP
void stmvl53l1_pm_suspend_stop(struct stmvl53l1_data *data)
{
	int rc;
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;
	struct device *dev = &i2c_data->client->dev;

	rc = _ctrl_stop(data);
	if (rc < 0)
		dev_err(dev, "stop failed %d aborting anyway\n", rc);
}
#endif

static long stmvl53l1_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	long ret;
	struct stmvl53l1_data *data =
			container_of(file->private_data,
				struct stmvl53l1_data, miscdev);
	ret = stmvl53l1_ioctl_handler(data, cmd, arg, (void __user *)arg);
	return ret;
}

static int __init stmvl53l1_init(void)
{
	int rc = -1;

	/* i2c/cci client specific init function */
	rc = stmvl53l1_module_func_tbl.init();

	return rc;
}

static void __exit stmvl53l1_exit(void)
{
	stmvl53l1_module_func_tbl.deinit(NULL);
	if (stmvl53l1_module_func_tbl.clean_up != NULL)
		stmvl53l1_module_func_tbl.clean_up();
}

MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(stmvl53l1_init);
module_exit(stmvl53l1_exit);
