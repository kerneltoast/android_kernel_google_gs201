/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Google Whitechapel Audio Metrics Driver
 *
 * Copyright (c) 2021 Google LLC
 *
 */


#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "audiometrics.h"

#define DRIVER_NAME "audiometrics"

#define SPEAKER_MAX_COUNT 4
#define AUDIOMETRIC_CH_LENGTH 16
#define AMCS_MAX_MINOR (1U)
#define AMCS_CDEV_NAME "amcs"
#define VOLUME_INDEX_MAX 10
#define MAX_WAVES_INSTANCE 5
#define ADAPTED_INFO_FEATURES_MAX 6
#define CODEC_MAX_COUNT 5
#define PCM_TYPE_COUNT_MAX 19
#define VOICE_TYPE_MAX 2
#define VOICE_DEVICE_RX_TYPE 8
#define VOICE_NOISE_LEVEL_MAX 12
#define OFFLOAD_EFFECTS_COUNT_MAX 16
#define OFFLOAD_EFFECTS_UUID_LENGTH 4
#define DSP_RECORD_TYPE_COUNT_MAX 20
#define CCA_SOURCE_MAX 2
#define CCA_SOURCE_VOICE 1

static struct platform_device *amcs_pdev;

/*
 * DSP Speech Up/Down time and counts
 */
struct wdsp_stat_priv_type {
	ktime_t ktime_zero;
	ktime_t uptime;
	ktime_t crashtime;
	s64 total_uptime;
	s64 total_downtime;
	u64 crash_count;
	u64 recover_count;
	uint32_t action;
	struct mutex lock;
};

/*
 * ToDo: Reserve a fd handle for audio metric logging ring buffer.
 */
struct amcs_ion_handle_type {
	__u32 handle;
	__s32 fd;
};

 /*
  * Audio Suez related metrics
  */
struct audio_sz_type {
	int32_t codec_state;
	int32_t codec_crashed_counter;
	int32_t hs_codec_state;
	int32_t hs_codec_crashed_counter;
	int32_t speaker_impedance[SPEAKER_MAX_COUNT];
	int32_t speaker_temp[SPEAKER_MAX_COUNT];
	int32_t speaker_excursion[SPEAKER_MAX_COUNT];
	int32_t speaker_heartbeat[SPEAKER_MAX_COUNT];
	char hwinfo_part_number[AUDIOMETRIC_CH_LENGTH];
	struct wdsp_stat_priv_type wdsp_stat_priv;
	uint32_t mic_broken_degrade;
	uint32_t ams_count;
	uint32_t cs_count;
	uint32_t cca_active[CCA_SOURCE_MAX];
	uint32_t cca_enable[CCA_SOURCE_MAX];
	uint32_t cca_cs[CCA_SOURCE_MAX];
	pdm_callback pdm_cb;
	uint32_t pdm_number;
	void* pdm_priv;
	int32_t waves_volume_ms_per_day[MAX_WAVES_INSTANCE][VOLUME_INDEX_MAX];
	uint32_t adapted_info_active_count_per_day[ADAPTED_INFO_FEATURES_MAX];
	uint32_t adapted_info_active_duration_ms_per_day[ADAPTED_INFO_FEATURES_MAX];
	int32_t bt_active_duration[CODEC_MAX_COUNT];
	int32_t pcm_latency_sum[PCM_TYPE_COUNT_MAX];
	int32_t pcm_latency_count[PCM_TYPE_COUNT_MAX];
	int32_t pcm_active_count[PCM_TYPE_COUNT_MAX];
	int32_t voice_noise_duration[VOICE_TYPE_MAX][VOICE_DEVICE_RX_TYPE][VOICE_NOISE_LEVEL_MAX];
	int32_t effect_uuid[OFFLOAD_EFFECTS_COUNT_MAX][OFFLOAD_EFFECTS_UUID_LENGTH];
	int32_t effect_active_seconds_per_day[OFFLOAD_EFFECTS_COUNT_MAX];
	int32_t offload_effects_count;
	int32_t dsp_usage_count[DSP_RECORD_TYPE_COUNT_MAX];
	int32_t dsp_usage_duration[DSP_RECORD_TYPE_COUNT_MAX];
	int32_t voice_call_count;
	int32_t voip_call_count;
	int32_t hal_restart_count;
	int32_t dsp_restart_count;
};

struct audiometrics_priv_type {
	struct cdev cdev;
	dev_t amcs_dev;
	struct class *class;
	struct device *device;
	struct audio_sz_type sz;
	/* TODO: audiohal logger ring buffers */
	struct amcs_ion_handle_type amcs_ion;
	struct mutex lock;
	int amcs_major;
};

static void amcs_report_mic_uevent(uint32_t mic_state, struct audiometrics_priv_type *priv)
{
	char event[25];
	char *env[] = { event, NULL };

	const uint8_t mic_break = FIELD_GET(MIC_BREAK_STAT_MIC_BREAK_MASK, mic_state);
	const uint8_t mic_degrade = FIELD_GET(MIC_BREAK_STAT_MIC_DEGRADE_MASK, mic_state);

	if (IS_ERR_OR_NULL(priv))
		return;

	if (IS_ERR_OR_NULL(priv->device))
		return;

	if (mic_break) {
		snprintf(event, sizeof(event), "MIC_BREAK_STATUS=%hhu",
			 mic_break);
		kobject_uevent_env(&priv->device->kobj, KOBJ_CHANGE, env);
	}

	if (mic_degrade) {
		snprintf(event, sizeof(event), "MIC_DEGRADE_STATUS=%hhu",
			 mic_degrade);
		kobject_uevent_env(&priv->device->kobj, KOBJ_CHANGE, env);
	}

	dev_dbg(&amcs_pdev->dev, "%s: (%d, %d)", __func__, mic_break, mic_degrade);
}

static ssize_t codec_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = NULL;
	int counts = 0;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -EINVAL;

	mutex_lock(&priv->lock);
	counts = scnprintf(buf, PAGE_SIZE, "%d", priv->sz.codec_state);
	mutex_unlock(&priv->lock);

	dev_dbg(dev, "%s: %s\n", __func__, buf);

	return counts;
}

static ssize_t hs_codec_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = NULL;
	int counts = 0;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -EINVAL;

	mutex_lock(&priv->lock);
	counts = scnprintf(buf, PAGE_SIZE, "%d", priv->sz.hs_codec_state);
	mutex_unlock(&priv->lock);

	dev_dbg(dev, "%s: %s\n", __func__, buf);

	return counts;
}


static ssize_t speaker_impedance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = NULL;
	int i, length;
	int scale = 100000;

	if (IS_ERR_OR_NULL(dev))
		return -ENODEV;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -ENODEV;

	mutex_lock(&priv->lock);

	length = 0;
	for (i = 0; i < SPEAKER_MAX_COUNT; i++) {
		if (priv->sz.speaker_impedance[i] < 0)
			continue;

		length += scnprintf(buf + length, PAGE_SIZE - length, "%.*s%d.%05d", i, ",",
				priv->sz.speaker_impedance[i] / scale,
				priv->sz.speaker_impedance[i] % scale);
	}

	mutex_unlock(&priv->lock);
	return length;
}

static ssize_t speaker_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = NULL;
	int i, length;
	int scale = 100000;

	if (IS_ERR_OR_NULL(dev))
		return -ENODEV;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -ENODEV;

	mutex_lock(&priv->lock);

	length = 0;
	for (i = 0; i < SPEAKER_MAX_COUNT; i++) {
		if (priv->sz.speaker_temp[i] < 0)
			continue;

		length += scnprintf(buf + length, PAGE_SIZE - length, "%.*s%d.%05d", i, ",",
				priv->sz.speaker_temp[i] / scale,
				priv->sz.speaker_temp[i] % scale);
	}

	mutex_unlock(&priv->lock);
	return length;
}


static ssize_t speaker_excursion_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = NULL;
	int i, length;
	int scale = 100000;

	if (IS_ERR_OR_NULL(dev))
		return -ENODEV;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -ENODEV;

	mutex_lock(&priv->lock);

	length = 0;
	for (i = 0; i < SPEAKER_MAX_COUNT; i++) {
		if (priv->sz.speaker_excursion[i] < 0)
			continue;

		length += scnprintf(buf + length, PAGE_SIZE - length, "%.*s%d.%05d", i, ",",
				priv->sz.speaker_excursion[i] / scale,
				priv->sz.speaker_excursion[i] % scale);
	}

	mutex_unlock(&priv->lock);
	return length;

}


static ssize_t speaker_heartbeat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = NULL;
	int i, length;

	if (IS_ERR_OR_NULL(dev))
		return -ENODEV;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -ENODEV;

	mutex_lock(&priv->lock);

	length = 0;
	for (i = 0; i < SPEAKER_MAX_COUNT; i++) {
		if (priv->sz.speaker_heartbeat[i] < 0)
			continue;

		length += scnprintf(buf + length, PAGE_SIZE - length, "%.*s%d", i, ",",
					priv->sz.speaker_heartbeat[i]);
	}

	mutex_unlock(&priv->lock);
	return length;
}

static ssize_t codec_crashed_counter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int counts = 0;

	if (IS_ERR_OR_NULL(priv))
		return -EINVAL;

	mutex_lock(&priv->lock);
	counts = scnprintf(buf, PAGE_SIZE, "%d", priv->sz.codec_crashed_counter);
	mutex_unlock(&priv->lock);

	return counts;
}

static ssize_t hwinfo_part_number_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int counts = 0;

	if (IS_ERR_OR_NULL(priv))
		return -EINVAL;

	mutex_lock(&priv->lock);
	counts = scnprintf(buf, PAGE_SIZE, "%s", priv->sz.hwinfo_part_number);
	mutex_unlock(&priv->lock);

	return counts;
}

static ssize_t wdsp_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t mic_broken_degrade_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = NULL;
	int counts = 0;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -EINVAL;

	mutex_lock(&priv->lock);

	counts = scnprintf(buf, PAGE_SIZE, "%d", priv->sz.mic_broken_degrade);

	mutex_unlock(&priv->lock);

	dev_dbg(dev, "%s: %s\n", __func__, buf);

	return counts;
}

static ssize_t ams_cs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv;
	int counts;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -EINVAL;

	mutex_lock(&priv->lock);
	counts = scnprintf(buf, PAGE_SIZE, "%u,%u", priv->sz.ams_count, priv->sz.cs_count);
	mutex_unlock(&priv->lock);

	return counts;
}

static ssize_t ams_rate_read_once_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv;
	int counts;
	uint milli_rate = 0;
	const int scale = 100000;


	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -EINVAL;

	mutex_lock(&priv->lock);

	if (priv->sz.cs_count)
		milli_rate = (priv->sz.ams_count * scale / priv->sz.cs_count);

	if (milli_rate > scale)
		milli_rate = scale;

	counts = scnprintf(buf, PAGE_SIZE, "%u", milli_rate);

	priv->sz.ams_count = 0;
	priv->sz.cs_count = 0;

	mutex_unlock(&priv->lock);
	return counts;
}

static ssize_t cca_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length;

	mutex_lock(&priv->lock);
	length = sysfs_emit_at(buf, 0, "%u %u %u ", priv->sz.cca_active[CCA_SOURCE_VOICE],
			priv->sz.cca_enable[CCA_SOURCE_VOICE], priv->sz.cca_cs[CCA_SOURCE_VOICE]);
	mutex_unlock(&priv->lock);

	return length;
}

static ssize_t cca_count_read_once_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int i, length;

	mutex_lock(&priv->lock);
	length = 0;
	for (i = 0; i < CCA_SOURCE_MAX; i++) {
		length += sysfs_emit_at(buf, length, "%u %u ", priv->sz.cca_active[i],
				priv->sz.cca_enable[i]);
		priv->sz.cca_active[i] = 0;
		priv->sz.cca_enable[i] = 0;
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report PDM silence detect on Recording path
 * Ex: result 0,1,0,0
 *     means PDM index 2 get silence detected
 */
static ssize_t pdm_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv;
	int length, i;

	if (IS_ERR_OR_NULL(dev))
		return -ENODEV;

	priv = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(priv))
		return -ENODEV;

	mutex_lock(&priv->lock);
	if (IS_ERR_OR_NULL(priv->sz.pdm_cb) ||
		IS_ERR_OR_NULL(priv->sz.pdm_priv) || priv->sz.pdm_number == 0) {
		length = -EINVAL;
		goto err;
	}

	length = 0;
	for (i = 0; i < priv->sz.pdm_number; i++)
		length += scnprintf(buf + length, PAGE_SIZE - length, "%.*s%d", i, ",",
				priv->sz.pdm_cb(priv->sz.pdm_priv, i));
err:
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report Waves Effects duration per volume range index.
 *
 * Ex: result 0 2 0 0 0 0 555 0 0 0 0 12345 1 1 0 0 0 0 0 0 0 0 0 12345
 *
 *     means instance= usb, active duration 555 milliseconds with volume range
 *           of [0.4-0.5] and 12345 milliseconds of volume range of [0.9-1.0]
 *           and instance = speaker, active duration 12345 milliseconds with
 *           volume range of [0.9-1.0]
 */
static ssize_t waves_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i, j;

	mutex_lock(&priv->lock);

	length = 0;
	for (i = 0; i < MAX_WAVES_INSTANCE; i++) {
		for(j = 0; j < VOLUME_INDEX_MAX; j++) {
			length += sysfs_emit_at(
					buf, length, "%d ", priv->sz.waves_volume_ms_per_day[i][j]);
			priv->sz.waves_volume_ms_per_day[i][j] = 0;
		}
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report active count of adapted Information such as thermal throttling.
 * Ex: 10 5 2 2 1 0
 *     means features 0 to 5 have count 10, 5, 2, 2, 1 and 0 respectively.
 */
static ssize_t adapted_info_active_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i;

	mutex_lock(&priv->lock);
	length = 0;
	for (i = 0; i < ADAPTED_INFO_FEATURES_MAX; i++) {
		length += sysfs_emit_at(
				buf,
				length,
				"%d ",
				priv->sz.adapted_info_active_count_per_day[i]);
		priv->sz.adapted_info_active_count_per_day[i] = 0;
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report audio PCM average latency for the past 24 hours.
 * Ex: result 153 32
 *   means PCM Type 0 and 1 have average latency 153ms and 32ms respectively.
 */
static ssize_t pcm_latency_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i;
	int32_t avg;

	mutex_lock(&priv->lock);
	length = 0;
	for (i = 0; i < PCM_TYPE_COUNT_MAX; i++) {
		avg = priv->sz.pcm_latency_sum[i] / priv->sz.pcm_latency_count[i];
		length += sysfs_emit_at(buf, length, "%d ", avg);
		priv->sz.pcm_latency_sum[i] = 0;
		priv->sz.pcm_latency_count[i] = 0;
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report audio PCM usage count for the past 24 hours.
 * Ex: result 50 142
 *   means PCM Type 0 and 1 has active count of 50 and 142 times respectively.
 */
static ssize_t pcm_count_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i;

	mutex_lock(&priv->lock);
	length = 0;
	for (i = 0; i < PCM_TYPE_COUNT_MAX; i++) {
		length += sysfs_emit_at(buf, length, "%d ", priv->sz.pcm_active_count[i]);
		priv->sz.pcm_active_count[i] = 0;
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report Adapted Information such as thermal throttling.
 * Ex: 3200 3029 130 3 500 0
 *     means features 0 to 5 has durations 3200, 3029, 130, 3, 500 and 0
 *           milliseconds respectively.
 */
static ssize_t adapted_info_active_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i;

	mutex_lock(&priv->lock);
	length = 0;
	for (i = 0; i < ADAPTED_INFO_FEATURES_MAX; i++) {
		length += sysfs_emit_at(
				buf,
				length,
				"%d ",
				priv->sz.adapted_info_active_duration_ms_per_day[i]);
		priv->sz.adapted_info_active_duration_ms_per_day[i] = 0;
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report BT usage.
 * Ex: result 10 20 30 40 50
 *     means Codec index 0 has duration 10 seconds
 *     and   Codec index 1 has duration 20 seconds
 *     and   Codec index 2 has duration 30 seconds
 *     and   Codec index 3 has duration 40 seconds
 *     and   Codec index 4 has duration 50 seconds
 */
static ssize_t bt_usage_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i;

	mutex_lock(&priv->lock);
	if (IS_ERR_OR_NULL(priv->sz.pdm_cb) ||
		IS_ERR_OR_NULL(priv->sz.pdm_priv) || priv->sz.pdm_number == 0) {
		length = -EINVAL;
		goto err;
	}

	length = 0;
	for (i = 0; i < CODEC_MAX_COUNT; i++) {
		length += sysfs_emit_at(buf, length, "%d ", priv->sz.bt_active_duration[i]);
		priv->sz.bt_active_duration[i] = 0;
	}
err:
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report Voice Info background level duration.
 * It will always report 192 numbers
 * Ex: result 1 1 1 40 ... 2
 *    means voice, receiver, noise level 1, is active 1 second per day.
 *          voice, receiver, noise level 2, is active 1 second per day.
 *          voice, receiver, noise level 3, is active 1 second per day.
 *          voice, receiver, noise level 4, is active 40 seconds per day.
 *          ...z
 *          voip, other, noise level 12, is active 2 seconds per day.
 */
static ssize_t voice_info_noise_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, type, rx, level, duration;

	mutex_lock(&priv->lock);
	length = 0;
	for (type = 0; type < VOICE_TYPE_MAX; type++) {
		for (rx = 0; rx < VOICE_DEVICE_RX_TYPE; rx++) {
			for (level = 0; level < VOICE_NOISE_LEVEL_MAX; level++) {
				duration = priv->sz.voice_noise_duration[type][rx][level];
				length += sysfs_emit_at(buf, length, "%d ", duration);
				priv->sz.voice_noise_duration[type][rx][level] = 0;
			}
		}
	}
	mutex_unlock(&priv->lock);
	return length;
}

void pdm_callback_register(pdm_callback callback, int pdm_total, void* pdm_priv)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(&amcs_pdev->dev);

	mutex_lock(&priv->lock);
	priv->sz.pdm_cb = callback;
	priv->sz.pdm_number = pdm_total;
	priv->sz.pdm_priv = pdm_priv;
	mutex_unlock(&priv->lock);
}
EXPORT_SYMBOL_GPL(pdm_callback_register);

/*
 * Report Offload Effects uuid.
 * Ex: result 1 2 3 4 2 3 4 5
 *
 *     means there are two uuids: 1 2 3 4 and 2 3 4 5
 */
static ssize_t offload_effects_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i, j;

	mutex_lock(&priv->lock);
	length = 0;
	for (i = 0; i < OFFLOAD_EFFECTS_COUNT_MAX; i++) {
		for (j = 0; j < OFFLOAD_EFFECTS_UUID_LENGTH; j++) {
			length += sysfs_emit_at(buf, length, "%d ",
					priv->sz.effect_uuid[i][j]);
			priv->sz.effect_uuid[i][j] = 0;
		}
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report Offload Effects duration.
 * Ex: result 10 20
 *
 *     means there are two offload effects with duration 10 and 20 seconds.
 */
static ssize_t offload_effects_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i;

	mutex_lock(&priv->lock);
	length = 0;
	for (i = 0; i < OFFLOAD_EFFECTS_COUNT_MAX; i++) {
		length += sysfs_emit_at(buf, length, "%d ",
				priv->sz.effect_active_seconds_per_day[i]);
		priv->sz.effect_active_seconds_per_day[i] = 0;
	}
	priv->sz.offload_effects_count = 0;
	mutex_unlock(&priv->lock);
	return length;
}


/*
 * Report audio DSP Record active count in the past day.
 * Ex: result 10 24
 *
 *   means Record type 0 is active count is 10 times
 *     and Record type 1 is active count is 24 times
 */
static ssize_t dsp_record_count_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i;

	length = 0;
	mutex_lock(&priv->lock);
	for (i = 0; i < DSP_RECORD_TYPE_COUNT_MAX; i++) {
		length += sysfs_emit_at(buf, length, "%d ", priv->sz.dsp_usage_count[i]);
		priv->sz.dsp_usage_count[i] = 0;
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report audio DSP Record active duration in the past day.
 * Ex: result 1234 2321
 *
 *   means Record type 0 is 1234 seconds in the past day.
 *     and Record type 1 is 2321 seconds in the past day.
 */
static ssize_t dsp_record_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length, i;

	length = 0;
	mutex_lock(&priv->lock);
	for (i = 0; i < DSP_RECORD_TYPE_COUNT_MAX; i++) {
		length += sysfs_emit_at(buf, length, "%d ", priv->sz.dsp_usage_duration[i]);
		priv->sz.dsp_usage_count[i] = 0;
	}
	mutex_unlock(&priv->lock);
	return length;
}

/*
 * Report call counts including voice-call and VoIP-call.
 * Ex: result 10 20
 *
 *     means there are 10 voice-call and 20 VoIP-call.
 */
static ssize_t call_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length = 0;

	mutex_lock(&priv->lock);
	length = sysfs_emit_at(buf, length, "%d %d", priv->sz.voice_call_count,
			priv->sz.voip_call_count);
	mutex_unlock(&priv->lock);
	priv->sz.voice_call_count = 0;
	priv->sz.voip_call_count = 0;
	return length;
}

/*
 * Report audio software restart count.
 * Ex: result 10 15
 *   means audio hal restarted 10 times
 *     and DSP       restarted 15 times in the past day.
 */
static ssize_t audio_software_restart_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(dev);
	int length = 0;

	mutex_lock(&priv->lock);
	length = sysfs_emit_at(buf, length, "%d %d",
	    priv->sz.hal_restart_count, priv->sz.dsp_restart_count);
	priv->sz.hal_restart_count = 0;
	priv->sz.dsp_restart_count = 0;
	mutex_unlock(&priv->lock);
	return length;
}

static int amcs_cdev_open(struct inode *inode, struct file *file)
{
	struct audiometrics_priv_type *priv = container_of(inode->i_cdev,
					struct audiometrics_priv_type, cdev);

	file->private_data = priv;
	return 0;
}

static int amcs_cdev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long amcs_cdev_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct audiometrics_priv_type *priv = file->private_data;
	long ret = -EINVAL;
	int i = 0;
	struct amcs_params params;
	uint32_t wave_instance, volume_index;
	uint32_t adapted_info_feature;
	uint32_t codec;
	uint32_t pcm_type;
	uint32_t type, rx, level;
	uint32_t record_type;
	uint32_t cca_source;
	bool is_voice_call;
	int index;

	dev_dbg(priv->device, "%s cmd = 0x%x", __func__, cmd);

	if (IS_ERR_OR_NULL(priv)) {
		dev_err(&amcs_pdev->dev, "%s: priv is null or err\n", __func__);
		return ret;
	}

	if (_IOC_TYPE(cmd) != AMCS_IOCTL_MAGIC) {
		dev_err(&amcs_pdev->dev, "%s: cmd 0x%08x is not AMCS IOCTL\n", __func__, cmd);
		return ret;
	}

	if (sizeof(params) != _IOC_SIZE(cmd)) {
		dev_err(&amcs_pdev->dev, "%s: size of cmd 0x%08x is invalid\n", __func__, cmd);
		return ret;
	}

	if (copy_from_user(&params, (struct amcs_params *)arg, _IOC_SIZE(cmd)))
		return ret;

	switch (cmd) {
	case AMCS_IOCTL_METRIC_UPDATE:
		switch (params.op) {

		case AMCS_OP_CODEC_STATUS:
			mutex_lock(&priv->lock);
			priv->sz.codec_state = params.val[0];
			mutex_unlock(&priv->lock);
			ret = 0;
		break;

		case AMCS_OP_CODEC_STATUS_HS:
			mutex_lock(&priv->lock);
			priv->sz.hs_codec_state = params.val[0];
			mutex_unlock(&priv->lock);
			ret = 0;
		break;

		case AMCS_OP_SPEAKER_IMP:
			mutex_lock(&priv->lock);
			for (i = 0; i < SPEAKER_MAX_COUNT && i < sizeof(params.val); i++)
				priv->sz.speaker_impedance[i] = params.val[i];
			mutex_unlock(&priv->lock);

			ret = 0;
		break;

		case AMCS_OP_SPEAKER_TEMP:
			mutex_lock(&priv->lock);
			for (i = 0; i < SPEAKER_MAX_COUNT && i < sizeof(params.val); i++)
				priv->sz.speaker_temp[i] = params.val[i];
			mutex_unlock(&priv->lock);

			ret = 0;
		break;

		case AMCS_OP_SPEAKER_HEART:
			mutex_lock(&priv->lock);
			for (i = 0; i < SPEAKER_MAX_COUNT && i < sizeof(params.val); i++)
				priv->sz.speaker_heartbeat[i] = params.val[i];
			mutex_unlock(&priv->lock);

			ret = 0;
		break;

		case AMCS_OP_SPEAKER_EXCUR:
			mutex_lock(&priv->lock);
			for (i = 0; i < SPEAKER_MAX_COUNT && i < sizeof(params.val); i++)
				priv->sz.speaker_excursion[i] = params.val[i];
			mutex_unlock(&priv->lock);

			ret = 0;
		break;

		case AMCS_OP_MIC_BROKEN_DEGRADE:
			mutex_lock(&priv->lock);
			priv->sz.mic_broken_degrade = (uint32_t)params.val[0];
			mutex_unlock(&priv->lock);

			if(priv->sz.mic_broken_degrade)
				amcs_report_mic_uevent(priv->sz.mic_broken_degrade, priv);
			ret = 0;
		break;

		case AMCS_OP_COUNTER:
			mutex_lock(&priv->lock);
			if (params.val[0] == AMCS_OP2_GET) {
				params.val[1] =	priv->sz.codec_crashed_counter;
				params.val[2] =	priv->sz.hs_codec_crashed_counter;
			} else if (params.val[0] == AMCS_OP2_SET) {
				priv->sz.codec_crashed_counter = params.val[1];
				priv->sz.hs_codec_crashed_counter = params.val[2];
			}
			mutex_unlock(&priv->lock);

			if (!copy_to_user((struct amcs_params *)arg, &params, _IOC_SIZE(cmd)))
				ret = 0;
			else
				ret = -EINVAL;

		break;

		case AMCS_OP_AMS:
			mutex_lock(&priv->lock);
			if (params.val[0] == AMCS_OP2_GET) {
				params.val[1] =	priv->sz.ams_count;
				params.val[2] =	priv->sz.cs_count;
			} else if (params.val[0] == AMCS_OP2_SET) {
				priv->sz.ams_count = params.val[1];
				priv->sz.cs_count = params.val[2];
			}
			mutex_unlock(&priv->lock);

			if (!copy_to_user((struct amcs_params *)arg, &params, _IOC_SIZE(cmd)))
				ret = 0;
			else
				ret = -EINVAL;
		break;

		case AMCS_OP_AMS_INCREASE:
			mutex_lock(&priv->lock);
			if (params.val[0] == AMCS_OP2_SET) {
				priv->sz.ams_count += params.val[1];
				priv->sz.cs_count += params.val[2];
			}
			mutex_unlock(&priv->lock);
			ret = 0;
		break;

		case AMCS_OP_CCA:
			mutex_lock(&priv->lock);
			if (params.val[0] == AMCS_OP2_GET) {
				params.val[1] =	priv->sz.cca_active[CCA_SOURCE_VOICE];
				params.val[2] =	priv->sz.cca_enable[CCA_SOURCE_VOICE];
				params.val[3] =	priv->sz.cca_cs[CCA_SOURCE_VOICE];
			} else if (params.val[0] == AMCS_OP2_SET) {
				priv->sz.cca_active[CCA_SOURCE_VOICE] = params.val[1];
				priv->sz.cca_enable[CCA_SOURCE_VOICE] = params.val[2];
				priv->sz.cca_cs[CCA_SOURCE_VOICE] = params.val[3];
			}
			mutex_unlock(&priv->lock);

			if (!copy_to_user((struct amcs_params *)arg, &params, _IOC_SIZE(cmd)))
				ret = 0;
			else
				ret = -EINVAL;
		break;

		case AMCS_OP_CCA_INCREASE:
			ret = 0;
			if (params.val[0] == AMCS_OP2_SET) {
				cca_source = params.val[4];
				if (cca_source >= CCA_SOURCE_MAX) {
					ret = -EINVAL;
				} else {
					mutex_lock(&priv->lock);
					priv->sz.cca_active[cca_source] += params.val[1];
					priv->sz.cca_enable[cca_source] += params.val[2];
					priv->sz.cca_cs[cca_source] += params.val[3];
					mutex_unlock(&priv->lock);
				}
			}
			break;

		case AMCS_OP_WAVES_VOLUME_INCREASE:
			ret = 0;
			wave_instance = params.val[0];
			volume_index = params.val[1];
			if (wave_instance >= MAX_WAVES_INSTANCE ||
				volume_index >= VOLUME_INDEX_MAX) {
				ret = -EINVAL;
			} else {
				mutex_lock(&priv->lock);
				priv->sz.waves_volume_ms_per_day[wave_instance][volume_index] +=
						params.val[2];
				mutex_unlock(&priv->lock);
			}

		break;

		case AMCS_OP_ADAPTED_INFO_FEATURE:
			ret = 0;
			adapted_info_feature = params.val[0];
			if (adapted_info_feature >= AMCS_OP_ADAPTED_INFO_FEATURE) {
					ret = -EINVAL;
					break;
			}
			mutex_lock(&priv->lock);
			priv->sz.adapted_info_active_count_per_day[adapted_info_feature] +=
					params.val[1];
			priv->sz.adapted_info_active_duration_ms_per_day[adapted_info_feature] +=
					params.val[2];
			mutex_unlock(&priv->lock);
		break;

		case AMCS_OP_BT_ACTIVE_DURATION_INCREASE:
			ret = 0;
			codec = params.val[0];
			if (codec >= CODEC_MAX_COUNT) {
				ret = -EINVAL;
			} else {
				mutex_lock(&priv->lock);
				priv->sz.bt_active_duration[codec] += params.val[1];
				mutex_unlock(&priv->lock);
			}
		break;

		case AMCS_OP_OFFLOAD_EFFECT_DURATION:
			ret = 0;
			mutex_lock(&priv->lock);
			for(i = 0; i < priv->sz.offload_effects_count; i++) {
				if (!memcmp(params.val,
						priv->sz.effect_uuid[i],
						sizeof(priv->sz.effect_uuid[i]))) {
					priv->sz.effect_active_seconds_per_day[i] += params.val[4];
					mutex_unlock(&priv->lock);
					return 0;
				}
			}

			index = priv->sz.offload_effects_count;
			if (index >= OFFLOAD_EFFECTS_COUNT_MAX) {
				ret = -EINVAL;
			} else {
				memcpy(priv->sz.effect_uuid[index],
						params.val, sizeof(priv->sz.effect_uuid[i]));
				priv->sz.effect_active_seconds_per_day[index] +=
						params.val[4];
				priv->sz.offload_effects_count++;
			}
			mutex_unlock(&priv->lock);
			break;

		case AMCS_OP_ADD_PCM_LATENCY:
		ret = 0;
		pcm_type = params.val[0];
		if (pcm_type >= PCM_TYPE_COUNT_MAX) {
			ret = -EINVAL;
		} else {
			mutex_lock(&priv->lock);
			priv->sz.pcm_latency_sum[pcm_type] += params.val[1];
			priv->sz.pcm_latency_count[pcm_type]++;
			mutex_unlock(&priv->lock);
		}
		break;

		case AMCS_OP_PCM_ACTIVE_COUNT_INCREASE:
		ret = 0;
		pcm_type = params.val[0];
		if (pcm_type >= PCM_TYPE_COUNT_MAX) {
			ret = -EINVAL;
		} else {
			mutex_lock(&priv->lock);
			priv->sz.pcm_active_count[pcm_type] += params.val[1];
			mutex_unlock(&priv->lock);
		}
		break;

		case AMCS_OP_VOICE_INFO_NOISE_LEVEL:
			ret = 0;
			type = params.val[0];
			rx = params.val[1];
			level = params.val[2];
			if (type >= VOICE_TYPE_MAX ||
					rx >= VOICE_DEVICE_RX_TYPE ||
					level >= VOICE_NOISE_LEVEL_MAX) {
				ret = -EINVAL;
				break;
			}
			mutex_lock(&priv->lock);
			priv->sz.voice_noise_duration[type][rx][level] += params.val[4];
			mutex_unlock(&priv->lock);
		break;

		case AMCS_OP_DSP_RECORD_USAGE_DURATION_INCREASE:
			ret = 0;
			record_type = params.val[0];
			if (record_type >= DSP_RECORD_TYPE_COUNT_MAX) {
				ret = -EINVAL;
			} else {
				mutex_lock(&priv->lock);
				priv->sz.dsp_usage_duration[record_type] += params.val[1];
				mutex_unlock(&priv->lock);
			}
			break;

		case AMCS_OP_DSP_RECORD_USAGE_COUNT_INCREASE:
			ret = 0;
			record_type = params.val[0];
			if (record_type >= DSP_RECORD_TYPE_COUNT_MAX) {
				ret = -EINVAL;
			} else {
				mutex_lock(&priv->lock);
				priv->sz.dsp_usage_count[record_type] += params.val[1];
				mutex_unlock(&priv->lock);
			}
			break;

		case AMCS_OP_CALL_COUNT_INCREASE:
			ret = 0;
			is_voice_call = params.val[0];
			if (is_voice_call) {
				priv->sz.voice_call_count++;
			} else {
				priv->sz.voip_call_count++;
			}
			break;

		case AMCS_OP_SOFTWARE_RESTART_INCREASE:
			ret = 0;
			mutex_lock(&priv->lock);
			priv->sz.hal_restart_count += params.val[0];
			priv->sz.dsp_restart_count += params.val[1];
			mutex_unlock(&priv->lock);
			break;

		default:
		dev_warn(priv->device, "%s, unsupported op = %d\n", __func__,
					params.op);
		ret = -EINVAL;
		break;

		}
	break;

	default:
		dev_err(priv->device, "Received IOCTL with invalid ID (%d) returning ENOTTY", cmd);
		ret = -ENOTTY;
	break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long amcs_cdev_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	if (_IOC_TYPE(cmd) != AMCS_IOCTL_MAGIC) {
		dev_err(&amcs_pdev->dev, "%s: cmd 0x%08x is not AMCS IOCTL\n", __func__, cmd);
		return -ENOTTY;
	}

	return amcs_cdev_unlocked_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define amcs_cdev_compat_ioctl NULL;
#endif

static char *amcs_devnode(struct device *dev, umode_t *mode)
{
	struct audiometrics_priv_type *priv = NULL;

	if (!mode || !dev)
		return NULL;

	priv = dev_get_drvdata(dev);

	if (!priv)
		return NULL;

	if (MAJOR(dev->devt) == priv->amcs_major)
		*mode = 0666;

	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}

static DEVICE_ATTR_RO(codec_state);
static DEVICE_ATTR_RO(hs_codec_state);
static DEVICE_ATTR_RO(speaker_impedance);
static DEVICE_ATTR_RO(speaker_temp);
static DEVICE_ATTR_RO(speaker_excursion);
static DEVICE_ATTR_RO(speaker_heartbeat);
static DEVICE_ATTR_RO(hwinfo_part_number);
static DEVICE_ATTR_RO(wdsp_stat);
static DEVICE_ATTR_RO(mic_broken_degrade);
static DEVICE_ATTR_RO(codec_crashed_counter);
static DEVICE_ATTR_RO(ams_cs);
static DEVICE_ATTR_RO(ams_rate_read_once);
static DEVICE_ATTR_RO(cca);
static DEVICE_ATTR_RO(cca_count_read_once);
static DEVICE_ATTR_RO(pdm_state);
static DEVICE_ATTR_RO(waves);
static DEVICE_ATTR_RO(adapted_info_active_count);
static DEVICE_ATTR_RO(adapted_info_active_duration);
static DEVICE_ATTR_RO(bt_usage);
static DEVICE_ATTR_RO(pcm_latency);
static DEVICE_ATTR_RO(pcm_count);
static DEVICE_ATTR_RO(voice_info_noise_level);
static DEVICE_ATTR_RO(offload_effects_id);
static DEVICE_ATTR_RO(offload_effects_duration);
static DEVICE_ATTR_RO(dsp_record_count);
static DEVICE_ATTR_RO(dsp_record_duration);
static DEVICE_ATTR_RO(call_count);
static DEVICE_ATTR_RO(audio_software_restart_count);


static struct attribute *audiometrics_fs_attrs[] = {
	&dev_attr_codec_state.attr,
	&dev_attr_hs_codec_state.attr,
	&dev_attr_speaker_impedance.attr,
	&dev_attr_speaker_temp.attr,
	&dev_attr_speaker_excursion.attr,
	&dev_attr_speaker_heartbeat.attr,
	&dev_attr_hwinfo_part_number.attr,
	&dev_attr_wdsp_stat.attr,
	&dev_attr_mic_broken_degrade.attr,
	&dev_attr_codec_crashed_counter.attr,
	&dev_attr_ams_cs.attr,
	&dev_attr_ams_rate_read_once.attr,
	&dev_attr_cca.attr,
	&dev_attr_cca_count_read_once.attr,
	&dev_attr_pdm_state.attr,
	&dev_attr_waves.attr,
	&dev_attr_adapted_info_active_count.attr,
	&dev_attr_adapted_info_active_duration.attr,
	&dev_attr_bt_usage.attr,
	&dev_attr_pcm_latency.attr,
	&dev_attr_pcm_count.attr,
	&dev_attr_voice_info_noise_level.attr,
	&dev_attr_offload_effects_id.attr,
	&dev_attr_offload_effects_duration.attr,
	&dev_attr_dsp_record_count.attr,
	&dev_attr_dsp_record_duration.attr,
	&dev_attr_call_count.attr,
	&dev_attr_audio_software_restart_count.attr,
	NULL,
};

static struct attribute_group audiometrics_fs_attr_group = {
	.attrs = audiometrics_fs_attrs,
};

static const struct file_operations amcs_fops = {
	.open = amcs_cdev_open,
	.release = amcs_cdev_release,
	.unlocked_ioctl = amcs_cdev_unlocked_ioctl,
	.compat_ioctl = amcs_cdev_compat_ioctl,

	.owner = THIS_MODULE,
};

static void init_hwinfo_revision(struct audiometrics_priv_type *priv)
{
	mutex_lock(&priv->lock);
	/* ToDo: If there are revision differences, we should invoke aoc api to append this. */
	scnprintf(priv->sz.hwinfo_part_number, AUDIOMETRIC_CH_LENGTH, "%s", "AOC");
	mutex_unlock(&priv->lock);
}

static void init_suez_speaker_default(struct audiometrics_priv_type *priv)
{
	int i;
	mutex_lock(&priv->lock);
	for (i = 0; i < SPEAKER_MAX_COUNT && i < 2; i++) {
		priv->sz.speaker_impedance[i] = 0;
		priv->sz.speaker_excursion[i] = 0;
		priv->sz.speaker_temp[i] = 0;
		priv->sz.speaker_heartbeat[i] = 0;
	}

	for (i = 2; i < SPEAKER_MAX_COUNT; i++) {
		priv->sz.speaker_impedance[i] = -1;
		priv->sz.speaker_excursion[i] = -1;
		priv->sz.speaker_temp[i] = -1;
		priv->sz.speaker_heartbeat[i] = -1;
	}
	mutex_unlock(&priv->lock);
}


static int amcs_init_cdev(struct audiometrics_priv_type *priv)
{
	int ret;

	ret = alloc_chrdev_region(&priv->amcs_dev, 0, AMCS_MAX_MINOR, AMCS_CDEV_NAME);
	if (ret != 0) {
		dev_err(&amcs_pdev->dev, "Failed to alloc chrdev region\n");
		goto err_init_cdev;
	}

	cdev_init(&priv->cdev, &amcs_fops);
	priv->cdev.owner = THIS_MODULE;

	ret = cdev_add(&priv->cdev, priv->amcs_dev, AMCS_MAX_MINOR);
	if (ret) {
		dev_err(&amcs_pdev->dev, "Failed to register chrdev\n");
		goto err_cdev_add;
	}

	priv->amcs_major = MAJOR(priv->amcs_dev);

	priv->class = class_create(THIS_MODULE, AMCS_CDEV_NAME);
	if (!priv->class) {
		dev_err(&amcs_pdev->dev, "Failed to create amcs class\n");
		ret = -ENXIO;
		goto err_class_create;
	}

	priv->class->devnode = amcs_devnode;

	priv->device = device_create(priv->class, NULL,
					 MKDEV(priv->amcs_major, 0),
					 priv, AMCS_CDEV_NAME);
	if (!priv->device) {
		dev_err(&amcs_pdev->dev, "Failed to create amcs device\n");
		ret = -ENXIO;
		goto err_device_create;
	}

	dev_dbg(&amcs_pdev->dev, "cdev registered\n");

	return ret;

err_device_create:
	class_destroy(priv->class);
err_class_create:
	cdev_del(&priv->cdev);
err_cdev_add:
	unregister_chrdev_region(priv->amcs_dev, 1);
err_init_cdev:

	return ret;
}

static void amcs_deinit_cdev(struct audiometrics_priv_type *priv)
{
	if (!priv)
		return;

	device_destroy(priv->class, priv->amcs_dev);
	class_destroy(priv->class);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->amcs_dev, AMCS_MAX_MINOR);
}


static int audiometrics_platform_probe(struct platform_device *pdev)
{
	int err;
	struct audiometrics_priv_type *priv = NULL;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);

	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, priv);

	mutex_init(&priv->lock);

	/* create cdev to export ioctl to ahal & metric service. */
	err = amcs_init_cdev(priv);
	if (err) {
		dev_err(&pdev->dev, "Failed to initialize cdev: %d\n", err);
		goto err_amcs_init_cdev;
	}

	/* create audio sysfs for suez */
	err = sysfs_create_group(&pdev->dev.kobj, &audiometrics_fs_attr_group);
	if (err) {
		dev_err(&pdev->dev, "Failed to initialize fs attrs: %d\n", err);
		goto audiometrics_fs_attr_group_err;
	}

	init_hwinfo_revision(priv);
	init_suez_speaker_default(priv);

	dev_dbg(&pdev->dev, "%s registered\n", __func__);
	return 0;

audiometrics_fs_attr_group_err:
	dev_err(&pdev->dev, "%s create sysfs failed, err=%d\n", __func__, err);
err_amcs_init_cdev:
	mutex_destroy(&priv->lock);
	amcs_deinit_cdev(priv);
	devm_kfree(&pdev->dev, priv);
	return err;
}

static int audiometrics_platform_remove(struct platform_device *pdev)
{
	struct audiometrics_priv_type *priv = dev_get_drvdata(&pdev->dev);

	amcs_deinit_cdev(priv);
	sysfs_remove_group(&pdev->dev.kobj, &audiometrics_fs_attr_group);
	mutex_destroy(&priv->lock);
	devm_kfree(&pdev->dev, priv);
	return 0;
}

struct platform_driver audiometrics_driver = {
	.probe = audiometrics_platform_probe,
	.remove = audiometrics_platform_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	}
};

static int amcs_init(void)
{
	int ret;

	amcs_pdev = platform_device_register_simple(DRIVER_NAME, -1, NULL, 0);

	if (IS_ERR(amcs_pdev))
		return PTR_ERR(amcs_pdev);

	ret = platform_driver_register(&audiometrics_driver);
	if (ret != 0)
		platform_device_unregister(amcs_pdev);

	return ret;
}

static void amcs_exit(void)
{
	platform_driver_unregister(&audiometrics_driver);
	platform_device_unregister(amcs_pdev);
}

module_init(amcs_init);
module_exit(amcs_exit);

MODULE_DESCRIPTION("Google AudioMetrics driver");
MODULE_LICENSE("GPL");
