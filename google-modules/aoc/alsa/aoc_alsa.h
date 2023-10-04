/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google Whitechapel AoC ALSA Driver
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AOC_ALSA_H
#define AOC_ALSA_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <linux/version.h>

#include <sound/compress_params.h>
#include <sound/compress_offload.h>
#include <sound/compress_driver.h>

#include "../aoc-interface.h"
#include "google-aoc-enum.h"

#define ALSA_AOC_CMD "alsa-aoc"
#define CMD_INPUT_CHANNEL "audio_input_control"
#define CMD_OUTPUT_CHANNEL "audio_output_control"
#define CMD_CHANNEL(_dev)                                                      \
	(strcmp(dev_name(&(_dev)->dev), CMD_INPUT_CHANNEL)) ? "output" : "input"

#define AOC_MMAP_PLAYBACK_SERVICE "audio_playback0"
#define AOC_MMAP_CAPTURE_SERVICE "audio_capture1"
#define AOC_COMPR_OFFLOAD_SERVICE "audio_playback6"
#define AOC_COMPR_OFFLOAD_EOF_SERVICE "decoder_eof"

enum uc_device_id {
	UC_AUDIO_RECORD = 8,
	UC_MMAP_RECORD = 9,
	UC_LOW_LATENCY_AUDIO_RECORD = 10,
	UC_ULTRASONIC_RECORD = 12
};

#define AOC_AUDIO_CAPUTRE_DEVICE_MASK                                                              \
	(1 << UC_AUDIO_RECORD | 1 << UC_MMAP_RECORD | 1 << UC_LOW_LATENCY_AUDIO_RECORD)

#define AOC_ULTRASONIC_CAPUTRE_DEVICE_MASK (1 << UC_ULTRASONIC_RECORD)

#define AOC_CAPUTRE_DEVICE_MASK (AOC_AUDIO_CAPUTRE_DEVICE_MASK | AOC_ULTRASONIC_CAPUTRE_DEVICE_MASK)

#define AOC_CMD_DEBUG_ENABLE
#define WAITING_TIME_MS 500

#define PCM_TIMER_INTERVAL_NANOSECS 10e6
#define COMPR_OFFLOAD_TIMER_INTERVAL_NANOSECS 5000e6
#define AOC_COMPR_HRTIMER_IRQ_HANDLER_BYPASS
#define DEFAULT_PCM_WAIT_TIME_IN_MSECS 10000
#define DEFAULT_VOICE_PCM_WAIT_TIME_IN_MSECS 500
#define COMPR_OFFLOAD_GAIN_RESET_TIME_DELAY_IN_MSECS 150
#define COMPR_INVALID_METADATA (-1)

/* Default mic and sink for audio capturing/playback */
#define DEFAULT_MICPHONE_ID 0
#define NUM_OF_BUILTIN_MIC 4
#define DEFAULT_AUDIO_SINK_ID 0
#define MAX_NUM_OF_SINKS_PER_STREAM 2

#define MAX_NUM_OF_INCALL_CAPTURE_STREAM 3

#define N_MIC_IN_SPATIAL_MODULE 3

/* TODO: the exact number has to be determined based on hardware platform*/
#define MAX_NUM_OF_SUBSTREAMS 32
#define MAX_NUM_OF_SINKS 5
#define AVAIL_SUBSTREAMS_MASK 0x0fff

#define AOC_AUDIO_SINK_BLOCK_ID_BASE 16
#define AOC_COMPR_OFFLOAD_DEFAULT_SR 48000
#define COMPR_OFFLOAD_KERNEL_TMP_BUF_SIZE PAGE_SIZE

/* TODO: may not needed*/
#define PLAYBACK_WATERMARK_DEFAULT 48000

#define  MIC_HW_GAIN_IN_CB_MIN -720
#define  MIC_HW_GAIN_IN_CB_MAX  240

#define SIDETONE_EQ_STAGE_NUM_MIN 1
#define SIDETONE_EQ_STAGE_NUM_MAX 5
#define SIDETONE_VOL_MIN -96
#define SIDETONE_VOL_MAX 0
#define SIDETONE_MIC_ID_MIN 0
#define SIDETONE_MIC_ID_MAX 3
#define SIDETONE_BIQUAD_PARAM_NUM 6
#define SIDETONE_BIQUAD_PARAM_MIN S32_MIN
#define SIDETONE_BIQUAD_PARAM_MAX S32_MAX

#define FLOAT_ZERO	0x00000000
#define FLOAT_ONE	0x3f800000

#define alsa2chip(vol) (vol) /* Convert alsa to chip volume */
#define chip2alsa(vol) (vol) /* Convert chip to alsa volume */

#define NULL_PATH -1

/* TODO: Copied from AoC repo and will be removed */
enum bluetooth_mode {
	AHS_BT_MODE_UNCONFIGURED = 0,
	AHS_BT_MODE_SCO,
	AHS_BT_MODE_ESCO,
	AHS_BT_MODE_A2DP_RAW,
	AHS_BT_MODE_A2DP_ENC_SBC,
	AHS_BT_MODE_A2DP_ENC_AAC,
	AHS_BT_MODE_A2DP_ENC_LC3,
	AHS_BT_MODE_BLE_ENC_LC3,
	AHS_BT_MODE_BLE_CONVERSATION,
	AHS_BT_MODE_A2DP_ENC_OPUS,
};

enum TelephonyModes {
	AHS_TELE_MODE_MODEM,
	AHS_TELE_MODE_VOIP_48,
	AHS_TELE_MODE_VOIP_44,
	AHS_TELE_MODE_VOIP_32,
	AHS_TELE_MODE_VOIP_24,
	AHS_TELE_MODE_VOIP_16,
	AHS_TELE_MODE_VOIP_8,
};

/* AoC USB Config parameters */
enum {
	USB_BUS_ID,
	USB_DEV_ID,
	USB_TX_EP_ID,
	USB_TX_FORMAT,
	USB_TX_SR,
	USB_TX_CH,
	USB_TX_BW,
	USB_RX_EP_ID,
	USB_RX_FORMAT,
	USB_RX_SR,
	USB_RX_CH,
	USB_RX_BW,
	USB_CFG_TO_AOC
};

/* AoC sidetone EQ */
enum { BIQUAD0 = 0, BIQUAD1, BIQUAD2, BIQUAD3, BIQUAD4, SIDETONE_EQ_BIQUAD_NUM };
enum { SIDETONE_CFG_VOL, SIDETONE_CFG_STAGE_NUM, SIDETONE_CFG_MIC_ID };

enum { CTRL_VOL_MUTE, CTRL_VOL_UNMUTE };
enum {
	PCM_PLAYBACK_VOLUME,
	PCM_PLAYBACK_MUTE,
	BUILDIN_MIC_POWER_STATE,
	BUILDIN_MIC_CAPTURE_LIST,
	BUILDIN_US_MIC_CAPTURE_LIST,
	A2DP_ENCODER_PARAMETERS,
};

enum aoc_playback_entry_point {
	ULL = 0,
	LL0,
	LL1,
	LL2,
	LL3,
	DEEP_BUFFER,
	OFF_LOAD,
	HAPTICS = 10,
	SIDETONE = 11,
	USB_HIFI = 13,
	SPEAKER_US = 14,
	IMMERSIVE = 15,
};

enum { NORMAL = 0, MMAPED, RAW, INCALL, HIFI, ANDROID_AEC, COMPRESS, CAP_INJ };

enum { BUILTIN_MIC0 = 0, BUILTIN_MIC1, BUILTIN_MIC2, BUILTIN_MIC3 };
enum { MIC_LOW_POWER_GAIN = 0, MIC_HIGH_POWER_GAIN, MIC_CURRENT_GAIN };
enum { DEFAULT_MIC = 0, BUILTIN_MIC, USB_MIC, BT_MIC, IN_CALL_MUSIC, NO_MIC=IN_CALL_MUSIC, ERASER };
enum aec_ref_source {
	DEFAULT_PLAYBACK = 0,
	SPEAKER_PLAYBACK,
	USB_PLAYBACK,
	BT_PLAYBACK,
	NUM_AEC_REF_SOURCE
};
enum { INCALL_CAPTURE_OFF = 0, INCALL_CAPTURE_UL, INCALL_CAPTURE_DL, INCALL_CAPTURE_UL_DL };
enum { NONBLOCKING = 0, BLOCKING = 1 };
enum { STOP = 0, START };
enum { PLAYBACK_MODE, VOICE_TX_MODE, VOICE_RX_MODE, HAPTICS_MODE, OFFLOAD_MODE };

struct aoc_chip {
	struct snd_card *card;
	struct snd_soc_jack jack; /* TODO: temporary use, need refactor  */

	uint32_t avail_substreams;
	struct aoc_alsa_stream *alsa_stream[MAX_NUM_OF_SUBSTREAMS];

	struct aoc_service_dev *dev_alsa_stream[MAX_NUM_OF_SUBSTREAMS];

	int default_mic_id;
	int buildin_mic_id_list[NUM_OF_BUILTIN_MIC];
	int buildin_us_mic_id_list[NUM_OF_BUILTIN_MIC];

	int default_sink_id;
	int sink_id_list[MAX_NUM_OF_SINKS_PER_STREAM];
	int sink_mode[AUDIO_OUTPUT_SINKS];

	int volume;
	int old_volume; /* Store the volume value while muted */
	int mute;
	int audio_capture_mic_source;
	int voice_call_mic_source;
	enum aec_ref_source ft_aec_ref_source;
	enum aec_ref_source eraser_aec_ref_source;
	int voice_call_mic_mute;
	int default_mic_hw_gain;
	int voice_call_audio_enable;
	int incall_capture_state[MAX_NUM_OF_INCALL_CAPTURE_STREAM];

	int telephony_curr_mic;
	int telephony_curr_sink;
	int telephony_expect_mic;
	int telephony_expect_sink;
	bool voip_rx_prepared;
	bool voip_tx_prepared;
	bool voip_path_vote[PORT_MAX];
	bool voice_path_vote[PORT_MAX];
	struct wakeup_source *wakelock;

	int compr_offload_volume;
	int mic_spatial_module_enable;
	int capture_eraser_enable;
	int cca_module_loaded;
	int sidetone_enable;
	int mic_loopback_enabled;
	int gapless_offload_enable;
	unsigned int opened;
	unsigned int capture_param_set;
	struct mutex audio_mutex;
	struct mutex audio_cmd_chan_mutex;
	spinlock_t audio_lock;
	long pcm_wait_time_in_ms;
	long voice_pcm_wait_time_in_ms;

	struct AUDIO_OUTPUT_BT_A2DP_ENC_CFG a2dp_encoder_cfg;
	struct CMD_AUDIO_OUTPUT_USB_CONFIG usb_sink_cfg;
	struct CMD_AUDIO_OUTPUT_USB_CONFIG_V2 usb_sink_cfg_v2;
	struct CMD_AUDIO_OUTPUT_GET_SIDETONE sidetone_cfg;
};

struct aoc_alsa_stream {
	struct aoc_chip *chip;
	struct snd_pcm_substream *substream;
	struct snd_compr_stream *cstream; /* compress offload stream */
	int compr_offload_codec;
	int gapless_offload_enable;
	int send_metadata;
	int eof_reach;
	uint32_t compr_padding;
	uint32_t compr_delay;
	uint64_t compr_pcm_io_sample_base;
	int offload_temp_data_buf_size;
	struct timer_list timer; /* For advancing the hw ptr */
	struct hrtimer hr_timer; /* For advancing the hw ptr */
	unsigned long timer_interval_ns;

	struct aoc_service_dev *dev;
	struct aoc_service_dev *dev_eof; /* Aoc service for EOF in compr offload */
	int idx; /* PCM device number */
	int entry_point_idx; /* Index of entry point, same as idx in playback */
	int stream_type; /* Normal pcm, incall, mmap, hifi, compr */

	int channels; /* Number of channels in audio */
	int params_rate; /* Sampling rate */
	int pcm_format_width; /* Number of bits */
	bool pcm_float_fmt; /* Floating point */

	struct vm_area_struct *vma; /* for MMAP */
	unsigned int period_size;
	unsigned int buffer_size;
	unsigned int pos;
	unsigned long hw_ptr_base; /* read/write pointers in ring buffer */
	unsigned long prev_consumed;
	int n_overflow;
	int open;
	int running;
	int draining;

	struct work_struct free_aoc_service_work;
	struct work_struct pcm_period_work;
};

void aoc_timer_start(struct aoc_alsa_stream *alsa_stream);
void aoc_timer_restart(struct aoc_alsa_stream *alsa_stream);
void aoc_timer_stop(struct aoc_alsa_stream *alsa_stream);
void aoc_timer_stop_sync(struct aoc_alsa_stream *alsa_stream);
void aoc_pcm_period_work_handler(struct work_struct *work);

int snd_aoc_new_ctl(struct aoc_chip *chip);
int snd_aoc_new_pcm(struct aoc_chip *chip);

int aoc_audio_setup(struct aoc_alsa_stream *alsa_stream);
int aoc_audio_open(struct aoc_alsa_stream *alsa_stream);
int aoc_audio_close(struct aoc_alsa_stream *alsa_stream);
int aoc_audio_set_params(struct aoc_alsa_stream *alsa_stream, uint32_t channels,
			 uint32_t samplerate, uint32_t bps, bool pcm_float_fmt, int source_mode);

int aoc_audio_start(struct aoc_alsa_stream *alsa_stream);
int aoc_audio_stop(struct aoc_alsa_stream *alsa_stream);
int aoc_audio_incall_start(struct aoc_alsa_stream *alsa_stream);
int aoc_audio_incall_stop(struct aoc_alsa_stream *alsa_stream);
int aoc_audio_voip_start(struct aoc_alsa_stream *alsa_stream);
int aoc_audio_voip_stop(struct aoc_alsa_stream *alsa_stream);

int aoc_pcm_device_to_stream_type(int device);

int aoc_audio_path_open(struct aoc_chip *chip, int src, int dest, bool be_on);
int aoc_audio_path_close(struct aoc_chip *chip, int src, int dest, bool be_on);
int aoc_phonecall_path_open(struct aoc_chip *chip, int src, int dst, bool capture);
int aoc_phonecall_path_close(struct aoc_chip *chip, int src, int dst, bool capture);
int aoc_voipcall_path_open(struct aoc_chip *chip, int src, int dst, bool capture);
int aoc_voipcall_path_close(struct aoc_chip *chip, int src, int dst, bool capture);

int aoc_audio_set_ctls(struct aoc_chip *chip);

int aoc_a2dp_get_enc_param_size(void);
int aoc_a2dp_set_enc_param(struct aoc_chip *chip, struct AUDIO_OUTPUT_BT_A2DP_ENC_CFG *cfg);

int aoc_set_builtin_mic_power_state(struct aoc_chip *chip, int iMic, int state);
int aoc_get_builtin_mic_power_state(struct aoc_chip *chip, int iMic);
int aoc_mic_clock_rate_get(struct aoc_chip *chip);
int aoc_mic_hw_gain_get(struct aoc_chip *chip, int state);
int aoc_mic_hw_gain_set(struct aoc_chip *chip, int state, int gain);
int aoc_mic_dc_blocker_get(struct aoc_chip *chip);
int aoc_mic_dc_blocker_set(struct aoc_chip *chip, int enable);

int aoc_mic_record_gain_get(struct aoc_chip *chip, long *val);
int aoc_mic_record_gain_set(struct aoc_chip *chip, long val);
int aoc_mmap_record_gain_get(struct aoc_chip *chip, long *val);
int aoc_mmap_record_gain_set(struct aoc_chip *chip, long val);
int aoc_audio_capture_mic_prepare(struct aoc_chip *chip);
int aoc_audio_capture_mic_close(struct aoc_chip *chip);
int aoc_audio_capture_active_stream_num(struct aoc_chip *chip);
int ap_data_control_trigger(struct aoc_chip *chip, struct aoc_alsa_stream *alsa_stream,
			    int record_cmd);
int ap_record_stop(struct aoc_chip *chip, struct aoc_alsa_stream *alsa_stream);
int aoc_capture_filter_runtime_control(struct aoc_chip *chip, uint32_t port_id, bool on);
int aoc_audio_capture_runtime_trigger(struct aoc_chip *chip, int ep_id, int dst, bool on);
int aoc_audio_capture_eraser_enable(struct aoc_chip *chip, long enable);
int aoc_eraser_aec_reference_set(struct aoc_chip *chip, long ref_source);

int aoc_load_cca_module(struct aoc_chip *chip, long load);

int aoc_voice_call_mic_mute(struct aoc_chip *chip, int mute);
int aoc_incall_capture_enable_get(struct aoc_chip *chip, int stream, long *val);
int aoc_incall_capture_enable_set(struct aoc_chip *chip, int stream, long val);
int aoc_incall_playback_enable_get(struct aoc_chip *chip, int stream, long *val);
int aoc_incall_playback_enable_set(struct aoc_chip *chip, int stream, long val);
int aoc_incall_playback_mic_channel_get(struct aoc_chip *chip, int stream, long *val);
int aoc_incall_playback_mic_channel_set(struct aoc_chip *chip, int stream, long val);
int aoc_incall_mic_sink_mute_get(struct aoc_chip *chip, int param, long *mute);
int aoc_incall_mic_sink_mute_set(struct aoc_chip *chip, int param, long mute);

int aoc_lvm_enable_get(struct aoc_chip *chip, long *enable);
int aoc_lvm_enable_set(struct aoc_chip *chip, long enable);
int aoc_decoder_ref_enable_get(struct aoc_chip *chip, long*enable);
int aoc_decoder_ref_enable_set(struct aoc_chip *chip, long enable);


int aoc_sidetone_enable(struct aoc_chip *chip, int enable);
int aoc_sidetone_cfg_get(struct aoc_chip *chip, int param, long *val);
int aoc_sidetone_cfg_set(struct aoc_chip *chip, int param, long val);
int aoc_sidetone_eq_get(struct aoc_chip *chip, int biquad_idx, long *val);
int aoc_sidetone_eq_set(struct aoc_chip *chip, int biquad_idx, long *val);

int aoc_get_dsp_state(struct aoc_chip *chip);
int aoc_get_asp_mode(struct aoc_chip *chip, int block, int component, int key);
int aoc_set_asp_mode(struct aoc_chip *chip, int block, int component, int key, int val);

int aoc_get_audio_dsp_mode(struct aoc_chip *chip, long *val);
int aoc_set_audio_dsp_mode(struct aoc_chip *chip, long val);

int aoc_get_builtin_mic_process_mode(struct aoc_chip *chip);
int aoc_set_builtin_mic_process_mode(struct aoc_chip *chip, long mode);

int aoc_spatial_module_start(struct aoc_chip *chip);
int aoc_spatial_module_stop(struct aoc_chip *chip);

int aoc_get_sink_state(struct aoc_chip *chip, int sink);
int aoc_get_sink_channel_bitmap(struct aoc_chip *chip, int sink);
int aoc_get_sink_mode(struct aoc_chip *chip, int sink);
int aoc_set_sink_mode(struct aoc_chip *chip, int sink, int mode);

int aoc_set_usb_config(struct aoc_chip *chip);
int aoc_set_usb_config_v2(struct aoc_chip *chip);

int aoc_audio_write(struct aoc_alsa_stream *alsa_stream, void *src,
		    uint32_t count);
int aoc_audio_read(struct aoc_alsa_stream *alsa_stream, void *dest,
		   uint32_t count);
int aoc_audio_volume_set(struct aoc_chip *chip, uint32_t volume,
			 int src, int dst);

int prepare_phonecall(struct aoc_alsa_stream *alsa_stream);
int teardown_phonecall(struct aoc_alsa_stream *alsa_stream);

int prepare_voipcall(struct aoc_alsa_stream *alsa_stream);
int teardown_voipcall(struct aoc_alsa_stream *alsa_stream);

void aoc_compr_offload_isr(struct aoc_service_dev *dev);
int aoc_compr_offload_setup(struct aoc_alsa_stream *alsa_stream, int type);
int aoc_compr_offload_send_metadata(struct aoc_alsa_stream *alsa_stream);
int aoc_compr_offload_partial_drain(struct aoc_alsa_stream *alsa_stream);
int aoc_compr_offload_close(struct aoc_alsa_stream *alsa_stream);
int aoc_compr_offload_get_io_samples(struct aoc_alsa_stream *alsa_stream, uint64_t *sample);
int aoc_compr_offload_flush_buffer(struct aoc_alsa_stream *alsa_stream);
int aoc_compr_pause(struct aoc_alsa_stream *alsa_stream);
int aoc_compr_resume(struct aoc_alsa_stream *alsa_stream);
int aoc_compr_offload_linear_gain_get(struct aoc_chip *chip, long *val);
int aoc_compr_offload_linear_gain_set(struct aoc_chip *chip, long *val);

int aoc_mic_loopback(struct aoc_chip *chip, int enable);

int aoc_pcm_init(void);
void aoc_pcm_exit(void);
int aoc_voice_init(void);
void aoc_voice_exit(void);
int aoc_compr_init(void);
void aoc_compr_exit(void);
int aoc_path_init(void);
void aoc_path_exit(void);
int aoc_nohost_init(void);
void aoc_nohost_exit(void);
int aoc_incall_init(void);
void aoc_incall_exit(void);
int aoc_voip_init(void);
void aoc_voip_exit(void);

int aoc_audio_us_record(struct aoc_chip *chip, bool enable);
#endif
