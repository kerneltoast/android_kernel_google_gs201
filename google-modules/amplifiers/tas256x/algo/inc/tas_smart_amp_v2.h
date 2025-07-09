/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** File:
**     tas25xx-algo-bin-utils.c
**
** Description:
**     Common algo related header.
**
** =============================================================================
*/
#ifndef __TAS_SMART_AMP__
#define __TAS_SMART_AMP__

#include <linux/types.h>
#include <sound/soc.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/device.h>

#define CONFIG_TAS25XX_ALGO_STEREO
#define CONFIG_SET_RE_IN_KERNEL

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
struct snd_soc_component;
#else
struct snd_soc_codec;
#endif

typedef enum {
	TISA_MOD_RX = 0,
	TISA_MOD_TX = 1,
	TISA_MOD_END = 2
} module_id_t;

#define CAPI_V2_TAS_TX_ENABLE           0x10012D14
#define CAPI_V2_TAS_TX_CFG              0x10012D16
#define CAPI_V2_TAS_RX_ENABLE           0x10012D13
#define CAPI_V2_TAS_RX_CFG              0x10012D15

#define TI_SMARTPA_VENDOR_ID            1234

/* FIXME */
/* update this value at OEM/ODM */
/* #pragma message("########### FIXME ################") */
/* #define SMARTAMP_SPEAKER_CALIBDATA_FILE "/mnt/vendor/persist/audio/tas25xx_calib.bin" */
#define SMARTAMP_SPEAKER_CALIBDATA_FILE "/persist/audio/smartamp_calib.bin"

#define MAX_DSP_PARAM_INDEX     2048
#define MAX_CHANNELS            2

#define TAS_SET_PARAM           0
#define TAS_GET_PARAM           1

#define CHANNEL0        1
#define CHANNEL1        2


#define TAS_DSP_SWAP_IDX       3

#define TAS_SA_GET_F0          3810
#define TAS_SA_GET_Q           3811
#define TAS_SA_GET_TV          3812
#define TAS_SA_GET_RE          3813
#define TAS_SA_CALIB_INIT      3814
#define TAS_SA_CALIB_DEINIT    3815
#define TAS_SA_SET_RE          3816
#define TAS_SA_F0_TEST_INIT    3817
#define TAS_SA_F0_TEST_DEINIT  3818
#define TAS_SA_SET_PROFILE     3819
#define TAS_SA_GET_STATUS      3821
#define TAS_SA_SET_SPKID       3822
#define TAS_SA_SET_TCAL        3823
#define TAS_SA_EXC_TEMP_STAT   3824

#define TAS_SA_IV_VBAT_FMT     3825

#define TAS_SA_VALID_INIT      3831
#define TAS_SA_VALID_DEINIT    3832
#define TAS_SA_GET_VALID_STATUS 3833
#define TAS_SA_SET_BYPASS_MODE  3834
#define TAS_SA_GET_OP_MODE      3835
#define TAS_SA_SET_INTERFACE_MODE 3836
#define TAS_PCM_CHANNEL_MAPPING 3837
#define TAS_SA_GET_RE_RANGE   3838

/*Added for DC Detection*/
#define CAPI_V2_TAS_SA_DC_DETECT 0x40404040

#define CALIB_START             1
#define CALIB_STOP              2
#define TEST_START              3
#define TEST_STOP               4

#define SLAVE1          0x98
#define SLAVE2          0x9A
#define SLAVE3          0x9C
#define SLAVE4          0x9E

#define CHANNEL0        1
#define CHANNEL1        2

#define TAS_SA_IS_SPL_IDX(X)    ((((X) >= 3810) && ((X) < 3899)) ? 1 : 0)
#define TAS_CALC_PARAM_IDX(I, LEN, CH)    ((I) | ((LEN) << 16) | ((CH) << 28))
#define AFE_SA_IS_SPL_IDX(X)    TAS_SA_IS_SPL_IDX(X)


typedef enum {
	IV_SENSE_FORMAT_NO_VBAT = 0,
	IV_SENSE_FORMAT_12_BIT_WITH_8BIT_VBAT = 1,
	IV_SENSE_FORMAT_8_BIT_WITH_8BIT_VBAT = 2,
	IV_SENSE_FORMAT_16_BIT_WITH_8BIT_VBAT = 3,
} ti_smartamp_iv_vbat_format_t;

/*
 * List all the other profiles other than none and calibration.
 */
#define TAS_ALGO_PROFILE_LIST          "MUSIC", "VOICE", "VOIP", "RINGTONE"

void tas25xx_parse_algo_dt(struct device_node *np);
typedef int (*dsp_set_get_f)(u8 *user_data, uint32_t param_id,
		uint8_t get_set, uint32_t length, module_id_t module_id);

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
void tas_smartamp_add_algo_controls(struct snd_soc_component *codec,
		struct device *dev, int number_of_channels);
void tas_smartamp_remove_algo_controls(struct snd_soc_component *codec);
void tas_smartamp_add_codec_mixer_controls(struct snd_soc_component *codec);
#else
void tas_smartamp_add_algo_controls(struct snd_soc_codec *codec,
		struct device *dev, int number_of_channels);
void tas_smartamp_remove_algo_controls(struct snd_soc_codec *codec);
void tas_smartamp_add_codec_mixer_controls(struct snd_soc_codec *codec);
#endif
bool tas25xx_set_iv_bit_fomat(int iv_data_with, int vbat, int update_now);

int get_iv_vbat_format(void);

void tas25xx_send_channel_mapping(void);

#if IS_ENABLED(CONFIG_TISA_KBIN_INTF)
void tas25xx_algo_set_active(void);
void tas25xx_algo_set_inactive(void);
void tas_smartamp_kbin_deinitalize(void);
#endif /* CONFIG_TISA_KBIN_INTF */

#if IS_ENABLED(CONFIG_TAS25XX_CALIB_VAL_BIG)
void tas25xx_send_algo_calibration(void);
void tas25xx_update_big_data(void);
#endif

void tas25xx_algo_set_device (struct device *dev);
struct device *tas25xx_algo_get_device(void);

int tas_calib_init(void);
void tas_calib_exit(void);

#endif /*__TAS_SMART_AMP__*/
