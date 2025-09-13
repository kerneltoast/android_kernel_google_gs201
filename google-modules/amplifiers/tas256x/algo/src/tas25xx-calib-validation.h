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
**     tas25xx-calib-validataion.h
**
** Description:
**     header file for calibration and validataion interface to SmartPA
**
** =============================================================================
*/

#ifndef _TAS25XX_ALGO_H
#define _TAS25XX_ALGO_H

#define MAX_STRING		200
#define CALIB_RETRY_COUNT		5
#define RDC_L					0
#define TEMP_L					1
#define RDC_R					2
#define TEMP_R					3
#define DEFAULT_AMBIENT_TEMP	25
#define CALIB_TIME				2
#define VALIDATION_TIME			3
#define STATUS_NONE				0x00
#define STATUS_SUCCESS			0x01
#define STATUS_FAIL				0xcc

#define TAS_SA_GET_RE_LOW		398
#define TAS_SA_GET_RE_HIGH		399
#define VALIDATION_SUCCESS	0xC00DC00D

#define TRANSF_USER_TO_IMPED(X, Y) \
		((X << 19) + ((Y << 19) / 100))
#define QFORMAT19				19
#define QFORMAT31				31

#define TAS25XX_SYSFS_CLASS_NAME	"tas25xx"
#define TAS25XX_CALIB_DIR_NAME		"calib"
#define TAS25XX_VALID_DIR_NAME		"valid"
#define TAS25XX_BD_DIR_NAME			"bigdata"

#define TAS25XX_EFS_CALIB_DATA_L	"/efs/tas25xx/calib_re"
#define TAS25XX_EFS_TEMP_DATA_L		"/efs/tas25xx/amb_temp"
#define TAS25XX_EFS_CALIB_DATA_R	"/efs/tas25xx/calib_re_r"
#define TAS25XX_EFS_TEMP_DATA_R		"/efs/tas25xx/amb_temp_r"

struct big_data {
	uint32_t exc_max;
	uint32_t exc_max_persist;
	uint32_t exc_over_count;
	uint32_t temp_max;
	uint32_t temp_max_persist;
	uint32_t temp_over_count;
};

struct tas25xx_algo {
	struct class *algo_class;
	struct device *calib_dev;
	struct device *valid_dev;
	struct device *bd_dev;
	struct big_data b_data[MAX_CHANNELS];
	struct delayed_work calib_work;
	struct delayed_work valid_work;
	uint8_t spk_count;
	uint32_t port;
	uint32_t calib_re[MAX_CHANNELS];
	uint32_t amb_temp[MAX_CHANNELS];
	bool calib_update[MAX_CHANNELS];
};

void tas25xx_algo_add_calib_valid_bigdata(uint8_t channels);
void tas25xx_algo_remove_calib_valid_bigdata(void);
struct tas25xx_algo *smartamp_get_sysfs_ptr(void);

#endif /* _TAS25XX_ALGO_H */