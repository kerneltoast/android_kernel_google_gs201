/*
 ** ============================================================================
 ** Copyright (c) 2016  Texas Instruments Inc.
 **
 ** This program is free software; you can redistribute it and/or modify it
 ** under
 ** the terms of the GNU General Public License as published by the Free
 ** Software Foundation; version 2.
 **
 ** This program is distributed in the hope that it will be useful, but WITHOUT
 ** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 ** FITNESS
 ** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 ** details.
 **
 ** File:
 **     tas25xx-calib-validataion.h
 **
 ** Description:
 **     header file for calibration and validataion interface to SmartPA
 **
 ** ============================================================================
 */

#ifndef _TAS25XX_ALGO_H
#define _TAS25XX_ALGO_H

#ifndef MAX_CHANNELS
#define MAX_CHANNELS 2
#endif

struct big_data {
	uint32_t exc_max;
	uint32_t exc_max_persist;
	uint32_t exc_over_count;
	uint32_t temp_max;
	uint32_t temp_max_persist;
	uint32_t temp_over_count;
};

struct le_flag_detection_info {
	u8 le_flag_detected;
	u32 le_flag_count;
	u64 le_flag_count_persist;
};

struct tas25xx_algo {
	struct class *algo_class;
	struct device *calib_dev;
	struct device *valid_dev;
	struct device *bd_dev;
	struct big_data b_data[MAX_CHANNELS];
	struct le_flag_detection_info b_le_flag[MAX_CHANNELS];
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

/*
 * Q31 to (decimal*100) number conversion
 * can be used for excursion
 */
int32_t tisa_get_q31_to_user(int32_t q31num);

/*
 * Q19 to (decimal*100) number conversion
 * can be used for resistance
 */
int32_t tisa_get_q19_to_user(int32_t q19num);

/* helper functions for skin protection */
void tas_set_algo_run_status(int enable);
int tas_get_coil_temp_nocheck(int id);
int tas_set_surface_temp_nocheck(int id, int temperature);

#endif /* _TAS25XX_ALGO_H */
