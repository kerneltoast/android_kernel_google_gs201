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
 ** details
 **
 ** File:
 **     tas25xx-algo-bin-utils.c
 **
 ** Description:
 **     Algorithm parameter's binary file utility header file.
 **
 ** ============================================================================
 */
#ifndef __BIN_IN_KERNEL_H__
#define __BIN_IN_KERNEL_H__

#include <linux/types.h>

#define SA_BIN_HDR_SIZE             16      /*Bin file header size*/
#define SA_BIN_MDATA_SZ             72
#define DEFAULT_SMARTAMP_PROFILE    0
#define NUM_SPK_TYPES               4
#define CUSTOM_VALUE_INDEX_PTR      19

struct bin_file_data_t {
	int uses_bin_file;
	int parse_done;
	int profile_id;
	int spk_id;
	int bytes_per_prof_per_channel;
	int number_of_profiles;
	int no_of_channels;
	int single_profile_size;
	int *profile_data;
};

int bin_file_parse_init(void);
bool bin_file_parse_status(void);
bool bin_file_get_profile_data(char **buf, int *size);
void bin_file_parse_deinit(void);

void bin_file_set_profile_id(int profile_id);

void bin_file_set_spk_id(int spk_id);

int bin_file_get_profile_id(void);
int bin_file_get_spk_id(void);
bool uses_bin_file(void);
int bin_file_get_per_profile_per_channel_param_count(void);
int *bin_file_get_profile_0_data_pointer(void);

int bin_file_get_number_of_channels(void);

void bin_file_set_device(struct device *dev);

/* Custom index access*/
/* index starts from 0 */
int32_t bin_file_get_custom_value_by_idx(int index_request, int *value);

#endif /*__BIN_IN_KERNEL_H__*/
