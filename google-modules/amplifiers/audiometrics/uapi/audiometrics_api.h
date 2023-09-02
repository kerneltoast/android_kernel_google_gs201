/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Google Whitechapel Audio Metrics Driver User Interface
 *
 * Copyright (c) 2021 Google LLC
 *
 */
#ifndef _AUDIOMETRICS_API_H
#define _AUDIOMETRICS_API_H

#include <linux/ioctl.h>
#include <linux/types.h>
#define AMCS_IOCTL_MAGIC 0xAD

struct amcs_ion_handle {
	uint32_t handle;
	int32_t fd;
};

#define AMCS_PARAMS_LENGTH_MAX 8
struct amcs_params {
	uint32_t op;
	uint32_t ret;
	int32_t val[AMCS_PARAMS_LENGTH_MAX];
};

enum amcs_params_op2_counter_cmd {
	AMCS_OP2_GET,
	AMCS_OP2_SET,
	AMCS_OP2_PARAMS_MAX,
};

enum amcs_params_op {
	AMCS_OP_CODEC_STATUS,
	AMCS_OP_CODEC_STATUS_HS,
	AMCS_OP_SPEAKER_IMP,
	AMCS_OP_SPEAKER_TEMP,
	AMCS_OP_SPEAKER_EXCUR,
	AMCS_OP_SPEAKER_HEART,
	AMCS_OP_WDSP_STAT,
	AMCS_OP_HW_PN,
	AMCS_OP_MIC_BROKEN_DEGRADE,
	AMCS_OP_COUNTER,
	AMCS_OP_AMS,
	AMCS_OP_AMS_INCREASE,
	AMCS_OP_CCA,
	AMCS_OP_CCA_INCREASE,
	AMCS_OP_PARAMS_MAX,
};

#define AMCS_IOCTL_METRIC_UPDATE _IOWR(AMCS_IOCTL_MAGIC, 0xD0, struct amcs_params)
#define AMCS_IOCTL_METRIC_GET_ION_FD _IOWR(AMCS_IOCTL_MAGIC, 0xD1, struct amcs_ion_handle)

#endif /* _AUDIOMETRICS_API_H */
