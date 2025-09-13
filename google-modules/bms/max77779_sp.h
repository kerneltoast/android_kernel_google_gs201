/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#ifndef MAX77779_SP_H_
#define MAX77779_SP_H_

#define RSBM_ADDR				0
#define RSBR_ADDR				4
#define SUFG_ADDR				8
#define RSOC_ADDR				10
#define FWHI_ADDR				12
#define FWSF_ADDR				16
#define MDLV_ADDR				20
#define RS_TAG_LENGTH				4
#define SU_TAG_LENGTH				1
#define RSOC_TAG_LENGTH				2
#define RS_TAG_OFFSET_ADDR			0
#define RS_TAG_OFFSET_LENGTH			1
#define RS_TAG_OFFSET_DATA			2
#define OPCODE_USER_SPACE_R_RES_LEN 32
#define FWHI_TAG_LENGTH				4
#define FWSF_TAG_LENGTH				4
#define MDLV_TAG_LENGTH				1


#define MAX77779_SP_DATA		0x80
#define MAX77779_SP_MAX_ADDR       	0xff

struct max77779_sp_data {
	struct device *dev;
	struct regmap *regmap;
	struct dentry *de;
	struct mutex  page_lock; /* might need spinlock */
	u32 debug_reg_address;
};

bool max77779_sp_is_reg(struct device *dev, unsigned int reg);
int max77779_sp_init(struct max77779_sp_data *data);
void max77779_sp_remove(struct max77779_sp_data *data);

#endif
