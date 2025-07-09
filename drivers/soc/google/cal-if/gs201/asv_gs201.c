// SPDX-License-Identifier: <GPL-2.0-only>
/*
 * asv function implementation
 *
 * Copyright 2022 Google LLC
 */
#include "asv_gs201.h"

#include <dt-bindings/clock/gs201.h>

#define OTP_CON_TOP_BASE	(0x10000000)
#define OTP_CON_TOP_ASV_TBL0_0	(OTP_CON_TOP_BASE + 0x9000)
#define ASV_TBL0_0_OFFSET	(0x0)
#define ASV_TBL0_1_OFFSET	(0x4)
#define ASV_TBL0_2_OFFSET	(0x8)
#define ASV_TBL0_3_OFFSET	(0xC)

static struct chip_info {
	char szWaferID[7];
	unsigned char nWaferNum;
	unsigned char nWaferX;
	unsigned char nWaferY;
	unsigned char nAsv_TABLE;

	unsigned char nAsv_CPUCL0;
	unsigned char nAsv_CPUCL1;
	unsigned char nAsv_CPUCL2;
	unsigned char nAsv_G3D;
	unsigned char nAsv_MIF;
	unsigned char nAsv_INT;
	unsigned char nAsv_CAM;
	unsigned char nAsv_TPU;
	unsigned char nAsv_AOC;

	int modi_CPUCL0:4;
	int modi_CPUCL1:4;
	int modi_CPUCL2:4;
	int modi_G3D:4;
	int modi_MIF:4;
	int modi_INT:4;
	int modi_CAM:4;
	int modi_TPU:4;
	int modi_AOC:4;

	unsigned int asv_vmin[12];
	unsigned int asv_freq[12];
} chip_info;

int asv_get_grp(unsigned int id)
{
	int grp = -1;

	switch(id) {
	case ACPM_DVFS_MIF:
		grp = chip_info.nAsv_MIF + chip_info.modi_MIF;;
		break;
	case ACPM_DVFS_INT:
		grp = chip_info.nAsv_INT + chip_info.modi_INT;
		break;
	case ACPM_DVFS_CPUCL0:
		grp = chip_info.nAsv_CPUCL0 + chip_info.modi_CPUCL0;
		break;
	case ACPM_DVFS_CPUCL1:
		grp = chip_info.nAsv_CPUCL1 + chip_info.modi_CPUCL1;
		break;
	case ACPM_DVFS_CPUCL2:
		grp = chip_info.nAsv_CPUCL2 + chip_info.modi_CPUCL2;
		break;
	case ACPM_DVFS_G3D:
		grp = chip_info.nAsv_G3D + chip_info.modi_G3D;
		break;
	case ACPM_DVFS_TPU:
		grp = chip_info.nAsv_TPU + chip_info.modi_TPU;
		break;
	case ACPM_DVFS_CAM:
		grp = chip_info.nAsv_CAM + chip_info.modi_CAM;
		break;
	default:
		break;
	}

	return grp;
}

int asv_get_ids_info(unsigned int id)
{
	int ids = 0;

	return ids;
}

/*
 * OTP bits are specified in
 * https://docs.google.com/spreadsheets/d/1EDGj2ZcTytrgBnCfwXxX3ChpMrVmtKuq5sjbs8e_jdw/edit?resourcekey=0-JkGfpJv2U7d7qsoNNKtskA#gid=916237836
 */
void asv_init(void)
{
	unsigned int uBits;
	void __iomem *asv_base = ioremap(OTP_CON_TOP_ASV_TBL0_0, 0x10);

	// Get Table Version
	uBits =  ioread32(asv_base + ASV_TBL0_3_OFFSET);
	chip_info.nAsv_TABLE = (uBits >> 0) & 0x7F;

	uBits = ioread32(asv_base + ASV_TBL0_0_OFFSET);
	chip_info.nAsv_CPUCL2 = (char)((uBits >> 0) & 0xF);
	chip_info.nAsv_CPUCL1 = (char)((uBits >> 4) & 0xF);
	chip_info.nAsv_CPUCL0 = (char)((uBits >> 8) & 0xF);
	chip_info.nAsv_G3D = (char)((uBits >> 12) & 0xF);
	chip_info.nAsv_MIF = (char)((uBits >> 16) & 0xF);
	chip_info.nAsv_TPU = (char)((uBits >> 20) & 0xF);
	chip_info.nAsv_INT = (char)((uBits >> 24) & 0xF);
	chip_info.nAsv_CAM = (char)((uBits >> 28) & 0xF);

	uBits = ioread32(asv_base + ASV_TBL0_1_OFFSET);
	chip_info.nAsv_AOC = (char)((uBits >> 0) & 0xF);
	chip_info.modi_AOC = (char)((uBits >> 28) & 0xF);

	uBits = ioread32(asv_base + ASV_TBL0_2_OFFSET);
	chip_info.modi_CPUCL2 = (char)((uBits >> 0) & 0xF);
	chip_info.modi_CPUCL1 = (char)((uBits >> 4) & 0xF);
	chip_info.modi_CPUCL0 = (char)((uBits >> 8) & 0xF);
	chip_info.modi_G3D = (char)((uBits >> 12) & 0xF);
	chip_info.modi_MIF = (char)((uBits >> 16) & 0xF);
	chip_info.modi_TPU = (char)((uBits >> 20) & 0xF);
	chip_info.modi_INT = (char)((uBits >> 24) & 0xF);
	chip_info.modi_CAM = (char)((uBits >> 28) & 0xF);
}