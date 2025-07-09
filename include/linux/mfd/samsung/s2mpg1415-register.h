/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/linux/mfd/samsung/s2mpg1415-register.h
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * header file including common register information of s2mpg14, s2mpg15
 */

#ifndef __LINUX_MFD_S2MPG1415_REGISTER_H
#define __LINUX_MFD_S2MPG1415_REGISTER_H

#include <linux/bits.h>
#include <dt-bindings/power/s2mpg1x-power.h>

#define S2MPG1415_METER_CHANNEL_MAX METER_CHANNEL_MAX
#define MASK(width, shift) GENMASK(shift + width - 1, shift)

enum s2mpg1415_meter_muxsel {
	MUXSEL_NONE,

	BUCK1 = 0x1,
	BUCK2 = 0x2,
	BUCK3 = 0x3,
	BUCK4 = 0x4,
	BUCK5 = 0x5,
	BUCK6 = 0x6,
	BUCK7 = 0x7,
	BUCK8 = 0x8,
	BUCK9 = 0x9,
	BUCK10 = 0xA,
	BUCK11 = 0xB,
	BUCK12 = 0xC,

	BUCKD = 0xD,
	BUCKA = 0xE,
	BUCKC = 0xF,

	BUCKBOOST = 0x10,

	VSEN_V1 = 0x1C,
	VSEN_V2 = 0x1D,
	VSEN_V3 = 0x1E,

	VSEN_P4 = 0x1C,
	VSEN_P5 = 0x1D,
	VSEN_P6 = 0x1E,

	VBAT = 0x1F,

	LDO1 = 0x21,
	LDO2 = 0x22,
	LDO3 = 0x23,
	LDO4 = 0x24,
	LDO5 = 0x25,
	LDO6 = 0x26,
	LDO7 = 0x27,
	LDO8 = 0x28,
	LDO9 = 0x29,
	LDO10 = 0x2A,
	LDO11 = 0x2B,
	LDO12 = 0x2C,
	LDO13 = 0x2D,
	LDO14 = 0x2E,
	LDO15 = 0x2F,
	LDO16 = 0x30,
	LDO17 = 0x31,
	LDO18 = 0x32,
	LDO19 = 0x33,
	LDO20 = 0x34,
	LDO21 = 0x35,
	LDO22 = 0x36,
	LDO23 = 0x37,
	LDO24 = 0x38,
	LDO25 = 0x39,
	LDO26 = 0x3A,
	LDO27 = 0x3B,
	LDO28 = 0x3C,
	LDO29 = 0x3D,
	LDO30 = 0x3E,
	LDO31 = 0x3F,

	VSEN_C1 = 0x5C,
	VSEN_C2 = 0x5D,
	VSEN_C3 = 0x5E,

	VSEN_C4 = 0x5C,
	VSEN_C5 = 0x5D,
	VSEN_C6 = 0x5E
};

/**
 * sec_opmode_data - regulator operation mode data
 * @id: regulator id
 * @mode: regulator operation mode
 */
struct sec_opmode_data {
	int id;
	unsigned int mode;
};

/**
 * struct sec_wtsr_smpl - settings for WTSR/SMPL
 * @wtsr_en:		WTSR Function Enable Control
 * @smpl_en:		SMPL Function Enable Control
 * @wtsr_timer_val:	Set the WTSR timer Threshold
 * @smpl_timer_val:	Set the SMPL timer Threshold
 * @check_jigon:	if this value is true, do not enable SMPL function when
 *			JIGONB is low(JIG cable is attached)
 */
struct sec_wtsr_smpl {
	bool wtsr_en;
	bool coldrst_en;
	bool smpl_en;
	bool sub_smpl_en;
	int wtsr_timer_val;
	int coldrst_timer_val;
	int smpl_timer_val;
	bool check_jigon;
};

#define REG_VAL(val, offset, mask) (((val) << (offset)) & (mask))

#define ENUM_STR(x, s, r)		\
	{				\
		case x:			\
			r = #x s;	\
			break;		\
	}

enum s2mpg1415_int_samp_rate {
	INT_7P_8125HZ,
	INT_15P_625HZ,
	INT_31P_25HZ,
	INT_62P_5HZ,
	INT_125HZ,
	INT_250HZ,
	INT_1000HZ,
	INT_FREQ_COUNT,
	INT_FREQ_NONE,
};

enum s2mpg1415_ext_samp_rate {
	EXT_7P_8125HZ,
	EXT_15P_625HZ,
	EXT_31P_25HZ,
	EXT_62P_5HZ,
	EXT_125HZ,
	EXT_FREQ_COUNT,
	EXT_FREQ_NONE,
};

enum s2mpg1415_int_acquisition_time_us {
	INT_7P_8125HZ_ACQUISITION_TIME_US = 128000,
	INT_15P_625HZ_ACQUISITION_TIME_US = 64000,
	INT_31P_25HZ_ACQUISITION_TIME_US = 32000,
	INT_62P_5HZ_ACQUISITION_TIME_US = 16000,
	INT_125HZ_ACQUISITION_TIME_US = 8000,
	INT_250HZ_ACQUISITION_TIME_US = 4000,
	INT_1000HZ_ACQUISITION_TIME_US = 1000,
};

enum s2mpg1415_meter_mode {
	S2MPG1415_METER_POWER,
	S2MPG1415_METER_CURRENT,
};

/* MUXSEL0~7 */
#define MUXSEL_MASK 0x7F

#define S2MPG1415_METER_LPF 0
#define S2MPG1415_METER_ACC 1

#define S2MPG1415_METER_LPF_BUF 3 /* 21-bit */
#define S2MPG1415_METER_ACC_BUF 6 /* 41-bit */
#define S2MPG1415_METER_COUNT_BUF 3 /* 20-bit */
#define S2MPG1415_METER_BUCKEN_BUF 2

/* S2MPG1415_METER_CTRL1 */
#define NTC_SAMP_RATE_SHIFT 5
#define NTC_SAMP_RATE_MASK (0x7 << NTC_SAMP_RATE_SHIFT)
#define INT_SAMP_RATE_SHIFT 2
#define INT_SAMP_RATE_MASK (0x7 << INT_SAMP_RATE_SHIFT)
#define EXT_METER_EN_MASK BIT(1)
#define METER_EN_MASK BIT(0)

typedef enum {
	NTC_0P15625HZ = 1,
	NTC_0P3125HZ,
	NTC_0P625HZ,
	NTC_1P25HZ,
	NTC_2P5HZ,
	NTC_5HZ,
	NTC_10HZ,
} s2mpg1415_ntc_samp_rate;

/* S2MPG1415_METER_CTRL2 */
#define ASYNC_RD_MASK BIT(7)
#define ONESHOT_RD_MASK BIT(6)
#define EXT_METER_CHANNEL_EN_ALL 0x7
#define EXT_METER_CHANNEL_EN_OFFSET 3
#define EXT_METER_CHANNEL_EN_MASK (0x7 << EXT_METER_CHANNEL_EN_OFFSET)
#define EXT_SAMP_RATE_MASK (0x7 << 0)

/* S2MPG1415_METER_CTRL5 */
#define SOFT_RST_MASK BIT(6)

#define _IQX(_pow, _type, _val) ((_type)((_val) * (1 << (_pow))))
#define _IQX_to_int(_pow, _iqval) ((_iqval) >> (_pow))
// Note: powx > powy
#define _IQX_to_IQY(_powx, _powy, _iqxval) \
	((_iqxval) >> ((_powx) - (_powy)))

#define _IQ30(_type, _val) _IQX(30, _type, _val)
#define _IQ30_to_IQ22(_iqval) _IQX_to_IQY(30, 22, _iqval)
#define _IQ22_to_int(_iqval) _IQX_to_int(22, _iqval)
#define _IQ30_to_int(_iqval) _IQX_to_int(30, _iqval)

/* 1-bit resolution define for meter */
/* Unit : 1mA, 1mW, 1mV, converted to unsigned _iq30 */
/* Note: A value above or equal 4.0 would cause an overflow on a u32 IQ30 */
#define INVALID_RESOLUTION ((u32)(0))
#define CMS_BUCK_CURRENT _IQ30(u32, 1.0989010989011)
#define CMS_BUCK_POWER _IQ30(u32, 0.006868131868131)
#define CMD_BUCK_CURRENT _IQ30(u32, 2.1978021978022)
#define CMD_BUCK_POWER _IQ30(u32, 0.013736263736263)
#define CMT_BUCK_CURRENT _IQ30(u32, 3.2967032967033)
#define CMT_BUCK_POWER _IQ30(u32, 0.020604395604395)
#define VM_CURRENT _IQ30(u32, 1.0989010989011)
#define VM_POWER _IQ30(u32, 0.013736263736263)
#define BB_CURRENT _IQ30(u32, 0.879120879)
#define BB_POWER _IQ30(u32, 0.010989011)
#define NLDO_CURRENT_150mA _IQ30(u32, 0.036630036630036)
#define NLDO_POWER_150mA _IQ30(u32, 0.000457875457875)
#define PLDO_CURRENT_150mA _IQ30(u32, 0.036630036630036)
#define PLDO_POWER_150mA _IQ30(u32, 0.000915750915750)
#define NLDO_CURRENT_300mA _IQ30(u32, 0.073260073260073)
#define NLDO_POWER_300mA _IQ30(u32, 0.000915750915750)
#define PLDO_CURRENT_300mA _IQ30(u32, 0.073260073260073)
#define PLDO_POWER_300mA _IQ30(u32, 0.001831501831501)
#define DVS_NLDO_CURRENT_450mA _IQ30(u32, 0.10989010989011)
#define DVS_NLDO_POWER_450mA _IQ30(u32, 0.000686813)
#define NLDO_CURRENT_450mA _IQ30(u32, 0.10989010989011)
#define NLDO_POWER_450mA _IQ30(u32, 0.001373626373626)
#define PLDO_CURRENT_600mA _IQ30(u32, 0.146520146520147)
#define PLDO_POWER_600mA _IQ30(u32, 0.003663003663003)
#define DVS_NLDO_CURRENT_800mA _IQ30(u32, 0.195360195360195)
#define DVS_NLDO_POWER_800mA_OFF _IQ30(u32, 0.002442002442002)
#define DVS_NLDO_POWER_800mA_ON _IQ30(u32, 0.001221001221001)
#define NLDO_CURRENT_800mA _IQ30(u32, 0.195360195360195)
#define NLDO_POWER_800mA _IQ30(u32, 0.002442002442002)
#define PLDO_CURRENT_800mA _IQ30(u32,0.195360195360195)
#define PLDO_POWER_800mA _IQ30(u32, 0.004884004884004)
#define DVS_NLDO_CURRENT_1200mA _IQ30(u32, 0.293040293040293)
#define DVS_NLDO_POWER_1200mA_OFF _IQ30(u32, 0.003663003663003)
#define DVS_NLDO_POWER_1200mA_ON _IQ30(u32, 0.001831501831501)
#define NLDO_CURRENT_1200mA _IQ30(u32, 0.293040293040293)
#define NLDO_POWER_1200mA _IQ30(u32, 0.001831501831501)
#define EXT_UNDER_62_5HZ_CURRENT _IQ30(u32, 0.007935698)
#define EXT_UNDER_62_5HZ_POWER _IQ30(u32, 0.01397676)
#define EXT_125HZ_CURRENT _IQ30(u32, 0.008899436)
#define EXT_125HZ_POWER _IQ30(u32, 0.0156741534246575)
#define HRMODE_CM_BUCK_CURRENT _IQ30(u32, 0.0017)
#define HRMODE_VM_BUCK_CURRENT _IQ30(u32, 0.0009)
#define HRMODE_LDOS_CURRENT _IQ30(u32, 0.001470588)

#define EXTERNAL_RESOLUTION_VRAIL _IQ30(u32, 2.1978021)
#define EXTERNAL_RESOLUTION_VSHUNT _IQ30(u32, 0.7935698) // mA/LSB
#define EXTERNAL_RESOLUTION_TRIM BIT(3) // 3 bits

#endif /* __LINUX_MFD_S2MPG1415_REGISTER_H */
