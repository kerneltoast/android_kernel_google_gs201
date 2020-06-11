/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/linux/mfd/samsung/s2mpg1x-register.h
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * header file including common register information of s2mpg10, s2mpg11
 */

#ifndef __LINUX_MFD_S2MPG1X_REGISTER_H
#define __LINUX_MFD_S2MPG1X_REGISTER_H

#define REG_VAL(val, offset, mask) (((val) << (offset)) & (mask))

#define ENUM_STR(x, r)                                                    \
	{                                                                 \
	case x:                                                           \
		r = #x;                                                   \
		break;                                                    \
	}

typedef enum {
	INT_7P_8125HZ = 0,
	INT_15P_625HZ,
	INT_31P_25HZ,
	INT_62P_5HZ,
	INT_125HZ,
	INT_250HZ,
	INT_500HZ,
	INT_1000HZ,
} s2mpg1x_int_samp_rate;

typedef enum {
	EXT_7P_628125HZ = 0,
	EXT_15P_25625HZ,
	EXT_30P_5125HZ,
	EXT_61P_025HZ,
	EXT_122P_05HZ,
} s2mpg1x_ext_samp_rate;

typedef enum {
	S2MPG1X_METER_POWER = 0,
	S2MPG1X_METER_CURRENT = 1,
} s2mpg1x_meter_mode;

/* MUXSEL0~7 */
#define MUXSEL_MASK 0x7F

#define S2MPG1X_METER_LPF 0
#define S2MPG1X_METER_ACC 1

#define S2MPG1X_METER_CHANNEL_MAX 8
#define S2MPG1X_METER_LPF_BUF 3 /* 21-bit */
#define S2MPG1X_METER_ACC_BUF 6 /* 41-bit */
#define S2MPG1X_METER_COUNT_BUF 3 /* 20-bit */

/* S2MPG1x_METER_CTRL1 */
#define METER_EN_MASK BIT(0)
#define EXT_METER_EN_MASK BIT(1)
#define INT_SAMP_RATE_SHIFT 2
#define INT_SAMP_RATE_MASK (0x7 << INT_SAMP_RATE_SHIFT)

/* S2MPG1x_METER_CTRL2 */
#define ASYNC_RD_MASK BIT(7)
#define ONESHOT_RD_MASK BIT(6)
#define EXT_METER_CHANNEL_EN_OFFSET 3
#define EXT_METER_CHANNEL_EN_MASK (0x7 << EXT_METER_CHANNEL_EN_OFFSET)
#define EXT_SAMP_RATE_MASK (0x7 << 0)

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
#define CMS_BUCK_CURRENT _IQ30(u32, 0.976800977)
#define CMS_BUCK_POWER _IQ30(u32, 0.006105006)
#define CMD_BUCK_CURRENT _IQ30(u32, 1.953601954)
#define CMD_BUCK_POWER _IQ30(u32, 0.012210012)
#define CMT_BUCK_CURRENT _IQ30(u32, 2.93040293)
#define CMT_BUCK_POWER _IQ30(u32, 0.018315018)
#define VM_CURRENT _IQ30(u32, 0.976800977)
#define VM_POWER _IQ30(u32, 0.012210012)
#define BB_CURRENT _IQ30(u32, 0.879120879)
#define BB_POWER _IQ30(u32, 0.010989011)
#define DVS_NLDO_CURRENT_150mA _IQ30(u32, 0.036630037)
#define DVS_NLDO_POWER_150mA _IQ30(u32, 0.000228938)
#define NLDO_CURRENT_150mA _IQ30(u32, 0.036630037)
#define NLDO_POWER_150mA _IQ30(u32, 0.000457875)
#define PLDO_CURRENT_150mA _IQ30(u32, 0.036630037)
#define PLDO_POWER_150mA _IQ30(u32, 0.000915751)
#define NLDO_CURRENT_300mA _IQ30(u32, 0.073260073)
#define NLDO_POWER_300mA _IQ30(u32, 0.000915751)
#define PLDO_CURRENT_300mA _IQ30(u32, 0.073260073)
#define PLDO_POWER_300mA _IQ30(u32, 0.001831502)
#define DVS_NLDO_CURRENT_450mA _IQ30(u32, 0.10989011)
#define DVS_NLDO_POWER_450mA _IQ30(u32, 0.000686813)
#define NLDO_CURRENT_450mA _IQ30(u32, 0.10989011)
#define NLDO_POWER_450mA _IQ30(u32, 0.001373626)
#define DVS_NLDO_CURRENT_800mA _IQ30(u32, 0.195360195)
#define DVS_NLDO_POWER_800mA _IQ30(u32, 0.001221001)
#define NLDO_CURRENT_800mA _IQ30(u32, 0.195360195)
#define NLDO_POWER_800mA _IQ30(u32, 0.002442002)
#define PLDO_CURRENT_800mA _IQ30(u32, 0.195360195)
#define PLDO_POWER_800mA _IQ30(u32, 0.004884005)
#define NLDO_CURRENT_1000mA _IQ30(u32, 0.244200244)
#define NLDO_POWER_1000mA _IQ30(u32, 0.001418251)
#define EXTERNAL_RESOLUTION_VRAIL _IQ30(u32, 1.3186813)
#define EXTERNAL_RESOLUTION_VSHUNT _IQ30(u32, 0.0079356982)
#define EXTERNAL_RESOLUTION_TRIM BIT(3) // 3 bits

#endif /* __LINUX_MFD_S2MPG1X_REGISTER_H */
