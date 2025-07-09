/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * rtc-s2mpg14.h
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 */

#ifndef __LINUX_MFD_SEC_RTC_H
#define __LINUX_MFD_SEC_RTC_H

/* RTC(0x2) Registers */
enum S2MPG14_RTC_REG {
	S2MPG14_RTC_CTRL = 0x0,
	S2MPG14_RTC_UPDATE = 0x1,
	S2MPG14_RTC_SMPL = 0x2,
	S2MPG14_RTC_WTSR = 0x3,
	S2MPG14_RTC_CAPSEL = 0x4,
	S2MPG14_RTC_MSEC = 0x5,
	S2MPG14_RTC_SEC = 0x6,
	S2MPG14_RTC_MIN = 0x7,
	S2MPG14_RTC_HOUR = 0x8,
	S2MPG14_RTC_WEEK = 0x9,
	S2MPG14_RTC_DAY = 0xA,
	S2MPG14_RTC_MON = 0xB,
	S2MPG14_RTC_YEAR = 0xC,
	S2MPG14_RTC_A0SEC = 0xD,
	S2MPG14_RTC_A0MIN = 0xE,
	S2MPG14_RTC_A0HOUR = 0xF,
	S2MPG14_RTC_A0WEEK = 0x10,
	S2MPG14_RTC_A0DAY = 0x11,
	S2MPG14_RTC_A0MON = 0x12,
	S2MPG14_RTC_A0YEAR = 0x13,
	S2MPG14_RTC_A1SEC = 0x14,
	S2MPG14_RTC_A1MIN = 0x15,
	S2MPG14_RTC_A1HOUR = 0x16,
	S2MPG14_RTC_A1WEEK = 0x17,
	S2MPG14_RTC_A1DAY = 0x18,
	S2MPG14_RTC_A1MON = 0x19,
	S2MPG14_RTC_A1YEAR = 0x1A,
	S2MPG14_RTC_OSC_CTRL = 0x1B,
	S2MPG14_RTC_NONCE0_0 = 0x1C,
	S2MPG14_RTC_NONCE0_1 = 0x1D,
	S2MPG14_RTC_NONCE0_2 = 0x1E,
	S2MPG14_RTC_NONCE0_3 = 0x1F,
	S2MPG14_RTC_NONCE0_4 = 0x20,
	S2MPG14_RTC_NONCE0_5 = 0x21,
	S2MPG14_RTC_NONCE0_6 = 0x22,
	S2MPG14_RTC_NONCE0_7 = 0x23,
	S2MPG14_RTC_NONCE1_0 = 0x24,
	S2MPG14_RTC_NONCE1_1 = 0x25,
	S2MPG14_RTC_NONCE1_2 = 0x26,
	S2MPG14_RTC_NONCE1_3 = 0x27,
	S2MPG14_RTC_NONCE1_4 = 0x28,
	S2MPG14_RTC_NONCE1_5 = 0x29,
	S2MPG14_RTC_NONCE1_6 = 0x2A,
	S2MPG14_RTC_NONCE1_7 = 0x2B,
	S2MPG14_RTC_SCRATCH0 = 0x2C,
	S2MPG14_RTC_SCRATCH1 = 0x2D,
	S2MPG14_RTC_SCRATCH2 = 0x2E,
	S2MPG14_RTC_SCRATCH3 = 0x2F,
	S2MPG14_RTC_SCRATCH4 = 0x30,
};

/* S2MPG14_RTC_CTRL (0x200) */
#define BCD_EN_SHIFT	0
#define MODEL24_SHIFT	1

/* S2MPG14_RTC_UPDATE (0x201) */
#define RTC_RUDR_SHIFT		0
#define RTC_WUDR_SHIFT		1
#define RTC_FREEZE_SHIFT	2
#define RTC_AUDR_SHIFT		4

/* S2MPG14_RTC_SMPL (0x202) */
#define SMPLT_SHIFT	1
#define SMPLT_MASK	(0x7 << SMPLT_SHIFT)
#define SMPL_EN_SHIFT	0
#define SMPL_EN_MASK	(0x1 << SMPL_EN_SHIFT)

/* S2MPG14_RTC_WTSR (0x203) */
#define WTSRT_SHIFT		1
#define WTSRT_MASK		(0x7 << WTSRT_SHIFT)
#define WTSR_EN_SHIFT		0
#define WTSR_EN_MASK		(0x1 << WTSR_EN_SHIFT)
#define COLDRST_EN_SHIFT	4
#define COLDRST_EN_MASK		(0x1 << COLDRST_EN_SHIFT)
#define COLDRST_TIMER_SHIFT	5
#define COLDRST_TIMER_MASK	(0x3 << COLDRST_TIMER_SHIFT)

/* RTC HOUR Register */
#define HOUR_PM_SHIFT 6

/* RTC Alarm Enable */
#define ALARM_ENABLE_SHIFT 7

/* PM STATUS2 Register */
#define RTCA0E_SHIFT 2
#define RTCA1E_SHIFT 1

#define WTSR_TIMER_BITS(v) (((v) << WTSRT_SHIFT) & WTSRT_MASK)
#define SMPL_TIMER_BITS(v) (((v) << SMPLT_SHIFT) & SMPLT_MASK)

/* RTC Optimize */
#define CAP_SEL_SHIFT 0
#define CAP_SEL_MASK (0x3 << CAP_SEL_SHIFT)
#define OSC_BIAS_UP_SHIFT 2
#define OSC_XIN_SHIFT 5
#define OSC_XIN_MASK (0x7 << OSC_XIN_SHIFT)
#define OSC_XOUT_SHIFT 2
#define OSC_XOUT_MASK (0x7 << OSC_XOUT_SHIFT)

#define NONCE0_CNT 8
#define NONCE0_0_SPECIAL_VAL 222 /* Can be anything except 0 */

struct s2m_rtc_info {
	struct device *dev;
	struct i2c_client *i2c;
	struct i2c_client *pmic_i2c;
	struct s2mpg14_dev *iodev;
	struct rtc_device *rtc_dev;

	/* mutex for RTC */
	struct mutex lock;
	struct delayed_work irq_work;
	int alarm0_irq;

	bool use_irq;
	bool wtsr_en;
	bool coldrst_en;
	bool smpl_en;
	bool alarm_enabled;
	u8 update_reg;
	bool use_alarm_workaround;
	bool alarm_check;
	u8 wudr_mask;
	u8 audr_mask;
};

/* RTC Counter Register offsets */
#ifdef CONFIG_RTC_DRV_S2MP
enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	NR_RTC_CNT_REGS,
};
#else
enum {
	/*	RTC_MSEC = 0, */
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	NR_RTC_CNT_REGS,
};
#endif

enum S2M_RTC_OP {
	S2M_RTC_READ,
	S2M_RTC_WRITE_TIME,
	S2M_RTC_WRITE_ALARM,
};

#endif /*  __LINUX_MFD_SEC_RTC_H */
