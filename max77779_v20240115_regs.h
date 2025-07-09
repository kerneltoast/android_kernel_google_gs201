/* SPDX-License-Identifier: GPL-2.0 */
/*
 ****************************************************************************
 *
 * Make sure license terms are appropriate for target project before copying
 *
 ****************************************************************************
 *
 * machine generated DO NOT MODIFY
 * source Sequoia_RegMap_customer_2024_01_15_v2.xml
 * 2024-01-26
 */

#ifndef SEQUOIA_REGMAP_CUSTOMER_2024_01_15_V2_REG_H_
#define SEQUOIA_REGMAP_CUSTOMER_2024_01_15_V2_REG_H_

/* needs linux/bits.h */

#define MAX77779_BFF(name, h, l) \
static inline uint8_t _ ## name ## _set(uint8_t r, uint8_t v) \
{ \
	return ((r & ~GENMASK(h, l)) | v << l); \
} \
\
static inline uint8_t _ ## name ## _get(uint8_t r) \
{ \
	return ((r & GENMASK(h, l)) >> l); \
}


#define FIELD2VALUE(field,value) \
	(((value) & field##_MASK) >> field##_SHIFT)
#define VALUE2FIELD(field,       value) \
	(((value) << field##_SHIFT) & field##_MASK)



/*******************************************************
 * Section: PMIC_FUNC 0x00 8
 */

/*
 * MAX77779_PMIC_ID,0x00,0b01111001,0x79,Reset_Type:O
 */
#define MAX77779_PMIC_ID	0x00

/*
 * MAX77779_PMIC_REVISION,0x01,0b00000010,0x2,Reset_Type:O
 * REV[0:3],VER[3:5]
 */
#define MAX77779_PMIC_REVISION	0x01
#define MAX77779_PMIC_REVISION_REV_SHIFT	0
#define MAX77779_PMIC_REVISION_REV_MASK	(0x7 << 0)
#define MAX77779_PMIC_REVISION_REV_CLEAR	(~(0x7 << 0))
#define MAX77779_PMIC_REVISION_VER_SHIFT	3
#define MAX77779_PMIC_REVISION_VER_MASK	(0x1f << 3)
#define MAX77779_PMIC_REVISION_VER_CLEAR	(~(0x1f << 3))
static inline const char *
max77779_pmic_revision_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " REV=%x",
		FIELD2VALUE(MAX77779_REV, val));
	i += scnprintf(&buff[i], len - i, " VER=%x",
		FIELD2VALUE(MAX77779_VER, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_revision_rev,2,0)
MAX77779_BFF(max77779_pmic_revision_ver,7,3)

/*
 * MAX77779_PMIC_OTP_REVISION,0x02,0b00100001,0x21,OTP:SHADOW, Reset_Type:O
 */
#define MAX77779_PMIC_OTP_REVISION	0x02

/*
 * MAX77779_PMIC_INTSRC_STS,0x22,0b00000000,0x0,Reset_Type:S
 * TCPC_INT,FG_INT,CHGR_INT,I2CM_INT,BATTVIMON_INT,GPIO_INT,VDROOP_INT,PMICTOP_INT
 */
#define MAX77779_PMIC_INTSRC_STS	0x22
#define MAX77779_PMIC_INTSRC_STS_TCPC_INT_SHIFT	0
#define MAX77779_PMIC_INTSRC_STS_TCPC_INT_MASK	(0x1 << 0)
#define MAX77779_PMIC_INTSRC_STS_TCPC_INT_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_INTSRC_STS_FG_INT_SHIFT	1
#define MAX77779_PMIC_INTSRC_STS_FG_INT_MASK	(0x1 << 1)
#define MAX77779_PMIC_INTSRC_STS_FG_INT_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_INTSRC_STS_CHGR_INT_SHIFT	2
#define MAX77779_PMIC_INTSRC_STS_CHGR_INT_MASK	(0x1 << 2)
#define MAX77779_PMIC_INTSRC_STS_CHGR_INT_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_INTSRC_STS_I2CM_INT_SHIFT	3
#define MAX77779_PMIC_INTSRC_STS_I2CM_INT_MASK	(0x1 << 3)
#define MAX77779_PMIC_INTSRC_STS_I2CM_INT_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_INTSRC_STS_BATTVIMON_INT_SHIFT	4
#define MAX77779_PMIC_INTSRC_STS_BATTVIMON_INT_MASK	(0x1 << 4)
#define MAX77779_PMIC_INTSRC_STS_BATTVIMON_INT_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_INTSRC_STS_GPIO_INT_SHIFT	5
#define MAX77779_PMIC_INTSRC_STS_GPIO_INT_MASK	(0x1 << 5)
#define MAX77779_PMIC_INTSRC_STS_GPIO_INT_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_INTSRC_STS_VDROOP_INT_SHIFT	6
#define MAX77779_PMIC_INTSRC_STS_VDROOP_INT_MASK	(0x1 << 6)
#define MAX77779_PMIC_INTSRC_STS_VDROOP_INT_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_INTSRC_STS_PMICTOP_INT_SHIFT	7
#define MAX77779_PMIC_INTSRC_STS_PMICTOP_INT_MASK	(0x1 << 7)
#define MAX77779_PMIC_INTSRC_STS_PMICTOP_INT_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_intsrc_sts_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TCPC_INT=%x",
		FIELD2VALUE(MAX77779_TCPC_INT, val));
	i += scnprintf(&buff[i], len - i, " FG_INT=%x",
		FIELD2VALUE(MAX77779_FG_INT, val));
	i += scnprintf(&buff[i], len - i, " CHGR_INT=%x",
		FIELD2VALUE(MAX77779_CHGR_INT, val));
	i += scnprintf(&buff[i], len - i, " I2CM_INT=%x",
		FIELD2VALUE(MAX77779_I2CM_INT, val));
	i += scnprintf(&buff[i], len - i, " BATTVIMON_INT=%x",
		FIELD2VALUE(MAX77779_BATTVIMON_INT, val));
	i += scnprintf(&buff[i], len - i, " GPIO_INT=%x",
		FIELD2VALUE(MAX77779_GPIO_INT, val));
	i += scnprintf(&buff[i], len - i, " VDROOP_INT=%x",
		FIELD2VALUE(MAX77779_VDROOP_INT, val));
	i += scnprintf(&buff[i], len - i, " PMICTOP_INT=%x",
		FIELD2VALUE(MAX77779_PMICTOP_INT, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_intsrc_sts_tcpc_int,0,0)
MAX77779_BFF(max77779_pmic_intsrc_sts_fg_int,1,1)
MAX77779_BFF(max77779_pmic_intsrc_sts_chgr_int,2,2)
MAX77779_BFF(max77779_pmic_intsrc_sts_i2cm_int,3,3)
MAX77779_BFF(max77779_pmic_intsrc_sts_battvimon_int,4,4)
MAX77779_BFF(max77779_pmic_intsrc_sts_gpio_int,5,5)
MAX77779_BFF(max77779_pmic_intsrc_sts_vdroop_int,6,6)
MAX77779_BFF(max77779_pmic_intsrc_sts_pmictop_int,7,7)

/*
 * MAX77779_PMIC_VDROOP_INT,0x23,0b00000000,0x0,Reset_Type:S
 * OILO2_CNT_INT,OILO1_CNT_INT,UVLO2_CNT_INT,UVLO1_CNT_INT,BAT_OILO2_INT,
 * BAT_OILO1_INT,SYS_UVLO2_INT,SYS_UVLO1_INT
 */
#define MAX77779_PMIC_VDROOP_INT	0x23
#define MAX77779_PMIC_VDROOP_INT_OILO2_CNT_INT_SHIFT	0
#define MAX77779_PMIC_VDROOP_INT_OILO2_CNT_INT_MASK	(0x1 << 0)
#define MAX77779_PMIC_VDROOP_INT_OILO2_CNT_INT_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_VDROOP_INT_OILO1_CNT_INT_SHIFT	1
#define MAX77779_PMIC_VDROOP_INT_OILO1_CNT_INT_MASK	(0x1 << 1)
#define MAX77779_PMIC_VDROOP_INT_OILO1_CNT_INT_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_VDROOP_INT_UVLO2_CNT_INT_SHIFT	2
#define MAX77779_PMIC_VDROOP_INT_UVLO2_CNT_INT_MASK	(0x1 << 2)
#define MAX77779_PMIC_VDROOP_INT_UVLO2_CNT_INT_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_VDROOP_INT_UVLO1_CNT_INT_SHIFT	3
#define MAX77779_PMIC_VDROOP_INT_UVLO1_CNT_INT_MASK	(0x1 << 3)
#define MAX77779_PMIC_VDROOP_INT_UVLO1_CNT_INT_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_SHIFT	4
#define MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_MASK	(0x1 << 4)
#define MAX77779_PMIC_VDROOP_INT_BAT_OILO2_INT_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_SHIFT	5
#define MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_MASK	(0x1 << 5)
#define MAX77779_PMIC_VDROOP_INT_BAT_OILO1_INT_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_SHIFT	6
#define MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_MASK	(0x1 << 6)
#define MAX77779_PMIC_VDROOP_INT_SYS_UVLO2_INT_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_SHIFT	7
#define MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_MASK	(0x1 << 7)
#define MAX77779_PMIC_VDROOP_INT_SYS_UVLO1_INT_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_vdroop_int_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " OILO2_CNT_INT=%x",
		FIELD2VALUE(MAX77779_OILO2_CNT_INT, val));
	i += scnprintf(&buff[i], len - i, " OILO1_CNT_INT=%x",
		FIELD2VALUE(MAX77779_OILO1_CNT_INT, val));
	i += scnprintf(&buff[i], len - i, " UVLO2_CNT_INT=%x",
		FIELD2VALUE(MAX77779_UVLO2_CNT_INT, val));
	i += scnprintf(&buff[i], len - i, " UVLO1_CNT_INT=%x",
		FIELD2VALUE(MAX77779_UVLO1_CNT_INT, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO2_INT=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_INT, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_INT=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_INT, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO2_INT=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2_INT, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO1_INT=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1_INT, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_vdroop_int_oilo2_cnt_int,0,0)
MAX77779_BFF(max77779_pmic_vdroop_int_oilo1_cnt_int,1,1)
MAX77779_BFF(max77779_pmic_vdroop_int_uvlo2_cnt_int,2,2)
MAX77779_BFF(max77779_pmic_vdroop_int_uvlo1_cnt_int,3,3)
MAX77779_BFF(max77779_pmic_vdroop_int_bat_oilo2_int,4,4)
MAX77779_BFF(max77779_pmic_vdroop_int_bat_oilo1_int,5,5)
MAX77779_BFF(max77779_pmic_vdroop_int_sys_uvlo2_int,6,6)
MAX77779_BFF(max77779_pmic_vdroop_int_sys_uvlo1_int,7,7)

/*
 * MAX77779_PMIC_INTB_MASK,0x24,0b11111111,0xff,OTP:SHADOW, Reset_Type:S
 * TCPC_INT_M,FG_INT_M,CHGR_INT_M,I2CM_INT_M,BATTVIMON_INT_M,GPIO_INT_M,
 * VDROOP_INT_M,PMICTOP_INT_M
 */
#define MAX77779_PMIC_INTB_MASK	0x24
#define MAX77779_PMIC_INTB_MASK_TCPC_INT_M_SHIFT	0
#define MAX77779_PMIC_INTB_MASK_TCPC_INT_M_MASK	(0x1 << 0)
#define MAX77779_PMIC_INTB_MASK_TCPC_INT_M_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_INTB_MASK_FG_INT_M_SHIFT	1
#define MAX77779_PMIC_INTB_MASK_FG_INT_M_MASK	(0x1 << 1)
#define MAX77779_PMIC_INTB_MASK_FG_INT_M_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_INTB_MASK_CHGR_INT_M_SHIFT	2
#define MAX77779_PMIC_INTB_MASK_CHGR_INT_M_MASK	(0x1 << 2)
#define MAX77779_PMIC_INTB_MASK_CHGR_INT_M_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_INTB_MASK_I2CM_INT_M_SHIFT	3
#define MAX77779_PMIC_INTB_MASK_I2CM_INT_M_MASK	(0x1 << 3)
#define MAX77779_PMIC_INTB_MASK_I2CM_INT_M_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_INTB_MASK_BATTVIMON_INT_M_SHIFT	4
#define MAX77779_PMIC_INTB_MASK_BATTVIMON_INT_M_MASK	(0x1 << 4)
#define MAX77779_PMIC_INTB_MASK_BATTVIMON_INT_M_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_INTB_MASK_GPIO_INT_M_SHIFT	5
#define MAX77779_PMIC_INTB_MASK_GPIO_INT_M_MASK	(0x1 << 5)
#define MAX77779_PMIC_INTB_MASK_GPIO_INT_M_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_INTB_MASK_VDROOP_INT_M_SHIFT	6
#define MAX77779_PMIC_INTB_MASK_VDROOP_INT_M_MASK	(0x1 << 6)
#define MAX77779_PMIC_INTB_MASK_VDROOP_INT_M_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_INTB_MASK_PMICTOP_INT_M_SHIFT	7
#define MAX77779_PMIC_INTB_MASK_PMICTOP_INT_M_MASK	(0x1 << 7)
#define MAX77779_PMIC_INTB_MASK_PMICTOP_INT_M_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_intb_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TCPC_INT_M=%x",
		FIELD2VALUE(MAX77779_TCPC_INT_M, val));
	i += scnprintf(&buff[i], len - i, " FG_INT_M=%x",
		FIELD2VALUE(MAX77779_FG_INT_M, val));
	i += scnprintf(&buff[i], len - i, " CHGR_INT_M=%x",
		FIELD2VALUE(MAX77779_CHGR_INT_M, val));
	i += scnprintf(&buff[i], len - i, " I2CM_INT_M=%x",
		FIELD2VALUE(MAX77779_I2CM_INT_M, val));
	i += scnprintf(&buff[i], len - i, " BATTVIMON_INT_M=%x",
		FIELD2VALUE(MAX77779_BATTVIMON_INT_M, val));
	i += scnprintf(&buff[i], len - i, " GPIO_INT_M=%x",
		FIELD2VALUE(MAX77779_GPIO_INT_M, val));
	i += scnprintf(&buff[i], len - i, " VDROOP_INT_M=%x",
		FIELD2VALUE(MAX77779_VDROOP_INT_M, val));
	i += scnprintf(&buff[i], len - i, " PMICTOP_INT_M=%x",
		FIELD2VALUE(MAX77779_PMICTOP_INT_M, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_intb_mask_tcpc_int_m,0,0)
MAX77779_BFF(max77779_pmic_intb_mask_fg_int_m,1,1)
MAX77779_BFF(max77779_pmic_intb_mask_chgr_int_m,2,2)
MAX77779_BFF(max77779_pmic_intb_mask_i2cm_int_m,3,3)
MAX77779_BFF(max77779_pmic_intb_mask_battvimon_int_m,4,4)
MAX77779_BFF(max77779_pmic_intb_mask_gpio_int_m,5,5)
MAX77779_BFF(max77779_pmic_intb_mask_vdroop_int_m,6,6)
MAX77779_BFF(max77779_pmic_intb_mask_pmictop_int_m,7,7)

/*
 * MAX77779_PMIC_SPMI_INT_MASK,0x25,0b11111111,0xff,Reset_Type:S
 * TCPC_INT_SM,FG_INT_SM,CHGR_INT_SM,I2CM_INT_SM,BATTVIMON_INT_SM,GPIO_INT_SM,
 * VDROOP_INT_SM,PMICTOP_INT_SM
 */
#define MAX77779_PMIC_SPMI_INT_MASK	0x25
#define MAX77779_PMIC_SPMI_INT_MASK_TCPC_INT_SM_SHIFT	0
#define MAX77779_PMIC_SPMI_INT_MASK_TCPC_INT_SM_MASK	(0x1 << 0)
#define MAX77779_PMIC_SPMI_INT_MASK_TCPC_INT_SM_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_SPMI_INT_MASK_FG_INT_SM_SHIFT	1
#define MAX77779_PMIC_SPMI_INT_MASK_FG_INT_SM_MASK	(0x1 << 1)
#define MAX77779_PMIC_SPMI_INT_MASK_FG_INT_SM_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_SPMI_INT_MASK_CHGR_INT_SM_SHIFT	2
#define MAX77779_PMIC_SPMI_INT_MASK_CHGR_INT_SM_MASK	(0x1 << 2)
#define MAX77779_PMIC_SPMI_INT_MASK_CHGR_INT_SM_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_SPMI_INT_MASK_I2CM_INT_SM_SHIFT	3
#define MAX77779_PMIC_SPMI_INT_MASK_I2CM_INT_SM_MASK	(0x1 << 3)
#define MAX77779_PMIC_SPMI_INT_MASK_I2CM_INT_SM_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_SPMI_INT_MASK_BATTVIMON_INT_SM_SHIFT	4
#define MAX77779_PMIC_SPMI_INT_MASK_BATTVIMON_INT_SM_MASK	(0x1 << 4)
#define MAX77779_PMIC_SPMI_INT_MASK_BATTVIMON_INT_SM_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_SPMI_INT_MASK_GPIO_INT_SM_SHIFT	5
#define MAX77779_PMIC_SPMI_INT_MASK_GPIO_INT_SM_MASK	(0x1 << 5)
#define MAX77779_PMIC_SPMI_INT_MASK_GPIO_INT_SM_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_SPMI_INT_MASK_VDROOP_INT_SM_SHIFT	6
#define MAX77779_PMIC_SPMI_INT_MASK_VDROOP_INT_SM_MASK	(0x1 << 6)
#define MAX77779_PMIC_SPMI_INT_MASK_VDROOP_INT_SM_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_SPMI_INT_MASK_PMICTOP_INT_SM_SHIFT	7
#define MAX77779_PMIC_SPMI_INT_MASK_PMICTOP_INT_SM_MASK	(0x1 << 7)
#define MAX77779_PMIC_SPMI_INT_MASK_PMICTOP_INT_SM_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_spmi_int_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TCPC_INT_SM=%x",
		FIELD2VALUE(MAX77779_TCPC_INT_SM, val));
	i += scnprintf(&buff[i], len - i, " FG_INT_SM=%x",
		FIELD2VALUE(MAX77779_FG_INT_SM, val));
	i += scnprintf(&buff[i], len - i, " CHGR_INT_SM=%x",
		FIELD2VALUE(MAX77779_CHGR_INT_SM, val));
	i += scnprintf(&buff[i], len - i, " I2CM_INT_SM=%x",
		FIELD2VALUE(MAX77779_I2CM_INT_SM, val));
	i += scnprintf(&buff[i], len - i, " BATTVIMON_INT_SM=%x",
		FIELD2VALUE(MAX77779_BATTVIMON_INT_SM, val));
	i += scnprintf(&buff[i], len - i, " GPIO_INT_SM=%x",
		FIELD2VALUE(MAX77779_GPIO_INT_SM, val));
	i += scnprintf(&buff[i], len - i, " VDROOP_INT_SM=%x",
		FIELD2VALUE(MAX77779_VDROOP_INT_SM, val));
	i += scnprintf(&buff[i], len - i, " PMICTOP_INT_SM=%x",
		FIELD2VALUE(MAX77779_PMICTOP_INT_SM, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_spmi_int_mask_tcpc_int_sm,0,0)
MAX77779_BFF(max77779_pmic_spmi_int_mask_fg_int_sm,1,1)
MAX77779_BFF(max77779_pmic_spmi_int_mask_chgr_int_sm,2,2)
MAX77779_BFF(max77779_pmic_spmi_int_mask_i2cm_int_sm,3,3)
MAX77779_BFF(max77779_pmic_spmi_int_mask_battvimon_int_sm,4,4)
MAX77779_BFF(max77779_pmic_spmi_int_mask_gpio_int_sm,5,5)
MAX77779_BFF(max77779_pmic_spmi_int_mask_vdroop_int_sm,6,6)
MAX77779_BFF(max77779_pmic_spmi_int_mask_pmictop_int_sm,7,7)

/*
 * MAX77779_PMIC_SPMI_INT_PRIORITY,0x26,0b00000001,0x1,Reset_Type:S
 * SPMI_INT_PR,SPR_7_1[1:7]
 */
#define MAX77779_PMIC_SPMI_INT_PRIORITY	0x26
#define MAX77779_PMIC_SPMI_INT_PRIORITY_SPMI_INT_PR_SHIFT	0
#define MAX77779_PMIC_SPMI_INT_PRIORITY_SPMI_INT_PR_MASK	(0x1 << 0)
#define MAX77779_PMIC_SPMI_INT_PRIORITY_SPMI_INT_PR_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_SPMI_INT_PRIORITY_SPR_7_1_SHIFT	1
#define MAX77779_PMIC_SPMI_INT_PRIORITY_SPR_7_1_MASK	(0x7f << 1)
#define MAX77779_PMIC_SPMI_INT_PRIORITY_SPR_7_1_CLEAR	(~(0x7f << 1))
static inline const char *
max77779_pmic_spmi_int_priority_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SPMI_INT_PR=%x",
		FIELD2VALUE(MAX77779_SPMI_INT_PR, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_1=%x",
		FIELD2VALUE(MAX77779_SPR_7_1, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_spmi_int_priority_spmi_int_pr,0,0)
MAX77779_BFF(max77779_pmic_spmi_int_priority_spr_7_1,7,1)

/*
 * MAX77779_PMIC_VDROOP_INT_MASK,0x27,0b11111111,0xff,OTP:SHADOW, Reset_Type:O
 * OILO2_CNT_M,OILO1_CNT_M,UVLO2_CNT_M,UVLO1_CNT_M,BAT_OILO2_M,BAT_OILO1_M,
 * SYS_UVLO2_M,SYS_UVLO1_M
 */
#define MAX77779_PMIC_VDROOP_INT_MASK	0x27
#define MAX77779_PMIC_VDROOP_INT_MASK_OILO2_CNT_M_SHIFT	0
#define MAX77779_PMIC_VDROOP_INT_MASK_OILO2_CNT_M_MASK	(0x1 << 0)
#define MAX77779_PMIC_VDROOP_INT_MASK_OILO2_CNT_M_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_VDROOP_INT_MASK_OILO1_CNT_M_SHIFT	1
#define MAX77779_PMIC_VDROOP_INT_MASK_OILO1_CNT_M_MASK	(0x1 << 1)
#define MAX77779_PMIC_VDROOP_INT_MASK_OILO1_CNT_M_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_VDROOP_INT_MASK_UVLO2_CNT_M_SHIFT	2
#define MAX77779_PMIC_VDROOP_INT_MASK_UVLO2_CNT_M_MASK	(0x1 << 2)
#define MAX77779_PMIC_VDROOP_INT_MASK_UVLO2_CNT_M_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_VDROOP_INT_MASK_UVLO1_CNT_M_SHIFT	3
#define MAX77779_PMIC_VDROOP_INT_MASK_UVLO1_CNT_M_MASK	(0x1 << 3)
#define MAX77779_PMIC_VDROOP_INT_MASK_UVLO1_CNT_M_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_VDROOP_INT_MASK_BAT_OILO2_M_SHIFT	4
#define MAX77779_PMIC_VDROOP_INT_MASK_BAT_OILO2_M_MASK	(0x1 << 4)
#define MAX77779_PMIC_VDROOP_INT_MASK_BAT_OILO2_M_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_VDROOP_INT_MASK_BAT_OILO1_M_SHIFT	5
#define MAX77779_PMIC_VDROOP_INT_MASK_BAT_OILO1_M_MASK	(0x1 << 5)
#define MAX77779_PMIC_VDROOP_INT_MASK_BAT_OILO1_M_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_VDROOP_INT_MASK_SYS_UVLO2_M_SHIFT	6
#define MAX77779_PMIC_VDROOP_INT_MASK_SYS_UVLO2_M_MASK	(0x1 << 6)
#define MAX77779_PMIC_VDROOP_INT_MASK_SYS_UVLO2_M_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_VDROOP_INT_MASK_SYS_UVLO1_M_SHIFT	7
#define MAX77779_PMIC_VDROOP_INT_MASK_SYS_UVLO1_M_MASK	(0x1 << 7)
#define MAX77779_PMIC_VDROOP_INT_MASK_SYS_UVLO1_M_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_vdroop_int_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " OILO2_CNT_M=%x",
		FIELD2VALUE(MAX77779_OILO2_CNT_M, val));
	i += scnprintf(&buff[i], len - i, " OILO1_CNT_M=%x",
		FIELD2VALUE(MAX77779_OILO1_CNT_M, val));
	i += scnprintf(&buff[i], len - i, " UVLO2_CNT_M=%x",
		FIELD2VALUE(MAX77779_UVLO2_CNT_M, val));
	i += scnprintf(&buff[i], len - i, " UVLO1_CNT_M=%x",
		FIELD2VALUE(MAX77779_UVLO1_CNT_M, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO2_M=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_M, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_M=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_M, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO2_M=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2_M, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO1_M=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1_M, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_vdroop_int_mask_oilo2_cnt_m,0,0)
MAX77779_BFF(max77779_pmic_vdroop_int_mask_oilo1_cnt_m,1,1)
MAX77779_BFF(max77779_pmic_vdroop_int_mask_uvlo2_cnt_m,2,2)
MAX77779_BFF(max77779_pmic_vdroop_int_mask_uvlo1_cnt_m,3,3)
MAX77779_BFF(max77779_pmic_vdroop_int_mask_bat_oilo2_m,4,4)
MAX77779_BFF(max77779_pmic_vdroop_int_mask_bat_oilo1_m,5,5)
MAX77779_BFF(max77779_pmic_vdroop_int_mask_sys_uvlo2_m,6,6)
MAX77779_BFF(max77779_pmic_vdroop_int_mask_sys_uvlo1_m,7,7)

/*
 * MAX77779_PMIC_VDROOP_INT_SPMI_MASK,0x28,0b11111111,0xff,Reset_Type:O
 * OILO2_CNT_SM,OILO1_CNT_SM,UVLO2_CNT_SM,UVLO1_CNT_SM,BAT_OILO2_SM,BAT_OILO1_SM,
 * SYS_UVLO2_SM,SYS_UVLO1_SM
 */
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK	0x28
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_OILO2_CNT_SM_SHIFT	0
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_OILO2_CNT_SM_MASK	(0x1 << 0)
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_OILO2_CNT_SM_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_OILO1_CNT_SM_SHIFT	1
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_OILO1_CNT_SM_MASK	(0x1 << 1)
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_OILO1_CNT_SM_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_UVLO2_CNT_SM_SHIFT	2
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_UVLO2_CNT_SM_MASK	(0x1 << 2)
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_UVLO2_CNT_SM_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_UVLO1_CNT_SM_SHIFT	3
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_UVLO1_CNT_SM_MASK	(0x1 << 3)
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_UVLO1_CNT_SM_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_BAT_OILO2_SM_SHIFT	4
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_BAT_OILO2_SM_MASK	(0x1 << 4)
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_BAT_OILO2_SM_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_BAT_OILO1_SM_SHIFT	5
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_BAT_OILO1_SM_MASK	(0x1 << 5)
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_BAT_OILO1_SM_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_SYS_UVLO2_SM_SHIFT	6
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_SYS_UVLO2_SM_MASK	(0x1 << 6)
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_SYS_UVLO2_SM_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_SYS_UVLO1_SM_SHIFT	7
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_SYS_UVLO1_SM_MASK	(0x1 << 7)
#define MAX77779_PMIC_VDROOP_INT_SPMI_MASK_SYS_UVLO1_SM_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_vdroop_int_spmi_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " OILO2_CNT_SM=%x",
		FIELD2VALUE(MAX77779_OILO2_CNT_SM, val));
	i += scnprintf(&buff[i], len - i, " OILO1_CNT_SM=%x",
		FIELD2VALUE(MAX77779_OILO1_CNT_SM, val));
	i += scnprintf(&buff[i], len - i, " UVLO2_CNT_SM=%x",
		FIELD2VALUE(MAX77779_UVLO2_CNT_SM, val));
	i += scnprintf(&buff[i], len - i, " UVLO1_CNT_SM=%x",
		FIELD2VALUE(MAX77779_UVLO1_CNT_SM, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO2_SM=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_SM, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_SM=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_SM, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO2_SM=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2_SM, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO1_SM=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1_SM, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_vdroop_int_spmi_mask_oilo2_cnt_sm,0,0)
MAX77779_BFF(max77779_pmic_vdroop_int_spmi_mask_oilo1_cnt_sm,1,1)
MAX77779_BFF(max77779_pmic_vdroop_int_spmi_mask_uvlo2_cnt_sm,2,2)
MAX77779_BFF(max77779_pmic_vdroop_int_spmi_mask_uvlo1_cnt_sm,3,3)
MAX77779_BFF(max77779_pmic_vdroop_int_spmi_mask_bat_oilo2_sm,4,4)
MAX77779_BFF(max77779_pmic_vdroop_int_spmi_mask_bat_oilo1_sm,5,5)
MAX77779_BFF(max77779_pmic_vdroop_int_spmi_mask_sys_uvlo2_sm,6,6)
MAX77779_BFF(max77779_pmic_vdroop_int_spmi_mask_sys_uvlo1_sm,7,7)

/*
 * MAX77779_PMIC_VDROOP_INT_SPMI_PRIORITY,0x29,0b00000001,0x1,Reset_Type:O
 * VDROOP_INT_PR,SPR_7_1[1:7]
 */
#define MAX77779_PMIC_VDROOP_INT_SPMI_PRIORITY	0x29
#define MAX77779_PMIC_VDROOP_INT_SPMI_PRIORITY_VDROOP_INT_PR_SHIFT	0
#define MAX77779_PMIC_VDROOP_INT_SPMI_PRIORITY_VDROOP_INT_PR_MASK	(0x1 << 0)
#define MAX77779_PMIC_VDROOP_INT_SPMI_PRIORITY_VDROOP_INT_PR_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_VDROOP_INT_SPMI_PRIORITY_SPR_7_1_SHIFT	1
#define MAX77779_PMIC_VDROOP_INT_SPMI_PRIORITY_SPR_7_1_MASK	(0x7f << 1)
#define MAX77779_PMIC_VDROOP_INT_SPMI_PRIORITY_SPR_7_1_CLEAR	(~(0x7f << 1))
static inline const char *
max77779_pmic_vdroop_int_spmi_priority_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " VDROOP_INT_PR=%x",
		FIELD2VALUE(MAX77779_VDROOP_INT_PR, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_1=%x",
		FIELD2VALUE(MAX77779_SPR_7_1, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_vdroop_int_spmi_priority_vdroop_int_pr,0,0)
MAX77779_BFF(max77779_pmic_vdroop_int_spmi_priority_spr_7_1,7,1)

/*
 * MAX77779_PMIC_INT_STS,0x2A,0b00000000,0x0,Reset_Type:S
 * SPR_0,SPMIERR_INT,VDDPOR_INT,SysMsgI_INT,SYSUVLO_INT,SYSOVLO_INT,TSHDN_INT,
 * APCmdResI_INT
 */
#define MAX77779_PMIC_INT_STS	0x2a
#define MAX77779_PMIC_INT_STS_SPR_0_SHIFT	0
#define MAX77779_PMIC_INT_STS_SPR_0_MASK	(0x1 << 0)
#define MAX77779_PMIC_INT_STS_SPR_0_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_INT_STS_SPMIERR_INT_SHIFT	1
#define MAX77779_PMIC_INT_STS_SPMIERR_INT_MASK	(0x1 << 1)
#define MAX77779_PMIC_INT_STS_SPMIERR_INT_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_INT_STS_VDDPOR_INT_SHIFT	2
#define MAX77779_PMIC_INT_STS_VDDPOR_INT_MASK	(0x1 << 2)
#define MAX77779_PMIC_INT_STS_VDDPOR_INT_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_INT_STS_SysMsgI_INT_SHIFT	3
#define MAX77779_PMIC_INT_STS_SysMsgI_INT_MASK	(0x1 << 3)
#define MAX77779_PMIC_INT_STS_SysMsgI_INT_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_INT_STS_SYSUVLO_INT_SHIFT	4
#define MAX77779_PMIC_INT_STS_SYSUVLO_INT_MASK	(0x1 << 4)
#define MAX77779_PMIC_INT_STS_SYSUVLO_INT_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_INT_STS_SYSOVLO_INT_SHIFT	5
#define MAX77779_PMIC_INT_STS_SYSOVLO_INT_MASK	(0x1 << 5)
#define MAX77779_PMIC_INT_STS_SYSOVLO_INT_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_INT_STS_TSHDN_INT_SHIFT	6
#define MAX77779_PMIC_INT_STS_TSHDN_INT_MASK	(0x1 << 6)
#define MAX77779_PMIC_INT_STS_TSHDN_INT_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_INT_STS_APCmdResI_INT_SHIFT	7
#define MAX77779_PMIC_INT_STS_APCmdResI_INT_MASK	(0x1 << 7)
#define MAX77779_PMIC_INT_STS_APCmdResI_INT_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_int_sts_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SPR_0=%x",
		FIELD2VALUE(MAX77779_SPR_0, val));
	i += scnprintf(&buff[i], len - i, " SPMIERR_INT=%x",
		FIELD2VALUE(MAX77779_SPMIERR_INT, val));
	i += scnprintf(&buff[i], len - i, " VDDPOR_INT=%x",
		FIELD2VALUE(MAX77779_VDDPOR_INT, val));
	i += scnprintf(&buff[i], len - i, " SysMsgI_INT=%x",
		FIELD2VALUE(MAX77779_SysMsgI_INT, val));
	i += scnprintf(&buff[i], len - i, " SYSUVLO_INT=%x",
		FIELD2VALUE(MAX77779_SYSUVLO_INT, val));
	i += scnprintf(&buff[i], len - i, " SYSOVLO_INT=%x",
		FIELD2VALUE(MAX77779_SYSOVLO_INT, val));
	i += scnprintf(&buff[i], len - i, " TSHDN_INT=%x",
		FIELD2VALUE(MAX77779_TSHDN_INT, val));
	i += scnprintf(&buff[i], len - i, " APCmdResI_INT=%x",
		FIELD2VALUE(MAX77779_APCmdResI_INT, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_int_sts_spr_0,0,0)
MAX77779_BFF(max77779_pmic_int_sts_spmierr_int,1,1)
MAX77779_BFF(max77779_pmic_int_sts_vddpor_int,2,2)
MAX77779_BFF(max77779_pmic_int_sts_sysmsgi_int,3,3)
MAX77779_BFF(max77779_pmic_int_sts_sysuvlo_int,4,4)
MAX77779_BFF(max77779_pmic_int_sts_sysovlo_int,5,5)
MAX77779_BFF(max77779_pmic_int_sts_tshdn_int,6,6)
MAX77779_BFF(max77779_pmic_int_sts_apcmdresi_int,7,7)

/*
 * MAX77779_PMIC_INT_MASK,0x2B,0b11111111,0xff,OTP:SHADOW, Reset_Type:S
 * FSHIP_NOT_RD,SPMIERR_M,VDDPOR_M,SysMsg_M,SYSUVLO_INT_M,SYSOVLO_INT_M,
 * TSHDN_INT_M,APCmdRes_M
 */
#define MAX77779_PMIC_INT_MASK	0x2b
#define MAX77779_PMIC_INT_MASK_FSHIP_NOT_RD_SHIFT	0
#define MAX77779_PMIC_INT_MASK_FSHIP_NOT_RD_MASK	(0x1 << 0)
#define MAX77779_PMIC_INT_MASK_FSHIP_NOT_RD_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_INT_MASK_SPMIERR_M_SHIFT	1
#define MAX77779_PMIC_INT_MASK_SPMIERR_M_MASK	(0x1 << 1)
#define MAX77779_PMIC_INT_MASK_SPMIERR_M_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_INT_MASK_VDDPOR_M_SHIFT	2
#define MAX77779_PMIC_INT_MASK_VDDPOR_M_MASK	(0x1 << 2)
#define MAX77779_PMIC_INT_MASK_VDDPOR_M_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_INT_MASK_SysMsg_M_SHIFT	3
#define MAX77779_PMIC_INT_MASK_SysMsg_M_MASK	(0x1 << 3)
#define MAX77779_PMIC_INT_MASK_SysMsg_M_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_INT_MASK_SYSUVLO_INT_M_SHIFT	4
#define MAX77779_PMIC_INT_MASK_SYSUVLO_INT_M_MASK	(0x1 << 4)
#define MAX77779_PMIC_INT_MASK_SYSUVLO_INT_M_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_INT_MASK_SYSOVLO_INT_M_SHIFT	5
#define MAX77779_PMIC_INT_MASK_SYSOVLO_INT_M_MASK	(0x1 << 5)
#define MAX77779_PMIC_INT_MASK_SYSOVLO_INT_M_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_INT_MASK_TSHDN_INT_M_SHIFT	6
#define MAX77779_PMIC_INT_MASK_TSHDN_INT_M_MASK	(0x1 << 6)
#define MAX77779_PMIC_INT_MASK_TSHDN_INT_M_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_INT_MASK_APCmdRes_M_SHIFT	7
#define MAX77779_PMIC_INT_MASK_APCmdRes_M_MASK	(0x1 << 7)
#define MAX77779_PMIC_INT_MASK_APCmdRes_M_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_int_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " FSHIP_NOT_RD=%x",
		FIELD2VALUE(MAX77779_FSHIP_NOT_RD, val));
	i += scnprintf(&buff[i], len - i, " SPMIERR_M=%x",
		FIELD2VALUE(MAX77779_SPMIERR_M, val));
	i += scnprintf(&buff[i], len - i, " VDDPOR_M=%x",
		FIELD2VALUE(MAX77779_VDDPOR_M, val));
	i += scnprintf(&buff[i], len - i, " SysMsg_M=%x",
		FIELD2VALUE(MAX77779_SysMsg_M, val));
	i += scnprintf(&buff[i], len - i, " SYSUVLO_INT_M=%x",
		FIELD2VALUE(MAX77779_SYSUVLO_INT_M, val));
	i += scnprintf(&buff[i], len - i, " SYSOVLO_INT_M=%x",
		FIELD2VALUE(MAX77779_SYSOVLO_INT_M, val));
	i += scnprintf(&buff[i], len - i, " TSHDN_INT_M=%x",
		FIELD2VALUE(MAX77779_TSHDN_INT_M, val));
	i += scnprintf(&buff[i], len - i, " APCmdRes_M=%x",
		FIELD2VALUE(MAX77779_APCmdRes_M, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_int_mask_fship_not_rd,0,0)
MAX77779_BFF(max77779_pmic_int_mask_spmierr_m,1,1)
MAX77779_BFF(max77779_pmic_int_mask_vddpor_m,2,2)
MAX77779_BFF(max77779_pmic_int_mask_sysmsg_m,3,3)
MAX77779_BFF(max77779_pmic_int_mask_sysuvlo_int_m,4,4)
MAX77779_BFF(max77779_pmic_int_mask_sysovlo_int_m,5,5)
MAX77779_BFF(max77779_pmic_int_mask_tshdn_int_m,6,6)
MAX77779_BFF(max77779_pmic_int_mask_apcmdres_m,7,7)

/*
 * MAX77779_PMIC_EVENT_CNT_CFG,0x30,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * ENABLE,SAMPLE_RATE[1:2],SPR_7_3[3:5]
 */
#define MAX77779_PMIC_EVENT_CNT_CFG	0x30
#define MAX77779_PMIC_EVENT_CNT_CFG_ENABLE_SHIFT	0
#define MAX77779_PMIC_EVENT_CNT_CFG_ENABLE_MASK	(0x1 << 0)
#define MAX77779_PMIC_EVENT_CNT_CFG_ENABLE_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_EVENT_CNT_CFG_SAMPLE_RATE_SHIFT	1
#define MAX77779_PMIC_EVENT_CNT_CFG_SAMPLE_RATE_MASK	(0x3 << 1)
#define MAX77779_PMIC_EVENT_CNT_CFG_SAMPLE_RATE_CLEAR	(~(0x3 << 1))
#define MAX77779_PMIC_EVENT_CNT_CFG_SPR_7_3_SHIFT	3
#define MAX77779_PMIC_EVENT_CNT_CFG_SPR_7_3_MASK	(0x1f << 3)
#define MAX77779_PMIC_EVENT_CNT_CFG_SPR_7_3_CLEAR	(~(0x1f << 3))
static inline const char *
max77779_pmic_event_cnt_cfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " ENABLE=%x",
		FIELD2VALUE(MAX77779_ENABLE, val));
	i += scnprintf(&buff[i], len - i, " SAMPLE_RATE=%x",
		FIELD2VALUE(MAX77779_SAMPLE_RATE, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_3=%x",
		FIELD2VALUE(MAX77779_SPR_7_3, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_event_cnt_cfg_enable,0,0)
MAX77779_BFF(max77779_pmic_event_cnt_cfg_sample_rate,2,1)
MAX77779_BFF(max77779_pmic_event_cnt_cfg_spr_7_3,7,3)

/*
 * MAX77779_PMIC_EVENT_CNT_OILO0_THR,0x31,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_EVENT_CNT_OILO0_THR	0x31

/*
 * MAX77779_PMIC_EVENT_CNT_OILO1_THR,0x32,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_EVENT_CNT_OILO1_THR	0x32

/*
 * MAX77779_PMIC_EVENT_CNT_UVLO0_THR,0x33,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_EVENT_CNT_UVLO0_THR	0x33

/*
 * MAX77779_PMIC_EVENT_CNT_UVLO1_THR,0x34,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_EVENT_CNT_UVLO1_THR	0x34

/*
 * MAX77779_PMIC_EVENT_CNT_OILO0,0x35,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_EVENT_CNT_OILO0	0x35

/*
 * MAX77779_PMIC_EVENT_CNT_OILO1,0x36,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_EVENT_CNT_OILO1	0x36

/*
 * MAX77779_PMIC_EVENT_CNT_UVLO0,0x37,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_EVENT_CNT_UVLO0	0x37

/*
 * MAX77779_PMIC_EVENT_CNT_UVLO1,0x38,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_EVENT_CNT_UVLO1	0x38

/*
 * MAX77779_PMIC_I2C_CNFG,0x40,0b00000000,0x0,Reset_Type:O
 * HS_EXT_EN,SPR_3_1[1:3],PAIR[4:3],SPR_7
 */
#define MAX77779_PMIC_I2C_CNFG	0x40
#define MAX77779_PMIC_I2C_CNFG_HS_EXT_EN_SHIFT	0
#define MAX77779_PMIC_I2C_CNFG_HS_EXT_EN_MASK	(0x1 << 0)
#define MAX77779_PMIC_I2C_CNFG_HS_EXT_EN_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_I2C_CNFG_SPR_3_1_SHIFT	1
#define MAX77779_PMIC_I2C_CNFG_SPR_3_1_MASK	(0x7 << 1)
#define MAX77779_PMIC_I2C_CNFG_SPR_3_1_CLEAR	(~(0x7 << 1))
#define MAX77779_PMIC_I2C_CNFG_PAIR_SHIFT	4
#define MAX77779_PMIC_I2C_CNFG_PAIR_MASK	(0x7 << 4)
#define MAX77779_PMIC_I2C_CNFG_PAIR_CLEAR	(~(0x7 << 4))
#define MAX77779_PMIC_I2C_CNFG_SPR_7_SHIFT	7
#define MAX77779_PMIC_I2C_CNFG_SPR_7_MASK	(0x1 << 7)
#define MAX77779_PMIC_I2C_CNFG_SPR_7_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_i2c_cnfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " HS_EXT_EN=%x",
		FIELD2VALUE(MAX77779_HS_EXT_EN, val));
	i += scnprintf(&buff[i], len - i, " SPR_3_1=%x",
		FIELD2VALUE(MAX77779_SPR_3_1, val));
	i += scnprintf(&buff[i], len - i, " PAIR=%x",
		FIELD2VALUE(MAX77779_PAIR, val));
	i += scnprintf(&buff[i], len - i, " SPR_7=%x",
		FIELD2VALUE(MAX77779_SPR_7, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_i2c_cnfg_hs_ext_en,0,0)
MAX77779_BFF(max77779_pmic_i2c_cnfg_spr_3_1,3,1)
MAX77779_BFF(max77779_pmic_i2c_cnfg_pair,6,4)
MAX77779_BFF(max77779_pmic_i2c_cnfg_spr_7,7,7)

/*
 * MAX77779_PMIC_SPMI_CNFG,0x41,0b00001101,0xd,OTP:SHADOW, Reset_Type:SPMI
 * CLOAD[0:2],SDA_PULLDOWN,SCL_PULLDOWN,VIO_HIGH,INTERNAL_CLK_ON,SPMI_HOLD_CLK_ON
 */
#define MAX77779_PMIC_SPMI_CNFG	0x41
#define MAX77779_PMIC_SPMI_CNFG_CLOAD_SHIFT	0
#define MAX77779_PMIC_SPMI_CNFG_CLOAD_MASK	(0x3 << 0)
#define MAX77779_PMIC_SPMI_CNFG_CLOAD_CLEAR	(~(0x3 << 0))
#define MAX77779_PMIC_SPMI_CNFG_SDA_PULLDOWN_SHIFT	2
#define MAX77779_PMIC_SPMI_CNFG_SDA_PULLDOWN_MASK	(0x1 << 2)
#define MAX77779_PMIC_SPMI_CNFG_SDA_PULLDOWN_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_SPMI_CNFG_SCL_PULLDOWN_SHIFT	3
#define MAX77779_PMIC_SPMI_CNFG_SCL_PULLDOWN_MASK	(0x1 << 3)
#define MAX77779_PMIC_SPMI_CNFG_SCL_PULLDOWN_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_SPMI_CNFG_VIO_HIGH_SHIFT	4
#define MAX77779_PMIC_SPMI_CNFG_VIO_HIGH_MASK	(0x1 << 4)
#define MAX77779_PMIC_SPMI_CNFG_VIO_HIGH_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_SPMI_CNFG_INTERNAL_CLK_ON_SHIFT	6
#define MAX77779_PMIC_SPMI_CNFG_INTERNAL_CLK_ON_MASK	(0x1 << 6)
#define MAX77779_PMIC_SPMI_CNFG_INTERNAL_CLK_ON_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_SPMI_CNFG_SPMI_HOLD_CLK_ON_SHIFT	7
#define MAX77779_PMIC_SPMI_CNFG_SPMI_HOLD_CLK_ON_MASK	(0x1 << 7)
#define MAX77779_PMIC_SPMI_CNFG_SPMI_HOLD_CLK_ON_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_spmi_cnfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CLOAD=%x",
		FIELD2VALUE(MAX77779_CLOAD, val));
	i += scnprintf(&buff[i], len - i, " SDA_PULLDOWN=%x",
		FIELD2VALUE(MAX77779_SDA_PULLDOWN, val));
	i += scnprintf(&buff[i], len - i, " SCL_PULLDOWN=%x",
		FIELD2VALUE(MAX77779_SCL_PULLDOWN, val));
	i += scnprintf(&buff[i], len - i, " VIO_HIGH=%x",
		FIELD2VALUE(MAX77779_VIO_HIGH, val));
	i += scnprintf(&buff[i], len - i, " INTERNAL_CLK_ON=%x",
		FIELD2VALUE(MAX77779_INTERNAL_CLK_ON, val));
	i += scnprintf(&buff[i], len - i, " SPMI_HOLD_CLK_ON=%x",
		FIELD2VALUE(MAX77779_SPMI_HOLD_CLK_ON, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_spmi_cnfg_cload,1,0)
MAX77779_BFF(max77779_pmic_spmi_cnfg_sda_pulldown,2,2)
MAX77779_BFF(max77779_pmic_spmi_cnfg_scl_pulldown,3,3)
MAX77779_BFF(max77779_pmic_spmi_cnfg_vio_high,4,4)
MAX77779_BFF(max77779_pmic_spmi_cnfg_internal_clk_on,6,6)
MAX77779_BFF(max77779_pmic_spmi_cnfg_spmi_hold_clk_on,7,7)

/*
 * MAX77779_PMIC_SPMI_MID,0x42,0b00010110,0x16,Reset_Type:SPMI
 * SPMI_MID[0:2],SPMI_MW_ENABLE,SPR_3,SPMI_ARB_MODE,SPR_6_5[5:2],SPMI_MSG_REPEAT
 */
#define MAX77779_PMIC_SPMI_MID	0x42
#define MAX77779_PMIC_SPMI_MID_SPMI_MID_SHIFT	0
#define MAX77779_PMIC_SPMI_MID_SPMI_MID_MASK	(0x3 << 0)
#define MAX77779_PMIC_SPMI_MID_SPMI_MID_CLEAR	(~(0x3 << 0))
#define MAX77779_PMIC_SPMI_MID_SPMI_MW_ENABLE_SHIFT	2
#define MAX77779_PMIC_SPMI_MID_SPMI_MW_ENABLE_MASK	(0x1 << 2)
#define MAX77779_PMIC_SPMI_MID_SPMI_MW_ENABLE_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_SPMI_MID_SPR_3_SHIFT	3
#define MAX77779_PMIC_SPMI_MID_SPR_3_MASK	(0x1 << 3)
#define MAX77779_PMIC_SPMI_MID_SPR_3_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_SPMI_MID_SPMI_ARB_MODE_SHIFT	4
#define MAX77779_PMIC_SPMI_MID_SPMI_ARB_MODE_MASK	(0x1 << 4)
#define MAX77779_PMIC_SPMI_MID_SPMI_ARB_MODE_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_SPMI_MID_SPR_6_5_SHIFT	5
#define MAX77779_PMIC_SPMI_MID_SPR_6_5_MASK	(0x3 << 5)
#define MAX77779_PMIC_SPMI_MID_SPR_6_5_CLEAR	(~(0x3 << 5))
#define MAX77779_PMIC_SPMI_MID_SPMI_MSG_REPEAT_SHIFT	7
#define MAX77779_PMIC_SPMI_MID_SPMI_MSG_REPEAT_MASK	(0x1 << 7)
#define MAX77779_PMIC_SPMI_MID_SPMI_MSG_REPEAT_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_spmi_mid_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SPMI_MID=%x",
		FIELD2VALUE(MAX77779_SPMI_MID, val));
	i += scnprintf(&buff[i], len - i, " SPMI_MW_ENABLE=%x",
		FIELD2VALUE(MAX77779_SPMI_MW_ENABLE, val));
	i += scnprintf(&buff[i], len - i, " SPR_3=%x",
		FIELD2VALUE(MAX77779_SPR_3, val));
	i += scnprintf(&buff[i], len - i, " SPMI_ARB_MODE=%x",
		FIELD2VALUE(MAX77779_SPMI_ARB_MODE, val));
	i += scnprintf(&buff[i], len - i, " SPR_6_5=%x",
		FIELD2VALUE(MAX77779_SPR_6_5, val));
	i += scnprintf(&buff[i], len - i, " SPMI_MSG_REPEAT=%x",
		FIELD2VALUE(MAX77779_SPMI_MSG_REPEAT, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_spmi_mid_spmi_mid,1,0)
MAX77779_BFF(max77779_pmic_spmi_mid_spmi_mw_enable,2,2)
MAX77779_BFF(max77779_pmic_spmi_mid_spr_3,3,3)
MAX77779_BFF(max77779_pmic_spmi_mid_spmi_arb_mode,4,4)
MAX77779_BFF(max77779_pmic_spmi_mid_spr_6_5,6,5)
MAX77779_BFF(max77779_pmic_spmi_mid_spmi_msg_repeat,7,7)

/*
 * MAX77779_PMIC_SPMI_MADDR,0x43,0b00000000,0x0,Reset_Type:SPMI
 */
#define MAX77779_PMIC_SPMI_MADDR	0x43

/*
 * MAX77779_PMIC_SPMI_STS,0x44,0b00000000,0x0,Reset_Type:O
 * SPMI_ARB_ERR,RSVD_6_1[1:6]
 */
#define MAX77779_PMIC_SPMI_STS	0x44
#define MAX77779_PMIC_SPMI_STS_SPMI_ARB_ERR_SHIFT	0
#define MAX77779_PMIC_SPMI_STS_SPMI_ARB_ERR_MASK	(0x1 << 0)
#define MAX77779_PMIC_SPMI_STS_SPMI_ARB_ERR_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_SPMI_STS_RSVD_6_1_SHIFT	1
#define MAX77779_PMIC_SPMI_STS_RSVD_6_1_MASK	(0x3f << 1)
#define MAX77779_PMIC_SPMI_STS_RSVD_6_1_CLEAR	(~(0x3f << 1))
static inline const char *
max77779_pmic_spmi_sts_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SPMI_ARB_ERR=%x",
		FIELD2VALUE(MAX77779_SPMI_ARB_ERR, val));
	i += scnprintf(&buff[i], len - i, " RSVD_6_1=%x",
		FIELD2VALUE(MAX77779_RSVD_6_1, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_spmi_sts_spmi_arb_err,0,0)
MAX77779_BFF(max77779_pmic_spmi_sts_rsvd_6_1,6,1)

/*
 * MAX77779_PMIC_SWRESET,0x50,0b00000000,0x0,Reset_Type:S, (Exception: SWR_RST bits are not register type which can retain data)
 * SWR_RST[0:6],IC_RST_MASK,VIO_OK_MASK
 */
#define MAX77779_PMIC_SWRESET	0x50
#define MAX77779_PMIC_SWRESET_SWR_RST_SHIFT	0
#define MAX77779_PMIC_SWRESET_SWR_RST_MASK	(0x3f << 0)
#define MAX77779_PMIC_SWRESET_SWR_RST_CLEAR	(~(0x3f << 0))
#define MAX77779_PMIC_SWRESET_IC_RST_MASK_SHIFT	6
#define MAX77779_PMIC_SWRESET_IC_RST_MASK_MASK	(0x1 << 6)
#define MAX77779_PMIC_SWRESET_IC_RST_MASK_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_SWRESET_VIO_OK_MASK_SHIFT	7
#define MAX77779_PMIC_SWRESET_VIO_OK_MASK_MASK	(0x1 << 7)
#define MAX77779_PMIC_SWRESET_VIO_OK_MASK_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_swreset_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SWR_RST=%x",
		FIELD2VALUE(MAX77779_SWR_RST, val));
	i += scnprintf(&buff[i], len - i, " IC_RST_MASK=%x",
		FIELD2VALUE(MAX77779_IC_RST_MASK, val));
	i += scnprintf(&buff[i], len - i, " VIO_OK_MASK=%x",
		FIELD2VALUE(MAX77779_VIO_OK_MASK, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_swreset_swr_rst,5,0)
MAX77779_BFF(max77779_pmic_swreset_ic_rst_mask,6,6)
MAX77779_BFF(max77779_pmic_swreset_vio_ok_mask,7,7)

/*
 * MAX77779_PMIC_CONTROL_FG,0x51,0b00010000,0x10,Reset_Type:F
 * TSHDN_DIS
 */
#define MAX77779_PMIC_CONTROL_FG	0x51
#define MAX77779_PMIC_CONTROL_FG_TSHDN_DIS_SHIFT	4
#define MAX77779_PMIC_CONTROL_FG_TSHDN_DIS_MASK	(0x1 << 4)
#define MAX77779_PMIC_CONTROL_FG_TSHDN_DIS_CLEAR	(~(0x1 << 4))
static inline const char *
max77779_pmic_control_fg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TSHDN_DIS=%x",
		FIELD2VALUE(MAX77779_TSHDN_DIS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_control_fg_tshdn_dis,4,4)

/*******************************************************
 * Section: RISCV_FUNC 0x60 8
 */

/*
 * MAX77779_PMIC_RISCV_DEVICE_ID,0x00,0b01111001,0x79,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_DEVICE_ID	0x60

/*
 * MAX77779_PMIC_RISCV_DEVICE_REV,0x01,0b00000001,0x1,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_DEVICE_REV	0x61

/*
 * MAX77779_PMIC_RISCV_FW_REV,0x02,0b00000001,0x1,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_FW_REV	0x62

/*
 * MAX77779_PMIC_RISCV_FW_SUB_REV,0x03,0b00010001,0x11,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_FW_SUB_REV	0x63

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT1,0x21,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT1	0x81

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT2,0x22,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT2	0x82

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT3,0x23,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT3	0x83

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT4,0x24,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT4	0x84

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT5,0x25,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT5	0x85

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT6,0x26,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT6	0x86

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT7,0x27,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT7	0x87

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT8,0x28,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT8	0x88

/*
 * MAX77779_PMIC_RISCV_AP_DATAOUT_OPCODE,0x29,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAOUT_OPCODE	0x89

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN0,0x51,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN0	0xb1

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN1,0x52,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN1	0xb2

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN2,0x53,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN2	0xb3

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN3,0x54,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN3	0xb4

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN4,0x55,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN4	0xb5

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN5,0x56,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN5	0xb6

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN6,0x57,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN6	0xb7

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN7,0x58,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN7	0xb8

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN8,0x59,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN8	0xb9

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN9,0x5A,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN9	0xba

/*
 * MAX77779_PMIC_RISCV_AP_DATAIN10,0x5B,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_AP_DATAIN10	0xbb

/*
 * MAX77779_PMIC_RISCV_SysMsg,0x5C,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_SysMsg	0xbc

/*
 * MAX77779_PMIC_RISCV_COMMAND_HW,0x5F,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_PMIC_RISCV_COMMAND_HW	0xbf

/*******************************************************
 * Section: GPIO 0xE8 8
 */

/*
 * MAX77779_PMIC_GPIO_SGPIO_INT,0x00,0b00000000,0x0,Reset_Type:O
 * SGPIO0_STS,SGPIO1_STS,SGPIO2_STS,SGPIO3_STS,SGPIO4_STS,SGPIO5_STS,
 * SGPIO6_STS,SGPIO7_STS
 */
#define MAX77779_PMIC_GPIO_SGPIO_INT	0xe8
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO0_STS_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO0_STS_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO0_STS_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO1_STS_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO1_STS_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO1_STS_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO2_STS_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO2_STS_MASK	(0x1 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO2_STS_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO3_STS_SHIFT	3
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO3_STS_MASK	(0x1 << 3)
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO3_STS_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO4_STS_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO4_STS_MASK	(0x1 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO4_STS_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO5_STS_SHIFT	5
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO5_STS_MASK	(0x1 << 5)
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO5_STS_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO6_STS_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO6_STS_MASK	(0x1 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO6_STS_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO7_STS_SHIFT	7
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO7_STS_MASK	(0x1 << 7)
#define MAX77779_PMIC_GPIO_SGPIO_INT_SGPIO7_STS_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_gpio_sgpio_int_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SGPIO0_STS=%x",
		FIELD2VALUE(MAX77779_SGPIO0_STS, val));
	i += scnprintf(&buff[i], len - i, " SGPIO1_STS=%x",
		FIELD2VALUE(MAX77779_SGPIO1_STS, val));
	i += scnprintf(&buff[i], len - i, " SGPIO2_STS=%x",
		FIELD2VALUE(MAX77779_SGPIO2_STS, val));
	i += scnprintf(&buff[i], len - i, " SGPIO3_STS=%x",
		FIELD2VALUE(MAX77779_SGPIO3_STS, val));
	i += scnprintf(&buff[i], len - i, " SGPIO4_STS=%x",
		FIELD2VALUE(MAX77779_SGPIO4_STS, val));
	i += scnprintf(&buff[i], len - i, " SGPIO5_STS=%x",
		FIELD2VALUE(MAX77779_SGPIO5_STS, val));
	i += scnprintf(&buff[i], len - i, " SGPIO6_STS=%x",
		FIELD2VALUE(MAX77779_SGPIO6_STS, val));
	i += scnprintf(&buff[i], len - i, " SGPIO7_STS=%x",
		FIELD2VALUE(MAX77779_SGPIO7_STS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_int_sgpio0_sts,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_int_sgpio1_sts,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_int_sgpio2_sts,2,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_int_sgpio3_sts,3,3)
MAX77779_BFF(max77779_pmic_gpio_sgpio_int_sgpio4_sts,4,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_int_sgpio5_sts,5,5)
MAX77779_BFF(max77779_pmic_gpio_sgpio_int_sgpio6_sts,6,6)
MAX77779_BFF(max77779_pmic_gpio_sgpio_int_sgpio7_sts,7,7)

/*
 * MAX77779_PMIC_GPIO_SGPIO_PU,0x01,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * PU0,PU1,PU2,PU3,PU4,PU5,PU6,PU7
 */
#define MAX77779_PMIC_GPIO_SGPIO_PU	0xe9
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU0_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU0_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU0_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU1_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU2_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU2_MASK	(0x1 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU2_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU3_SHIFT	3
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU3_MASK	(0x1 << 3)
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU3_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU4_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU4_MASK	(0x1 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU4_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU5_SHIFT	5
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU5_MASK	(0x1 << 5)
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU5_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU6_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU6_MASK	(0x1 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU6_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU7_SHIFT	7
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU7_MASK	(0x1 << 7)
#define MAX77779_PMIC_GPIO_SGPIO_PU_PU7_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_gpio_sgpio_pu_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " PU0=%x",
		FIELD2VALUE(MAX77779_PU0, val));
	i += scnprintf(&buff[i], len - i, " PU1=%x",
		FIELD2VALUE(MAX77779_PU1, val));
	i += scnprintf(&buff[i], len - i, " PU2=%x",
		FIELD2VALUE(MAX77779_PU2, val));
	i += scnprintf(&buff[i], len - i, " PU3=%x",
		FIELD2VALUE(MAX77779_PU3, val));
	i += scnprintf(&buff[i], len - i, " PU4=%x",
		FIELD2VALUE(MAX77779_PU4, val));
	i += scnprintf(&buff[i], len - i, " PU5=%x",
		FIELD2VALUE(MAX77779_PU5, val));
	i += scnprintf(&buff[i], len - i, " PU6=%x",
		FIELD2VALUE(MAX77779_PU6, val));
	i += scnprintf(&buff[i], len - i, " PU7=%x",
		FIELD2VALUE(MAX77779_PU7, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_pu_pu0,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pu_pu1,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pu_pu2,2,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pu_pu3,3,3)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pu_pu4,4,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pu_pu5,5,5)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pu_pu6,6,6)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pu_pu7,7,7)

/*
 * MAX77779_PMIC_GPIO_SGPIO_PD,0x02,0b01011110,0x5e,OTP:SHADOW, Reset_Type:O
 * PD0,PD1,PD2,PD3,PD4,PD5,PD6,PD7
 */
#define MAX77779_PMIC_GPIO_SGPIO_PD	0xea
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD0_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD0_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD0_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD1_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD2_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD2_MASK	(0x1 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD2_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD3_SHIFT	3
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD3_MASK	(0x1 << 3)
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD3_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD4_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD4_MASK	(0x1 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD4_CLEAR	(~(0x1 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD5_SHIFT	5
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD5_MASK	(0x1 << 5)
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD5_CLEAR	(~(0x1 << 5))
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD6_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD6_MASK	(0x1 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD6_CLEAR	(~(0x1 << 6))
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD7_SHIFT	7
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD7_MASK	(0x1 << 7)
#define MAX77779_PMIC_GPIO_SGPIO_PD_PD7_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_pmic_gpio_sgpio_pd_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " PD0=%x",
		FIELD2VALUE(MAX77779_PD0, val));
	i += scnprintf(&buff[i], len - i, " PD1=%x",
		FIELD2VALUE(MAX77779_PD1, val));
	i += scnprintf(&buff[i], len - i, " PD2=%x",
		FIELD2VALUE(MAX77779_PD2, val));
	i += scnprintf(&buff[i], len - i, " PD3=%x",
		FIELD2VALUE(MAX77779_PD3, val));
	i += scnprintf(&buff[i], len - i, " PD4=%x",
		FIELD2VALUE(MAX77779_PD4, val));
	i += scnprintf(&buff[i], len - i, " PD5=%x",
		FIELD2VALUE(MAX77779_PD5, val));
	i += scnprintf(&buff[i], len - i, " PD6=%x",
		FIELD2VALUE(MAX77779_PD6, val));
	i += scnprintf(&buff[i], len - i, " PD7=%x",
		FIELD2VALUE(MAX77779_PD7, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_pd_pd0,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pd_pd1,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pd_pd2,2,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pd_pd3,3,3)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pd_pd4,4,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pd_pd5,5,5)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pd_pd6,6,6)
MAX77779_BFF(max77779_pmic_gpio_sgpio_pd_pd7,7,7)

/*
 * MAX77779_PMIC_GPIO_AGPIO_PU,0x03,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * PU0,PU1,PU2,PU3,SPR_7_4[4:4]
 */
#define MAX77779_PMIC_GPIO_AGPIO_PU	0xeb
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU0_SHIFT	0
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU0_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU0_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU1_SHIFT	1
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU2_SHIFT	2
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU2_MASK	(0x1 << 2)
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU2_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU3_SHIFT	3
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU3_MASK	(0x1 << 3)
#define MAX77779_PMIC_GPIO_AGPIO_PU_PU3_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_GPIO_AGPIO_PU_SPR_7_4_SHIFT	4
#define MAX77779_PMIC_GPIO_AGPIO_PU_SPR_7_4_MASK	(0xf << 4)
#define MAX77779_PMIC_GPIO_AGPIO_PU_SPR_7_4_CLEAR	(~(0xf << 4))
static inline const char *
max77779_pmic_gpio_agpio_pu_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " PU0=%x",
		FIELD2VALUE(MAX77779_PU0, val));
	i += scnprintf(&buff[i], len - i, " PU1=%x",
		FIELD2VALUE(MAX77779_PU1, val));
	i += scnprintf(&buff[i], len - i, " PU2=%x",
		FIELD2VALUE(MAX77779_PU2, val));
	i += scnprintf(&buff[i], len - i, " PU3=%x",
		FIELD2VALUE(MAX77779_PU3, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_4=%x",
		FIELD2VALUE(MAX77779_SPR_7_4, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_agpio_pu_pu0,0,0)
MAX77779_BFF(max77779_pmic_gpio_agpio_pu_pu1,1,1)
MAX77779_BFF(max77779_pmic_gpio_agpio_pu_pu2,2,2)
MAX77779_BFF(max77779_pmic_gpio_agpio_pu_pu3,3,3)
MAX77779_BFF(max77779_pmic_gpio_agpio_pu_spr_7_4,7,4)

/*
 * MAX77779_PMIC_GPIO_AGPIO_PD,0x04,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * PD0,PD1,PD2,PD3,SPR_7_4[4:4]
 */
#define MAX77779_PMIC_GPIO_AGPIO_PD	0xec
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD0_SHIFT	0
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD0_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD0_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD1_SHIFT	1
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD2_SHIFT	2
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD2_MASK	(0x1 << 2)
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD2_CLEAR	(~(0x1 << 2))
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD3_SHIFT	3
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD3_MASK	(0x1 << 3)
#define MAX77779_PMIC_GPIO_AGPIO_PD_PD3_CLEAR	(~(0x1 << 3))
#define MAX77779_PMIC_GPIO_AGPIO_PD_SPR_7_4_SHIFT	4
#define MAX77779_PMIC_GPIO_AGPIO_PD_SPR_7_4_MASK	(0xf << 4)
#define MAX77779_PMIC_GPIO_AGPIO_PD_SPR_7_4_CLEAR	(~(0xf << 4))
static inline const char *
max77779_pmic_gpio_agpio_pd_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " PD0=%x",
		FIELD2VALUE(MAX77779_PD0, val));
	i += scnprintf(&buff[i], len - i, " PD1=%x",
		FIELD2VALUE(MAX77779_PD1, val));
	i += scnprintf(&buff[i], len - i, " PD2=%x",
		FIELD2VALUE(MAX77779_PD2, val));
	i += scnprintf(&buff[i], len - i, " PD3=%x",
		FIELD2VALUE(MAX77779_PD3, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_4=%x",
		FIELD2VALUE(MAX77779_SPR_7_4, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_agpio_pd_pd0,0,0)
MAX77779_BFF(max77779_pmic_gpio_agpio_pd_pd1,1,1)
MAX77779_BFF(max77779_pmic_gpio_agpio_pd_pd2,2,2)
MAX77779_BFF(max77779_pmic_gpio_agpio_pd_pd3,3,3)
MAX77779_BFF(max77779_pmic_gpio_agpio_pd_spr_7_4,7,4)

/*
 * MAX77779_PMIC_GPIO_SGPIO_CNFG0,0x05,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * DATA,VGPI_EN,MODE[2:2],DBNC_SEL[4:2],IRQ_SEL[6:2]
 */
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0	0xed
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_VGPI_EN_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_VGPI_EN_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_VGPI_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_IRQ_SEL_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_IRQ_SEL_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG0_IRQ_SEL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_sgpio_cnfg0_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " VGPI_EN=%x",
		FIELD2VALUE(MAX77779_VGPI_EN, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " IRQ_SEL=%x",
		FIELD2VALUE(MAX77779_IRQ_SEL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg0_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg0_vgpi_en,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg0_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg0_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg0_irq_sel,7,6)

/*
 * MAX77779_PMIC_GPIO_SGPIO_CNFG1,0x06,0b00000100,0x4,OTP:SHADOW, Reset_Type:O
 * DATA,VGPI_EN,MODE[2:2],DBNC_SEL[4:2],IRQ_SEL[6:2]
 */
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1	0xee
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_VGPI_EN_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_VGPI_EN_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_VGPI_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_IRQ_SEL_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_IRQ_SEL_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG1_IRQ_SEL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_sgpio_cnfg1_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " VGPI_EN=%x",
		FIELD2VALUE(MAX77779_VGPI_EN, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " IRQ_SEL=%x",
		FIELD2VALUE(MAX77779_IRQ_SEL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg1_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg1_vgpi_en,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg1_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg1_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg1_irq_sel,7,6)

/*
 * MAX77779_PMIC_GPIO_SGPIO_CNFG2,0x07,0b00000100,0x4,OTP:SHADOW, Reset_Type:O
 * DATA,VGPI_EN,MODE[2:2],DBNC_SEL[4:2],IRQ_SEL[6:2]
 */
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2	0xef
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_VGPI_EN_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_VGPI_EN_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_VGPI_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_IRQ_SEL_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_IRQ_SEL_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG2_IRQ_SEL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_sgpio_cnfg2_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " VGPI_EN=%x",
		FIELD2VALUE(MAX77779_VGPI_EN, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " IRQ_SEL=%x",
		FIELD2VALUE(MAX77779_IRQ_SEL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg2_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg2_vgpi_en,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg2_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg2_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg2_irq_sel,7,6)

/*
 * MAX77779_PMIC_GPIO_SGPIO_CNFG3,0x08,0b00000100,0x4,OTP:SHADOW, Reset_Type:O
 * DATA,VGPI_EN,MODE[2:2],DBNC_SEL[4:2],IRQ_SEL[6:2]
 */
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3	0xf0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_VGPI_EN_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_VGPI_EN_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_VGPI_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_IRQ_SEL_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_IRQ_SEL_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG3_IRQ_SEL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_sgpio_cnfg3_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " VGPI_EN=%x",
		FIELD2VALUE(MAX77779_VGPI_EN, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " IRQ_SEL=%x",
		FIELD2VALUE(MAX77779_IRQ_SEL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg3_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg3_vgpi_en,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg3_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg3_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg3_irq_sel,7,6)

/*
 * MAX77779_PMIC_GPIO_SGPIO_CNFG4,0x09,0b00000100,0x4,OTP:SHADOW, Reset_Type:O
 * DATA,VGPI_EN,MODE[2:2],DBNC_SEL[4:2],IRQ_SEL[6:2]
 */
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4	0xf1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_VGPI_EN_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_VGPI_EN_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_VGPI_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_IRQ_SEL_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_IRQ_SEL_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG4_IRQ_SEL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_sgpio_cnfg4_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " VGPI_EN=%x",
		FIELD2VALUE(MAX77779_VGPI_EN, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " IRQ_SEL=%x",
		FIELD2VALUE(MAX77779_IRQ_SEL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg4_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg4_vgpi_en,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg4_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg4_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg4_irq_sel,7,6)

/*
 * MAX77779_PMIC_GPIO_SGPIO_CNFG5,0x0A,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * DATA,VGPI_EN,MODE[2:2],DBNC_SEL[4:2],IRQ_SEL[6:2]
 */
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5	0xf2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_VGPI_EN_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_VGPI_EN_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_VGPI_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_IRQ_SEL_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_IRQ_SEL_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG5_IRQ_SEL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_sgpio_cnfg5_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " VGPI_EN=%x",
		FIELD2VALUE(MAX77779_VGPI_EN, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " IRQ_SEL=%x",
		FIELD2VALUE(MAX77779_IRQ_SEL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg5_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg5_vgpi_en,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg5_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg5_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg5_irq_sel,7,6)

/*
 * MAX77779_PMIC_GPIO_SGPIO_CNFG6,0x0B,0b00000100,0x4,OTP:SHADOW, Reset_Type:O
 * DATA,VGPI_EN,MODE[2:2],DBNC_SEL[4:2],IRQ_SEL[6:2]
 */
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6	0xf3
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_VGPI_EN_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_VGPI_EN_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_VGPI_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_IRQ_SEL_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_IRQ_SEL_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG6_IRQ_SEL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_sgpio_cnfg6_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " VGPI_EN=%x",
		FIELD2VALUE(MAX77779_VGPI_EN, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " IRQ_SEL=%x",
		FIELD2VALUE(MAX77779_IRQ_SEL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg6_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg6_vgpi_en,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg6_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg6_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg6_irq_sel,7,6)

/*
 * MAX77779_PMIC_GPIO_SGPIO_CNFG7,0x0C,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * DATA,VGPI_EN,MODE[2:2],DBNC_SEL[4:2],IRQ_SEL[6:2]
 */
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7	0xf4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_VGPI_EN_SHIFT	1
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_VGPI_EN_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_VGPI_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_IRQ_SEL_SHIFT	6
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_IRQ_SEL_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_SGPIO_CNFG7_IRQ_SEL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_sgpio_cnfg7_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " VGPI_EN=%x",
		FIELD2VALUE(MAX77779_VGPI_EN, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " IRQ_SEL=%x",
		FIELD2VALUE(MAX77779_IRQ_SEL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg7_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg7_vgpi_en,1,1)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg7_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg7_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_sgpio_cnfg7_irq_sel,7,6)

/*
 * MAX77779_PMIC_GPIO_AGPIO_CNFG0,0x0D,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * DATA,SPR_1,MODE[2:2],DBNC_SEL[4:2],RSVD_7_6[6:2]
 */
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0	0xf5
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_SPR_1_SHIFT	1
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_SPR_1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_SPR_1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_RSVD_7_6_SHIFT	6
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_RSVD_7_6_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG0_RSVD_7_6_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_agpio_cnfg0_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " SPR_1=%x",
		FIELD2VALUE(MAX77779_SPR_1, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " RSVD_7_6=%x",
		FIELD2VALUE(MAX77779_RSVD_7_6, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg0_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg0_spr_1,1,1)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg0_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg0_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg0_rsvd_7_6,7,6)

/*
 * MAX77779_PMIC_GPIO_AGPIO_CNFG1,0x0E,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * DATA,SPR_1,MODE[2:2],DBNC_SEL[4:2],RSVD_7_6[6:2]
 */
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1	0xf6
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_SPR_1_SHIFT	1
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_SPR_1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_SPR_1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_RSVD_7_6_SHIFT	6
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_RSVD_7_6_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG1_RSVD_7_6_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_agpio_cnfg1_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " SPR_1=%x",
		FIELD2VALUE(MAX77779_SPR_1, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " RSVD_7_6=%x",
		FIELD2VALUE(MAX77779_RSVD_7_6, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg1_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg1_spr_1,1,1)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg1_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg1_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg1_rsvd_7_6,7,6)

/*
 * MAX77779_PMIC_GPIO_AGPIO_CNFG2,0x0F,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * DATA,SPR_1,MODE[2:2],DBNC_SEL[4:2],RSVD_7_6[6:2]
 */
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2	0xf7
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_SPR_1_SHIFT	1
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_SPR_1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_SPR_1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_RSVD_7_6_SHIFT	6
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_RSVD_7_6_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG2_RSVD_7_6_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_agpio_cnfg2_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " SPR_1=%x",
		FIELD2VALUE(MAX77779_SPR_1, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " RSVD_7_6=%x",
		FIELD2VALUE(MAX77779_RSVD_7_6, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg2_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg2_spr_1,1,1)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg2_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg2_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg2_rsvd_7_6,7,6)

/*
 * MAX77779_PMIC_GPIO_AGPIO_CNFG3,0x10,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * DATA,SPR_1,MODE[2:2],DBNC_SEL[4:2],RSVD_7_6[6:2]
 */
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3	0xf8
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_SPR_1_SHIFT	1
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_SPR_1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_SPR_1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_DBNC_SEL_SHIFT	4
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_DBNC_SEL_MASK	(0x3 << 4)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_DBNC_SEL_CLEAR	(~(0x3 << 4))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_RSVD_7_6_SHIFT	6
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_RSVD_7_6_MASK	(0x3 << 6)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG3_RSVD_7_6_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_pmic_gpio_agpio_cnfg3_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " SPR_1=%x",
		FIELD2VALUE(MAX77779_SPR_1, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " DBNC_SEL=%x",
		FIELD2VALUE(MAX77779_DBNC_SEL, val));
	i += scnprintf(&buff[i], len - i, " RSVD_7_6=%x",
		FIELD2VALUE(MAX77779_RSVD_7_6, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg3_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg3_spr_1,1,1)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg3_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg3_dbnc_sel,5,4)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg3_rsvd_7_6,7,6)

/*
 * MAX77779_PMIC_GPIO_AGPIO_CNFG4,0x11,0b00001000,0x8,OTP:SHADOW, Reset_Type:O
 * DATA,SPR_1,MODE[2:2],RSVD_7_4[4:4]
 */
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4	0xf9
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_SPR_1_SHIFT	1
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_SPR_1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_SPR_1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_RSVD_7_4_SHIFT	4
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_RSVD_7_4_MASK	(0xf << 4)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG4_RSVD_7_4_CLEAR	(~(0xf << 4))
static inline const char *
max77779_pmic_gpio_agpio_cnfg4_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " SPR_1=%x",
		FIELD2VALUE(MAX77779_SPR_1, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " RSVD_7_4=%x",
		FIELD2VALUE(MAX77779_RSVD_7_4, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg4_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg4_spr_1,1,1)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg4_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg4_rsvd_7_4,7,4)

/*
 * MAX77779_PMIC_GPIO_AGPIO_CNFG5,0x12,0b00001000,0x8,OTP:SHADOW, Reset_Type:O
 * DATA,SPR_1,MODE[2:2],RSVD_7_4[4:4]
 */
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5	0xfa
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_DATA_SHIFT	0
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_DATA_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_DATA_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_SPR_1_SHIFT	1
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_SPR_1_MASK	(0x1 << 1)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_SPR_1_CLEAR	(~(0x1 << 1))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_MODE_SHIFT	2
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_MODE_MASK	(0x3 << 2)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_MODE_CLEAR	(~(0x3 << 2))
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_RSVD_7_4_SHIFT	4
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_RSVD_7_4_MASK	(0xf << 4)
#define MAX77779_PMIC_GPIO_AGPIO_CNFG5_RSVD_7_4_CLEAR	(~(0xf << 4))
static inline const char *
max77779_pmic_gpio_agpio_cnfg5_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DATA=%x",
		FIELD2VALUE(MAX77779_DATA, val));
	i += scnprintf(&buff[i], len - i, " SPR_1=%x",
		FIELD2VALUE(MAX77779_SPR_1, val));
	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " RSVD_7_4=%x",
		FIELD2VALUE(MAX77779_RSVD_7_4, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg5_data,0,0)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg5_spr_1,1,1)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg5_mode,3,2)
MAX77779_BFF(max77779_pmic_gpio_agpio_cnfg5_rsvd_7_4,7,4)

/*
 * MAX77779_PMIC_GPIO_VGPI_CNFG,0x13,0b00000001,0x1,Reset_Type:O
 * VGPI_PR,SPR_7_1[1:7]
 */
#define MAX77779_PMIC_GPIO_VGPI_CNFG	0xfb
#define MAX77779_PMIC_GPIO_VGPI_CNFG_VGPI_PR_SHIFT	0
#define MAX77779_PMIC_GPIO_VGPI_CNFG_VGPI_PR_MASK	(0x1 << 0)
#define MAX77779_PMIC_GPIO_VGPI_CNFG_VGPI_PR_CLEAR	(~(0x1 << 0))
#define MAX77779_PMIC_GPIO_VGPI_CNFG_SPR_7_1_SHIFT	1
#define MAX77779_PMIC_GPIO_VGPI_CNFG_SPR_7_1_MASK	(0x7f << 1)
#define MAX77779_PMIC_GPIO_VGPI_CNFG_SPR_7_1_CLEAR	(~(0x7f << 1))
static inline const char *
max77779_pmic_gpio_vgpi_cnfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " VGPI_PR=%x",
		FIELD2VALUE(MAX77779_VGPI_PR, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_1=%x",
		FIELD2VALUE(MAX77779_SPR_7_1, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_pmic_gpio_vgpi_cnfg_vgpi_pr,0,0)
MAX77779_BFF(max77779_pmic_gpio_vgpi_cnfg_spr_7_1,7,1)

/*******************************************************
 * Section: JEITA_FUNC 0x00 8
 */

/*
 * MAX77779_CHG_CHGIN_I_ADC_L,0x00,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_CHGIN_I_ADC_L	0x00

/*
 * MAX77779_CHG_CHGIN_I_ADC_H,0x01,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_CHGIN_I_ADC_H	0x01

/*
 * MAX77779_CHG_CHGIN_V_ADC_L,0x02,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_CHGIN_V_ADC_L	0x02

/*
 * MAX77779_CHG_CHGIN_V_ADC_H,0x03,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_CHGIN_V_ADC_H	0x03

/*
 * MAX77779_CHG_WCIN_I_ADC_L,0x04,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_WCIN_I_ADC_L	0x04

/*
 * MAX77779_CHG_WCIN_I_ADC_H,0x05,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_WCIN_I_ADC_H	0x05

/*
 * MAX77779_CHG_WCIN_V_ADC_L,0x06,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_WCIN_V_ADC_L	0x06

/*
 * MAX77779_CHG_WCIN_V_ADC_H,0x07,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_WCIN_V_ADC_H	0x07

/*
 * MAX77779_CHG_THM1_TEMP_L,0x08,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_THM1_TEMP_L	0x08

/*
 * MAX77779_CHG_THM1_TEMP_H,0x09,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_THM1_TEMP_H	0x09

/*
 * MAX77779_CHG_THM2_TEMP_L,0x0A,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_THM2_TEMP_L	0x0a

/*
 * MAX77779_CHG_THM2_TEMP_H,0x0B,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_THM2_TEMP_H	0x0b

/*
 * MAX77779_CHG_THM3_TEMP_L,0x0C,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_THM3_TEMP_L	0x0c

/*
 * MAX77779_CHG_THM3_TEMP_H,0x0D,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_THM3_TEMP_H	0x0d

/*
 * MAX77779_CHG_BATT_ID1_ADC_L,0x0E,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_BATT_ID1_ADC_L	0x0e

/*
 * MAX77779_CHG_BATT_ID1_ADC_H,0x0F,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_BATT_ID1_ADC_H	0x0f

/*
 * MAX77779_CHG_BATT_ID2_ADC_L,0x10,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_BATT_ID2_ADC_L	0x10

/*
 * MAX77779_CHG_BATT_ID2_ADC_H,0x11,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_BATT_ID2_ADC_H	0x11

/*
 * MAX77779_CHG_JEITA_CTRL,0x12,0b00000000,0x0,Reset_Type:O
 * IN_ADC_FORCE,THM2_TEMP_FORCE,THM3_CHECK_EN,BATT_ID1_DO,BATT_ID2_DO
 */
#define MAX77779_CHG_JEITA_CTRL	0x12
#define MAX77779_CHG_JEITA_CTRL_IN_ADC_FORCE_SHIFT	0
#define MAX77779_CHG_JEITA_CTRL_IN_ADC_FORCE_MASK	(0x1 << 0)
#define MAX77779_CHG_JEITA_CTRL_IN_ADC_FORCE_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_JEITA_CTRL_THM2_TEMP_FORCE_SHIFT	1
#define MAX77779_CHG_JEITA_CTRL_THM2_TEMP_FORCE_MASK	(0x1 << 1)
#define MAX77779_CHG_JEITA_CTRL_THM2_TEMP_FORCE_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_JEITA_CTRL_THM3_CHECK_EN_SHIFT	2
#define MAX77779_CHG_JEITA_CTRL_THM3_CHECK_EN_MASK	(0x1 << 2)
#define MAX77779_CHG_JEITA_CTRL_THM3_CHECK_EN_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_JEITA_CTRL_BATT_ID1_DO_SHIFT	3
#define MAX77779_CHG_JEITA_CTRL_BATT_ID1_DO_MASK	(0x1 << 3)
#define MAX77779_CHG_JEITA_CTRL_BATT_ID1_DO_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_JEITA_CTRL_BATT_ID2_DO_SHIFT	4
#define MAX77779_CHG_JEITA_CTRL_BATT_ID2_DO_MASK	(0x1 << 4)
#define MAX77779_CHG_JEITA_CTRL_BATT_ID2_DO_CLEAR	(~(0x1 << 4))
static inline const char *
max77779_chg_jeita_ctrl_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " IN_ADC_FORCE=%x",
		FIELD2VALUE(MAX77779_IN_ADC_FORCE, val));
	i += scnprintf(&buff[i], len - i, " THM2_TEMP_FORCE=%x",
		FIELD2VALUE(MAX77779_THM2_TEMP_FORCE, val));
	i += scnprintf(&buff[i], len - i, " THM3_CHECK_EN=%x",
		FIELD2VALUE(MAX77779_THM3_CHECK_EN, val));
	i += scnprintf(&buff[i], len - i, " BATT_ID1_DO=%x",
		FIELD2VALUE(MAX77779_BATT_ID1_DO, val));
	i += scnprintf(&buff[i], len - i, " BATT_ID2_DO=%x",
		FIELD2VALUE(MAX77779_BATT_ID2_DO, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_jeita_ctrl_in_adc_force,0,0)
MAX77779_BFF(max77779_chg_jeita_ctrl_thm2_temp_force,1,1)
MAX77779_BFF(max77779_chg_jeita_ctrl_thm3_check_en,2,2)
MAX77779_BFF(max77779_chg_jeita_ctrl_batt_id1_do,3,3)
MAX77779_BFF(max77779_chg_jeita_ctrl_batt_id2_do,4,4)

/*
 * MAX77779_CHG_JEITA_FLAGS,0x13,0b00000000,0x0,Reset_Type:S
 * CHGIN_ADC_ON,WCIN_ADC_ON,THM3_TEMP_ON,THM2_TEMP_ON,THM1_TEMP_ON,BATT_ID2_ADC_OK,
 * BATT_ID1_ADC_OK
 */
#define MAX77779_CHG_JEITA_FLAGS	0x13
#define MAX77779_CHG_JEITA_FLAGS_CHGIN_ADC_ON_SHIFT	0
#define MAX77779_CHG_JEITA_FLAGS_CHGIN_ADC_ON_MASK	(0x1 << 0)
#define MAX77779_CHG_JEITA_FLAGS_CHGIN_ADC_ON_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_JEITA_FLAGS_WCIN_ADC_ON_SHIFT	1
#define MAX77779_CHG_JEITA_FLAGS_WCIN_ADC_ON_MASK	(0x1 << 1)
#define MAX77779_CHG_JEITA_FLAGS_WCIN_ADC_ON_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_JEITA_FLAGS_THM3_TEMP_ON_SHIFT	2
#define MAX77779_CHG_JEITA_FLAGS_THM3_TEMP_ON_MASK	(0x1 << 2)
#define MAX77779_CHG_JEITA_FLAGS_THM3_TEMP_ON_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_JEITA_FLAGS_THM2_TEMP_ON_SHIFT	3
#define MAX77779_CHG_JEITA_FLAGS_THM2_TEMP_ON_MASK	(0x1 << 3)
#define MAX77779_CHG_JEITA_FLAGS_THM2_TEMP_ON_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_JEITA_FLAGS_THM1_TEMP_ON_SHIFT	4
#define MAX77779_CHG_JEITA_FLAGS_THM1_TEMP_ON_MASK	(0x1 << 4)
#define MAX77779_CHG_JEITA_FLAGS_THM1_TEMP_ON_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_JEITA_FLAGS_BATT_ID2_ADC_OK_SHIFT	5
#define MAX77779_CHG_JEITA_FLAGS_BATT_ID2_ADC_OK_MASK	(0x1 << 5)
#define MAX77779_CHG_JEITA_FLAGS_BATT_ID2_ADC_OK_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_JEITA_FLAGS_BATT_ID1_ADC_OK_SHIFT	6
#define MAX77779_CHG_JEITA_FLAGS_BATT_ID1_ADC_OK_MASK	(0x1 << 6)
#define MAX77779_CHG_JEITA_FLAGS_BATT_ID1_ADC_OK_CLEAR	(~(0x1 << 6))
static inline const char *
max77779_chg_jeita_flags_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CHGIN_ADC_ON=%x",
		FIELD2VALUE(MAX77779_CHGIN_ADC_ON, val));
	i += scnprintf(&buff[i], len - i, " WCIN_ADC_ON=%x",
		FIELD2VALUE(MAX77779_WCIN_ADC_ON, val));
	i += scnprintf(&buff[i], len - i, " THM3_TEMP_ON=%x",
		FIELD2VALUE(MAX77779_THM3_TEMP_ON, val));
	i += scnprintf(&buff[i], len - i, " THM2_TEMP_ON=%x",
		FIELD2VALUE(MAX77779_THM2_TEMP_ON, val));
	i += scnprintf(&buff[i], len - i, " THM1_TEMP_ON=%x",
		FIELD2VALUE(MAX77779_THM1_TEMP_ON, val));
	i += scnprintf(&buff[i], len - i, " BATT_ID2_ADC_OK=%x",
		FIELD2VALUE(MAX77779_BATT_ID2_ADC_OK, val));
	i += scnprintf(&buff[i], len - i, " BATT_ID1_ADC_OK=%x",
		FIELD2VALUE(MAX77779_BATT_ID1_ADC_OK, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_jeita_flags_chgin_adc_on,0,0)
MAX77779_BFF(max77779_chg_jeita_flags_wcin_adc_on,1,1)
MAX77779_BFF(max77779_chg_jeita_flags_thm3_temp_on,2,2)
MAX77779_BFF(max77779_chg_jeita_flags_thm2_temp_on,3,3)
MAX77779_BFF(max77779_chg_jeita_flags_thm1_temp_on,4,4)
MAX77779_BFF(max77779_chg_jeita_flags_batt_id2_adc_ok,5,5)
MAX77779_BFF(max77779_chg_jeita_flags_batt_id1_adc_ok,6,6)

/*
 * MAX77779_CHG_COP_CTRL,0x20,0b00000000,0x0,Reset_Type:S
 * COP_EN,COP_LIMIT_WD_EN,COP_WARN_STS,COP_ALERT_STS
 */
#define MAX77779_CHG_COP_CTRL	0x20
#define MAX77779_CHG_COP_CTRL_COP_EN_SHIFT	0
#define MAX77779_CHG_COP_CTRL_COP_EN_MASK	(0x1 << 0)
#define MAX77779_CHG_COP_CTRL_COP_EN_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_COP_CTRL_COP_LIMIT_WD_EN_SHIFT	1
#define MAX77779_CHG_COP_CTRL_COP_LIMIT_WD_EN_MASK	(0x1 << 1)
#define MAX77779_CHG_COP_CTRL_COP_LIMIT_WD_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_COP_CTRL_COP_WARN_STS_SHIFT	6
#define MAX77779_CHG_COP_CTRL_COP_WARN_STS_MASK	(0x1 << 6)
#define MAX77779_CHG_COP_CTRL_COP_WARN_STS_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_COP_CTRL_COP_ALERT_STS_SHIFT	7
#define MAX77779_CHG_COP_CTRL_COP_ALERT_STS_MASK	(0x1 << 7)
#define MAX77779_CHG_COP_CTRL_COP_ALERT_STS_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_cop_ctrl_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " COP_EN=%x",
		FIELD2VALUE(MAX77779_COP_EN, val));
	i += scnprintf(&buff[i], len - i, " COP_LIMIT_WD_EN=%x",
		FIELD2VALUE(MAX77779_COP_LIMIT_WD_EN, val));
	i += scnprintf(&buff[i], len - i, " COP_WARN_STS=%x",
		FIELD2VALUE(MAX77779_COP_WARN_STS, val));
	i += scnprintf(&buff[i], len - i, " COP_ALERT_STS=%x",
		FIELD2VALUE(MAX77779_COP_ALERT_STS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cop_ctrl_cop_en,0,0)
MAX77779_BFF(max77779_chg_cop_ctrl_cop_limit_wd_en,1,1)
MAX77779_BFF(max77779_chg_cop_ctrl_cop_warn_sts,6,6)
MAX77779_BFF(max77779_chg_cop_ctrl_cop_alert_sts,7,7)

/*
 * MAX77779_CHG_COP_DEBOUNCE,0x21,0b00000000,0x0,Reset_Type:S
 * COP_DB_TIME[0:3]
 */
#define MAX77779_CHG_COP_DEBOUNCE	0x21
#define MAX77779_CHG_COP_DEBOUNCE_COP_DB_TIME_SHIFT	0
#define MAX77779_CHG_COP_DEBOUNCE_COP_DB_TIME_MASK	(0x7 << 0)
#define MAX77779_CHG_COP_DEBOUNCE_COP_DB_TIME_CLEAR	(~(0x7 << 0))
static inline const char *
max77779_chg_cop_debounce_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " COP_DB_TIME=%x",
		FIELD2VALUE(MAX77779_COP_DB_TIME, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cop_debounce_cop_db_time,2,0)

/*
 * MAX77779_CHG_COP_WARN_L,0x22,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_COP_WARN_L	0x22

/*
 * MAX77779_CHG_COP_WARN_H,0x23,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_COP_WARN_H	0x23

/*
 * MAX77779_CHG_COP_LIMIT_L,0x24,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_COP_LIMIT_L	0x24

/*
 * MAX77779_CHG_COP_LIMIT_H,0x25,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_CHG_COP_LIMIT_H	0x25

/*******************************************************
 * Section: CHARGER_FUNC 0xB0 8
 */

/*
 * MAX77779_CHG_INT,0x00,0b00000000,0x0,Reset_Type:S
 * BYP_I,THM2_I,INLIM_I,BAT_I,CHG_I,WCIN_I,CHGIN_I,AICL_I
 */
#define MAX77779_CHG_INT	0xb0
#define MAX77779_CHG_INT_BYP_I_SHIFT	0
#define MAX77779_CHG_INT_BYP_I_MASK	(0x1 << 0)
#define MAX77779_CHG_INT_BYP_I_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_INT_THM2_I_SHIFT	1
#define MAX77779_CHG_INT_THM2_I_MASK	(0x1 << 1)
#define MAX77779_CHG_INT_THM2_I_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_INT_INLIM_I_SHIFT	2
#define MAX77779_CHG_INT_INLIM_I_MASK	(0x1 << 2)
#define MAX77779_CHG_INT_INLIM_I_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_INT_BAT_I_SHIFT	3
#define MAX77779_CHG_INT_BAT_I_MASK	(0x1 << 3)
#define MAX77779_CHG_INT_BAT_I_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_INT_CHG_I_SHIFT	4
#define MAX77779_CHG_INT_CHG_I_MASK	(0x1 << 4)
#define MAX77779_CHG_INT_CHG_I_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_INT_WCIN_I_SHIFT	5
#define MAX77779_CHG_INT_WCIN_I_MASK	(0x1 << 5)
#define MAX77779_CHG_INT_WCIN_I_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_INT_CHGIN_I_SHIFT	6
#define MAX77779_CHG_INT_CHGIN_I_MASK	(0x1 << 6)
#define MAX77779_CHG_INT_CHGIN_I_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_INT_AICL_I_SHIFT	7
#define MAX77779_CHG_INT_AICL_I_MASK	(0x1 << 7)
#define MAX77779_CHG_INT_AICL_I_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_int_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BYP_I=%x",
		FIELD2VALUE(MAX77779_BYP_I, val));
	i += scnprintf(&buff[i], len - i, " THM2_I=%x",
		FIELD2VALUE(MAX77779_THM2_I, val));
	i += scnprintf(&buff[i], len - i, " INLIM_I=%x",
		FIELD2VALUE(MAX77779_INLIM_I, val));
	i += scnprintf(&buff[i], len - i, " BAT_I=%x",
		FIELD2VALUE(MAX77779_BAT_I, val));
	i += scnprintf(&buff[i], len - i, " CHG_I=%x",
		FIELD2VALUE(MAX77779_CHG_I, val));
	i += scnprintf(&buff[i], len - i, " WCIN_I=%x",
		FIELD2VALUE(MAX77779_WCIN_I, val));
	i += scnprintf(&buff[i], len - i, " CHGIN_I=%x",
		FIELD2VALUE(MAX77779_CHGIN_I, val));
	i += scnprintf(&buff[i], len - i, " AICL_I=%x",
		FIELD2VALUE(MAX77779_AICL_I, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_int_byp_i,0,0)
MAX77779_BFF(max77779_chg_int_thm2_i,1,1)
MAX77779_BFF(max77779_chg_int_inlim_i,2,2)
MAX77779_BFF(max77779_chg_int_bat_i,3,3)
MAX77779_BFF(max77779_chg_int_chg_i,4,4)
MAX77779_BFF(max77779_chg_int_wcin_i,5,5)
MAX77779_BFF(max77779_chg_int_chgin_i,6,6)
MAX77779_BFF(max77779_chg_int_aicl_i,7,7)

/*
 * MAX77779_CHG_INT2,0x01,0b00000000,0x0,Reset_Type:S
 * CHG_STA_DONE_I,CHG_STA_TO_I,CHG_STA_CV_I,CHG_STA_CC_I,COP_WARN_I,COP_ALERT_I,
 * COP_LIMIT_WD_I,INSEL_I
 */
#define MAX77779_CHG_INT2	0xb1
#define MAX77779_CHG_INT2_CHG_STA_DONE_I_SHIFT	0
#define MAX77779_CHG_INT2_CHG_STA_DONE_I_MASK	(0x1 << 0)
#define MAX77779_CHG_INT2_CHG_STA_DONE_I_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_INT2_CHG_STA_TO_I_SHIFT	1
#define MAX77779_CHG_INT2_CHG_STA_TO_I_MASK	(0x1 << 1)
#define MAX77779_CHG_INT2_CHG_STA_TO_I_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_INT2_CHG_STA_CV_I_SHIFT	2
#define MAX77779_CHG_INT2_CHG_STA_CV_I_MASK	(0x1 << 2)
#define MAX77779_CHG_INT2_CHG_STA_CV_I_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_INT2_CHG_STA_CC_I_SHIFT	3
#define MAX77779_CHG_INT2_CHG_STA_CC_I_MASK	(0x1 << 3)
#define MAX77779_CHG_INT2_CHG_STA_CC_I_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_INT2_COP_WARN_I_SHIFT	4
#define MAX77779_CHG_INT2_COP_WARN_I_MASK	(0x1 << 4)
#define MAX77779_CHG_INT2_COP_WARN_I_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_INT2_COP_ALERT_I_SHIFT	5
#define MAX77779_CHG_INT2_COP_ALERT_I_MASK	(0x1 << 5)
#define MAX77779_CHG_INT2_COP_ALERT_I_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_INT2_COP_LIMIT_WD_I_SHIFT	6
#define MAX77779_CHG_INT2_COP_LIMIT_WD_I_MASK	(0x1 << 6)
#define MAX77779_CHG_INT2_COP_LIMIT_WD_I_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_INT2_INSEL_I_SHIFT	7
#define MAX77779_CHG_INT2_INSEL_I_MASK	(0x1 << 7)
#define MAX77779_CHG_INT2_INSEL_I_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_int2_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CHG_STA_DONE_I=%x",
		FIELD2VALUE(MAX77779_CHG_STA_DONE_I, val));
	i += scnprintf(&buff[i], len - i, " CHG_STA_TO_I=%x",
		FIELD2VALUE(MAX77779_CHG_STA_TO_I, val));
	i += scnprintf(&buff[i], len - i, " CHG_STA_CV_I=%x",
		FIELD2VALUE(MAX77779_CHG_STA_CV_I, val));
	i += scnprintf(&buff[i], len - i, " CHG_STA_CC_I=%x",
		FIELD2VALUE(MAX77779_CHG_STA_CC_I, val));
	i += scnprintf(&buff[i], len - i, " COP_WARN_I=%x",
		FIELD2VALUE(MAX77779_COP_WARN_I, val));
	i += scnprintf(&buff[i], len - i, " COP_ALERT_I=%x",
		FIELD2VALUE(MAX77779_COP_ALERT_I, val));
	i += scnprintf(&buff[i], len - i, " COP_LIMIT_WD_I=%x",
		FIELD2VALUE(MAX77779_COP_LIMIT_WD_I, val));
	i += scnprintf(&buff[i], len - i, " INSEL_I=%x",
		FIELD2VALUE(MAX77779_INSEL_I, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_int2_chg_sta_done_i,0,0)
MAX77779_BFF(max77779_chg_int2_chg_sta_to_i,1,1)
MAX77779_BFF(max77779_chg_int2_chg_sta_cv_i,2,2)
MAX77779_BFF(max77779_chg_int2_chg_sta_cc_i,3,3)
MAX77779_BFF(max77779_chg_int2_cop_warn_i,4,4)
MAX77779_BFF(max77779_chg_int2_cop_alert_i,5,5)
MAX77779_BFF(max77779_chg_int2_cop_limit_wd_i,6,6)
MAX77779_BFF(max77779_chg_int2_insel_i,7,7)

/*
 * MAX77779_CHG_INT_MASK,0x03,0b11111111,0xff,OTP:SHADOW, Reset_Type:O
 * BYP_M,THM2_M,INLIM_M,BAT_M,CHG_M,WCIN_M,CHGIN_M,AICL_M
 */
#define MAX77779_CHG_INT_MASK	0xb3
#define MAX77779_CHG_INT_MASK_BYP_M_SHIFT	0
#define MAX77779_CHG_INT_MASK_BYP_M_MASK	(0x1 << 0)
#define MAX77779_CHG_INT_MASK_BYP_M_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_INT_MASK_THM2_M_SHIFT	1
#define MAX77779_CHG_INT_MASK_THM2_M_MASK	(0x1 << 1)
#define MAX77779_CHG_INT_MASK_THM2_M_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_INT_MASK_INLIM_M_SHIFT	2
#define MAX77779_CHG_INT_MASK_INLIM_M_MASK	(0x1 << 2)
#define MAX77779_CHG_INT_MASK_INLIM_M_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_INT_MASK_BAT_M_SHIFT	3
#define MAX77779_CHG_INT_MASK_BAT_M_MASK	(0x1 << 3)
#define MAX77779_CHG_INT_MASK_BAT_M_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_INT_MASK_CHG_M_SHIFT	4
#define MAX77779_CHG_INT_MASK_CHG_M_MASK	(0x1 << 4)
#define MAX77779_CHG_INT_MASK_CHG_M_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_INT_MASK_WCIN_M_SHIFT	5
#define MAX77779_CHG_INT_MASK_WCIN_M_MASK	(0x1 << 5)
#define MAX77779_CHG_INT_MASK_WCIN_M_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_INT_MASK_CHGIN_M_SHIFT	6
#define MAX77779_CHG_INT_MASK_CHGIN_M_MASK	(0x1 << 6)
#define MAX77779_CHG_INT_MASK_CHGIN_M_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_INT_MASK_AICL_M_SHIFT	7
#define MAX77779_CHG_INT_MASK_AICL_M_MASK	(0x1 << 7)
#define MAX77779_CHG_INT_MASK_AICL_M_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_int_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BYP_M=%x",
		FIELD2VALUE(MAX77779_BYP_M, val));
	i += scnprintf(&buff[i], len - i, " THM2_M=%x",
		FIELD2VALUE(MAX77779_THM2_M, val));
	i += scnprintf(&buff[i], len - i, " INLIM_M=%x",
		FIELD2VALUE(MAX77779_INLIM_M, val));
	i += scnprintf(&buff[i], len - i, " BAT_M=%x",
		FIELD2VALUE(MAX77779_BAT_M, val));
	i += scnprintf(&buff[i], len - i, " CHG_M=%x",
		FIELD2VALUE(MAX77779_CHG_M, val));
	i += scnprintf(&buff[i], len - i, " WCIN_M=%x",
		FIELD2VALUE(MAX77779_WCIN_M, val));
	i += scnprintf(&buff[i], len - i, " CHGIN_M=%x",
		FIELD2VALUE(MAX77779_CHGIN_M, val));
	i += scnprintf(&buff[i], len - i, " AICL_M=%x",
		FIELD2VALUE(MAX77779_AICL_M, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_int_mask_byp_m,0,0)
MAX77779_BFF(max77779_chg_int_mask_thm2_m,1,1)
MAX77779_BFF(max77779_chg_int_mask_inlim_m,2,2)
MAX77779_BFF(max77779_chg_int_mask_bat_m,3,3)
MAX77779_BFF(max77779_chg_int_mask_chg_m,4,4)
MAX77779_BFF(max77779_chg_int_mask_wcin_m,5,5)
MAX77779_BFF(max77779_chg_int_mask_chgin_m,6,6)
MAX77779_BFF(max77779_chg_int_mask_aicl_m,7,7)

/*
 * MAX77779_CHG_INT2_MASK,0x04,0b11111111,0xff,OTP:SHADOW, Reset_Type:O
 * CHG_STA_DONE_M,CHG_STA_TO_M,CHG_STA_CV_M,CHG_STA_CC_M,COP_WARN_M,COP_ALERT_M,
 * COP_LIMIT_WD_M,INSEL_M
 */
#define MAX77779_CHG_INT2_MASK	0xb4
#define MAX77779_CHG_INT2_MASK_CHG_STA_DONE_M_SHIFT	0
#define MAX77779_CHG_INT2_MASK_CHG_STA_DONE_M_MASK	(0x1 << 0)
#define MAX77779_CHG_INT2_MASK_CHG_STA_DONE_M_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_INT2_MASK_CHG_STA_TO_M_SHIFT	1
#define MAX77779_CHG_INT2_MASK_CHG_STA_TO_M_MASK	(0x1 << 1)
#define MAX77779_CHG_INT2_MASK_CHG_STA_TO_M_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_INT2_MASK_CHG_STA_CV_M_SHIFT	2
#define MAX77779_CHG_INT2_MASK_CHG_STA_CV_M_MASK	(0x1 << 2)
#define MAX77779_CHG_INT2_MASK_CHG_STA_CV_M_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_INT2_MASK_CHG_STA_CC_M_SHIFT	3
#define MAX77779_CHG_INT2_MASK_CHG_STA_CC_M_MASK	(0x1 << 3)
#define MAX77779_CHG_INT2_MASK_CHG_STA_CC_M_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_INT2_MASK_COP_WARN_M_SHIFT	4
#define MAX77779_CHG_INT2_MASK_COP_WARN_M_MASK	(0x1 << 4)
#define MAX77779_CHG_INT2_MASK_COP_WARN_M_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_INT2_MASK_COP_ALERT_M_SHIFT	5
#define MAX77779_CHG_INT2_MASK_COP_ALERT_M_MASK	(0x1 << 5)
#define MAX77779_CHG_INT2_MASK_COP_ALERT_M_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_INT2_MASK_COP_LIMIT_WD_M_SHIFT	6
#define MAX77779_CHG_INT2_MASK_COP_LIMIT_WD_M_MASK	(0x1 << 6)
#define MAX77779_CHG_INT2_MASK_COP_LIMIT_WD_M_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_INT2_MASK_INSEL_M_SHIFT	7
#define MAX77779_CHG_INT2_MASK_INSEL_M_MASK	(0x1 << 7)
#define MAX77779_CHG_INT2_MASK_INSEL_M_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_int2_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CHG_STA_DONE_M=%x",
		FIELD2VALUE(MAX77779_CHG_STA_DONE_M, val));
	i += scnprintf(&buff[i], len - i, " CHG_STA_TO_M=%x",
		FIELD2VALUE(MAX77779_CHG_STA_TO_M, val));
	i += scnprintf(&buff[i], len - i, " CHG_STA_CV_M=%x",
		FIELD2VALUE(MAX77779_CHG_STA_CV_M, val));
	i += scnprintf(&buff[i], len - i, " CHG_STA_CC_M=%x",
		FIELD2VALUE(MAX77779_CHG_STA_CC_M, val));
	i += scnprintf(&buff[i], len - i, " COP_WARN_M=%x",
		FIELD2VALUE(MAX77779_COP_WARN_M, val));
	i += scnprintf(&buff[i], len - i, " COP_ALERT_M=%x",
		FIELD2VALUE(MAX77779_COP_ALERT_M, val));
	i += scnprintf(&buff[i], len - i, " COP_LIMIT_WD_M=%x",
		FIELD2VALUE(MAX77779_COP_LIMIT_WD_M, val));
	i += scnprintf(&buff[i], len - i, " INSEL_M=%x",
		FIELD2VALUE(MAX77779_INSEL_M, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_int2_mask_chg_sta_done_m,0,0)
MAX77779_BFF(max77779_chg_int2_mask_chg_sta_to_m,1,1)
MAX77779_BFF(max77779_chg_int2_mask_chg_sta_cv_m,2,2)
MAX77779_BFF(max77779_chg_int2_mask_chg_sta_cc_m,3,3)
MAX77779_BFF(max77779_chg_int2_mask_cop_warn_m,4,4)
MAX77779_BFF(max77779_chg_int2_mask_cop_alert_m,5,5)
MAX77779_BFF(max77779_chg_int2_mask_cop_limit_wd_m,6,6)
MAX77779_BFF(max77779_chg_int2_mask_insel_m,7,7)

/*
 * MAX77779_CHG_INT_OK,0x06,0b10011111,0x9f,Reset_Type:S
 * BYP_OK,THM2_OK,INLIM_OK,BAT_OK,CHG_OK,WCIN_OK,CHGIN_OK,AICL_OK
 */
#define MAX77779_CHG_INT_OK	0xb6
#define MAX77779_CHG_INT_OK_BYP_OK_SHIFT	0
#define MAX77779_CHG_INT_OK_BYP_OK_MASK	(0x1 << 0)
#define MAX77779_CHG_INT_OK_BYP_OK_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_INT_OK_THM2_OK_SHIFT	1
#define MAX77779_CHG_INT_OK_THM2_OK_MASK	(0x1 << 1)
#define MAX77779_CHG_INT_OK_THM2_OK_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_INT_OK_INLIM_OK_SHIFT	2
#define MAX77779_CHG_INT_OK_INLIM_OK_MASK	(0x1 << 2)
#define MAX77779_CHG_INT_OK_INLIM_OK_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_INT_OK_BAT_OK_SHIFT	3
#define MAX77779_CHG_INT_OK_BAT_OK_MASK	(0x1 << 3)
#define MAX77779_CHG_INT_OK_BAT_OK_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_INT_OK_CHG_OK_SHIFT	4
#define MAX77779_CHG_INT_OK_CHG_OK_MASK	(0x1 << 4)
#define MAX77779_CHG_INT_OK_CHG_OK_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_INT_OK_WCIN_OK_SHIFT	5
#define MAX77779_CHG_INT_OK_WCIN_OK_MASK	(0x1 << 5)
#define MAX77779_CHG_INT_OK_WCIN_OK_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_INT_OK_CHGIN_OK_SHIFT	6
#define MAX77779_CHG_INT_OK_CHGIN_OK_MASK	(0x1 << 6)
#define MAX77779_CHG_INT_OK_CHGIN_OK_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_INT_OK_AICL_OK_SHIFT	7
#define MAX77779_CHG_INT_OK_AICL_OK_MASK	(0x1 << 7)
#define MAX77779_CHG_INT_OK_AICL_OK_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_int_ok_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BYP_OK=%x",
		FIELD2VALUE(MAX77779_BYP_OK, val));
	i += scnprintf(&buff[i], len - i, " THM2_OK=%x",
		FIELD2VALUE(MAX77779_THM2_OK, val));
	i += scnprintf(&buff[i], len - i, " INLIM_OK=%x",
		FIELD2VALUE(MAX77779_INLIM_OK, val));
	i += scnprintf(&buff[i], len - i, " BAT_OK=%x",
		FIELD2VALUE(MAX77779_BAT_OK, val));
	i += scnprintf(&buff[i], len - i, " CHG_OK=%x",
		FIELD2VALUE(MAX77779_CHG_OK, val));
	i += scnprintf(&buff[i], len - i, " WCIN_OK=%x",
		FIELD2VALUE(MAX77779_WCIN_OK, val));
	i += scnprintf(&buff[i], len - i, " CHGIN_OK=%x",
		FIELD2VALUE(MAX77779_CHGIN_OK, val));
	i += scnprintf(&buff[i], len - i, " AICL_OK=%x",
		FIELD2VALUE(MAX77779_AICL_OK, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_int_ok_byp_ok,0,0)
MAX77779_BFF(max77779_chg_int_ok_thm2_ok,1,1)
MAX77779_BFF(max77779_chg_int_ok_inlim_ok,2,2)
MAX77779_BFF(max77779_chg_int_ok_bat_ok,3,3)
MAX77779_BFF(max77779_chg_int_ok_chg_ok,4,4)
MAX77779_BFF(max77779_chg_int_ok_wcin_ok,5,5)
MAX77779_BFF(max77779_chg_int_ok_chgin_ok,6,6)
MAX77779_BFF(max77779_chg_int_ok_aicl_ok,7,7)

/*
 * MAX77779_CHG_DETAILS_00,0x07,0b10000000,0x80,Reset_Type:S
 * TREG,SPSN_DTLS[1:2],WCIN_DTLS[3:2],CHGIN_DTLS[5:2],VDROOP1_OK
 */
#define MAX77779_CHG_DETAILS_00	0xb7
#define MAX77779_CHG_DETAILS_00_TREG_SHIFT	0
#define MAX77779_CHG_DETAILS_00_TREG_MASK	(0x1 << 0)
#define MAX77779_CHG_DETAILS_00_TREG_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_DETAILS_00_SPSN_DTLS_SHIFT	1
#define MAX77779_CHG_DETAILS_00_SPSN_DTLS_MASK	(0x3 << 1)
#define MAX77779_CHG_DETAILS_00_SPSN_DTLS_CLEAR	(~(0x3 << 1))
#define MAX77779_CHG_DETAILS_00_WCIN_DTLS_SHIFT	3
#define MAX77779_CHG_DETAILS_00_WCIN_DTLS_MASK	(0x3 << 3)
#define MAX77779_CHG_DETAILS_00_WCIN_DTLS_CLEAR	(~(0x3 << 3))
#define MAX77779_CHG_DETAILS_00_CHGIN_DTLS_SHIFT	5
#define MAX77779_CHG_DETAILS_00_CHGIN_DTLS_MASK	(0x3 << 5)
#define MAX77779_CHG_DETAILS_00_CHGIN_DTLS_CLEAR	(~(0x3 << 5))
#define MAX77779_CHG_DETAILS_00_VDROOP1_OK_SHIFT	7
#define MAX77779_CHG_DETAILS_00_VDROOP1_OK_MASK	(0x1 << 7)
#define MAX77779_CHG_DETAILS_00_VDROOP1_OK_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_details_00_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TREG=%x",
		FIELD2VALUE(MAX77779_TREG, val));
	i += scnprintf(&buff[i], len - i, " SPSN_DTLS=%x",
		FIELD2VALUE(MAX77779_SPSN_DTLS, val));
	i += scnprintf(&buff[i], len - i, " WCIN_DTLS=%x",
		FIELD2VALUE(MAX77779_WCIN_DTLS, val));
	i += scnprintf(&buff[i], len - i, " CHGIN_DTLS=%x",
		FIELD2VALUE(MAX77779_CHGIN_DTLS, val));
	i += scnprintf(&buff[i], len - i, " VDROOP1_OK=%x",
		FIELD2VALUE(MAX77779_VDROOP1_OK, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_details_00_treg,0,0)
MAX77779_BFF(max77779_chg_details_00_spsn_dtls,2,1)
MAX77779_BFF(max77779_chg_details_00_wcin_dtls,4,3)
MAX77779_BFF(max77779_chg_details_00_chgin_dtls,6,5)
MAX77779_BFF(max77779_chg_details_00_vdroop1_ok,7,7)

/*
 * MAX77779_CHG_DETAILS_01,0x08,0b11111000,0xf8,Reset_Type:S
 * CHG_DTLS[0:4],BAT_DTLS[4:3],VDROOP2_OK
 */
#define MAX77779_CHG_DETAILS_01	0xb8
#define MAX77779_CHG_DETAILS_01_CHG_DTLS_SHIFT	0
#define MAX77779_CHG_DETAILS_01_CHG_DTLS_MASK	(0xf << 0)
#define MAX77779_CHG_DETAILS_01_CHG_DTLS_CLEAR	(~(0xf << 0))
#define MAX77779_CHG_DETAILS_01_BAT_DTLS_SHIFT	4
#define MAX77779_CHG_DETAILS_01_BAT_DTLS_MASK	(0x7 << 4)
#define MAX77779_CHG_DETAILS_01_BAT_DTLS_CLEAR	(~(0x7 << 4))
#define MAX77779_CHG_DETAILS_01_VDROOP2_OK_SHIFT	7
#define MAX77779_CHG_DETAILS_01_VDROOP2_OK_MASK	(0x1 << 7)
#define MAX77779_CHG_DETAILS_01_VDROOP2_OK_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_details_01_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CHG_DTLS=%x",
		FIELD2VALUE(MAX77779_CHG_DTLS, val));
	i += scnprintf(&buff[i], len - i, " BAT_DTLS=%x",
		FIELD2VALUE(MAX77779_BAT_DTLS, val));
	i += scnprintf(&buff[i], len - i, " VDROOP2_OK=%x",
		FIELD2VALUE(MAX77779_VDROOP2_OK, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_details_01_chg_dtls,3,0)
MAX77779_BFF(max77779_chg_details_01_bat_dtls,6,4)
MAX77779_BFF(max77779_chg_details_01_vdroop2_ok,7,7)

/*
 * MAX77779_CHG_DETAILS_02,0x09,0b00000000,0x0,Reset_Type:S
 * BYP_DTLS[0:4],WCIN_STS,CHGIN_STS,NXT_BCK_INPUT[6:2]
 */
#define MAX77779_CHG_DETAILS_02	0xb9
#define MAX77779_CHG_DETAILS_02_BYP_DTLS_SHIFT	0
#define MAX77779_CHG_DETAILS_02_BYP_DTLS_MASK	(0xf << 0)
#define MAX77779_CHG_DETAILS_02_BYP_DTLS_CLEAR	(~(0xf << 0))
#define MAX77779_CHG_DETAILS_02_WCIN_STS_SHIFT	4
#define MAX77779_CHG_DETAILS_02_WCIN_STS_MASK	(0x1 << 4)
#define MAX77779_CHG_DETAILS_02_WCIN_STS_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_DETAILS_02_CHGIN_STS_SHIFT	5
#define MAX77779_CHG_DETAILS_02_CHGIN_STS_MASK	(0x1 << 5)
#define MAX77779_CHG_DETAILS_02_CHGIN_STS_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_DETAILS_02_NXT_BCK_INPUT_SHIFT	6
#define MAX77779_CHG_DETAILS_02_NXT_BCK_INPUT_MASK	(0x3 << 6)
#define MAX77779_CHG_DETAILS_02_NXT_BCK_INPUT_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_chg_details_02_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BYP_DTLS=%x",
		FIELD2VALUE(MAX77779_BYP_DTLS, val));
	i += scnprintf(&buff[i], len - i, " WCIN_STS=%x",
		FIELD2VALUE(MAX77779_WCIN_STS, val));
	i += scnprintf(&buff[i], len - i, " CHGIN_STS=%x",
		FIELD2VALUE(MAX77779_CHGIN_STS, val));
	i += scnprintf(&buff[i], len - i, " NXT_BCK_INPUT=%x",
		FIELD2VALUE(MAX77779_NXT_BCK_INPUT, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_details_02_byp_dtls,3,0)
MAX77779_BFF(max77779_chg_details_02_wcin_sts,4,4)
MAX77779_BFF(max77779_chg_details_02_chgin_sts,5,5)
MAX77779_BFF(max77779_chg_details_02_nxt_bck_input,7,6)

/*
 * MAX77779_CHG_DETAILS_03,0x0A,0b00010010,0x12,Reset_Type:S
 * THM1_DTLS[0:3],THM3_DTLS[3:3],JEITA_AUX_DTLS[6:2]
 */
#define MAX77779_CHG_DETAILS_03	0xba
#define MAX77779_CHG_DETAILS_03_THM1_DTLS_SHIFT	0
#define MAX77779_CHG_DETAILS_03_THM1_DTLS_MASK	(0x7 << 0)
#define MAX77779_CHG_DETAILS_03_THM1_DTLS_CLEAR	(~(0x7 << 0))
#define MAX77779_CHG_DETAILS_03_THM3_DTLS_SHIFT	3
#define MAX77779_CHG_DETAILS_03_THM3_DTLS_MASK	(0x7 << 3)
#define MAX77779_CHG_DETAILS_03_THM3_DTLS_CLEAR	(~(0x7 << 3))
#define MAX77779_CHG_DETAILS_03_JEITA_AUX_DTLS_SHIFT	6
#define MAX77779_CHG_DETAILS_03_JEITA_AUX_DTLS_MASK	(0x3 << 6)
#define MAX77779_CHG_DETAILS_03_JEITA_AUX_DTLS_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_chg_details_03_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " THM1_DTLS=%x",
		FIELD2VALUE(MAX77779_THM1_DTLS, val));
	i += scnprintf(&buff[i], len - i, " THM3_DTLS=%x",
		FIELD2VALUE(MAX77779_THM3_DTLS, val));
	i += scnprintf(&buff[i], len - i, " JEITA_AUX_DTLS=%x",
		FIELD2VALUE(MAX77779_JEITA_AUX_DTLS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_details_03_thm1_dtls,2,0)
MAX77779_BFF(max77779_chg_details_03_thm3_dtls,5,3)
MAX77779_BFF(max77779_chg_details_03_jeita_aux_dtls,7,6)

/*
 * MAX77779_CHG_DETAILS_04,0x0B,0b00000000,0x0,Reset_Type:S
 * FSHIP_EXIT_DTLS[0:2],PD_DTLS[2:2],BAT_OILO1_OPEN,BAT_OILO2_OPEN,BCK_MAXVAL_STS
 */
#define MAX77779_CHG_DETAILS_04	0xbb
#define MAX77779_CHG_DETAILS_04_FSHIP_EXIT_DTLS_SHIFT	0
#define MAX77779_CHG_DETAILS_04_FSHIP_EXIT_DTLS_MASK	(0x3 << 0)
#define MAX77779_CHG_DETAILS_04_FSHIP_EXIT_DTLS_CLEAR	(~(0x3 << 0))
#define MAX77779_CHG_DETAILS_04_PD_DTLS_SHIFT	2
#define MAX77779_CHG_DETAILS_04_PD_DTLS_MASK	(0x3 << 2)
#define MAX77779_CHG_DETAILS_04_PD_DTLS_CLEAR	(~(0x3 << 2))
#define MAX77779_CHG_DETAILS_04_BAT_OILO1_OPEN_SHIFT	4
#define MAX77779_CHG_DETAILS_04_BAT_OILO1_OPEN_MASK	(0x1 << 4)
#define MAX77779_CHG_DETAILS_04_BAT_OILO1_OPEN_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_DETAILS_04_BAT_OILO2_OPEN_SHIFT	5
#define MAX77779_CHG_DETAILS_04_BAT_OILO2_OPEN_MASK	(0x1 << 5)
#define MAX77779_CHG_DETAILS_04_BAT_OILO2_OPEN_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_DETAILS_04_BCK_MAXVAL_STS_SHIFT	6
#define MAX77779_CHG_DETAILS_04_BCK_MAXVAL_STS_MASK	(0x1 << 6)
#define MAX77779_CHG_DETAILS_04_BCK_MAXVAL_STS_CLEAR	(~(0x1 << 6))
static inline const char *
max77779_chg_details_04_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " FSHIP_EXIT_DTLS=%x",
		FIELD2VALUE(MAX77779_FSHIP_EXIT_DTLS, val));
	i += scnprintf(&buff[i], len - i, " PD_DTLS=%x",
		FIELD2VALUE(MAX77779_PD_DTLS, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_OPEN=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_OPEN, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO2_OPEN=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_OPEN, val));
	i += scnprintf(&buff[i], len - i, " BCK_MAXVAL_STS=%x",
		FIELD2VALUE(MAX77779_BCK_MAXVAL_STS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_details_04_fship_exit_dtls,1,0)
MAX77779_BFF(max77779_chg_details_04_pd_dtls,3,2)
MAX77779_BFF(max77779_chg_details_04_bat_oilo1_open,4,4)
MAX77779_BFF(max77779_chg_details_04_bat_oilo2_open,5,5)
MAX77779_BFF(max77779_chg_details_04_bck_maxval_sts,6,6)

/*
 * MAX77779_CHG_CNFG_00,0x0C,0b00000100,0x4,OTP:SHADOW, Reset_Type:O
 * MODE[0:4],BYPV_RAMP_BYPASS,CP_EN,WDTCLR[6:2]
 */
#define MAX77779_CHG_CNFG_00	0xbc
#define MAX77779_CHG_CNFG_00_MODE_SHIFT	0
#define MAX77779_CHG_CNFG_00_MODE_MASK	(0xf << 0)
#define MAX77779_CHG_CNFG_00_MODE_CLEAR	(~(0xf << 0))
#define MAX77779_CHG_CNFG_00_BYPV_RAMP_BYPASS_SHIFT	4
#define MAX77779_CHG_CNFG_00_BYPV_RAMP_BYPASS_MASK	(0x1 << 4)
#define MAX77779_CHG_CNFG_00_BYPV_RAMP_BYPASS_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_CNFG_00_CP_EN_SHIFT	5
#define MAX77779_CHG_CNFG_00_CP_EN_MASK	(0x1 << 5)
#define MAX77779_CHG_CNFG_00_CP_EN_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_CNFG_00_WDTCLR_SHIFT	6
#define MAX77779_CHG_CNFG_00_WDTCLR_MASK	(0x3 << 6)
#define MAX77779_CHG_CNFG_00_WDTCLR_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_chg_cnfg_00_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " MODE=%x",
		FIELD2VALUE(MAX77779_MODE, val));
	i += scnprintf(&buff[i], len - i, " BYPV_RAMP_BYPASS=%x",
		FIELD2VALUE(MAX77779_BYPV_RAMP_BYPASS, val));
	i += scnprintf(&buff[i], len - i, " CP_EN=%x",
		FIELD2VALUE(MAX77779_CP_EN, val));
	i += scnprintf(&buff[i], len - i, " WDTCLR=%x",
		FIELD2VALUE(MAX77779_WDTCLR, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_00_mode,3,0)
MAX77779_BFF(max77779_chg_cnfg_00_bypv_ramp_bypass,4,4)
MAX77779_BFF(max77779_chg_cnfg_00_cp_en,5,5)
MAX77779_BFF(max77779_chg_cnfg_00_wdtclr,7,6)

/*
 * MAX77779_CHG_CNFG_01,0x0D,0b10011001,0x99,OTP:SHADOW, Reset_Type:O
 * FCHGTIME[0:3],RECYCLE_EN,CHG_RSTRT[4:2],LSEL,PQEN
 */
#define MAX77779_CHG_CNFG_01	0xbd
#define MAX77779_CHG_CNFG_01_FCHGTIME_SHIFT	0
#define MAX77779_CHG_CNFG_01_FCHGTIME_MASK	(0x7 << 0)
#define MAX77779_CHG_CNFG_01_FCHGTIME_CLEAR	(~(0x7 << 0))
#define MAX77779_CHG_CNFG_01_RECYCLE_EN_SHIFT	3
#define MAX77779_CHG_CNFG_01_RECYCLE_EN_MASK	(0x1 << 3)
#define MAX77779_CHG_CNFG_01_RECYCLE_EN_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_CNFG_01_CHG_RSTRT_SHIFT	4
#define MAX77779_CHG_CNFG_01_CHG_RSTRT_MASK	(0x3 << 4)
#define MAX77779_CHG_CNFG_01_CHG_RSTRT_CLEAR	(~(0x3 << 4))
#define MAX77779_CHG_CNFG_01_LSEL_SHIFT	6
#define MAX77779_CHG_CNFG_01_LSEL_MASK	(0x1 << 6)
#define MAX77779_CHG_CNFG_01_LSEL_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_CNFG_01_PQEN_SHIFT	7
#define MAX77779_CHG_CNFG_01_PQEN_MASK	(0x1 << 7)
#define MAX77779_CHG_CNFG_01_PQEN_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_cnfg_01_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " FCHGTIME=%x",
		FIELD2VALUE(MAX77779_FCHGTIME, val));
	i += scnprintf(&buff[i], len - i, " RECYCLE_EN=%x",
		FIELD2VALUE(MAX77779_RECYCLE_EN, val));
	i += scnprintf(&buff[i], len - i, " CHG_RSTRT=%x",
		FIELD2VALUE(MAX77779_CHG_RSTRT, val));
	i += scnprintf(&buff[i], len - i, " LSEL=%x",
		FIELD2VALUE(MAX77779_LSEL, val));
	i += scnprintf(&buff[i], len - i, " PQEN=%x",
		FIELD2VALUE(MAX77779_PQEN, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_01_fchgtime,2,0)
MAX77779_BFF(max77779_chg_cnfg_01_recycle_en,3,3)
MAX77779_BFF(max77779_chg_cnfg_01_chg_rstrt,5,4)
MAX77779_BFF(max77779_chg_cnfg_01_lsel,6,6)
MAX77779_BFF(max77779_chg_cnfg_01_pqen,7,7)

/*
 * MAX77779_CHG_CNFG_02,0x0E,0b00000111,0x7,OTP:SHADOW, Reset_Type:O
 * CHGCC[0:6]
 */
#define MAX77779_CHG_CNFG_02	0xbe
#define MAX77779_CHG_CNFG_02_CHGCC_SHIFT	0
#define MAX77779_CHG_CNFG_02_CHGCC_MASK	(0x3f << 0)
#define MAX77779_CHG_CNFG_02_CHGCC_CLEAR	(~(0x3f << 0))
static inline const char *
max77779_chg_cnfg_02_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CHGCC=%x",
		FIELD2VALUE(MAX77779_CHGCC, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_02_chgcc,5,0)

/*
 * MAX77779_CHG_CNFG_03,0x0F,0b11011001,0xd9,OTP:SHADOW, Reset_Type:O
 * TO_ITH[0:3],TO_TIME[3:3],AUTO_FSHIP_MODE_EN,SYS_TRACK_DIS
 */
#define MAX77779_CHG_CNFG_03	0xbf
#define MAX77779_CHG_CNFG_03_TO_ITH_SHIFT	0
#define MAX77779_CHG_CNFG_03_TO_ITH_MASK	(0x7 << 0)
#define MAX77779_CHG_CNFG_03_TO_ITH_CLEAR	(~(0x7 << 0))
#define MAX77779_CHG_CNFG_03_TO_TIME_SHIFT	3
#define MAX77779_CHG_CNFG_03_TO_TIME_MASK	(0x7 << 3)
#define MAX77779_CHG_CNFG_03_TO_TIME_CLEAR	(~(0x7 << 3))
#define MAX77779_CHG_CNFG_03_AUTO_FSHIP_MODE_EN_SHIFT	6
#define MAX77779_CHG_CNFG_03_AUTO_FSHIP_MODE_EN_MASK	(0x1 << 6)
#define MAX77779_CHG_CNFG_03_AUTO_FSHIP_MODE_EN_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_CNFG_03_SYS_TRACK_DIS_SHIFT	7
#define MAX77779_CHG_CNFG_03_SYS_TRACK_DIS_MASK	(0x1 << 7)
#define MAX77779_CHG_CNFG_03_SYS_TRACK_DIS_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_cnfg_03_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TO_ITH=%x",
		FIELD2VALUE(MAX77779_TO_ITH, val));
	i += scnprintf(&buff[i], len - i, " TO_TIME=%x",
		FIELD2VALUE(MAX77779_TO_TIME, val));
	i += scnprintf(&buff[i], len - i, " AUTO_FSHIP_MODE_EN=%x",
		FIELD2VALUE(MAX77779_AUTO_FSHIP_MODE_EN, val));
	i += scnprintf(&buff[i], len - i, " SYS_TRACK_DIS=%x",
		FIELD2VALUE(MAX77779_SYS_TRACK_DIS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_03_to_ith,2,0)
MAX77779_BFF(max77779_chg_cnfg_03_to_time,5,3)
MAX77779_BFF(max77779_chg_cnfg_03_auto_fship_mode_en,6,6)
MAX77779_BFF(max77779_chg_cnfg_03_sys_track_dis,7,7)

/*
 * MAX77779_CHG_CNFG_04,0x10,0b00010100,0x14,OTP:SHADOW, Reset_Type:O
 * CHG_CV_PRM[0:6]
 */
#define MAX77779_CHG_CNFG_04	0xc0
#define MAX77779_CHG_CNFG_04_CHG_CV_PRM_SHIFT	0
#define MAX77779_CHG_CNFG_04_CHG_CV_PRM_MASK	(0x3f << 0)
#define MAX77779_CHG_CNFG_04_CHG_CV_PRM_CLEAR	(~(0x3f << 0))
static inline const char *
max77779_chg_cnfg_04_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CHG_CV_PRM=%x",
		FIELD2VALUE(MAX77779_CHG_CV_PRM, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_04_chg_cv_prm,5,0)

/*
 * MAX77779_CHG_CNFG_05,0x11,0b00110110,0x36,OTP:SHADOW, Reset_Type:O
 * OTG_ILIM[0:4],WCSM_ILIM[4:4]
 */
#define MAX77779_CHG_CNFG_05	0xc1
#define MAX77779_CHG_CNFG_05_OTG_ILIM_SHIFT	0
#define MAX77779_CHG_CNFG_05_OTG_ILIM_MASK	(0xf << 0)
#define MAX77779_CHG_CNFG_05_OTG_ILIM_CLEAR	(~(0xf << 0))
#define MAX77779_CHG_CNFG_05_WCSM_ILIM_SHIFT	4
#define MAX77779_CHG_CNFG_05_WCSM_ILIM_MASK	(0xf << 4)
#define MAX77779_CHG_CNFG_05_WCSM_ILIM_CLEAR	(~(0xf << 4))
static inline const char *
max77779_chg_cnfg_05_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " OTG_ILIM=%x",
		FIELD2VALUE(MAX77779_OTG_ILIM, val));
	i += scnprintf(&buff[i], len - i, " WCSM_ILIM=%x",
		FIELD2VALUE(MAX77779_WCSM_ILIM, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_05_otg_ilim,3,0)
MAX77779_BFF(max77779_chg_cnfg_05_wcsm_ilim,7,4)

/*
 * MAX77779_CHG_CNFG_06,0x12,0b00000001,0x1,OTP:SHADOW, Reset_Type:O
 * WCIN_PD_DIS,CHGPROT[2:2],CHG_CTM_KEY[4:4]
 */
#define MAX77779_CHG_CNFG_06	0xc2
#define MAX77779_CHG_CNFG_06_WCIN_PD_DIS_SHIFT	0
#define MAX77779_CHG_CNFG_06_WCIN_PD_DIS_MASK	(0x1 << 0)
#define MAX77779_CHG_CNFG_06_WCIN_PD_DIS_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_CNFG_06_CHGPROT_SHIFT	2
#define MAX77779_CHG_CNFG_06_CHGPROT_MASK	(0x3 << 2)
#define MAX77779_CHG_CNFG_06_CHGPROT_CLEAR	(~(0x3 << 2))
#define MAX77779_CHG_CNFG_06_CHG_CTM_KEY_SHIFT	4
#define MAX77779_CHG_CNFG_06_CHG_CTM_KEY_MASK	(0xf << 4)
#define MAX77779_CHG_CNFG_06_CHG_CTM_KEY_CLEAR	(~(0xf << 4))
static inline const char *
max77779_chg_cnfg_06_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " WCIN_PD_DIS=%x",
		FIELD2VALUE(MAX77779_WCIN_PD_DIS, val));
	i += scnprintf(&buff[i], len - i, " CHGPROT=%x",
		FIELD2VALUE(MAX77779_CHGPROT, val));
	i += scnprintf(&buff[i], len - i, " CHG_CTM_KEY=%x",
		FIELD2VALUE(MAX77779_CHG_CTM_KEY, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_06_wcin_pd_dis,0,0)
MAX77779_BFF(max77779_chg_cnfg_06_chgprot,3,2)
MAX77779_BFF(max77779_chg_cnfg_06_chg_ctm_key,7,4)

/*
 * MAX77779_CHG_CNFG_07,0x13,0b00110000,0x30,OTP:SHADOW, Reset_Type:O
 * FSHIP_MODE,REGTEMP[3:4]
 */
#define MAX77779_CHG_CNFG_07	0xc3
#define MAX77779_CHG_CNFG_07_FSHIP_MODE_SHIFT	0
#define MAX77779_CHG_CNFG_07_FSHIP_MODE_MASK	(0x1 << 0)
#define MAX77779_CHG_CNFG_07_FSHIP_MODE_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_CNFG_07_REGTEMP_SHIFT	3
#define MAX77779_CHG_CNFG_07_REGTEMP_MASK	(0xf << 3)
#define MAX77779_CHG_CNFG_07_REGTEMP_CLEAR	(~(0xf << 3))
static inline const char *
max77779_chg_cnfg_07_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " FSHIP_MODE=%x",
		FIELD2VALUE(MAX77779_FSHIP_MODE, val));
	i += scnprintf(&buff[i], len - i, " REGTEMP=%x",
		FIELD2VALUE(MAX77779_REGTEMP, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_07_fship_mode,0,0)
MAX77779_BFF(max77779_chg_cnfg_07_regtemp,6,3)

/*
 * MAX77779_CHG_CNFG_08,0x14,0b00000101,0x5,OTP:SHADOW, Reset_Type:O
 * FSW[0:2],THM1_JEITA_EN,ICHGCC_COOL,VCHGCV_COOL,ICHGCC_WARM,VCHGCV_WARM
 */
#define MAX77779_CHG_CNFG_08	0xc4
#define MAX77779_CHG_CNFG_08_FSW_SHIFT	0
#define MAX77779_CHG_CNFG_08_FSW_MASK	(0x3 << 0)
#define MAX77779_CHG_CNFG_08_FSW_CLEAR	(~(0x3 << 0))
#define MAX77779_CHG_CNFG_08_THM1_JEITA_EN_SHIFT	2
#define MAX77779_CHG_CNFG_08_THM1_JEITA_EN_MASK	(0x1 << 2)
#define MAX77779_CHG_CNFG_08_THM1_JEITA_EN_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_CNFG_08_ICHGCC_COOL_SHIFT	3
#define MAX77779_CHG_CNFG_08_ICHGCC_COOL_MASK	(0x1 << 3)
#define MAX77779_CHG_CNFG_08_ICHGCC_COOL_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_CNFG_08_VCHGCV_COOL_SHIFT	4
#define MAX77779_CHG_CNFG_08_VCHGCV_COOL_MASK	(0x1 << 4)
#define MAX77779_CHG_CNFG_08_VCHGCV_COOL_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_CNFG_08_ICHGCC_WARM_SHIFT	5
#define MAX77779_CHG_CNFG_08_ICHGCC_WARM_MASK	(0x1 << 5)
#define MAX77779_CHG_CNFG_08_ICHGCC_WARM_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_CNFG_08_VCHGCV_WARM_SHIFT	6
#define MAX77779_CHG_CNFG_08_VCHGCV_WARM_MASK	(0x1 << 6)
#define MAX77779_CHG_CNFG_08_VCHGCV_WARM_CLEAR	(~(0x1 << 6))
static inline const char *
max77779_chg_cnfg_08_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " FSW=%x",
		FIELD2VALUE(MAX77779_FSW, val));
	i += scnprintf(&buff[i], len - i, " THM1_JEITA_EN=%x",
		FIELD2VALUE(MAX77779_THM1_JEITA_EN, val));
	i += scnprintf(&buff[i], len - i, " ICHGCC_COOL=%x",
		FIELD2VALUE(MAX77779_ICHGCC_COOL, val));
	i += scnprintf(&buff[i], len - i, " VCHGCV_COOL=%x",
		FIELD2VALUE(MAX77779_VCHGCV_COOL, val));
	i += scnprintf(&buff[i], len - i, " ICHGCC_WARM=%x",
		FIELD2VALUE(MAX77779_ICHGCC_WARM, val));
	i += scnprintf(&buff[i], len - i, " VCHGCV_WARM=%x",
		FIELD2VALUE(MAX77779_VCHGCV_WARM, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_08_fsw,1,0)
MAX77779_BFF(max77779_chg_cnfg_08_thm1_jeita_en,2,2)
MAX77779_BFF(max77779_chg_cnfg_08_ichgcc_cool,3,3)
MAX77779_BFF(max77779_chg_cnfg_08_vchgcv_cool,4,4)
MAX77779_BFF(max77779_chg_cnfg_08_ichgcc_warm,5,5)
MAX77779_BFF(max77779_chg_cnfg_08_vchgcv_warm,6,6)

/*
 * MAX77779_CHG_CNFG_09,0x15,0b00010011,0x13,OTP:SHADOW, Reset_Type:O
 * CHGIN_ILIM[0:7],NO_AUTOIBUS
 */
#define MAX77779_CHG_CNFG_09	0xc5
#define MAX77779_CHG_CNFG_09_CHGIN_ILIM_SHIFT	0
#define MAX77779_CHG_CNFG_09_CHGIN_ILIM_MASK	(0x7f << 0)
#define MAX77779_CHG_CNFG_09_CHGIN_ILIM_CLEAR	(~(0x7f << 0))
#define MAX77779_CHG_CNFG_09_NO_AUTOIBUS_SHIFT	7
#define MAX77779_CHG_CNFG_09_NO_AUTOIBUS_MASK	(0x1 << 7)
#define MAX77779_CHG_CNFG_09_NO_AUTOIBUS_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_cnfg_09_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CHGIN_ILIM=%x",
		FIELD2VALUE(MAX77779_CHGIN_ILIM, val));
	i += scnprintf(&buff[i], len - i, " NO_AUTOIBUS=%x",
		FIELD2VALUE(MAX77779_NO_AUTOIBUS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_09_chgin_ilim,6,0)
MAX77779_BFF(max77779_chg_cnfg_09_no_autoibus,7,7)

/*
 * MAX77779_CHG_CNFG_10,0x16,0b00010011,0x13,OTP:SHADOW, Reset_Type:O
 * WCIN_ILIM[0:7],CHGIN_ILIM_SPEED
 */
#define MAX77779_CHG_CNFG_10	0xc6
#define MAX77779_CHG_CNFG_10_WCIN_ILIM_SHIFT	0
#define MAX77779_CHG_CNFG_10_WCIN_ILIM_MASK	(0x7f << 0)
#define MAX77779_CHG_CNFG_10_WCIN_ILIM_CLEAR	(~(0x7f << 0))
#define MAX77779_CHG_CNFG_10_CHGIN_ILIM_SPEED_SHIFT	7
#define MAX77779_CHG_CNFG_10_CHGIN_ILIM_SPEED_MASK	(0x1 << 7)
#define MAX77779_CHG_CNFG_10_CHGIN_ILIM_SPEED_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_cnfg_10_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " WCIN_ILIM=%x",
		FIELD2VALUE(MAX77779_WCIN_ILIM, val));
	i += scnprintf(&buff[i], len - i, " CHGIN_ILIM_SPEED=%x",
		FIELD2VALUE(MAX77779_CHGIN_ILIM_SPEED, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_10_wcin_ilim,6,0)
MAX77779_BFF(max77779_chg_cnfg_10_chgin_ilim_speed,7,7)

/*
 * MAX77779_CHG_CNFG_11,0x17,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 */
#define MAX77779_CHG_CNFG_11	0xc7

/*
 * MAX77779_CHG_CNFG_12,0x18,0b01101000,0x68,OTP:SHADOW, Reset_Type:O
 * DISKIP,WCIN_REG[1:2],VCHGIN_REG[3:2],CHGINSEL,WCINSEL,CHG_EN
 */
#define MAX77779_CHG_CNFG_12	0xc8
#define MAX77779_CHG_CNFG_12_DISKIP_SHIFT	0
#define MAX77779_CHG_CNFG_12_DISKIP_MASK	(0x1 << 0)
#define MAX77779_CHG_CNFG_12_DISKIP_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_CNFG_12_WCIN_REG_SHIFT	1
#define MAX77779_CHG_CNFG_12_WCIN_REG_MASK	(0x3 << 1)
#define MAX77779_CHG_CNFG_12_WCIN_REG_CLEAR	(~(0x3 << 1))
#define MAX77779_CHG_CNFG_12_VCHGIN_REG_SHIFT	3
#define MAX77779_CHG_CNFG_12_VCHGIN_REG_MASK	(0x3 << 3)
#define MAX77779_CHG_CNFG_12_VCHGIN_REG_CLEAR	(~(0x3 << 3))
#define MAX77779_CHG_CNFG_12_CHGINSEL_SHIFT	5
#define MAX77779_CHG_CNFG_12_CHGINSEL_MASK	(0x1 << 5)
#define MAX77779_CHG_CNFG_12_CHGINSEL_CLEAR	(~(0x1 << 5))
#define MAX77779_CHG_CNFG_12_WCINSEL_SHIFT	6
#define MAX77779_CHG_CNFG_12_WCINSEL_MASK	(0x1 << 6)
#define MAX77779_CHG_CNFG_12_WCINSEL_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_CNFG_12_CHG_EN_SHIFT	7
#define MAX77779_CHG_CNFG_12_CHG_EN_MASK	(0x1 << 7)
#define MAX77779_CHG_CNFG_12_CHG_EN_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_cnfg_12_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DISKIP=%x",
		FIELD2VALUE(MAX77779_DISKIP, val));
	i += scnprintf(&buff[i], len - i, " WCIN_REG=%x",
		FIELD2VALUE(MAX77779_WCIN_REG, val));
	i += scnprintf(&buff[i], len - i, " VCHGIN_REG=%x",
		FIELD2VALUE(MAX77779_VCHGIN_REG, val));
	i += scnprintf(&buff[i], len - i, " CHGINSEL=%x",
		FIELD2VALUE(MAX77779_CHGINSEL, val));
	i += scnprintf(&buff[i], len - i, " WCINSEL=%x",
		FIELD2VALUE(MAX77779_WCINSEL, val));
	i += scnprintf(&buff[i], len - i, " CHG_EN=%x",
		FIELD2VALUE(MAX77779_CHG_EN, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_12_diskip,0,0)
MAX77779_BFF(max77779_chg_cnfg_12_wcin_reg,2,1)
MAX77779_BFF(max77779_chg_cnfg_12_vchgin_reg,4,3)
MAX77779_BFF(max77779_chg_cnfg_12_chginsel,5,5)
MAX77779_BFF(max77779_chg_cnfg_12_wcinsel,6,6)
MAX77779_BFF(max77779_chg_cnfg_12_chg_en,7,7)

/*
 * MAX77779_CHG_CNFG_13,0x19,0b00110011,0x33,OTP:SHADOW, Reset_Type:O
 * USB_TEMP_THR[0:3],THM2_HW_CTRL,WCIN_SS_EN,CHGIN_SS_EN
 */
#define MAX77779_CHG_CNFG_13	0xc9
#define MAX77779_CHG_CNFG_13_USB_TEMP_THR_SHIFT	0
#define MAX77779_CHG_CNFG_13_USB_TEMP_THR_MASK	(0x7 << 0)
#define MAX77779_CHG_CNFG_13_USB_TEMP_THR_CLEAR	(~(0x7 << 0))
#define MAX77779_CHG_CNFG_13_THM2_HW_CTRL_SHIFT	3
#define MAX77779_CHG_CNFG_13_THM2_HW_CTRL_MASK	(0x1 << 3)
#define MAX77779_CHG_CNFG_13_THM2_HW_CTRL_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_CNFG_13_WCIN_SS_EN_SHIFT	4
#define MAX77779_CHG_CNFG_13_WCIN_SS_EN_MASK	(0x1 << 4)
#define MAX77779_CHG_CNFG_13_WCIN_SS_EN_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_CNFG_13_CHGIN_SS_EN_SHIFT	5
#define MAX77779_CHG_CNFG_13_CHGIN_SS_EN_MASK	(0x1 << 5)
#define MAX77779_CHG_CNFG_13_CHGIN_SS_EN_CLEAR	(~(0x1 << 5))
static inline const char *
max77779_chg_cnfg_13_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " USB_TEMP_THR=%x",
		FIELD2VALUE(MAX77779_USB_TEMP_THR, val));
	i += scnprintf(&buff[i], len - i, " THM2_HW_CTRL=%x",
		FIELD2VALUE(MAX77779_THM2_HW_CTRL, val));
	i += scnprintf(&buff[i], len - i, " WCIN_SS_EN=%x",
		FIELD2VALUE(MAX77779_WCIN_SS_EN, val));
	i += scnprintf(&buff[i], len - i, " CHGIN_SS_EN=%x",
		FIELD2VALUE(MAX77779_CHGIN_SS_EN, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_13_usb_temp_thr,2,0)
MAX77779_BFF(max77779_chg_cnfg_13_thm2_hw_ctrl,3,3)
MAX77779_BFF(max77779_chg_cnfg_13_wcin_ss_en,4,4)
MAX77779_BFF(max77779_chg_cnfg_13_chgin_ss_en,5,5)

/*
 * MAX77779_CHG_CNFG_14,0x1A,0b01100000,0x60,OTP:SHADOW, Reset_Type:O
 * TPWROUT[3:3],AICL[6:2]
 */
#define MAX77779_CHG_CNFG_14	0xca
#define MAX77779_CHG_CNFG_14_TPWROUT_SHIFT	3
#define MAX77779_CHG_CNFG_14_TPWROUT_MASK	(0x7 << 3)
#define MAX77779_CHG_CNFG_14_TPWROUT_CLEAR	(~(0x7 << 3))
#define MAX77779_CHG_CNFG_14_AICL_SHIFT	6
#define MAX77779_CHG_CNFG_14_AICL_MASK	(0x3 << 6)
#define MAX77779_CHG_CNFG_14_AICL_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_chg_cnfg_14_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TPWROUT=%x",
		FIELD2VALUE(MAX77779_TPWROUT, val));
	i += scnprintf(&buff[i], len - i, " AICL=%x",
		FIELD2VALUE(MAX77779_AICL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_14_tpwrout,5,3)
MAX77779_BFF(max77779_chg_cnfg_14_aicl,7,6)

/*
 * MAX77779_CHG_CNFG_15,0x1B,0b00000000,0x0,OTP:SHADOW, Reset_Type:O
 * WDTEN,PRIMARY_DC,SPSN_DET_EN,OTG_V_PGM,MINVSYS[4:2]
 */
#define MAX77779_CHG_CNFG_15	0xcb
#define MAX77779_CHG_CNFG_15_WDTEN_SHIFT	0
#define MAX77779_CHG_CNFG_15_WDTEN_MASK	(0x1 << 0)
#define MAX77779_CHG_CNFG_15_WDTEN_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_CNFG_15_PRIMARY_DC_SHIFT	1
#define MAX77779_CHG_CNFG_15_PRIMARY_DC_MASK	(0x1 << 1)
#define MAX77779_CHG_CNFG_15_PRIMARY_DC_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_CNFG_15_SPSN_DET_EN_SHIFT	2
#define MAX77779_CHG_CNFG_15_SPSN_DET_EN_MASK	(0x1 << 2)
#define MAX77779_CHG_CNFG_15_SPSN_DET_EN_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_CNFG_15_OTG_V_PGM_SHIFT	3
#define MAX77779_CHG_CNFG_15_OTG_V_PGM_MASK	(0x1 << 3)
#define MAX77779_CHG_CNFG_15_OTG_V_PGM_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_CNFG_15_MINVSYS_SHIFT	4
#define MAX77779_CHG_CNFG_15_MINVSYS_MASK	(0x3 << 4)
#define MAX77779_CHG_CNFG_15_MINVSYS_CLEAR	(~(0x3 << 4))
static inline const char *
max77779_chg_cnfg_15_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " WDTEN=%x",
		FIELD2VALUE(MAX77779_WDTEN, val));
	i += scnprintf(&buff[i], len - i, " PRIMARY_DC=%x",
		FIELD2VALUE(MAX77779_PRIMARY_DC, val));
	i += scnprintf(&buff[i], len - i, " SPSN_DET_EN=%x",
		FIELD2VALUE(MAX77779_SPSN_DET_EN, val));
	i += scnprintf(&buff[i], len - i, " OTG_V_PGM=%x",
		FIELD2VALUE(MAX77779_OTG_V_PGM, val));
	i += scnprintf(&buff[i], len - i, " MINVSYS=%x",
		FIELD2VALUE(MAX77779_MINVSYS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_15_wdten,0,0)
MAX77779_BFF(max77779_chg_cnfg_15_primary_dc,1,1)
MAX77779_BFF(max77779_chg_cnfg_15_spsn_det_en,2,2)
MAX77779_BFF(max77779_chg_cnfg_15_otg_v_pgm,3,3)
MAX77779_BFF(max77779_chg_cnfg_15_minvsys,5,4)

/*
 * MAX77779_CHG_CNFG_16,0x1C,0b11110000,0xf0,OTP:SHADOW, Reset_Type:O
 * SLOWLX[0:2],DIS_IR_CTRL,INLIM_CLK[3:2],AUTO_FSHIP_TIME[5:3]
 */
#define MAX77779_CHG_CNFG_16	0xcc
#define MAX77779_CHG_CNFG_16_SLOWLX_SHIFT	0
#define MAX77779_CHG_CNFG_16_SLOWLX_MASK	(0x3 << 0)
#define MAX77779_CHG_CNFG_16_SLOWLX_CLEAR	(~(0x3 << 0))
#define MAX77779_CHG_CNFG_16_DIS_IR_CTRL_SHIFT	2
#define MAX77779_CHG_CNFG_16_DIS_IR_CTRL_MASK	(0x1 << 2)
#define MAX77779_CHG_CNFG_16_DIS_IR_CTRL_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_CNFG_16_INLIM_CLK_SHIFT	3
#define MAX77779_CHG_CNFG_16_INLIM_CLK_MASK	(0x3 << 3)
#define MAX77779_CHG_CNFG_16_INLIM_CLK_CLEAR	(~(0x3 << 3))
#define MAX77779_CHG_CNFG_16_AUTO_FSHIP_TIME_SHIFT	5
#define MAX77779_CHG_CNFG_16_AUTO_FSHIP_TIME_MASK	(0x7 << 5)
#define MAX77779_CHG_CNFG_16_AUTO_FSHIP_TIME_CLEAR	(~(0x7 << 5))
static inline const char *
max77779_chg_cnfg_16_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SLOWLX=%x",
		FIELD2VALUE(MAX77779_SLOWLX, val));
	i += scnprintf(&buff[i], len - i, " DIS_IR_CTRL=%x",
		FIELD2VALUE(MAX77779_DIS_IR_CTRL, val));
	i += scnprintf(&buff[i], len - i, " INLIM_CLK=%x",
		FIELD2VALUE(MAX77779_INLIM_CLK, val));
	i += scnprintf(&buff[i], len - i, " AUTO_FSHIP_TIME=%x",
		FIELD2VALUE(MAX77779_AUTO_FSHIP_TIME, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_16_slowlx,1,0)
MAX77779_BFF(max77779_chg_cnfg_16_dis_ir_ctrl,2,2)
MAX77779_BFF(max77779_chg_cnfg_16_inlim_clk,4,3)
MAX77779_BFF(max77779_chg_cnfg_16_auto_fship_time,7,5)

/*
 * MAX77779_CHG_CNFG_17,0x1D,0b00000011,0x3,OTP:SHADOW, Reset_Type:O
 * THM3_JEITA_EN,JEITA_AUX_EN,JEITA_AUX_ZONE,ICHGCC_JAUX,VCHGCV_JAUX,
 * VDP1_STP_BST,VDP2_STP_BST
 */
#define MAX77779_CHG_CNFG_17	0xcd
#define MAX77779_CHG_CNFG_17_THM3_JEITA_EN_SHIFT	0
#define MAX77779_CHG_CNFG_17_THM3_JEITA_EN_MASK	(0x1 << 0)
#define MAX77779_CHG_CNFG_17_THM3_JEITA_EN_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_CNFG_17_JEITA_AUX_EN_SHIFT	1
#define MAX77779_CHG_CNFG_17_JEITA_AUX_EN_MASK	(0x1 << 1)
#define MAX77779_CHG_CNFG_17_JEITA_AUX_EN_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_CNFG_17_JEITA_AUX_ZONE_SHIFT	2
#define MAX77779_CHG_CNFG_17_JEITA_AUX_ZONE_MASK	(0x1 << 2)
#define MAX77779_CHG_CNFG_17_JEITA_AUX_ZONE_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_CNFG_17_ICHGCC_JAUX_SHIFT	3
#define MAX77779_CHG_CNFG_17_ICHGCC_JAUX_MASK	(0x1 << 3)
#define MAX77779_CHG_CNFG_17_ICHGCC_JAUX_CLEAR	(~(0x1 << 3))
#define MAX77779_CHG_CNFG_17_VCHGCV_JAUX_SHIFT	4
#define MAX77779_CHG_CNFG_17_VCHGCV_JAUX_MASK	(0x1 << 4)
#define MAX77779_CHG_CNFG_17_VCHGCV_JAUX_CLEAR	(~(0x1 << 4))
#define MAX77779_CHG_CNFG_17_VDP1_STP_BST_SHIFT	6
#define MAX77779_CHG_CNFG_17_VDP1_STP_BST_MASK	(0x1 << 6)
#define MAX77779_CHG_CNFG_17_VDP1_STP_BST_CLEAR	(~(0x1 << 6))
#define MAX77779_CHG_CNFG_17_VDP2_STP_BST_SHIFT	7
#define MAX77779_CHG_CNFG_17_VDP2_STP_BST_MASK	(0x1 << 7)
#define MAX77779_CHG_CNFG_17_VDP2_STP_BST_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_chg_cnfg_17_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " THM3_JEITA_EN=%x",
		FIELD2VALUE(MAX77779_THM3_JEITA_EN, val));
	i += scnprintf(&buff[i], len - i, " JEITA_AUX_EN=%x",
		FIELD2VALUE(MAX77779_JEITA_AUX_EN, val));
	i += scnprintf(&buff[i], len - i, " JEITA_AUX_ZONE=%x",
		FIELD2VALUE(MAX77779_JEITA_AUX_ZONE, val));
	i += scnprintf(&buff[i], len - i, " ICHGCC_JAUX=%x",
		FIELD2VALUE(MAX77779_ICHGCC_JAUX, val));
	i += scnprintf(&buff[i], len - i, " VCHGCV_JAUX=%x",
		FIELD2VALUE(MAX77779_VCHGCV_JAUX, val));
	i += scnprintf(&buff[i], len - i, " VDP1_STP_BST=%x",
		FIELD2VALUE(MAX77779_VDP1_STP_BST, val));
	i += scnprintf(&buff[i], len - i, " VDP2_STP_BST=%x",
		FIELD2VALUE(MAX77779_VDP2_STP_BST, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cnfg_17_thm3_jeita_en,0,0)
MAX77779_BFF(max77779_chg_cnfg_17_jeita_aux_en,1,1)
MAX77779_BFF(max77779_chg_cnfg_17_jeita_aux_zone,2,2)
MAX77779_BFF(max77779_chg_cnfg_17_ichgcc_jaux,3,3)
MAX77779_BFF(max77779_chg_cnfg_17_vchgcv_jaux,4,4)
MAX77779_BFF(max77779_chg_cnfg_17_vdp1_stp_bst,6,6)
MAX77779_BFF(max77779_chg_cnfg_17_vdp2_stp_bst,7,7)

/*
 * MAX77779_SYS_UVLO1_CNFG_0,0x1E,0b00001100,0xc,OTP:SHADOW, Reset_Type:O
 * SYS_UVLO1[0:4],SYS_UVLO1_HYST[4:2]
 */
#define MAX77779_SYS_UVLO1_CNFG_0	0xce
#define MAX77779_SYS_UVLO1_CNFG_0_SYS_UVLO1_SHIFT	0
#define MAX77779_SYS_UVLO1_CNFG_0_SYS_UVLO1_MASK	(0xf << 0)
#define MAX77779_SYS_UVLO1_CNFG_0_SYS_UVLO1_CLEAR	(~(0xf << 0))
#define MAX77779_SYS_UVLO1_CNFG_0_SYS_UVLO1_HYST_SHIFT	4
#define MAX77779_SYS_UVLO1_CNFG_0_SYS_UVLO1_HYST_MASK	(0x3 << 4)
#define MAX77779_SYS_UVLO1_CNFG_0_SYS_UVLO1_HYST_CLEAR	(~(0x3 << 4))
static inline const char *
max77779_sys_uvlo1_cnfg_0_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SYS_UVLO1=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO1_HYST=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1_HYST, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_sys_uvlo1_cnfg_0_sys_uvlo1,3,0)
MAX77779_BFF(max77779_sys_uvlo1_cnfg_0_sys_uvlo1_hyst,5,4)

/*
 * MAX77779_SYS_UVLO1_CNFG_1,0x1F,0b10000000,0x80,OTP:SHADOW, Reset_Type:O
 * SYS_UVLO1_REL[0:2],SYS_UVLO1_DET,SYS_UVLO1_VDRP1_EN
 */
#define MAX77779_SYS_UVLO1_CNFG_1	0xcf
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_REL_SHIFT	0
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_REL_MASK	(0x3 << 0)
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_REL_CLEAR	(~(0x3 << 0))
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_DET_SHIFT	4
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_DET_MASK	(0x1 << 4)
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_DET_CLEAR	(~(0x1 << 4))
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_VDRP1_EN_SHIFT	7
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_VDRP1_EN_MASK	(0x1 << 7)
#define MAX77779_SYS_UVLO1_CNFG_1_SYS_UVLO1_VDRP1_EN_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_sys_uvlo1_cnfg_1_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SYS_UVLO1_REL=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1_REL, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO1_DET=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1_DET, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO1_VDRP1_EN=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1_VDRP1_EN, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_sys_uvlo1_cnfg_1_sys_uvlo1_rel,1,0)
MAX77779_BFF(max77779_sys_uvlo1_cnfg_1_sys_uvlo1_det,4,4)
MAX77779_BFF(max77779_sys_uvlo1_cnfg_1_sys_uvlo1_vdrp1_en,7,7)

/*
 * MAX77779_SYS_UVLO2_CNFG_0,0x20,0b00001000,0x8,OTP:SHADOW, Reset_Type:O
 * SYS_UVLO2[0:4],SYS_UVLO2_HYST[4:2]
 */
#define MAX77779_SYS_UVLO2_CNFG_0	0xd0
#define MAX77779_SYS_UVLO2_CNFG_0_SYS_UVLO2_SHIFT	0
#define MAX77779_SYS_UVLO2_CNFG_0_SYS_UVLO2_MASK	(0xf << 0)
#define MAX77779_SYS_UVLO2_CNFG_0_SYS_UVLO2_CLEAR	(~(0xf << 0))
#define MAX77779_SYS_UVLO2_CNFG_0_SYS_UVLO2_HYST_SHIFT	4
#define MAX77779_SYS_UVLO2_CNFG_0_SYS_UVLO2_HYST_MASK	(0x3 << 4)
#define MAX77779_SYS_UVLO2_CNFG_0_SYS_UVLO2_HYST_CLEAR	(~(0x3 << 4))
static inline const char *
max77779_sys_uvlo2_cnfg_0_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SYS_UVLO2=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO2_HYST=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2_HYST, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_sys_uvlo2_cnfg_0_sys_uvlo2,3,0)
MAX77779_BFF(max77779_sys_uvlo2_cnfg_0_sys_uvlo2_hyst,5,4)

/*
 * MAX77779_SYS_UVLO2_CNFG_1,0x21,0b10000000,0x80,OTP:SHADOW, Reset_Type:O
 * SYS_UVLO2_REL[0:2],SYS_UVLO2_DET,SYS_UVLO2_VDRP2_EN
 */
#define MAX77779_SYS_UVLO2_CNFG_1	0xd1
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_REL_SHIFT	0
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_REL_MASK	(0x3 << 0)
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_REL_CLEAR	(~(0x3 << 0))
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_DET_SHIFT	4
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_DET_MASK	(0x1 << 4)
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_DET_CLEAR	(~(0x1 << 4))
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_VDRP2_EN_SHIFT	7
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_VDRP2_EN_MASK	(0x1 << 7)
#define MAX77779_SYS_UVLO2_CNFG_1_SYS_UVLO2_VDRP2_EN_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_sys_uvlo2_cnfg_1_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SYS_UVLO2_REL=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2_REL, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO2_DET=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2_DET, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO2_VDRP2_EN=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2_VDRP2_EN, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_sys_uvlo2_cnfg_1_sys_uvlo2_rel,1,0)
MAX77779_BFF(max77779_sys_uvlo2_cnfg_1_sys_uvlo2_det,4,4)
MAX77779_BFF(max77779_sys_uvlo2_cnfg_1_sys_uvlo2_vdrp2_en,7,7)

/*
 * MAX77779_BAT_OILO1_CNFG_0,0x22,0b00010000,0x10,OTP:SHADOW, Reset_Type:O
 * BAT_OILO1[0:5]
 */
#define MAX77779_BAT_OILO1_CNFG_0	0xd2
#define MAX77779_BAT_OILO1_CNFG_0_BAT_OILO1_SHIFT	0
#define MAX77779_BAT_OILO1_CNFG_0_BAT_OILO1_MASK	(0x1f << 0)
#define MAX77779_BAT_OILO1_CNFG_0_BAT_OILO1_CLEAR	(~(0x1f << 0))
static inline const char *
max77779_bat_oilo1_cnfg_0_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OILO1=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bat_oilo1_cnfg_0_bat_oilo1,4,0)

/*
 * MAX77779_BAT_OILO1_CNFG_1,0x23,0b00100000,0x20,OTP:SHADOW, Reset_Type:O
 * BAT_OILO1_DET[0:5],BAT_OILO1_REL[5:3]
 */
#define MAX77779_BAT_OILO1_CNFG_1	0xd3
#define MAX77779_BAT_OILO1_CNFG_1_BAT_OILO1_DET_SHIFT	0
#define MAX77779_BAT_OILO1_CNFG_1_BAT_OILO1_DET_MASK	(0x1f << 0)
#define MAX77779_BAT_OILO1_CNFG_1_BAT_OILO1_DET_CLEAR	(~(0x1f << 0))
#define MAX77779_BAT_OILO1_CNFG_1_BAT_OILO1_REL_SHIFT	5
#define MAX77779_BAT_OILO1_CNFG_1_BAT_OILO1_REL_MASK	(0x7 << 5)
#define MAX77779_BAT_OILO1_CNFG_1_BAT_OILO1_REL_CLEAR	(~(0x7 << 5))
static inline const char *
max77779_bat_oilo1_cnfg_1_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OILO1_DET=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_DET, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_REL=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_REL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bat_oilo1_cnfg_1_bat_oilo1_det,4,0)
MAX77779_BFF(max77779_bat_oilo1_cnfg_1_bat_oilo1_rel,7,5)

/*
 * MAX77779_BAT_OILO1_CNFG_2,0x24,0b00100000,0x20,OTP:SHADOW, Reset_Type:O
 * BAT_OILO1_INT_DET[0:5],BAT_OILO1_INT_REL[5:3]
 */
#define MAX77779_BAT_OILO1_CNFG_2	0xd4
#define MAX77779_BAT_OILO1_CNFG_2_BAT_OILO1_INT_DET_SHIFT	0
#define MAX77779_BAT_OILO1_CNFG_2_BAT_OILO1_INT_DET_MASK	(0x1f << 0)
#define MAX77779_BAT_OILO1_CNFG_2_BAT_OILO1_INT_DET_CLEAR	(~(0x1f << 0))
#define MAX77779_BAT_OILO1_CNFG_2_BAT_OILO1_INT_REL_SHIFT	5
#define MAX77779_BAT_OILO1_CNFG_2_BAT_OILO1_INT_REL_MASK	(0x7 << 5)
#define MAX77779_BAT_OILO1_CNFG_2_BAT_OILO1_INT_REL_CLEAR	(~(0x7 << 5))
static inline const char *
max77779_bat_oilo1_cnfg_2_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OILO1_INT_DET=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_INT_DET, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_INT_REL=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_INT_REL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bat_oilo1_cnfg_2_bat_oilo1_int_det,4,0)
MAX77779_BFF(max77779_bat_oilo1_cnfg_2_bat_oilo1_int_rel,7,5)

/*
 * MAX77779_BAT_OILO1_CNFG_3,0x25,0b01000000,0x40,OTP:SHADOW, Reset_Type:O
 * BAT_OPEN_TO_1[0:4],BAT_OILO_INT_CLR,BAT_OILO1_VDRP1_EN,BAT_OILO1_VDRP2_EN
 */
#define MAX77779_BAT_OILO1_CNFG_3	0xd5
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OPEN_TO_1_SHIFT	0
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OPEN_TO_1_MASK	(0xf << 0)
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OPEN_TO_1_CLEAR	(~(0xf << 0))
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO_INT_CLR_SHIFT	5
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO_INT_CLR_MASK	(0x1 << 5)
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO_INT_CLR_CLEAR	(~(0x1 << 5))
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO1_VDRP1_EN_SHIFT	6
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO1_VDRP1_EN_MASK	(0x1 << 6)
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO1_VDRP1_EN_CLEAR	(~(0x1 << 6))
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO1_VDRP2_EN_SHIFT	7
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO1_VDRP2_EN_MASK	(0x1 << 7)
#define MAX77779_BAT_OILO1_CNFG_3_BAT_OILO1_VDRP2_EN_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_bat_oilo1_cnfg_3_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OPEN_TO_1=%x",
		FIELD2VALUE(MAX77779_BAT_OPEN_TO_1, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO_INT_CLR=%x",
		FIELD2VALUE(MAX77779_BAT_OILO_INT_CLR, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_VDRP1_EN=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_VDRP1_EN, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_VDRP2_EN=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_VDRP2_EN, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bat_oilo1_cnfg_3_bat_open_to_1,3,0)
MAX77779_BFF(max77779_bat_oilo1_cnfg_3_bat_oilo_int_clr,5,5)
MAX77779_BFF(max77779_bat_oilo1_cnfg_3_bat_oilo1_vdrp1_en,6,6)
MAX77779_BFF(max77779_bat_oilo1_cnfg_3_bat_oilo1_vdrp2_en,7,7)

/*
 * MAX77779_BAT_OILO2_CNFG_0,0x26,0b00000110,0x6,OTP:SHADOW, Reset_Type:O
 * BAT_OILO2[0:5]
 */
#define MAX77779_BAT_OILO2_CNFG_0	0xd6
#define MAX77779_BAT_OILO2_CNFG_0_BAT_OILO2_SHIFT	0
#define MAX77779_BAT_OILO2_CNFG_0_BAT_OILO2_MASK	(0x1f << 0)
#define MAX77779_BAT_OILO2_CNFG_0_BAT_OILO2_CLEAR	(~(0x1f << 0))
static inline const char *
max77779_bat_oilo2_cnfg_0_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OILO2=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bat_oilo2_cnfg_0_bat_oilo2,4,0)

/*
 * MAX77779_BAT_OILO2_CNFG_1,0x27,0b00100000,0x20,OTP:SHADOW, Reset_Type:O
 * BAT_OILO2_DET[0:5],BAT_OILO2_REL[5:3]
 */
#define MAX77779_BAT_OILO2_CNFG_1	0xd7
#define MAX77779_BAT_OILO2_CNFG_1_BAT_OILO2_DET_SHIFT	0
#define MAX77779_BAT_OILO2_CNFG_1_BAT_OILO2_DET_MASK	(0x1f << 0)
#define MAX77779_BAT_OILO2_CNFG_1_BAT_OILO2_DET_CLEAR	(~(0x1f << 0))
#define MAX77779_BAT_OILO2_CNFG_1_BAT_OILO2_REL_SHIFT	5
#define MAX77779_BAT_OILO2_CNFG_1_BAT_OILO2_REL_MASK	(0x7 << 5)
#define MAX77779_BAT_OILO2_CNFG_1_BAT_OILO2_REL_CLEAR	(~(0x7 << 5))
static inline const char *
max77779_bat_oilo2_cnfg_1_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OILO2_DET=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_DET, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO2_REL=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_REL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bat_oilo2_cnfg_1_bat_oilo2_det,4,0)
MAX77779_BFF(max77779_bat_oilo2_cnfg_1_bat_oilo2_rel,7,5)

/*
 * MAX77779_BAT_OILO2_CNFG_2,0x28,0b00100000,0x20,OTP:SHADOW, Reset_Type:O
 * BAT_OILO2_INT_DET[0:5],BAT_OILO2_INT_REL[5:3]
 */
#define MAX77779_BAT_OILO2_CNFG_2	0xd8
#define MAX77779_BAT_OILO2_CNFG_2_BAT_OILO2_INT_DET_SHIFT	0
#define MAX77779_BAT_OILO2_CNFG_2_BAT_OILO2_INT_DET_MASK	(0x1f << 0)
#define MAX77779_BAT_OILO2_CNFG_2_BAT_OILO2_INT_DET_CLEAR	(~(0x1f << 0))
#define MAX77779_BAT_OILO2_CNFG_2_BAT_OILO2_INT_REL_SHIFT	5
#define MAX77779_BAT_OILO2_CNFG_2_BAT_OILO2_INT_REL_MASK	(0x7 << 5)
#define MAX77779_BAT_OILO2_CNFG_2_BAT_OILO2_INT_REL_CLEAR	(~(0x7 << 5))
static inline const char *
max77779_bat_oilo2_cnfg_2_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OILO2_INT_DET=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_INT_DET, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO2_INT_REL=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_INT_REL, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bat_oilo2_cnfg_2_bat_oilo2_int_det,4,0)
MAX77779_BFF(max77779_bat_oilo2_cnfg_2_bat_oilo2_int_rel,7,5)

/*
 * MAX77779_BAT_OILO2_CNFG_3,0x29,0b10001000,0x88,OTP:SHADOW, Reset_Type:O
 * BAT_OPEN_TO_2[0:4],BAT_OILO2_VDRP1_EN,BAT_OILO2_VDRP2_EN
 */
#define MAX77779_BAT_OILO2_CNFG_3	0xd9
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OPEN_TO_2_SHIFT	0
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OPEN_TO_2_MASK	(0xf << 0)
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OPEN_TO_2_CLEAR	(~(0xf << 0))
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OILO2_VDRP1_EN_SHIFT	6
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OILO2_VDRP1_EN_MASK	(0x1 << 6)
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OILO2_VDRP1_EN_CLEAR	(~(0x1 << 6))
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OILO2_VDRP2_EN_SHIFT	7
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OILO2_VDRP2_EN_MASK	(0x1 << 7)
#define MAX77779_BAT_OILO2_CNFG_3_BAT_OILO2_VDRP2_EN_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_bat_oilo2_cnfg_3_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OPEN_TO_2=%x",
		FIELD2VALUE(MAX77779_BAT_OPEN_TO_2, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO2_VDRP1_EN=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_VDRP1_EN, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO2_VDRP2_EN=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_VDRP2_EN, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bat_oilo2_cnfg_3_bat_open_to_2,3,0)
MAX77779_BFF(max77779_bat_oilo2_cnfg_3_bat_oilo2_vdrp1_en,6,6)
MAX77779_BFF(max77779_bat_oilo2_cnfg_3_bat_oilo2_vdrp2_en,7,7)

/*
 * MAX77779_CHG_CUST_TM,0x2E,0b00000000,0x0,Reset_Type:O
 * BAT_OILO2_CTM,BAT_OILO1_CTM,SYS_UVLO2_CTM,SYS_UVLO1_CTM
 */
#define MAX77779_CHG_CUST_TM	0xde
#define MAX77779_CHG_CUST_TM_BAT_OILO2_CTM_SHIFT	0
#define MAX77779_CHG_CUST_TM_BAT_OILO2_CTM_MASK	(0x1 << 0)
#define MAX77779_CHG_CUST_TM_BAT_OILO2_CTM_CLEAR	(~(0x1 << 0))
#define MAX77779_CHG_CUST_TM_BAT_OILO1_CTM_SHIFT	1
#define MAX77779_CHG_CUST_TM_BAT_OILO1_CTM_MASK	(0x1 << 1)
#define MAX77779_CHG_CUST_TM_BAT_OILO1_CTM_CLEAR	(~(0x1 << 1))
#define MAX77779_CHG_CUST_TM_SYS_UVLO2_CTM_SHIFT	2
#define MAX77779_CHG_CUST_TM_SYS_UVLO2_CTM_MASK	(0x1 << 2)
#define MAX77779_CHG_CUST_TM_SYS_UVLO2_CTM_CLEAR	(~(0x1 << 2))
#define MAX77779_CHG_CUST_TM_SYS_UVLO1_CTM_SHIFT	3
#define MAX77779_CHG_CUST_TM_SYS_UVLO1_CTM_MASK	(0x1 << 3)
#define MAX77779_CHG_CUST_TM_SYS_UVLO1_CTM_CLEAR	(~(0x1 << 3))
static inline const char *
max77779_chg_cust_tm_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BAT_OILO2_CTM=%x",
		FIELD2VALUE(MAX77779_BAT_OILO2_CTM, val));
	i += scnprintf(&buff[i], len - i, " BAT_OILO1_CTM=%x",
		FIELD2VALUE(MAX77779_BAT_OILO1_CTM, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO2_CTM=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO2_CTM, val));
	i += scnprintf(&buff[i], len - i, " SYS_UVLO1_CTM=%x",
		FIELD2VALUE(MAX77779_SYS_UVLO1_CTM, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_chg_cust_tm_bat_oilo2_ctm,0,0)
MAX77779_BFF(max77779_chg_cust_tm_bat_oilo1_ctm,1,1)
MAX77779_BFF(max77779_chg_cust_tm_sys_uvlo2_ctm,2,2)
MAX77779_BFF(max77779_chg_cust_tm_sys_uvlo1_ctm,3,3)

/*******************************************************
 * Section: FG_RAM 0x00 16
 */

/*
 * MAX77779_FG_Status,0x00,0b00000010,0x2,Reset_Type:S
 * PONR,Imn,Bst,CmdFwDone,Imx,dSOCi,Vmn,Tmn,Smn,Bi,Vmx,Tmx,Smx,Br
 */
#define MAX77779_FG_Status	0x00
#define MAX77779_FG_Status_PONR_SHIFT	1
#define MAX77779_FG_Status_PONR_MASK	(0x1 << 1)
#define MAX77779_FG_Status_PONR_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_Status_Imn_SHIFT	2
#define MAX77779_FG_Status_Imn_MASK	(0x1 << 2)
#define MAX77779_FG_Status_Imn_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_Status_Bst_SHIFT	3
#define MAX77779_FG_Status_Bst_MASK	(0x1 << 3)
#define MAX77779_FG_Status_Bst_CLEAR	(~(0x1 << 3))
#define MAX77779_FG_Status_CmdFwDone_SHIFT	4
#define MAX77779_FG_Status_CmdFwDone_MASK	(0x1 << 4)
#define MAX77779_FG_Status_CmdFwDone_CLEAR	(~(0x1 << 4))
#define MAX77779_FG_Status_Imx_SHIFT	6
#define MAX77779_FG_Status_Imx_MASK	(0x1 << 6)
#define MAX77779_FG_Status_Imx_CLEAR	(~(0x1 << 6))
#define MAX77779_FG_Status_dSOCi_SHIFT	7
#define MAX77779_FG_Status_dSOCi_MASK	(0x1 << 7)
#define MAX77779_FG_Status_dSOCi_CLEAR	(~(0x1 << 7))
#define MAX77779_FG_Status_Vmn_SHIFT	8
#define MAX77779_FG_Status_Vmn_MASK	(0x1 << 8)
#define MAX77779_FG_Status_Vmn_CLEAR	(~(0x1 << 8))
#define MAX77779_FG_Status_Tmn_SHIFT	9
#define MAX77779_FG_Status_Tmn_MASK	(0x1 << 9)
#define MAX77779_FG_Status_Tmn_CLEAR	(~(0x1 << 9))
#define MAX77779_FG_Status_Smn_SHIFT	10
#define MAX77779_FG_Status_Smn_MASK	(0x1 << 10)
#define MAX77779_FG_Status_Smn_CLEAR	(~(0x1 << 10))
#define MAX77779_FG_Status_Bi_SHIFT	11
#define MAX77779_FG_Status_Bi_MASK	(0x1 << 11)
#define MAX77779_FG_Status_Bi_CLEAR	(~(0x1 << 11))
#define MAX77779_FG_Status_Vmx_SHIFT	12
#define MAX77779_FG_Status_Vmx_MASK	(0x1 << 12)
#define MAX77779_FG_Status_Vmx_CLEAR	(~(0x1 << 12))
#define MAX77779_FG_Status_Tmx_SHIFT	13
#define MAX77779_FG_Status_Tmx_MASK	(0x1 << 13)
#define MAX77779_FG_Status_Tmx_CLEAR	(~(0x1 << 13))
#define MAX77779_FG_Status_Smx_SHIFT	14
#define MAX77779_FG_Status_Smx_MASK	(0x1 << 14)
#define MAX77779_FG_Status_Smx_CLEAR	(~(0x1 << 14))
#define MAX77779_FG_Status_Br_SHIFT	15
#define MAX77779_FG_Status_Br_MASK	(0x1 << 15)
#define MAX77779_FG_Status_Br_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_fg_status_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " PONR=%x",
		FIELD2VALUE(MAX77779_PONR, val));
	i += scnprintf(&buff[i], len - i, " Imn=%x",
		FIELD2VALUE(MAX77779_Imn, val));
	i += scnprintf(&buff[i], len - i, " Bst=%x",
		FIELD2VALUE(MAX77779_Bst, val));
	i += scnprintf(&buff[i], len - i, " CmdFwDone=%x",
		FIELD2VALUE(MAX77779_CmdFwDone, val));
	i += scnprintf(&buff[i], len - i, " Imx=%x",
		FIELD2VALUE(MAX77779_Imx, val));
	i += scnprintf(&buff[i], len - i, " dSOCi=%x",
		FIELD2VALUE(MAX77779_dSOCi, val));
	i += scnprintf(&buff[i], len - i, " Vmn=%x",
		FIELD2VALUE(MAX77779_Vmn, val));
	i += scnprintf(&buff[i], len - i, " Tmn=%x",
		FIELD2VALUE(MAX77779_Tmn, val));
	i += scnprintf(&buff[i], len - i, " Smn=%x",
		FIELD2VALUE(MAX77779_Smn, val));
	i += scnprintf(&buff[i], len - i, " Bi=%x",
		FIELD2VALUE(MAX77779_Bi, val));
	i += scnprintf(&buff[i], len - i, " Vmx=%x",
		FIELD2VALUE(MAX77779_Vmx, val));
	i += scnprintf(&buff[i], len - i, " Tmx=%x",
		FIELD2VALUE(MAX77779_Tmx, val));
	i += scnprintf(&buff[i], len - i, " Smx=%x",
		FIELD2VALUE(MAX77779_Smx, val));
	i += scnprintf(&buff[i], len - i, " Br=%x",
		FIELD2VALUE(MAX77779_Br, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_status_ponr,1,1)
MAX77779_BFF(max77779_fg_status_imn,2,2)
MAX77779_BFF(max77779_fg_status_bst,3,3)
MAX77779_BFF(max77779_fg_status_cmdfwdone,4,4)
MAX77779_BFF(max77779_fg_status_imx,6,6)
MAX77779_BFF(max77779_fg_status_dsoci,7,7)
MAX77779_BFF(max77779_fg_status_vmn,8,8)
MAX77779_BFF(max77779_fg_status_tmn,9,9)
MAX77779_BFF(max77779_fg_status_smn,10,10)
MAX77779_BFF(max77779_fg_status_bi,11,11)
MAX77779_BFF(max77779_fg_status_vmx,12,12)
MAX77779_BFF(max77779_fg_status_tmx,13,13)
MAX77779_BFF(max77779_fg_status_smx,14,14)
MAX77779_BFF(max77779_fg_status_br,15,15)

/*
 * MAX77779_FG_ConfigPWR,0x01,0b00000000,0x0,Reset_Type:S
 * VIO_SHDN,CWIN_WK,PWRONB1_WK,PWRONB2_WK
 */
#define MAX77779_FG_ConfigPWR	0x01
#define MAX77779_FG_ConfigPWR_VIO_SHDN_SHIFT	0
#define MAX77779_FG_ConfigPWR_VIO_SHDN_MASK	(0x1 << 0)
#define MAX77779_FG_ConfigPWR_VIO_SHDN_CLEAR	(~(0x1 << 0))
#define MAX77779_FG_ConfigPWR_CWIN_WK_SHIFT	1
#define MAX77779_FG_ConfigPWR_CWIN_WK_MASK	(0x1 << 1)
#define MAX77779_FG_ConfigPWR_CWIN_WK_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_ConfigPWR_PWRONB1_WK_SHIFT	2
#define MAX77779_FG_ConfigPWR_PWRONB1_WK_MASK	(0x1 << 2)
#define MAX77779_FG_ConfigPWR_PWRONB1_WK_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_ConfigPWR_PWRONB2_WK_SHIFT	3
#define MAX77779_FG_ConfigPWR_PWRONB2_WK_MASK	(0x1 << 3)
#define MAX77779_FG_ConfigPWR_PWRONB2_WK_CLEAR	(~(0x1 << 3))
static inline const char *
max77779_fg_configpwr_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " VIO_SHDN=%x",
		FIELD2VALUE(MAX77779_VIO_SHDN, val));
	i += scnprintf(&buff[i], len - i, " CWIN_WK=%x",
		FIELD2VALUE(MAX77779_CWIN_WK, val));
	i += scnprintf(&buff[i], len - i, " PWRONB1_WK=%x",
		FIELD2VALUE(MAX77779_PWRONB1_WK, val));
	i += scnprintf(&buff[i], len - i, " PWRONB2_WK=%x",
		FIELD2VALUE(MAX77779_PWRONB2_WK, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_configpwr_vio_shdn,0,0)
MAX77779_BFF(max77779_fg_configpwr_cwin_wk,1,1)
MAX77779_BFF(max77779_fg_configpwr_pwronb1_wk,2,2)
MAX77779_BFF(max77779_fg_configpwr_pwronb2_wk,3,3)

/*
 * MAX77779_FG_HibCfg,0x02,0b00000000,0x0,Reset_Type:S
 * HibScalar[0:3],HibExitTime[3:2],inusefixed5to7[5:3],HibThreshold[8:4],
 * HibEnterTime[12:3],EnHib
 */
#define MAX77779_FG_HibCfg	0x02
#define MAX77779_FG_HibCfg_HibScalar_SHIFT	0
#define MAX77779_FG_HibCfg_HibScalar_MASK	(0x7 << 0)
#define MAX77779_FG_HibCfg_HibScalar_CLEAR	(~(0x7 << 0))
#define MAX77779_FG_HibCfg_HibExitTime_SHIFT	3
#define MAX77779_FG_HibCfg_HibExitTime_MASK	(0x3 << 3)
#define MAX77779_FG_HibCfg_HibExitTime_CLEAR	(~(0x3 << 3))
#define MAX77779_FG_HibCfg_inusefixed5to7_SHIFT	5
#define MAX77779_FG_HibCfg_inusefixed5to7_MASK	(0x7 << 5)
#define MAX77779_FG_HibCfg_inusefixed5to7_CLEAR	(~(0x7 << 5))
#define MAX77779_FG_HibCfg_HibThreshold_SHIFT	8
#define MAX77779_FG_HibCfg_HibThreshold_MASK	(0xf << 8)
#define MAX77779_FG_HibCfg_HibThreshold_CLEAR	(~(0xf << 8))
#define MAX77779_FG_HibCfg_HibEnterTime_SHIFT	12
#define MAX77779_FG_HibCfg_HibEnterTime_MASK	(0x7 << 12)
#define MAX77779_FG_HibCfg_HibEnterTime_CLEAR	(~(0x7 << 12))
#define MAX77779_FG_HibCfg_EnHib_SHIFT	15
#define MAX77779_FG_HibCfg_EnHib_MASK	(0x1 << 15)
#define MAX77779_FG_HibCfg_EnHib_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_fg_hibcfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " HibScalar=%x",
		FIELD2VALUE(MAX77779_HibScalar, val));
	i += scnprintf(&buff[i], len - i, " HibExitTime=%x",
		FIELD2VALUE(MAX77779_HibExitTime, val));
	i += scnprintf(&buff[i], len - i, " inusefixed5to7=%x",
		FIELD2VALUE(MAX77779_inusefixed5to7, val));
	i += scnprintf(&buff[i], len - i, " HibThreshold=%x",
		FIELD2VALUE(MAX77779_HibThreshold, val));
	i += scnprintf(&buff[i], len - i, " HibEnterTime=%x",
		FIELD2VALUE(MAX77779_HibEnterTime, val));
	i += scnprintf(&buff[i], len - i, " EnHib=%x",
		FIELD2VALUE(MAX77779_EnHib, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_hibcfg_hibscalar,2,0)
MAX77779_BFF(max77779_fg_hibcfg_hibexittime,4,3)
MAX77779_BFF(max77779_fg_hibcfg_inusefixed5to7,7,5)
MAX77779_BFF(max77779_fg_hibcfg_hibthreshold,11,8)
MAX77779_BFF(max77779_fg_hibcfg_hibentertime,14,12)
MAX77779_BFF(max77779_fg_hibcfg_enhib,15,15)

/*
 * MAX77779_FG_VAlrtTh,0x03,0b1111111100000000,0xff00,Reset_Type:S
 * VMIN[0:8],VMAX[8:8]
 */
#define MAX77779_FG_VAlrtTh	0x03
#define MAX77779_FG_VAlrtTh_VMIN_SHIFT	0
#define MAX77779_FG_VAlrtTh_VMIN_MASK	(0xff << 0)
#define MAX77779_FG_VAlrtTh_VMIN_CLEAR	(~(0xff << 0))
#define MAX77779_FG_VAlrtTh_VMAX_SHIFT	8
#define MAX77779_FG_VAlrtTh_VMAX_MASK	(0xff << 8)
#define MAX77779_FG_VAlrtTh_VMAX_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_valrtth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " VMIN=%x",
		FIELD2VALUE(MAX77779_VMIN, val));
	i += scnprintf(&buff[i], len - i, " VMAX=%x",
		FIELD2VALUE(MAX77779_VMAX, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_valrtth_vmin,7,0)
MAX77779_BFF(max77779_fg_valrtth_vmax,15,8)

/*
 * MAX77779_FG_TAlrtTh,0x04,0b111111110000000,0x7f80,Reset_Type:S
 * TMIN[0:8],TMAX[8:8]
 */
#define MAX77779_FG_TAlrtTh	0x04
#define MAX77779_FG_TAlrtTh_TMIN_SHIFT	0
#define MAX77779_FG_TAlrtTh_TMIN_MASK	(0xff << 0)
#define MAX77779_FG_TAlrtTh_TMIN_CLEAR	(~(0xff << 0))
#define MAX77779_FG_TAlrtTh_TMAX_SHIFT	8
#define MAX77779_FG_TAlrtTh_TMAX_MASK	(0xff << 8)
#define MAX77779_FG_TAlrtTh_TMAX_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_talrtth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TMIN=%x",
		FIELD2VALUE(MAX77779_TMIN, val));
	i += scnprintf(&buff[i], len - i, " TMAX=%x",
		FIELD2VALUE(MAX77779_TMAX, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_talrtth_tmin,7,0)
MAX77779_BFF(max77779_fg_talrtth_tmax,15,8)

/*
 * MAX77779_FG_SAlrtTh,0x05,0b1111111100000000,0xff00,Reset_Type:S
 * SMIN[0:8],SMAX[8:8]
 */
#define MAX77779_FG_SAlrtTh	0x05
#define MAX77779_FG_SAlrtTh_SMIN_SHIFT	0
#define MAX77779_FG_SAlrtTh_SMIN_MASK	(0xff << 0)
#define MAX77779_FG_SAlrtTh_SMIN_CLEAR	(~(0xff << 0))
#define MAX77779_FG_SAlrtTh_SMAX_SHIFT	8
#define MAX77779_FG_SAlrtTh_SMAX_MASK	(0xff << 8)
#define MAX77779_FG_SAlrtTh_SMAX_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_salrtth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SMIN=%x",
		FIELD2VALUE(MAX77779_SMIN, val));
	i += scnprintf(&buff[i], len - i, " SMAX=%x",
		FIELD2VALUE(MAX77779_SMAX, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_salrtth_smin,7,0)
MAX77779_BFF(max77779_fg_salrtth_smax,15,8)

/*
 * MAX77779_FG_RepCap,0x06,0b1010011100,0x29c,Reset_Type:S
 */
#define MAX77779_FG_RepCap	0x06

/*
 * MAX77779_FG_RepSOC,0x07,0b10110010001110,0x2c8e,Reset_Type:S
 */
#define MAX77779_FG_RepSOC	0x07

/*
 * MAX77779_FG_ShdnTimer,0x08,0b00000000,0x0,Reset_Type:S
 * CTR[0:13],THR[13:3]
 */
#define MAX77779_FG_ShdnTimer	0x08
#define MAX77779_FG_ShdnTimer_CTR_SHIFT	0
#define MAX77779_FG_ShdnTimer_CTR_MASK	(0x1fff << 0)
#define MAX77779_FG_ShdnTimer_CTR_CLEAR	(~(0x1fff << 0))
#define MAX77779_FG_ShdnTimer_THR_SHIFT	13
#define MAX77779_FG_ShdnTimer_THR_MASK	(0x7 << 13)
#define MAX77779_FG_ShdnTimer_THR_CLEAR	(~(0x7 << 13))
static inline const char *
max77779_fg_shdntimer_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CTR=%x",
		FIELD2VALUE(MAX77779_CTR, val));
	i += scnprintf(&buff[i], len - i, " THR=%x",
		FIELD2VALUE(MAX77779_THR, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_shdntimer_ctr,12,0)
MAX77779_BFF(max77779_fg_shdntimer_thr,15,13)

/*
 * MAX77779_FG_MaxMinTemp,0x09,0b1111001111110011,0xf3f3,Reset_Type:S
 * MINTEMP[0:8],MAXTEMP[8:8]
 */
#define MAX77779_FG_MaxMinTemp	0x09
#define MAX77779_FG_MaxMinTemp_MINTEMP_SHIFT	0
#define MAX77779_FG_MaxMinTemp_MINTEMP_MASK	(0xff << 0)
#define MAX77779_FG_MaxMinTemp_MINTEMP_CLEAR	(~(0xff << 0))
#define MAX77779_FG_MaxMinTemp_MAXTEMP_SHIFT	8
#define MAX77779_FG_MaxMinTemp_MAXTEMP_MASK	(0xff << 8)
#define MAX77779_FG_MaxMinTemp_MAXTEMP_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_maxmintemp_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " MINTEMP=%x",
		FIELD2VALUE(MAX77779_MINTEMP, val));
	i += scnprintf(&buff[i], len - i, " MAXTEMP=%x",
		FIELD2VALUE(MAX77779_MAXTEMP, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_maxmintemp_mintemp,7,0)
MAX77779_BFF(max77779_fg_maxmintemp_maxtemp,15,8)

/*
 * MAX77779_FG_MaxMinCurr,0x0A,0b1111111111111111,0xffff,Reset_Type:S
 * MINCURR[0:8],MAXCURR[8:8]
 */
#define MAX77779_FG_MaxMinCurr	0x0a
#define MAX77779_FG_MaxMinCurr_MINCURR_SHIFT	0
#define MAX77779_FG_MaxMinCurr_MINCURR_MASK	(0xff << 0)
#define MAX77779_FG_MaxMinCurr_MINCURR_CLEAR	(~(0xff << 0))
#define MAX77779_FG_MaxMinCurr_MAXCURR_SHIFT	8
#define MAX77779_FG_MaxMinCurr_MAXCURR_MASK	(0xff << 8)
#define MAX77779_FG_MaxMinCurr_MAXCURR_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_maxmincurr_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " MINCURR=%x",
		FIELD2VALUE(MAX77779_MINCURR, val));
	i += scnprintf(&buff[i], len - i, " MAXCURR=%x",
		FIELD2VALUE(MAX77779_MAXCURR, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_maxmincurr_mincurr,7,0)
MAX77779_BFF(max77779_fg_maxmincurr_maxcurr,15,8)

/*
 * MAX77779_FG_MaxMinVolt,0x0B,0b1011110110111101,0xbdbd,Reset_Type:S
 * MINVOLT[0:8],MAXVOLT[8:8]
 */
#define MAX77779_FG_MaxMinVolt	0x0b
#define MAX77779_FG_MaxMinVolt_MINVOLT_SHIFT	0
#define MAX77779_FG_MaxMinVolt_MINVOLT_MASK	(0xff << 0)
#define MAX77779_FG_MaxMinVolt_MINVOLT_CLEAR	(~(0xff << 0))
#define MAX77779_FG_MaxMinVolt_MAXVOLT_SHIFT	8
#define MAX77779_FG_MaxMinVolt_MAXVOLT_MASK	(0xff << 8)
#define MAX77779_FG_MaxMinVolt_MAXVOLT_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_maxminvolt_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " MINVOLT=%x",
		FIELD2VALUE(MAX77779_MINVOLT, val));
	i += scnprintf(&buff[i], len - i, " MAXVOLT=%x",
		FIELD2VALUE(MAX77779_MAXVOLT, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_maxminvolt_minvolt,7,0)
MAX77779_BFF(max77779_fg_maxminvolt_maxvolt,15,8)

/*
 * MAX77779_FG_Config,0x0C,0b10000001010000,0x2050,Reset_Type:S
 * Ber,Bei,Aen,AlrtEdge,SHDN,Tex,Ten,IS,VS,TS,SS
 */
#define MAX77779_FG_Config	0x0c
#define MAX77779_FG_Config_Ber_SHIFT	0
#define MAX77779_FG_Config_Ber_MASK	(0x1 << 0)
#define MAX77779_FG_Config_Ber_CLEAR	(~(0x1 << 0))
#define MAX77779_FG_Config_Bei_SHIFT	1
#define MAX77779_FG_Config_Bei_MASK	(0x1 << 1)
#define MAX77779_FG_Config_Bei_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_Config_Aen_SHIFT	2
#define MAX77779_FG_Config_Aen_MASK	(0x1 << 2)
#define MAX77779_FG_Config_Aen_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_Config_AlrtEdge_SHIFT	5
#define MAX77779_FG_Config_AlrtEdge_MASK	(0x1 << 5)
#define MAX77779_FG_Config_AlrtEdge_CLEAR	(~(0x1 << 5))
#define MAX77779_FG_Config_SHDN_SHIFT	7
#define MAX77779_FG_Config_SHDN_MASK	(0x1 << 7)
#define MAX77779_FG_Config_SHDN_CLEAR	(~(0x1 << 7))
#define MAX77779_FG_Config_Tex_SHIFT	8
#define MAX77779_FG_Config_Tex_MASK	(0x1 << 8)
#define MAX77779_FG_Config_Tex_CLEAR	(~(0x1 << 8))
#define MAX77779_FG_Config_Ten_SHIFT	9
#define MAX77779_FG_Config_Ten_MASK	(0x1 << 9)
#define MAX77779_FG_Config_Ten_CLEAR	(~(0x1 << 9))
#define MAX77779_FG_Config_IS_SHIFT	11
#define MAX77779_FG_Config_IS_MASK	(0x1 << 11)
#define MAX77779_FG_Config_IS_CLEAR	(~(0x1 << 11))
#define MAX77779_FG_Config_VS_SHIFT	12
#define MAX77779_FG_Config_VS_MASK	(0x1 << 12)
#define MAX77779_FG_Config_VS_CLEAR	(~(0x1 << 12))
#define MAX77779_FG_Config_TS_SHIFT	13
#define MAX77779_FG_Config_TS_MASK	(0x1 << 13)
#define MAX77779_FG_Config_TS_CLEAR	(~(0x1 << 13))
#define MAX77779_FG_Config_SS_SHIFT	14
#define MAX77779_FG_Config_SS_MASK	(0x1 << 14)
#define MAX77779_FG_Config_SS_CLEAR	(~(0x1 << 14))
static inline const char *
max77779_fg_config_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " Ber=%x",
		FIELD2VALUE(MAX77779_Ber, val));
	i += scnprintf(&buff[i], len - i, " Bei=%x",
		FIELD2VALUE(MAX77779_Bei, val));
	i += scnprintf(&buff[i], len - i, " Aen=%x",
		FIELD2VALUE(MAX77779_Aen, val));
	i += scnprintf(&buff[i], len - i, " AlrtEdge=%x",
		FIELD2VALUE(MAX77779_AlrtEdge, val));
	i += scnprintf(&buff[i], len - i, " SHDN=%x",
		FIELD2VALUE(MAX77779_SHDN, val));
	i += scnprintf(&buff[i], len - i, " Tex=%x",
		FIELD2VALUE(MAX77779_Tex, val));
	i += scnprintf(&buff[i], len - i, " Ten=%x",
		FIELD2VALUE(MAX77779_Ten, val));
	i += scnprintf(&buff[i], len - i, " IS=%x",
		FIELD2VALUE(MAX77779_IS, val));
	i += scnprintf(&buff[i], len - i, " VS=%x",
		FIELD2VALUE(MAX77779_VS, val));
	i += scnprintf(&buff[i], len - i, " TS=%x",
		FIELD2VALUE(MAX77779_TS, val));
	i += scnprintf(&buff[i], len - i, " SS=%x",
		FIELD2VALUE(MAX77779_SS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_config_ber,0,0)
MAX77779_BFF(max77779_fg_config_bei,1,1)
MAX77779_BFF(max77779_fg_config_aen,2,2)
MAX77779_BFF(max77779_fg_config_alrtedge,5,5)
MAX77779_BFF(max77779_fg_config_shdn,7,7)
MAX77779_BFF(max77779_fg_config_tex,8,8)
MAX77779_BFF(max77779_fg_config_ten,9,9)
MAX77779_BFF(max77779_fg_config_is,11,11)
MAX77779_BFF(max77779_fg_config_vs,12,12)
MAX77779_BFF(max77779_fg_config_ts,13,13)
MAX77779_BFF(max77779_fg_config_ss,14,14)

/*
 * MAX77779_FG_MixSOC,0x0D,0b10110100100001,0x2d21,Reset_Type:S
 */
#define MAX77779_FG_MixSOC	0x0d

/*
 * MAX77779_FG_AvSOC,0x0E,0b10110011010110,0x2cd6,Reset_Type:S
 */
#define MAX77779_FG_AvSOC	0x0e

/*
 * MAX77779_FG_MiscCfg,0x0F,0b11100001110000,0x3870,Reset_Type:S
 * SACFG[0:2],enBi1,bit3,MixRate[5:5],InitVFG,bit11,FUS[12:4]
 */
#define MAX77779_FG_MiscCfg	0x0f
#define MAX77779_FG_MiscCfg_SACFG_SHIFT	0
#define MAX77779_FG_MiscCfg_SACFG_MASK	(0x3 << 0)
#define MAX77779_FG_MiscCfg_SACFG_CLEAR	(~(0x3 << 0))
#define MAX77779_FG_MiscCfg_enBi1_SHIFT	2
#define MAX77779_FG_MiscCfg_enBi1_MASK	(0x1 << 2)
#define MAX77779_FG_MiscCfg_enBi1_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_MiscCfg_bit3_SHIFT	3
#define MAX77779_FG_MiscCfg_bit3_MASK	(0x1 << 3)
#define MAX77779_FG_MiscCfg_bit3_CLEAR	(~(0x1 << 3))
#define MAX77779_FG_MiscCfg_MixRate_SHIFT	5
#define MAX77779_FG_MiscCfg_MixRate_MASK	(0x1f << 5)
#define MAX77779_FG_MiscCfg_MixRate_CLEAR	(~(0x1f << 5))
#define MAX77779_FG_MiscCfg_InitVFG_SHIFT	10
#define MAX77779_FG_MiscCfg_InitVFG_MASK	(0x1 << 10)
#define MAX77779_FG_MiscCfg_InitVFG_CLEAR	(~(0x1 << 10))
#define MAX77779_FG_MiscCfg_bit11_SHIFT	11
#define MAX77779_FG_MiscCfg_bit11_MASK	(0x1 << 11)
#define MAX77779_FG_MiscCfg_bit11_CLEAR	(~(0x1 << 11))
#define MAX77779_FG_MiscCfg_FUS_SHIFT	12
#define MAX77779_FG_MiscCfg_FUS_MASK	(0xf << 12)
#define MAX77779_FG_MiscCfg_FUS_CLEAR	(~(0xf << 12))
static inline const char *
max77779_fg_misccfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SACFG=%x",
		FIELD2VALUE(MAX77779_SACFG, val));
	i += scnprintf(&buff[i], len - i, " enBi1=%x",
		FIELD2VALUE(MAX77779_enBi1, val));
	i += scnprintf(&buff[i], len - i, " bit3=%x",
		FIELD2VALUE(MAX77779_bit3, val));
	i += scnprintf(&buff[i], len - i, " MixRate=%x",
		FIELD2VALUE(MAX77779_MixRate, val));
	i += scnprintf(&buff[i], len - i, " InitVFG=%x",
		FIELD2VALUE(MAX77779_InitVFG, val));
	i += scnprintf(&buff[i], len - i, " bit11=%x",
		FIELD2VALUE(MAX77779_bit11, val));
	i += scnprintf(&buff[i], len - i, " FUS=%x",
		FIELD2VALUE(MAX77779_FUS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_misccfg_sacfg,1,0)
MAX77779_BFF(max77779_fg_misccfg_enbi1,2,2)
MAX77779_BFF(max77779_fg_misccfg_bit3,3,3)
MAX77779_BFF(max77779_fg_misccfg_mixrate,9,5)
MAX77779_BFF(max77779_fg_misccfg_initvfg,10,10)
MAX77779_BFF(max77779_fg_misccfg_bit11,11,11)
MAX77779_BFF(max77779_fg_misccfg_fus,15,12)

/*
 * MAX77779_FG_FullCapRep,0x10,0b10111011100,0x5dc,Reset_Type:S
 */
#define MAX77779_FG_FullCapRep	0x10

/*
 * MAX77779_FG_TTE,0x11,0b1111111111111111,0xffff,Reset_Type:S
 */
#define MAX77779_FG_TTE	0x11

/*
 * MAX77779_FG_QRTable00,0x12,0b11110000000000,0x3c00,Reset_Type:S
 */
#define MAX77779_FG_QRTable00	0x12

/*
 * MAX77779_FG_FullSocThr,0x13,0b101000000000101,0x5005,Reset_Type:S
 */
#define MAX77779_FG_FullSocThr	0x13

/*
 * MAX77779_FG_RSlow,0x14,0b1010010000,0x290,Reset_Type:S
 */
#define MAX77779_FG_RSlow	0x14

/*
 * MAX77779_FG_Age,0x16,0b110010000000000,0x6400,Reset_Type:S
 */
#define MAX77779_FG_Age	0x16

/*
 * MAX77779_FG_Cycles,0x17,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_Cycles	0x17

/*
 * MAX77779_FG_DesignCap,0x18,0b10111011100,0x5dc,Reset_Type:S
 */
#define MAX77779_FG_DesignCap	0x18

/*
 * MAX77779_FG_AvgVCell,0x19,0b1011010000000000,0xb400,Reset_Type:S
 */
#define MAX77779_FG_AvgVCell	0x19

/*
 * MAX77779_FG_VCell,0x1A,0b1011010000000000,0xb400,Reset_Type:S
 */
#define MAX77779_FG_VCell	0x1a

/*
 * MAX77779_FG_Temp,0x1B,0b1011000000000,0x1600,Reset_Type:S
 */
#define MAX77779_FG_Temp	0x1b

/*
 * MAX77779_FG_Current,0x1C,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_Current	0x1c

/*
 * MAX77779_FG_AvgCurrent,0x1D,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_AvgCurrent	0x1d

/*
 * MAX77779_FG_VEmpty,0x1F,0b1010010101100001,0xa561,Reset_Type:S
 * VR[0:7],VE[7:9]
 */
#define MAX77779_FG_VEmpty	0x1f
#define MAX77779_FG_VEmpty_VR_SHIFT	0
#define MAX77779_FG_VEmpty_VR_MASK	(0x7f << 0)
#define MAX77779_FG_VEmpty_VR_CLEAR	(~(0x7f << 0))
#define MAX77779_FG_VEmpty_VE_SHIFT	7
#define MAX77779_FG_VEmpty_VE_MASK	(0x1ff << 7)
#define MAX77779_FG_VEmpty_VE_CLEAR	(~(0x1ff << 7))
static inline const char *
max77779_fg_vempty_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " VR=%x",
		FIELD2VALUE(MAX77779_VR, val));
	i += scnprintf(&buff[i], len - i, " VE=%x",
		FIELD2VALUE(MAX77779_VE, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_vempty_vr,6,0)
MAX77779_BFF(max77779_fg_vempty_ve,15,7)

/*
 * MAX77779_FG_TTF,0x20,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_TTF	0x20

/*
 * MAX77779_FG_DevName,0x21,0b101000100000000,0x5100,Reset_Type:S
 */
#define MAX77779_FG_DevName	0x21

/*
 * MAX77779_FG_QRTable10,0x22,0b1101110000000,0x1b80,Reset_Type:S
 */
#define MAX77779_FG_QRTable10	0x22

/*
 * MAX77779_FG_FullCapNom,0x23,0b10111011100,0x5dc,Reset_Type:S
 */
#define MAX77779_FG_FullCapNom	0x23

/*
 * MAX77779_FG_FullCap,0x24,0b10111010110,0x5d6,Reset_Type:S
 */
#define MAX77779_FG_FullCap	0x24

/*
 * MAX77779_FG_VFRemCap,0x25,0b1010100101,0x2a5,Reset_Type:S
 */
#define MAX77779_FG_VFRemCap	0x25

/*
 * MAX77779_FG_MixCap,0x26,0b1010100100,0x2a4,Reset_Type:S
 */
#define MAX77779_FG_MixCap	0x26

/*
 * MAX77779_FG_AvCap,0x27,0b1010011110,0x29e,Reset_Type:S
 */
#define MAX77779_FG_AvCap	0x27

/*
 * MAX77779_FG_IChgTerm,0x29,0b1010000000,0x280,Reset_Type:S
 */
#define MAX77779_FG_IChgTerm	0x29

/*
 * MAX77779_FG_FullCapFltr,0x2B,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_FullCapFltr	0x2b

/*
 * MAX77779_FG_QResidual,0x2E,0b00000110,0x6,Reset_Type:S
 */
#define MAX77779_FG_QResidual	0x2e

/*
 * MAX77779_FG_LearnCfg,0x2F,0b100011000000010,0x4602,Reset_Type:S
 * RCx,FiltEmpty,LearnStage[4:3],FCLm[8:2]
 */
#define MAX77779_FG_LearnCfg	0x2f
#define MAX77779_FG_LearnCfg_RCx_SHIFT	2
#define MAX77779_FG_LearnCfg_RCx_MASK	(0x1 << 2)
#define MAX77779_FG_LearnCfg_RCx_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_LearnCfg_FiltEmpty_SHIFT	3
#define MAX77779_FG_LearnCfg_FiltEmpty_MASK	(0x1 << 3)
#define MAX77779_FG_LearnCfg_FiltEmpty_CLEAR	(~(0x1 << 3))
#define MAX77779_FG_LearnCfg_LearnStage_SHIFT	4
#define MAX77779_FG_LearnCfg_LearnStage_MASK	(0x7 << 4)
#define MAX77779_FG_LearnCfg_LearnStage_CLEAR	(~(0x7 << 4))
#define MAX77779_FG_LearnCfg_FCLm_SHIFT	8
#define MAX77779_FG_LearnCfg_FCLm_MASK	(0x3 << 8)
#define MAX77779_FG_LearnCfg_FCLm_CLEAR	(~(0x3 << 8))
static inline const char *
max77779_fg_learncfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " RCx=%x",
		FIELD2VALUE(MAX77779_RCx, val));
	i += scnprintf(&buff[i], len - i, " FiltEmpty=%x",
		FIELD2VALUE(MAX77779_FiltEmpty, val));
	i += scnprintf(&buff[i], len - i, " LearnStage=%x",
		FIELD2VALUE(MAX77779_LearnStage, val));
	i += scnprintf(&buff[i], len - i, " FCLm=%x",
		FIELD2VALUE(MAX77779_FCLm, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_learncfg_rcx,2,2)
MAX77779_BFF(max77779_fg_learncfg_filtempty,3,3)
MAX77779_BFF(max77779_fg_learncfg_learnstage,6,4)
MAX77779_BFF(max77779_fg_learncfg_fclm,9,8)

/*
 * MAX77779_FG_QRTable20,0x32,0b101100000100,0xb04,Reset_Type:S
 */
#define MAX77779_FG_QRTable20	0x32

/*
 * MAX77779_FG_DieTemp,0x34,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_DieTemp	0x34

/*
 * MAX77779_FG_AvgTA,0x35,0b1011000000000,0x1600,Reset_Type:S
 */
#define MAX77779_FG_AvgTA	0x35

/*
 * MAX77779_FG_VFOCV,0x37,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_VFOCV	0x37

/*
 * MAX77779_FG_VFSOC,0x39,0b10110100101101,0x2d2d,Reset_Type:S
 */
#define MAX77779_FG_VFSOC	0x39

/*
 * MAX77779_FG_SOCHold,0x3A,0b111000000000010,0x7002,Reset_Type:S
 * EmptySocHold[0:5],EmptyVoltHold[5:7],HoldEn99,InputSoCHold,IbattNoLoad[14:2]
 */
#define MAX77779_FG_SOCHold	0x3a
#define MAX77779_FG_SOCHold_EmptySocHold_SHIFT	0
#define MAX77779_FG_SOCHold_EmptySocHold_MASK	(0x1f << 0)
#define MAX77779_FG_SOCHold_EmptySocHold_CLEAR	(~(0x1f << 0))
#define MAX77779_FG_SOCHold_EmptyVoltHold_SHIFT	5
#define MAX77779_FG_SOCHold_EmptyVoltHold_MASK	(0x7f << 5)
#define MAX77779_FG_SOCHold_EmptyVoltHold_CLEAR	(~(0x7f << 5))
#define MAX77779_FG_SOCHold_HoldEn99_SHIFT	12
#define MAX77779_FG_SOCHold_HoldEn99_MASK	(0x1 << 12)
#define MAX77779_FG_SOCHold_HoldEn99_CLEAR	(~(0x1 << 12))
#define MAX77779_FG_SOCHold_InputSoCHold_SHIFT	13
#define MAX77779_FG_SOCHold_InputSoCHold_MASK	(0x1 << 13)
#define MAX77779_FG_SOCHold_InputSoCHold_CLEAR	(~(0x1 << 13))
#define MAX77779_FG_SOCHold_IbattNoLoad_SHIFT	14
#define MAX77779_FG_SOCHold_IbattNoLoad_MASK	(0x3 << 14)
#define MAX77779_FG_SOCHold_IbattNoLoad_CLEAR	(~(0x3 << 14))
static inline const char *
max77779_fg_sochold_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " EmptySocHold=%x",
		FIELD2VALUE(MAX77779_EmptySocHold, val));
	i += scnprintf(&buff[i], len - i, " EmptyVoltHold=%x",
		FIELD2VALUE(MAX77779_EmptyVoltHold, val));
	i += scnprintf(&buff[i], len - i, " HoldEn99=%x",
		FIELD2VALUE(MAX77779_HoldEn99, val));
	i += scnprintf(&buff[i], len - i, " InputSoCHold=%x",
		FIELD2VALUE(MAX77779_InputSoCHold, val));
	i += scnprintf(&buff[i], len - i, " IbattNoLoad=%x",
		FIELD2VALUE(MAX77779_IbattNoLoad, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_sochold_emptysochold,4,0)
MAX77779_BFF(max77779_fg_sochold_emptyvolthold,11,5)
MAX77779_BFF(max77779_fg_sochold_holden99,12,12)
MAX77779_BFF(max77779_fg_sochold_inputsochold,13,13)
MAX77779_BFF(max77779_fg_sochold_ibattnoload,15,14)

/*
 * MAX77779_FG_FStat,0x3D,0b100000000,0x100,Reset_Type:S
 * DNR,LDMdl,Timer_Start,RelDt2,FQ,EDet,RelDt,DeBn
 */
#define MAX77779_FG_FStat	0x3d
#define MAX77779_FG_FStat_DNR_SHIFT	0
#define MAX77779_FG_FStat_DNR_MASK	(0x1 << 0)
#define MAX77779_FG_FStat_DNR_CLEAR	(~(0x1 << 0))
#define MAX77779_FG_FStat_LDMdl_SHIFT	1
#define MAX77779_FG_FStat_LDMdl_MASK	(0x1 << 1)
#define MAX77779_FG_FStat_LDMdl_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_FStat_Timer_Start_SHIFT	5
#define MAX77779_FG_FStat_Timer_Start_MASK	(0x1 << 5)
#define MAX77779_FG_FStat_Timer_Start_CLEAR	(~(0x1 << 5))
#define MAX77779_FG_FStat_RelDt2_SHIFT	6
#define MAX77779_FG_FStat_RelDt2_MASK	(0x1 << 6)
#define MAX77779_FG_FStat_RelDt2_CLEAR	(~(0x1 << 6))
#define MAX77779_FG_FStat_FQ_SHIFT	7
#define MAX77779_FG_FStat_FQ_MASK	(0x1 << 7)
#define MAX77779_FG_FStat_FQ_CLEAR	(~(0x1 << 7))
#define MAX77779_FG_FStat_EDet_SHIFT	8
#define MAX77779_FG_FStat_EDet_MASK	(0x1 << 8)
#define MAX77779_FG_FStat_EDet_CLEAR	(~(0x1 << 8))
#define MAX77779_FG_FStat_RelDt_SHIFT	9
#define MAX77779_FG_FStat_RelDt_MASK	(0x1 << 9)
#define MAX77779_FG_FStat_RelDt_CLEAR	(~(0x1 << 9))
#define MAX77779_FG_FStat_DeBn_SHIFT	12
#define MAX77779_FG_FStat_DeBn_MASK	(0x1 << 12)
#define MAX77779_FG_FStat_DeBn_CLEAR	(~(0x1 << 12))
static inline const char *
max77779_fg_fstat_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DNR=%x",
		FIELD2VALUE(MAX77779_DNR, val));
	i += scnprintf(&buff[i], len - i, " LDMdl=%x",
		FIELD2VALUE(MAX77779_LDMdl, val));
	i += scnprintf(&buff[i], len - i, " Timer_Start=%x",
		FIELD2VALUE(MAX77779_Timer_Start, val));
	i += scnprintf(&buff[i], len - i, " RelDt2=%x",
		FIELD2VALUE(MAX77779_RelDt2, val));
	i += scnprintf(&buff[i], len - i, " FQ=%x",
		FIELD2VALUE(MAX77779_FQ, val));
	i += scnprintf(&buff[i], len - i, " EDet=%x",
		FIELD2VALUE(MAX77779_EDet, val));
	i += scnprintf(&buff[i], len - i, " RelDt=%x",
		FIELD2VALUE(MAX77779_RelDt, val));
	i += scnprintf(&buff[i], len - i, " DeBn=%x",
		FIELD2VALUE(MAX77779_DeBn, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_fstat_dnr,0,0)
MAX77779_BFF(max77779_fg_fstat_ldmdl,1,1)
MAX77779_BFF(max77779_fg_fstat_timer_start,5,5)
MAX77779_BFF(max77779_fg_fstat_reldt2,6,6)
MAX77779_BFF(max77779_fg_fstat_fq,7,7)
MAX77779_BFF(max77779_fg_fstat_edet,8,8)
MAX77779_BFF(max77779_fg_fstat_reldt,9,9)
MAX77779_BFF(max77779_fg_fstat_debn,12,12)

/*
 * MAX77779_FG_InputSoCHoldSts,0x3E,0b10001000000111100000,0x881e0,Reset_Type:S
 * InputSoCHoldNow,FDet_triggered_flag,ISHA,InputSoCHoldTO[3:7],InputsHistory[10:4],
 * InputsSts
 */
#define MAX77779_FG_InputSoCHoldSts	0x3e
#define MAX77779_FG_InputSoCHoldSts_InputSoCHoldNow_SHIFT	0
#define MAX77779_FG_InputSoCHoldSts_InputSoCHoldNow_MASK	(0x1 << 0)
#define MAX77779_FG_InputSoCHoldSts_InputSoCHoldNow_CLEAR	(~(0x1 << 0))
#define MAX77779_FG_InputSoCHoldSts_FDet_triggered_flag_SHIFT	1
#define MAX77779_FG_InputSoCHoldSts_FDet_triggered_flag_MASK	(0x1 << 1)
#define MAX77779_FG_InputSoCHoldSts_FDet_triggered_flag_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_InputSoCHoldSts_ISHA_SHIFT	2
#define MAX77779_FG_InputSoCHoldSts_ISHA_MASK	(0x1 << 2)
#define MAX77779_FG_InputSoCHoldSts_ISHA_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_InputSoCHoldSts_InputSoCHoldTO_SHIFT	3
#define MAX77779_FG_InputSoCHoldSts_InputSoCHoldTO_MASK	(0x7f << 3)
#define MAX77779_FG_InputSoCHoldSts_InputSoCHoldTO_CLEAR	(~(0x7f << 3))
#define MAX77779_FG_InputSoCHoldSts_InputsHistory_SHIFT	10
#define MAX77779_FG_InputSoCHoldSts_InputsHistory_MASK	(0xf << 10)
#define MAX77779_FG_InputSoCHoldSts_InputsHistory_CLEAR	(~(0xf << 10))
#define MAX77779_FG_InputSoCHoldSts_InputsSts_SHIFT	14
#define MAX77779_FG_InputSoCHoldSts_InputsSts_MASK	(0x1 << 14)
#define MAX77779_FG_InputSoCHoldSts_InputsSts_CLEAR	(~(0x1 << 14))
static inline const char *
max77779_fg_inputsocholdsts_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " InputSoCHoldNow=%x",
		FIELD2VALUE(MAX77779_InputSoCHoldNow, val));
	i += scnprintf(&buff[i], len - i, " FDet_triggered_flag=%x",
		FIELD2VALUE(MAX77779_FDet_triggered_flag, val));
	i += scnprintf(&buff[i], len - i, " ISHA=%x",
		FIELD2VALUE(MAX77779_ISHA, val));
	i += scnprintf(&buff[i], len - i, " InputSoCHoldTO=%x",
		FIELD2VALUE(MAX77779_InputSoCHoldTO, val));
	i += scnprintf(&buff[i], len - i, " InputsHistory=%x",
		FIELD2VALUE(MAX77779_InputsHistory, val));
	i += scnprintf(&buff[i], len - i, " InputsSts=%x",
		FIELD2VALUE(MAX77779_InputsSts, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_inputsocholdsts_inputsocholdnow,0,0)
MAX77779_BFF(max77779_fg_inputsocholdsts_fdet_triggered_flag,1,1)
MAX77779_BFF(max77779_fg_inputsocholdsts_isha,2,2)
MAX77779_BFF(max77779_fg_inputsocholdsts_inputsocholdto,9,3)
MAX77779_BFF(max77779_fg_inputsocholdsts_inputshistory,13,10)
MAX77779_BFF(max77779_fg_inputsocholdsts_inputssts,14,14)

/*
 * MAX77779_FG_Timer,0x3F,0b00100000,0x20,Reset_Type:S
 */
#define MAX77779_FG_Timer	0x3f

/*
 * MAX77779_FG_QRTable30,0x42,0b100010000101,0x885,Reset_Type:S
 */
#define MAX77779_FG_QRTable30	0x42

/*
 * MAX77779_FG_dQAcc,0x45,0b101110110,0x176,Reset_Type:S
 */
#define MAX77779_FG_dQAcc	0x45

/*
 * MAX77779_FG_dPAcc,0x46,0b1100100000,0x320,Reset_Type:S
 */
#define MAX77779_FG_dPAcc	0x46

/*
 * MAX77779_FG_RlxSOC,0x47,0b1000000000010,0x1002,Reset_Type:S
 */
#define MAX77779_FG_RlxSOC	0x47

/*
 * MAX77779_FG_VFSOC0,0x48,0b10110100101101,0x2d2d,Reset_Type:S
 */
#define MAX77779_FG_VFSOC0	0x48

/*
 * MAX77779_FG_QH0,0x4C,0b1111111111111111,0xffff,Reset_Type:S
 */
#define MAX77779_FG_QH0	0x4c

/*
 * MAX77779_FG_QH,0x4D,0b1111111111111111,0xffff,Reset_Type:S
 */
#define MAX77779_FG_QH	0x4d

/*
 * MAX77779_FG_QL,0x4E,0b1111110011010111,0xfcd7,Reset_Type:S
 */
#define MAX77779_FG_QL	0x4e

/*
 * MAX77779_FG_VSys,0x52,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_VSys	0x52

/*
 * MAX77779_FG_CCCurrent,0x54,0b1010000000,0x280,Reset_Type:S
 */
#define MAX77779_FG_CCCurrent	0x54

/*
 * MAX77779_FG_CurrentOffsetCal,0x62,0b00000000,0x0,Reset_Type:S
 * FDET_Done,StartAccumulating,has256CyclesReached,LearningOffsetSts[3:2],
 * OffsetQCDone,LearnedOffsetValue[6:10]
 */
#define MAX77779_FG_CurrentOffsetCal	0x62
#define MAX77779_FG_CurrentOffsetCal_FDET_Done_SHIFT	0
#define MAX77779_FG_CurrentOffsetCal_FDET_Done_MASK	(0x1 << 0)
#define MAX77779_FG_CurrentOffsetCal_FDET_Done_CLEAR	(~(0x1 << 0))
#define MAX77779_FG_CurrentOffsetCal_StartAccumulating_SHIFT	1
#define MAX77779_FG_CurrentOffsetCal_StartAccumulating_MASK	(0x1 << 1)
#define MAX77779_FG_CurrentOffsetCal_StartAccumulating_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_CurrentOffsetCal_has256CyclesReached_SHIFT	2
#define MAX77779_FG_CurrentOffsetCal_has256CyclesReached_MASK	(0x1 << 2)
#define MAX77779_FG_CurrentOffsetCal_has256CyclesReached_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_CurrentOffsetCal_LearningOffsetSts_SHIFT	3
#define MAX77779_FG_CurrentOffsetCal_LearningOffsetSts_MASK	(0x3 << 3)
#define MAX77779_FG_CurrentOffsetCal_LearningOffsetSts_CLEAR	(~(0x3 << 3))
#define MAX77779_FG_CurrentOffsetCal_OffsetQCDone_SHIFT	5
#define MAX77779_FG_CurrentOffsetCal_OffsetQCDone_MASK	(0x1 << 5)
#define MAX77779_FG_CurrentOffsetCal_OffsetQCDone_CLEAR	(~(0x1 << 5))
#define MAX77779_FG_CurrentOffsetCal_LearnedOffsetValue_SHIFT	6
#define MAX77779_FG_CurrentOffsetCal_LearnedOffsetValue_MASK	(0x3ff << 6)
#define MAX77779_FG_CurrentOffsetCal_LearnedOffsetValue_CLEAR	(~(0x3ff << 6))
static inline const char *
max77779_fg_currentoffsetcal_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " FDET_Done=%x",
		FIELD2VALUE(MAX77779_FDET_Done, val));
	i += scnprintf(&buff[i], len - i, " StartAccumulating=%x",
		FIELD2VALUE(MAX77779_StartAccumulating, val));
	i += scnprintf(&buff[i], len - i, " has256CyclesReached=%x",
		FIELD2VALUE(MAX77779_has256CyclesReached, val));
	i += scnprintf(&buff[i], len - i, " LearningOffsetSts=%x",
		FIELD2VALUE(MAX77779_LearningOffsetSts, val));
	i += scnprintf(&buff[i], len - i, " OffsetQCDone=%x",
		FIELD2VALUE(MAX77779_OffsetQCDone, val));
	i += scnprintf(&buff[i], len - i, " LearnedOffsetValue=%x",
		FIELD2VALUE(MAX77779_LearnedOffsetValue, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_currentoffsetcal_fdet_done,0,0)
MAX77779_BFF(max77779_fg_currentoffsetcal_startaccumulating,1,1)
MAX77779_BFF(max77779_fg_currentoffsetcal_has256cyclesreached,2,2)
MAX77779_BFF(max77779_fg_currentoffsetcal_learningoffsetsts,4,3)
MAX77779_BFF(max77779_fg_currentoffsetcal_offsetqcdone,5,5)
MAX77779_BFF(max77779_fg_currentoffsetcal_learnedoffsetvalue,15,6)

/*
 * MAX77779_FG_AccumulatedCurrent,0x63,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_AccumulatedCurrent	0x63

/*
 * MAX77779_FG_DebugInfo,0x6F,0b100100111111001,0x49f9,Reset_Type:S
 * PatchUse[0:2]
 */
#define MAX77779_FG_DebugInfo	0x6f
#define MAX77779_FG_DebugInfo_PatchUse_SHIFT	0
#define MAX77779_FG_DebugInfo_PatchUse_MASK	(0x3 << 0)
#define MAX77779_FG_DebugInfo_PatchUse_CLEAR	(~(0x3 << 0))
static inline const char *
max77779_fg_debuginfo_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " PatchUse=%x",
		FIELD2VALUE(MAX77779_PatchUse, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_debuginfo_patchuse,1,0)

/*
 * MAX77779_FG_OCV0,0x80,0b1001011101100000,0x9760,Reset_Type:S
 */
#define MAX77779_FG_OCV0	0x80

/*
 * MAX77779_FG_OCV1,0x81,0b1010010100010000,0xa510,Reset_Type:S
 */
#define MAX77779_FG_OCV1	0x81

/*
 * MAX77779_FG_OCV2,0x82,0b1011000100000000,0xb100,Reset_Type:S
 */
#define MAX77779_FG_OCV2	0x82

/*
 * MAX77779_FG_OCV3,0x83,0b1011011000000000,0xb600,Reset_Type:S
 */
#define MAX77779_FG_OCV3	0x83

/*
 * MAX77779_FG_OCV4,0x84,0b1011011110100000,0xb7a0,Reset_Type:S
 */
#define MAX77779_FG_OCV4	0x84

/*
 * MAX77779_FG_OCV5,0x85,0b1011100100000000,0xb900,Reset_Type:S
 */
#define MAX77779_FG_OCV5	0x85

/*
 * MAX77779_FG_OCV6,0x86,0b1011101001110000,0xba70,Reset_Type:S
 */
#define MAX77779_FG_OCV6	0x86

/*
 * MAX77779_FG_OCV7,0x87,0b1011110001110000,0xbc70,Reset_Type:S
 */
#define MAX77779_FG_OCV7	0x87

/*
 * MAX77779_FG_OCV8,0x88,0b1011110111100000,0xbde0,Reset_Type:S
 */
#define MAX77779_FG_OCV8	0x88

/*
 * MAX77779_FG_OCV9,0x89,0b1011111111000000,0xbfc0,Reset_Type:S
 */
#define MAX77779_FG_OCV9	0x89

/*
 * MAX77779_FG_OCV10,0x8A,0b1100001001010000,0xc250,Reset_Type:S
 */
#define MAX77779_FG_OCV10	0x8a

/*
 * MAX77779_FG_OCV11,0x8B,0b1100010100010000,0xc510,Reset_Type:S
 */
#define MAX77779_FG_OCV11	0x8b

/*
 * MAX77779_FG_OCV12,0x8C,0b1100100110010000,0xc990,Reset_Type:S
 */
#define MAX77779_FG_OCV12	0x8c

/*
 * MAX77779_FG_OCV13,0x8D,0b1100111010100000,0xcea0,Reset_Type:S
 */
#define MAX77779_FG_OCV13	0x8d

/*
 * MAX77779_FG_OCV14,0x8E,0b1101000001000000,0xd040,Reset_Type:S
 */
#define MAX77779_FG_OCV14	0x8e

/*
 * MAX77779_FG_OCV15,0x8F,0b1101011101010000,0xd750,Reset_Type:S
 */
#define MAX77779_FG_OCV15	0x8f

/*
 * MAX77779_FG_CAP0,0x90,0b01100000,0x60,Reset_Type:S
 */
#define MAX77779_FG_CAP0	0x90

/*
 * MAX77779_FG_CAP1,0x91,0b100100000,0x120,Reset_Type:S
 */
#define MAX77779_FG_CAP1	0x91

/*
 * MAX77779_FG_CAP2,0x92,0b1001000000,0x240,Reset_Type:S
 */
#define MAX77779_FG_CAP2	0x92

/*
 * MAX77779_FG_CAP3,0x93,0b110110000000,0xd80,Reset_Type:S
 */
#define MAX77779_FG_CAP3	0x93

/*
 * MAX77779_FG_CAP4,0x94,0b100010110000,0x8b0,Reset_Type:S
 */
#define MAX77779_FG_CAP4	0x94

/*
 * MAX77779_FG_CAP5,0x95,0b10110010000,0x590,Reset_Type:S
 */
#define MAX77779_FG_CAP5	0x95

/*
 * MAX77779_FG_CAP6,0x96,0b1001000000000,0x1200,Reset_Type:S
 */
#define MAX77779_FG_CAP6	0x96

/*
 * MAX77779_FG_CAP7,0x97,0b11001000010000,0x3210,Reset_Type:S
 */
#define MAX77779_FG_CAP7	0x97

/*
 * MAX77779_FG_CAP8,0x98,0b111011100000,0xee0,Reset_Type:S
 */
#define MAX77779_FG_CAP8	0x98

/*
 * MAX77779_FG_CAP9,0x99,0b101001000000,0xa40,Reset_Type:S
 */
#define MAX77779_FG_CAP9	0x99

/*
 * MAX77779_FG_CAP10,0x9A,0b100101010000,0x950,Reset_Type:S
 */
#define MAX77779_FG_CAP10	0x9a

/*
 * MAX77779_FG_CAP11,0x9B,0b100011100000,0x8e0,Reset_Type:S
 */
#define MAX77779_FG_CAP11	0x9b

/*
 * MAX77779_FG_CAP12,0x9C,0b100000000000,0x800,Reset_Type:S
 */
#define MAX77779_FG_CAP12	0x9c

/*
 * MAX77779_FG_CAP13,0x9D,0b11110000000,0x780,Reset_Type:S
 */
#define MAX77779_FG_CAP13	0x9d

/*
 * MAX77779_FG_CAP14,0x9E,0b11010110000,0x6b0,Reset_Type:S
 */
#define MAX77779_FG_CAP14	0x9e

/*
 * MAX77779_FG_CAP15,0x9F,0b11010110000,0x6b0,Reset_Type:S
 */
#define MAX77779_FG_CAP15	0x9f

/*
 * MAX77779_FG_CGain,0xA0,0b100000000000000,0x4000,Reset_Type:S
 * C_Offset[0:6],C_Gain[6:10]
 */
#define MAX77779_FG_CGain	0xa0
#define MAX77779_FG_CGain_C_Offset_SHIFT	0
#define MAX77779_FG_CGain_C_Offset_MASK	(0x3f << 0)
#define MAX77779_FG_CGain_C_Offset_CLEAR	(~(0x3f << 0))
#define MAX77779_FG_CGain_C_Gain_SHIFT	6
#define MAX77779_FG_CGain_C_Gain_MASK	(0x3ff << 6)
#define MAX77779_FG_CGain_C_Gain_CLEAR	(~(0x3ff << 6))
static inline const char *
max77779_fg_cgain_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " C_Offset=%x",
		FIELD2VALUE(MAX77779_C_Offset, val));
	i += scnprintf(&buff[i], len - i, " C_Gain=%x",
		FIELD2VALUE(MAX77779_C_Gain, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_cgain_c_offset,5,0)
MAX77779_BFF(max77779_fg_cgain_c_gain,15,6)

/*
 * MAX77779_FG_ModelCfg,0xA3,0b00000000,0x0,Reset_Type:S
 * ModelID[4:4],VChg,Refresh
 */
#define MAX77779_FG_ModelCfg	0xa3
#define MAX77779_FG_ModelCfg_ModelID_SHIFT	4
#define MAX77779_FG_ModelCfg_ModelID_MASK	(0xf << 4)
#define MAX77779_FG_ModelCfg_ModelID_CLEAR	(~(0xf << 4))
#define MAX77779_FG_ModelCfg_VChg_SHIFT	10
#define MAX77779_FG_ModelCfg_VChg_MASK	(0x1 << 10)
#define MAX77779_FG_ModelCfg_VChg_CLEAR	(~(0x1 << 10))
#define MAX77779_FG_ModelCfg_Refresh_SHIFT	15
#define MAX77779_FG_ModelCfg_Refresh_MASK	(0x1 << 15)
#define MAX77779_FG_ModelCfg_Refresh_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_fg_modelcfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " ModelID=%x",
		FIELD2VALUE(MAX77779_ModelID, val));
	i += scnprintf(&buff[i], len - i, " VChg=%x",
		FIELD2VALUE(MAX77779_VChg, val));
	i += scnprintf(&buff[i], len - i, " Refresh=%x",
		FIELD2VALUE(MAX77779_Refresh, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_modelcfg_modelid,7,4)
MAX77779_BFF(max77779_fg_modelcfg_vchg,10,10)
MAX77779_BFF(max77779_fg_modelcfg_refresh,15,15)

/*
 * MAX77779_FG_Config2,0xAB,0b00000000,0x0,Reset_Type:S
 * Reset_VFOCV,TAlrtEn,dSOCen,FCNBlock,NRld,LDMdl
 */
#define MAX77779_FG_Config2	0xab
#define MAX77779_FG_Config2_Reset_VFOCV_SHIFT	0
#define MAX77779_FG_Config2_Reset_VFOCV_MASK	(0x1 << 0)
#define MAX77779_FG_Config2_Reset_VFOCV_CLEAR	(~(0x1 << 0))
#define MAX77779_FG_Config2_TAlrtEn_SHIFT	6
#define MAX77779_FG_Config2_TAlrtEn_MASK	(0x1 << 6)
#define MAX77779_FG_Config2_TAlrtEn_CLEAR	(~(0x1 << 6))
#define MAX77779_FG_Config2_dSOCen_SHIFT	7
#define MAX77779_FG_Config2_dSOCen_MASK	(0x1 << 7)
#define MAX77779_FG_Config2_dSOCen_CLEAR	(~(0x1 << 7))
#define MAX77779_FG_Config2_FCNBlock_SHIFT	11
#define MAX77779_FG_Config2_FCNBlock_MASK	(0x1 << 11)
#define MAX77779_FG_Config2_FCNBlock_CLEAR	(~(0x1 << 11))
#define MAX77779_FG_Config2_NRld_SHIFT	14
#define MAX77779_FG_Config2_NRld_MASK	(0x1 << 14)
#define MAX77779_FG_Config2_NRld_CLEAR	(~(0x1 << 14))
#define MAX77779_FG_Config2_LDMdl_SHIFT	15
#define MAX77779_FG_Config2_LDMdl_MASK	(0x1 << 15)
#define MAX77779_FG_Config2_LDMdl_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_fg_config2_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " Reset_VFOCV=%x",
		FIELD2VALUE(MAX77779_Reset_VFOCV, val));
	i += scnprintf(&buff[i], len - i, " TAlrtEn=%x",
		FIELD2VALUE(MAX77779_TAlrtEn, val));
	i += scnprintf(&buff[i], len - i, " dSOCen=%x",
		FIELD2VALUE(MAX77779_dSOCen, val));
	i += scnprintf(&buff[i], len - i, " FCNBlock=%x",
		FIELD2VALUE(MAX77779_FCNBlock, val));
	i += scnprintf(&buff[i], len - i, " NRld=%x",
		FIELD2VALUE(MAX77779_NRld, val));
	i += scnprintf(&buff[i], len - i, " LDMdl=%x",
		FIELD2VALUE(MAX77779_LDMdl, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_config2_reset_vfocv,0,0)
MAX77779_BFF(max77779_fg_config2_talrten,6,6)
MAX77779_BFF(max77779_fg_config2_dsocen,7,7)
MAX77779_BFF(max77779_fg_config2_fcnblock,11,11)
MAX77779_BFF(max77779_fg_config2_nrld,14,14)
MAX77779_BFF(max77779_fg_config2_ldmdl,15,15)

/*
 * MAX77779_FG_Status2,0xB0,0b00000000,0x0,Reset_Type:S
 * Hib,FullDet
 */
#define MAX77779_FG_Status2	0xb0
#define MAX77779_FG_Status2_Hib_SHIFT	1
#define MAX77779_FG_Status2_Hib_MASK	(0x1 << 1)
#define MAX77779_FG_Status2_Hib_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_Status2_FullDet_SHIFT	5
#define MAX77779_FG_Status2_FullDet_MASK	(0x1 << 5)
#define MAX77779_FG_Status2_FullDet_CLEAR	(~(0x1 << 5))
static inline const char *
max77779_fg_status2_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " Hib=%x",
		FIELD2VALUE(MAX77779_Hib, val));
	i += scnprintf(&buff[i], len - i, " FullDet=%x",
		FIELD2VALUE(MAX77779_FullDet, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_status2_hib,1,1)
MAX77779_BFF(max77779_fg_status2_fulldet,5,5)

/*
 * MAX77779_FG_VRipple,0xB2,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_VRipple	0xb2

/*
 * MAX77779_FG_IAlrtTh,0xB4,0b111111110000000,0x7f80,Reset_Type:S
 * IMIN[0:8],IMAX[8:8]
 */
#define MAX77779_FG_IAlrtTh	0xb4
#define MAX77779_FG_IAlrtTh_IMIN_SHIFT	0
#define MAX77779_FG_IAlrtTh_IMIN_MASK	(0xff << 0)
#define MAX77779_FG_IAlrtTh_IMIN_CLEAR	(~(0xff << 0))
#define MAX77779_FG_IAlrtTh_IMAX_SHIFT	8
#define MAX77779_FG_IAlrtTh_IMAX_MASK	(0xff << 8)
#define MAX77779_FG_IAlrtTh_IMAX_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_ialrtth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " IMIN=%x",
		FIELD2VALUE(MAX77779_IMIN, val));
	i += scnprintf(&buff[i], len - i, " IMAX=%x",
		FIELD2VALUE(MAX77779_IMAX, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_ialrtth_imin,7,0)
MAX77779_BFF(max77779_fg_ialrtth_imax,15,8)

/*
 * MAX77779_FG_ic_info,0xBA,0b10000000001,0x401,Reset_Type:S
 * TestProgramRev[8:8]
 */
#define MAX77779_FG_ic_info	0xba
#define MAX77779_FG_ic_info_TestProgramRev_SHIFT	8
#define MAX77779_FG_ic_info_TestProgramRev_MASK	(0xff << 8)
#define MAX77779_FG_ic_info_TestProgramRev_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_ic_info_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TestProgramRev=%x",
		FIELD2VALUE(MAX77779_TestProgramRev, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_ic_info_testprogramrev,15,8)

/*
 * MAX77779_FG_TimerH,0xBE,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_TimerH	0xbe

/*
 * MAX77779_FG_Thm1,0xD0,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_Thm1	0xd0

/*
 * MAX77779_FG_Thm2,0xD1,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_Thm2	0xd1

/*
 * MAX77779_FG_Thm3,0xD2,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_Thm3	0xd2

/*
 * MAX77779_FG_BattID1,0xD3,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_BattID1	0xd3

/*
 * MAX77779_FG_BattID2,0xD4,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_BattID2	0xd4

/*
 * MAX77779_FG_IChgin,0xD5,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_IChgin	0xd5

/*
 * MAX77779_FG_IWCin,0xD6,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_IWCin	0xd6

/*
 * MAX77779_FG_ChgIn,0xD7,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_ChgIn	0xd7

/*
 * MAX77779_FG_WcIn,0xD8,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_WcIn	0xd8

/*
 * MAX77779_FG_TrimVbattGain,0xD9,0b00011000,0x18,Reset_Type:S
 * vbattgtrim[0:12]
 */
#define MAX77779_FG_TrimVbattGain	0xd9
#define MAX77779_FG_TrimVbattGain_vbattgtrim_SHIFT	0
#define MAX77779_FG_TrimVbattGain_vbattgtrim_MASK	(0xfff << 0)
#define MAX77779_FG_TrimVbattGain_vbattgtrim_CLEAR	(~(0xfff << 0))
static inline const char *
max77779_fg_trimvbattgain_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " vbattgtrim=%x",
		FIELD2VALUE(MAX77779_vbattgtrim, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_trimvbattgain_vbattgtrim,11,0)

/*
 * MAX77779_FG_TrimIbattGain,0xDA,0b01010111,0x57,Reset_Type:S
 * ibattgtrim[0:12]
 */
#define MAX77779_FG_TrimIbattGain	0xda
#define MAX77779_FG_TrimIbattGain_ibattgtrim_SHIFT	0
#define MAX77779_FG_TrimIbattGain_ibattgtrim_MASK	(0xfff << 0)
#define MAX77779_FG_TrimIbattGain_ibattgtrim_CLEAR	(~(0xfff << 0))
static inline const char *
max77779_fg_trimibattgain_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " ibattgtrim=%x",
		FIELD2VALUE(MAX77779_ibattgtrim, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_trimibattgain_ibattgtrim,11,0)

/*
 * MAX77779_FG_TrimBattOffset,0xDB,0b1100011110101,0x18f5,Reset_Type:S
 * ibattotrim[0:8],vbattotrim[8:8]
 */
#define MAX77779_FG_TrimBattOffset	0xdb
#define MAX77779_FG_TrimBattOffset_ibattotrim_SHIFT	0
#define MAX77779_FG_TrimBattOffset_ibattotrim_MASK	(0xff << 0)
#define MAX77779_FG_TrimBattOffset_ibattotrim_CLEAR	(~(0xff << 0))
#define MAX77779_FG_TrimBattOffset_vbattotrim_SHIFT	8
#define MAX77779_FG_TrimBattOffset_vbattotrim_MASK	(0xff << 8)
#define MAX77779_FG_TrimBattOffset_vbattotrim_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_trimbattoffset_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " ibattotrim=%x",
		FIELD2VALUE(MAX77779_ibattotrim, val));
	i += scnprintf(&buff[i], len - i, " vbattotrim=%x",
		FIELD2VALUE(MAX77779_vbattotrim, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_trimbattoffset_ibattotrim,7,0)
MAX77779_BFF(max77779_fg_trimbattoffset_vbattotrim,15,8)

/*******************************************************
 * Section: FG_FUNC 0xE0 16
 */

/*
 * MAX77779_FG_FG_INT_STS,0x00,0b00000010,0x2,Reset_Type:F
 * PONR,Imn,Bst,Imx,dSOCi,Vmn,Tmn,Smn,Bi,Vmx,Tmx,Smx,Br
 */
#define MAX77779_FG_FG_INT_STS	0xe0
#define MAX77779_FG_FG_INT_STS_PONR_SHIFT	1
#define MAX77779_FG_FG_INT_STS_PONR_MASK	(0x1 << 1)
#define MAX77779_FG_FG_INT_STS_PONR_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_FG_INT_STS_Imn_SHIFT	2
#define MAX77779_FG_FG_INT_STS_Imn_MASK	(0x1 << 2)
#define MAX77779_FG_FG_INT_STS_Imn_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_FG_INT_STS_Bst_SHIFT	3
#define MAX77779_FG_FG_INT_STS_Bst_MASK	(0x1 << 3)
#define MAX77779_FG_FG_INT_STS_Bst_CLEAR	(~(0x1 << 3))
#define MAX77779_FG_FG_INT_STS_Imx_SHIFT	6
#define MAX77779_FG_FG_INT_STS_Imx_MASK	(0x1 << 6)
#define MAX77779_FG_FG_INT_STS_Imx_CLEAR	(~(0x1 << 6))
#define MAX77779_FG_FG_INT_STS_dSOCi_SHIFT	7
#define MAX77779_FG_FG_INT_STS_dSOCi_MASK	(0x1 << 7)
#define MAX77779_FG_FG_INT_STS_dSOCi_CLEAR	(~(0x1 << 7))
#define MAX77779_FG_FG_INT_STS_Vmn_SHIFT	8
#define MAX77779_FG_FG_INT_STS_Vmn_MASK	(0x1 << 8)
#define MAX77779_FG_FG_INT_STS_Vmn_CLEAR	(~(0x1 << 8))
#define MAX77779_FG_FG_INT_STS_Tmn_SHIFT	9
#define MAX77779_FG_FG_INT_STS_Tmn_MASK	(0x1 << 9)
#define MAX77779_FG_FG_INT_STS_Tmn_CLEAR	(~(0x1 << 9))
#define MAX77779_FG_FG_INT_STS_Smn_SHIFT	10
#define MAX77779_FG_FG_INT_STS_Smn_MASK	(0x1 << 10)
#define MAX77779_FG_FG_INT_STS_Smn_CLEAR	(~(0x1 << 10))
#define MAX77779_FG_FG_INT_STS_Bi_SHIFT	11
#define MAX77779_FG_FG_INT_STS_Bi_MASK	(0x1 << 11)
#define MAX77779_FG_FG_INT_STS_Bi_CLEAR	(~(0x1 << 11))
#define MAX77779_FG_FG_INT_STS_Vmx_SHIFT	12
#define MAX77779_FG_FG_INT_STS_Vmx_MASK	(0x1 << 12)
#define MAX77779_FG_FG_INT_STS_Vmx_CLEAR	(~(0x1 << 12))
#define MAX77779_FG_FG_INT_STS_Tmx_SHIFT	13
#define MAX77779_FG_FG_INT_STS_Tmx_MASK	(0x1 << 13)
#define MAX77779_FG_FG_INT_STS_Tmx_CLEAR	(~(0x1 << 13))
#define MAX77779_FG_FG_INT_STS_Smx_SHIFT	14
#define MAX77779_FG_FG_INT_STS_Smx_MASK	(0x1 << 14)
#define MAX77779_FG_FG_INT_STS_Smx_CLEAR	(~(0x1 << 14))
#define MAX77779_FG_FG_INT_STS_Br_SHIFT	15
#define MAX77779_FG_FG_INT_STS_Br_MASK	(0x1 << 15)
#define MAX77779_FG_FG_INT_STS_Br_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_fg_fg_int_sts_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " PONR=%x",
		FIELD2VALUE(MAX77779_PONR, val));
	i += scnprintf(&buff[i], len - i, " Imn=%x",
		FIELD2VALUE(MAX77779_Imn, val));
	i += scnprintf(&buff[i], len - i, " Bst=%x",
		FIELD2VALUE(MAX77779_Bst, val));
	i += scnprintf(&buff[i], len - i, " Imx=%x",
		FIELD2VALUE(MAX77779_Imx, val));
	i += scnprintf(&buff[i], len - i, " dSOCi=%x",
		FIELD2VALUE(MAX77779_dSOCi, val));
	i += scnprintf(&buff[i], len - i, " Vmn=%x",
		FIELD2VALUE(MAX77779_Vmn, val));
	i += scnprintf(&buff[i], len - i, " Tmn=%x",
		FIELD2VALUE(MAX77779_Tmn, val));
	i += scnprintf(&buff[i], len - i, " Smn=%x",
		FIELD2VALUE(MAX77779_Smn, val));
	i += scnprintf(&buff[i], len - i, " Bi=%x",
		FIELD2VALUE(MAX77779_Bi, val));
	i += scnprintf(&buff[i], len - i, " Vmx=%x",
		FIELD2VALUE(MAX77779_Vmx, val));
	i += scnprintf(&buff[i], len - i, " Tmx=%x",
		FIELD2VALUE(MAX77779_Tmx, val));
	i += scnprintf(&buff[i], len - i, " Smx=%x",
		FIELD2VALUE(MAX77779_Smx, val));
	i += scnprintf(&buff[i], len - i, " Br=%x",
		FIELD2VALUE(MAX77779_Br, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_fg_int_sts_ponr,1,1)
MAX77779_BFF(max77779_fg_fg_int_sts_imn,2,2)
MAX77779_BFF(max77779_fg_fg_int_sts_bst,3,3)
MAX77779_BFF(max77779_fg_fg_int_sts_imx,6,6)
MAX77779_BFF(max77779_fg_fg_int_sts_dsoci,7,7)
MAX77779_BFF(max77779_fg_fg_int_sts_vmn,8,8)
MAX77779_BFF(max77779_fg_fg_int_sts_tmn,9,9)
MAX77779_BFF(max77779_fg_fg_int_sts_smn,10,10)
MAX77779_BFF(max77779_fg_fg_int_sts_bi,11,11)
MAX77779_BFF(max77779_fg_fg_int_sts_vmx,12,12)
MAX77779_BFF(max77779_fg_fg_int_sts_tmx,13,13)
MAX77779_BFF(max77779_fg_fg_int_sts_smx,14,14)
MAX77779_BFF(max77779_fg_fg_int_sts_br,15,15)

/*
 * MAX77779_FG_FG_INT_MASK,0x01,0b00000010,0x2,Reset_Type:F
 * POR_m,Imn_m,Bst_m,Imx_m,dSOCi_m,Vmn_m,Tmn_m,Smn_m,Bi_m,Vmx_m,Tmx_m,Smx_m,Br_m
 */
#define MAX77779_FG_FG_INT_MASK	0xe1
#define MAX77779_FG_FG_INT_MASK_POR_m_SHIFT	1
#define MAX77779_FG_FG_INT_MASK_POR_m_MASK	(0x1 << 1)
#define MAX77779_FG_FG_INT_MASK_POR_m_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_FG_INT_MASK_Imn_m_SHIFT	2
#define MAX77779_FG_FG_INT_MASK_Imn_m_MASK	(0x1 << 2)
#define MAX77779_FG_FG_INT_MASK_Imn_m_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_FG_INT_MASK_Bst_m_SHIFT	3
#define MAX77779_FG_FG_INT_MASK_Bst_m_MASK	(0x1 << 3)
#define MAX77779_FG_FG_INT_MASK_Bst_m_CLEAR	(~(0x1 << 3))
#define MAX77779_FG_FG_INT_MASK_Imx_m_SHIFT	6
#define MAX77779_FG_FG_INT_MASK_Imx_m_MASK	(0x1 << 6)
#define MAX77779_FG_FG_INT_MASK_Imx_m_CLEAR	(~(0x1 << 6))
#define MAX77779_FG_FG_INT_MASK_dSOCi_m_SHIFT	7
#define MAX77779_FG_FG_INT_MASK_dSOCi_m_MASK	(0x1 << 7)
#define MAX77779_FG_FG_INT_MASK_dSOCi_m_CLEAR	(~(0x1 << 7))
#define MAX77779_FG_FG_INT_MASK_Vmn_m_SHIFT	8
#define MAX77779_FG_FG_INT_MASK_Vmn_m_MASK	(0x1 << 8)
#define MAX77779_FG_FG_INT_MASK_Vmn_m_CLEAR	(~(0x1 << 8))
#define MAX77779_FG_FG_INT_MASK_Tmn_m_SHIFT	9
#define MAX77779_FG_FG_INT_MASK_Tmn_m_MASK	(0x1 << 9)
#define MAX77779_FG_FG_INT_MASK_Tmn_m_CLEAR	(~(0x1 << 9))
#define MAX77779_FG_FG_INT_MASK_Smn_m_SHIFT	10
#define MAX77779_FG_FG_INT_MASK_Smn_m_MASK	(0x1 << 10)
#define MAX77779_FG_FG_INT_MASK_Smn_m_CLEAR	(~(0x1 << 10))
#define MAX77779_FG_FG_INT_MASK_Bi_m_SHIFT	11
#define MAX77779_FG_FG_INT_MASK_Bi_m_MASK	(0x1 << 11)
#define MAX77779_FG_FG_INT_MASK_Bi_m_CLEAR	(~(0x1 << 11))
#define MAX77779_FG_FG_INT_MASK_Vmx_m_SHIFT	12
#define MAX77779_FG_FG_INT_MASK_Vmx_m_MASK	(0x1 << 12)
#define MAX77779_FG_FG_INT_MASK_Vmx_m_CLEAR	(~(0x1 << 12))
#define MAX77779_FG_FG_INT_MASK_Tmx_m_SHIFT	13
#define MAX77779_FG_FG_INT_MASK_Tmx_m_MASK	(0x1 << 13)
#define MAX77779_FG_FG_INT_MASK_Tmx_m_CLEAR	(~(0x1 << 13))
#define MAX77779_FG_FG_INT_MASK_Smx_m_SHIFT	14
#define MAX77779_FG_FG_INT_MASK_Smx_m_MASK	(0x1 << 14)
#define MAX77779_FG_FG_INT_MASK_Smx_m_CLEAR	(~(0x1 << 14))
#define MAX77779_FG_FG_INT_MASK_Br_m_SHIFT	15
#define MAX77779_FG_FG_INT_MASK_Br_m_MASK	(0x1 << 15)
#define MAX77779_FG_FG_INT_MASK_Br_m_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_fg_fg_int_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " POR_m=%x",
		FIELD2VALUE(MAX77779_POR_m, val));
	i += scnprintf(&buff[i], len - i, " Imn_m=%x",
		FIELD2VALUE(MAX77779_Imn_m, val));
	i += scnprintf(&buff[i], len - i, " Bst_m=%x",
		FIELD2VALUE(MAX77779_Bst_m, val));
	i += scnprintf(&buff[i], len - i, " Imx_m=%x",
		FIELD2VALUE(MAX77779_Imx_m, val));
	i += scnprintf(&buff[i], len - i, " dSOCi_m=%x",
		FIELD2VALUE(MAX77779_dSOCi_m, val));
	i += scnprintf(&buff[i], len - i, " Vmn_m=%x",
		FIELD2VALUE(MAX77779_Vmn_m, val));
	i += scnprintf(&buff[i], len - i, " Tmn_m=%x",
		FIELD2VALUE(MAX77779_Tmn_m, val));
	i += scnprintf(&buff[i], len - i, " Smn_m=%x",
		FIELD2VALUE(MAX77779_Smn_m, val));
	i += scnprintf(&buff[i], len - i, " Bi_m=%x",
		FIELD2VALUE(MAX77779_Bi_m, val));
	i += scnprintf(&buff[i], len - i, " Vmx_m=%x",
		FIELD2VALUE(MAX77779_Vmx_m, val));
	i += scnprintf(&buff[i], len - i, " Tmx_m=%x",
		FIELD2VALUE(MAX77779_Tmx_m, val));
	i += scnprintf(&buff[i], len - i, " Smx_m=%x",
		FIELD2VALUE(MAX77779_Smx_m, val));
	i += scnprintf(&buff[i], len - i, " Br_m=%x",
		FIELD2VALUE(MAX77779_Br_m, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_fg_int_mask_por_m,1,1)
MAX77779_BFF(max77779_fg_fg_int_mask_imn_m,2,2)
MAX77779_BFF(max77779_fg_fg_int_mask_bst_m,3,3)
MAX77779_BFF(max77779_fg_fg_int_mask_imx_m,6,6)
MAX77779_BFF(max77779_fg_fg_int_mask_dsoci_m,7,7)
MAX77779_BFF(max77779_fg_fg_int_mask_vmn_m,8,8)
MAX77779_BFF(max77779_fg_fg_int_mask_tmn_m,9,9)
MAX77779_BFF(max77779_fg_fg_int_mask_smn_m,10,10)
MAX77779_BFF(max77779_fg_fg_int_mask_bi_m,11,11)
MAX77779_BFF(max77779_fg_fg_int_mask_vmx_m,12,12)
MAX77779_BFF(max77779_fg_fg_int_mask_tmx_m,13,13)
MAX77779_BFF(max77779_fg_fg_int_mask_smx_m,14,14)
MAX77779_BFF(max77779_fg_fg_int_mask_br_m,15,15)

/*
 * MAX77779_FG_Command_fw,0x09,0b00000000,0x0,Reset_Type:F
 * CMD_FW[0:12]
 */
#define MAX77779_FG_Command_fw	0xe9
#define MAX77779_FG_Command_fw_CMD_FW_SHIFT	0
#define MAX77779_FG_Command_fw_CMD_FW_MASK	(0xfff << 0)
#define MAX77779_FG_Command_fw_CMD_FW_CLEAR	(~(0xfff << 0))
static inline const char *
max77779_fg_command_fw_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CMD_FW=%x",
		FIELD2VALUE(MAX77779_CMD_FW, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_command_fw_cmd_fw,11,0)

/*
 * MAX77779_FG_Command_ack,0x0A,0b00000000,0x0,Reset_Type:F
 * CMD_FW_BUSY
 */
#define MAX77779_FG_Command_ack	0xea
#define MAX77779_FG_Command_ack_CMD_FW_BUSY_SHIFT	0
#define MAX77779_FG_Command_ack_CMD_FW_BUSY_MASK	(0x1 << 0)
#define MAX77779_FG_Command_ack_CMD_FW_BUSY_CLEAR	(~(0x1 << 0))
static inline const char *
max77779_fg_command_ack_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CMD_FW_BUSY=%x",
		FIELD2VALUE(MAX77779_CMD_FW_BUSY, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_command_ack_cmd_fw_busy,0,0)

/*
 * MAX77779_FG_USR,0x1F,0b00001110,0xe,Reset_Type:F
 * NLOCK,VLOCK,RLOCK
 */
#define MAX77779_FG_USR	0xff
#define MAX77779_FG_USR_NLOCK_SHIFT	1
#define MAX77779_FG_USR_NLOCK_MASK	(0x1 << 1)
#define MAX77779_FG_USR_NLOCK_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_USR_VLOCK_SHIFT	2
#define MAX77779_FG_USR_VLOCK_MASK	(0x1 << 2)
#define MAX77779_FG_USR_VLOCK_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_USR_RLOCK_SHIFT	3
#define MAX77779_FG_USR_RLOCK_MASK	(0x1 << 3)
#define MAX77779_FG_USR_RLOCK_CLEAR	(~(0x1 << 3))
static inline const char *
max77779_fg_usr_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " NLOCK=%x",
		FIELD2VALUE(MAX77779_NLOCK, val));
	i += scnprintf(&buff[i], len - i, " VLOCK=%x",
		FIELD2VALUE(MAX77779_VLOCK, val));
	i += scnprintf(&buff[i], len - i, " RLOCK=%x",
		FIELD2VALUE(MAX77779_RLOCK, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_usr_nlock,1,1)
MAX77779_BFF(max77779_fg_usr_vlock,2,2)
MAX77779_BFF(max77779_fg_usr_rlock,3,3)

/*******************************************************
 * Section: FG_NVM 0x00 16
 */

/*
 * MAX77779_FG_NVM_nVAlrtTh,0x8C,0b1111111100000000,0xff00,Reset_Type:S
 * nVMIN[0:8],nVMAX[8:8]
 */
#define MAX77779_FG_NVM_nVAlrtTh	0x8c
#define MAX77779_FG_NVM_nVAlrtTh_nVMIN_SHIFT	0
#define MAX77779_FG_NVM_nVAlrtTh_nVMIN_MASK	(0xff << 0)
#define MAX77779_FG_NVM_nVAlrtTh_nVMIN_CLEAR	(~(0xff << 0))
#define MAX77779_FG_NVM_nVAlrtTh_nVMAX_SHIFT	8
#define MAX77779_FG_NVM_nVAlrtTh_nVMAX_MASK	(0xff << 8)
#define MAX77779_FG_NVM_nVAlrtTh_nVMAX_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_nvm_nvalrtth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " nVMIN=%x",
		FIELD2VALUE(MAX77779_nVMIN, val));
	i += scnprintf(&buff[i], len - i, " nVMAX=%x",
		FIELD2VALUE(MAX77779_nVMAX, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nvalrtth_nvmin,7,0)
MAX77779_BFF(max77779_fg_nvm_nvalrtth_nvmax,15,8)

/*
 * MAX77779_FG_NVM_nTAlrtTh,0x8D,0b111111110000000,0x7f80,Reset_Type:S
 * nTMIN[0:8],nTMAX[8:8]
 */
#define MAX77779_FG_NVM_nTAlrtTh	0x8d
#define MAX77779_FG_NVM_nTAlrtTh_nTMIN_SHIFT	0
#define MAX77779_FG_NVM_nTAlrtTh_nTMIN_MASK	(0xff << 0)
#define MAX77779_FG_NVM_nTAlrtTh_nTMIN_CLEAR	(~(0xff << 0))
#define MAX77779_FG_NVM_nTAlrtTh_nTMAX_SHIFT	8
#define MAX77779_FG_NVM_nTAlrtTh_nTMAX_MASK	(0xff << 8)
#define MAX77779_FG_NVM_nTAlrtTh_nTMAX_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_nvm_ntalrtth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " nTMIN=%x",
		FIELD2VALUE(MAX77779_nTMIN, val));
	i += scnprintf(&buff[i], len - i, " nTMAX=%x",
		FIELD2VALUE(MAX77779_nTMAX, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_ntalrtth_ntmin,7,0)
MAX77779_BFF(max77779_fg_nvm_ntalrtth_ntmax,15,8)

/*
 * MAX77779_FG_NVM_nIAlrtTh,0x8E,0b111111110000000,0x7f80,Reset_Type:S
 * nIMIN[0:8],nIMAX[8:8]
 */
#define MAX77779_FG_NVM_nIAlrtTh	0x8e
#define MAX77779_FG_NVM_nIAlrtTh_nIMIN_SHIFT	0
#define MAX77779_FG_NVM_nIAlrtTh_nIMIN_MASK	(0xff << 0)
#define MAX77779_FG_NVM_nIAlrtTh_nIMIN_CLEAR	(~(0xff << 0))
#define MAX77779_FG_NVM_nIAlrtTh_nIMAX_SHIFT	8
#define MAX77779_FG_NVM_nIAlrtTh_nIMAX_MASK	(0xff << 8)
#define MAX77779_FG_NVM_nIAlrtTh_nIMAX_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_nvm_nialrtth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " nIMIN=%x",
		FIELD2VALUE(MAX77779_nIMIN, val));
	i += scnprintf(&buff[i], len - i, " nIMAX=%x",
		FIELD2VALUE(MAX77779_nIMAX, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nialrtth_nimin,7,0)
MAX77779_BFF(max77779_fg_nvm_nialrtth_nimax,15,8)

/*
 * MAX77779_FG_NVM_nSAlrtTh,0x8F,0b1111111100000000,0xff00,Reset_Type:S
 * nSMIN[0:8],nSMAX[8:8]
 */
#define MAX77779_FG_NVM_nSAlrtTh	0x8f
#define MAX77779_FG_NVM_nSAlrtTh_nSMIN_SHIFT	0
#define MAX77779_FG_NVM_nSAlrtTh_nSMIN_MASK	(0xff << 0)
#define MAX77779_FG_NVM_nSAlrtTh_nSMIN_CLEAR	(~(0xff << 0))
#define MAX77779_FG_NVM_nSAlrtTh_nSMAX_SHIFT	8
#define MAX77779_FG_NVM_nSAlrtTh_nSMAX_MASK	(0xff << 8)
#define MAX77779_FG_NVM_nSAlrtTh_nSMAX_CLEAR	(~(0xff << 8))
static inline const char *
max77779_fg_nvm_nsalrtth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " nSMIN=%x",
		FIELD2VALUE(MAX77779_nSMIN, val));
	i += scnprintf(&buff[i], len - i, " nSMAX=%x",
		FIELD2VALUE(MAX77779_nSMAX, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nsalrtth_nsmin,7,0)
MAX77779_BFF(max77779_fg_nvm_nsalrtth_nsmax,15,8)

/*
 * MAX77779_FG_NVM_nIChgTerm,0x9C,0b1010000000,0x280,Reset_Type:S
 */
#define MAX77779_FG_NVM_nIChgTerm	0x9c

/*
 * MAX77779_FG_NVM_nFilterCfg,0x9D,0b1100111010100100,0xcea4,Reset_Type:S
 */
#define MAX77779_FG_NVM_nFilterCfg	0x9d

/*
 * MAX77779_FG_NVM_nModelCfg,0x9E,0b10000000000,0x400,Reset_Type:S
 */
#define MAX77779_FG_NVM_nModelCfg	0x9e

/*
 * MAX77779_FG_NVM_nLearnCfg,0x9F,0b00000000,0x0,Reset_Type:S
 * RCx,FiltEmpty,LearnStage[4:3]
 */
#define MAX77779_FG_NVM_nLearnCfg	0x9f
#define MAX77779_FG_NVM_nLearnCfg_RCx_SHIFT	2
#define MAX77779_FG_NVM_nLearnCfg_RCx_MASK	(0x1 << 2)
#define MAX77779_FG_NVM_nLearnCfg_RCx_CLEAR	(~(0x1 << 2))
#define MAX77779_FG_NVM_nLearnCfg_FiltEmpty_SHIFT	3
#define MAX77779_FG_NVM_nLearnCfg_FiltEmpty_MASK	(0x1 << 3)
#define MAX77779_FG_NVM_nLearnCfg_FiltEmpty_CLEAR	(~(0x1 << 3))
#define MAX77779_FG_NVM_nLearnCfg_LearnStage_SHIFT	4
#define MAX77779_FG_NVM_nLearnCfg_LearnStage_MASK	(0x7 << 4)
#define MAX77779_FG_NVM_nLearnCfg_LearnStage_CLEAR	(~(0x7 << 4))
static inline const char *
max77779_fg_nvm_nlearncfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " RCx=%x",
		FIELD2VALUE(MAX77779_RCx, val));
	i += scnprintf(&buff[i], len - i, " FiltEmpty=%x",
		FIELD2VALUE(MAX77779_FiltEmpty, val));
	i += scnprintf(&buff[i], len - i, " LearnStage=%x",
		FIELD2VALUE(MAX77779_LearnStage, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nlearncfg_rcx,2,2)
MAX77779_BFF(max77779_fg_nvm_nlearncfg_filtempty,3,3)
MAX77779_BFF(max77779_fg_nvm_nlearncfg_learnstage,6,4)

/*
 * MAX77779_FG_NVM_nQRTable00,0xA0,0b1000010000000,0x1080,Reset_Type:S
 */
#define MAX77779_FG_NVM_nQRTable00	0xa0

/*
 * MAX77779_FG_NVM_nQRTable10,0xA1,0b10000001000000,0x2040,Reset_Type:S
 */
#define MAX77779_FG_NVM_nQRTable10	0xa1

/*
 * MAX77779_FG_NVM_nQRTable20,0xA2,0b11110001000,0x788,Reset_Type:S
 */
#define MAX77779_FG_NVM_nQRTable20	0xa2

/*
 * MAX77779_FG_NVM_nQRTable30,0xA3,0b100010000000,0x880,Reset_Type:S
 */
#define MAX77779_FG_NVM_nQRTable30	0xa3

/*
 * MAX77779_FG_NVM_nCycles,0xA4,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_NVM_nCycles	0xa4

/*
 * MAX77779_FG_NVM_nFullCapNom,0xA5,0b110101001000,0xd48,Reset_Type:S
 */
#define MAX77779_FG_NVM_nFullCapNom	0xa5

/*
 * MAX77779_FG_NVM_nRComp0,0xA6,0b01110000,0x70,Reset_Type:S
 */
#define MAX77779_FG_NVM_nRComp0	0xa6

/*
 * MAX77779_FG_NVM_nTempCo,0xA7,0b10001000111110,0x223e,Reset_Type:S
 */
#define MAX77779_FG_NVM_nTempCo	0xa7

/*
 * MAX77779_FG_NVM_nFullCapRep,0xA9,0b110101001000,0xd48,Reset_Type:S
 */
#define MAX77779_FG_NVM_nFullCapRep	0xa9

/*
 * MAX77779_FG_NVM_nTimerH,0xAF,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_FG_NVM_nTimerH	0xaf

/*
 * MAX77779_FG_NVM_nRippleCfg,0xB1,0b1000000100,0x204,Reset_Type:S
 */
#define MAX77779_FG_NVM_nRippleCfg	0xb1

/*
 * MAX77779_FG_NVM_nMiscCfg,0xB2,0b11000001110000,0x3070,Reset_Type:S
 */
#define MAX77779_FG_NVM_nMiscCfg	0xb2

/*
 * MAX77779_FG_NVM_nDesignCap,0xB3,0b10111011100,0x5dc,Reset_Type:S
 */
#define MAX77779_FG_NVM_nDesignCap	0xb3

/*
 * MAX77779_FG_NVM_RelaxCFG,0xB6,0b100000111001,0x839,Reset_Type:S
 * dT[0:4],dV[4:5],Load[9:7]
 */
#define MAX77779_FG_NVM_RelaxCFG	0xb6
#define MAX77779_FG_NVM_RelaxCFG_dT_SHIFT	0
#define MAX77779_FG_NVM_RelaxCFG_dT_MASK	(0xf << 0)
#define MAX77779_FG_NVM_RelaxCFG_dT_CLEAR	(~(0xf << 0))
#define MAX77779_FG_NVM_RelaxCFG_dV_SHIFT	4
#define MAX77779_FG_NVM_RelaxCFG_dV_MASK	(0x1f << 4)
#define MAX77779_FG_NVM_RelaxCFG_dV_CLEAR	(~(0x1f << 4))
#define MAX77779_FG_NVM_RelaxCFG_Load_SHIFT	9
#define MAX77779_FG_NVM_RelaxCFG_Load_MASK	(0x7f << 9)
#define MAX77779_FG_NVM_RelaxCFG_Load_CLEAR	(~(0x7f << 9))
static inline const char *
max77779_fg_nvm_relaxcfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " dT=%x",
		FIELD2VALUE(MAX77779_dT, val));
	i += scnprintf(&buff[i], len - i, " dV=%x",
		FIELD2VALUE(MAX77779_dV, val));
	i += scnprintf(&buff[i], len - i, " Load=%x",
		FIELD2VALUE(MAX77779_Load, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_relaxcfg_dt,3,0)
MAX77779_BFF(max77779_fg_nvm_relaxcfg_dv,8,4)
MAX77779_BFF(max77779_fg_nvm_relaxcfg_load,15,9)

/*
 * MAX77779_FG_NVM_nConvgCfg,0xB7,0b10001001000001,0x2241,Reset_Type:S
 * VoltLowOff[7:5],RepLow[12:4]
 */
#define MAX77779_FG_NVM_nConvgCfg	0xb7
#define MAX77779_FG_NVM_nConvgCfg_VoltLowOff_SHIFT	7
#define MAX77779_FG_NVM_nConvgCfg_VoltLowOff_MASK	(0x1f << 7)
#define MAX77779_FG_NVM_nConvgCfg_VoltLowOff_CLEAR	(~(0x1f << 7))
#define MAX77779_FG_NVM_nConvgCfg_RepLow_SHIFT	12
#define MAX77779_FG_NVM_nConvgCfg_RepLow_MASK	(0xf << 12)
#define MAX77779_FG_NVM_nConvgCfg_RepLow_CLEAR	(~(0xf << 12))
static inline const char *
max77779_fg_nvm_nconvgcfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " VoltLowOff=%x",
		FIELD2VALUE(MAX77779_VoltLowOff, val));
	i += scnprintf(&buff[i], len - i, " RepLow=%x",
		FIELD2VALUE(MAX77779_RepLow, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nconvgcfg_voltlowoff,11,7)
MAX77779_BFF(max77779_fg_nvm_nconvgcfg_replow,15,12)

/*
 * MAX77779_FG_NVM_nHibCfg,0xBB,0b00000000,0x0,Reset_Type:S
 * HibScalar[0:3],HibExitTime[3:2],inusefixed5to7[5:3],HibThreshold[8:4],
 * HibEnterTime[12:3],EnHib
 */
#define MAX77779_FG_NVM_nHibCfg	0xbb
#define MAX77779_FG_NVM_nHibCfg_HibScalar_SHIFT	0
#define MAX77779_FG_NVM_nHibCfg_HibScalar_MASK	(0x7 << 0)
#define MAX77779_FG_NVM_nHibCfg_HibScalar_CLEAR	(~(0x7 << 0))
#define MAX77779_FG_NVM_nHibCfg_HibExitTime_SHIFT	3
#define MAX77779_FG_NVM_nHibCfg_HibExitTime_MASK	(0x3 << 3)
#define MAX77779_FG_NVM_nHibCfg_HibExitTime_CLEAR	(~(0x3 << 3))
#define MAX77779_FG_NVM_nHibCfg_inusefixed5to7_SHIFT	5
#define MAX77779_FG_NVM_nHibCfg_inusefixed5to7_MASK	(0x7 << 5)
#define MAX77779_FG_NVM_nHibCfg_inusefixed5to7_CLEAR	(~(0x7 << 5))
#define MAX77779_FG_NVM_nHibCfg_HibThreshold_SHIFT	8
#define MAX77779_FG_NVM_nHibCfg_HibThreshold_MASK	(0xf << 8)
#define MAX77779_FG_NVM_nHibCfg_HibThreshold_CLEAR	(~(0xf << 8))
#define MAX77779_FG_NVM_nHibCfg_HibEnterTime_SHIFT	12
#define MAX77779_FG_NVM_nHibCfg_HibEnterTime_MASK	(0x7 << 12)
#define MAX77779_FG_NVM_nHibCfg_HibEnterTime_CLEAR	(~(0x7 << 12))
#define MAX77779_FG_NVM_nHibCfg_EnHib_SHIFT	15
#define MAX77779_FG_NVM_nHibCfg_EnHib_MASK	(0x1 << 15)
#define MAX77779_FG_NVM_nHibCfg_EnHib_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_fg_nvm_nhibcfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " HibScalar=%x",
		FIELD2VALUE(MAX77779_HibScalar, val));
	i += scnprintf(&buff[i], len - i, " HibExitTime=%x",
		FIELD2VALUE(MAX77779_HibExitTime, val));
	i += scnprintf(&buff[i], len - i, " inusefixed5to7=%x",
		FIELD2VALUE(MAX77779_inusefixed5to7, val));
	i += scnprintf(&buff[i], len - i, " HibThreshold=%x",
		FIELD2VALUE(MAX77779_HibThreshold, val));
	i += scnprintf(&buff[i], len - i, " HibEnterTime=%x",
		FIELD2VALUE(MAX77779_HibEnterTime, val));
	i += scnprintf(&buff[i], len - i, " EnHib=%x",
		FIELD2VALUE(MAX77779_EnHib, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nhibcfg_hibscalar,2,0)
MAX77779_BFF(max77779_fg_nvm_nhibcfg_hibexittime,4,3)
MAX77779_BFF(max77779_fg_nvm_nhibcfg_inusefixed5to7,7,5)
MAX77779_BFF(max77779_fg_nvm_nhibcfg_hibthreshold,11,8)
MAX77779_BFF(max77779_fg_nvm_nhibcfg_hibentertime,14,12)
MAX77779_BFF(max77779_fg_nvm_nhibcfg_enhib,15,15)

/*
 * MAX77779_FG_NVM_nFullSOCThr,0xBC,0b101000000000101,0x5005,Reset_Type:S
 */
#define MAX77779_FG_NVM_nFullSOCThr	0xbc

/*
 * MAX77779_FG_NVM_nNVCfg0,0xC0,0b00000000,0x0,Reset_Type:S
 * Reserved_2_0[0:3],enAF,Reserved_15_4[4:12]
 */
#define MAX77779_FG_NVM_nNVCfg0	0xc0
#define MAX77779_FG_NVM_nNVCfg0_Reserved_2_0_SHIFT	0
#define MAX77779_FG_NVM_nNVCfg0_Reserved_2_0_MASK	(0x7 << 0)
#define MAX77779_FG_NVM_nNVCfg0_Reserved_2_0_CLEAR	(~(0x7 << 0))
#define MAX77779_FG_NVM_nNVCfg0_enAF_SHIFT	3
#define MAX77779_FG_NVM_nNVCfg0_enAF_MASK	(0x1 << 3)
#define MAX77779_FG_NVM_nNVCfg0_enAF_CLEAR	(~(0x1 << 3))
#define MAX77779_FG_NVM_nNVCfg0_Reserved_15_4_SHIFT	4
#define MAX77779_FG_NVM_nNVCfg0_Reserved_15_4_MASK	(0xfff << 4)
#define MAX77779_FG_NVM_nNVCfg0_Reserved_15_4_CLEAR	(~(0xfff << 4))
static inline const char *
max77779_fg_nvm_nnvcfg0_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " Reserved_2_0=%x",
		FIELD2VALUE(MAX77779_Reserved_2_0, val));
	i += scnprintf(&buff[i], len - i, " enAF=%x",
		FIELD2VALUE(MAX77779_enAF, val));
	i += scnprintf(&buff[i], len - i, " Reserved_15_4=%x",
		FIELD2VALUE(MAX77779_Reserved_15_4, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nnvcfg0_reserved_2_0,2,0)
MAX77779_BFF(max77779_fg_nvm_nnvcfg0_enaf,3,3)
MAX77779_BFF(max77779_fg_nvm_nnvcfg0_reserved_15_4,15,4)

/*
 * MAX77779_FG_NVM_nVEmpty,0xC6,0b1010010101100001,0xa561,Reset_Type:S
 * VR[0:7],VE[7:9]
 */
#define MAX77779_FG_NVM_nVEmpty	0xc6
#define MAX77779_FG_NVM_nVEmpty_VR_SHIFT	0
#define MAX77779_FG_NVM_nVEmpty_VR_MASK	(0x7f << 0)
#define MAX77779_FG_NVM_nVEmpty_VR_CLEAR	(~(0x7f << 0))
#define MAX77779_FG_NVM_nVEmpty_VE_SHIFT	7
#define MAX77779_FG_NVM_nVEmpty_VE_MASK	(0x1ff << 7)
#define MAX77779_FG_NVM_nVEmpty_VE_CLEAR	(~(0x1ff << 7))
static inline const char *
max77779_fg_nvm_nvempty_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " VR=%x",
		FIELD2VALUE(MAX77779_VR, val));
	i += scnprintf(&buff[i], len - i, " VE=%x",
		FIELD2VALUE(MAX77779_VE, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nvempty_vr,6,0)
MAX77779_BFF(max77779_fg_nvm_nvempty_ve,15,7)

/*
 * MAX77779_FG_NVM_nThermCfg3,0xC8,0b111000110111110,0x71be,Reset_Type:S
 */
#define MAX77779_FG_NVM_nThermCfg3	0xc8

/*
 * MAX77779_FG_NVM_nThermCfg2,0xC9,0b111000110111110,0x71be,Reset_Type:S
 */
#define MAX77779_FG_NVM_nThermCfg2	0xc9

/*
 * MAX77779_FG_NVM_nThermCfg,0xCA,0b111000110111110,0x71be,Reset_Type:S
 */
#define MAX77779_FG_NVM_nThermCfg	0xca

/*
 * MAX77779_FG_NVM_nProtMiscTh,0xD6,0b00100000,0x20,Reset_Type:S
 * CurrentOffCalEn,ApplyLearnedOffEn,LearnedOffNoQC
 */
#define MAX77779_FG_NVM_nProtMiscTh	0xd6
#define MAX77779_FG_NVM_nProtMiscTh_CurrentOffCalEn_SHIFT	0
#define MAX77779_FG_NVM_nProtMiscTh_CurrentOffCalEn_MASK	(0x1 << 0)
#define MAX77779_FG_NVM_nProtMiscTh_CurrentOffCalEn_CLEAR	(~(0x1 << 0))
#define MAX77779_FG_NVM_nProtMiscTh_ApplyLearnedOffEn_SHIFT	1
#define MAX77779_FG_NVM_nProtMiscTh_ApplyLearnedOffEn_MASK	(0x1 << 1)
#define MAX77779_FG_NVM_nProtMiscTh_ApplyLearnedOffEn_CLEAR	(~(0x1 << 1))
#define MAX77779_FG_NVM_nProtMiscTh_LearnedOffNoQC_SHIFT	2
#define MAX77779_FG_NVM_nProtMiscTh_LearnedOffNoQC_MASK	(0x1 << 2)
#define MAX77779_FG_NVM_nProtMiscTh_LearnedOffNoQC_CLEAR	(~(0x1 << 2))
static inline const char *
max77779_fg_nvm_nprotmiscth_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " CurrentOffCalEn=%x",
		FIELD2VALUE(MAX77779_CurrentOffCalEn, val));
	i += scnprintf(&buff[i], len - i, " ApplyLearnedOffEn=%x",
		FIELD2VALUE(MAX77779_ApplyLearnedOffEn, val));
	i += scnprintf(&buff[i], len - i, " LearnedOffNoQC=%x",
		FIELD2VALUE(MAX77779_LearnedOffNoQC, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_fg_nvm_nprotmiscth_currentoffcalen,0,0)
MAX77779_BFF(max77779_fg_nvm_nprotmiscth_applylearnedoffen,1,1)
MAX77779_BFF(max77779_fg_nvm_nprotmiscth_learnedoffnoqc,2,2)

/*******************************************************
 * Section: I2C_Master 0x00 8
 */

/*
 * MAX77779_I2CM_INTERRUPT,0x00,0b00000000,0x0,Reset_Type:IM
 * DONEI,SPR_6_1[1:6],ERRI
 */
#define MAX77779_I2CM_INTERRUPT	0x00
#define MAX77779_I2CM_INTERRUPT_DONEI_SHIFT	0
#define MAX77779_I2CM_INTERRUPT_DONEI_MASK	(0x1 << 0)
#define MAX77779_I2CM_INTERRUPT_DONEI_CLEAR	(~(0x1 << 0))
#define MAX77779_I2CM_INTERRUPT_SPR_6_1_SHIFT	1
#define MAX77779_I2CM_INTERRUPT_SPR_6_1_MASK	(0x3f << 1)
#define MAX77779_I2CM_INTERRUPT_SPR_6_1_CLEAR	(~(0x3f << 1))
#define MAX77779_I2CM_INTERRUPT_ERRI_SHIFT	7
#define MAX77779_I2CM_INTERRUPT_ERRI_MASK	(0x1 << 7)
#define MAX77779_I2CM_INTERRUPT_ERRI_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_i2cm_interrupt_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DONEI=%x",
		FIELD2VALUE(MAX77779_DONEI, val));
	i += scnprintf(&buff[i], len - i, " SPR_6_1=%x",
		FIELD2VALUE(MAX77779_SPR_6_1, val));
	i += scnprintf(&buff[i], len - i, " ERRI=%x",
		FIELD2VALUE(MAX77779_ERRI, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_i2cm_interrupt_donei,0,0)
MAX77779_BFF(max77779_i2cm_interrupt_spr_6_1,6,1)
MAX77779_BFF(max77779_i2cm_interrupt_erri,7,7)

/*
 * MAX77779_I2CM_INTMASK,0x01,0b11111111,0xff,Reset_Type:IM
 * DONEIM,SPR_6_1[1:6],ERRIM
 */
#define MAX77779_I2CM_INTMASK	0x01
#define MAX77779_I2CM_INTMASK_DONEIM_SHIFT	0
#define MAX77779_I2CM_INTMASK_DONEIM_MASK	(0x1 << 0)
#define MAX77779_I2CM_INTMASK_DONEIM_CLEAR	(~(0x1 << 0))
#define MAX77779_I2CM_INTMASK_SPR_6_1_SHIFT	1
#define MAX77779_I2CM_INTMASK_SPR_6_1_MASK	(0x3f << 1)
#define MAX77779_I2CM_INTMASK_SPR_6_1_CLEAR	(~(0x3f << 1))
#define MAX77779_I2CM_INTMASK_ERRIM_SHIFT	7
#define MAX77779_I2CM_INTMASK_ERRIM_MASK	(0x1 << 7)
#define MAX77779_I2CM_INTMASK_ERRIM_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_i2cm_intmask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " DONEIM=%x",
		FIELD2VALUE(MAX77779_DONEIM, val));
	i += scnprintf(&buff[i], len - i, " SPR_6_1=%x",
		FIELD2VALUE(MAX77779_SPR_6_1, val));
	i += scnprintf(&buff[i], len - i, " ERRIM=%x",
		FIELD2VALUE(MAX77779_ERRIM, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_i2cm_intmask_doneim,0,0)
MAX77779_BFF(max77779_i2cm_intmask_spr_6_1,6,1)
MAX77779_BFF(max77779_i2cm_intmask_errim,7,7)

/*
 * MAX77779_I2CM_STATUS,0x02,0b00000000,0x0,Reset_Type:IM
 * ERROR[0:7],BUS
 */
#define MAX77779_I2CM_STATUS	0x02
#define MAX77779_I2CM_STATUS_ERROR_SHIFT	0
#define MAX77779_I2CM_STATUS_ERROR_MASK	(0x7f << 0)
#define MAX77779_I2CM_STATUS_ERROR_CLEAR	(~(0x7f << 0))
#define MAX77779_I2CM_STATUS_BUS_SHIFT	7
#define MAX77779_I2CM_STATUS_BUS_MASK	(0x1 << 7)
#define MAX77779_I2CM_STATUS_BUS_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_i2cm_status_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " ERROR=%x",
		FIELD2VALUE(MAX77779_ERROR, val));
	i += scnprintf(&buff[i], len - i, " BUS=%x",
		FIELD2VALUE(MAX77779_BUS, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_i2cm_status_error,6,0)
MAX77779_BFF(max77779_i2cm_status_bus,7,7)

/*
 * MAX77779_I2CM_TIMEOUT,0x03,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TIMEOUT	0x03

/*
 * MAX77779_I2CM_CONTROL,0x04,0b00110000,0x30,Reset_Type:IM
 * I2CEN,CLOCK_SPEED[1:2],OEN,SDA,SCL,SCLO,SDAO
 */
#define MAX77779_I2CM_CONTROL	0x04
#define MAX77779_I2CM_CONTROL_I2CEN_SHIFT	0
#define MAX77779_I2CM_CONTROL_I2CEN_MASK	(0x1 << 0)
#define MAX77779_I2CM_CONTROL_I2CEN_CLEAR	(~(0x1 << 0))
#define MAX77779_I2CM_CONTROL_CLOCK_SPEED_SHIFT	1
#define MAX77779_I2CM_CONTROL_CLOCK_SPEED_MASK	(0x3 << 1)
#define MAX77779_I2CM_CONTROL_CLOCK_SPEED_CLEAR	(~(0x3 << 1))
#define MAX77779_I2CM_CONTROL_OEN_SHIFT	3
#define MAX77779_I2CM_CONTROL_OEN_MASK	(0x1 << 3)
#define MAX77779_I2CM_CONTROL_OEN_CLEAR	(~(0x1 << 3))
#define MAX77779_I2CM_CONTROL_SDA_SHIFT	4
#define MAX77779_I2CM_CONTROL_SDA_MASK	(0x1 << 4)
#define MAX77779_I2CM_CONTROL_SDA_CLEAR	(~(0x1 << 4))
#define MAX77779_I2CM_CONTROL_SCL_SHIFT	5
#define MAX77779_I2CM_CONTROL_SCL_MASK	(0x1 << 5)
#define MAX77779_I2CM_CONTROL_SCL_CLEAR	(~(0x1 << 5))
#define MAX77779_I2CM_CONTROL_SCLO_SHIFT	6
#define MAX77779_I2CM_CONTROL_SCLO_MASK	(0x1 << 6)
#define MAX77779_I2CM_CONTROL_SCLO_CLEAR	(~(0x1 << 6))
#define MAX77779_I2CM_CONTROL_SDAO_SHIFT	7
#define MAX77779_I2CM_CONTROL_SDAO_MASK	(0x1 << 7)
#define MAX77779_I2CM_CONTROL_SDAO_CLEAR	(~(0x1 << 7))
static inline const char *
max77779_i2cm_control_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " I2CEN=%x",
		FIELD2VALUE(MAX77779_I2CEN, val));
	i += scnprintf(&buff[i], len - i, " CLOCK_SPEED=%x",
		FIELD2VALUE(MAX77779_CLOCK_SPEED, val));
	i += scnprintf(&buff[i], len - i, " OEN=%x",
		FIELD2VALUE(MAX77779_OEN, val));
	i += scnprintf(&buff[i], len - i, " SDA=%x",
		FIELD2VALUE(MAX77779_SDA, val));
	i += scnprintf(&buff[i], len - i, " SCL=%x",
		FIELD2VALUE(MAX77779_SCL, val));
	i += scnprintf(&buff[i], len - i, " SCLO=%x",
		FIELD2VALUE(MAX77779_SCLO, val));
	i += scnprintf(&buff[i], len - i, " SDAO=%x",
		FIELD2VALUE(MAX77779_SDAO, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_i2cm_control_i2cen,0,0)
MAX77779_BFF(max77779_i2cm_control_clock_speed,2,1)
MAX77779_BFF(max77779_i2cm_control_oen,3,3)
MAX77779_BFF(max77779_i2cm_control_sda,4,4)
MAX77779_BFF(max77779_i2cm_control_scl,5,5)
MAX77779_BFF(max77779_i2cm_control_sclo,6,6)
MAX77779_BFF(max77779_i2cm_control_sdao,7,7)

/*
 * MAX77779_I2CM_SLADD,0x05,0b00000000,0x0,Reset_Type:IM
 * TO_SRC,SLAVE_ID[1:7]
 */
#define MAX77779_I2CM_SLADD	0x05
#define MAX77779_I2CM_SLADD_TO_SRC_SHIFT	0
#define MAX77779_I2CM_SLADD_TO_SRC_MASK	(0x1 << 0)
#define MAX77779_I2CM_SLADD_TO_SRC_CLEAR	(~(0x1 << 0))
#define MAX77779_I2CM_SLADD_SLAVE_ID_SHIFT	1
#define MAX77779_I2CM_SLADD_SLAVE_ID_MASK	(0x7f << 1)
#define MAX77779_I2CM_SLADD_SLAVE_ID_CLEAR	(~(0x7f << 1))
static inline const char *
max77779_i2cm_sladd_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TO_SRC=%x",
		FIELD2VALUE(MAX77779_TO_SRC, val));
	i += scnprintf(&buff[i], len - i, " SLAVE_ID=%x",
		FIELD2VALUE(MAX77779_SLAVE_ID, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_i2cm_sladd_to_src,0,0)
MAX77779_BFF(max77779_i2cm_sladd_slave_id,7,1)

/*
 * MAX77779_I2CM_TXDATA_CNT,0x06,0b00000000,0x0,Reset_Type:IM
 * TXCNT[0:6],SPR_7_6[6:2]
 */
#define MAX77779_I2CM_TXDATA_CNT	0x06
#define MAX77779_I2CM_TXDATA_CNT_TXCNT_SHIFT	0
#define MAX77779_I2CM_TXDATA_CNT_TXCNT_MASK	(0x3f << 0)
#define MAX77779_I2CM_TXDATA_CNT_TXCNT_CLEAR	(~(0x3f << 0))
#define MAX77779_I2CM_TXDATA_CNT_SPR_7_6_SHIFT	6
#define MAX77779_I2CM_TXDATA_CNT_SPR_7_6_MASK	(0x3 << 6)
#define MAX77779_I2CM_TXDATA_CNT_SPR_7_6_CLEAR	(~(0x3 << 6))
static inline const char *
max77779_i2cm_txdata_cnt_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " TXCNT=%x",
		FIELD2VALUE(MAX77779_TXCNT, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_6=%x",
		FIELD2VALUE(MAX77779_SPR_7_6, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_i2cm_txdata_cnt_txcnt,5,0)
MAX77779_BFF(max77779_i2cm_txdata_cnt_spr_7_6,7,6)

/*
 * MAX77779_I2CM_TX_BUFFER_0,0x07,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_0	0x07

/*
 * MAX77779_I2CM_TX_BUFFER_1,0x08,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_1	0x08

/*
 * MAX77779_I2CM_TX_BUFFER_2,0x09,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_2	0x09

/*
 * MAX77779_I2CM_TX_BUFFER_3,0x0A,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_3	0x0a

/*
 * MAX77779_I2CM_TX_BUFFER_4,0x0B,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_4	0x0b

/*
 * MAX77779_I2CM_TX_BUFFER_5,0x0C,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_5	0x0c

/*
 * MAX77779_I2CM_TX_BUFFER_6,0x0D,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_6	0x0d

/*
 * MAX77779_I2CM_TX_BUFFER_7,0x0E,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_7	0x0e

/*
 * MAX77779_I2CM_TX_BUFFER_8,0x0F,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_8	0x0f

/*
 * MAX77779_I2CM_TX_BUFFER_9,0x10,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_9	0x10

/*
 * MAX77779_I2CM_TX_BUFFER_10,0x11,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_10	0x11

/*
 * MAX77779_I2CM_TX_BUFFER_11,0x12,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_11	0x12

/*
 * MAX77779_I2CM_TX_BUFFER_12,0x13,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_12	0x13

/*
 * MAX77779_I2CM_TX_BUFFER_13,0x14,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_13	0x14

/*
 * MAX77779_I2CM_TX_BUFFER_14,0x15,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_14	0x15

/*
 * MAX77779_I2CM_TX_BUFFER_15,0x16,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_15	0x16

/*
 * MAX77779_I2CM_TX_BUFFER_16,0x17,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_16	0x17

/*
 * MAX77779_I2CM_TX_BUFFER_17,0x18,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_17	0x18

/*
 * MAX77779_I2CM_TX_BUFFER_18,0x19,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_18	0x19

/*
 * MAX77779_I2CM_TX_BUFFER_19,0x1A,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_19	0x1a

/*
 * MAX77779_I2CM_TX_BUFFER_20,0x1B,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_20	0x1b

/*
 * MAX77779_I2CM_TX_BUFFER_21,0x1C,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_21	0x1c

/*
 * MAX77779_I2CM_TX_BUFFER_22,0x1D,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_22	0x1d

/*
 * MAX77779_I2CM_TX_BUFFER_23,0x1E,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_23	0x1e

/*
 * MAX77779_I2CM_TX_BUFFER_24,0x1F,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_24	0x1f

/*
 * MAX77779_I2CM_TX_BUFFER_25,0x20,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_25	0x20

/*
 * MAX77779_I2CM_TX_BUFFER_26,0x21,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_26	0x21

/*
 * MAX77779_I2CM_TX_BUFFER_27,0x22,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_27	0x22

/*
 * MAX77779_I2CM_TX_BUFFER_28,0x23,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_28	0x23

/*
 * MAX77779_I2CM_TX_BUFFER_29,0x24,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_29	0x24

/*
 * MAX77779_I2CM_TX_BUFFER_30,0x25,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_30	0x25

/*
 * MAX77779_I2CM_TX_BUFFER_31,0x26,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_31	0x26

/*
 * MAX77779_I2CM_TX_BUFFER_32,0x27,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_32	0x27

/*
 * MAX77779_I2CM_TX_BUFFER_33,0x28,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_TX_BUFFER_33	0x28

/*
 * MAX77779_I2CM_RXDATA_CNT,0x29,0b00000000,0x0,Reset_Type:IM
 * RXCNT[0:5],SPR_7_5[5:3]
 */
#define MAX77779_I2CM_RXDATA_CNT	0x29
#define MAX77779_I2CM_RXDATA_CNT_RXCNT_SHIFT	0
#define MAX77779_I2CM_RXDATA_CNT_RXCNT_MASK	(0x1f << 0)
#define MAX77779_I2CM_RXDATA_CNT_RXCNT_CLEAR	(~(0x1f << 0))
#define MAX77779_I2CM_RXDATA_CNT_SPR_7_5_SHIFT	5
#define MAX77779_I2CM_RXDATA_CNT_SPR_7_5_MASK	(0x7 << 5)
#define MAX77779_I2CM_RXDATA_CNT_SPR_7_5_CLEAR	(~(0x7 << 5))
static inline const char *
max77779_i2cm_rxdata_cnt_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " RXCNT=%x",
		FIELD2VALUE(MAX77779_RXCNT, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_5=%x",
		FIELD2VALUE(MAX77779_SPR_7_5, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_i2cm_rxdata_cnt_rxcnt,4,0)
MAX77779_BFF(max77779_i2cm_rxdata_cnt_spr_7_5,7,5)

/*
 * MAX77779_I2CM_CMD,0x2A,0b00000000,0x0,Reset_Type:IM
 * I2CMWRITE,I2CMREAD,SPR_7_2[2:6]
 */
#define MAX77779_I2CM_CMD	0x2a
#define MAX77779_I2CM_CMD_I2CMWRITE_SHIFT	0
#define MAX77779_I2CM_CMD_I2CMWRITE_MASK	(0x1 << 0)
#define MAX77779_I2CM_CMD_I2CMWRITE_CLEAR	(~(0x1 << 0))
#define MAX77779_I2CM_CMD_I2CMREAD_SHIFT	1
#define MAX77779_I2CM_CMD_I2CMREAD_MASK	(0x1 << 1)
#define MAX77779_I2CM_CMD_I2CMREAD_CLEAR	(~(0x1 << 1))
#define MAX77779_I2CM_CMD_SPR_7_2_SHIFT	2
#define MAX77779_I2CM_CMD_SPR_7_2_MASK	(0x3f << 2)
#define MAX77779_I2CM_CMD_SPR_7_2_CLEAR	(~(0x3f << 2))
static inline const char *
max77779_i2cm_cmd_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " I2CMWRITE=%x",
		FIELD2VALUE(MAX77779_I2CMWRITE, val));
	i += scnprintf(&buff[i], len - i, " I2CMREAD=%x",
		FIELD2VALUE(MAX77779_I2CMREAD, val));
	i += scnprintf(&buff[i], len - i, " SPR_7_2=%x",
		FIELD2VALUE(MAX77779_SPR_7_2, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_i2cm_cmd_i2cmwrite,0,0)
MAX77779_BFF(max77779_i2cm_cmd_i2cmread,1,1)
MAX77779_BFF(max77779_i2cm_cmd_spr_7_2,7,2)

/*
 * MAX77779_I2CM_RX_BUFFER_0,0x2B,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_0	0x2b

/*
 * MAX77779_I2CM_RX_BUFFER_1,0x2C,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_1	0x2c

/*
 * MAX77779_I2CM_RX_BUFFER_2,0x2D,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_2	0x2d

/*
 * MAX77779_I2CM_RX_BUFFER_3,0x2E,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_3	0x2e

/*
 * MAX77779_I2CM_RX_BUFFER_4,0x2F,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_4	0x2f

/*
 * MAX77779_I2CM_RX_BUFFER_5,0x30,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_5	0x30

/*
 * MAX77779_I2CM_RX_BUFFER_6,0x31,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_6	0x31

/*
 * MAX77779_I2CM_RX_BUFFER_7,0x32,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_7	0x32

/*
 * MAX77779_I2CM_RX_BUFFER_8,0x33,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_8	0x33

/*
 * MAX77779_I2CM_RX_BUFFER_9,0x34,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_9	0x34

/*
 * MAX77779_I2CM_RX_BUFFER_10,0x35,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_10	0x35

/*
 * MAX77779_I2CM_RX_BUFFER_11,0x36,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_11	0x36

/*
 * MAX77779_I2CM_RX_BUFFER_12,0x37,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_12	0x37

/*
 * MAX77779_I2CM_RX_BUFFER_13,0x38,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_13	0x38

/*
 * MAX77779_I2CM_RX_BUFFER_14,0x39,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_14	0x39

/*
 * MAX77779_I2CM_RX_BUFFER_15,0x3A,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_15	0x3a

/*
 * MAX77779_I2CM_RX_BUFFER_16,0x3B,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_16	0x3b

/*
 * MAX77779_I2CM_RX_BUFFER_17,0x3C,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_17	0x3c

/*
 * MAX77779_I2CM_RX_BUFFER_18,0x3D,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_18	0x3d

/*
 * MAX77779_I2CM_RX_BUFFER_19,0x3E,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_19	0x3e

/*
 * MAX77779_I2CM_RX_BUFFER_20,0x3F,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_20	0x3f

/*
 * MAX77779_I2CM_RX_BUFFER_21,0x40,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_21	0x40

/*
 * MAX77779_I2CM_RX_BUFFER_22,0x41,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_22	0x41

/*
 * MAX77779_I2CM_RX_BUFFER_23,0x42,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_23	0x42

/*
 * MAX77779_I2CM_RX_BUFFER_24,0x43,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_24	0x43

/*
 * MAX77779_I2CM_RX_BUFFER_25,0x44,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_25	0x44

/*
 * MAX77779_I2CM_RX_BUFFER_26,0x45,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_26	0x45

/*
 * MAX77779_I2CM_RX_BUFFER_27,0x46,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_27	0x46

/*
 * MAX77779_I2CM_RX_BUFFER_28,0x47,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_28	0x47

/*
 * MAX77779_I2CM_RX_BUFFER_29,0x48,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_29	0x48

/*
 * MAX77779_I2CM_RX_BUFFER_30,0x49,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_30	0x49

/*
 * MAX77779_I2CM_RX_BUFFER_31,0x4A,0b00000000,0x0,Reset_Type:IM
 */
#define MAX77779_I2CM_RX_BUFFER_31	0x4a

/*******************************************************
 * Section: BATTVIMON_FUNC 0x00 16
 */

/*
 * MAX77779_BVIM_INT_STS,0x00,0b00000000,0x0,Reset_Type:F
 * BVIM_Samples_Rdy
 */
#define MAX77779_BVIM_INT_STS	0x00
#define MAX77779_BVIM_INT_STS_BVIM_Samples_Rdy_SHIFT	0
#define MAX77779_BVIM_INT_STS_BVIM_Samples_Rdy_MASK	(0x1 << 0)
#define MAX77779_BVIM_INT_STS_BVIM_Samples_Rdy_CLEAR	(~(0x1 << 0))
static inline const char *
max77779_bvim_int_sts_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BVIM_Samples_Rdy=%x",
		FIELD2VALUE(MAX77779_BVIM_Samples_Rdy, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_int_sts_bvim_samples_rdy,0,0)

/*
 * MAX77779_BVIM_MASK,0x01,0b00000001,0x1,Reset_Type:F
 * BVIM_Samples_Rdy_m
 */
#define MAX77779_BVIM_MASK	0x01
#define MAX77779_BVIM_MASK_BVIM_Samples_Rdy_m_SHIFT	0
#define MAX77779_BVIM_MASK_BVIM_Samples_Rdy_m_MASK	(0x1 << 0)
#define MAX77779_BVIM_MASK_BVIM_Samples_Rdy_m_CLEAR	(~(0x1 << 0))
static inline const char *
max77779_bvim_mask_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BVIM_Samples_Rdy_m=%x",
		FIELD2VALUE(MAX77779_BVIM_Samples_Rdy_m, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_mask_bvim_samples_rdy_m,0,0)

/*
 * MAX77779_BVIM_CTRL,0x10,0b00000000,0x0,Reset_Type:F
 * BVIMON_TRIG
 */
#define MAX77779_BVIM_CTRL	0x10
#define MAX77779_BVIM_CTRL_BVIMON_TRIG_SHIFT	0
#define MAX77779_BVIM_CTRL_BVIMON_TRIG_MASK	(0x1 << 0)
#define MAX77779_BVIM_CTRL_BVIMON_TRIG_CLEAR	(~(0x1 << 0))
static inline const char *
max77779_bvim_ctrl_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BVIMON_TRIG=%x",
		FIELD2VALUE(MAX77779_BVIMON_TRIG, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_ctrl_bvimon_trig,0,0)

/*******************************************************
 * Section: BVIM_CONTROL 0x60 16
 */

/*
 * MAX77779_BVIM_bvim_cfg,0x00,0b00000000,0x0,Reset_Type:S
 * smpl_n[0:8],smpl_m[8:3],cnt_run,batoiolo1_stop,batoiolo2_stop,top_fault_stop,
 * vioaok_stop
 */
#define MAX77779_BVIM_bvim_cfg	0x60
#define MAX77779_BVIM_bvim_cfg_smpl_n_SHIFT	0
#define MAX77779_BVIM_bvim_cfg_smpl_n_MASK	(0xff << 0)
#define MAX77779_BVIM_bvim_cfg_smpl_n_CLEAR	(~(0xff << 0))
#define MAX77779_BVIM_bvim_cfg_smpl_m_SHIFT	8
#define MAX77779_BVIM_bvim_cfg_smpl_m_MASK	(0x7 << 8)
#define MAX77779_BVIM_bvim_cfg_smpl_m_CLEAR	(~(0x7 << 8))
#define MAX77779_BVIM_bvim_cfg_cnt_run_SHIFT	11
#define MAX77779_BVIM_bvim_cfg_cnt_run_MASK	(0x1 << 11)
#define MAX77779_BVIM_bvim_cfg_cnt_run_CLEAR	(~(0x1 << 11))
#define MAX77779_BVIM_bvim_cfg_batoiolo1_stop_SHIFT	12
#define MAX77779_BVIM_bvim_cfg_batoiolo1_stop_MASK	(0x1 << 12)
#define MAX77779_BVIM_bvim_cfg_batoiolo1_stop_CLEAR	(~(0x1 << 12))
#define MAX77779_BVIM_bvim_cfg_batoiolo2_stop_SHIFT	13
#define MAX77779_BVIM_bvim_cfg_batoiolo2_stop_MASK	(0x1 << 13)
#define MAX77779_BVIM_bvim_cfg_batoiolo2_stop_CLEAR	(~(0x1 << 13))
#define MAX77779_BVIM_bvim_cfg_top_fault_stop_SHIFT	14
#define MAX77779_BVIM_bvim_cfg_top_fault_stop_MASK	(0x1 << 14)
#define MAX77779_BVIM_bvim_cfg_top_fault_stop_CLEAR	(~(0x1 << 14))
#define MAX77779_BVIM_bvim_cfg_vioaok_stop_SHIFT	15
#define MAX77779_BVIM_bvim_cfg_vioaok_stop_MASK	(0x1 << 15)
#define MAX77779_BVIM_bvim_cfg_vioaok_stop_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_bvim_bvim_cfg_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " smpl_n=%x",
		FIELD2VALUE(MAX77779_smpl_n, val));
	i += scnprintf(&buff[i], len - i, " smpl_m=%x",
		FIELD2VALUE(MAX77779_smpl_m, val));
	i += scnprintf(&buff[i], len - i, " cnt_run=%x",
		FIELD2VALUE(MAX77779_cnt_run, val));
	i += scnprintf(&buff[i], len - i, " batoiolo1_stop=%x",
		FIELD2VALUE(MAX77779_batoiolo1_stop, val));
	i += scnprintf(&buff[i], len - i, " batoiolo2_stop=%x",
		FIELD2VALUE(MAX77779_batoiolo2_stop, val));
	i += scnprintf(&buff[i], len - i, " top_fault_stop=%x",
		FIELD2VALUE(MAX77779_top_fault_stop, val));
	i += scnprintf(&buff[i], len - i, " vioaok_stop=%x",
		FIELD2VALUE(MAX77779_vioaok_stop, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_bvim_cfg_smpl_n,7,0)
MAX77779_BFF(max77779_bvim_bvim_cfg_smpl_m,10,8)
MAX77779_BFF(max77779_bvim_bvim_cfg_cnt_run,11,11)
MAX77779_BFF(max77779_bvim_bvim_cfg_batoiolo1_stop,12,12)
MAX77779_BFF(max77779_bvim_bvim_cfg_batoiolo2_stop,13,13)
MAX77779_BFF(max77779_bvim_bvim_cfg_top_fault_stop,14,14)
MAX77779_BFF(max77779_bvim_bvim_cfg_vioaok_stop,15,15)

/*
 * MAX77779_BVIM_smpl_math,0x01,0b00000000,0x0,Reset_Type:S
 * math_avg,math_min,math_max,SPR_6_3[3:4],smpl_start_add[7:9]
 */
#define MAX77779_BVIM_smpl_math	0x61
#define MAX77779_BVIM_smpl_math_math_avg_SHIFT	0
#define MAX77779_BVIM_smpl_math_math_avg_MASK	(0x1 << 0)
#define MAX77779_BVIM_smpl_math_math_avg_CLEAR	(~(0x1 << 0))
#define MAX77779_BVIM_smpl_math_math_min_SHIFT	1
#define MAX77779_BVIM_smpl_math_math_min_MASK	(0x1 << 1)
#define MAX77779_BVIM_smpl_math_math_min_CLEAR	(~(0x1 << 1))
#define MAX77779_BVIM_smpl_math_math_max_SHIFT	2
#define MAX77779_BVIM_smpl_math_math_max_MASK	(0x1 << 2)
#define MAX77779_BVIM_smpl_math_math_max_CLEAR	(~(0x1 << 2))
#define MAX77779_BVIM_smpl_math_SPR_6_3_SHIFT	3
#define MAX77779_BVIM_smpl_math_SPR_6_3_MASK	(0xf << 3)
#define MAX77779_BVIM_smpl_math_SPR_6_3_CLEAR	(~(0xf << 3))
#define MAX77779_BVIM_smpl_math_smpl_start_add_SHIFT	7
#define MAX77779_BVIM_smpl_math_smpl_start_add_MASK	(0x1ff << 7)
#define MAX77779_BVIM_smpl_math_smpl_start_add_CLEAR	(~(0x1ff << 7))
static inline const char *
max77779_bvim_smpl_math_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " math_avg=%x",
		FIELD2VALUE(MAX77779_math_avg, val));
	i += scnprintf(&buff[i], len - i, " math_min=%x",
		FIELD2VALUE(MAX77779_math_min, val));
	i += scnprintf(&buff[i], len - i, " math_max=%x",
		FIELD2VALUE(MAX77779_math_max, val));
	i += scnprintf(&buff[i], len - i, " SPR_6_3=%x",
		FIELD2VALUE(MAX77779_SPR_6_3, val));
	i += scnprintf(&buff[i], len - i, " smpl_start_add=%x",
		FIELD2VALUE(MAX77779_smpl_start_add, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_smpl_math_math_avg,0,0)
MAX77779_BFF(max77779_bvim_smpl_math_math_min,1,1)
MAX77779_BFF(max77779_bvim_smpl_math_math_max,2,2)
MAX77779_BFF(max77779_bvim_smpl_math_spr_6_3,6,3)
MAX77779_BFF(max77779_bvim_smpl_math_smpl_start_add,15,7)

/*
 * MAX77779_BVIM_bvim_trig,0x02,0b00000000,0x0,Reset_Type:S
 * trig_now,batoilo1_tr,batoilo2_tr,sysuvlo1_tr,sysuvlo2_tr,vbatt_tr,
 * ibatt_tr,SPR_7,vbatt_avg_tr,vbatt_min_tr,vbatt_max_tr,ibatt_avg_tr,ibatt_min_tr,ibatt_max_tr,oilo_start_source,oilo_stop_source
 */
#define MAX77779_BVIM_bvim_trig	0x62
#define MAX77779_BVIM_bvim_trig_trig_now_SHIFT	0
#define MAX77779_BVIM_bvim_trig_trig_now_MASK	(0x1 << 0)
#define MAX77779_BVIM_bvim_trig_trig_now_CLEAR	(~(0x1 << 0))
#define MAX77779_BVIM_bvim_trig_batoilo1_tr_SHIFT	1
#define MAX77779_BVIM_bvim_trig_batoilo1_tr_MASK	(0x1 << 1)
#define MAX77779_BVIM_bvim_trig_batoilo1_tr_CLEAR	(~(0x1 << 1))
#define MAX77779_BVIM_bvim_trig_batoilo2_tr_SHIFT	2
#define MAX77779_BVIM_bvim_trig_batoilo2_tr_MASK	(0x1 << 2)
#define MAX77779_BVIM_bvim_trig_batoilo2_tr_CLEAR	(~(0x1 << 2))
#define MAX77779_BVIM_bvim_trig_sysuvlo1_tr_SHIFT	3
#define MAX77779_BVIM_bvim_trig_sysuvlo1_tr_MASK	(0x1 << 3)
#define MAX77779_BVIM_bvim_trig_sysuvlo1_tr_CLEAR	(~(0x1 << 3))
#define MAX77779_BVIM_bvim_trig_sysuvlo2_tr_SHIFT	4
#define MAX77779_BVIM_bvim_trig_sysuvlo2_tr_MASK	(0x1 << 4)
#define MAX77779_BVIM_bvim_trig_sysuvlo2_tr_CLEAR	(~(0x1 << 4))
#define MAX77779_BVIM_bvim_trig_vbatt_tr_SHIFT	5
#define MAX77779_BVIM_bvim_trig_vbatt_tr_MASK	(0x1 << 5)
#define MAX77779_BVIM_bvim_trig_vbatt_tr_CLEAR	(~(0x1 << 5))
#define MAX77779_BVIM_bvim_trig_ibatt_tr_SHIFT	6
#define MAX77779_BVIM_bvim_trig_ibatt_tr_MASK	(0x1 << 6)
#define MAX77779_BVIM_bvim_trig_ibatt_tr_CLEAR	(~(0x1 << 6))
#define MAX77779_BVIM_bvim_trig_SPR_7_SHIFT	7
#define MAX77779_BVIM_bvim_trig_SPR_7_MASK	(0x1 << 7)
#define MAX77779_BVIM_bvim_trig_SPR_7_CLEAR	(~(0x1 << 7))
#define MAX77779_BVIM_bvim_trig_vbatt_avg_tr_SHIFT	8
#define MAX77779_BVIM_bvim_trig_vbatt_avg_tr_MASK	(0x1 << 8)
#define MAX77779_BVIM_bvim_trig_vbatt_avg_tr_CLEAR	(~(0x1 << 8))
#define MAX77779_BVIM_bvim_trig_vbatt_min_tr_SHIFT	9
#define MAX77779_BVIM_bvim_trig_vbatt_min_tr_MASK	(0x1 << 9)
#define MAX77779_BVIM_bvim_trig_vbatt_min_tr_CLEAR	(~(0x1 << 9))
#define MAX77779_BVIM_bvim_trig_vbatt_max_tr_SHIFT	10
#define MAX77779_BVIM_bvim_trig_vbatt_max_tr_MASK	(0x1 << 10)
#define MAX77779_BVIM_bvim_trig_vbatt_max_tr_CLEAR	(~(0x1 << 10))
#define MAX77779_BVIM_bvim_trig_ibatt_avg_tr_SHIFT	11
#define MAX77779_BVIM_bvim_trig_ibatt_avg_tr_MASK	(0x1 << 11)
#define MAX77779_BVIM_bvim_trig_ibatt_avg_tr_CLEAR	(~(0x1 << 11))
#define MAX77779_BVIM_bvim_trig_ibatt_min_tr_SHIFT	12
#define MAX77779_BVIM_bvim_trig_ibatt_min_tr_MASK	(0x1 << 12)
#define MAX77779_BVIM_bvim_trig_ibatt_min_tr_CLEAR	(~(0x1 << 12))
#define MAX77779_BVIM_bvim_trig_ibatt_max_tr_SHIFT	13
#define MAX77779_BVIM_bvim_trig_ibatt_max_tr_MASK	(0x1 << 13)
#define MAX77779_BVIM_bvim_trig_ibatt_max_tr_CLEAR	(~(0x1 << 13))
#define MAX77779_BVIM_bvim_trig_oilo_start_source_SHIFT	14
#define MAX77779_BVIM_bvim_trig_oilo_start_source_MASK	(0x1 << 14)
#define MAX77779_BVIM_bvim_trig_oilo_start_source_CLEAR	(~(0x1 << 14))
#define MAX77779_BVIM_bvim_trig_oilo_stop_source_SHIFT	15
#define MAX77779_BVIM_bvim_trig_oilo_stop_source_MASK	(0x1 << 15)
#define MAX77779_BVIM_bvim_trig_oilo_stop_source_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_bvim_bvim_trig_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " trig_now=%x",
		FIELD2VALUE(MAX77779_trig_now, val));
	i += scnprintf(&buff[i], len - i, " batoilo1_tr=%x",
		FIELD2VALUE(MAX77779_batoilo1_tr, val));
	i += scnprintf(&buff[i], len - i, " batoilo2_tr=%x",
		FIELD2VALUE(MAX77779_batoilo2_tr, val));
	i += scnprintf(&buff[i], len - i, " sysuvlo1_tr=%x",
		FIELD2VALUE(MAX77779_sysuvlo1_tr, val));
	i += scnprintf(&buff[i], len - i, " sysuvlo2_tr=%x",
		FIELD2VALUE(MAX77779_sysuvlo2_tr, val));
	i += scnprintf(&buff[i], len - i, " vbatt_tr=%x",
		FIELD2VALUE(MAX77779_vbatt_tr, val));
	i += scnprintf(&buff[i], len - i, " ibatt_tr=%x",
		FIELD2VALUE(MAX77779_ibatt_tr, val));
	i += scnprintf(&buff[i], len - i, " SPR_7=%x",
		FIELD2VALUE(MAX77779_SPR_7, val));
	i += scnprintf(&buff[i], len - i, " vbatt_avg_tr=%x",
		FIELD2VALUE(MAX77779_vbatt_avg_tr, val));
	i += scnprintf(&buff[i], len - i, " vbatt_min_tr=%x",
		FIELD2VALUE(MAX77779_vbatt_min_tr, val));
	i += scnprintf(&buff[i], len - i, " vbatt_max_tr=%x",
		FIELD2VALUE(MAX77779_vbatt_max_tr, val));
	i += scnprintf(&buff[i], len - i, " ibatt_avg_tr=%x",
		FIELD2VALUE(MAX77779_ibatt_avg_tr, val));
	i += scnprintf(&buff[i], len - i, " ibatt_min_tr=%x",
		FIELD2VALUE(MAX77779_ibatt_min_tr, val));
	i += scnprintf(&buff[i], len - i, " ibatt_max_tr=%x",
		FIELD2VALUE(MAX77779_ibatt_max_tr, val));
	i += scnprintf(&buff[i], len - i, " oilo_start_source=%x",
		FIELD2VALUE(MAX77779_oilo_start_source, val));
	i += scnprintf(&buff[i], len - i, " oilo_stop_source=%x",
		FIELD2VALUE(MAX77779_oilo_stop_source, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_bvim_trig_trig_now,0,0)
MAX77779_BFF(max77779_bvim_bvim_trig_batoilo1_tr,1,1)
MAX77779_BFF(max77779_bvim_bvim_trig_batoilo2_tr,2,2)
MAX77779_BFF(max77779_bvim_bvim_trig_sysuvlo1_tr,3,3)
MAX77779_BFF(max77779_bvim_bvim_trig_sysuvlo2_tr,4,4)
MAX77779_BFF(max77779_bvim_bvim_trig_vbatt_tr,5,5)
MAX77779_BFF(max77779_bvim_bvim_trig_ibatt_tr,6,6)
MAX77779_BFF(max77779_bvim_bvim_trig_spr_7,7,7)
MAX77779_BFF(max77779_bvim_bvim_trig_vbatt_avg_tr,8,8)
MAX77779_BFF(max77779_bvim_bvim_trig_vbatt_min_tr,9,9)
MAX77779_BFF(max77779_bvim_bvim_trig_vbatt_max_tr,10,10)
MAX77779_BFF(max77779_bvim_bvim_trig_ibatt_avg_tr,11,11)
MAX77779_BFF(max77779_bvim_bvim_trig_ibatt_min_tr,12,12)
MAX77779_BFF(max77779_bvim_bvim_trig_ibatt_max_tr,13,13)
MAX77779_BFF(max77779_bvim_bvim_trig_oilo_start_source,14,14)
MAX77779_BFF(max77779_bvim_bvim_trig_oilo_stop_source,15,15)

/*
 * MAX77779_BVIM_bvimtr,0x03,0b00000000,0x0,Reset_Type:S
 * v_md[0:2],i_md[2:2],v_savg_md[4:2],v_smin_md[6:2],v_smax_md[8:2],i_savg_md[10:2],
 * i_smin_md[12:2],i_smax_md[14:2]
 */
#define MAX77779_BVIM_bvimtr	0x63
#define MAX77779_BVIM_bvimtr_v_md_SHIFT	0
#define MAX77779_BVIM_bvimtr_v_md_MASK	(0x3 << 0)
#define MAX77779_BVIM_bvimtr_v_md_CLEAR	(~(0x3 << 0))
#define MAX77779_BVIM_bvimtr_i_md_SHIFT	2
#define MAX77779_BVIM_bvimtr_i_md_MASK	(0x3 << 2)
#define MAX77779_BVIM_bvimtr_i_md_CLEAR	(~(0x3 << 2))
#define MAX77779_BVIM_bvimtr_v_savg_md_SHIFT	4
#define MAX77779_BVIM_bvimtr_v_savg_md_MASK	(0x3 << 4)
#define MAX77779_BVIM_bvimtr_v_savg_md_CLEAR	(~(0x3 << 4))
#define MAX77779_BVIM_bvimtr_v_smin_md_SHIFT	6
#define MAX77779_BVIM_bvimtr_v_smin_md_MASK	(0x3 << 6)
#define MAX77779_BVIM_bvimtr_v_smin_md_CLEAR	(~(0x3 << 6))
#define MAX77779_BVIM_bvimtr_v_smax_md_SHIFT	8
#define MAX77779_BVIM_bvimtr_v_smax_md_MASK	(0x3 << 8)
#define MAX77779_BVIM_bvimtr_v_smax_md_CLEAR	(~(0x3 << 8))
#define MAX77779_BVIM_bvimtr_i_savg_md_SHIFT	10
#define MAX77779_BVIM_bvimtr_i_savg_md_MASK	(0x3 << 10)
#define MAX77779_BVIM_bvimtr_i_savg_md_CLEAR	(~(0x3 << 10))
#define MAX77779_BVIM_bvimtr_i_smin_md_SHIFT	12
#define MAX77779_BVIM_bvimtr_i_smin_md_MASK	(0x3 << 12)
#define MAX77779_BVIM_bvimtr_i_smin_md_CLEAR	(~(0x3 << 12))
#define MAX77779_BVIM_bvimtr_i_smax_md_SHIFT	14
#define MAX77779_BVIM_bvimtr_i_smax_md_MASK	(0x3 << 14)
#define MAX77779_BVIM_bvimtr_i_smax_md_CLEAR	(~(0x3 << 14))
static inline const char *
max77779_bvim_bvimtr_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " v_md=%x",
		FIELD2VALUE(MAX77779_v_md, val));
	i += scnprintf(&buff[i], len - i, " i_md=%x",
		FIELD2VALUE(MAX77779_i_md, val));
	i += scnprintf(&buff[i], len - i, " v_savg_md=%x",
		FIELD2VALUE(MAX77779_v_savg_md, val));
	i += scnprintf(&buff[i], len - i, " v_smin_md=%x",
		FIELD2VALUE(MAX77779_v_smin_md, val));
	i += scnprintf(&buff[i], len - i, " v_smax_md=%x",
		FIELD2VALUE(MAX77779_v_smax_md, val));
	i += scnprintf(&buff[i], len - i, " i_savg_md=%x",
		FIELD2VALUE(MAX77779_i_savg_md, val));
	i += scnprintf(&buff[i], len - i, " i_smin_md=%x",
		FIELD2VALUE(MAX77779_i_smin_md, val));
	i += scnprintf(&buff[i], len - i, " i_smax_md=%x",
		FIELD2VALUE(MAX77779_i_smax_md, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_bvimtr_v_md,1,0)
MAX77779_BFF(max77779_bvim_bvimtr_i_md,3,2)
MAX77779_BFF(max77779_bvim_bvimtr_v_savg_md,5,4)
MAX77779_BFF(max77779_bvim_bvimtr_v_smin_md,7,6)
MAX77779_BFF(max77779_bvim_bvimtr_v_smax_md,9,8)
MAX77779_BFF(max77779_bvim_bvimtr_i_savg_md,11,10)
MAX77779_BFF(max77779_bvim_bvimtr_i_smin_md,13,12)
MAX77779_BFF(max77779_bvim_bvimtr_i_smax_md,15,14)

/*
 * MAX77779_BVIM_bvim_vtr_mth,0x04,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_vtr_mth	0x64

/*
 * MAX77779_BVIM_bvim_itr_mth,0x05,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_itr_mth	0x65

/*
 * MAX77779_BVIM_bvim_vtr_smin_th,0x06,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_vtr_smin_th	0x66

/*
 * MAX77779_BVIM_bvim_vtr_smax_th,0x07,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_vtr_smax_th	0x67

/*
 * MAX77779_BVIM_bvim_vtr_savg_th,0x08,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_vtr_savg_th	0x68

/*
 * MAX77779_BVIM_bvim_itr_smin_th,0x09,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_itr_smin_th	0x69

/*
 * MAX77779_BVIM_bvim_itr_smax_th,0x0A,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_itr_smax_th	0x6a

/*
 * MAX77779_BVIM_bvim_itr_savg_th,0x0B,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_itr_savg_th	0x6b

/*
 * MAX77779_BVIM_bvim_sts,0x0C,0b00000000,0x0,Reset_Type:S
 * bvim_osc[0:9],tr_sts[9:2],bvim_stopped,stop_sts_oilo1,stop_sts_oilo2,
 * stop_sts_fault,stop_sts_vioaok
 */
#define MAX77779_BVIM_bvim_sts	0x6c
#define MAX77779_BVIM_bvim_sts_bvim_osc_SHIFT	0
#define MAX77779_BVIM_bvim_sts_bvim_osc_MASK	(0x1ff << 0)
#define MAX77779_BVIM_bvim_sts_bvim_osc_CLEAR	(~(0x1ff << 0))
#define MAX77779_BVIM_bvim_sts_tr_sts_SHIFT	9
#define MAX77779_BVIM_bvim_sts_tr_sts_MASK	(0x3 << 9)
#define MAX77779_BVIM_bvim_sts_tr_sts_CLEAR	(~(0x3 << 9))
#define MAX77779_BVIM_bvim_sts_bvim_stopped_SHIFT	11
#define MAX77779_BVIM_bvim_sts_bvim_stopped_MASK	(0x1 << 11)
#define MAX77779_BVIM_bvim_sts_bvim_stopped_CLEAR	(~(0x1 << 11))
#define MAX77779_BVIM_bvim_sts_stop_sts_oilo1_SHIFT	12
#define MAX77779_BVIM_bvim_sts_stop_sts_oilo1_MASK	(0x1 << 12)
#define MAX77779_BVIM_bvim_sts_stop_sts_oilo1_CLEAR	(~(0x1 << 12))
#define MAX77779_BVIM_bvim_sts_stop_sts_oilo2_SHIFT	13
#define MAX77779_BVIM_bvim_sts_stop_sts_oilo2_MASK	(0x1 << 13)
#define MAX77779_BVIM_bvim_sts_stop_sts_oilo2_CLEAR	(~(0x1 << 13))
#define MAX77779_BVIM_bvim_sts_stop_sts_fault_SHIFT	14
#define MAX77779_BVIM_bvim_sts_stop_sts_fault_MASK	(0x1 << 14)
#define MAX77779_BVIM_bvim_sts_stop_sts_fault_CLEAR	(~(0x1 << 14))
#define MAX77779_BVIM_bvim_sts_stop_sts_vioaok_SHIFT	15
#define MAX77779_BVIM_bvim_sts_stop_sts_vioaok_MASK	(0x1 << 15)
#define MAX77779_BVIM_bvim_sts_stop_sts_vioaok_CLEAR	(~(0x1 << 15))
static inline const char *
max77779_bvim_bvim_sts_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " bvim_osc=%x",
		FIELD2VALUE(MAX77779_bvim_osc, val));
	i += scnprintf(&buff[i], len - i, " tr_sts=%x",
		FIELD2VALUE(MAX77779_tr_sts, val));
	i += scnprintf(&buff[i], len - i, " bvim_stopped=%x",
		FIELD2VALUE(MAX77779_bvim_stopped, val));
	i += scnprintf(&buff[i], len - i, " stop_sts_oilo1=%x",
		FIELD2VALUE(MAX77779_stop_sts_oilo1, val));
	i += scnprintf(&buff[i], len - i, " stop_sts_oilo2=%x",
		FIELD2VALUE(MAX77779_stop_sts_oilo2, val));
	i += scnprintf(&buff[i], len - i, " stop_sts_fault=%x",
		FIELD2VALUE(MAX77779_stop_sts_fault, val));
	i += scnprintf(&buff[i], len - i, " stop_sts_vioaok=%x",
		FIELD2VALUE(MAX77779_stop_sts_vioaok, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_bvim_sts_bvim_osc,8,0)
MAX77779_BFF(max77779_bvim_bvim_sts_tr_sts,10,9)
MAX77779_BFF(max77779_bvim_bvim_sts_bvim_stopped,11,11)
MAX77779_BFF(max77779_bvim_bvim_sts_stop_sts_oilo1,12,12)
MAX77779_BFF(max77779_bvim_bvim_sts_stop_sts_oilo2,13,13)
MAX77779_BFF(max77779_bvim_bvim_sts_stop_sts_fault,14,14)
MAX77779_BFF(max77779_bvim_bvim_sts_stop_sts_vioaok,15,15)

/*
 * MAX77779_BVIM_bvim_rs,0x0D,0b00000000,0x0,Reset_Type:S
 * rsc[0:9],bvim_rts[9:4],SPR_15_13[13:3]
 */
#define MAX77779_BVIM_bvim_rs	0x6d
#define MAX77779_BVIM_bvim_rs_rsc_SHIFT	0
#define MAX77779_BVIM_bvim_rs_rsc_MASK	(0x1ff << 0)
#define MAX77779_BVIM_bvim_rs_rsc_CLEAR	(~(0x1ff << 0))
#define MAX77779_BVIM_bvim_rs_bvim_rts_SHIFT	9
#define MAX77779_BVIM_bvim_rs_bvim_rts_MASK	(0xf << 9)
#define MAX77779_BVIM_bvim_rs_bvim_rts_CLEAR	(~(0xf << 9))
#define MAX77779_BVIM_bvim_rs_SPR_15_13_SHIFT	13
#define MAX77779_BVIM_bvim_rs_SPR_15_13_MASK	(0x7 << 13)
#define MAX77779_BVIM_bvim_rs_SPR_15_13_CLEAR	(~(0x7 << 13))
static inline const char *
max77779_bvim_bvim_rs_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " rsc=%x",
		FIELD2VALUE(MAX77779_rsc, val));
	i += scnprintf(&buff[i], len - i, " bvim_rts=%x",
		FIELD2VALUE(MAX77779_bvim_rts, val));
	i += scnprintf(&buff[i], len - i, " SPR_15_13=%x",
		FIELD2VALUE(MAX77779_SPR_15_13, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_bvim_rs_rsc,8,0)
MAX77779_BFF(max77779_bvim_bvim_rs_bvim_rts,12,9)
MAX77779_BFF(max77779_bvim_bvim_rs_spr_15_13,15,13)

/*
 * MAX77779_BVIM_bvim_rlap,0x0E,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_rlap	0x6e

/*
 * MAX77779_BVIM_bvim_rfap,0x0F,0b00000000,0x0,Reset_Type:S
 */
#define MAX77779_BVIM_bvim_rfap	0x6f

/*******************************************************
 * Section: BVIM_PAGE 0x70 16
 */

/*
 * MAX77779_BVIM_PAGE_CTRL,0x00,0b00000000,0x0,Reset_Type:F
 * BVIM_DATA_PAGE[0:2]
 */
#define MAX77779_BVIM_PAGE_CTRL	0x70
#define MAX77779_BVIM_PAGE_CTRL_BVIM_DATA_PAGE_SHIFT	0
#define MAX77779_BVIM_PAGE_CTRL_BVIM_DATA_PAGE_MASK	(0x3 << 0)
#define MAX77779_BVIM_PAGE_CTRL_BVIM_DATA_PAGE_CLEAR	(~(0x3 << 0))
static inline const char *
max77779_bvim_page_ctrl_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " BVIM_DATA_PAGE=%x",
		FIELD2VALUE(MAX77779_BVIM_DATA_PAGE, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_bvim_page_ctrl_bvim_data_page,1,0)

/*******************************************************
 * Section: BVIM_DATA 0x80 16
 */

/*******************************************************
 * Section: SP_PAGE 0x70 16
 */

/*
 * MAX77779_SP_PAGE_CTRL,0x00,0b00000000,0x0,Reset_Type:SP
 * SP_DATA_PAGE[0:2]
 */
#define MAX77779_SP_PAGE_CTRL	0x70
#define MAX77779_SP_PAGE_CTRL_SP_DATA_PAGE_SHIFT	0
#define MAX77779_SP_PAGE_CTRL_SP_DATA_PAGE_MASK	(0x3 << 0)
#define MAX77779_SP_PAGE_CTRL_SP_DATA_PAGE_CLEAR	(~(0x3 << 0))
static inline const char *
max77779_sp_page_ctrl_cstr(char *buff, size_t len, int val)
{
#ifdef CONFIG_SCNPRINTF_DEBUG
	int i = 0;

	i += scnprintf(&buff[i], len - i, " SP_DATA_PAGE=%x",
		FIELD2VALUE(MAX77779_SP_DATA_PAGE, val));
#else
	buff[0] = 0;
#endif
	return buff;
}

MAX77779_BFF(max77779_sp_page_ctrl_sp_data_page,1,0)

/*
 * MAX77779_SP_TST,0x0F,0b00000000,0x0,Reset_Type:SP
 */
#define MAX77779_SP_TST	0x7f

/*******************************************************
 * Section: SP_DATA 0x80 16
 */

#endif /* SEQUOIA_REGMAP_CUSTOMER_2024_01_15_V2_REG_H_ */
