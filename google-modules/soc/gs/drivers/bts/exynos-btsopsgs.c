// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License Version 2 as published
 * by the Free Software Foundation.
 *
 * BTS Bus Traffic Shaper device driver operation functions set
 *
 */

#include <linux/io.h>
#include <dt-bindings/soc/google/gs-bts.h>

#if ! IS_ENABLED(CONFIG_SOC_ZUMA)
#include "regs-btsgs101.h"
#else
#include "regs-btsgs.h"
#endif
#include "bts.h"

/****************************************************************
 *			init ops functions			*
 ****************************************************************/
static void init_trexbts(void __iomem *base)
{
	unsigned int w1;

	if (!base)
		return;

	/* high [27:24] mid [19:16] threshold */
	w1 = __raw_readl(base + SCI_CTRL);
	w1 |= (0xB << HIGH_THRESHOLD_SHIFT) | (0x7 << MID_THRESHOLD_SHIFT);
	__raw_writel(w1, base + SCI_CTRL);

	__raw_writel(0x10101010, base + TH_IMM_R_0);
	__raw_writel(0x10101010, base + TH_IMM_W_0);

	__raw_writel(0x08080808, base + TH_HIGH_R_0);
	__raw_writel(0x08080808, base + TH_HIGH_W_0);
}

static void init_sciqfull(void __iomem *base)
{
}

static void init_smcqbusy(void __iomem *base)
{
	unsigned int threshold = DEFAULT_QBUSY_TH;
	unsigned int tmp_reg;

	if (!base)
		return;

	if (threshold > 0x3F)
		threshold = 0x3F;

	tmp_reg = __raw_readl(base + SMC_SCHEDCTL_BUNDLE_CTRL4);
	tmp_reg &= ~(0x3F << 16);
	tmp_reg &= ~(0x3F << 24);
	tmp_reg |= (threshold << 16); //W
	tmp_reg |= (threshold << 24); //R

	__raw_writel(tmp_reg, base + SMC_SCHEDCTL_BUNDLE_CTRL4);
}

static void init_qmax(void __iomem *base)
{
	unsigned int threshold;
	unsigned int tmp_reg;

	if (!base)
		return;

	/* Read threshold set */
	threshold = DEFAULT_QMAX_RD_TH;

	if (threshold > 0xFFFF)
		threshold = 0xFFFF;

	tmp_reg = __raw_readl(base + QMAX_THRESHOLD_R);
	tmp_reg &= ~(0xFFFF << 0);
	tmp_reg |= (threshold << 0);

	__raw_writel(tmp_reg, base + QMAX_THRESHOLD_R);

	/* Write threshold set */
	threshold = DEFAULT_QMAX_WR_TH;

	if (threshold > 0xFFFF)
		threshold = 0xFFFF;

	tmp_reg = __raw_readl(base + QMAX_THRESHOLD_W);
	tmp_reg &= ~(0xFFFF << 0);
	tmp_reg |= (threshold << 0);

	__raw_writel(tmp_reg, base + QMAX_THRESHOLD_W);
}

/****************************************************************
 *			ip bts ops functions			*
 ****************************************************************/
static int set_ipbts_qos(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg_r;
	unsigned int tmp_reg_w;
	unsigned int tmp_reg;

	if (!base || !stat)
		return -ENODATA;

	/* xCON[8] = AxQoS override *1:override, 0:bypass */
	if (stat->bypass) {
		tmp_reg_r = __raw_readl(base + RCON);
		tmp_reg_r &= ~(1 << AXQOS_BYPASS);
		tmp_reg_w = __raw_readl(base + WCON);
		tmp_reg_w &= ~(1 << AXQOS_BYPASS);

		__raw_writel(tmp_reg_r, base + RCON);
		__raw_writel(tmp_reg_w, base + WCON);

		return 0;
	}

	if (stat->arqos > MAX_QOS)
		stat->arqos = MAX_QOS;

	/* xCON[15:12] = AxQoS value */
	tmp_reg_r = __raw_readl(base + RCON);
	tmp_reg_r &= ~((1 << AXQOS_BYPASS) | (MAX_QOS << AXQOS_VAL));
	tmp_reg_r |= (1 << AXQOS_BYPASS) | (stat->arqos << AXQOS_VAL);

	if (stat->awqos > MAX_QOS)
		stat->awqos = MAX_QOS;

	/* xCON[15:12] = AxQoS_value */
	tmp_reg_w = __raw_readl(base + WCON);
	tmp_reg_w &= ~((1 << AXQOS_BYPASS) | (MAX_QOS << AXQOS_VAL));
	tmp_reg_w |= (1 << AXQOS_BYPASS) | (stat->awqos << AXQOS_VAL);

	__raw_writel(tmp_reg_r, base + RCON);
	__raw_writel(tmp_reg_w, base + WCON);

	/* CONTROL[0] = QoS on/off */
	tmp_reg = __raw_readl(base + CON);
	tmp_reg &= ~(1 << AXQOS_ONOFF);
	tmp_reg |= (1 << AXQOS_ONOFF);
	__raw_writel(tmp_reg, base + CON);

	return 0;
}

static int get_ipbts_qos(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg_r;
	unsigned int tmp_reg_w;

	if (!base || !stat)
		return -ENODATA;

	tmp_reg_r = __raw_readl(base + RCON) & (1 << AXQOS_BYPASS);
	tmp_reg_r >>= AXQOS_BYPASS;
	tmp_reg_w = __raw_readl(base + WCON) & (1 << AXQOS_BYPASS);
	tmp_reg_w >>= AXQOS_BYPASS;

	if (!tmp_reg_r || !tmp_reg_w)
		stat->bypass = true;
	else
		stat->bypass = false;

	tmp_reg_r = __raw_readl(base + RCON) & (MAX_QOS << AXQOS_VAL);
	tmp_reg_r >>= AXQOS_VAL;

	tmp_reg_w = __raw_readl(base + WCON) & (MAX_QOS << AXQOS_VAL);
	tmp_reg_w >>= AXQOS_VAL;

	stat->arqos = tmp_reg_r;
	stat->awqos = tmp_reg_w;

	return 0;
}

static int set_ipbts_mo(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg_r;
	unsigned int tmp_reg_w;

	if (!base || !stat)
		return -ENODATA;

	if (stat->rmo > MAX_MO || !stat->rmo)
		stat->rmo = MAX_MO;

	tmp_reg_r = __raw_readl(base + RBLK_UPPER);
	tmp_reg_r &= ~(MAX_MO << BLOCK_UPPER);
	tmp_reg_r |= (stat->rmo << BLOCK_UPPER);

	if (stat->wmo > MAX_MO || !stat->wmo)
		stat->wmo = MAX_MO;

	tmp_reg_w = __raw_readl(base + WBLK_UPPER);
	tmp_reg_w &= ~(MAX_MO << BLOCK_UPPER);
	tmp_reg_w |= (stat->wmo << BLOCK_UPPER);

	__raw_writel(tmp_reg_r, base + RBLK_UPPER);
	__raw_writel(tmp_reg_w, base + WBLK_UPPER);

	return 0;
}

static int get_ipbts_mo(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg_r;
	unsigned int tmp_reg_w;

	if (!base || !stat)
		return -ENODATA;

	tmp_reg_r = __raw_readl(base + RBLK_UPPER) & (MAX_MO << BLOCK_UPPER);
	tmp_reg_r >>= BLOCK_UPPER;

	tmp_reg_w = __raw_readl(base + WBLK_UPPER) & (MAX_MO << BLOCK_UPPER);
	tmp_reg_w >>= BLOCK_UPPER;

	stat->rmo = tmp_reg_r;
	stat->wmo = tmp_reg_w;

	return 0;
}

static int set_ipbts_urgent(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg;

	if (!base || !stat)
		return -ENODATA;

	if (stat->qurgent_th_r > MAX_QUTH)
		stat->qurgent_th_r = MAX_QUTH;
	if (stat->qurgent_th_w > MAX_QUTH)
		stat->qurgent_th_w = MAX_QUTH;

	/* Read QUrgent */
	tmp_reg = __raw_readl(base + TIMEOUT_R0);
	tmp_reg = tmp_reg & ~((MAX_QUTH << TIMEOUT_CNT_VC0)
			| (MAX_QUTH << TIMEOUT_CNT_VC1)
			| (MAX_QUTH << TIMEOUT_CNT_VC2)
			| (MAX_QUTH << TIMEOUT_CNT_VC3));
	tmp_reg = tmp_reg | (stat->qurgent_th_r << TIMEOUT_CNT_VC0)
		| (stat->qurgent_th_r << TIMEOUT_CNT_VC1)
		| (stat->qurgent_th_r << TIMEOUT_CNT_VC2)
		| (stat->qurgent_th_r << TIMEOUT_CNT_VC3);

	__raw_writel(tmp_reg, base + TIMEOUT_R0);

	/* Write QUrgent */
	tmp_reg = __raw_readl(base + TIMEOUT_W0);
	tmp_reg = tmp_reg & ~((MAX_QUTH << TIMEOUT_CNT_VC0)
			| (MAX_QUTH << TIMEOUT_CNT_VC1)
			| (MAX_QUTH << TIMEOUT_CNT_VC2)
			| (MAX_QUTH << TIMEOUT_CNT_VC3));

	tmp_reg = tmp_reg | (stat->qurgent_th_w << TIMEOUT_CNT_VC0)
		| (stat->qurgent_th_w << TIMEOUT_CNT_VC1)
		| (stat->qurgent_th_w << TIMEOUT_CNT_VC2)
		| (stat->qurgent_th_w << TIMEOUT_CNT_VC3);

	__raw_writel(tmp_reg, base + TIMEOUT_W0);

	tmp_reg = __raw_readl(base + CON);
	tmp_reg &= ~((1 << QURGENT_EN) | (1 << EX_QURGENT_EN));

	if (stat->qurgent_on)
		tmp_reg |= (1 << QURGENT_EN);
	if (stat->ex_qurgent_on)
		tmp_reg |= (1 << EX_QURGENT_EN);

	__raw_writel(tmp_reg, base + CON);

	return 0;
}

static int get_ipbts_urgent(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg;

	if (!base || !stat)
		return -ENODATA;

	tmp_reg = __raw_readl(base + TIMEOUT_R0);
	stat->qurgent_th_r = (tmp_reg & (MAX_QUTH << TIMEOUT_CNT_VC0)) >> TIMEOUT_CNT_VC0;
	tmp_reg = __raw_readl(base + TIMEOUT_W0);
	stat->qurgent_th_w = (tmp_reg & (MAX_QUTH << TIMEOUT_CNT_VC0)) >> TIMEOUT_CNT_VC0;

	tmp_reg = __raw_readl(base + CON);
	stat->qurgent_on = (tmp_reg & (1 << QURGENT_EN)) >> QURGENT_EN;
	stat->ex_qurgent_on = (tmp_reg & (1 << EX_QURGENT_EN)) >> EX_QURGENT_EN;

	return 0;
}

static int set_ipbts_blocking(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg_r;
	unsigned int tmp_reg_w;

	if (!base || !stat)
		return -ENODATA;

	if (stat->blocking_on) {
		if (stat->qfull_limit_r > MAX_MO)
			stat->qfull_limit_r = MAX_MO;
		if (stat->qfull_limit_w > MAX_MO)
			stat->qfull_limit_w = MAX_MO;

		tmp_reg_r = __raw_readl(base + RBLK_UPPER_FULL);
		tmp_reg_r &= ~(MAX_MO << BLOCK_UPPER);
		tmp_reg_r |= (stat->qfull_limit_r << BLOCK_UPPER);

		tmp_reg_w = __raw_readl(base + WBLK_UPPER_FULL);
		tmp_reg_w &= ~(MAX_MO << BLOCK_UPPER);
		tmp_reg_w |= (stat->qfull_limit_w << BLOCK_UPPER);

		__raw_writel(tmp_reg_r, base + RBLK_UPPER_FULL);
		__raw_writel(tmp_reg_w, base + WBLK_UPPER_FULL);

		if (stat->qbusy_limit_r > MAX_MO)
			stat->qbusy_limit_r = MAX_MO;
		if (stat->qbusy_limit_w > MAX_MO)
			stat->qbusy_limit_w = MAX_MO;

		tmp_reg_r = __raw_readl(base + RBLK_UPPER_BUSY);
		tmp_reg_r &= ~(MAX_MO << BLOCK_UPPER);
		tmp_reg_r |= (stat->qbusy_limit_r << BLOCK_UPPER);

		tmp_reg_w = __raw_readl(base + WBLK_UPPER_BUSY);
		tmp_reg_w &= ~(MAX_MO << BLOCK_UPPER);
		tmp_reg_w |= (stat->qbusy_limit_w << BLOCK_UPPER);

		__raw_writel(tmp_reg_r, base + RBLK_UPPER_BUSY);
		__raw_writel(tmp_reg_w, base + WBLK_UPPER_BUSY);

		if (stat->qmax0_limit_r > MAX_MO)
			stat->qmax0_limit_r = MAX_MO;
		if (stat->qmax0_limit_w > MAX_MO)
			stat->qmax0_limit_w = MAX_MO;

		if (stat->qmax1_limit_r > MAX_MO)
			stat->qmax1_limit_r = MAX_MO;
		if (stat->qmax1_limit_w > MAX_MO)
			stat->qmax1_limit_w = MAX_MO;

		tmp_reg_r = __raw_readl(base + RBLK_UPPER_MAX);
		tmp_reg_r &=
			~((MAX_MO << BLOCK_UPPER1) | (MAX_MO << BLOCK_UPPER0));
		tmp_reg_r |= ((stat->qmax1_limit_r << BLOCK_UPPER1) |
			      (stat->qmax0_limit_r << BLOCK_UPPER0));

		tmp_reg_w = __raw_readl(base + WBLK_UPPER_MAX);
		tmp_reg_w &=
			~((MAX_MO << BLOCK_UPPER1) | (MAX_MO << BLOCK_UPPER0));
		tmp_reg_w |= ((stat->qmax1_limit_w << BLOCK_UPPER1) |
			      (stat->qmax0_limit_w << BLOCK_UPPER0));

		__raw_writel(tmp_reg_r, base + RBLK_UPPER_MAX);
		__raw_writel(tmp_reg_w, base + WBLK_UPPER_MAX);

		tmp_reg_r = __raw_readl(base + RCON);
		tmp_reg_r &= ~(1 << BLOCKING_EN);
		tmp_reg_r |= (1 << BLOCKING_EN);
		__raw_writel(tmp_reg_r, base + RCON);

		tmp_reg_w = __raw_readl(base + WCON);
		tmp_reg_w &= ~(1 << BLOCKING_EN);
		tmp_reg_w |= (1 << BLOCKING_EN);
		__raw_writel(tmp_reg_w, base + WCON);
	} else {
		tmp_reg_r = __raw_readl(base + RCON);
		tmp_reg_r &= ~(1 << BLOCKING_EN);
		__raw_writel(tmp_reg_r, base + RCON);

		tmp_reg_w = __raw_readl(base + WCON);
		tmp_reg_w &= ~(1 << BLOCKING_EN);
		__raw_writel(tmp_reg_w, base + WCON);
	}

	return 0;
}

static int get_ipbts_blocking(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg;

	tmp_reg = __raw_readl(base + RCON);
	stat->blocking_on = (tmp_reg & (1 << BLOCKING_EN)) ? true : false;

	if (!stat->blocking_on)
		return 0;

	tmp_reg = __raw_readl(base + RBLK_UPPER_FULL);
	stat->qfull_limit_r =
		(tmp_reg & (MAX_MO << BLOCK_UPPER)) >> BLOCK_UPPER;

	tmp_reg = __raw_readl(base + WBLK_UPPER_FULL);
	stat->qfull_limit_w =
		(tmp_reg & (MAX_MO << BLOCK_UPPER)) >> BLOCK_UPPER;

	tmp_reg = __raw_readl(base + RBLK_UPPER_BUSY);
	stat->qbusy_limit_r =
		(tmp_reg & (MAX_MO << BLOCK_UPPER)) >> BLOCK_UPPER;

	tmp_reg = __raw_readl(base + WBLK_UPPER_BUSY);
	stat->qbusy_limit_w =
		(tmp_reg & (MAX_MO << BLOCK_UPPER)) >> BLOCK_UPPER;

	tmp_reg = __raw_readl(base + RBLK_UPPER_MAX);
	stat->qmax0_limit_r =
		(tmp_reg & (MAX_MO << BLOCK_UPPER0)) >> BLOCK_UPPER0;
	stat->qmax1_limit_r =
		(tmp_reg & (MAX_MO << BLOCK_UPPER1)) >> BLOCK_UPPER1;

	tmp_reg = __raw_readl(base + WBLK_UPPER_MAX);
	stat->qmax0_limit_w =
		(tmp_reg & (MAX_MO << BLOCK_UPPER0)) >> BLOCK_UPPER0;
	stat->qmax1_limit_w =
		(tmp_reg & (MAX_MO << BLOCK_UPPER1)) >> BLOCK_UPPER1;

	return 0;
}

static int set_ipbts(void __iomem *base, struct bts_stat *stat)
{
	int ret;

	if (!base || !stat)
		return -ENODATA;

	if (!stat->stat_on)
		return 0;

	ret = set_ipbts_qos(base, stat);
	if (ret) {
		pr_err("set_ipbts_qos failed! ret=%d\n", ret);
		return ret;
	}
	ret = set_ipbts_mo(base, stat);
	if (ret) {
		pr_err("set_ipbts_mo failed! ret=%d\n", ret);
		return ret;
	}
	ret = set_ipbts_urgent(base, stat);
	if (ret) {
		pr_err("set_ipbts_urgent failed! ret=%d\n", ret);
		return ret;
	}
	ret = set_ipbts_blocking(base, stat);
	if (ret) {
		pr_err("set_ipbts_blocking failed! ret=%d\n", ret);
		return ret;
	}

	return ret;
}

static int get_ipbts(void __iomem *base, struct bts_stat *stat)
{
	int ret;

	if (!base || !stat)
		return -ENODATA;

	ret = get_ipbts_qos(base, stat);
	if (ret) {
		pr_err("get_ipbts_qos failed! ret=%d\n", ret);
		return ret;
	}
	ret = get_ipbts_mo(base, stat);
	if (ret) {
		pr_err("get_ipbts_mo failed! ret=%d\n", ret);
		return ret;
	}
	ret = get_ipbts_urgent(base, stat);
	if (ret) {
		pr_err("get_ipbts_urgent failed! ret=%d\n", ret);
		return ret;
	}
	ret = get_ipbts_blocking(base, stat);
	if (ret) {
		pr_err("get_ipbts_blocking failed! ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/****************************************************************
 *			sci bts ops functions			*
 ****************************************************************/
static int set_scibts_mo(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg;

	if (!base || !stat)
		return -ENODATA;

	if (stat->rmo > MAX_MO || !stat->rmo)
		stat->rmo = MAX_MO;

	if (stat->wmo > MAX_MO || !stat->wmo)
		stat->wmo = MAX_MO;

	/*
	 * CRP_CTL3_X (Olympus 0~3)
	 * MaxOutstandingPort0Rd [7:0]   (Olympus cpu use 0/3, not use 1/2)
	 * MaxOutstandingPort0Wr [15:8]  (Olympus cpu use 0/3, not use 1/2)
	 * MaxOutstandingPort1Rd [23:16] (Olympus gpu use 0/1/2/3)
	 * MaxOutstandingPort1Wr [31:24] (Olympus gpu use 0/1/2/3)
	 */
	tmp_reg = stat->rmo << RMO_PORT_1 | stat->wmo << WMO_PORT_1;

	__raw_writel(tmp_reg, base);

	return 0;
}

static int get_scibts_mo(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg;
	unsigned int tmp_reg_r;
	unsigned int tmp_reg_w;

	if (!base || !stat)
		return -ENODATA;

	tmp_reg = __raw_readl(base);

	tmp_reg_r = (tmp_reg & (0xFF << RMO_PORT_1)) >> RMO_PORT_1;
	tmp_reg_w = (tmp_reg & (0xFF << WMO_PORT_1)) >> WMO_PORT_1;

	stat->rmo = tmp_reg_r;
	stat->wmo = tmp_reg_w;

	return 0;
}

static int set_scibts_qos(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg;
	void __iomem *qos_base = NULL;

	if (!stat)
		return -ENODATA;

	qos_base = stat->qos_va_base;

	if (!qos_base)
		return 0;

	/* QoS on */
	if (stat->arqos || stat->awqos) { /* on */
		tmp_reg = __raw_readl(qos_base + CORE_QOS_EN);
		tmp_reg |= (1 << SCIQOS_EN);
		__raw_writel(tmp_reg, qos_base + CORE_QOS_EN);
	} else {
		/* QoS off */
		tmp_reg = __raw_readl(qos_base + CORE_QOS_EN);
		tmp_reg &= ~(1 << SCIQOS_EN);
		__raw_writel(tmp_reg, qos_base + CORE_QOS_EN);
	}

	if (stat->arqos > SCIMAX_QOS)
		stat->arqos = SCIMAX_QOS;

	if (stat->awqos > SCIMAX_QOS)
		stat->awqos = SCIMAX_QOS;

	/* AxQoS value */
	tmp_reg = __raw_readl(qos_base);
	tmp_reg &= ~((SCIMAX_QOS << SCIQOS_W) | (SCIMAX_QOS << SCIQOS_R));
	tmp_reg |= (stat->arqos << SCIQOS_R) | (stat->awqos << SCIQOS_W);

	__raw_writel(tmp_reg, qos_base);

	return 0;
}

static int get_scibts_qos(void __iomem *base, struct bts_stat *stat)
{
	unsigned int tmp_reg;
	unsigned int tmp_reg_r;
	unsigned int tmp_reg_w;
	void __iomem *qos_base;

	if (!stat)
		return -ENODATA;

	qos_base = stat->qos_va_base;

	if (!qos_base)
		return 0;

	tmp_reg = __raw_readl(qos_base + CORE_QOS_EN) & (1 << SCIQOS_EN);
	tmp_reg >>= SCIQOS_EN;

	if (tmp_reg)
		stat->bypass = false;
	else
		stat->bypass = true;

	tmp_reg_w = __raw_readl(qos_base) & (SCIMAX_QOS << SCIQOS_W);
	tmp_reg_w >>= SCIQOS_W;

	tmp_reg_r = __raw_readl(qos_base) & (SCIMAX_QOS << SCIQOS_R);
	tmp_reg_r >>= SCIQOS_R;

	stat->arqos = tmp_reg_r;
	stat->awqos = tmp_reg_w;

	return 0;
}

static int set_scibts(void __iomem *base, struct bts_stat *stat)
{
	int ret;

	if (!base || !stat)
		return -ENODATA;

	if (!stat->stat_on)
		return 0;

	ret = set_scibts_qos(base, stat);
	if (ret) {
		pr_err("set_scibts_qos failed! ret=%d\n", ret);
		return ret;
	}

	ret = set_scibts_mo(base, stat);
	if (ret) {
		pr_err("set_scibts_mo failed! ret=%d\n", ret);
		return ret;
	}

	return ret;
}

static int get_scibts(void __iomem *base, struct bts_stat *stat)
{
	int ret;

	if (!base || !stat)
		return -ENODATA;

	ret = get_scibts_qos(base, stat);
	if (ret) {
		pr_err("get_scibts_qos failed! ret=%d\n", ret);
		return ret;
	}

	ret = get_scibts_mo(base, stat);
	if (ret) {
		pr_err("get_scibts_mo failed! ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/****************************************************************
 *›     ›       ›       int bts ops functions›  ›       ›       *
 ****************************************************************/
static int set_int_vc(void __iomem *base, unsigned int value)
{
	if (!base)
		return -ENODATA;

	__raw_writel(value, base);

	return 0;
}

static int get_int_vc(void __iomem *base, unsigned int *value)
{
	if (!base)
		return -ENODATA;

	*value = __raw_readl(base);

	return 0;
}

/****************************************************************
 *			register ops functions			*
 ****************************************************************/
int register_btsops(struct bts_info *info)
{
	unsigned int type = info->type;

	info->ops = kmalloc(sizeof(*info->ops), GFP_KERNEL);
	if (!info->ops)
		return -ENOMEM;

	switch (type) {
	case IP_BTS:
		info->ops->init_bts = NULL;
		info->ops->set_bts = set_ipbts;
		info->ops->get_bts = get_ipbts;
		info->ops->set_qos = set_ipbts_qos;
		info->ops->get_qos = get_ipbts_qos;
		info->ops->set_mo = set_ipbts_mo;
		info->ops->get_mo = get_ipbts_mo;
		info->ops->set_urgent = set_ipbts_urgent;
		info->ops->get_urgent = get_ipbts_urgent;
		info->ops->set_blocking = set_ipbts_blocking;
		info->ops->get_blocking = get_ipbts_blocking;
		info->ops->set_vc = NULL;
		info->ops->get_vc = NULL;
		break;
	case TREX_BTS:
		info->ops->init_bts = init_trexbts;
		info->ops->set_bts = NULL;
		info->ops->get_bts = NULL;
		info->ops->set_qos = NULL;
		info->ops->get_qos = NULL;
		info->ops->set_mo = NULL;
		info->ops->get_mo = NULL;
		info->ops->set_urgent = NULL;
		info->ops->get_urgent = NULL;
		info->ops->set_blocking = NULL;
		info->ops->get_blocking = NULL;
		info->ops->set_vc = NULL;
		info->ops->get_vc = NULL;
		break;
	case SCI_BTS:
		info->ops->init_bts = init_sciqfull;
		info->ops->set_bts = set_scibts;
		info->ops->get_bts = get_scibts;
		info->ops->set_qos = set_scibts_qos;
		info->ops->get_qos = get_scibts_qos;
		info->ops->set_mo = set_scibts_mo;
		info->ops->get_mo = get_scibts_mo;
		info->ops->set_urgent = NULL;
		info->ops->get_urgent = NULL;
		info->ops->set_blocking = NULL;
		info->ops->get_blocking = NULL;
		info->ops->set_vc = NULL;
		info->ops->get_vc = NULL;
		break;
	case SMC_BTS:
		info->ops->init_bts = init_smcqbusy;
		info->ops->set_bts = NULL;
		info->ops->get_bts = NULL;
		info->ops->set_qos = NULL;
		info->ops->get_qos = NULL;
		info->ops->set_mo = NULL;
		info->ops->get_mo = NULL;
		info->ops->set_urgent = NULL;
		info->ops->get_urgent = NULL;
		info->ops->set_blocking = NULL;
		info->ops->get_blocking = NULL;
		info->ops->set_vc = NULL;
		info->ops->get_vc = NULL;
		break;
	case BUSC_BTS:
		info->ops->init_bts = init_qmax;
		info->ops->set_bts = NULL;
		info->ops->get_bts = NULL;
		info->ops->set_qos = NULL;
		info->ops->get_qos = NULL;
		info->ops->set_mo = NULL;
		info->ops->get_mo = NULL;
		info->ops->set_urgent = NULL;
		info->ops->get_urgent = NULL;
		info->ops->set_blocking = NULL;
		info->ops->get_blocking = NULL;
		info->ops->set_vc = NULL;
		info->ops->get_vc = NULL;
		break;
	case DREX_BTS:
		info->ops->init_bts = NULL;
		info->ops->set_bts = NULL;
		info->ops->get_bts = NULL;
		info->ops->set_qos = NULL;
		info->ops->get_qos = NULL;
		info->ops->set_mo = NULL;
		info->ops->get_mo = NULL;
		info->ops->set_urgent = NULL;
		info->ops->get_urgent = NULL;
		info->ops->set_blocking = NULL;
		info->ops->get_blocking = NULL;
		info->ops->set_vc = NULL;
		info->ops->get_vc = NULL;
		break;
	case INTERNAL_BTS:
		info->ops->init_bts = NULL;
		info->ops->set_bts = NULL;
		info->ops->get_bts = NULL;
		info->ops->set_qos = NULL;
		info->ops->get_qos = NULL;
		info->ops->set_mo = NULL;
		info->ops->get_mo = NULL;
		info->ops->set_urgent = NULL;
		info->ops->get_urgent = NULL;
		info->ops->set_blocking = NULL;
		info->ops->get_blocking = NULL;
		info->ops->set_vc = set_int_vc;
		info->ops->get_vc = get_int_vc;
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(register_btsops);

MODULE_LICENSE("GPL");
