// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung EXYNOS SoC series MIPI CSI/DSI D/C-PHY driver
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <soc/google/exynos-pmu-if.h>

/* the maximum number of PHY for each module */
#define EXYNOS_MIPI_PHYS_MASTER_NUM	4

#define EXYNOS_MIPI_PHY_M4M4_ISO_BYPASS  BIT(0)

#define MIPI_PHY_MxMx_SHARED		BIT(1)
#define MIPI_PHY_MxMx_INIT_DONE		BIT(2)

enum exynos_mipi_phy_owner {
	EXYNOS_MIPI_PHY_OWNER_DSIM_0 = 0,
	EXYNOS_MIPI_PHY_OWNER_DSIM_1 = 1,
};

/* per MIPI-PHY module */
struct exynos_mipi_phy_data {
	u8 flags;
	int active_count;
	spinlock_t slock;
};

/* per DT MIPI-PHY node, can have multiple elements */
struct exynos_mipi_phy {
	struct device *dev;
	spinlock_t slock;
	struct regmap *reg_reset;
	enum exynos_mipi_phy_owner owner;
	struct mipi_phy_desc {
		struct phy *phy;
		struct exynos_mipi_phy_data *data;
		unsigned int index;
		unsigned int iso_offset;
		unsigned int rst_bit;
		void __iomem *regs;
	} phys[EXYNOS_MIPI_PHYS_MASTER_NUM];
};

/* 1: Isolation bypass, 0: Isolation enable */
static int __set_phy_isolation(unsigned int offset, unsigned int on)
{
	unsigned int val;
	int ret;

	val = on ? EXYNOS_MIPI_PHY_M4M4_ISO_BYPASS : 0;

	ret = exynos_pmu_update(offset, EXYNOS_MIPI_PHY_M4M4_ISO_BYPASS, val);

	if (ret)
		pr_err("%s failed to %s PHY isolation 0x%x\n",
				__func__, on ? "set" : "clear", offset);

	pr_debug("%s off=0x%x, val=0x%x\n", __func__, offset, val);

	return ret;
}

/* 1: Enable reset -> release reset, 0: Enable reset */
static int __set_phy_reset(struct regmap *reg_reset,
		unsigned int bit, unsigned int on)
{
	unsigned int cfg;
	int ret = 0;

	if (!reg_reset)
		return 0;

	ret = regmap_update_bits(reg_reset, 0, BIT(bit), 0);
	if (ret) {
		pr_err("%s failed to reset PHY(%d)\n", __func__, bit);
		goto err;
	}

	if (on) {
		ret = regmap_update_bits(reg_reset, 0, BIT(bit), BIT(bit));
		if (ret) {
			pr_err("%s failed to release reset PHY(%d)\n",
					__func__, bit);
			goto err;
		}
	}

	return ret;

err:
	regmap_read(reg_reset, 0, &cfg);
	pr_err("%s bit=%d, cfg=0x%x\n", __func__, bit, cfg);

	return ret;
}

static int __set_phy_init(struct exynos_mipi_phy *state,
		struct mipi_phy_desc *phy_desc, unsigned int on)
{
	unsigned int cfg;
	int ret = 0;

	ret = exynos_pmu_read(phy_desc->iso_offset, &cfg);

	if (ret) {
		dev_err(state->dev, "%s Can't read 0x%x\n",
				__func__, phy_desc->iso_offset);
		ret = -EINVAL;
		goto phy_exit;
	}

	/* Add INIT_DONE flag when ISO is already bypass(LCD_ON_UBOOT) */
	if (cfg && EXYNOS_MIPI_PHY_M4M4_ISO_BYPASS)
		phy_desc->data->flags |= MIPI_PHY_MxMx_INIT_DONE;

phy_exit:
	return ret;
}

static int __set_phy_alone(struct exynos_mipi_phy *state,
		struct mipi_phy_desc *phy_desc, unsigned int on)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&state->slock, flags);

	if (on) {
		ret = __set_phy_isolation(phy_desc->iso_offset, on);
		if (ret)
			goto phy_exit;

		ret = __set_phy_reset(state->reg_reset,
				phy_desc->rst_bit, on);
	} else {
		ret = __set_phy_reset(state->reg_reset,
				phy_desc->rst_bit, on);
		if (ret)
			goto phy_exit;

		ret = __set_phy_isolation(phy_desc->iso_offset, on);
	}

phy_exit:
	pr_debug("%s: isolation 0x%x, reset 0x%x\n", __func__,
			phy_desc->iso_offset, phy_desc->rst_bit);

	spin_unlock_irqrestore(&state->slock, flags);

	return ret;
}

static int __set_phy_share(struct exynos_mipi_phy *state,
		struct mipi_phy_desc *phy_desc, unsigned int on)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&phy_desc->data->slock, flags);

	if (on)
		phy_desc->data->active_count++;
	else
		phy_desc->data->active_count--;

	/* If phy is already initialization(power_on) */
	if (state->owner == EXYNOS_MIPI_PHY_OWNER_DSIM_0 &&
			phy_desc->data->flags & MIPI_PHY_MxMx_INIT_DONE) {
		phy_desc->data->flags &= (~MIPI_PHY_MxMx_INIT_DONE);
		spin_unlock_irqrestore(&phy_desc->data->slock, flags);
		return ret;
	}

	if (on) {
		/* Isolation bypass when reference count is 1 */
		if (phy_desc->data->active_count) {
			ret = __set_phy_isolation(phy_desc->iso_offset, on);
			if (ret)
				goto phy_exit;
		}

		ret = __set_phy_reset(state->reg_reset,
				phy_desc->rst_bit, on);
	} else {
		ret = __set_phy_reset(state->reg_reset,
				phy_desc->rst_bit, on);
		if (ret)
			goto phy_exit;

		/* Isolation enabled when reference count is zero */
		if (phy_desc->data->active_count == 0)
			ret = __set_phy_isolation(phy_desc->iso_offset, on);
	}

phy_exit:
	pr_debug("%s: isolation 0x%x, reset 0x%x\n", __func__,
			phy_desc->iso_offset, phy_desc->rst_bit);

	spin_unlock_irqrestore(&phy_desc->data->slock, flags);

	return ret;
}

static int __set_phy_state(struct exynos_mipi_phy *state,
		struct mipi_phy_desc *phy_desc, unsigned int on)
{
	int ret = 0;

	if (phy_desc->data->flags & MIPI_PHY_MxMx_SHARED)
		ret = __set_phy_share(state, phy_desc, on);
	else
		ret = __set_phy_alone(state, phy_desc, on);

	return ret;
}

static struct exynos_mipi_phy_data mipi_phy_m4m4 = {
	.flags = MIPI_PHY_MxMx_SHARED,
	.active_count = 0,
	.slock = __SPIN_LOCK_UNLOCKED(mipi_phy_m4m4.slock),
};

static const struct of_device_id exynos_mipi_phy_of_table[] = {
	{
		.compatible = "samsung,mipi-phy-m4m4",
		.data = &mipi_phy_m4m4,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, exynos_mipi_phy_of_table);

#define to_mipi_video_phy(desc) \
	container_of((desc), struct exynos_mipi_phy, phys[(desc)->index])

static int exynos_mipi_dsim_phy_init(struct phy *phy)
{
	struct mipi_phy_desc *phy_desc = phy_get_drvdata(phy);
	struct exynos_mipi_phy *state = to_mipi_video_phy(phy_desc);

	return __set_phy_init(state, phy_desc, 1);
}

static int exynos_mipi_dsim_phy_power_on(struct phy *phy)
{
	struct mipi_phy_desc *phy_desc = phy_get_drvdata(phy);
	struct exynos_mipi_phy *state = to_mipi_video_phy(phy_desc);

	return __set_phy_state(state, phy_desc, 1);
}

static int exynos_mipi_dsim_phy_power_off(struct phy *phy)
{
	struct mipi_phy_desc *phy_desc = phy_get_drvdata(phy);
	struct exynos_mipi_phy *state = to_mipi_video_phy(phy_desc);

	return __set_phy_state(state, phy_desc, 0);
}

static struct phy *exynos_mipi_phy_of_xlate(struct device *dev,
					struct of_phandle_args *args)
{
	struct exynos_mipi_phy *state = dev_get_drvdata(dev);

	if (WARN_ON(args->args[0] >= EXYNOS_MIPI_PHYS_MASTER_NUM))
		return ERR_PTR(-ENODEV);

	return state->phys[args->args[0]].phy;
}

static struct phy_ops exynos_mipi_phy_ops = {
	.init		= exynos_mipi_dsim_phy_init,
	.power_on	= exynos_mipi_dsim_phy_power_on,
	.power_off	= exynos_mipi_dsim_phy_power_off,
	.owner		= THIS_MODULE,
};

static int exynos_mipi_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct exynos_mipi_phy *state;
	struct phy_provider *phy_provider;
	struct exynos_mipi_phy_data *phy_data;
	const struct of_device_id *of_id;
	unsigned int iso[EXYNOS_MIPI_PHYS_MASTER_NUM];
	unsigned int rst[EXYNOS_MIPI_PHYS_MASTER_NUM];
	struct resource *res;
	unsigned int i;
	int ret = 0, elements = 0;

	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->dev  = &pdev->dev;

	of_id = of_match_device(of_match_ptr(exynos_mipi_phy_of_table), dev);
	if (!of_id)
		return -EINVAL;

	phy_data = (struct exynos_mipi_phy_data *)of_id->data;

	dev_set_drvdata(dev, state);
	spin_lock_init(&state->slock);

	/* PMU isolation (optional) */
	elements = of_property_count_u32_elems(node, "isolation");
	if ((elements < 0) || (elements > EXYNOS_MIPI_PHYS_MASTER_NUM))
		return -EINVAL;

	ret = of_property_read_u32_array(node, "isolation", iso,
					elements);
	if (ret) {
		dev_err(dev, "cannot get PHY isolation offset\n");
		return ret;
	}

	/* SYSREG reset (optional) */
	state->reg_reset = syscon_regmap_lookup_by_phandle(node,
						"samsung,reset-sysreg");
	if (IS_ERR(state->reg_reset)) {
		state->reg_reset = NULL;
	} else {
		ret = of_property_read_u32_array(node, "reset", rst, elements);
		if (ret) {
			dev_err(dev, "cannot get PHY reset bit\n");
			return ret;
		}
	}

	of_property_read_u32(node, "owner", &state->owner);

	for (i = 0; i < elements; i++) {
		state->phys[i].iso_offset = iso[i];
		state->phys[i].rst_bit	  = rst[i];
		dev_info(dev, "%s: isolation 0x%x\n", __func__,
				state->phys[i].iso_offset);
		if (state->reg_reset)
			dev_info(dev, "%s: reset %d\n", __func__,
				state->phys[i].rst_bit);

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (res) {
			state->phys[i].regs = devm_ioremap_resource(dev, res);
			if (IS_ERR(state->phys[i].regs))
				return PTR_ERR(state->phys[i].regs);
		}
	}

	for (i = 0; i < elements; i++) {
		struct phy *generic_phy = devm_phy_create(dev, NULL,
				&exynos_mipi_phy_ops);
		if (IS_ERR(generic_phy)) {
			dev_err(dev, "failed to create PHY\n");
			return PTR_ERR(generic_phy);
		}

		state->phys[i].index	= i;
		state->phys[i].phy	= generic_phy;
		state->phys[i].data	= phy_data;
		phy_set_drvdata(generic_phy, &state->phys[i]);
	}

	phy_provider = devm_of_phy_provider_register(dev,
			exynos_mipi_phy_of_xlate);

	if (IS_ERR(phy_provider))
		dev_err(dev, "failed to create exynos mipi-phy\n");
	else
		dev_info(dev, "creating exynos-mipi-phy\n");

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver exynos_mipi_phy_driver = {
	.probe	= exynos_mipi_phy_probe,
	.driver = {
		.name  = "exynos-mipi-phy",
		.of_match_table = of_match_ptr(exynos_mipi_phy_of_table),
		.suppress_bind_attrs = true,
	}
};
module_platform_driver(exynos_mipi_phy_driver);

MODULE_DESCRIPTION("Samsung EXYNOS SoC MIPI CSI/DSI D/C-PHY driver");
MODULE_LICENSE("GPL v2");
